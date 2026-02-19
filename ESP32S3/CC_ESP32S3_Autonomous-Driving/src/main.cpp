/**
 * @file semi_autonomous_mode.cpp
 * @author Tristan Fladischer
 * @brief ESP32S3 CrazyCar source code for semi-autonomous-mode
 * @version 0.1
 * @date 2026-01-14
 * 
 * @copyright Copyright (c) 2026
 * 
 */

/*
 * This program implements semi-autonomous CrazyCar runs,
 * continuously receiving sensordata from ESP, and
 * receiving motor pulses in return, which then control
 * a steering servo and esc.
 */

#include <Arduino.h>
#include "Types.hpp"
#include "AuxUart.hpp"
#include "Debug.hpp"

#include "model_data.h" // The trained model formatted as C array

// TFLM
#include "tensorflow/lite/micro/micro_interpreter.h"
#include "tensorflow/lite/micro/micro_mutable_op_resolver.h"
#include "tensorflow/lite/schema/schema_generated.h"

// ESP32S3 Pinout
constexpr uint8_t ESP_S2 = D0;
constexpr uint8_t ESP_S3 = D1;
constexpr uint8_t ESP_S4 = D2;
constexpr uint8_t ESP_NEO_PIXEL = D3;
constexpr uint8_t ESP_SDA = D4;
constexpr uint8_t ESP_SCL = D5;
constexpr uint8_t ESP_STEERING_PWM_IN = D10;
constexpr uint8_t ESP_ESC_PWM_IN = D9;
constexpr uint8_t ESP_S1 = D8;

/*
 * INPUT (FEATURES) in order:
 *
 * 1. "packetNumber"
 * 2. "leftDistance"
 * 3. "middleDistance"
 * 4. "rightDistance"
 * 5. "ax"
 * 6. "ay"
 * 7. "az"
 * 8. "gx"
 * 9. "gy"
 * 10. "gz"
 * 11. "velocity"
 * 
 */
static constexpr int kInputDim  = 11;

// Min/max constraints for our features and labels. These will be used for normalizing and denormalizing
const float PACKET_NUMBER_MAX = 129.0F, PACKET_NUMBER_MIN = 0.0F; /* is actually an integer, our function takes floats however */
const float LEFT_DT_MAX = 2.559999943F, MIDDLE_DT_MAX = 2.559999943F, RIGHT_DT_MAX = 2.559999943F;
const float LEFT_DT_MIN = 0.457947195F, MIDDLE_DT_MIN = 0.012512218F, RIGHT_DT_MIN = 0.290283471F;
const float AX_MAX = 1.365966797F, AY_MAX = 1.030273438F, AZ_MAX = 1.999938965F;
const float AX_MIN = -2.000000000F, AY_MIN = -1.912597656F, AZ_MIN = -0.482421875F;
const float GX_MAX = 80.893127441F, GY_MAX = 96.442749023F, GZ_MAX = 174.435119629F;
const float GX_MIN = -250.137405396F, GY_MIN = -69.312980652F, GZ_MIN = -250.137405396F;
const float VELOCITY_MAX = 5000.000000000F, VELOCITY_MIN = 0.000000000F;
const float SERVO_PULSE_MAX = 1963.000000000F, SERVO_PULSE_MIN = 1400;//1011.000000000F;
const float ESC_PULSE_MAX = 1939.000000000F, ESC_PULSE_MIN = 1400; //1011.000000000F;

/*
 * OUTPUT (LABELS) in order:
 *
 * 1. "servoPulse"
 * 2. "escPulse"
 *
*/
static constexpr int kOutputDim = 2;

// 96kb size for the model arena
static constexpr int kArenaSize = 96 * 1024;
static uint8_t tensor_arena[kArenaSize];

// creating model and interpreter
static const tflite::Model* model = nullptr;
static tflite::MicroInterpreter* interpreter = nullptr;

// Size of UART packet in bytes
const byte SENSOR_FRAME_LENGTH = sizeof(SensorData);

// Enumns
enum CommState { WAIT_FOR_START, RUNNING };

enum CycleState {
	READ_SENSOR_DATA,
	NORMALIZE_SENSOR_DATA,
	FEED_AI_MODEL,
	DENORMALIZE_AI_OUTPUT,
	SEND_MOTOR_PULSES
};

// State-Machine
CommState commState = WAIT_FOR_START;
CycleState cycleState = READ_SENSOR_DATA;

// FreeRTOS Queue and TCP task
QueueHandle_t packetQueue;
void tcpSenderTask(void *pvParameters);

// Helper functions
static inline float clampf(float val, float lo, float hi);
static inline float norm(float x, float x_min, float x_max);
static inline float denorm(float y, float x_min, float x_max);

void setup() {
	vTaskDelay(pdMS_TO_TICKS(3000)); // Give some time for initialization

	Serial.begin(115200); // DEBUG Serial
	vTaskDelay(pdMS_TO_TICKS(100));

	auxUartInit(); // Serial to Arduino Mega
	vTaskDelay(pdMS_TO_TICKS(100));

	model = tflite::GetModel(g_model); // g_model is the C array

	/*
	 * the model uses only 2 operations:
	 *
	 * 1. Tanh (activation function)
	 * 2. FullyConnected (layers are connected densly)
	 * 
	 * an op resolver is just a lookup table that stores the corresponding
	 * function pointers for a certain operation.
	 * 
	 */
	static tflite::MicroMutableOpResolver<4> resolver;
	resolver.AddFullyConnected();
	resolver.AddTanh();

	// NOTE: tflm_esp32 uses the MicroInterpreter constructor that does NOT take ErrorReporter*
	// ErrorReporter* caused some problems since other frameworks did not integrate well in platform IO,
	// that's why we use tflm_esp32
	static tflite::MicroInterpreter static_interpreter(
		model,
		resolver,
		tensor_arena,
		kArenaSize,
		nullptr,   // MicroResourceVariables*
		nullptr,   // MicroProfilerInterface*
		false      // should_preserve_all_tensors
	);

	interpreter = &static_interpreter;

	if (interpreter->AllocateTensors() != kTfLiteOk) {
		Serial.println("AllocateTensors FAILED");
		while (true) delay(1000);
	}

	Serial.println("READY");
}

/*
* The loop implements part of the semi-autonomous driving. We receive 
* sensor-data from the inter-board-UART, feed it to the AI Model, send
* the generated output back to the inter-board-UART and repeat the process.
* - everything else is handled by the ATMEGA
*
* 1. Wait for START_OF_COMM_BYTE
* 2. Await and extract sensordata
* 3. normalize data to [-1; 1] and feed AI model
* 4. extract and denormalize AI outputs to [-1; 1]
* 5. Send back MotorPulses to ATMEGA, repeat 1)
*/
void loop() {
	static SensorData sensorData;

	TfLiteTensor* input = interpreter->input(0); // Input Layer of the model
	TfLiteTensor* output = interpreter->output(0); // Output Lay of the model

	/*
	 * Master-State-Machine: handles only communication state (WAIT_FOR_START, RUNNING)
	 */
	switch (commState) {
		// here we want to wait for the START_OF_COMM_BYTE
		case WAIT_FOR_START:
			if (auxUart.available() && auxUart.read() == START_OF_COMM_BYTE) {
				Serial.println("SoC...");

				// Clearing UART buffer to ensure stable synchronization
				while (auxUart.available()) auxUart.read();

				// Start of Communication deteced, switch to RUNNING
				cycleState = READ_SENSOR_DATA;
				commState = RUNNING;
			}
		break;
		
		// Communication is ongoing:
		case RUNNING:
			// First we need to check if the communication ended:
			if (auxUart.available() && auxUart.peek() == END_OF_COMM_BYTE) {
				auxUart.read();
				Serial.println("EoC...");
				while (auxUart.available()) auxUart.read(); // Clear UART buffer
				commState = WAIT_FOR_START;
				break;
			}

			/*
			 * Secondary-State-Machine: handles everything AI related and details of communication
			 */
			switch (cycleState) {
				case READ_SENSOR_DATA:
					if (readSensorFrame(sensorData)) {
						sensorDataDebugPrint(sensorData);
						cycleState = NORMALIZE_SENSOR_DATA;
						Serial.println("Fetch successful ...");
					}
				break;
				case NORMALIZE_SENSOR_DATA:
					// Fill input layer with normalized data
					input->data.f[0]  = norm((float)sensorData.packetNumber, PACKET_NUMBER_MIN, PACKET_NUMBER_MAX);
					input->data.f[1]  = norm(sensorData.leftDistance, LEFT_DT_MIN, LEFT_DT_MAX);
					input->data.f[2]  = norm(sensorData.middleDistance, MIDDLE_DT_MIN, MIDDLE_DT_MAX);
					input->data.f[3]  = norm(sensorData.rightDistance, RIGHT_DT_MIN, RIGHT_DT_MAX);
					input->data.f[4]  = norm(sensorData.ax, AX_MIN, AX_MAX);
					input->data.f[5]  = norm(sensorData.ay, AY_MIN, AY_MAX);
					input->data.f[6]  = norm(sensorData.az, AZ_MIN, AZ_MAX);
					input->data.f[7]  = norm(sensorData.gx, GX_MIN, GX_MAX);
					input->data.f[8]  = norm(sensorData.gy, GY_MIN, GY_MAX);
					input->data.f[9]  = norm(sensorData.gz, GZ_MIN, GZ_MAX);
					input->data.f[10] = norm(sensorData.velocity, VELOCITY_MIN, VELOCITY_MAX);

					cycleState = FEED_AI_MODEL;
				break;
				case FEED_AI_MODEL:
					// Invoke the interpreter
					if (interpreter->Invoke() != kTfLiteOk) {
						Serial.println("Invoke FAILED");
						delay(1000);
						return;
					}

					
					Serial.print("raw y0="); Serial.println(output->data.f[0], 6);
					Serial.println("raw y1="); Serial.println(output->data.f[1], 6);
					cycleState = DENORMALIZE_AI_OUTPUT;
				break;
				case DENORMALIZE_AI_OUTPUT:
					sensorData.servoPulse = denorm(output->data.f[0] , SERVO_PULSE_MIN, SERVO_PULSE_MAX);
					sensorData.escPulse = denorm(output->data.f[1], ESC_PULSE_MIN, ESC_PULSE_MAX);
					cycleState = SEND_MOTOR_PULSES;

					// // print outputs
					// Serial.print("y=[");
					// Serial.print(sensorData.servoPulse, 6);
					// Serial.print(", ");
					// Serial.print(sensorData.escPulse, 6);
					// Serial.println("]");
				break;
				case SEND_MOTOR_PULSES:
					// We only ever send the same sensorData struct because we modify it in-place
					// Otherwise we would need to create new Structs every cycle
					sendMotorDataToMega(&sensorData); 
					cycleState = READ_SENSOR_DATA;
					Serial.println(output->type);
				break;
			}
		break;
	}
}

/**
 * @brief clamp 'val' to 'lo', 'hi'
 * 
 * @param val value to clamp
 * @param lo low constraint
 * @param hi high constraint
 * @return float 
 */
static inline float clampf(float val, float lo, float hi) {
	return (val < lo) ? lo : (val > hi) ? hi : val;
}

/**
 * @brief normalize input [x_min; x_max] to [-1; 1]]
 * 
 * @return float
 */
static inline float norm(float x, float x_min, float x_max)
{
    float range = x_max - x_min;
    if (range == 0.0f)
        return 0.0f;  // definiertes Fallback-Verhalten

	float clamped = clampf(x, x_min, x_max);
    return 2.0f * (clamped - x_min) / range - 1.0f;
}

/**
 * @brief denormalize output [-1; 1] to [x_min; x_max]
 * 
 * @return float
 */
static inline float denorm(float y, float x_min, float x_max)
{
	float clamped = clampf(y, -1.0f, 1.0f);
    return ( (clamped + 1.0f) * 0.5f ) * (x_max - x_min) + x_min;
}

// /**
//  * @file main.cpp
//  * @author Tristan Fladischer
//  * @brief ESP32S3 training-data collection src code for CrazyCar.
//  * @version 0.1
//  * @date 2025-10-31
//  * 
//  * @copyright Copyright (c) 2025
//  * 
//  */

// #include <Arduino.h>
// #include <WiFi.h>
// #include "esp_wifi.h"
// #include "Types.hpp"
// #include "AuxUart.hpp"
// #include "secrets.hpp"
// #include "WifiUtils.hpp"
// #include "Debug.hpp"

// // ESP32S3 Pins
// constexpr uint8_t ESP_S2 = D0;
// constexpr uint8_t ESP_S3 = D1;
// constexpr uint8_t ESP_S4 = D2;
// constexpr uint8_t ESP_NEO_PIXEL = D3;
// constexpr uint8_t ESP_SDA = D4;
// constexpr uint8_t ESP_SCL = D5;
// constexpr uint8_t ESP_STEERING_PWM_IN = D10;
// constexpr uint8_t ESP_ESC_PWM_IN = D9;
// constexpr uint8_t ESP_S1 = D8;

// /*
//  * TCP Server IP & Port
//  * Since the server is a laptop, the IP is temporary
//  */
// const char* HOST = "192.168.8.106"; 
// const uint16_t PORT = 5000;

// // Size of UART packet
// const byte SENSOR_FRAME_LENGTH = sizeof(SensorData);

// // Enums for our control logic (2 state machines)
// enum CommState { WAIT_FOR_START, RUNNING };
// enum CycleState { SEND_MOTOR_PULSES, READ_SENSOR_DATA, APPEND_DATA_TO_QUEUE };

// CommState commState = WAIT_FOR_START;
// CycleState cycleState = SEND_MOTOR_PULSES;

// // FreeRTOS Queue and TCP task
// QueueHandle_t packetQueue;
// void tcpSenderTask(void *pvParameters);

// void setup() {
// 	vTaskDelay(pdMS_TO_TICKS(3000)); // ESP32S3 Serial needs the delay to actually work in the setup

// 	Serial.begin(115200); // DEBUG Serial
// 	vTaskDelay(pdMS_TO_TICKS(100));

// 	auxUartInit(); // Inter-Board Serial
// 	vTaskDelay(pdMS_TO_TICKS(100));

// 	// RC-Receiver pins
// 	pinMode(ESP_STEERING_PWM_IN, INPUT);
// 	pinMode(ESP_ESC_PWM_IN, INPUT);

// 	/*
// 	* creating a queue for our SensorData structs, so
// 	* that a seperate task can send them away over TCP
// 	* - the task is handled by a core asynchronously
// 	*/
// 	packetQueue = xQueueCreate(1000, sizeof(SensorData)); // 1000 structs space (58kb) simply because we can afford it
// 	xTaskCreatePinnedToCore(tcpSenderTask, "TCP Sender", 4096, NULL, 1, NULL, 1);

// 	// Connect to WiFi and TCPServer
// 	// SSID and PASS come from 'src/secrets.hpp'. since you don't see the file, just declare the variables somewhere
// 	connectToNetwork(SSID, PASS);
// 	connectToServer(HOST, PORT);
// }

// /*
// * Sending Motor-Pulses to Arduino Mega over UART (auxUart),
// * then waiting for Arduino to send back the whole SensorData struct,
// * and finally send the received SensorData struct away to TCP Server
// * - This cycle should repeat as fast as possible, in fixed timestamps
// *
// * 1. Wait for START_OF_COMM_BYTE
// * 2. Read Motor Pulses and send them to Arduino MEGA
// * 3. Wait for incoming SensorData struct and read it
// * 4. Send struct away over WiFi (this happens outside loop via FreeRtos task)
// */
// void loop() {
// 	static SensorData motorData, sensorData;

// 	// Master-StateMachine - handles only communication state (waiting, running)
// 	switch (commState) {
// 		// here we want to wait for the START_OF_COMM_BYTE
// 		case WAIT_FOR_START:
// 			if (auxUart.available() && auxUart.read() == START_OF_COMM_BYTE) {
// 				Serial.println("SoC...");

// 				// clearing auxUart buf to ensure stable synchronization
// 				while (auxUart.available()) auxUart.read();

// 				// Start of Communication deteced, switch to RUNNING
// 				cycleState = SEND_MOTOR_PULSES;
// 				commState = RUNNING;
// 			}
// 		break;
		
// 		// Communication is underway:
// 		case RUNNING:
// 			// First we need to check if the communication ended:
// 			if (auxUart.available() && auxUart.peek() == END_OF_COMM_BYTE) {
// 				auxUart.read();
// 				Serial.println("EoC...");
// 				while (auxUart.available()) auxUart.read(); // reset uart buffer
// 				commState = WAIT_FOR_START;
// 				break;
// 			}

// 			// Secondary StateMachine - Controls details of communication
// 			switch (cycleState) {
// 				case SEND_MOTOR_PULSES:
// 					Serial.println("Reading Motorpulses ...");
// 					// reading RC-Receiver pins and sending the Signals to Arduino MEGA
// 					motorData.escPulse = pulseIn(ESP_ESC_PWM_IN, HIGH, 25000);
// 					motorData.servoPulse = pulseIn(ESP_STEERING_PWM_IN, HIGH, 25000);
// 					Serial.print("ESC: ");
// 					Serial.println(motorData.escPulse);
// 					Serial.print("SERVO: ");
// 					Serial.println(motorData.escPulse);
// 					sendMotorDataToMega(&motorData);
// 					Serial.println("Sent Motorpulses ...");

// 					// switch to next state
// 					cycleState = READ_SENSOR_DATA;
// 				break;

// 				case READ_SENSOR_DATA:
// 					//Serial.println("Fetching data from UART...");
// 					if (readSensorFrame(sensorData)) {
// 						sensorDataDebugPrint(sensorData);
// 						cycleState = APPEND_DATA_TO_QUEUE;
// 						Serial.println("Fetch successful ...");
// 					}
// 				break;

// 				case APPEND_DATA_TO_QUEUE:
// 					Serial.println("Appending data to queue ...");
// 					xQueueSend(packetQueue, &sensorData, 0);

// 					// switch to original state -> repeat cycle
// 					cycleState = SEND_MOTOR_PULSES;
// 				break;
// 			}
// 		break;
// 	}
// }

// /**
//  * @brief Sends SensorData to TCP Server, operates on reserved CPU core
//  * 
//  * @param pvParameters NULL
//  */
// void tcpSenderTask(void *pvParameters) {
// 	for (;;) {
// 		SensorData packet;
// 		Serial.println("Hello from Queue Task");
// 		if (xQueueReceive(packetQueue, &packet, 0)) {
// 			if (client.connected()) {
// 				Serial.println("Sending Packet");
// 				// Sending a SensorData packet to TCP server in raw bytes
// 				client.write((uint8_t*)&packet, sizeof(packet));
// 				vTaskDelay(10 / portTICK_PERIOD_MS);
// 			}
// 			else {
// 				// If we are not connected to server, try reconnect
// 				// since this task does not block the main loop, we
// 				// can safely wait for the reconnect. The queue will
// 				// buffer any incoming SensorData from loop.
// 				client.connect(HOST, PORT);
// 				vTaskDelay(1000 / portTICK_PERIOD_MS);
// 			}
// 		} else {
// 			vTaskDelay(5 / portTICK_PERIOD_MS);
// 		}
// 	}
// }