/**
 * @file main.cpp
 * @author Tristan Fladischer
 * @brief ESP32S3 training-data collection src code for CrazyCar.
 * @version 0.1
 * @date 2025-10-31
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#include <Arduino.h>
#include <WiFi.h>
#include "esp_wifi.h"
#include "Types.hpp"
#include "AuxUart.hpp"
#include "secrets.hpp"
#include "WifiUtils.hpp"
#include "Debug.hpp"

// ESP32S3 Pins
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
 * TCP Server IP & Port
 * Since the server is a laptop, the IP is temporary
 */
const char* HOST = "192.168.8.106"; 
const uint16_t PORT = 5000;

// Size of UART packet
const byte SENSOR_FRAME_LENGTH = sizeof(SensorData);

// Enums for our control logic (2 state machines)
enum CommState { WAIT_FOR_START, RUNNING };
enum CycleState { SEND_MOTOR_PULSES, READ_SENSOR_DATA, APPEND_DATA_TO_QUEUE };

CommState commState = WAIT_FOR_START;
CycleState cycleState = SEND_MOTOR_PULSES;

// FreeRTOS Queue and TCP task
QueueHandle_t packetQueue;
void tcpSenderTask(void *pvParameters);

void setup() {
	vTaskDelay(pdMS_TO_TICKS(3000)); // ESP32S3 Serial needs the delay to actually work in the setup

	Serial.begin(115200); // DEBUG Serial
	vTaskDelay(pdMS_TO_TICKS(100));

	auxUartInit(); // Inter-Board Serial
	vTaskDelay(pdMS_TO_TICKS(100));

	// RC-Receiver pins
	pinMode(ESP_STEERING_PWM_IN, INPUT);
	pinMode(ESP_ESC_PWM_IN, INPUT);

	/*
	* creating a queue for our SensorData structs, so
	* that a seperate task can send them away over TCP
	* - the task is handled by a core asynchronously
	*/
	packetQueue = xQueueCreate(1000, sizeof(SensorData)); // 1000 structs space (58kb) simply because we can afford it
	xTaskCreatePinnedToCore(tcpSenderTask, "TCP Sender", 4096, NULL, 1, NULL, 1);

	// Connect to WiFi and TCPServer
	// SSID and PASS come from 'src/secrets.hpp'. since you don't see the file, just declare the variables somewhere
	connectToNetwork(SSID, PASS);
	connectToServer(HOST, PORT);
}

/*
* Sending Motor-Pulses to Arduino Mega over UART (auxUart),
* then waiting for Arduino to send back the whole SensorData struct,
* and finally send the received SensorData struct away to TCP Server
* - This cycle should repeat as fast as possible, in fixed timestamps
*
* 1. Wait for START_OF_COMM_BYTE
* 2. Read Motor Pulses and send them to Arduino MEGA
* 3. Wait for incoming SensorData struct and read it
* 4. Send struct away over WiFi (this happens outside loop via FreeRtos task)
*/
void loop() {
	static SensorData motorData, sensorData;

	// Master-StateMachine - handles only communication state (waiting, running)
	switch (commState) {
		// here we want to wait for the START_OF_COMM_BYTE
		case WAIT_FOR_START:
			if (auxUart.available() && auxUart.read() == START_OF_COMM_BYTE) {
				Serial.println("SoC...");

				// clearing auxUart buf to ensure stable synchronization
				while (auxUart.available()) auxUart.read();

				// Start of Communication deteced, switch to RUNNING
				cycleState = SEND_MOTOR_PULSES;
				commState = RUNNING;
			}
		break;
		
		// Communication is underway:
		case RUNNING:
			// First we need to check if the communication ended:
			if (auxUart.available() && auxUart.peek() == END_OF_COMM_BYTE) {
				auxUart.read();
				Serial.println("EoC...");
				while (auxUart.available()) auxUart.read(); // reset uart buffer
				commState = WAIT_FOR_START;
				break;
			}

			// Secondary StateMachine - Controls details of communication
			switch (cycleState) {
				case SEND_MOTOR_PULSES:
					Serial.println("Reading Motorpulses ...");
					// reading RC-Receiver pins and sending the Signals to Arduino MEGA
					motorData.escPulse = pulseIn(ESP_ESC_PWM_IN, HIGH, 25000);
					motorData.servoPulse = pulseIn(ESP_STEERING_PWM_IN, HIGH, 25000);
					Serial.print("ESC: ");
					Serial.println(motorData.escPulse);
					Serial.print("SERVO: ");
					Serial.println(motorData.escPulse);
					sendMotorDataToMega(&motorData);
					Serial.println("Sent Motorpulses ...");

					// switch to next state
					cycleState = READ_SENSOR_DATA;
				break;

				case READ_SENSOR_DATA:
					//Serial.println("Fetching data from UART...");
					if (readSensorFrame(sensorData)) {
						sensorDataDebugPrint(sensorData);
						cycleState = APPEND_DATA_TO_QUEUE;
						Serial.println("Fetch successful ...");
					}
				break;

				case APPEND_DATA_TO_QUEUE:
					Serial.println("Appending data to queue ...");
					xQueueSend(packetQueue, &sensorData, 0);

					// switch to original state -> repeat cycle
					cycleState = SEND_MOTOR_PULSES;
				break;
			}
		break;
	}
}

/**
 * @brief Sends SensorData to TCP Server, operates on reserved CPU core
 * 
 * @param pvParameters NULL
 */
void tcpSenderTask(void *pvParameters) {
	for (;;) {
		SensorData packet;
		Serial.println("Hello from Queue Task");
		if (xQueueReceive(packetQueue, &packet, 0)) {
			if (client.connected()) {
				Serial.println("Sending Packet");
				// Sending a SensorData packet to TCP server in raw bytes
				client.write((uint8_t*)&packet, sizeof(packet));
				vTaskDelay(10 / portTICK_PERIOD_MS);
			}
			else {
				// If we are not connected to server, try reconnect
				// since this task does not block the main loop, we
				// can safely wait for the reconnect. The queue will
				// buffer any incoming SensorData from loop.
				client.connect(HOST, PORT);
				vTaskDelay(1000 / portTICK_PERIOD_MS);
			}
		} else {
			vTaskDelay(5 / portTICK_PERIOD_MS);
		}
	}
}