/**
 * @file AuxUart.hpp
 * @author Tristan Fladischer
 * @brief implementation of hardwareSerial1 (auxUart) functions for CrazyCar
 * @version 0.1
 * @date 2025-11-03
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#include "AuxUart.hpp"

HardwareSerial auxUart(1);

void auxUartInit() {
    auxUart.begin(BAUD, SERIAL_8N1, ESP_RX, ESP_TX); // Data connection to Arduino
}

/**
 * @brief Send SensorData struct over AuxUart in binary (only the motor pulses matter)
 * 
 * @param data SensorData struct - holds all data from sensor measurements
 */
void sendMotorDataToMega(SensorData* data) {
    // transmitt each primitive in binary format
    sendValue(&data->startByte);
    sendValue(&data->packetNumber);
    sendValue(&data->velocity);
    sendValue(&data->batteryVoltage);
    sendValue(&data->leftDistance);
    sendValue(&data->middleDistance);
    sendValue(&data->rightDistance);
    sendValue(&data->ax);
    sendValue(&data->ay);
    sendValue(&data->az);
    sendValue(&data->gx);
    sendValue(&data->gy);
    sendValue(&data->gz);
    sendValue(&data->servoPulse);
    sendValue(&data->escPulse);
    sendValue(&data->stopByte);
}


// returns true and stores sensor frame in 'out'
bool readSensorFrame(SensorData &out) {
    static uint8_t buffer[sizeof(SensorData)];
    static uint8_t index = 0;

    while (auxUart.available()) {
        uint8_t b = auxUart.read();

        // Warte auf Start-Byte
        if (index == 0) {
            if (b != START_OF_FRAME_BYTE) {
                continue;
            }
            buffer[index++] = b;
            continue;
        }

        // Byte ins Buffer legen
        buffer[index++] = b;

        // Wenn Buffer voll ist
        if (index >= sizeof(SensorData)) {
            if (buffer[sizeof(SensorData) - 1] == END_OF_FRAME_BYTE) {
                memcpy(&out, buffer, sizeof(SensorData));
                index = 0;
                while(auxUart.available()) auxUart.read();
                return true; // Frame vollständig!
            } else {
                // Falsches Ende -> Reset
                index = 0;
            }
        }
    }
    return false; // Noch nicht vollständig
}
