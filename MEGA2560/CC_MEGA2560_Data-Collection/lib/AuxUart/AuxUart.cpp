/**
 * @file AuxUart.hpp
 * @author Tristan Fladischer
 * @brief implementation of all functions related to Seria3 (auxUart) for the CrazyCar
 * @version 0.1
 * @date 2025-11-03
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#include "AuxUart.hpp"

/**
 * @brief Transmitt SensorData struct over Serial3 in binary (Serial.write())
 * 
 * @param data pointer to SensorData struct - holds all data from sensor measurements
 */
void sendSensorDataToEsp(SensorData* data) {
    /*
    * Transmitting each sensor Value in binary
    * sendValue() casts each value into a byte pointer,
    * so that Serial3 can send over all datatypes in the
    * same way.
    */

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

/**
 * @brief Empties the Serial3 buffer
 * 
 */
void clearUart3Buffer() {
    while (Serial3.available()) Serial3.read();
}