/**
 * @file AuxUart.hpp
 * @author Tristan Fladischer
 * @brief declaration of all functions related to Seria3 (auxUart) for the CrazyCar
 * @version 0.1
 * @date 2025-11-03
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#pragma once

#include <Arduino.h>
#include "Sensors.hpp"
#include "Types.hpp"

constexpr unsigned long BAUD = 115200; // UART3 Baudrate
constexpr uint8_t START_OF_COMM_BYTE = 0x53; // 'S'
constexpr uint8_t END_OF_COMM_BYTE = 0x45; // 'E'

void sendSensorDataToEsp(SensorData* data);
void clearUart3Buffer();

/**
 * @brief takes pointer to a sensorValue and casts it to
 * a uint8_t pointer, so that Serial3 can send raw bytes.
 * 
 * @tparam T
 * @param value - pointer to sensor-value (can be of any pointer type)
 */
template<typename T>
void sendValue(T* value) {
    uint8_t* pValue = reinterpret_cast<uint8_t*>(value); // casting to byte pointer
    Serial3.write(pValue, sizeof(T)); // send over UART3
}