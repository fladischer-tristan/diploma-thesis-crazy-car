/**
 * @file AuxUart.hpp
 * @author Tristan Fladischer
 * @brief declaration of hardwareSerial1 (auxUart) functions for CrazyCar
 * @version 0.1
 * @date 2025-11-03
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#pragma once

#include <Arduino.h>
#include "Types.hpp"

constexpr uint8_t ESP_RX = D6;
constexpr uint8_t ESP_TX = D7;
constexpr ulong BAUD = 115200;
constexpr uint8_t START_OF_COMM_BYTE = 0x53; // 'S'
constexpr uint8_t END_OF_COMM_BYTE = 0x45; // 'E'

extern HardwareSerial auxUart;

void auxUartInit();
void sendMotorDataToMega(SensorData *data);
bool readSensorFrame(SensorData &out);

/**
 * @brief takes pointer to a sensor-value, casts it to
 * a uint8_t (byte) pointer, so that UART can send raw
 * bytes.
 * 
 * @tparam T
 * @param value - pointer to sensor-value (can be of any pointer type)
 */
template<typename T>
void sendValue(T *value) {
    uint8_t* pValue = reinterpret_cast<uint8_t*>(value); // casting to byte pointer
    auxUart.write(pValue, sizeof(T)); // send over UART3
}