/**
 * @file Types.hpp
 * @author Tristan Fladischer
 * @brief declaration of all custom types used for the project
 * @version 0.1
 * @date 2025-11-03
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#pragma once

constexpr uint8_t START_OF_FRAME_BYTE = 0xAA;
constexpr uint8_t END_OF_FRAME_BYTE = 0x55;

#pragma pack(push, 1)  // disable struct padding (with padding, the struct would be bigger)

// struct for storing all sensor-data in compact packet (good for Serial)
struct SensorData {
    uint8_t startByte = START_OF_FRAME_BYTE; // SoF
    uint32_t packetNumber = 0; // ID
    float velocity = 0.0F;
    float batteryVoltage = 0.0F;
    float leftDistance = 0.0F, middleDistance = 0.0F, rightDistance = 0.0F;
    float ax = 0.0F, ay = 0.0F, az = 0.0F;
    float gx = 0.0F, gy = 0.0F, gz = 0.0F;
    uint32_t servoPulse = 0, escPulse = 0;
    uint8_t stopByte = END_OF_FRAME_BYTE; // EoF
};

#pragma pack(pop)