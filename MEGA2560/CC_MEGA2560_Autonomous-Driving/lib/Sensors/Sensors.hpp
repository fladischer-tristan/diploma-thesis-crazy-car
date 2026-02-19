/**
 * @file Sensors.hpp
 * @author Tristan Fladischer
 * @brief declaration of all functions related to Sensors of the CrazyCar
 * @version 0.1
 * @date 2025-11-03
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#pragma once

#include <Arduino.h>
#include <Wire.h>
#include "Types.hpp"

// IR sensors and battery sensor
constexpr uint8_t LEFTSENSOR_PIN =  A1;
constexpr uint8_t MIDDLESENSOR_PIN = A3;
constexpr uint8_t RIGHTSENSOR_PIN = A2;
constexpr uint8_t VBAT_PIN = A0;

// gyrosensor
constexpr uint8_t MPU9250_ADDR = 0x68; // I2C addr
constexpr float ACCEL_SENSITIVITY = 16384.0; // ±2g - config can be read from register
constexpr float GYRO_SENSITIVITY = 131.0; // ±250°/s - config can be read from register

// hall sensor and hall sample rate
constexpr uint8_t HALL_SENSOR_1_PIN = 18;
constexpr uint8_t HALL_SENSOR_2_PIN = 49;

// constants for calculation
constexpr float V_REF = 2.56; // Internal Reference Voltage
constexpr int ADC_MAX = 1023; // (2^10)-1 ... 1023
constexpr int PULSE_DISTANCE = 5; // physical distance between 2 hallsensor pulses

void sensorInit();
void readAllSensorData(SensorData& sensorData);
void readBatteryVoltage(float& batteryVoltage);
float adcToVoltage(int adc);
float calculateVelocity(int16_t pulseCount, const unsigned long sampleRate);
void readGyroSensor(float& ax, float& ay, float& az, float& gx, float& gy, float& gz);
void readDistanceSensors(float& leftDistance, float& middleDistance, float& rightDistance);