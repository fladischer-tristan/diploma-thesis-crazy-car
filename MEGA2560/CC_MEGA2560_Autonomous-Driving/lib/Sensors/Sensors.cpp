/**
 * @file Sensors.cpp
 * @author Tristan Fladischer
 * @brief implementation of all functions related to Sensors of the CrazyCar
 * @version 0.1
 * @date 2025-11-03
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#include "Sensors.hpp"

/**
 * @brief Initializing I2C bus, setting pinModes and reference Voltage
 * 
 */
void sensorInit() {
    Wire.begin();
    pinMode(HALL_SENSOR_1_PIN, INPUT_PULLUP);
    pinMode(HALL_SENSOR_2_PIN, INPUT_PULLUP);
    analogReference(INTERNAL2V56);
}

/**
 * @brief Converts digitalRead() value to analog Voltage Signal through RefVoltage (2.56V)
 * 
 * @param adc digitalRead() signal
 * @return voltage
 */
float adcToVoltage(int adc) {
	return ((float)adc * V_REF) / ADC_MAX;
}

/**
 * @brief calculates velocity of vehicle through 
 * 
 * @param pulseCount amount of hallsensor pulses detected
 * @return float - velocity in mm/s
 */
float calculateVelocity(const int16_t pulseCount, const unsigned long sampleRate) { 
    return (float)(pulseCount * PULSE_DISTANCE) / (float)(sampleRate / 1000.0f); // mm/s
}

/**
 * @brief reads & normalizes acceleration and gyroscope values of MPU9250 (I2C) and stores them in passed references
 * 
 * @param ax pass reference - acceleration in x dir
 * @param ay pass reference - acceleration in y dir
 * @param az pass reference - acceleration in z dir
 * @param gx pass reference - angular speed in x dir
 * @param gy pass reference - angular speed in y dir
 * @param gz pass reference - angular speed in z dir
 */
void readGyroSensor(float& ax, float& ay, float& az, float& gx, float& gy, float& gz) {
    const byte accelReg = 0x3B; // first acceleration register
    const byte gyroReg = 0x43; // first gyroscope register

    Wire.beginTransmission(MPU9250_ADDR);
    Wire.write(accelReg);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU9250_ADDR, 6, true); // requesting 6 bytes from MPU9250 (2 bytes each for 3 values)

    // acceleration values (g)
    ax = (float)((Wire.read() << 8) | Wire.read()) / ACCEL_SENSITIVITY; // read 1st byte, bit shift 8 to left, then add 2nd byte
    ay = (float)((Wire.read() << 8) | Wire.read()) / ACCEL_SENSITIVITY;
    az = (float)((Wire.read() << 8) | Wire.read()) / ACCEL_SENSITIVITY;

    Wire.beginTransmission(MPU9250_ADDR);
    Wire.write(gyroReg);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU9250_ADDR, 6, true);

    // gyroscope values (°/s)
    gx = (float)((Wire.read() << 8) | Wire.read()) / GYRO_SENSITIVITY;
    gy = (float)((Wire.read() << 8) | Wire.read()) / GYRO_SENSITIVITY;
    gz = (float)((Wire.read() << 8) | Wire.read()) / GYRO_SENSITIVITY;
}

/**
 * @brief reads all IR sensors and stores in passed references
 * 
 * @param leftDistance pass reference - left distance
 * @param middleDistance pass reference - middle distance
 * @param rightDistance pass reference - right distance
 */
void readDistanceSensors(float& leftDistance, float& middleDistance, float& rightDistance) {
    leftDistance = adcToVoltage(analogRead(LEFTSENSOR_PIN));
    middleDistance = adcToVoltage(analogRead(MIDDLESENSOR_PIN));
    rightDistance = adcToVoltage(analogRead(RIGHTSENSOR_PIN));
}

/**
 * @brief Read vehicles battery voltage
 * 
 * @param batterVoltage pass reference - battery voltage
 */
void readBatteryVoltage(float& batterVoltage) {
    batterVoltage = adcToVoltage(analogRead(VBAT_PIN));
}

/**
 * @brief Reads all the vehicle Sensors except HALL (which is done through ISR)
 * 
 * @param sensorData 
 */
void readAllSensorData(SensorData& sensorData) {
    readGyroSensor(
        sensorData.ax,
        sensorData.ay,
        sensorData.az,
        sensorData.gx,
        sensorData.gy,
        sensorData.gz
    );
    readDistanceSensors(
        sensorData.leftDistance,
        sensorData.middleDistance,
        sensorData.rightDistance
    );
    readBatteryVoltage(sensorData.batteryVoltage);
}