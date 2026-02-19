#include "Debug.hpp"

void sensorDataDebugPrint(SensorData& sensorData) {
	Serial.print("sizeof(SensorData): ");
	Serial.println((unsigned long)sizeof(sensorData));
	Serial.print(F("id: ")); Serial.print(sensorData.packetNumber);
	Serial.print(F(" vel: ")); Serial.print(sensorData.velocity, 2);
	Serial.print(F(" bat: ")); Serial.print(sensorData.batteryVoltage, 2);

	Serial.print(F(" | dist L/M/R: "));
	Serial.print(sensorData.leftDistance, 2); Serial.print(" / ");
	Serial.print(sensorData.middleDistance, 2); Serial.print(" / ");
	Serial.print(sensorData.rightDistance, 2);

	Serial.print(F(" | acc: "));
	Serial.print(sensorData.ax, 2); Serial.print(" ");
	Serial.print(sensorData.ay, 2); Serial.print(" ");
	Serial.print(sensorData.az, 2);

	Serial.print(F(" | gyro: "));
	Serial.print(sensorData.gx, 2); Serial.print(" ");
	Serial.print(sensorData.gy, 2); Serial.print(" ");
	Serial.print(sensorData.gz, 2);

	Serial.print(F(" | servo: "));
	Serial.print(sensorData.servoPulse);
	Serial.print(F(" esc: "));
	Serial.println(sensorData.escPulse);
}