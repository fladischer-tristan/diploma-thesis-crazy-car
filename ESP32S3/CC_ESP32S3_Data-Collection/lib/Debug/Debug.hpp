#pragma once

#include <Arduino.h>
#include "Types.hpp"
#include "AuxUart.hpp"

// Printing the contents of a SensorData struct to the Serial for DEBUG purpose
void sensorDataDebugPrint(SensorData& sensorData);