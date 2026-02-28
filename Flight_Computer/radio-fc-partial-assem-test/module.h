#pragma once

// Config
#include "src/config.h"

// Arduino Dependency
#include <Wire.h>
#include <SPI.h>

// Sensor Dependency
#include "src/sensors/bmp390_sensor.h"
#include "src/sensors/lsm6dsm_sensor.h"
#include "src/sensors/lsm6dsm_minimal.h"

// Utilities
#include "src/utils/data_extractor.h"