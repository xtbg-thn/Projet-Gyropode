// This file contains configuration constants and parameters for the project

#ifndef CONFIG_H // Include guard to prevent multiple inclusions
#define CONFIG_H

// Sampling rate for the control system in milliseconds
const float SAMPLE_RATE = 10.0; // Sample every 10 ms

// Filter coefficients for the control system
const float FILTER_TAU = 1000.0; // Time constant for the filter in milliseconds

// Maximum and minimum values for sensor readings
const float MAX_SENSOR_VALUE = 32768.0; // Maximum value for MPU6050 sensor
const float MIN_SENSOR_VALUE = -32768.0; // Minimum value for MPU6050 sensor

// Control parameters
const float CONTROL_GAIN = 1.0; // Gain for the control algorithm

#endif // CONFIG_H