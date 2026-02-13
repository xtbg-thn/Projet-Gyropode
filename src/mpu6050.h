// This header file declares the functions and variables related to the MPU6050 sensor.

// Include guard to prevent multiple inclusions of this header file
#ifndef MPU6050_H
#define MPU6050_H

#include <Arduino.h> // Include Arduino core library for basic functionalities
#include <Wire.h>   // Include Wire library for I2C communication

// MPU6050 I2C address
#define MPU6050_ADDRESS 0x68

// Function prototypes for MPU6050 operations
void mpu6050_init(); // Function to initialize the MPU6050 sensor
void mpu6050_read(float *accelData, float *gyroData); // Function to read accelerometer and gyroscope data

// Variables to hold sensor data
extern float accelX, accelY, accelZ; // Accelerometer data
extern float gyroX, gyroY, gyroZ;     // Gyroscope data

#endif // MPU6050_H