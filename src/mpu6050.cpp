// filepath: c:\Users\TEMP.GEII2.033\gyro-control\gyro-control\src\mpu6050.cpp
#include "mpu6050.h" // Include the header file for MPU6050 functions
#include <Wire.h>    // Include the Wire library for I2C communication

// Function to initialize the MPU6050 sensor
void MPU6050_Init() {
    Wire.begin(); // Start I2C communication
    Wire.beginTransmission(MPU6050_ADDRESS); // Begin transmission to the MPU6050
    Wire.write(PWR_MGMT_1); // Specify the power management register
    Wire.write(0); // Set the register to zero to wake up the MPU6050
    Wire.endTransmission(); // End the transmission
}

// Function to read accelerometer and gyroscope data from the MPU6050
void MPU6050_Read_Data(int16_t *accelData, int16_t *gyroData) {
    Wire.beginTransmission(MPU6050_ADDRESS); // Begin transmission to the MPU6050
    Wire.write(ACCEL_XOUT_H); // Specify the starting register for accelerometer data
    Wire.endTransmission(false); // End transmission but keep the connection alive
    Wire.requestFrom(MPU6050_ADDRESS, 14); // Request 14 bytes of data from the MPU6050

    // Read accelerometer data
    accelData[0] = (Wire.read() << 8) | Wire.read(); // Read X-axis accelerometer data
    accelData[1] = (Wire.read() << 8) | Wire.read(); // Read Y-axis accelerometer data
    accelData[2] = (Wire.read() << 8) | Wire.read(); // Read Z-axis accelerometer data

    // Read temperature data (not used in this example)
    Wire.read(); // Read and discard temperature high byte
    Wire.read(); // Read and discard temperature low byte

    // Read gyroscope data
    gyroData[0] = (Wire.read() << 8) | Wire.read(); // Read X-axis gyroscope data
    gyroData[1] = (Wire.read() << 8) | Wire.read(); // Read Y-axis gyroscope data
    gyroData[2] = (Wire.read() << 8) | Wire.read(); // Read Z-axis gyroscope data
}