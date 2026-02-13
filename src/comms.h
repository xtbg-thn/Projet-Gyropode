// filepath: gyro-control/gyro-control/src/comms.h
#ifndef COMMS_H
#define COMMS_H

#include <Arduino.h> // Include the Arduino core library for basic functions

// Function to initialize communication
void initComms();

// Function to send data over serial
void sendData(float data);

// Function to receive data over serial
float receiveData();

#endif // COMMS_H