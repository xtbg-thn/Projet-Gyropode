#include <Arduino.h>
#include "comms.h"

// Function to initialize serial communication
void initSerial() {
    Serial.begin(115200); // Start serial communication at 115200 baud rate
    while (!Serial); // Wait for the serial port to connect (only needed for native USB)
    Serial.println("Serial communication initialized."); // Print a message to indicate successful initialization
}

// Function to send data over serial
void sendData(float data) {
    Serial.printf("Sending data: %f\n", data); // Print the data being sent
}

// Function to receive data over serial
float receiveData() {
    if (Serial.available() > 0) { // Check if there is data available to read
        float receivedValue = Serial.parseFloat(); // Read a float value from the serial buffer
        Serial.printf("Received data: %f\n", receivedValue); // Print the received data
        return receivedValue; // Return the received value
    }
    return 0; // Return 0 if no data is available
}