# Gyro Control System

![Gyropode](images/gyro.png)

## Overview
This project implements a control system using the MPU6050 sensor. The system is designed to read data from the MPU6050, process it, and implement control logic based on the sensor readings. The project is structured into multiple source files for better organization and modularity.

## Project Structure
```
gyro-control
├── src
│   ├── main.cpp          # Entry point of the application
│   ├── mpu6050.h        # Header file for MPU6050 functions
│   ├── mpu6050.cpp      # Implementation of MPU6050 functions
│   ├── control.h        # Header file for control functions
│   ├── control.cpp      # Implementation of control functions
│   ├── comms.h          # Header file for communication functions
│   └── comms.cpp        # Implementation of communication functions
├── include
│   └── config.h         # Configuration constants and parameters
├── lib
│   └── MPU6050          # MPU6050 library files
├── platformio.ini       # PlatformIO configuration file
├── .gitignore           # Git ignore file
└── README.md            # Project documentation
```

## Setup Instructions
1. **Clone the Repository**: 
   Clone this repository to your local machine using:
   ```
   git clone <repository-url>
   ```

2. **Install PlatformIO**: 
   Ensure you have PlatformIO installed in your development environment.

3. **Open the Project**: 
   Open the project folder in your PlatformIO IDE.

4. **Install Dependencies**: 
   The MPU6050 library is included in the `lib` directory. Ensure all dependencies are installed by running:
   ```
   pio lib install
   ```

5. **Build the Project**: 
   Build the project using the PlatformIO build command.

6. **Upload to Board**: 
   Connect your ESP32 board and upload the code using:
   ```
   pio run --target upload
   ```

## Usage
- Once the code is uploaded, the system will start reading data from the MPU6050 sensor.
- The control logic will process the sensor data and execute the defined control tasks.
- Use the serial monitor to view the output and debug information.

## Contributing
Contributions are welcome! Please fork the repository and submit a pull request for any improvements or bug fixes.

## License
This project is licensed under the MIT License. See the LICENSE file for more details
