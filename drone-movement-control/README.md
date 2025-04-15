# C++ Drone Control System

This project is a prototype of a drone control system written in C++, focusing on architectural design and modular components. The system implements features such as attitude control, position control, data logging, and state management.

## Features

- Drone attitude control (Roll, Pitch, Yaw)
- PID algorithm-based position and speed control
- Motor output management
- Battery monitoring
- State management and event tracking
- Data logging (in both CSV and JSON formats)
- Expandable callback system

## System Architecture

The system is composed of the following main components:

### 1. DroneController

The core controller is responsible for:
- Processing sensor inputs
- Executing PID control computations
- Managing motor outputs
- Handling system state changes

### 2. Hardware Abstraction

- `MPU6050`: IMU sensor interface used for obtaining attitude data
- `MotorController`: Motor control interface
- `EncoderReader`: Encoder reader for position and speed measurements

### 3. Control Algorithms

- `PIDController`: Implements the PID control algorithm, supporting both position and speed control modes

### 4. State Management and Data Logging

- `DroneStatusInterface`: The state management and data export interface is used to:
  - Collect and record system status data
  - Export flight data in CSV format
  - Log event records in JSON format
  - Support callback functions for data processing and export

### 5. Callback System

The system features a rich set of callback interfaces to respond to various events:
- Attitude update callbacks
- Battery voltage update callbacks
- Motor output callbacks
- Error handling callbacks
- State change callbacks
- Data export callbacks

## Building and Running

### Dependencies

- C++17 or higher
- CMake 3.10+
- Linux system (for I2C and GPIO access)

### Build Steps

```bash
mkdir build
cd build
cmake ..
make

### Running the System

```bash
./cpp_drone

## Data Logging Files

The system will generate the following log files:

1. `drone_log.txt` - System log recording various events and state changes
2. `drone_status.csv` - Flight data logs in CSV format (attitude, battery voltage, motor outputs, etc.)
3. `drone_events.json` - Event logs in JSON format (errors, state changes, etc.)
