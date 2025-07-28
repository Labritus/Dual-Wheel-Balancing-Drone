# Dual-Wheel Balancing Drone Control System

A real-time C++ control system for dual-wheel balancing drones, featuring advanced IMU-based stabilization, PID control algorithms, and comprehensive hardware abstraction. This system is designed for embedded Linux platforms and supports real-time operation with microsecond-precision timing.

## 🚀 Features

### Core Control Systems
- **Real-time balance control** with Kalman filtering and complementary filters
- **Dual PID controllers** for position and velocity control
- **MPU6050 IMU integration** with DMP (Digital Motion Processor) support
- **Motor speed control** with encoder feedback
- **Ultrasonic distance sensing** for obstacle detection
- **I2C slave communication** for external command interface

### Hardware Abstraction & Interfaces
- **GPIO control** via libgpiod v2.0+ for cross-platform compatibility
- **I2C communication** for sensor and peripheral interfaces
- **USART/Serial communication** for debugging and telemetry
- **OLED display support** for real-time status visualization
- **LED status indicators** and key input handling

### Advanced Features
- **Real-time threading** with priority scheduling
- **Memory management** with custom allocators
- **Event system** for asynchronous operations
- **Latency monitoring** and performance profiling
- **Comprehensive testing framework** with unit and integration tests
- **Data logging** in CSV and JSON formats
- **Callback system** for extensible functionality

## 🏗️ System Architecture

### Core Components

#### 1. Balance Control (`Balance.hpp`)
Real-time balance controller implementing:
- Complementary and Kalman filtering for attitude estimation
- PID-based stabilization algorithms
- Motor output management
- Sensor fusion from MPU6050

#### 2. Hardware Abstraction Layer
- **`MPU6050.hpp`**: IMU sensor interface with DMP support
- **`Motor.hpp`**: PWM motor control with speed feedback
- **`GPIOHelper.hpp`**: Cross-platform GPIO operations via libgpiod
- **`I2CHelper.hpp`**: I2C bus communication abstraction
- **`Ultrasonic.hpp`**: Distance measurement interface

#### 3. Control Systems (`Control.hpp`)
- **`PIDController.hpp`**: Configurable PID implementation
- **`MotorController.h`**: High-level motor management
- **`Filter.hpp`**: Digital signal processing filters
- **`KalmanFilter.hpp`**: State estimation algorithms

#### 4. Real-Time Infrastructure
- **`RealTimeThread.hpp`**: Priority-based threading system
- **`LatencyMonitor.hpp`**: Performance measurement tools
- **`MemoryManager.hpp`**: Real-time memory allocation
- **`EventSystem.hpp`**: Asynchronous event handling

#### 5. Communication & Display
- **`I2CSlave.hpp`**: External command interface
- **`USART.hpp`/`USART3.hpp`**: Serial communication
- **`OLED.hpp`**: Real-time status display
- **`Show.hpp`**: Data visualization utilities

## 🛠️ System Requirements

### Hardware Requirements
- **Linux-based system** (Raspberry Pi 4 recommended)
- **GPIO pins** available for hardware interface
- **I2C bus support** for sensor communication
- **USB ports** for additional peripherals

### Software Requirements
- **Operating System**: Linux (Ubuntu 18.04+, Raspberry Pi OS)
- **Compiler**: GCC/G++ with C++17 support
- **Build System**: CMake 3.10 or higher
- **Threading**: POSIX threads (pthread)

## 📦 Dependencies & Installation

### Required System Packages

#### Ubuntu/Debian Systems:
```bash
sudo apt update
sudo apt install -y \
    build-essential \
    cmake \
    pkg-config \
    libgpiod-dev \
    libgpiod2 \
    libi2c-dev \
    i2c-tools \
    git
```

#### Raspberry Pi OS:
```bash
sudo apt update
sudo apt install -y \
    build-essential \
    cmake \
    pkg-config \
    libgpiod-dev \
    libgpiod2 \
    libi2c-dev \
    raspberrypi-kernel-headers \
    i2c-tools \
    git
```

### Critical Dependency: libgpiod v2.0+
This project requires **libgpiod version 2.0 or higher** for GPIO control. Verify installation:

```bash
pkg-config --modversion libgpiod
```

If the version is below 2.0, compile from source:

```bash
# Download and compile libgpiod v2.0+
git clone https://git.kernel.org/pub/scm/libs/libgpiod/libgpiod.git
cd libgpiod
git checkout v2.0
./autogen.sh --enable-tools=yes --prefix=/usr/local
make -j$(nproc)
sudo make install
sudo ldconfig
```

## 🔧 Build Process

### Step-by-Step Build Instructions

#### 1. Clone and Navigate to Project
```bash
git clone <repository-url>
cd Dual-Wheel-Balancing-Drone-final/Dual-Wheel-Balancing-Drone-main/drone-movement-control
```

#### 2. Create Build Directory
```bash
mkdir build
cd build
```

#### 3. Configure with CMake
```bash
cmake ..
```

**Note**: CMake will automatically check for dependencies and display status messages:
- libgpiod version and configuration
- Include directories and libraries
- Compilation flags

#### 4. Compile the Project
```bash
make -j$(nproc)
```

For verbose output to debug compilation issues:
```bash
make VERBOSE=1
```

#### 5. Verify Build Success
After successful compilation, you should see the following executables in the build directory:
- `balance_drone` - Main drone control application
- `balance_drone_tests` - Test suite
- `callback_test_app` - Legacy callback testing utility

### Build Targets
```bash
# Main executable
make balance_drone

# Test suite
make balance_drone_tests

# Run all tests
make run_tests
# Or directly:
./balance_drone_tests

# Performance benchmarks
make performance_test
# Or directly:
./balance_drone_tests --performance

# Legacy callback tests
make callback_test_app
```

### Configuration Options
```bash
# Debug build (with debugging symbols)
cmake -DCMAKE_BUILD_TYPE=Debug ..
make

# Release build (optimized for performance)
cmake -DCMAKE_BUILD_TYPE=Release ..
make

# Custom compiler flags
cmake -DCMAKE_CXX_FLAGS="-O3 -march=native" ..
make

# Custom installation directory
cmake -DCMAKE_INSTALL_PREFIX=/opt/drone ..
make
```

### System Installation
```bash
# Install to system directories
sudo make install
```

This installs:
- Executables to `/usr/local/bin/`
- Documentation to `/usr/local/share/doc/balance_drone/`

### Build Verification
```bash
# Check executable dependencies
ldd balance_drone

# Verify GPIO library linking
ldd balance_drone | grep gpiod

# Test basic functionality
./balance_drone --help
```

## 🚁 Running the System

### Prerequisites
1. **GPIO Permissions**:
```bash
sudo usermod -a -G gpio $USER
newgrp gpio
```

2. **I2C Interface** (Raspberry Pi):
```bash
sudo raspi-config
# Interface Options > I2C > Enable
```

3. **Hardware Connections**: Ensure proper wiring of:
   - MPU6050 IMU (I2C: SDA/SCL)
   - Motors (PWM pins)
   - Ultrasonic sensor (GPIO)
   - OLED display (I2C)

### Execution
```bash
# Run main control system
./balance_drone

# Run with elevated privileges if needed
sudo ./balance_drone

# Execute test suite
./balance_drone_tests

# Run performance tests
./balance_drone_tests --performance
```

## 📊 Data Output & Monitoring

The system generates comprehensive telemetry data:

### Log Files
- **System logs**: Real-time events and state changes
- **CSV data**: Sensor readings, control outputs, timestamps
- **JSON events**: Error conditions, state transitions, performance metrics

### Real-Time Monitoring
- **OLED display**: Live attitude, motor speeds, system status
- **Serial output**: Debug information and telemetry
- **Performance metrics**: Loop timing, latency measurements

## 🧪 Testing & Validation

### Test Categories
- **Unit tests**: Individual component validation
- **Integration tests**: System-level functionality
- **Performance tests**: Real-time constraints verification
- **Hardware-in-the-loop**: Sensor and actuator testing

### Running Tests
```bash
# All tests
./balance_drone_tests

# Specific test categories
./balance_drone_tests --gtest_filter="Balance*"
./balance_drone_tests --gtest_filter="PID*"

# Performance benchmarks
./balance_drone_tests --performance
```

## 🔧 Hardware Setup

### Required Components
- **Microcontroller**: Raspberry Pi 4 or compatible Linux SBC
- **IMU**: MPU6050 6-axis gyroscope/accelerometer
- **Motors**: DC motors with encoders
- **Distance sensor**: HC-SR04 ultrasonic sensor
- **Display**: SSD1306 OLED (optional)
- **Power supply**: Appropriate for motors and electronics

### Pin Configuration
Refer to your specific hardware setup for GPIO pin assignments:
- I2C1: SDA (GPIO2), SCL (GPIO3)
- Motor PWM: Configurable GPIO pins
- Ultrasonic: Trigger/Echo pins
- Status LEDs: Configurable GPIO pins

### I2C Device Detection
```bash
# Install i2c-tools
sudo apt install i2c-tools

# Scan for connected devices
sudo i2cdetect -y 1
```

## 🚨 Troubleshooting

### Common Build Issues

#### 1. libgpiod Not Found
```
CMake Error: Could NOT find PkgConfig (missing: PKG_CONFIG_EXECUTABLE)
```
**Solution**: Install pkg-config and libgpiod-dev packages:
```bash
sudo apt install pkg-config libgpiod-dev libgpiod2
```

#### 2. GPIO Permission Denied
```
Error: Failed to open GPIO chip
```
**Solution**: 
- Run with sudo privileges, or
- Add user to gpio group: `sudo usermod -a -G gpio $USER`
- Log out and back in, or run: `newgrp gpio`

#### 3. I2C Device Not Found
```
Error: Failed to open I2C device
```
**Solution**:
- Enable I2C interface in system configuration:
  ```bash
  sudo raspi-config
  # Navigate to: Interface Options > I2C > Enable
  ```
- Check device permissions: `ls -l /dev/i2c-*`
- Add user to i2c group: `sudo usermod -a -G i2c $USER`

#### 4. Compilation Errors
```
error: 'thread' is not a member of 'std'
```
**Solution**: Ensure C++17 support and pthread linking are properly configured

#### 5. Real-Time Performance Issues
```bash
# Check system load and CPU governor
cat /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor
# Set to 'performance' for real-time operation
echo performance | sudo tee /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor
```

### Runtime Issues

**Hardware Detection Problems**
```bash
# Check I2C devices
sudo i2cdetect -y 1

# Check GPIO chip availability
ls /dev/gpiochip*

# Verify permissions
groups $USER
```

**Memory or Threading Issues**
- Ensure sufficient system memory (recommended: 1GB+ available)
- Check for conflicting processes using GPIO/I2C resources
- Verify real-time scheduling permissions

## 🤝 Development

### Code Structure
```
├── inc/              # Header files
│   ├── control_classes/  # Control algorithm headers
│   └── dmp/             # MPU6050 DMP headers
├── src/              # Implementation files
│   ├── control_classes/  # Control algorithm implementations
│   └── dmp/             # MPU6050 DMP implementations
├── tests/            # Unit and integration tests
└── CMakeLists.txt    # Build configuration
```

### Adding New Features
1. Add source files to `src/` directory
2. Add headers to `inc/` directory
3. Update CMakeLists.txt if needed
4. Rebuild with `make`

### Code Style Guidelines
- Follow existing C++17 conventions
- Use proper header guards
- Implement proper error handling
- Add unit tests for new functionality

### Contributing Guidelines
- Follow existing C++17 code style
- Add unit tests for new features
- Update documentation for API changes
- Ensure real-time constraints are maintained
- Test on target hardware platforms

### Performance Notes
- The system is optimized for real-time performance
- Uses O2/O3 optimization levels
- Implements multi-threading for concurrent operations
- Designed for low-latency sensor processing

## 📄 License

This project is developed for educational and research purposes. Ensure proper safety measures when working with drone hardware.

## 🔗 Additional Resources

- [MPU6050 Documentation](MPU6050%20Descriptions.pdf) - Sensor specifications
- Hardware schematics and wiring diagrams in parent directory
- CMakeLists.txt - Complete build configuration with all targets and dependencies

## 📞 Support

For build issues or questions:
1. Check this README thoroughly for troubleshooting steps
2. Verify all dependencies are correctly installed
3. Ensure hardware connections are proper
4. Check system logs for detailed error messages
5. Test individual components using the provided test suite

---

**⚠️ Safety Notice**: This system controls physical hardware. Always test in a safe environment with proper safety measures in place.
