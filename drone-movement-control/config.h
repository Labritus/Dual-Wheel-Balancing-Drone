#ifndef CONFIG_H
#define CONFIG_H

// System parameters
constexpr int CONTROL_FREQUENCY = 20;      // Control frequency (Hz)
constexpr int CONTROL_INTERVAL_MS = 50;    // Control interval (ms)

// MPU6050 parameters
constexpr int MPU6050_I2C_ADDRESS = 0x68;  // I2C address of MPU6050
constexpr int DEFAULT_MPU_HZ = 200;        // MPU data update frequency

// Motor control parameters
constexpr int PWM_FREQUENCY = 10000;       // PWM frequency (Hz)
constexpr int PWM_MAX_VALUE = 6900;        // Maximum PWM value
constexpr int PWM_MIN_VALUE = -6900;       // Minimum PWM value

// Default PID parameters
constexpr float DEFAULT_POSITION_KP = 40.0f;
constexpr float DEFAULT_POSITION_KI = 0.1f;
constexpr float DEFAULT_POSITION_KD = 200.0f;
constexpr float DEFAULT_VELOCITY_KP = 5.0f;
constexpr float DEFAULT_VELOCITY_KI = 5.0f;

// PID amplitude limit parameters
constexpr float AMPLITUDE_PKP = 2.0f;
constexpr float AMPLITUDE_PKI = 0.1f;
constexpr float AMPLITUDE_PKD = 3.0f;
constexpr float AMPLITUDE_VKP = 1.0f;
constexpr float AMPLITUDE_VKI = 1.0f;

// Encoder parameters
constexpr int ENCODER_PERIOD = 65535;      // Encoder count period

// Battery voltage threshold (mV)
constexpr int BATTERY_MIN_VOLTAGE = 1000;  // Minimum battery voltage, motors stop below this value

// GPIO pin definitions (adjust according to your hardware)
// SPI pins for controlling MPU6050
constexpr int MPU6050_SPI_CHANNEL = 0;     // SPI channel
constexpr int MPU6050_CS_PIN = 8;          // Chip select pin

// Motor control pins
constexpr int MOTOR_AIN1_PIN = 17;         // Motor A direction control 1
constexpr int MOTOR_AIN2_PIN = 18;         // Motor A direction control 2
constexpr int MOTOR_PWMA_PIN = 12;         // Motor A PWM control

constexpr int MOTOR_BIN1_PIN = 22;         // Motor B direction control 1
constexpr int MOTOR_BIN2_PIN = 23;         // Motor B direction control 2
constexpr int MOTOR_PWMB_PIN = 13;         // Motor B PWM control

// Encoder pins
constexpr int ENCODER_A_PIN1 = 24;         // Encoder A channel 1
constexpr int ENCODER_A_PIN2 = 25;         // Encoder A channel 2
constexpr int ENCODER_B_PIN1 = 26;         // Encoder B channel 1
constexpr int ENCODER_B_PIN2 = 27;         // Encoder B channel 2

// LED pin
constexpr int LED_PIN = 5;                 // LED indicator pin

// Button pin
constexpr int KEY_PIN = 6;                 // Button pin

#endif // CONFIG_H
