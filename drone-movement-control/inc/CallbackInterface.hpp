#ifndef CALLBACKINTERFACE_HPP
#define CALLBACKINTERFACE_HPP

// Forward declarations
class SensorData;
class ControlEvent;

/**
 * @brief Sensor data callback interface
 */
class SensorCallback {
public:
    virtual ~SensorCallback() = default;
    /**
     * @brief Sensor data update callback
     * @param data Latest sensor data
     */
    virtual void onSensorDataUpdated(const SensorData& data) = 0;
};

/**
 * @brief Control event callback interface
 */
class ControlCallback {
public:
    virtual ~ControlCallback() = default;
    /**
     * @brief Control event trigger callback
     * @param event Control event that occurred
     */
    virtual void onControlEvent(const ControlEvent& event) = 0;
};

/**
 * @brief Sensor data structure
 */
class SensorData {
public:
    float angleBalance;
    float gyroBalance;
    float gyroTurn;
    int accelerationZ;
    int encoderLeft;
    int encoderRight;
    int voltage;
    float distance;
};

/**
 * @brief Control event structure
 */
class ControlEvent {
public:
    enum Type {
        EVENT_STOP,
        EVENT_START,
        EVENT_ERROR,
        EVENT_WARNING,
        EVENT_BUTTON_PRESSED
    };

    Type type;
    int data;
};

#endif // CALLBACKINTERFACE_HPP