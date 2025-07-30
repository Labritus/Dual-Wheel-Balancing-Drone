#ifndef PID_CONTROLLER_HPP
#define PID_CONTROLLER_HPP

class PIDController {
private:
    // PID parameters
    float kp_;          // Proportional gain
    float ki_;          // Integral gain
    float kd_;          // Derivative gain
    
    // PID variables
    float setpoint_;    // Target value
    float integral_;    // Integral sum
    float prev_error_;  // Previous error for derivative calculation
    float output_min_;  // Minimum output limit
    float output_max_;  // Maximum output limit
    
    // Anti-windup parameters
    bool anti_windup_enabled_;
    float integral_limit_;
    
public:
    /**
     * Constructor
     * @param kp Proportional gain
     * @param ki Integral gain
     * @param kd Derivative gain
     * @param output_min Minimum output value
     * @param output_max Maximum output value
     */
    PIDController(float kp = 0.0f, float ki = 0.0f, float kd = 0.0f,
                  float output_min = -1.0f, float output_max = 1.0f);
    
    /**
     * Destructor
     */
    ~PIDController() = default;
    
    /**
     * Set PID parameters
     * @param kp Proportional gain
     * @param ki Integral gain
     * @param kd Derivative gain
     */
    void setParameters(float kp, float ki, float kd);
    
    /**
     * Set output limits
     * @param min Minimum output value
     * @param max Maximum output value
     */
    void setOutputLimits(float min, float max);
    
    /**
     * Enable/disable anti-windup protection
     * @param enabled Whether to enable anti-windup
     * @param limit Integral limit value
     */
    void setAntiWindup(bool enabled, float limit = 0.0f);
    
    /**
     * Set target setpoint
     * @param setpoint Target value
     */
    void setSetpoint(float setpoint);
    
    /**
     * Reset PID controller state
     */
    void reset();
    
    /**
     * Compute PID output
     * @param process_value Current process value
     * @param dt Time since last call (in seconds)
     * @return Calculated output
     */
    float compute(float process_value, float dt);
    
    /**
     * Get current setpoint
     * @return Current target value
     */
    float getSetpoint() const { return setpoint_; }
    
    /**
     * Get current PID parameters
     * @param kp Reference to store proportional gain
     * @param ki Reference to store integral gain
     * @param kd Reference to store derivative gain
     */
    void getParameters(float& kp, float& ki, float& kd) const;
};

#endif // PID_CONTROLLER_HPP