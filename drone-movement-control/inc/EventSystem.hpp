#ifndef __EVENT_SYSTEM_HPP
#define __EVENT_SYSTEM_HPP

#include <functional>
#include <vector>
#include <unordered_map>
#include <memory>
#include <mutex>
#include <atomic>
#include <queue>
#include <thread>
#include <condition_variable>
#include <typeindex>
#include <any>

// Base event class
class Event {
public:
    virtual ~Event() = default;
    virtual std::type_index getType() const = 0;
    
    // Event timestamp
    uint64_t timestamp;
    
protected:
    Event() : timestamp(getCurrentTimestamp()) {}
    
private:
    static uint64_t getCurrentTimestamp();
};

// Templated event class
template<typename T>
class TypedEvent : public Event {
public:
    explicit TypedEvent(T data) : data_(std::move(data)) {}
    
    std::type_index getType() const override {
        return std::type_index(typeid(T));
    }
    
    const T& getData() const { return data_; }
    T& getData() { return data_; }
    
private:
    T data_;
};

// Event handler interface
class EventHandler {
public:
    virtual ~EventHandler() = default;
    virtual void handle(const Event& event) = 0;
    virtual std::type_index getEventType() const = 0;
};

// Templated event handler
template<typename EventType>
class TypedEventHandler : public EventHandler {
public:
    using HandlerFunction = std::function<void(const EventType&)>;
    
    explicit TypedEventHandler(HandlerFunction handler) 
        : handler_(std::move(handler)) {}
    
    void handle(const Event& event) override {
        const auto* typed_event = dynamic_cast<const TypedEvent<EventType>*>(&event);
        if (typed_event) {
            handler_(typed_event->getData());
        }
    }
    
    std::type_index getEventType() const override {
        return std::type_index(typeid(EventType));
    }
    
private:
    HandlerFunction handler_;
};

// Event bus for fast, lock-free event processing
class EventBus {
public:
    EventBus();
    ~EventBus();
    
    // Subscribe to events
    template<typename EventType>
    void subscribe(std::function<void(const EventType&)> handler) {
        auto typed_handler = std::make_shared<TypedEventHandler<EventType>>(handler);
        subscribe(typed_handler);
    }
    
    void subscribe(std::shared_ptr<EventHandler> handler);
    
    // Publish events (thread-safe)
    template<typename EventType>
    void publish(EventType event) {
        auto typed_event = std::make_unique<TypedEvent<EventType>>(std::move(event));
        publish(std::move(typed_event));
    }
    
    void publish(std::unique_ptr<Event> event);
    
    // Synchronous event processing (for real-time critical events)
    template<typename EventType>
    void publishSync(EventType event) {
        auto typed_event = std::make_unique<TypedEvent<EventType>>(std::move(event));
        processEventSync(*typed_event);
    }
    
    // Start/stop event processing thread
    void start();
    void stop();
    
    // Process pending events (can be called from main thread)
    void processPendingEvents();
    
    // Statistics
    size_t getQueueSize() const { return event_queue_.size(); }
    uint64_t getProcessedEventCount() const { return processed_events_.load(); }
    
private:
    std::unordered_map<std::type_index, std::vector<std::shared_ptr<EventHandler>>> handlers_;
    mutable std::mutex handlers_mutex_;
    
    std::queue<std::unique_ptr<Event>> event_queue_;
    mutable std::mutex queue_mutex_;
    std::condition_variable queue_cv_;
    
    std::atomic<bool> running_{false};
    std::thread processing_thread_;
    
    std::atomic<uint64_t> processed_events_{0};
    
    void processEventSync(const Event& event);
    void processEvent(std::unique_ptr<Event> event);
    void processingLoop();
};

// Specific event types for the balance system
struct SensorDataEvent {
    float angle;
    float gyro;
    float acceleration_z;
    uint32_t distance;
    int temperature;
    int voltage;
    uint64_t timestamp;
};

struct MotorControlEvent {
    int left_speed;
    int right_speed;
    bool emergency_stop;
};

struct SystemStateEvent {
    enum State {
        INITIALIZING,
        RUNNING,
        BALANCING,
        FOLLOWING,
        AVOIDING,
        STOPPED,
        ERROR
    };
    
    State current_state;
    State previous_state;
    std::string message;
};

struct UserCommandEvent {
    enum Command {
        START,
        STOP,
        CALIBRATE,
        FOLLOW_MODE,
        AVOID_MODE,
        RESET
    };
    
    Command command;
    std::unordered_map<std::string, std::any> parameters;
};

struct LatencyViolationEvent {
    std::string component;
    uint64_t measured_latency_us;
    uint64_t max_allowed_us;
    float severity; // 0.0 - 1.0
};

// Global event bus instance
class GlobalEventBus {
public:
    static EventBus& getInstance();
    
private:
    static std::unique_ptr<EventBus> instance_;
    static std::mutex instance_mutex_;
};

// Convenience macros for event handling
#define SUBSCRIBE_EVENT(EventType, handler) \
    GlobalEventBus::getInstance().subscribe<EventType>(handler)

#define PUBLISH_EVENT(EventType, ...) \
    GlobalEventBus::getInstance().publish<EventType>({__VA_ARGS__})

#define PUBLISH_EVENT_SYNC(EventType, ...) \
    GlobalEventBus::getInstance().publishSync<EventType>({__VA_ARGS__})

// RAII event subscription helper
template<typename EventType>
class ScopedEventSubscription {
public:
    explicit ScopedEventSubscription(std::function<void(const EventType&)> handler) {
        GlobalEventBus::getInstance().subscribe<EventType>(handler);
    }
};

#define SCOPED_EVENT_SUBSCRIPTION(EventType, handler) \
    ScopedEventSubscription<EventType> _subscription(handler)

#endif // __EVENT_SYSTEM_HPP