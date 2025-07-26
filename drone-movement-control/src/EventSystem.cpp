#include "EventSystem.hpp"
#include "Delay.hpp"
#include <chrono>
#include <algorithm>

// Event timestamp implementation
uint64_t Event::getCurrentTimestamp() {
    return Timer::getMicroseconds();
}

// EventBus implementation
EventBus::EventBus() = default;

EventBus::~EventBus() {
    stop();
}

void EventBus::subscribe(std::shared_ptr<EventHandler> handler) {
    std::lock_guard<std::mutex> lock(handlers_mutex_);
    auto event_type = handler->getEventType();
    handlers_[event_type].push_back(handler);
}

void EventBus::publish(std::unique_ptr<Event> event) {
    {
        std::lock_guard<std::mutex> lock(queue_mutex_);
        event_queue_.push(std::move(event));
    }
    // No need to notify - using non-blocking polling instead
}

void EventBus::start() {
    if (running_.load()) {
        return;
    }
    
    running_.store(true);
    processing_thread_ = std::thread(&EventBus::processingLoop, this);
}

void EventBus::stop() {
    if (!running_.load()) {
        return;
    }
    
    running_.store(false);
    queue_cv_.notify_all();
    
    if (processing_thread_.joinable()) {
        processing_thread_.join();
    }
    
    // Process any remaining events
    processPendingEvents();
}

void EventBus::processPendingEvents() {
    std::unique_lock<std::mutex> lock(queue_mutex_);
    while (!event_queue_.empty()) {
        auto event = std::move(event_queue_.front());
        event_queue_.pop();
        lock.unlock();
        
        processEvent(std::move(event));
        
        lock.lock();
    }
}

void EventBus::processEventSync(const Event& event) {
    std::lock_guard<std::mutex> lock(handlers_mutex_);
    auto event_type = event.getType();
    auto it = handlers_.find(event_type);
    
    if (it != handlers_.end()) {
        for (const auto& handler : it->second) {
            try {
                handler->handle(event);
            } catch (const std::exception& e) {
                // Log error but continue processing
                // In production, you might want to use a proper logging system
            }
        }
    }
    
    processed_events_.fetch_add(1);
}

void EventBus::processEvent(std::unique_ptr<Event> event) {
    if (!event) return;
    
    processEventSync(*event);
}

void EventBus::processingLoop() {
    while (running_.load()) {
        std::unique_lock<std::mutex> lock(queue_mutex_);
        
        // Non-blocking event processing - check for events without blocking wait
        if (!event_queue_.empty()) {
            // Process all available events
            while (!event_queue_.empty() && running_.load()) {
                auto event = std::move(event_queue_.front());
                event_queue_.pop();
                lock.unlock();
                
                processEvent(std::move(event));
                
                lock.lock();
            }
        } else {
            // No events available - yield to other threads instead of blocking
            lock.unlock();
            std::this_thread::yield();
        }
    }
}

// GlobalEventBus implementation
std::unique_ptr<EventBus> GlobalEventBus::instance_ = nullptr;
std::mutex GlobalEventBus::instance_mutex_;

EventBus& GlobalEventBus::getInstance() {
    std::lock_guard<std::mutex> lock(instance_mutex_);
    if (!instance_) {
        instance_ = std::make_unique<EventBus>();
        instance_->start();
    }
    return *instance_;
}