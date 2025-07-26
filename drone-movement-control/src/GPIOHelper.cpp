#include "GPIOHelper.hpp"
#include <gpiod.h>
#include <iostream>
#include <poll.h>
#include <unistd.h>

std::unique_ptr<gpiod_chip, void(*)(gpiod_chip*)> GPIOHelper::chip_{nullptr, gpiod_chip_close};
std::unordered_map<unsigned int, gpiod_line_request*> GPIOHelper::requests_;
std::mutex GPIOHelper::gpio_mutex_;
std::atomic<bool> GPIOHelper::initialized_{false};

bool GPIOHelper::init(const char* chipname) {
    std::lock_guard<std::mutex> lock(gpio_mutex_);
    
    if (initialized_.load()) {
        return true;
    }
    
    chip_.reset(gpiod_chip_open(chipname));
    if (!chip_) {
        std::cerr << "Failed to open GPIO chip: " << chipname << std::endl;
        return false;
    }
    
    initialized_.store(true);
    return true;
}

void GPIOHelper::shutdown() {
    std::lock_guard<std::mutex> lock(gpio_mutex_);
    
    if (!initialized_.load()) {
        return;
    }
    
    for (auto& pair : requests_) {
        if (pair.second) {
            gpiod_line_request_release(pair.second);
        }
    }
    requests_.clear();
    chip_.reset();
    initialized_.store(false);
}

bool GPIOHelper::setMode(unsigned int pin, GPIOMode mode) {
    std::lock_guard<std::mutex> lock(gpio_mutex_);
    
    if (!initialized_.load() || !chip_) {
        return false;
    }
    
    auto existing_request = requests_.find(pin);
    if (existing_request != requests_.end()) {
        gpiod_line_request_release(existing_request->second);
        requests_.erase(existing_request);
    }
    
    struct gpiod_request_config *req_cfg = gpiod_request_config_new();
    if (!req_cfg) {
        std::cerr << "Failed to create request config" << std::endl;
        return false;
    }
    
    gpiod_request_config_set_consumer(req_cfg, CONSUMER);
    
    struct gpiod_line_config *line_cfg = gpiod_line_config_new();
    if (!line_cfg) {
        gpiod_request_config_free(req_cfg);
        std::cerr << "Failed to create line config" << std::endl;
        return false;
    }
    
    struct gpiod_line_settings *settings = gpiod_line_settings_new();
    if (!settings) {
        gpiod_line_config_free(line_cfg);
        gpiod_request_config_free(req_cfg);
        std::cerr << "Failed to create line settings" << std::endl;
        return false;
    }
    
    if (mode == GPIOMode::OUTPUT) {
        gpiod_line_settings_set_direction(settings, GPIOD_LINE_DIRECTION_OUTPUT);
        gpiod_line_settings_set_output_value(settings, GPIOD_LINE_VALUE_INACTIVE);
    } else {
        gpiod_line_settings_set_direction(settings, GPIOD_LINE_DIRECTION_INPUT);
    }
    
    if (gpiod_line_config_add_line_settings(line_cfg, &pin, 1, settings) < 0) {
        gpiod_line_settings_free(settings);
        gpiod_line_config_free(line_cfg);
        gpiod_request_config_free(req_cfg);
        std::cerr << "Failed to add line settings" << std::endl;
        return false;
    }
    
    gpiod_line_request* request = gpiod_chip_request_lines(chip_.get(), req_cfg, line_cfg);
    
    gpiod_line_settings_free(settings);
    gpiod_line_config_free(line_cfg);
    gpiod_request_config_free(req_cfg);
    
    if (!request) {
        std::cerr << "Failed to request GPIO line " << pin << std::endl;
        return false;
    }
    
    requests_[pin] = request;
    return true;
}

bool GPIOHelper::setValue(unsigned int pin, GPIOValue value) {
    std::lock_guard<std::mutex> lock(gpio_mutex_);
    
    if (!initialized_.load() || !chip_) {
        return false;
    }
    
    auto it = requests_.find(pin);
    if (it == requests_.end()) {
        if (!setMode(pin, GPIOMode::OUTPUT)) {
            return false;
        }
        it = requests_.find(pin);
    }
    
    gpiod_line_value val = (value == GPIOValue::HIGH) ? GPIOD_LINE_VALUE_ACTIVE : GPIOD_LINE_VALUE_INACTIVE;
    
    return gpiod_line_request_set_value(it->second, pin, val) == 0;
}

GPIOValue GPIOHelper::getValue(unsigned int pin) {
    std::lock_guard<std::mutex> lock(gpio_mutex_);
    
    if (!initialized_.load() || !chip_) {
        return GPIOValue::LOW;
    }
    
    auto it = requests_.find(pin);
    if (it == requests_.end()) {
        if (!setMode(pin, GPIOMode::INPUT)) {
            return GPIOValue::LOW;
        }
        it = requests_.find(pin);
    }
    
    gpiod_line_value value = gpiod_line_request_get_value(it->second, pin);
    
    if (value < 0) {
        return GPIOValue::LOW;
    }
    
    return (value == GPIOD_LINE_VALUE_ACTIVE) ? GPIOValue::HIGH : GPIOValue::LOW;
}

bool GPIOHelper::setupInterrupt(unsigned int pin, GPIOEdge edge) {
    std::lock_guard<std::mutex> lock(gpio_mutex_);
    
    if (!initialized_.load() || !chip_) {
        return false;
    }
    
    auto existing_request = requests_.find(pin);
    if (existing_request != requests_.end()) {
        gpiod_line_request_release(existing_request->second);
        requests_.erase(existing_request);
    }
    
    struct gpiod_request_config *req_cfg = gpiod_request_config_new();
    if (!req_cfg) {
        std::cerr << "Failed to create request config" << std::endl;
        return false;
    }
    
    gpiod_request_config_set_consumer(req_cfg, CONSUMER);
    
    struct gpiod_line_config *line_cfg = gpiod_line_config_new();
    if (!line_cfg) {
        gpiod_request_config_free(req_cfg);
        std::cerr << "Failed to create line config" << std::endl;
        return false;
    }
    
    struct gpiod_line_settings *settings = gpiod_line_settings_new();
    if (!settings) {
        gpiod_line_config_free(line_cfg);
        gpiod_request_config_free(req_cfg);
        std::cerr << "Failed to create line settings" << std::endl;
        return false;
    }
    
    gpiod_line_settings_set_direction(settings, GPIOD_LINE_DIRECTION_INPUT);
    
    switch (edge) {
        case GPIOEdge::RISING:
            gpiod_line_settings_set_edge_detection(settings, GPIOD_LINE_EDGE_RISING);
            break;
        case GPIOEdge::FALLING:
            gpiod_line_settings_set_edge_detection(settings, GPIOD_LINE_EDGE_FALLING);
            break;
        case GPIOEdge::BOTH:
            gpiod_line_settings_set_edge_detection(settings, GPIOD_LINE_EDGE_BOTH);
            break;
        default:
            gpiod_line_settings_free(settings);
            gpiod_line_config_free(line_cfg);
            gpiod_request_config_free(req_cfg);
            return false;
    }
    
    if (gpiod_line_config_add_line_settings(line_cfg, &pin, 1, settings) < 0) {
        gpiod_line_settings_free(settings);
        gpiod_line_config_free(line_cfg);
        gpiod_request_config_free(req_cfg);
        std::cerr << "Failed to add line settings" << std::endl;
        return false;
    }
    
    gpiod_line_request* request = gpiod_chip_request_lines(chip_.get(), req_cfg, line_cfg);
    
    gpiod_line_settings_free(settings);
    gpiod_line_config_free(line_cfg);
    gpiod_request_config_free(req_cfg);
    
    if (!request) {
        std::cerr << "Failed to setup interrupt for GPIO line " << pin << std::endl;
        return false;
    }
    
    requests_[pin] = request;
    return true;
}

bool GPIOHelper::waitForEdge(unsigned int pin, int timeout_ms) {
    std::lock_guard<std::mutex> lock(gpio_mutex_);
    
    if (!initialized_.load() || !chip_) {
        return false;
    }
    
    auto it = requests_.find(pin);
    if (it == requests_.end()) {
        return false;
    }
    
    int fd = gpiod_line_request_get_fd(it->second);
    if (fd < 0) {
        return false;
    }
    
    // Non-blocking edge detection - use timeout 0 for immediate check
    struct pollfd pfd;
    pfd.fd = fd;
    pfd.events = POLLIN | POLLPRI;
    pfd.revents = 0;
    
    // Use non-blocking poll (timeout = 0) to avoid blocking the thread
    int ret = poll(&pfd, 1, 0);  // 0 timeout = non-blocking
    
    if (ret > 0 && (pfd.revents & (POLLIN | POLLPRI))) {
        gpiod_edge_event_buffer* buffer = gpiod_edge_event_buffer_new(1);
        if (buffer) {
            ret = gpiod_line_request_read_edge_events(it->second, buffer, 1);
            gpiod_edge_event_buffer_free(buffer);
            return ret > 0;
        }
    }
    
    // For applications needing timeout behavior, they should use polling timer
    if (timeout_ms > 0) {
        printf("INFO: GPIOHelper::waitForEdge made non-blocking. "
               "For timeout behavior, use Timer::schedulePeriodicCallback\n");
    }
    
    return false;
}