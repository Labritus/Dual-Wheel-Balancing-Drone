#include "I2CSlave.hpp"
#include "System.hpp"
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <thread>
#include <atomic>
#include <mutex>
#include <errno.h> // Required for errno
#include <stdio.h> // Required for printf
#include <string.h> // Required for strerror
#include <fcntl.h> // Required for fcntl

// Buffer size
#define I2C_SLAVE_BUFFER_SIZE 32

// Data buffer and state variables
static uint8_t i2c_rx_buffer[I2C_SLAVE_BUFFER_SIZE];
static std::atomic<uint8_t> i2c_rx_count{0};
static std::atomic<bool> message_ready{false};
static std::atomic<bool> i2c_running{false};
static std::thread i2c_thread;
static std::mutex buffer_mutex;
static int i2c_fd = -1;

// Initialize I2C slave on Raspberry Pi
void I2CSlave::init() {
    // Open I2C device
    i2c_fd = open("/dev/i2c-1", O_RDWR);
    if (i2c_fd < 0) {
        return; // Failed to open I2C device
    }
    
    // Set slave address (0x08)
    if (ioctl(i2c_fd, I2C_SLAVE, 0x08) < 0) {
        close(i2c_fd);
        i2c_fd = -1;
        return;
    }
    
    // Start I2C reading thread
    i2c_running.store(true);
    i2c_thread = std::thread([]() {
        uint8_t temp_buffer[I2C_SLAVE_BUFFER_SIZE];
        
        // Set I2C file descriptor to non-blocking mode to prevent blocking reads
        int flags = fcntl(i2c_fd, F_GETFL, 0);
        if (flags >= 0) {
            fcntl(i2c_fd, F_SETFL, flags | O_NONBLOCK);
        }
        
        while (i2c_running.load()) {
            // Try to read from I2C (now non-blocking)
            ssize_t bytes_read = read(i2c_fd, temp_buffer, I2C_SLAVE_BUFFER_SIZE - 1);
            
            if (bytes_read > 0) {
                std::lock_guard<std::mutex> lock(buffer_mutex);
                
                // Copy data to buffer
                memcpy(i2c_rx_buffer, temp_buffer, bytes_read);
                i2c_rx_buffer[bytes_read] = '\0';  // Null terminate
                i2c_rx_count.store(bytes_read);
                message_ready.store(true);
            } else if (bytes_read < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
                // Handle actual errors (not just "no data available")
                printf("WARNING: I2C read error: %s\n", strerror(errno));
            }
            
            // Non-blocking yield for real-time systems
            std::this_thread::yield();
        }
    });
}

// Check if a new message was received
bool I2CSlave::hasNewMessage() {
    return message_ready.load();
}

// Clear message flag
void I2CSlave::clearMessageFlag() {
    std::lock_guard<std::mutex> lock(buffer_mutex);
    message_ready.store(false);
    i2c_rx_count.store(0);
}

// Cleanup I2C resources
void I2CSlave::cleanup() {
    i2c_running.store(false);
    
    if (i2c_thread.joinable()) {
        i2c_thread.join();
    }
    
    if (i2c_fd >= 0) {
        close(i2c_fd);
        i2c_fd = -1;
    }
}

// Check if received message is "PERSON_DETECTED"
bool I2CSlave::isPersonDetected() {
    const char* person_detected = "PERSON_DETECTED";
    size_t expected_len = strlen(person_detected);
    
    std::lock_guard<std::mutex> lock(buffer_mutex);
    
    if (i2c_rx_count.load() != expected_len) {
        return false;
    }

    // Compare received message with "PERSON_DETECTED"
    return memcmp(i2c_rx_buffer, person_detected, expected_len) == 0;
}

// Get current buffer contents (for debugging)
std::string I2CSlave::getLastMessage() {
    std::lock_guard<std::mutex> lock(buffer_mutex);
    return std::string(reinterpret_cast<char*>(i2c_rx_buffer), i2c_rx_count.load());
}

// Check if I2C is properly initialized
bool I2CSlave::isInitialized() {
    return i2c_fd >= 0 && i2c_running.load();
}
