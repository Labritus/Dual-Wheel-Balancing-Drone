#include "../inc/TestFramework.hpp"
#include "../inc/Balance.hpp"
#include "../inc/LatencyMonitor.hpp"
#include "../inc/EventSystem.hpp"
#include "../inc/MemoryManager.hpp"
#include "../inc/System.hpp"

// Test Balance system initialization
TEST(BalanceSystem, Initialization) {
    ASSERT_TRUE(System::init());
    ASSERT_TRUE(Balance::init());
    ASSERT_TRUE(Balance::isInitialized());
    ASSERT_TRUE(Balance::isRunning());
}

// Test Balance system thread safety
TEST(BalanceSystem, ThreadSafety) {
    Balance::init();
    
    // Test concurrent access to sensor data
    std::vector<std::thread> threads;
    std::atomic<bool> test_running{true};
    std::atomic<int> error_count{0};
    
    // Start multiple threads reading sensor data
    for (int i = 0; i < 4; ++i) {
        threads.emplace_back([&]() {
            while (test_running.load()) {
                try {
                    float angle = Balance::getCurrentAngle();
                    float gyro = Balance::getCurrentGyro();
                    int left_speed = Balance::getLeftSpeed();
                    int right_speed = Balance::getRightSpeed();
                    
                    // Basic sanity checks
                    ASSERT_GE(angle, -180.0f);
                    ASSERT_LE(angle, 180.0f);
                    
                } catch (...) {
                    error_count.fetch_add(1);
                }
                
                std::this_thread::sleep_for(std::chrono::microseconds(100));
            }
        });
    }
    
    // Let threads run for a short time
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    test_running.store(false);
    
    // Wait for all threads to complete
    for (auto& t : threads) {
        t.join();
    }
    
    ASSERT_EQ(error_count.load(), 0);
}

// Test Balance system callback mechanism
TEST(BalanceSystem, CallbackMechanism) {
    Balance::init();
    
    std::atomic<bool> callback_invoked{false};
    std::atomic<int> callback_count{0};
    
    Balance::setCallback([&](float angle, float gyro, int left_speed, int right_speed) {
        callback_invoked.store(true);
        callback_count.fetch_add(1);
        
        // Verify callback parameters are reasonable
        ASSERT_GE(angle, -180.0f);
        ASSERT_LE(angle, 180.0f);
    });
    
    // Process some iterations
    for (int i = 0; i < 10; ++i) {
        Balance::processOnce();
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    
    ASSERT_TRUE(callback_invoked.load());
    ASSERT_GT(callback_count.load(), 0);
}

// Test latency monitoring system
TEST(LatencyMonitor, BasicFunctionality) {
    LatencyMonitor monitor("Test Monitor", 1000); // 1ms threshold
    
    // Test measurement
    monitor.startMeasurement();
    std::this_thread::sleep_for(std::chrono::microseconds(500)); // 0.5ms
    monitor.endMeasurement();
    
    ASSERT_EQ(monitor.getViolationCount(), 0);
    ASSERT_GT(monitor.getMaxLatency(), 400); // Should be around 500µs
    ASSERT_LT(monitor.getMaxLatency(), 700); // But with some tolerance
}

// Test latency violation detection
TEST(LatencyMonitor, ViolationDetection) {
    LatencyMonitor monitor("Violation Test", 500); // 0.5ms threshold
    
    std::atomic<bool> violation_detected{false};
    monitor.setViolationCallback([&](const std::string& name, uint64_t latency) {
        violation_detected.store(true);
        ASSERT_EQ(name, "Violation Test");
        ASSERT_GT(latency, 500);
    });
    
    // Cause a violation
    monitor.startMeasurement();
    std::this_thread::sleep_for(std::chrono::milliseconds(1)); // 1ms > 0.5ms threshold
    monitor.endMeasurement();
    
    ASSERT_TRUE(violation_detected.load());
    ASSERT_EQ(monitor.getViolationCount(), 1);
}

// Test scoped latency measurement
TEST(LatencyMonitor, ScopedMeasurement) {
    LatencyMonitor monitor("Scoped Test", 2000); // 2ms threshold
    
    {
        MEASURE_LATENCY(monitor);
        std::this_thread::sleep_for(std::chrono::microseconds(500));
    }
    
    ASSERT_EQ(monitor.getMeasurementCount(), 1);
    ASSERT_EQ(monitor.getViolationCount(), 0);
}

// Test event system basic functionality
TEST(EventSystem, BasicPublishSubscribe) {
    auto& event_bus = GlobalEventBus::getInstance();
    
    std::atomic<bool> event_received{false};
    std::atomic<float> received_angle{0.0f};
    
    // Subscribe to sensor data events
    SUBSCRIBE_EVENT(SensorDataEvent, [&](const SensorDataEvent& event) {
        event_received.store(true);
        received_angle.store(event.angle);
    });
    
    // Publish an event
    PUBLISH_EVENT_SYNC(SensorDataEvent, 45.0f, 10.0f, 2.0f, 100, 25, 12000, Timer::getMicroseconds());
    
    ASSERT_TRUE(event_received.load());
    ASSERT_FLOAT_EQ(received_angle.load(), 45.0f, 0.01f);
}

// Test event system asynchronous processing
TEST(EventSystem, AsynchronousProcessing) {
    auto& event_bus = GlobalEventBus::getInstance();
    
    std::atomic<int> events_processed{0};
    
    SUBSCRIBE_EVENT(SystemStateEvent, [&](const SystemStateEvent& event) {
        events_processed.fetch_add(1);
    });
    
    // Publish multiple events
    for (int i = 0; i < 5; ++i) {
        PUBLISH_EVENT(SystemStateEvent, SystemStateEvent::RUNNING, SystemStateEvent::INITIALIZING, "Test message");
    }
    
    // Wait for asynchronous processing
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    event_bus.processPendingEvents();
    
    ASSERT_EQ(events_processed.load(), 5);
}

// Test memory management with RAII
TEST(MemoryManager, RAIIPatterns) {
    auto& memory_manager = MemoryManager::getInstance();
    
    // Test UniqueResource
    {
        int* ptr = static_cast<int*>(MEM_ALLOC(sizeof(int)));
        *ptr = 42;
        
        auto resource = memory_manager.makeUniqueResource(ptr);
        ASSERT_EQ(*resource, 42);
        
        // Resource should be automatically freed when going out of scope
    }
    
    // Test scoped cleanup
    bool cleanup_called = false;
    {
        SCOPED_CLEANUP(cleanup_called = true);
        // cleanup_called should be false here
        ASSERT_FALSE(cleanup_called);
    }
    // cleanup_called should be true after scope exit
    ASSERT_TRUE(cleanup_called);
}

// Test memory pool allocation
TEST(MemoryManager, MemoryPoolAllocation) {
    // Test int pool
    int* int_ptr = pool_allocate<int>();
    ASSERT_NE(int_ptr, nullptr);
    *int_ptr = 123;
    ASSERT_EQ(*int_ptr, 123);
    pool_deallocate(int_ptr);
    
    // Test float pool
    float* float_ptr = pool_allocate<float>();
    ASSERT_NE(float_ptr, nullptr);
    *float_ptr = 3.14f;
    ASSERT_FLOAT_EQ(*float_ptr, 3.14f, 0.001f);
    pool_deallocate(float_ptr);
}

// Performance test for Balance::processOnce()
TEST(BalanceSystem, ProcessOncePerformance) {
    Balance::init();
    
    PerformanceTest::measureLatency("Balance::processOnce", []() {
        Balance::processOnce();
    }, 5000, 1000); // 5ms max latency, 1000 iterations
}

// Performance test for event system
TEST(EventSystem, EventProcessingPerformance) {
    auto& event_bus = GlobalEventBus::getInstance();
    
    SUBSCRIBE_EVENT(SensorDataEvent, [](const SensorDataEvent& event) {
        // Minimal processing
    });
    
    PerformanceTest::measureThroughput("Event Processing", []() {
        PUBLISH_EVENT_SYNC(SensorDataEvent, 0.0f, 0.0f, 0.0f, 0, 0, 0, 0);
    }, std::chrono::milliseconds(100));
}

// Memory usage test
TEST(MemoryManager, MemoryUsageTest) {
    PerformanceTest::measureMemoryUsage("Memory Pool Usage", []() {
        std::vector<int*> ptrs;
        
        // Allocate from pool
        for (int i = 0; i < 100; ++i) {
            int* ptr = pool_allocate<int>();
            if (ptr) {
                *ptr = i;
                ptrs.push_back(ptr);
            }
        }
        
        // Deallocate
        for (int* ptr : ptrs) {
            pool_deallocate(ptr);
        }
    });
}

// Integration test combining all systems
TEST(Integration, RealTimeSystemIntegration) {
    // Initialize all systems
    ASSERT_TRUE(System::init());
    ASSERT_TRUE(Balance::init());
    
    auto& event_bus = GlobalEventBus::getInstance();
    auto& latency_monitor = SystemLatencyMonitors::getBalanceControlMonitor();
    
    // Set up event subscribers
    std::atomic<int> sensor_events{0};
    SUBSCRIBE_EVENT(SensorDataEvent, [&](const SensorDataEvent& event) {
        sensor_events.fetch_add(1);
    });
    
    // Run real-time simulation
    for (int i = 0; i < 100; ++i) {
        MEASURE_LATENCY(latency_monitor);
        
        Balance::processOnce();
        
        // Publish sensor event
        PUBLISH_EVENT(SensorDataEvent, i * 0.1f, i * 0.05f, 9.8f, 50 + i, 25, 12000, Timer::getMicroseconds());
        
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    
    // Process remaining events
    event_bus.processPendingEvents();
    
    // Verify system performance
    ASSERT_EQ(latency_monitor.getViolationCount(), 0);
    ASSERT_GT(sensor_events.load(), 90); // Should have processed most events
    ASSERT_FALSE(MemoryManager::getInstance().hasLeaks());
    
    // Print system performance report
    std::cout << SystemLatencyMonitors::generateSystemReport();
}