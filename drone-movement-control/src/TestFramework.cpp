#include "TestFramework.hpp"
#include "MemoryManager.hpp"
#include "Delay.hpp"
#include <algorithm>
#include <numeric>
#include <iomanip>

TestRunner& TestRunner::getInstance() {
    static TestRunner instance;
    return instance;
}

void TestRunner::registerTest(const std::string& suite_name, 
                             const std::string& test_name,
                             TestFunction test_func,
                             const std::string& file,
                             int line) {
    registered_tests_[suite_name].push_back({test_name, test_func, file, line});
}

void TestRunner::runAllTests() {
    clear();
    
    if (global_setup_) {
        global_setup_();
    }
    
    for (const auto& suite_pair : registered_tests_) {
        runTestSuite(suite_pair.first);
    }
    
    if (global_teardown_) {
        global_teardown_();
    }
}

void TestRunner::runTestSuite(const std::string& suite_name) {
    auto it = registered_tests_.find(suite_name);
    if (it == registered_tests_.end()) {
        return;
    }
    
    TestSuite& suite = test_suites_[suite_name];
    suite.name = suite_name;
    
    // Suite setup
    auto setup_it = suite_setups_.find(suite_name);
    if (setup_it != suite_setups_.end()) {
        setup_it->second();
    }
    
    auto start_time = std::chrono::high_resolution_clock::now();
    
    for (const auto& test_info : it->second) {
        runSingleTest(suite_name, test_info);
    }
    
    auto end_time = std::chrono::high_resolution_clock::now();
    suite.total_duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
    
    // Suite teardown
    auto teardown_it = suite_teardowns_.find(suite_name);
    if (teardown_it != suite_teardowns_.end()) {
        teardown_it->second();
    }
}

void TestRunner::runTest(const std::string& suite_name, const std::string& test_name) {
    auto suite_it = registered_tests_.find(suite_name);
    if (suite_it == registered_tests_.end()) {
        return;
    }
    
    auto test_it = std::find_if(suite_it->second.begin(), suite_it->second.end(),
                               [&](const TestInfo& info) {
                                   return info.test_name == test_name;
                               });
    
    if (test_it != suite_it->second.end()) {
        runSingleTest(suite_name, *test_it);
    }
}

void TestRunner::runSingleTest(const std::string& suite_name, const TestInfo& test_info) {
    TestResult result;
    result.test_name = test_info.test_name;
    result.file = test_info.file;
    result.line = test_info.line;
    
    auto start_time = std::chrono::high_resolution_clock::now();
    
    try {
        test_info.test_func();
        result.passed = true;
        result.message = "PASSED";
    } catch (const AssertionException& e) {
        result.passed = false;
        result.message = e.what();
    } catch (const std::exception& e) {
        result.passed = false;
        result.message = std::string("Unexpected exception: ") + e.what();
    } catch (...) {
        result.passed = false;
        result.message = "Unknown exception caught";
    }
    
    auto end_time = std::chrono::high_resolution_clock::now();
    result.duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
    
    TestSuite& suite = test_suites_[suite_name];
    suite.results.push_back(result);
    
    if (result.passed) {
        suite.passed_count++;
    } else {
        suite.failed_count++;
    }
}

void TestRunner::setSuiteSetup(const std::string& suite_name, std::function<void()> setup) {
    suite_setups_[suite_name] = setup;
}

void TestRunner::setSuiteTeardown(const std::string& suite_name, std::function<void()> teardown) {
    suite_teardowns_[suite_name] = teardown;
}

void TestRunner::clear() {
    test_suites_.clear();
}

void TestRunner::printResults() const {
    std::cout << "\n=== TEST RESULTS ===\n";
    
    for (const auto& suite_pair : test_suites_) {
        const TestSuite& suite = suite_pair.second;
        
        std::cout << "\nSuite: " << suite.name << "\n";
        std::cout << "Duration: " << suite.total_duration.count() << " µs\n";
        std::cout << "Passed: " << suite.passed_count << ", Failed: " << suite.failed_count << "\n";
        
        for (const auto& result : suite.results) {
            std::cout << "  [" << (result.passed ? "PASS" : "FAIL") << "] " 
                     << result.test_name << " (" << result.duration.count() << " µs)";
            
            if (!result.passed) {
                std::cout << "\n    " << result.message;
            }
            std::cout << "\n";
        }
    }
}

void TestRunner::printSummary() const {
    int total_passed = 0;
    int total_failed = 0;
    std::chrono::microseconds total_duration{0};
    
    for (const auto& suite_pair : test_suites_) {
        const TestSuite& suite = suite_pair.second;
        total_passed += suite.passed_count;
        total_failed += suite.failed_count;
        total_duration += suite.total_duration;
    }
    
    std::cout << "\n=== TEST SUMMARY ===\n";
    std::cout << "Total Tests: " << (total_passed + total_failed) << "\n";
    std::cout << "Passed: " << total_passed << "\n";
    std::cout << "Failed: " << total_failed << "\n";
    std::cout << "Success Rate: " << std::fixed << std::setprecision(1) 
             << (100.0 * total_passed / (total_passed + total_failed)) << "%\n";
    std::cout << "Total Duration: " << total_duration.count() << " µs\n";
    
    if (total_failed > 0) {
        std::cout << "\nFAILED TESTS:\n";
        for (const auto& suite_pair : test_suites_) {
            const TestSuite& suite = suite_pair.second;
            for (const auto& result : suite.results) {
                if (!result.passed) {
                    std::cout << "  " << suite.name << "::" << result.test_name << "\n";
                    std::cout << "    " << result.message << "\n";
                }
            }
        }
    }
}

// Performance test implementations
void PerformanceTest::measureLatency(const std::string& test_name, 
                                    std::function<void()> func,
                                    uint64_t max_latency_us,
                                    size_t iterations) {
    std::vector<uint64_t> latencies;
    latencies.reserve(iterations);
    
    for (size_t i = 0; i < iterations; ++i) {
        auto start = std::chrono::high_resolution_clock::now();
        func();
        auto end = std::chrono::high_resolution_clock::now();
        
        auto latency = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
        latencies.push_back(latency);
    }
    
    // Calculate statistics
    std::sort(latencies.begin(), latencies.end());
    
    auto min_latency = latencies.front();
    auto max_latency = latencies.back();
    auto avg_latency = std::accumulate(latencies.begin(), latencies.end(), 0ULL) / iterations;
    auto p95_latency = latencies[iterations * 95 / 100];
    auto p99_latency = latencies[iterations * 99 / 100];
    
    auto violations = std::count_if(latencies.begin(), latencies.end(),
                                   [max_latency_us](uint64_t latency) {
                                       return latency > max_latency_us;
                                   });
    
    std::cout << "\n=== LATENCY TEST: " << test_name << " ===\n";
    std::cout << "Iterations: " << iterations << "\n";
    std::cout << "Min: " << min_latency << " µs\n";
    std::cout << "Max: " << max_latency << " µs\n";
    std::cout << "Avg: " << avg_latency << " µs\n";
    std::cout << "95th percentile: " << p95_latency << " µs\n";
    std::cout << "99th percentile: " << p99_latency << " µs\n";
    std::cout << "Violations (>" << max_latency_us << " µs): " << violations 
             << " (" << (100.0 * violations / iterations) << "%)\n";
    
    // Assert performance requirements
    ASSERT_LE(p95_latency, max_latency_us);
}

void PerformanceTest::measureThroughput(const std::string& test_name,
                                       std::function<void()> func,
                                       std::chrono::milliseconds duration) {
    auto start_time = std::chrono::steady_clock::now();
    auto end_time = start_time + duration;
    
    size_t operations = 0;
    while (std::chrono::steady_clock::now() < end_time) {
        func();
        operations++;
    }
    
    auto actual_duration = std::chrono::steady_clock::now() - start_time;
    auto actual_ms = std::chrono::duration_cast<std::chrono::milliseconds>(actual_duration).count();
    
    double ops_per_sec = (1000.0 * operations) / actual_ms;
    
    std::cout << "\n=== THROUGHPUT TEST: " << test_name << " ===\n";
    std::cout << "Duration: " << actual_ms << " ms\n";
    std::cout << "Operations: " << operations << "\n";
    std::cout << "Throughput: " << std::fixed << std::setprecision(2) << ops_per_sec << " ops/sec\n";
}

void PerformanceTest::measureMemoryUsage(const std::string& test_name,
                                        std::function<void()> func) {
    auto& memory_manager = MemoryManager::getInstance();
    
    auto before_allocated = memory_manager.getTotalAllocated();
    auto before_peak = memory_manager.getPeakAllocated();
    
    func();
    
    auto after_allocated = memory_manager.getTotalAllocated();
    auto after_peak = memory_manager.getPeakAllocated();
    
    auto allocated_diff = after_allocated - before_allocated;
    auto peak_diff = after_peak - before_peak;
    
    std::cout << "\n=== MEMORY TEST: " << test_name << " ===\n";
    std::cout << "Memory allocated during test: " << allocated_diff << " bytes\n";
    std::cout << "Peak memory increase: " << peak_diff << " bytes\n";
    std::cout << "Memory leaks: " << (memory_manager.hasLeaks() ? "YES" : "NO") << "\n";
    
    // Assert no memory leaks
    ASSERT_FALSE(memory_manager.hasLeaks());
}