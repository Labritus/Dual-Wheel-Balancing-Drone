#ifndef __TEST_FRAMEWORK_HPP
#define __TEST_FRAMEWORK_HPP

#include <functional>
#include <vector>
#include <string>
#include <memory>
#include <chrono>
#include <iostream>
#include <sstream>
#include <exception>
#include <unordered_map>

// Test result structure
struct TestResult {
    std::string test_name;
    bool passed;
    std::string message;
    std::chrono::microseconds duration;
    std::string file;
    int line;
};

// Test suite structure
struct TestSuite {
    std::string name;
    std::vector<TestResult> results;
    std::chrono::microseconds total_duration{0};
    int passed_count = 0;
    int failed_count = 0;
};

// Assertion exception
class AssertionException : public std::exception {
public:
    explicit AssertionException(const std::string& message) : message_(message) {}
    const char* what() const noexcept override { return message_.c_str(); }
    
private:
    std::string message_;
};

// Test runner class
class TestRunner {
public:
    using TestFunction = std::function<void()>;
    
    static TestRunner& getInstance();
    
    // Register a test
    void registerTest(const std::string& suite_name, 
                     const std::string& test_name,
                     TestFunction test_func,
                     const std::string& file = "",
                     int line = 0);
    
    // Run all tests
    void runAllTests();
    
    // Run specific test suite
    void runTestSuite(const std::string& suite_name);
    
    // Run specific test
    void runTest(const std::string& suite_name, const std::string& test_name);
    
    // Get results
    const std::unordered_map<std::string, TestSuite>& getResults() const { return test_suites_; }
    
    // Print results
    void printResults() const;
    void printSummary() const;
    
    // Setup and teardown hooks
    void setGlobalSetup(std::function<void()> setup) { global_setup_ = setup; }
    void setGlobalTeardown(std::function<void()> teardown) { global_teardown_ = teardown; }
    void setSuiteSetup(const std::string& suite_name, std::function<void()> setup);
    void setSuiteTeardown(const std::string& suite_name, std::function<void()> teardown);
    
    // Clear all results
    void clear();
    
private:
    struct TestInfo {
        std::string test_name;
        TestFunction test_func;
        std::string file;
        int line;
    };
    
    std::unordered_map<std::string, std::vector<TestInfo>> registered_tests_;
    std::unordered_map<std::string, TestSuite> test_suites_;
    
    std::function<void()> global_setup_;
    std::function<void()> global_teardown_;
    std::unordered_map<std::string, std::function<void()>> suite_setups_;
    std::unordered_map<std::string, std::function<void()>> suite_teardowns_;
    
    TestRunner() = default;
    
    void runSingleTest(const std::string& suite_name, const TestInfo& test_info);
};

// Test registration helper
class TestRegistrar {
public:
    TestRegistrar(const std::string& suite_name,
                 const std::string& test_name,
                 TestRunner::TestFunction test_func,
                 const std::string& file,
                 int line) {
        TestRunner::getInstance().registerTest(suite_name, test_name, test_func, file, line);
    }
};

// Assertion macros
#define ASSERT_TRUE(condition) \
    do { \
        if (!(condition)) { \
            std::ostringstream oss; \
            oss << "Assertion failed: " << #condition << " at " << __FILE__ << ":" << __LINE__; \
            throw AssertionException(oss.str()); \
        } \
    } while(0)

#define ASSERT_FALSE(condition) \
    ASSERT_TRUE(!(condition))

#define ASSERT_EQ(expected, actual) \
    do { \
        if ((expected) != (actual)) { \
            std::ostringstream oss; \
            oss << "Assertion failed: expected " << (expected) << " but got " << (actual) \
                << " at " << __FILE__ << ":" << __LINE__; \
            throw AssertionException(oss.str()); \
        } \
    } while(0)

#define ASSERT_NE(expected, actual) \
    do { \
        if ((expected) == (actual)) { \
            std::ostringstream oss; \
            oss << "Assertion failed: expected " << (expected) << " to not equal " << (actual) \
                << " at " << __FILE__ << ":" << __LINE__; \
            throw AssertionException(oss.str()); \
        } \
    } while(0)

#define ASSERT_LT(left, right) \
    do { \
        if (!((left) < (right))) { \
            std::ostringstream oss; \
            oss << "Assertion failed: expected " << (left) << " < " << (right) \
                << " at " << __FILE__ << ":" << __LINE__; \
            throw AssertionException(oss.str()); \
        } \
    } while(0)

#define ASSERT_LE(left, right) \
    do { \
        if (!((left) <= (right))) { \
            std::ostringstream oss; \
            oss << "Assertion failed: expected " << (left) << " <= " << (right) \
                << " at " << __FILE__ << ":" << __LINE__; \
            throw AssertionException(oss.str()); \
        } \
    } while(0)

#define ASSERT_GT(left, right) \
    do { \
        if (!((left) > (right))) { \
            std::ostringstream oss; \
            oss << "Assertion failed: expected " << (left) << " > " << (right) \
                << " at " << __FILE__ << ":" << __LINE__; \
            throw AssertionException(oss.str()); \
        } \
    } while(0)

#define ASSERT_GE(left, right) \
    do { \
        if (!((left) >= (right))) { \
            std::ostringstream oss; \
            oss << "Assertion failed: expected " << (left) << " >= " << (right) \
                << " at " << __FILE__ << ":" << __LINE__; \
            throw AssertionException(oss.str()); \
        } \
    } while(0)

#define ASSERT_FLOAT_EQ(expected, actual, tolerance) \
    do { \
        auto diff = std::abs((expected) - (actual)); \
        if (diff > (tolerance)) { \
            std::ostringstream oss; \
            oss << "Assertion failed: expected " << (expected) << " but got " << (actual) \
                << " (difference " << diff << " > tolerance " << (tolerance) << ")" \
                << " at " << __FILE__ << ":" << __LINE__; \
            throw AssertionException(oss.str()); \
        } \
    } while(0)

#define ASSERT_THROW(statement, expected_exception) \
    do { \
        bool caught_expected = false; \
        try { \
            statement; \
        } catch (const expected_exception&) { \
            caught_expected = true; \
        } catch (...) { \
            std::ostringstream oss; \
            oss << "Assertion failed: expected " << #expected_exception \
                << " but caught different exception at " << __FILE__ << ":" << __LINE__; \
            throw AssertionException(oss.str()); \
        } \
        if (!caught_expected) { \
            std::ostringstream oss; \
            oss << "Assertion failed: expected " << #expected_exception \
                << " but no exception was thrown at " << __FILE__ << ":" << __LINE__; \
            throw AssertionException(oss.str()); \
        } \
    } while(0)

#define ASSERT_NO_THROW(statement) \
    do { \
        try { \
            statement; \
        } catch (...) { \
            std::ostringstream oss; \
            oss << "Assertion failed: expected no exception but one was thrown" \
                << " at " << __FILE__ << ":" << __LINE__; \
            throw AssertionException(oss.str()); \
        } \
    } while(0)

// Test registration macro
#define TEST(suite_name, test_name) \
    void test_##suite_name##_##test_name(); \
    static TestRegistrar registrar_##suite_name##_##test_name( \
        #suite_name, #test_name, test_##suite_name##_##test_name, __FILE__, __LINE__); \
    void test_##suite_name##_##test_name()

// Mock framework for testing
template<typename T>
class Mock {
public:
    Mock() = default;
    virtual ~Mock() = default;
    
    // Method call tracking
    struct MethodCall {
        std::string method_name;
        std::vector<std::string> parameters;
        std::chrono::steady_clock::time_point timestamp;
    };
    
    // Record method call
    void recordCall(const std::string& method_name, 
                   const std::vector<std::string>& params = {}) {
        method_calls_.push_back({method_name, params, std::chrono::steady_clock::now()});
    }
    
    // Verify method was called
    bool wasMethodCalled(const std::string& method_name) const {
        return std::find_if(method_calls_.begin(), method_calls_.end(),
                           [&](const MethodCall& call) {
                               return call.method_name == method_name;
                           }) != method_calls_.end();
    }
    
    // Get call count
    size_t getCallCount(const std::string& method_name) const {
        return std::count_if(method_calls_.begin(), method_calls_.end(),
                            [&](const MethodCall& call) {
                                return call.method_name == method_name;
                            });
    }
    
    // Clear call history
    void clearCalls() { method_calls_.clear(); }
    
    // Get all calls
    const std::vector<MethodCall>& getCalls() const { return method_calls_; }
    
protected:
    std::vector<MethodCall> method_calls_;
};

// Real-time performance test helpers
class PerformanceTest {
public:
    static void measureLatency(const std::string& test_name, 
                              std::function<void()> func,
                              uint64_t max_latency_us = 5000,
                              size_t iterations = 1000);
    
    static void measureThroughput(const std::string& test_name,
                                 std::function<void()> func,
                                 std::chrono::milliseconds duration = std::chrono::milliseconds(1000));
    
    static void measureMemoryUsage(const std::string& test_name,
                                  std::function<void()> func);
};

#endif // __TEST_FRAMEWORK_HPP