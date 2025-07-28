#include "../inc/TestFramework.hpp"
#include "../inc/System.hpp"

int main() {
    std::cout << "Starting ENG5220 Real-time Balance System Tests...\n\n";
    
    auto& test_runner = TestRunner::getInstance();
    
    // Set up global test setup/teardown
    test_runner.setGlobalSetup([]() {
        std::cout << "Global test setup: Initializing system...\n";
    });
    
    test_runner.setGlobalTeardown([]() {
        std::cout << "Global test teardown: Cleaning up system...\n";
        System::shutdown();
    });
    
    // Set up suite-specific setup/teardown
    test_runner.setSuiteSetup("BalanceSystem", []() {
        std::cout << "Balance system test setup\n";
    });
    
    test_runner.setSuiteTeardown("BalanceSystem", []() {
        std::cout << "Balance system test teardown\n";
    });
    
    // Run all tests
    test_runner.runAllTests();
    
    // Print detailed results
    test_runner.printResults();
    test_runner.printSummary();
    
    // Determine exit code based on test results
    bool all_passed = true;
    for (const auto& suite_pair : test_runner.getResults()) {
        if (suite_pair.second.failed_count > 0) {
            all_passed = false;
            break;
        }
    }
    
    if (all_passed) {
        std::cout << "\n🎉 All tests passed! System meets ENG5220 requirements.\n";
        return 0;
    } else {
        std::cout << "\n❌ Some tests failed. System needs improvement.\n";
        return 1;
    }
}