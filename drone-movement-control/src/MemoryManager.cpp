#include "MemoryManager.hpp"

// Static member definitions
std::unique_ptr<MemoryManager> MemoryManager::instance_ = nullptr;
std::mutex MemoryManager::instance_mutex_;

// Global memory pools
MemoryPool<int, 1024> g_int_pool;
MemoryPool<float, 1024> g_float_pool;
MemoryPool<double, 512> g_double_pool;