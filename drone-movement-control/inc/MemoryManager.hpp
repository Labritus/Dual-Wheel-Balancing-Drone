#ifndef MEMORYMANAGER_HPP
#define MEMORYMANAGER_HPP

#include <cstdlib>
#include <unordered_map>
#include <string>
#include <iostream>
#include <memory>
#include <mutex>
#include <atomic>
#include <vector>
#include <functional>
#include <type_traits>

// RAII Memory Pool for real-time systems
template<typename T, size_t PoolSize = 1024>
class MemoryPool {
public:
    MemoryPool() : next_free_(0) {
        // Pre-allocate memory pool
        for (size_t i = 0; i < PoolSize - 1; ++i) {
            reinterpret_cast<size_t*>(&pool_[i])[0] = i + 1;
        }
        reinterpret_cast<size_t*>(&pool_[PoolSize - 1])[0] = SIZE_MAX;
    }
    
    T* allocate() {
        std::lock_guard<std::mutex> lock(mutex_);
        if (next_free_ == SIZE_MAX) {
            return nullptr; // Pool exhausted
        }
        
        size_t index = next_free_;
        next_free_ = reinterpret_cast<size_t*>(&pool_[index])[0];
        allocated_count_.fetch_add(1);
        
        return reinterpret_cast<T*>(&pool_[index]);
    }
    
    void deallocate(T* ptr) {
        if (!ptr) return;
        
        std::lock_guard<std::mutex> lock(mutex_);
        size_t index = ptr - reinterpret_cast<T*>(pool_);
        if (index >= PoolSize) {
            return; // Invalid pointer
        }
        
        reinterpret_cast<size_t*>(&pool_[index])[0] = next_free_;
        next_free_ = index;
        allocated_count_.fetch_sub(1);
    }
    
    size_t getAllocatedCount() const { return allocated_count_.load(); }
    size_t getPoolSize() const { return PoolSize; }
    
private:
    alignas(T) char pool_[PoolSize * sizeof(T)];
    size_t next_free_;
    std::mutex mutex_;
    std::atomic<size_t> allocated_count_{0};
};

// RAII wrapper for automatic memory management
template<typename T>
class UniqueResource {
public:
    using Deleter = std::function<void(T*)>;
    
    UniqueResource() : ptr_(nullptr), deleter_(nullptr) {}
    
    UniqueResource(T* ptr, Deleter deleter) 
        : ptr_(ptr), deleter_(deleter) {}
    
    ~UniqueResource() {
        reset();
    }
    
    // Move semantics
    UniqueResource(UniqueResource&& other) noexcept 
        : ptr_(other.ptr_), deleter_(std::move(other.deleter_)) {
        other.ptr_ = nullptr;
        other.deleter_ = nullptr;
    }
    
    UniqueResource& operator=(UniqueResource&& other) noexcept {
        if (this != &other) {
            reset();
            ptr_ = other.ptr_;
            deleter_ = std::move(other.deleter_);
            other.ptr_ = nullptr;
            other.deleter_ = nullptr;
        }
        return *this;
    }
    
    // No copy semantics
    UniqueResource(const UniqueResource&) = delete;
    UniqueResource& operator=(const UniqueResource&) = delete;
    
    T* get() const { return ptr_; }
    T* operator->() const { return ptr_; }
    T& operator*() const { return *ptr_; }
    
    explicit operator bool() const { return ptr_ != nullptr; }
    
    T* release() {
        T* temp = ptr_;
        ptr_ = nullptr;
        deleter_ = nullptr;
        return temp;
    }
    
    void reset(T* ptr = nullptr, Deleter deleter = nullptr) {
        if (ptr_ && deleter_) {
            deleter_(ptr_);
        }
        ptr_ = ptr;
        deleter_ = deleter;
    }
    
private:
    T* ptr_;
    Deleter deleter_;
};

// Enhanced Memory Manager with RAII patterns
class MemoryManager {
private:
    static std::unique_ptr<MemoryManager> instance_;
    static std::mutex instance_mutex_;
    
    mutable std::mutex allocations_mutex_;
    std::unordered_map<void*, std::string> allocations_;
    std::atomic<bool> tracking_enabled_{true};
    std::atomic<size_t> total_allocated_{0};
    std::atomic<size_t> peak_allocated_{0};
    std::vector<std::function<void()>> cleanup_handlers_;

    MemoryManager() = default;

public:
    static MemoryManager& getInstance() {
        std::lock_guard<std::mutex> lock(instance_mutex_);
        if (!instance_) {
            instance_ = std::unique_ptr<MemoryManager>(new MemoryManager());
        }
        return *instance_;
    }
    
    ~MemoryManager() {
        // Execute cleanup handlers
        for (auto& handler : cleanup_handlers_) {
            try {
                handler();
            } catch (...) {
                // Ignore exceptions during cleanup
            }
        }
        
        // Report memory leaks
        std::lock_guard<std::mutex> lock(allocations_mutex_);
        if (tracking_enabled_.load() && !allocations_.empty()) {
            std::cerr << "Memory leak detected! Unfreed allocations:" << std::endl;
            for (const auto& alloc : allocations_) {
                std::cerr << "  Address: " << alloc.first << ", File: " << alloc.second << std::endl;
            }
        }
    }

    void* allocate(size_t size, const char* file, int line) {
        void* ptr = std::malloc(size);
        if (ptr && tracking_enabled_.load()) {
            std::lock_guard<std::mutex> lock(allocations_mutex_);
            allocations_[ptr] = std::string(file) + ":" + std::to_string(line);
            total_allocated_.fetch_add(size);
            
            size_t current_total = total_allocated_.load();
            size_t current_peak = peak_allocated_.load();
            while (current_total > current_peak && 
                   !peak_allocated_.compare_exchange_weak(current_peak, current_total)) {
                // Retry if CAS failed
            }
        }
        return ptr;
    }

    void deallocate(void* ptr, size_t size = 0) {
        if (ptr && tracking_enabled_.load()) {
            std::lock_guard<std::mutex> lock(allocations_mutex_);
            auto it = allocations_.find(ptr);
            if (it != allocations_.end()) {
                allocations_.erase(it);
                if (size > 0) {
                    total_allocated_.fetch_sub(size);
                }
            } else {
                std::cerr << "Double free or invalid pointer: " << ptr << std::endl;
            }
        }
        std::free(ptr);
    }

    void enableTracking(bool enable) {
        tracking_enabled_.store(enable);
    }

    bool hasLeaks() const {
        std::lock_guard<std::mutex> lock(allocations_mutex_);
        return !allocations_.empty();
    }
    
    size_t getLeakCount() const {
        std::lock_guard<std::mutex> lock(allocations_mutex_);
        return allocations_.size();
    }
    
    size_t getTotalAllocated() const {
        return total_allocated_.load();
    }
    
    size_t getPeakAllocated() const {
        return peak_allocated_.load();
    }
    
    // Register cleanup handler for RAII
    void addCleanupHandler(std::function<void()> handler) {
        cleanup_handlers_.push_back(handler);
    }
    
    // Create RAII resource wrapper
    template<typename T>
    UniqueResource<T> makeUniqueResource(T* ptr) {
        return UniqueResource<T>(ptr, [this](T* p) {
            this->deallocate(p, sizeof(T));
        });
    }
    
    // Create RAII resource with custom deleter
    template<typename T, typename Deleter>
    UniqueResource<T> makeUniqueResource(T* ptr, Deleter deleter) {
        return UniqueResource<T>(ptr, deleter);
    }

    // Prevent copying
    MemoryManager(const MemoryManager&) = delete;
    MemoryManager& operator=(const MemoryManager&) = delete;
};

// Memory allocation macros with enhanced tracking
#define MEM_ALLOC(size) MemoryManager::getInstance().allocate(size, __FILE__, __LINE__)
#define MEM_FREE(ptr) MemoryManager::getInstance().deallocate(ptr)
#define MEM_FREE_SIZED(ptr, size) MemoryManager::getInstance().deallocate(ptr, size)

// RAII helper macros
#define MAKE_UNIQUE_RESOURCE(type, ptr) MemoryManager::getInstance().makeUniqueResource<type>(ptr)
#define SCOPED_RESOURCE(var, type, ptr) auto var = MAKE_UNIQUE_RESOURCE(type, ptr)

// Stack-based RAII guard for cleanup
class ScopedCleanup {
public:
    explicit ScopedCleanup(std::function<void()> cleanup) : cleanup_(cleanup) {}
    ~ScopedCleanup() { if (cleanup_) cleanup_(); }
    
    // Move semantics
    ScopedCleanup(ScopedCleanup&& other) noexcept : cleanup_(std::move(other.cleanup_)) {
        other.cleanup_ = nullptr;
    }
    
    // No copy semantics
    ScopedCleanup(const ScopedCleanup&) = delete;
    ScopedCleanup& operator=(const ScopedCleanup&) = delete;
    ScopedCleanup& operator=(ScopedCleanup&&) = delete;
    
private:
    std::function<void()> cleanup_;
};

#define SCOPED_CLEANUP(cleanup_code) ScopedCleanup _cleanup_guard([&]() { cleanup_code; })

// Global memory pools for common types
extern MemoryPool<int, 1024> g_int_pool;
extern MemoryPool<float, 1024> g_float_pool;
extern MemoryPool<double, 512> g_double_pool;

// Pool allocation helpers
template<typename T>
T* pool_allocate() {
    if constexpr (std::is_same_v<T, int>) {
        return g_int_pool.allocate();
    } else if constexpr (std::is_same_v<T, float>) {
        return g_float_pool.allocate();
    } else if constexpr (std::is_same_v<T, double>) {
        return g_double_pool.allocate();
    } else {
        return static_cast<T*>(MEM_ALLOC(sizeof(T)));
    }
}

template<typename T>
void pool_deallocate(T* ptr) {
    if (!ptr) return;
    
    if constexpr (std::is_same_v<T, int>) {
        g_int_pool.deallocate(ptr);
    } else if constexpr (std::is_same_v<T, float>) {
        g_float_pool.deallocate(ptr);
    } else if constexpr (std::is_same_v<T, double>) {
        g_double_pool.deallocate(ptr);
    } else {
        MEM_FREE_SIZED(ptr, sizeof(T));
    }
}

#endif // MEMORYMANAGER_HPP