#ifndef REALTIMETHREAD_HPP
#define REALTIMETHREAD_HPP

#include <pthread.h>
#include <atomic>
#include <functional>

class RealTimeThread {
private:
    pthread_t thread;
    std::atomic<bool> running;
    std::atomic<bool> paused;
    std::function<void()> task;
    int priority;
    int period_ms;
    static void* threadFunc(void* arg);

public:
    RealTimeThread(std::function<void()> task, int priority = 90, int period_ms = 10);
    ~RealTimeThread();
    void start();
    void stop();
    void pause();
    void resume();
    bool isRunning() const { return running; }
    bool isPaused() const { return paused; }
};

#endif // REALTIMETHREAD_HPP