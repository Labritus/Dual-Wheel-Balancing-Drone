#include "RealTimeThread.hpp"
#include <iostream>
#include <cstring>
#include <unistd.h>

RealTimeThread::RealTimeThread(std::function<void()> task, int priority, int period_ms)
    : task(std::move(task)), priority(priority), period_ms(period_ms), running(false), paused(false) {}

RealTimeThread::~RealTimeThread() {
    stop();
}

void* RealTimeThread::threadFunc(void* arg) {
    RealTimeThread* thread = static_cast<RealTimeThread*>(arg);
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);

    while (thread->running) {
        if (!thread->paused) {
            thread->task();
        }

        // Periodic scheduling
        ts.tv_nsec += thread->period_ms * 1000000;
        while (ts.tv_nsec >= 1000000000) {
            ts.tv_sec++;
            ts.tv_nsec -= 1000000000;
        }
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ts, nullptr);
    }
    return nullptr;
}

void RealTimeThread::start() {
    if (running) return;
    running = true;
    paused = false;

    pthread_attr_t attr;
    struct sched_param param;
    pthread_attr_init(&attr);

    // Set thread scheduling policy to real-time FIFO
    pthread_attr_setschedpolicy(&attr, SCHED_FIFO);
    param.sched_priority = priority;
    pthread_attr_setschedparam(&attr, &param);
    pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED);

    if (pthread_create(&thread, &attr, threadFunc, this) != 0) {
        std::cerr << "Failed to create real-time thread: " << strerror(errno) << std::endl;
        running = false;
    }
    pthread_attr_destroy(&attr);
}

void RealTimeThread::stop() {
    if (!running) return;
    running = false;
    pthread_join(thread, nullptr);
}

void RealTimeThread::pause() {
    paused = true;
}

void RealTimeThread::resume() {
    paused = false;
}