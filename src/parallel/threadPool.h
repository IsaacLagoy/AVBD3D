#ifndef THREADPOOL_H
#define THREADPOOL_H

#include <condition_variable>
#include <functional>
#include <iostream>
#include <mutex>
#include <queue>
#include <thread>

class ThreadPool {
    std::vector<std::thread> workers;
    std::queue<std::function<void()>> tasks;

    std::mutex queueMutex;
    std::condition_variable cv;
    std::condition_variable done_cv;

    bool stop = false;
    size_t activeTasks = 0;

    public:
    ThreadPool(size_t numThreads);
    ~ThreadPool();

    template <typename F, typename... Args>
    void enqueue(F&& f, Args&&... args);

    void wait();
};

#endif