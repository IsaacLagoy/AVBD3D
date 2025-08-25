#include "threadPool.h"

ThreadPool::ThreadPool(size_t numThreads) {
    for (size_t i = 0; i < numThreads; i++) {
        workers.emplace_back([this] {
            while (true) {
                std::function<void()> task;

                {   // Critical section for queue
                    std::unique_lock<std::mutex> lock(queueMutex);
                    cv.wait(lock, [this] {
                        return stop || !tasks.empty();
                    });

                    if (stop && tasks.empty())
                        return;

                    task = std::move(tasks.front());
                    tasks.pop();
                    ++activeTasks;  // mark new active task
                }

                task();

                {   // decrement active task count
                    std::unique_lock<std::mutex> lock(queueMutex);
                    --activeTasks;
                    if (tasks.empty() && activeTasks == 0) {
                        done_cv.notify_all();
                    }
                }
            }
        });
    }
}

ThreadPool::~ThreadPool() {
    {
        std::unique_lock<std::mutex> lock(queueMutex);
        stop = true;
    }
    cv.notify_all();
    for (auto& t : workers)
        t.join();
}

template <typename F, typename... Args>
void ThreadPool::enqueue(F&& f, Args&&... args) {
    {
        std::unique_lock<std::mutex> lock(queueMutex);
        tasks.emplace([fn = std::forward<F>(f), ... args = std::forward<Args>(args)]() {
            fn(args...);
        });
    }
    cv.notify_one();
}

void ThreadPool::wait() {
    std::unique_lock<std::mutex> lock(queueMutex);
    done_cv.wait(lock, [this] {
        return tasks.empty() && activeTasks == 0;
    });
}

