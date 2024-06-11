#pragma once

#include <mutex>
#include <thread>
#include <algorithm>

#include "ringbuffer.h"
#include "tm_shared.h"
#include "lock.h"

struct TaskMaster;

#define NUM_THREADS num_threads

// Num threads doesn't include the main thread -- so we're spawning the hardware thread number -1 for main and -1 for OS stuff.
const uint32_t num_threads = std::min(std::thread::hardware_concurrency() - 2, (uint32_t) 64);

BOOLEAN nanosleep(LONGLONG ns);

struct TaskSync {
    Lock tc_lock;

    volatile int task_counter = 0;
    
    void wait() {
        while (1) {
            tc_lock.lock();
            //TraceLog(LOG_DEBUG, TextFormat("TC: %d", task_counter));
            if (task_counter == 0) break;
            tc_lock.unlock();
            //std::this_thread::yield();
        }
        tc_lock.unlock();
    };
};

struct Task {
    TaskType task_type;
    void *argument_struct = NULL;

    TaskSync *sync = NULL;

    void *on_complete = NULL;
};

void runner(TaskMaster *task_master, uint8_t thread_id);

struct TaskMaster {
    uint32_t front = 0;
    uint32_t back = 0;

    //uint32_t num_threads = std::min(std::thread::hardware_concurrency() - 1, (uint32_t) 64);
    
    // To be used by dynamic performance scaler
    bool sleep_enabled = true;

    RingBuffer<Task, 4096> ts_task_buffer;
    std::thread threads[64];

    Lock lock;

    Task get_task() {
        Task task;

        //This should hang until a task is available
        try_lock:

        //while (!ts_task_buffer.unsafe_not_empty()) {}

        //Aquire front lock
        this->lock.lock();
            if (!(this->ts_task_buffer.pop_front(task))) {
                this->lock.unlock();
                //std::this_thread::yield();
                if (sleep_enabled) nanosleep(10000);
                goto try_lock;
            }
        this->lock.unlock();

        //Return the actual task, because after this point the allocation in the array may be deleted (and they're very small memory wise)
        return task;
    };

    void start_threads() {
        for (int i = 0; i < num_threads; i++) {
            threads[i] = std::thread(runner, this, i);
        }
    }

    void queue_stop_all() {
        lock.lock();
        for (int i = 0; i < num_threads; i++) {
            ts_task_buffer.push_back(
                Task { .task_type = TaskType::STOP }
            );
        }
        lock.unlock();
    }

    void join_all() {
        for (int i = 0; i < num_threads; i++) {
            this->threads[i].join();
        }
    }
};
