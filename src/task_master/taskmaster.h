#pragma once

#include <mutex>
#include <thread>

#include "ringbuffer.h"
#include "tm_shared.h"

#define NUM_THREADS 4

typedef volatile int spinlock_t;

struct Lock {
    spinlock_t lock_internal;

    void lock() {
        //Should probably try and avoid using these legacy built-ins
        while (__sync_lock_test_and_set(&lock_internal, 1)) {}
    }

    void unlock() {
        __sync_synchronize();
        lock_internal = 0;
    }
};

//typedef std::mutex Lock;

struct TaskMaster;

struct TaskSync {
    Lock tc_lock;
    volatile int task_counter;

    void wait() {
        while (1) {
            tc_lock.lock();
            if (task_counter == 0) break;
            tc_lock.unlock();
            std::this_thread::yield();
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

    RingBuffer<Task, 2048> ts_task_buffer;
    std::thread threads[NUM_THREADS];

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
                std::this_thread::yield();
                goto try_lock;
            }
        this->lock.unlock();

        //Return the actual task, because after this point the allocation in the array may be deleted (and they're very small memory wise)
        return task;
    };

    void start_threads() {
        for (int i = 0; i < NUM_THREADS; i++) {
            threads[i] = std::thread(runner, this, i);
        }
    }

    void queue_stop_all() {
        lock.lock();
        for (int i = 0; i < NUM_THREADS; i++) {
            ts_task_buffer.push_back(
                Task { .task_type = TaskType::STOP }
            );
        }
        lock.unlock();
    }

    void join_all() {
        for (int i = 0; i < NUM_THREADS; i++) {
            this->threads[i].join();
        }
    }
};
