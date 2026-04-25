#pragma once

#include <mutex>
#include <thread>
#include <algorithm>
#include <cstdint>

#include "ringbuffer.h"
#include "lock.h"

template <typename Spec>
struct TaskMaster;

// Spawn one less worker thread than number of threads the system has
const uint32_t num_threads = std::min(std::thread::hardware_concurrency() - 1, (uint32_t) 64);

BOOLEAN nanosleep(LONGLONG ns);

struct TaskSync {
    Lock tc_lock;

    volatile uint32_t task_counter = 0;
    
    void wait() {
        while (1) {
            tc_lock.lock();
            if (task_counter == 0) break;
            tc_lock.unlock();
        }
        tc_lock.unlock();
    };
};

template <typename Spec>
struct TaskMaster {
    using Task = typename Spec::Task;

    uint32_t front = 0;
    uint32_t back = 0;
    
    // Enable thread sleeping (tiny performance decrease but means performance usage scales down when there are few boids)
    bool sleep_enabled = true;

    RingBufferMPMC<Task, 4096> ts_task_buffer;
    std::thread threads[64];

    Lock lock;

    Task get_task() {
        Task task;

        //This should hang until a task is available
        try_lock:

        //Aquire front lock
        this->lock.lock();
            if (!(this->ts_task_buffer.pop_front(task))) {
                this->lock.unlock();
                if (sleep_enabled) nanosleep(10000);
                goto try_lock;
            }
        this->lock.unlock();

        //Return the actual task, because after this point the allocation in the array may be deleted (and they're only 32 bytes)
        return task;
    };

    void start_threads() {
        for (int i = 0; i < num_threads; i++) {
            threads[i] = std::thread(&TaskMaster::runner, this, i);
        }
    }

    void queue_stop_all() {
        lock.lock();
        for (int i = 0; i < num_threads; i++) {
            ts_task_buffer.push_back(
                Spec::stop_task()
            );
        }
        lock.unlock();
    }

    void join_all() {
        for (int i = 0; i < num_threads; i++) {
            this->threads[i].join();
        }
    }

private:
    static void runner(TaskMaster *task_master, uint8_t thread_id) {
        while (1) {
            Task current_task = task_master->get_task();

            if (Spec::execute(task_master, current_task)) {
                return;
            }

            task_master->complete(current_task);
        }
    }

    void complete(Task &current_task) {
        if (current_task.sync != NULL) {
            current_task.sync->tc_lock.lock();
                if (current_task.on_complete != NULL && current_task.sync->task_counter == 1) {
                    current_task.sync->task_counter -= 1;
                    current_task.on_complete(this, &current_task);
                } else {
                    current_task.sync->task_counter -= 1;
                }
            current_task.sync->tc_lock.unlock();
        }
    }
};
