#pragma once

#include <mutex>
#include <thread>

#include "ringbuffer.h"
#include "tm_shared.h"

typedef std::mutex Lock;

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
    void *argument_struct;

    TaskSync *sync;

    void *on_complete;
};

struct TaskMaster {
    uint32_t front;
    uint32_t back;

    RingBuffer<Task, 12024> ts_task_buffer;

    Lock lock;

    Task getTask() {
        Task task;

        //This should hang until a task is available
        try_lock:

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
};

void runner(TaskMaster *task_master, uint8_t thread_id);

