#include "taskmaster.h"
#include "tm_shared.h"

void function1(uint8_t thread_id) {
    //DEBUG("Function 1 executed on thread %d\n", thread_id);
}

void function2(uint8_t thread_id) {
   //DEBUG("Function 2 executed on thread %d\n", thread_id);
}

void runner(TaskMaster *task_master, uint8_t thread_id) {
    while(1) {

        //Wait for task
        Task current_task = task_master->getTask();

        // Test case switch and function pointers
        switch (current_task.task_type) {
            case TaskType::TEST_TASK1:
                function1(thread_id);
                break;
            
            case TaskType::TEST_TASK2:
                function2(thread_id);
                break;

            case TaskType::TEST_TASK3:
                {
                    auto s = ((function3_args *) current_task.argument_struct);
                    function3(thread_id, s->text);
                }
                break;
            
            case TaskType::STOP:
                return;
        }

        if (current_task.sync != NULL) {
            current_task.sync->tc_lock.lock();
                if (current_task.on_complete && current_task.sync->task_counter == 1) {
                    current_task.sync->task_counter -= 1;
                    ((void (*)(TaskMaster *, Task *)) current_task.on_complete)(task_master, &current_task);
                } else {
                    current_task.sync->task_counter -= 1;
                }
            current_task.sync->tc_lock.unlock();
        }
    }
}

