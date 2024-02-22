#pragma once

#include <cstdint>

enum TaskType {
    TEST_TASK1,
    TEST_TASK2,
    TEST_TASK3,
    STOP,
};

struct function3_args {
    const char *text;
};

void doLotsOfWork ();

void function3(uint8_t thread_id, const char *string);

