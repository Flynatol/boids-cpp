#pragma once

#include <cstdint>
#include "..\boidlist.h"
#include "..\boidmap.h"
#include "..\ui.h"

enum TaskType {
    TEST_TASK1,
    TEST_TASK3,
    ROW_RUNNER_PASS_ONE,
    ROW_RUNNER_PASS_TWO,
    STOP,
};

struct function3_args {
    const char *text;
};

struct row_runner_args {
    BoidMap *boid_map;
    uint32_t y;
    Rules *rules; 
    row_runner_args* arg_store;
    BoidList* boid_list;
};

void doLotsOfWork ();

void function3(uint8_t thread_id, const char *string);

