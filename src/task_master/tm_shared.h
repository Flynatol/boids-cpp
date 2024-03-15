#pragma once

#include <cstdint>
#include "..\boidlist.h"
#include "..\boidmap.h"
#include "..\ui.h"

enum TaskType {
    POPULATE,
    REBUILD,
    ROW_RUNNER_PASS_ONE,
    ROW_RUNNER_PASS_TWO,
    STOP,
};

struct function3_args {
    const char *text;
};

struct rebuild_args {
    BoidMap *boid_map;
    uint32_t y;
    Boid *index_buffer;
    BoidList* boid_list;
};

struct populate_args {
    BoidMap *boid_map;
    uint32_t start;
    uint32_t task_size;
    BoidList* boid_list;
    rebuild_args* rebuild_args;
};

struct row_runner_args {
    BoidMap *boid_map;
    uint32_t y;
    Rules *rules; 
    row_runner_args* arg_store;
    BoidList* boid_list;
    populate_args* pop_args;
};

void doLotsOfWork ();

void function3(uint8_t thread_id, const char *string);

