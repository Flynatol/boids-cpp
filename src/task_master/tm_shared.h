#pragma once

#include <cstdint>
#include "..\boidlist.h"
#include "..\boidmap.h"
#include "..\ui.h"

enum TaskType {
    POPULATE,
    REBUILD,
    ROW_RUNNER,
    STOP,
};

struct function3_args {
    const char *text;
};

struct rebuild_args {
    uint32_t y;
    Boid *index_buffer;
};

struct populate_args {
    uint32_t start;
    uint32_t task_size;
    rebuild_args* rebuild_args;
    uint32_t num_tasks;
};

struct row_runner_args {
    uint32_t y;
    Rules *rules; 
    row_runner_args* arg_store;
    populate_args* pop_args;
};
