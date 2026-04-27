#pragma once

#include <cstdint>

#include "sim/boidlist.h"
#include "sim/boidmap.h"
#include "task_master/taskmaster.h"

struct Rules;

enum TaskType {
    POPULATE,
    REBUILD,
    ROW_RUNNER,
    STOP,
};

struct rebuild_args {
    uint32_t y;
    Boid* index_buffer;
};

struct populate_args {
    uint32_t start;
    uint32_t task_size;
    rebuild_args* rebuild_args;
    uint32_t num_tasks;
};

struct row_runner_args {
    uint32_t y;
    Rules* rules;
};

struct BoidTaskSpec;
using BoidTaskMaster = TaskMaster<BoidTaskSpec>;

union BoidTaskArg {
    populate_args* populate;
    rebuild_args* rebuild;
    row_runner_args* row_runner;
};

struct BoidTask {
    TaskType task_type;
    BoidTaskArg arg = {};
    TaskSync* sync = nullptr;
    void (*on_complete)(BoidTaskMaster* task_master, BoidTask* current_task) = nullptr;
};

struct BoidTaskSpec {
    using Task = BoidTask;

    static Task stop_task() {
        return Task { .task_type = TaskType::STOP };
    }

    static bool execute(BoidTaskMaster* task_master, Task& current_task);
};

using Task = BoidTask;

extern BoidList* boid_list;
extern BoidMap* boid_map;

void populate_map2(BoidTaskMaster* task_master, TaskSync* task_monitor, populate_args* arg_list, uint32_t num_tasks);
void update_boids2(row_runner_args* arg_list, BoidTaskMaster* task_master, TaskSync* task_monitor);
void rebuild_list2(rebuild_args* arg_list, BoidTaskMaster* task_master, TaskSync* task_monitor);

bool boids_cpu_supports_avx512();
bool boids_avx512_compiled();
void update_cell2_avx512(int x, int y, const Rules* rules, const BoidList* boid_list);
