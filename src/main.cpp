#include <iostream>
#include <raylib.h>

#include <raymath.h>
#include <cstdint>
#include <array>
#include <vector>
#include <format>
#include <cstring>
#include <ctime>
#include <random>
#include <chrono>
#include <fstream>
#include <thread>
#include <future>
#include <algorithm>
#include <limits>
#include <immintrin.h>

#define NO_FONT_AWESOME

#include "boidlist.h"
#include "boidmap.h"
#include "ui.h"

#include ".\task_master\taskmaster.h"
#include ".\task_master\tm_shared.h"

#include <glad/glad.h>

#include "rlgl.h"


#define TRACY_CALLSTACK 32
#include "Tracy.hpp"

#pragma comment(lib, "Winmm.lib")

const float TRIANGLE_SIZE = 5.f;
const uint16_t SIGHT_RANGE = 100;

#define FRAME_RATE_LIMIT 165
#define BOID_DENSITY_MAGIC_NUMBER 2304.0
#define CELL_WIDTH 100
#define USE_MULTICORE
#define DEBUG_ENABLED

//#define RUNNER_STORE

#ifdef DEBUG_ENABLED
    #define TIME_NOW std::chrono::high_resolution_clock::now()
    #define DEBUG(...) TraceLog(LOG_DEBUG, TextFormat(__VA_ARGS__));
#endif
#ifndef DEBUG_ENABLED
    #define TIME_NOW (0.f)
    #define DEBUG(...)
#endif

static const Vector2 triangle[3] = {
    Vector2 {0.f, 2 * TRIANGLE_SIZE},
    Vector2 {-TRIANGLE_SIZE, -2 * TRIANGLE_SIZE},
    Vector2 {TRIANGLE_SIZE, -2 * TRIANGLE_SIZE}
};


typedef int32_t Boid;

BoidList* boid_list;
BoidMap* boid_map;

struct PerfMonitor {
    int tick_counter = 0;
    float tps = 0;
    float rolling_average = 0;
} typedef PerfMonitor;

void DrawMeshInstanced2(Mesh mesh, Material material, int instances, float *boid_x, float *boid_y, float *boid_vx, float *boid_vy);

inline void write_map_to_list(int map_cell, const Boid* index_buffer) {
    auto main_buffer = boid_list->m_boid_store;
    auto back_buffer = boid_list->m_backbuffer;

    Boid index = index_buffer[map_cell];
    Boid current = boid_map->m_boid_map[map_cell];

    if (current != -1) {
        boid_map->m_boid_map[map_cell] = index;
    }

    while (current != -1) {
        back_buffer->xs[index] = main_buffer->xs[current];
        back_buffer->ys[index] = main_buffer->ys[current];
        back_buffer->vxs[index] = main_buffer->vxs[current];
        back_buffer->vys[index] = main_buffer->vys[current];
        back_buffer->homes[index] = main_buffer->homes[current];
        back_buffer->depth[index] = main_buffer->depth[current];

        Boid next = main_buffer->index_next[current];

        if (next != -1) {
            back_buffer->index_next[index] = index + 1;
        } else {
            back_buffer->index_next[index] = -1;
        }

        current = next;
        index++;
    }
}

void write_row_to_list(const uint32_t row, const Boid* index_buffer) {
    ZoneScoped;
    for (int x = 0; x < boid_map->m_xsize; x += 1) {
        write_map_to_list(boid_map->m_xsize * row + x, index_buffer);
    }
}

inline void place_boid(Boid boid_to_place) {
    Boid map_pos = boid_map->get_map_pos_nearest(boid_list->m_boid_store->xs[boid_to_place], boid_list->m_boid_store->ys[boid_to_place]);

    boid_map->safety[map_pos].lock();
        Boid old_head = boid_map->m_boid_map[map_pos];
        boid_map->m_boid_map[map_pos] = boid_to_place;
        boid_list->m_boid_store->index_next[boid_to_place] = old_head;
        boid_list->m_boid_store->depth[boid_to_place] = (old_head != -1) ? boid_list->m_boid_store->depth[old_head] + 1 : 1;
    boid_map->safety[map_pos].unlock();
}

void block_populate(int thread_start_pos) {
    //TODO work out a better fix for this
    //We need to work out a better way to allocate work between threads

    auto t = thread_start_pos == ((num_threads - 1) * (boid_list->m_size / num_threads));
    auto correction = boid_list->m_size - (thread_start_pos + (boid_list->m_size / num_threads));

    for (int i = 0; i < (boid_list->m_size) / num_threads + correction * t; i++) {
        place_boid(thread_start_pos + i);
    }
}

inline void populate_n(uint32_t start, uint32_t task_size) {
    ZoneScoped;
    for (uint32_t i = start; i < start + task_size; i++) {
        place_boid(i);
    }
}

void rebuild_list2(rebuild_args* arg_list, TaskMaster *task_master, TaskSync *task_monitor);

void populate_map2(TaskMaster *task_master, TaskSync *task_monitor, populate_args* arg_list, uint32_t num_tasks) {
    memset(boid_map->m_boid_map, -1, sizeof(Boid) * boid_map->m_xsize * boid_map->m_ysize);
    task_master->lock.lock();
    uint32_t tasks_added = 0;

    //Generate tasks
    for (int y = 0; y < num_tasks; y++) {
        task_master->ts_task_buffer.push_back(
            Task {
                .task_type = TaskType::POPULATE,
                .argument_struct = &arg_list[y],
                .sync = task_monitor,
                .on_complete =  (void *) (+[](TaskMaster *task_master, Task *current_task) {
                    auto old_args = ((populate_args *) current_task->argument_struct);

                }),
            }
        );

        tasks_added++;
    }

    task_monitor->task_counter += tasks_added;

    //Go!
    task_master->lock.unlock();
}


template <typename T, typename U>
union ExtractVec {
    U vector;
    T data[8];
};

template <typename T, typename U>
void debug_vector(U vector, const char* format) {
    ExtractVec<T, U> vec = {vector};
    for (int i = 0; i < 8; i++) {
        DEBUG(format, vec.data[i]);
    }
}

inline __m256i vector_sum(__m256i v) {
    __m256i temp = _mm256_hadd_epi32(v, v);
    temp = _mm256_hadd_epi32(temp, temp);
    __m256i fliptemp = _mm256_permute2f128_si256(temp, temp, 1);
    return _mm256_add_epi32(temp, fliptemp);
}

inline __m256 vector_sum(__m256 v) {
    auto temp = _mm256_hadd_ps(v, v);
    temp = _mm256_hadd_ps(temp, temp);
    auto fliptemp = _mm256_permute2f128_ps(temp, temp, 1);
    return _mm256_add_ps(temp, fliptemp);
}

static Mesh GenMeshCustom()
{
    Mesh mesh = { 0 };
    mesh.triangleCount = 1;
    mesh.vertexCount = mesh.triangleCount*3;
    mesh.vertices = (float *) MemAlloc(mesh.vertexCount*3*sizeof(float));    // 3 vertices, 3 coordinates each (x, y, z)
    mesh.colors = (unsigned char *) MemAlloc(mesh.vertexCount*4*sizeof(unsigned char));

    // Vertex at (0, 0, 0)
    mesh.vertices[0] = -TRIANGLE_SIZE;
    mesh.vertices[1] = 0.;
    mesh.vertices[2] = -2 * TRIANGLE_SIZE;

    // Vertex at (1, 0, 2)
    mesh.vertices[3] = 0.;
    mesh.vertices[4] = 0.;
    mesh.vertices[5] = 2 * TRIANGLE_SIZE;

    // Vertex at (2, 0, 0)
    mesh.vertices[6] = TRIANGLE_SIZE;
    mesh.vertices[7] = 0.;
    mesh.vertices[8] = -2 * TRIANGLE_SIZE;

    mesh.colors[0] = 255;
    mesh.colors[1] = 0;
    mesh.colors[2] = 0;
    mesh.colors[3] = 255;

    mesh.colors[4] = 0;
    mesh.colors[5] = 255;
    mesh.colors[6] = 0;
    mesh.colors[7] = 255;

    mesh.colors[8] = 0;
    mesh.colors[9] = 0;
    mesh.colors[10] = 255;
    mesh.colors[11] = 255;

    // Upload mesh data from CPU (RAM) to GPU (VRAM) memory
    UploadMesh(&mesh, false);

    return mesh;
}


inline void update_cell2(const int x, const int y, const Rules *rules, const BoidList *boid_list) {
    const auto world_height = boid_map->m_cell_size * boid_map->m_ysize;
    const auto world_width = boid_map->m_cell_size * boid_map->m_xsize;

    //Maybe add these to task allocator
    Boid cell_begin = boid_map->m_boid_map[y * boid_map->m_xsize + x];

    // If this cell is empty then just return
    if (cell_begin == -1 ) return;
    Boid cell_end = cell_begin + boid_list->m_boid_store->depth[cell_begin];

    const auto xs = boid_list->m_boid_store->xs;
    const auto ys = boid_list->m_boid_store->ys;
    const auto vxs = boid_list->m_boid_store->vxs;
    const auto vys = boid_list->m_boid_store->vys;
    const auto homes = boid_list->m_boid_store->homes;

    const auto ads_vec = _mm256_set1_ps(rules->avoid_distance_squared);
    const auto srs_vec = _mm256_set1_ps(rules->sight_range_squared);
    const auto af_vec  = _mm256_set1_ps(rules->avoid_factor);

    for (Boid current_boid = cell_begin; current_boid < cell_end; current_boid += 8) {
        auto current_xs_vec = _mm256_loadu_ps(&xs[current_boid]);
        auto current_ys_vec = _mm256_loadu_ps(&ys[current_boid]);

        //Tracking vecs
        __m256 sep_x_vec = _mm256_set1_ps(0.);
        __m256 sep_y_vec = _mm256_set1_ps(0.);

        __m256 avg_x_vec = _mm256_set1_ps(0.);
        __m256 avg_y_vec = _mm256_set1_ps(0.);

        __m256 avg_vx_vec = _mm256_set1_ps(0.);
        __m256 avg_vy_vec = _mm256_set1_ps(0.);

        __m256i isc = _mm256_set1_epi32(0);

        //For each row
        for (int cy = -1; cy <= 1; cy++) {
            Boid row_begin = -1;
            Boid row_end = -1;

            //Todo check if this would be faster branchless -- probably not worth the readability
            for (int cx = -1; cx <= 1; cx++) {
                // This line could be slow
                Boid current = boid_map->get_coord(y + cy, x + cx);
                if (current != -1) {
                    if (row_begin == -1) row_begin = current;
                    //This line is probably bad too
                    row_end = current + boid_list->m_boid_store->depth[current];
                }
            }

            // For each block of boids in current row
            for (Boid nearby_boid = row_begin; nearby_boid < row_end; nearby_boid += 8) {
                auto nearby_xs_vec = _mm256_loadu_ps(&xs[nearby_boid]);
                auto nearby_ys_vec = _mm256_loadu_ps(&ys[nearby_boid]);

                auto nearby_vxs_vec = _mm256_loadu_ps(&vxs[nearby_boid]);
                auto nearby_vys_vec = _mm256_loadu_ps(&vys[nearby_boid]);

                for (int i = 0; i < 8; i++) {
                    const auto xs_delta = _mm256_sub_ps(current_xs_vec, nearby_xs_vec);
                    const auto ys_delta = _mm256_sub_ps(current_ys_vec, nearby_ys_vec);
                    const auto ds_vec = _mm256_add_ps(_mm256_mul_ps(xs_delta, xs_delta), _mm256_mul_ps(ys_delta, ys_delta));

                    //Sight range squared
                    const auto srs_mask = _mm256_cmp_ps(ds_vec, srs_vec, _CMP_LT_OS);

                    //Avoid distance squared
                    const auto ads_mask = _mm256_cmp_ps(ds_vec, ads_vec, _CMP_LT_OS);

                    //In sight, but not within avoid range.
                    const auto sna_bitmask = _mm256_andnot_ps(ads_mask, srs_mask);

                    //In sight, but not within avoid range. Warning - not a bit mask!
                    const auto sna_fpmask = _mm256_and_ps(sna_bitmask, _mm256_set1_ps(1.));

                    const auto ads_take_ds = _mm256_sub_ps(ads_vec, ds_vec);
                    const auto ads_take_ds_sqr = _mm256_mul_ps(ads_take_ds, ads_take_ds);

                    sep_x_vec = _mm256_add_ps(sep_x_vec, _mm256_and_ps(ads_mask, _mm256_mul_ps(xs_delta, ads_take_ds_sqr)));
                    sep_y_vec = _mm256_add_ps(sep_y_vec, _mm256_and_ps(ads_mask, _mm256_mul_ps(ys_delta, ads_take_ds_sqr)));

                    avg_vx_vec = _mm256_fmadd_ps(sna_fpmask, nearby_vxs_vec, avg_vx_vec);
                    avg_vy_vec = _mm256_fmadd_ps(sna_fpmask, nearby_vys_vec, avg_vy_vec);

                    avg_x_vec = _mm256_fmadd_ps(sna_fpmask, nearby_xs_vec, avg_x_vec);
                    avg_y_vec = _mm256_fmadd_ps(sna_fpmask, nearby_ys_vec, avg_y_vec);

                    isc = _mm256_add_epi32(isc, _mm256_cvtps_epi32(sna_fpmask));

                    //Then permute
                    nearby_xs_vec   = _mm256_permutevar8x32_ps(nearby_xs_vec,  _mm256_set_epi32(0, 7, 6, 5, 4, 3, 2, 1));
                    nearby_ys_vec   = _mm256_permutevar8x32_ps(nearby_ys_vec,  _mm256_set_epi32(0, 7, 6, 5, 4, 3, 2, 1));
                    nearby_vxs_vec  = _mm256_permutevar8x32_ps(nearby_vxs_vec, _mm256_set_epi32(0, 7, 6, 5, 4, 3, 2, 1));
                    nearby_vys_vec  = _mm256_permutevar8x32_ps(nearby_vys_vec, _mm256_set_epi32(0, 7, 6, 5, 4, 3, 2, 1));
                }


                //UNROLL8(compute, i);
            }


        }

        //Write out the data from tracking vecs
        //I should probably change the type of ISC
        __m256 isc_mask = _mm256_castsi256_ps(_mm256_cmpgt_epi32(isc, _mm256_set1_epi32(0)));

        //Avoidance
        auto vxs_out = _mm256_loadu_ps(&vxs[current_boid]);
        auto vys_out = _mm256_loadu_ps(&vys[current_boid]);

        vxs_out = _mm256_fmadd_ps(sep_x_vec, af_vec, vxs_out);
        vys_out = _mm256_fmadd_ps(sep_y_vec, af_vec, vys_out);

        auto cvt_isc = _mm256_cvtepi32_ps(isc);

        //Alignment
        avg_vx_vec = _mm256_div_ps(avg_vx_vec, cvt_isc);
        avg_vy_vec = _mm256_div_ps(avg_vy_vec, cvt_isc);

        vxs_out = _mm256_fmadd_ps(_mm256_set1_ps(rules->alignment_factor), _mm256_and_ps(isc_mask, _mm256_sub_ps(avg_vx_vec, vxs_out)), vxs_out);
        vys_out = _mm256_fmadd_ps(_mm256_set1_ps(rules->alignment_factor), _mm256_and_ps(isc_mask, _mm256_sub_ps(avg_vy_vec, vys_out)), vys_out);

        //Cohesion
        avg_x_vec = _mm256_div_ps(avg_x_vec, cvt_isc);
        avg_y_vec = _mm256_div_ps(avg_y_vec, cvt_isc);

        vxs_out = _mm256_fmadd_ps(_mm256_set1_ps(rules->cohesion_factor), _mm256_and_ps(isc_mask, _mm256_sub_ps(avg_x_vec, current_xs_vec)), vxs_out);
        vys_out = _mm256_fmadd_ps(_mm256_set1_ps(rules->cohesion_factor), _mm256_and_ps(isc_mask, _mm256_sub_ps(avg_y_vec, current_ys_vec)), vys_out);

        //Window edges
        const auto rf = _mm256_set1_ps(rules->edge_factor);
        const auto ew = _mm256_set1_ps(rules->edge_width);
        const auto wh = _mm256_set1_ps(world_height);
        const auto ww = _mm256_set1_ps(world_width);

        auto cmpx_1 = _mm256_cmp_ps(current_xs_vec, ew, _CMP_LT_OS);
        auto cmpx_2 = _mm256_cmp_ps(current_xs_vec, _mm256_set1_ps(world_width - rules->edge_width), _CMP_GT_OS);
        auto cmpy_1 = _mm256_cmp_ps(current_ys_vec, ew, _CMP_LT_OS);
        auto cmpy_2 = _mm256_cmp_ps(current_ys_vec, _mm256_set1_ps(world_height - rules->edge_width), _CMP_GT_OS);

        vxs_out = _mm256_add_ps(vxs_out, _mm256_sub_ps(_mm256_and_ps(cmpx_1, rf), _mm256_and_ps(cmpx_2, rf)));
        vys_out = _mm256_add_ps(vys_out, _mm256_sub_ps(_mm256_and_ps(cmpy_1, rf), _mm256_and_ps(cmpy_2, rf)));

        auto const minspeed = 3.0f;
        auto const maxspeed = 4.0f;

        auto rspeed_vec = _mm256_rsqrt_ps(_mm256_add_ps(_mm256_mul_ps(vxs_out, vxs_out), _mm256_mul_ps(vys_out, vys_out)));

        auto ispeed_vec = _mm256_mul_ps(rspeed_vec, _mm256_set1_ps(maxspeed));
        auto lspeed_vec = _mm256_mul_ps(rspeed_vec, _mm256_set1_ps(minspeed));

        auto too_slow = _mm256_cmp_ps(rspeed_vec, _mm256_set1_ps(1.0/minspeed), _CMP_GT_OS);
        auto too_fast = _mm256_cmp_ps(rspeed_vec, _mm256_set1_ps(1.0/maxspeed), _CMP_LT_OS);
        auto too_inf = _mm256_cmp_ps(rspeed_vec, _mm256_set1_ps(std::numeric_limits<float>::max()), _CMP_GT_OS);

        auto multiplers = _mm256_set1_ps(1.0);
        multiplers = _mm256_blendv_ps(multiplers, lspeed_vec, too_slow);
        multiplers = _mm256_blendv_ps(multiplers, ispeed_vec, too_fast);
        multiplers = _mm256_blendv_ps(multiplers, _mm256_set1_ps(1.0), too_inf);

        vxs_out = _mm256_mul_ps(multiplers, vxs_out);
        vys_out = _mm256_mul_ps(multiplers, vys_out);

        ////////////////////////////////////////////

        const auto homes_vec = _mm256_loadu_si256((const __m256i*) &homes[current_boid]);

        auto home_index_x_vec = _mm256_cvtepi32_ps(_mm256_add_epi32(_mm256_and_si256(homes_vec, _mm256_set1_epi32(15)), _mm256_set1_epi32(1)));
        auto home_index_y_vec = _mm256_cvtepi32_ps(_mm256_add_epi32(_mm256_srlv_epi32(homes_vec, _mm256_set1_epi32(4)), _mm256_set1_epi32(1)));

        auto home_loc_x_vec = _mm256_fmadd_ps(home_index_x_vec, _mm256_set1_ps((float) ((world_width - rules->edge_width * 2) / (16 + 1))), ew);
        auto home_loc_y_vec = _mm256_fmadd_ps(home_index_y_vec, _mm256_set1_ps((float) ((world_height - rules->edge_width * 2) / (9 + 1))), ew);

        auto dx_vec = _mm256_sub_ps(home_loc_x_vec, current_xs_vec);
        auto dy_vec = _mm256_sub_ps(home_loc_y_vec, current_ys_vec);

        vxs_out = _mm256_fmadd_ps(dx_vec, _mm256_set1_ps(rules->homing), vxs_out);
        vys_out = _mm256_fmadd_ps(dy_vec, _mm256_set1_ps(rules->homing), vys_out);


#ifndef RUNNER_STORE
        __m256 temp = _mm256_add_ps(_mm256_set1_ps((float)current_boid), _mm256_set_ps(7., 6., 5., 4., 3., 2., 1., 0.));
        __m256 out_mask = _mm256_cmp_ps(temp, _mm256_set1_ps(cell_end), _CMP_LT_OS);
        // ONLY WRITE OUT VELS OF BOIDS ACTUALLY IN OUR CELL

        auto xs_out = _mm256_add_ps(current_xs_vec, _mm256_and_ps(vxs_out, out_mask));
        auto ys_out = _mm256_add_ps(current_ys_vec, _mm256_and_ps(vys_out, out_mask));

        _mm256_storeu_ps(&xs[current_boid], xs_out);
        _mm256_storeu_ps(&ys[current_boid], ys_out);
#endif
        ///////////////////////////////////////////

        _mm256_storeu_ps(&vxs[current_boid], vxs_out);
        _mm256_storeu_ps(&vys[current_boid], vys_out);
    }
}


void debug_matrix(Matrix matrix) {
    float16 matrix2 = MatrixToFloatV(matrix);
    DEBUG("Begin matrix")
    for (int i = 0; i < 4; i++) {
        DEBUG("%f, %f, %f, %f", matrix2.v[i],matrix2.v[i+4],matrix2.v[i+8],matrix2.v[i+12]);
    }
    DEBUG("End matrix")
}

inline void update_non_interacting2(const BoidMap* boid_map, const Rules* rules, const BoidList* boid_list) {
    ZoneScoped;

    const auto xs = boid_list->m_boid_store->xs;
    const auto ys = boid_list->m_boid_store->ys;
    const auto vxs = boid_list->m_boid_store->vxs;
    const auto vys = boid_list->m_boid_store->vys;

    for (Boid boid = 0; boid < boid_list->m_size; boid += 8) {
        _mm256_store_ps(&xs[boid], _mm256_add_ps(_mm256_load_ps(&vxs[boid]), _mm256_load_ps(&xs[boid])));
        _mm256_store_ps(&ys[boid], _mm256_add_ps(_mm256_load_ps(&vys[boid]), _mm256_load_ps(&ys[boid])));
    }
}

inline void row_runner(const int y, const Rules *rules) {
    ZoneScoped;

#ifndef RUNNER_STORE
    for (int x = 0; x < boid_map->m_xsize; x++) {
        update_cell2(x, y, rules, boid_list);
    }
#endif

#ifdef RUNNER_STORE
    Boid first_cell_begin = -1;
    Boid last_cell_begin = -1;

    for (int x = 0; x < boid_map->m_xsize; x++) {
        Boid cell_begin = boid_map->m_boid_map[y * boid_map->m_xsize + x];

        if (first_cell_begin == -1) first_cell_begin = cell_begin;
        if (cell_begin != -1) last_cell_begin = cell_begin;
        update_cell2(x, y, rules, boid_list);
    }

    if (first_cell_begin == -1) return;
    Boid last_cell_end = last_cell_begin + boid_list->m_boid_store->depth[last_cell_begin];

    Boid boid;
    for (boid = first_cell_begin; boid < (last_cell_end - 8); boid += 8) {
        _mm256_store_ps(&boid_list->m_boid_store->xs[boid], _mm256_add_ps(_mm256_load_ps(&boid_list->m_boid_store->vxs[boid]), _mm256_load_ps(&boid_list->m_boid_store->xs[boid])));
        _mm256_store_ps(&boid_list->m_boid_store->ys[boid], _mm256_add_ps(_mm256_load_ps(&boid_list->m_boid_store->vys[boid]), _mm256_load_ps(&boid_list->m_boid_store->ys[boid])));
    }

    auto temp = _mm256_add_ps(_mm256_set1_ps((float) boid), _mm256_set_ps(7., 6., 5., 4., 3., 2., 1., 0.));
    auto out_mask = _mm256_cmp_ps(temp, _mm256_set1_ps(last_cell_end), _CMP_LT_OS);

    _mm256_store_ps(&boid_list->m_boid_store->xs[boid], _mm256_add_ps(_mm256_and_ps(_mm256_load_ps(&boid_list->m_boid_store->vxs[boid]), out_mask), _mm256_load_ps(&boid_list->m_boid_store->xs[boid])));
    _mm256_store_ps(&boid_list->m_boid_store->ys[boid], _mm256_add_ps(_mm256_and_ps(_mm256_load_ps(&boid_list->m_boid_store->vys[boid]), out_mask), _mm256_load_ps(&boid_list->m_boid_store->ys[boid])));

#endif
}


void runner(TaskMaster *task_master, uint8_t thread_id) {
    while(1) {
        //Wait for task
        Task current_task = task_master->get_task();

        switch (current_task.task_type) {
            case TaskType::ROW_RUNNER: {
                    auto s = ((row_runner_args *) current_task.argument_struct);
                    row_runner(s->y, s->rules);
                }
                break;

            case TaskType::REBUILD: {
                    auto s = ((rebuild_args *) current_task.argument_struct);
                    write_row_to_list(s->y, s->index_buffer);
                }
                break;

            case TaskType::POPULATE: {
                    auto s = ((populate_args *) current_task.argument_struct);
                    populate_n(s->start, s->task_size);
                }
                break;

            case TaskType::STOP:
                return;
        }

        if (current_task.sync != NULL) {
            current_task.sync->tc_lock.lock();
                if (current_task.on_complete != NULL && current_task.sync->task_counter == 1) {
                    current_task.sync->task_counter -= 1;
                    ((void (*)(TaskMaster *, Task *)) current_task.on_complete)(task_master, &current_task);
                } else {
                    current_task.sync->task_counter -= 1;
                }
            current_task.sync->tc_lock.unlock();
        }
    }
}

void update_boids2(row_runner_args* arg_list, TaskMaster *task_master, TaskSync *task_monitor) {
    //  Add tasks to task master
    //  Spin up some threads
    //  Let em go
    task_master->lock.lock();

    uint32_t tasks_added = 0;

    for (int y = 0; y < boid_map->m_ysize; y += 2) {
        task_master->ts_task_buffer.push_back(
            Task {
                .task_type = TaskType::ROW_RUNNER,
                .argument_struct = &arg_list[y],
                .sync = task_monitor,
                .on_complete = (void *) (+[](TaskMaster *task_master, Task *current_task)
                {
                    auto old_args = ((row_runner_args *) current_task->argument_struct);
                    uint32_t tasks_added = 0;
                    task_master->lock.lock();

                    for (int y = 1; y < boid_map->m_ysize; y += 2) {
                        task_master->ts_task_buffer.push_back(
                            Task {
                                .task_type = TaskType::ROW_RUNNER,
                                .argument_struct = &old_args->arg_store[y],
                                .sync = current_task->sync,
                                .on_complete = (void *) (+[](TaskMaster *task_master, Task *current_task) {
                                    auto old_args = ((row_runner_args *) current_task->argument_struct);
                                    //update_non_interacting2(boid_map, old_args->rules, boid_list);
                                    populate_map2(task_master, current_task->sync, old_args->pop_args, old_args->pop_args->num_tasks);

                                    FrameMarkEnd("Update Boids");
                                }),
                            }
                        );
                        tasks_added++;
                    }

                    current_task->sync->task_counter = tasks_added;
                    task_master->lock.unlock();
                }),
            }
        );

        tasks_added++;
    }

    task_monitor->tc_lock.lock();
    task_monitor->task_counter += tasks_added;
    task_monitor->tc_lock.unlock();

    task_master->lock.unlock();


}

void rebuild_list2(rebuild_args* arg_list, TaskMaster *task_master, TaskSync *task_monitor) {
    ZoneScoped;
    task_master->lock.lock();

    auto start = TIME_NOW;

    uint32_t tasks_added = 0;

    int counter = 0;
    // this is taking about 0.6 ms
    for (int i = 0; i < boid_map->m_xsize * boid_map->m_ysize; i++) {
        Boid current = boid_map->m_boid_map[i];
        arg_list->index_buffer[i] = counter;
        counter += (current != -1) * boid_list->m_boid_store->depth[current];
    }

    auto pre_task_gen = TIME_NOW;

    //Generate tasks
    for (int y = 0; y < boid_map->m_ysize; y++) {
        task_master->ts_task_buffer.push_back(
            Task {
                .task_type = TaskType::REBUILD,
                .argument_struct = &arg_list[y],
                .sync = task_monitor,
                .on_complete = (void *) (+[](TaskMaster *task_master, Task *current_task) {
                        ZoneScoped
                        auto temp_boid_store = boid_list->m_boid_store;
                        boid_list->m_boid_store = boid_list->m_backbuffer;
                        boid_list->m_backbuffer = temp_boid_store;
                    }),
            }
        );

        tasks_added++;
    }

    auto done = TIME_NOW;

    task_monitor->task_counter += tasks_added;

    //DEBUG("rb_list: %0.4f, comp: %0.4f", std::chrono::duration<double, std::milli>(pre_task_gen - start).count(), std::chrono::duration<double, std::milli>(done - pre_task_gen).count());

    //Go!
    task_master->lock.unlock();
}

Matrix MatrixLookDown(Vector3 eye)
{
    Matrix result = { 0 };

    result.m0 = 1.0f;
    result.m1 = 0.0f;
    result.m2 = 0.0f;
    result.m3 = 0.0f;
    result.m4 = 0.0f;
    result.m5 = 0.0f;
    result.m6 = 1.0f;
    result.m7 = 0.0f;
    result.m8 = 0.0f;
    result.m9 = -1.0f;
    result.m10 = 0.0f;
    result.m11 = 0.0f;
    result.m12 = -(eye.x);
    result.m13 = (eye.z);
    result.m14 = -(eye.y);
    result.m15 = 1.0f;

    return result;
}

void fBeginMode3D(Camera camera) {
    //rlDrawRenderBatchActive();      // Update and draw internal render batch

    rlMatrixMode(RL_PROJECTION);    // Switch to projection matrix
    rlPushMatrix();                 // Save previous matrix, which contains the settings for the 2d ortho projection
    rlLoadIdentity();               // Reset current matrix (projection)

    float aspect = (float) GetScreenWidth() / (float) GetScreenHeight();


    // Setup orthographic projection
    double top = camera.fovy/2.0;
    double right = top*aspect;

    rlOrtho(-right, right, -top,top, RL_CULL_DISTANCE_NEAR, RL_CULL_DISTANCE_FAR);


    rlMatrixMode(RL_MODELVIEW);     // Switch back to modelview matrix
    rlLoadIdentity();               // Reset current matrix (modelview)

    // Setup Camera view
    Matrix matView = MatrixLookDown(camera.position);
    rlMultMatrixf(MatrixToFloat(matView));      // Multiply modelview matrix by view matrix (camera)
}

void render(Ui *ui, Rules &rules, Camera2D cam, Camera3D camera, Mesh *tri, Material *matInstances) {
    ZoneScoped;
    BeginDrawing();
        ClearBackground(BLACK);

        auto t_start_3d = TIME_NOW;
        //We could cull here by offsetting the start pointers by some amount
        auto const boid_store = boid_list->m_backbuffer;
        int offset = 0;

        fBeginMode3D(camera);
            DrawMeshInstanced2(*tri, *matInstances, boid_list->m_size - offset, boid_store->xs + offset, boid_store->ys + offset, boid_store->vxs + offset, boid_store->vys + offset);
        EndMode3D();

        auto t_end_3d = TIME_NOW;

        ui->Render(cam, camera, rules);

    EndDrawing();
}

// Get the world space position for a 2d camera screen space position
Vector3 GetScreenToWorld3D(Vector2 position, Camera camera)
{
    Matrix invMatCamera = MatrixInvert(GetCameraMatrix(camera));
    return Vector3Transform( Vector3 { position.x, position.y, 0 }, invMatCamera);

    //return Vector3 {transform.x, transform.y};
}


int main(int argc, char* argv[]) {

    uint32_t num_boids = (argc > 1) ? atoi(argv[1]) : (2000000);

    SetTraceLogLevel(LOG_ALL);

    if (num_boids < num_threads) {
        TraceLog(LOG_ERROR, TextFormat("Please request more boids than threads (%d)! Defaulting to 1000", num_threads));
        num_boids = 1000;
    }

    InitWindow(0, 0, "RayLib Boids!");
    SetTargetFPS(FRAME_RATE_LIMIT);

    const int screen_width = GetScreenWidth();
    const int screen_height = GetScreenHeight();

    DEBUG("Using %d args", argc);
    DEBUG("Using %d threads", num_threads);

    TraceLog(LOG_DEBUG, TextFormat("Boid size is: %d bytes", sizeof(Boid)));

    Mesh tri = GenMeshCustom();

    Shader boid_shader = LoadShader(TextFormat(RESOURCES_PATH "shaders/glsl%i/directional.vs", 330),
                                    TextFormat(RESOURCES_PATH "/shaders/glsl%i/simple.fs", 330));

    boid_shader.locs[SHADER_LOC_VECTOR_VIEW] = GetShaderLocation(boid_shader, "viewPos");
    boid_shader.locs[SHADER_LOC_MATRIX_MODEL] = GetShaderLocationAttrib(boid_shader, "instanceTransform");

    boid_shader.locs[26] = GetShaderLocationAttrib(boid_shader, "boid_x");
    boid_shader.locs[27] = GetShaderLocationAttrib(boid_shader, "boid_y");
    boid_shader.locs[28] = GetShaderLocationAttrib(boid_shader, "boid_vx");
    boid_shader.locs[29] = GetShaderLocationAttrib(boid_shader, "boid_vy");

    DEBUG("boid_x at: %d", boid_shader.locs[26]);
    DEBUG("viewPos at: %d", boid_shader.locs[SHADER_LOC_VECTOR_VIEW]);
    DEBUG("instanceTransform at: %d", boid_shader.locs[SHADER_LOC_MATRIX_MODEL]);

    Material matInstances = LoadMaterialDefault();
    matInstances.shader = boid_shader;
    matInstances.maps[MATERIAL_MAP_DIFFUSE].color = RED;

    Rules rules = {
        .avoid_distance_squared = 1000.f,
        .avoid_factor = 0.00000002f,
        .sight_range = SIGHT_RANGE,
        .sight_range_squared = SIGHT_RANGE * SIGHT_RANGE,
        .alignment_factor = 0.05f,
        .cohesion_factor = 0.0005f,
        .edge_width = 300,
        .edge_factor = 0.05,
        .rand = 0.1,
        .homing = 0.0000005,
        .show_lines = false,
        .min_speed = 2,
        .max_speed = 3,
    };

    Ui ui;

    PerfMonitor perf_monitor;

    const uint32_t world_size_mult = std::ceil(sqrt((BOID_DENSITY_MAGIC_NUMBER / ((double) screen_height)) * ((double)num_boids / (double) screen_width )));

    const int world_width = screen_width * world_size_mult;
    const int world_height = screen_height * world_size_mult;

    Camera2D cam = { 0 };
    cam.zoom = static_cast<float>(screen_width) / world_width;

    Camera camera = {
        .position = Vector3 {world_width/2.f, 200.f, world_height/2.f},
        .target = Vector3 {world_width/2.f, 0.0f, world_height/2.f},
        .up =  Vector3 { 0.0f, 0.0f, -1.0f },
        .fovy = (float) world_height,
        .projection = CAMERA_ORTHOGRAPHIC,
    };

    BoidList boid_list_local(num_boids);
    BoidMap boid_map_local(world_height, world_width, CELL_WIDTH);

    boid_list = &boid_list_local;
    boid_map = &boid_map_local;

    const float home_width = ((world_width - rules.edge_width * 2) / (16 + 1));
    const float home_height = ((world_height - rules.edge_width * 2) / (9 + 1));
    
    std::default_random_engine generator;
    std::uniform_real_distribution<float> width_distribution2 (-home_width / 2.f, home_width / 2.f);
    std::uniform_real_distribution<float> height_distribution2 (-home_height / 2.f, home_height / 2.f);

    //Populate some boids
    for (int i = 0; i < boid_list->m_size; i++) {
        boid_list->m_boid_store->index_next[i] = -1;
        boid_list->m_boid_store->vxs[i] = (std::rand() % 3) - 1;
        boid_list->m_boid_store->vys[i] = (std::rand() % 3) - 1;
        boid_list->m_boid_store->homes[i] = rand() % 144;

        int home_index_y = boid_list->m_boid_store->homes[i] / 16;
        int home_index_x = boid_list->m_boid_store->homes[i] % 16;

        boid_list->m_boid_store->xs[i] = (home_index_x + 1) * home_width  + rules.edge_width + width_distribution2(generator);
        boid_list->m_boid_store->ys[i] = (home_index_y + 1) * home_height + rules.edge_width + height_distribution2(generator);
    }

    //This number was found through experimentation, might not be optimal for all number of boids (10000)
    const uint32_t task_size = 10000;
    const uint32_t num_tasks = (boid_list->m_size + (task_size - 1)) / task_size; //to force rounding up
    populate_args *args_populate = new populate_args[num_tasks];


    auto args_update = new row_runner_args[boid_map->m_ysize];
    auto args_rebuild = new rebuild_args[boid_map->m_ysize];

    Boid *index_buffer = (Boid *) malloc(boid_map->m_xsize * boid_map->m_ysize * sizeof(Boid));

    for (uint32_t i = 0; i < boid_map->m_ysize; i++) {
        args_update[i] = row_runner_args {
            .y = i,
            .rules = &rules,
            .arg_store = args_update,
            .pop_args = args_populate,
        };

        args_rebuild[i] = rebuild_args {
            .y = i,
            .index_buffer = index_buffer,
        };
    }

    DEBUG("Num tasks: %d", num_tasks);

    for (uint32_t i = 0; i < num_tasks; i++) {
        args_populate[i] = populate_args {
            .start = i * task_size,
            .task_size = std::min(task_size, boid_list->m_size - (i * task_size)),
            .rebuild_args = args_rebuild,
        };
    }
    args_populate[0].num_tasks = num_tasks;

    TaskMaster task_master;
    task_master.start_threads();

    TaskSync task_populate;
    populate_map2(&task_master, &task_populate, args_populate, num_tasks);
    task_populate.wait();

    TaskSync task_update;

    TaskSync task_rebuild;
    rebuild_list2(args_rebuild, &task_master, &task_rebuild);
    task_rebuild.wait();

    FrameMarkStart("Update Boids");
    update_boids2(args_update, &task_master, &task_update);
    task_update.wait();

    rlEnableShader(matInstances.shader.id);

    float camera_zoom = (1. / world_size_mult);

    DEBUG("Starting main loop");
    while (WindowShouldClose() == false){
        auto t_update_start = TIME_NOW;

        FrameMarkStart("Update Boids");

        //TODO move camera stuff to class
        if (IsMouseButtonDown(MOUSE_BUTTON_RIGHT)){
            Vector2 delta = Vector2Scale(GetMouseDelta(), -1.0f / cam.zoom);

            cam.target = Vector2Add(cam.target, delta);

            camera.position = Vector3 {camera.position.x + delta.x, camera.position.y, camera.position.z + delta.y };
            camera.target = Vector3 {camera.target.x + delta.x, camera.target.y, camera.target.z + delta.y };
        }

        if (float wheel = GetMouseWheelMove())
        {
            Vector2 mouseWorldPos = GetScreenToWorld2D(GetMousePosition(), cam);
            Vector3 mouseWorldPos2 = GetScreenToWorld3D(GetMousePosition(), camera);


            DEBUG("OG: [X: %f, Y: %f] NEW: [X: %f, Y: %f, Z: %f]", mouseWorldPos.x, mouseWorldPos.y, mouseWorldPos2.x, mouseWorldPos2.y, mouseWorldPos2.z)
            
            auto m_pos = GetMousePosition();

            cam.offset = m_pos;
            cam.target = mouseWorldPos;
            cam.zoom += wheel * 0.125f;

            if (cam.zoom < (1./world_size_mult)) cam.zoom = (1./world_size_mult);

            camera.fovy = screen_height / cam.zoom;
            
            camera.position = Vector3 {
                .x = (screen_width * 0.5f - m_pos.x) / cam.zoom + mouseWorldPos.x,
                .y = camera.position.y,
                .z = (screen_height * 0.5f - m_pos.y) / cam.zoom + mouseWorldPos.y,
            };

            camera.target = Vector3 {camera.position.x, 0.f, camera.position.z};
        }

        // Update camera position in shader
        float cameraPos[3] = { camera.position.x, camera.position.y, camera.position.z };
        SetShaderValue(boid_shader, boid_shader.locs[SHADER_LOC_VECTOR_VIEW], cameraPos, SHADER_UNIFORM_VEC3);

        FrameMarkStart("wait_rebuild");
        task_rebuild.wait();
        FrameMarkEnd("wait_rebuild");

        update_boids2(args_update, &task_master, &task_update);

        auto t_start_drawing = TIME_NOW;
        FrameMarkStart("Render");
        render(&ui, rules, cam, camera, &tri, &matInstances);
        FrameMarkEnd("Render");
        auto t_end_drawing = TIME_NOW;

        task_update.wait();

        // Rendering and new state now done -- Flipping buffer allowed
        rebuild_list2(args_rebuild, &task_master, &task_rebuild);

        auto t_update_end = TIME_NOW;

        DEBUG("Rebuild: na, render: %0.4f, comp: %0.4f", std::chrono::duration<double, std::milli>(t_end_drawing-t_start_drawing).count(), std::chrono::duration<double, std::milli>(t_update_end-t_update_start).count());

        FrameMark;
    }

    task_master.queue_stop_all();
    task_master.join_all();

    rl_CloseWindow();
    return 0;
}

unsigned int flLoadVertexBuffer(const void *buffer, int size, unsigned int prev_id)
{
    if (!prev_id) {
        unsigned int id = 0;

        glGenBuffers(1, &id);
        glBindBuffer(GL_ARRAY_BUFFER, id);
        glBufferData(GL_ARRAY_BUFFER, size, buffer, GL_STREAM_DRAW);

        return id;
    } else {
        glBindBuffer(GL_ARRAY_BUFFER, prev_id);
        glBufferSubData(GL_ARRAY_BUFFER, 0, size, buffer);

        return prev_id;
    }
}


// Instancing required variables
unsigned int instances_boid_x_ID = 0, instances_boid_y_ID = 0, instances_boid_vx_ID = 0, instances_boid_vy_ID = 0;

// Modified version of DrawMeshInstanced from the raylib module "rmodels.c"
// My modifications bind the x, y, vx, and vy of each boids as vertex attributes
// We can then use these to generate the transformation matricies on the GPU instead.
void DrawMeshInstanced2(Mesh mesh, Material material, int instances, float *boid_x, float *boid_y, float *boid_vx, float *boid_vy)
{
    FrameMarkStart("Begin Render");
    #define MAX_MATERIAL_MAPS 12
    #define MAX_MESH_VERTEX_BUFFERS 7


    // Bind shader program
    rlEnableShader(material.shader.id);

    unsigned int id = 0;
    glGenBuffers(1, &id);

    // Send required data to shader (matrices, values)
    //-----------------------------------------------------
    // Upload to shader material.colDiffuse
    if (material.shader.locs[SHADER_LOC_COLOR_DIFFUSE] != -1)
    {
        float values[4] = {
            (float)material.maps[MATERIAL_MAP_DIFFUSE].color.r/255.0f,
            (float)material.maps[MATERIAL_MAP_DIFFUSE].color.g/255.0f,
            (float)material.maps[MATERIAL_MAP_DIFFUSE].color.b/255.0f,
            (float)material.maps[MATERIAL_MAP_DIFFUSE].color.a/255.0f
        };

        rlSetUniform(material.shader.locs[SHADER_LOC_COLOR_DIFFUSE], values, SHADER_UNIFORM_VEC4, 1);
    }

    // Get a copy of current matrices to work with,
    // just in case stereo render is required, and we need to modify them
    // NOTE: At this point the modelview matrix just contains the view matrix (camera)
    // That's because BeginMode3D() sets it and there is no model-drawing function
    // that modifies it, all use rlPushMatrix() and rlPopMatrix()
    //Matrix matView = rlGetMatrixModelview();
    //Matrix matModelView = MatrixIdentity();
    //Matrix matProjection = rlGetMatrixProjection();

    // Upload view and projection matrices (if locations available)
    //if (material.shader.locs[SHADER_LOC_MATRIX_VIEW] != -1) rlSetUniformMatrix(material.shader.locs[SHADER_LOC_MATRIX_VIEW], matView);
    //if (material.shader.locs[SHADER_LOC_MATRIX_PROJECTION] != -1) rlSetUniformMatrix(material.shader.locs[SHADER_LOC_MATRIX_PROJECTION], matProjection);

    // Enable mesh VAO to attach new buffer
    rlEnableVertexArray(mesh.vaoId);

    FrameMarkEnd("Begin Render");
    FrameMarkStart("Upload");

    instances_boid_x_ID = flLoadVertexBuffer(boid_x, instances*sizeof(float), instances_boid_x_ID);
    rlEnableVertexAttribute(material.shader.locs[26]);
    rlSetVertexAttribute(material.shader.locs[26], 1, RL_FLOAT, false, sizeof(float), 0);
    rlSetVertexAttributeDivisor(material.shader.locs[26], 1);

    instances_boid_y_ID = flLoadVertexBuffer(boid_y, instances*sizeof(float), instances_boid_y_ID);
    rlEnableVertexAttribute(material.shader.locs[27]);
    rlSetVertexAttribute(material.shader.locs[27], 1, RL_FLOAT, false, sizeof(float), 0);
    rlSetVertexAttributeDivisor(material.shader.locs[27], 1);

    instances_boid_vx_ID = flLoadVertexBuffer(boid_vx, instances*sizeof(float), instances_boid_vx_ID);
    rlEnableVertexAttribute(material.shader.locs[28]);
    rlSetVertexAttribute(material.shader.locs[28], 1, RL_FLOAT, false, sizeof(float), 0);
    rlSetVertexAttributeDivisor(material.shader.locs[28], 1);

    instances_boid_vy_ID = flLoadVertexBuffer(boid_vy, instances*sizeof(float), instances_boid_vy_ID);
    rlEnableVertexAttribute(material.shader.locs[29]);
    rlSetVertexAttribute(material.shader.locs[29], 1, RL_FLOAT, false, sizeof(float), 0);
    rlSetVertexAttributeDivisor(material.shader.locs[29], 1);

    FrameMarkEnd("Upload");
    FrameMarkStart("End Render");

    // Accumulate internal matrix transform (push/pop) and view matrix
    // NOTE: In this case, model instance transformation must be computed in the shader
    Matrix matModelView = MatrixMultiply(rlGetMatrixTransform(), rlGetMatrixModelview());

    // Try binding vertex array objects (VAO)
    rlEnableVertexArray(mesh.vaoId);

    // Calculate model-view-projection matrix (MVP)
    Matrix matModelViewProjection = MatrixMultiply(matModelView, rlGetMatrixProjection());

    // Send combined model-view-projection matrix to shader
    rlSetUniformMatrix(material.shader.locs[SHADER_LOC_MATRIX_MVP], matModelViewProjection);

    // Draw mesh instanced
    rlDrawVertexArrayInstanced(0, mesh.vertexCount, instances);


    // Unbind all bound texture maps
    for (int i = 0; i < MAX_MATERIAL_MAPS; i++)
    {
        if (material.maps[i].texture.id > 0)
        {
            //DEBUG("UNBINDING TEXTURE")
            // Select current shader texture slot
            rlActiveTextureSlot(i);

            // Disable texture for active slot
            if ((i == MATERIAL_MAP_IRRADIANCE) ||
                (i == MATERIAL_MAP_PREFILTER) ||
                (i == MATERIAL_MAP_CUBEMAP)) rlDisableTextureCubemap();
            else rlDisableTexture();
        }
    }

    FrameMarkEnd("End Render");
}
