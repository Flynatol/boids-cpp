#include "sim/simulation.h"

#include <algorithm>
#include <cstring>
#include <cmath>
#include <immintrin.h>
#include <limits>
#include <vector>

#include "app/app_config.h"
#include "ui/ui.h"

BoidList* boid_list;
BoidMap* boid_map;

namespace {

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
    ZoneScopedN("rebuild_row_copy_task");
    for (int x = 0; x < boid_map->m_xsize; x += 1) {
        write_map_to_list(boid_map->m_xsize * row + x, index_buffer);
    }
}

inline void place_boid(Boid boid_to_place) {
    Boid map_pos = boid_map->get_map_pos_nearest(
        boid_list->m_boid_store->xs[boid_to_place],
        boid_list->m_boid_store->ys[boid_to_place]
    );

    boid_map->safety[map_pos].lock();
        Boid old_head = boid_map->m_boid_map[map_pos];
        boid_map->m_boid_map[map_pos] = boid_to_place;
        boid_list->m_boid_store->index_next[boid_to_place] = old_head;
        boid_list->m_boid_store->depth[boid_to_place] = (old_head != -1) ? boid_list->m_boid_store->depth[old_head] + 1 : 1;
    boid_map->safety[map_pos].unlock();
}

inline void populate_n(const uint32_t start, const uint32_t task_size) {
    ZoneScoped;
    for (int32_t i = start; i < start + task_size; i++) {
        place_boid(i);
    }
}

template <typename T, typename U>
union ExtractVec {
    U vector;
    T data[8];
};

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

inline void update_cell2(const int x, const int y, const Rules* rules, const BoidList* boid_list) {
    const auto world_height = boid_map->m_cell_size * boid_map->m_ysize;
    const auto world_width = boid_map->m_cell_size * boid_map->m_xsize;

    Boid cell_begin = boid_map->m_boid_map[y * boid_map->m_xsize + x];

    if (cell_begin == -1) return;

    const BoidStore* src_store = boid_list->m_boid_store;
    BoidStore* dst_store = boid_list->m_backbuffer;

    const int32_t* depth = src_store->depth;
    Boid cell_end = cell_begin + depth[cell_begin];

    const auto xs = src_store->xs;
    const auto ys = src_store->ys;
    const auto vxs = src_store->vxs;
    const auto vys = src_store->vys;
    const auto homes = src_store->homes;
    const auto ads_vec = _mm256_set1_ps(rules->avoid_distance_squared);
    const auto srs_vec = _mm256_set1_ps(rules->sight_range_squared);
    const auto af_vec = _mm256_set1_ps(rules->avoid_factor);

    Boid near_row_begin[3] = {-1, -1, -1};
    Boid near_row_end[3] = {-1, -1, -1};

    for (int cy = -1; cy <= 1; cy++) {
        Boid row_begin = -1;
        Boid row_end = -1;

        for (int cx = -1; cx <= 1; cx++) {
            const Boid current = boid_map->get_coord(y + cy, x + cx);
            if (current != -1) {
                if (row_begin == -1) row_begin = current;
                row_end = current + depth[current];
            }
        }

        near_row_begin[cy + 1] = row_begin;
        near_row_end[cy + 1] = row_end;
    }

    for (Boid current_boid = cell_begin; current_boid < cell_end; current_boid += 8) {
        auto current_xs_vec = _mm256_load_ps(&xs[current_boid]);
        auto current_ys_vec = _mm256_load_ps(&ys[current_boid]);

        __m256 sep_x_vec = _mm256_set1_ps(0.);
        __m256 sep_y_vec = _mm256_set1_ps(0.);

        __m256 avg_x_vec = _mm256_set1_ps(0.);
        __m256 avg_y_vec = _mm256_set1_ps(0.);

        __m256 avg_vx_vec = _mm256_set1_ps(0.);
        __m256 avg_vy_vec = _mm256_set1_ps(0.);

        __m256i isc = _mm256_set1_epi32(0);

        for (int row_i = 0; row_i < 3; row_i++) {
            const Boid row_begin = near_row_begin[row_i];
            const Boid row_end = near_row_end[row_i];

            for (Boid nearby_boid = row_begin; nearby_boid < row_end; nearby_boid += 8) {
                auto nearby_xs_vec = _mm256_load_ps(&xs[nearby_boid]);
                auto nearby_ys_vec = _mm256_load_ps(&ys[nearby_boid]);

                auto nearby_vxs_vec = _mm256_load_ps(&vxs[nearby_boid]);
                auto nearby_vys_vec = _mm256_load_ps(&vys[nearby_boid]);

                for (int i = 0; i < 8; i++) {
                    const auto xs_delta = _mm256_sub_ps(current_xs_vec, nearby_xs_vec);
                    const auto ys_delta = _mm256_sub_ps(current_ys_vec, nearby_ys_vec);
                    const auto ds_vec = _mm256_add_ps(_mm256_mul_ps(xs_delta, xs_delta), _mm256_mul_ps(ys_delta, ys_delta));

                    const auto srs_mask = _mm256_cmp_ps(ds_vec, srs_vec, _CMP_LT_OS);
                    const auto ads_mask = _mm256_cmp_ps(ds_vec, ads_vec, _CMP_LT_OS);
                    const auto sna_bitmask = _mm256_andnot_ps(ads_mask, srs_mask);
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

                    nearby_xs_vec = _mm256_permutevar8x32_ps(nearby_xs_vec, _mm256_set_epi32(0, 7, 6, 5, 4, 3, 2, 1));
                    nearby_ys_vec = _mm256_permutevar8x32_ps(nearby_ys_vec, _mm256_set_epi32(0, 7, 6, 5, 4, 3, 2, 1));
                    nearby_vxs_vec = _mm256_permutevar8x32_ps(nearby_vxs_vec, _mm256_set_epi32(0, 7, 6, 5, 4, 3, 2, 1));
                    nearby_vys_vec = _mm256_permutevar8x32_ps(nearby_vys_vec, _mm256_set_epi32(0, 7, 6, 5, 4, 3, 2, 1));
                }
            }
        }

        __m256 isc_mask = _mm256_castsi256_ps(_mm256_cmpgt_epi32(isc, _mm256_set1_epi32(0)));

        auto vxs_out = _mm256_load_ps(&vxs[current_boid]);
        auto vys_out = _mm256_load_ps(&vys[current_boid]);

        vxs_out = _mm256_fmadd_ps(sep_x_vec, af_vec, vxs_out);
        vys_out = _mm256_fmadd_ps(sep_y_vec, af_vec, vys_out);

        auto cvt_isc = _mm256_cvtepi32_ps(isc);

#ifdef APPROXIMATE_NEIGHBOR_AVERAGES
        auto inv_isc = _mm256_rcp_ps(cvt_isc);
        inv_isc = _mm256_mul_ps(inv_isc, _mm256_sub_ps(_mm256_set1_ps(2.0f), _mm256_mul_ps(cvt_isc, inv_isc)));
        inv_isc = _mm256_and_ps(inv_isc, isc_mask);

        avg_vx_vec = _mm256_mul_ps(avg_vx_vec, inv_isc);
        avg_vy_vec = _mm256_mul_ps(avg_vy_vec, inv_isc);
#else
        avg_vx_vec = _mm256_div_ps(avg_vx_vec, cvt_isc);
        avg_vy_vec = _mm256_div_ps(avg_vy_vec, cvt_isc);
#endif

        vxs_out = _mm256_fmadd_ps(_mm256_set1_ps(rules->alignment_factor), _mm256_and_ps(isc_mask, _mm256_sub_ps(avg_vx_vec, vxs_out)), vxs_out);
        vys_out = _mm256_fmadd_ps(_mm256_set1_ps(rules->alignment_factor), _mm256_and_ps(isc_mask, _mm256_sub_ps(avg_vy_vec, vys_out)), vys_out);

#ifdef APPROXIMATE_NEIGHBOR_AVERAGES
        avg_x_vec = _mm256_mul_ps(avg_x_vec, inv_isc);
        avg_y_vec = _mm256_mul_ps(avg_y_vec, inv_isc);
#else
        avg_x_vec = _mm256_div_ps(avg_x_vec, cvt_isc);
        avg_y_vec = _mm256_div_ps(avg_y_vec, cvt_isc);
#endif

        vxs_out = _mm256_fmadd_ps(_mm256_set1_ps(rules->cohesion_factor), _mm256_and_ps(isc_mask, _mm256_sub_ps(avg_x_vec, current_xs_vec)), vxs_out);
        vys_out = _mm256_fmadd_ps(_mm256_set1_ps(rules->cohesion_factor), _mm256_and_ps(isc_mask, _mm256_sub_ps(avg_y_vec, current_ys_vec)), vys_out);

        const auto rf = _mm256_set1_ps(rules->edge_factor);
        const auto ew = _mm256_set1_ps(rules->edge_width);

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

        auto too_slow = _mm256_cmp_ps(rspeed_vec, _mm256_set1_ps(1.0 / minspeed), _CMP_GT_OS);
        auto too_fast = _mm256_cmp_ps(rspeed_vec, _mm256_set1_ps(1.0 / maxspeed), _CMP_LT_OS);
        auto too_inf = _mm256_cmp_ps(rspeed_vec, _mm256_set1_ps(std::numeric_limits<float>::max()), _CMP_GT_OS);

        auto multiplers = _mm256_set1_ps(1.0);
        multiplers = _mm256_blendv_ps(multiplers, lspeed_vec, too_slow);
        multiplers = _mm256_blendv_ps(multiplers, ispeed_vec, too_fast);
        multiplers = _mm256_blendv_ps(multiplers, _mm256_set1_ps(1.0), too_inf);

        vxs_out = _mm256_mul_ps(multiplers, vxs_out);
        vys_out = _mm256_mul_ps(multiplers, vys_out);

        const auto homes_vec = _mm256_loadu_si256((const __m256i*) &homes[current_boid]);

        auto home_index_x_vec = _mm256_cvtepi32_ps(_mm256_add_epi32(_mm256_and_si256(homes_vec, _mm256_set1_epi32(15)), _mm256_set1_epi32(1)));
        auto home_index_y_vec = _mm256_cvtepi32_ps(_mm256_add_epi32(_mm256_srlv_epi32(homes_vec, _mm256_set1_epi32(4)), _mm256_set1_epi32(1)));

        auto home_loc_x_vec = _mm256_fmadd_ps(home_index_x_vec, _mm256_set1_ps((float) ((world_width - rules->edge_width * 2) / (16 + 1))), ew);
        auto home_loc_y_vec = _mm256_fmadd_ps(home_index_y_vec, _mm256_set1_ps((float) ((world_height - rules->edge_width * 2) / (9 + 1))), ew);

        auto dx_vec = _mm256_sub_ps(home_loc_x_vec, current_xs_vec);
        auto dy_vec = _mm256_sub_ps(home_loc_y_vec, current_ys_vec);

        vxs_out = _mm256_fmadd_ps(dx_vec, _mm256_set1_ps(rules->homing), vxs_out);
        vys_out = _mm256_fmadd_ps(dy_vec, _mm256_set1_ps(rules->homing), vys_out);

        __m256 temp = _mm256_add_ps(_mm256_set1_ps((float) current_boid), _mm256_set_ps(7., 6., 5., 4., 3., 2., 1., 0.));
        __m256 out_mask = _mm256_cmp_ps(temp, _mm256_set1_ps(cell_end), _CMP_LT_OS);

        auto xs_out = _mm256_add_ps(current_xs_vec, vxs_out);
        auto ys_out = _mm256_add_ps(current_ys_vec, vys_out);

        if (current_boid + 8 <= cell_end) {
            _mm256_storeu_ps(&dst_store->xs[current_boid], xs_out);
            _mm256_storeu_ps(&dst_store->ys[current_boid], ys_out);
            _mm256_storeu_ps(&dst_store->vxs[current_boid], vxs_out);
            _mm256_storeu_ps(&dst_store->vys[current_boid], vys_out);
            _mm256_storeu_si256((__m256i*) &dst_store->homes[current_boid], homes_vec);
        } else {
            const auto out_mask_i = _mm256_castps_si256(out_mask);
            _mm256_maskstore_ps(&dst_store->xs[current_boid], out_mask_i, xs_out);
            _mm256_maskstore_ps(&dst_store->ys[current_boid], out_mask_i, ys_out);
            _mm256_maskstore_ps(&dst_store->vxs[current_boid], out_mask_i, vxs_out);
            _mm256_maskstore_ps(&dst_store->vys[current_boid], out_mask_i, vys_out);
            _mm256_maskstore_epi32((int*) &dst_store->homes[current_boid], out_mask_i, homes_vec);
        }
    }
}

inline void row_runner(const int y, const Rules* rules) {
    ZoneScoped;
    static const bool use_avx512 = boids_cpu_supports_avx512();

    for (int x = 0; x < boid_map->m_xsize; x++) {
        if (use_avx512) {
            update_cell2_avx512(x, y, rules, boid_list);
        } else {
            update_cell2(x, y, rules, boid_list);
        }
    }
}

void swap_buffers() {
    ZoneScoped;
    auto* temp = boid_list->m_boid_store;
    boid_list->m_boid_store = boid_list->m_backbuffer;
    boid_list->m_backbuffer = temp;
}

} // namespace

bool BoidTaskSpec::execute(BoidTaskMaster*, Task& current_task) {
    switch (current_task.task_type) {
        case TaskType::ROW_RUNNER: {
            auto s = current_task.arg.row_runner;
            row_runner(s->y, s->rules);
            break;
        }
        case TaskType::REBUILD: {
            auto s = current_task.arg.rebuild;
            write_row_to_list(s->y, s->index_buffer);
            break;
        }
        case TaskType::POPULATE: {
            auto s = current_task.arg.populate;
            populate_n(s->start, s->task_size);
            break;
        }
        case TaskType::STOP:
            return true;
    }

    return false;
}

void populate_map2(BoidTaskMaster* task_master, TaskSync* task_monitor, populate_args* arg_list, uint32_t num_tasks) {
    memset(boid_map->m_boid_map, -1, sizeof(Boid) * boid_map->m_xsize * boid_map->m_ysize);
    task_master->lock.lock();
    uint32_t tasks_added = 0;

    for (int y = 0; y < num_tasks; y++) {
        task_master->ts_task_buffer.push_back(
            Task {
                .task_type = TaskType::POPULATE,
                .arg = { .populate = &arg_list[y] },
                .sync = task_monitor,
            }
        );

        tasks_added++;
    }

    task_monitor->task_counter += tasks_added;
    task_master->lock.unlock();
    if (task_master->sleep_enabled) {
        task_master->task_semaphore.release(tasks_added);
    }
}

void update_boids2(row_runner_args* arg_list, BoidTaskMaster* task_master, TaskSync* task_monitor) {
    task_master->lock.lock();

    uint32_t tasks_added = 0;

    for (int y = 0; y < boid_map->m_ysize; y++) {
        task_master->ts_task_buffer.push_back(
            Task {
                .task_type = TaskType::ROW_RUNNER,
                .arg = { .row_runner = &arg_list[y] },
                .sync = task_monitor,
                .on_complete = nullptr,
            }
        );

        tasks_added++;
    }

    task_monitor->tc_lock.lock();
    task_monitor->task_counter += tasks_added;
    task_monitor->tc_lock.unlock();

    task_master->lock.unlock();
    if (task_master->sleep_enabled) {
        task_master->task_semaphore.release(tasks_added);
    }
}

namespace {

void rebuild_list2(rebuild_args* arg_list, BoidTaskMaster* task_master) {
    ZoneScopedN("rebuild_list2");
    swap_buffers();

    const uint32_t boid_count = boid_list->m_size;
    const uint32_t num_cells = boid_map->m_xsize * boid_map->m_ysize;

    constexpr uint32_t task_size = 10000;
    const uint32_t populate_task_count = (boid_count + (task_size - 1)) / task_size;
    static std::vector<populate_args> populate_arg_store;
    static std::vector<Task> populate_task_store;
    static TaskSync populate_sync;

    const bool rebuild_populate_tasks = populate_arg_store.size() != populate_task_count || populate_task_store.size() != populate_task_count;
    if (rebuild_populate_tasks) {
        populate_arg_store.resize(populate_task_count);
        populate_task_store.resize(populate_task_count);
    }

    if (rebuild_populate_tasks) {
        for (uint32_t i = 0; i < populate_task_count; i++) {
            populate_arg_store[i] = populate_args {
                .start = i * task_size,
                .task_size = std::min(task_size, boid_count - (i * task_size)),
                .rebuild_args = arg_list,
                .num_tasks = populate_task_count,
            };
            populate_task_store[i] = Task {
                .task_type = TaskType::POPULATE,
                .arg = { .populate = &populate_arg_store[i] },
                .sync = &populate_sync,
                .on_complete = nullptr,
            };
        }
    }
    {
        ZoneScopedN("rebuild_row_memset");
        memset(boid_map->m_boid_map, -1, sizeof(Boid) * boid_map->m_xsize * boid_map->m_ysize);
    }
    {
        ZoneScopedN("rebuild_row_populate_enqueue");
        task_master->lock.lock();
        task_master->ts_task_buffer.push_all_back(populate_task_store.data(), populate_task_count);
        populate_sync.task_counter += populate_task_count;
        task_master->lock.unlock();
        if (task_master->sleep_enabled) {
            task_master->task_semaphore.release(populate_task_count);
        }
    }
    {
        ZoneScopedN("rebuild_row_populate_wait");
        populate_sync.wait();
    }

    Boid* cell_offsets = arg_list->index_buffer;
    Boid* cell_counts = boid_map->m_index_buffer;
    const Boid* map = boid_map->m_boid_map;
    const int32_t* depth = boid_list->m_boid_store->depth;

    {
        ZoneScopedN("rebuild_row_prefix_offsets");
        Boid running = 0;
        for (uint32_t cell = 0; cell < num_cells; cell++) {
            const Boid current = map[cell];
            const Boid count = (current != -1) ? depth[current] : 0;
            cell_offsets[cell] = running;
            cell_counts[cell] = count;
            running += count;
        }
    }

    TaskSync row_sync;
    uint32_t row_tasks_added = 0;

    {
        ZoneScopedN("rebuild_row_copy_enqueue");
        task_master->lock.lock();
        for (int y = 0; y < boid_map->m_ysize; y++) {
            task_master->ts_task_buffer.push_back(
                Task {
                    .task_type = TaskType::REBUILD,
                    .arg = { .rebuild = &arg_list[y] },
                    .sync = &row_sync,
                    .on_complete = nullptr,
                }
            );
            row_tasks_added++;
        }
        row_sync.task_counter += row_tasks_added;
        task_master->lock.unlock();
        if (task_master->sleep_enabled) {
            task_master->task_semaphore.release(row_tasks_added);
        }
    }

    {
        ZoneScopedN("rebuild_row_copy_wait");
        row_sync.wait();
    }

    swap_buffers();
}

} // namespace

void rebuild_list2(rebuild_args* arg_list, BoidTaskMaster* task_master, TaskSync* task_monitor) {
    ZoneScoped;
    rebuild_list2(arg_list, task_master);

    if (task_monitor != nullptr) {
        task_monitor->task_counter = 0;
    }
}
