#include <iostream>
#include <raylib.h>
#include <raymath.h>
#include <rlgl.h>
#include <cstdint>
#include <array>
#include <vector>
#include <format>
#include <cstring> 
#include <ctime>
#include <random>
#include <chrono>
#include <immintrin.h>
#include <fstream>

#define NO_FONT_AWESOME

#include "boidlist.h"
#include "boidmap.h"
#include "ui.h"
#include <thread>
#include <future>

std::ofstream myfile;

const float TRIANGLE_SIZE = 5.f;
const uint16_t SIGHT_RANGE = 100;

#define FRAME_RATE_LIMIT 165
#define WORLD_SIZE_MULT 25
//#define BOID_DENSITY_MAGIC_NUMBER todo
#define NUM_THREADS 15
#define USE_MULTICORE 

static const Vector2 triangle[3] = {
    Vector2 {0.f, 2 * TRIANGLE_SIZE},
    Vector2 {-TRIANGLE_SIZE, -2 * TRIANGLE_SIZE},
    Vector2 {TRIANGLE_SIZE, -2 * TRIANGLE_SIZE}
};

//#define DEBUG(...) myfile << TextFormat(__VA_ARGS__) << '\n';
//#define DEBUG(...) ;
#define DEBUG(...) TraceLog(LOG_DEBUG, TextFormat(__VA_ARGS__));
//#define DEBUG(...)



typedef int32_t Boid;

struct PerfMonitor {
    int tick_counter = 0;
    float tps;
    float rolling_average;
} typedef PerfMonitor;

void DrawMeshInstanced2(Mesh mesh, Material material, int instances, float *boid_x, float *boid_y, float *boid_vx, float *boid_vy);

void write_map_to_list(int i, const Boid* index_buffer, const BoidList *boid_list, const BoidMap *boid_map) {
    auto main_buffer = boid_list->m_boid_store;
    auto back_buffer = boid_list->m_backbuffer;

    int index = index_buffer[i];
    Boid current = boid_map->get_absolute(i);

    if (current != -1) {
        boid_map->m_boid_map[i] = index;
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

//TODO rewrite this s.t. it writes boids in reverse order 
void write_map_to_list_back(int i, const Boid* index_buffer, const BoidList *boid_list, const BoidMap *boid_map) {
    auto main_buffer = boid_list->m_boid_store;
    auto back_buffer = boid_list->m_backbuffer;

    int index = index_buffer[i];
    Boid current = boid_map->get_absolute(i);

    if (current != -1) {
        boid_map->m_boid_map[i] = index;
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

void block_writer(int thread_start_pos, const Boid* index_buffer, const BoidList *boid_list, const BoidMap *boid_map) {
    for (int i = 0; i < (boid_map->m_xsize * boid_map->m_ysize) / NUM_THREADS; i++) {
        write_map_to_list(thread_start_pos + i, index_buffer, boid_list, boid_map);
    }
}

void jump_writer(int thread_start_pos, const Boid* index_buffer, const BoidList *boid_list, const BoidMap *boid_map) {
    for (int i = thread_start_pos; i < (boid_map->m_xsize * boid_map->m_ysize); i += NUM_THREADS) {
        write_map_to_list(i, index_buffer, boid_list, boid_map);
    }
}

//std::future<void> render_task
void rebuild_list(BoidList& boid_list, const BoidMap& boid_map) {
    auto t_start = std::chrono::high_resolution_clock::now();
    BoidStore *main_buffer = boid_list.m_boid_store;
    BoidStore *back_buffer = boid_list.m_backbuffer;
    /*
    int index = 0;
    //Go through every cell in the map (find every head node)
    for (int i = 0; i < boid_map.m_xsize * boid_map.m_ysize; i++) {
        //Set current to the head node of the current cell (i)
        Boid current = boid_map.get_absolute(i);
        
        //Then update the position in the map to the new position of this head node
        if (current != -1) {
            boid_map.m_boid_map[i] = index;
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

    boid_list.m_boid_store = back_buffer;
    boid_list.m_backbuffer = main_buffer;

    */
    // NEW PLAN
    // ONE PASS to store index to write to in new list for each cell
    // THEN USE MT to update the list from many points at once.

    //Boid index_buffer[boid_map.m_xsize * boid_map.m_ysize];

    //An array that tells us the index that that the looked up boid should be written to in the new array
    Boid *index_buffer = (Boid *) malloc(boid_map.m_xsize * boid_map.m_ysize * sizeof(Boid)); //Weirdly this seems to be the fastest of the following options
    //Boid *index_buffer = boid_map.m_index_buffer;
    //Boid index_buffer[boid_map.m_xsize * boid_map.m_ysize];

    int counter = 0;
    for (int i = 0; i < boid_map.m_xsize * boid_map.m_ysize; i++) {
        Boid current = boid_map.m_boid_map[i];
        index_buffer[i] = counter;
        counter += (current != -1) * main_buffer->depth[current];   
    }

    auto t_mid = std::chrono::high_resolution_clock::now();

    //std::vector<std::future<void>> pool;
    //pool.resize(NUM_THREADS);

    std::thread threads[NUM_THREADS];
    
    for (int i = 0; i < NUM_THREADS; i++) {
        int num = i * ((boid_map.m_xsize * boid_map.m_ysize) / NUM_THREADS);
        threads[i] = std::thread(block_writer, num, (Boid *) index_buffer, &boid_list, &boid_map);
        //pool[i] = std::async(std::launch::async, block_writer, num, (Boid *) index_buffer, &boid_list, &boid_map);
    }
    
    for (int i = 0; i < NUM_THREADS; i++) {
        threads[i].join();
        //pool[i].wait();
    }
    
    free(index_buffer);

    boid_list.m_boid_store = back_buffer;
    boid_list.m_backbuffer = main_buffer;

    auto t_end = std::chrono::high_resolution_clock::now();
}

inline void place_boid(const BoidMap& map, const BoidList& boid_list, Boid boid_to_place) {
    Boid map_pos = map.get_map_pos_nearest(boid_list.m_boid_store->xs[boid_to_place], boid_list.m_boid_store->ys[boid_to_place]);
    Boid old_head = map.m_boid_map[map_pos];
    map.m_boid_map[map_pos] = boid_to_place;
    boid_list.m_boid_store->index_next[boid_to_place] = old_head;
    boid_list.m_boid_store->depth[boid_to_place] = (old_head != -1) ? boid_list.m_boid_store->depth[old_head] + 1 : 1;
}

void block_populate(int thread_start_pos, const BoidList *boid_list, const BoidMap *boid_map) {
    for (int i = 0; i < (boid_list->m_size) / NUM_THREADS; i++) {
        place_boid(*boid_map, *boid_list, thread_start_pos + i);
    }
}


void populate_map(BoidList& boid_list, BoidMap& boid_map) {
    
    memset(boid_map.m_boid_map, -1, sizeof(Boid) * boid_map.m_xsize * boid_map.m_ysize);

    //TODO: slight problem here is that we are WILL reverse the memory positions in each cell each update.
    //Maybe we can fix this writing into the new list in reverse order in the rebuild list function (Use the depth to work out correct positions).
    //Or we can fix this by building our linked list s.t. the first boid we find in a cell will stay as the head.
    //This would be performance intensive.
    
    /*
    for (Boid boid_to_place = 0; boid_to_place < boid_list.m_size; boid_to_place++) {
        Boid map_pos = boid_map.get_map_pos_nearest(boid_list.m_boid_store->xs[boid_to_place], boid_list.m_boid_store->ys[boid_to_place]);
        Boid old_head = boid_map.m_boid_map[map_pos];
        boid_map.m_boid_map[map_pos] = boid_to_place;
        boid_list.m_boid_store->index_next[boid_to_place] = old_head;
        boid_list.m_boid_store->depth[boid_to_place] = (old_head != -1) ? boid_list.m_boid_store->depth[old_head] + 1 : 1;
    }
    */

    // To parallelize this we could use an array of mutexs to represent to the head of each map cell
    // If we put distant rows on each thread unlikely to have waiting.

    //Alternatively as no boid can move more than one grid square in a tick we could work in different rows spaced out by a couple rows
    //Alternatively alternatively we can just space out a lot and hope. (this probably isn't as terrible as it sounds) 

    //std::thread threads[NUM_THREADS];
    std::vector<std::future<void>> pool;
    pool.resize(NUM_THREADS);
    
    for (int i = 0; i < NUM_THREADS; i++) {
        int num = i * (boid_list.m_size / NUM_THREADS);
        pool[i] = std::async(std::launch::async, block_populate, num, &boid_list, &boid_map);
    }
    
    for (int i = 0; i < NUM_THREADS; i++) {
        pool[i].wait();
    }
    
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
    __m256i fliptemp = (__m256i) _mm256_permute2f128_ps((__m256) temp, (__m256) temp, 1);
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

inline void update_cell(const BoidMap& map, const int x, const int y, const Rules& rules, Boid selected_boid, const BoidList& boid_list) {
    Boid cell_to_update = map.get_coord(y, x);
    if (cell_to_update == -1 ) return;

    const auto xs = boid_list.m_boid_store->xs;
    const auto ys = boid_list.m_boid_store->ys;
    const auto vxs = boid_list.m_boid_store->vxs;
    const auto vys = boid_list.m_boid_store->vys;
    
    Boid cell_begin = cell_to_update;
    Boid cell_end = cell_begin + boid_list.m_boid_store->depth[cell_begin];

    for (int cy = -1; cy <= 1; cy++) {
        //Work out the memory range we're calculating on
        Boid row_begin = -1;
        Boid row_end = -1;

        //Todo check if this would be faster branchless -- probably not worth the readability
        for (int cx = -1; cx <= 1; cx++) {
            Boid current = map.get_coord(y + cy, x + cx);
            if (current != -1) {
                if (row_begin == -1) row_begin = current;
                row_end = current + boid_list.m_boid_store->depth[current];
            }
        }
        
        //Check if any cells have work for us
        if (row_begin != -1) { 
            //For each boid in current cell
            for (Boid current_boid = cell_begin; current_boid < cell_end; current_boid++) {
                float sep_x = 0, sep_y = 0;

                //Variables for tracking aligment
                float avg_vx = 0, avg_vy = 0;

                //Variables for tracking cohesion
                float avg_x = 0, avg_y = 0;
                
                //Remember to mask!
                __m256 sep_x_vec = _mm256_set1_ps(0.);
                __m256 sep_y_vec = _mm256_set1_ps(0.);
                __m256 avg_x_vec = _mm256_set1_ps(0.);
                __m256 avg_y_vec = _mm256_set1_ps(0.);
                __m256 avg_vx_vec = _mm256_set1_ps(0.);
                __m256 avg_vy_vec = _mm256_set1_ps(0.);

                uint_fast16_t in_sight_counter = 0; 
                __m256i isc = _mm256_set1_epi32(0);

                __m256i read_mask;
                
                /* Keeping this here as an explaination for the SIMD code below
                //Check against each boid in the row currently being processed
                for (Boid nearby_boid = row_end; nearby_boid < row_end; nearby_boid++) {

                    int_fast32_t dist_squared = (xs[current_boid] - xs[nearby_boid]) * (xs[current_boid] - xs[nearby_boid]) + (ys[current_boid] - ys[nearby_boid]) * (ys[current_boid] - ys[nearby_boid]);

                    const bool pc_srs = dist_squared < rules.sight_range_squared;

                    const bool pc_ads = dist_squared < rules.avoid_distance_squared;

                    const bool in_sight_but_not_avoid = !pc_ads & pc_srs;
                    
                    sep_x += (pc_ads) * (xs[current_boid] - xs[nearby_boid]) * (rules.avoid_distance_squared - dist_squared) * (rules.avoid_distance_squared - dist_squared);
                    sep_y += (pc_ads) * (ys[current_boid] - ys[nearby_boid]) * (rules.avoid_distance_squared - dist_squared) * (rules.avoid_distance_squared - dist_squared);
                    
                    avg_vx += in_sight_but_not_avoid * vxs[nearby_boid];
                    avg_vy += in_sight_but_not_avoid * vys[nearby_boid];

                    avg_x  += in_sight_but_not_avoid * xs[nearby_boid];
                    avg_y  += in_sight_but_not_avoid * ys[nearby_boid];

                    in_sight_counter += in_sight_but_not_avoid;
                }
                */

                const auto ads_vec = _mm256_set1_ps(rules.avoid_distance_squared);
                const auto srs_vec = _mm256_set1_ps(rules.sight_range_squared);

                for (Boid nearby_boid = row_begin; nearby_boid < row_end; nearby_boid += 8) {
                    //int bytes_left = row_end - nearby_boid; 
                    //read_mask = _mm256_set_epi32((bytes_left > 7) * 0xFFFFFFFF, (bytes_left > 6) * 0xFFFFFFFF, (bytes_left > 5) * 0xFFFFFFFF, (bytes_left > 4) * 0xFFFFFFFF, (bytes_left > 3) * 0xFFFFFFFF, (bytes_left > 2) * 0xFFFFFFFF, (bytes_left > 1) * 0xFFFFFFFF, 0xFFFFFFFF);

                    //read_mask = _mm256_set1_epi32(0xFFFFFFFF);
                    const auto nearby_xs_vec = _mm256_loadu_ps(&xs[nearby_boid]);
                    const auto nearby_ys_vec = _mm256_loadu_ps(&ys[nearby_boid]);

                    const auto nearby_vxs_vec = _mm256_loadu_ps(&vxs[nearby_boid]);
                    const auto nearby_vys_vec = _mm256_loadu_ps(&vys[nearby_boid]);

                    const auto current_xs_vec = _mm256_set1_ps(xs[current_boid]);
                    const auto current_ys_vec = _mm256_set1_ps(ys[current_boid]);              
                    
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

                    //__m256i test = _mm256_add_epi32(isc, _mm256_cvtps_epi32(sna_fpmask));
                    //isc = _mm256_add_epi32(isc, (__m256i) _mm256_and_ps((__m256) read_mask, (__m256) _mm256_cvtps_epi32(sna_fpmask)));
                    isc = _mm256_add_epi32(isc, _mm256_cvtps_epi32(sna_fpmask));
                }
                
                ExtractVec<uint32_t, __m256i> extractor_i = {vector_sum(isc)};
                in_sight_counter = extractor_i.data[0];
                //if (!(extractor_i.data[0] == in_sight_counter)) DEBUG("%d, %d", extractor_i.data[0], in_sight_counter);

                ExtractVec<float, __m256> extractor_f = {vector_sum(avg_vx_vec)};
                //if (!(abs(avg_vx - extractor_f.data[0]) < 0.001)) DEBUG("%f %f %d", extractor_f.data[0], avg_vx, abs(avg_vx - extractor_f.data[0]) < 0.011);
                avg_vx = extractor_f.data[0];
                extractor_f.vector = vector_sum(avg_vy_vec);
                avg_vy = extractor_f.data[0];

                extractor_f.vector = vector_sum(avg_x_vec);
                avg_x = extractor_f.data[0];
                extractor_f.vector = vector_sum(avg_y_vec);
                avg_y = extractor_f.data[0];

                extractor_f.vector = vector_sum(sep_x_vec);
                sep_x = extractor_f.data[0];
                extractor_f.vector = vector_sum(sep_y_vec);
                sep_y = extractor_f.data[0];
                
                //TraceLog(LOG_DEBUG, TextFormat("vector saw: %d --- actual %d", isc_out.data[0], in_sight_counter));

                //Avoidance
                vxs[current_boid] += sep_x * rules.avoid_factor;
                vys[current_boid] += sep_y * rules.avoid_factor;

                //When we are using SIMD we can use masks to avoid all the in sight counters
                //Alignment
                avg_vx = avg_vx/(in_sight_counter + (in_sight_counter == 0));
                avg_vy = avg_vy/(in_sight_counter + (in_sight_counter == 0));

                vxs[current_boid] += (in_sight_counter > 0) * (avg_vx - vxs[current_boid]) * rules.alignment_factor;
                vys[current_boid] += (in_sight_counter > 0) * (avg_vy - vys[current_boid]) * rules.alignment_factor;

                //Cohesion
                avg_x = avg_x/(in_sight_counter + (in_sight_counter == 0));
                avg_y = avg_y/(in_sight_counter + (in_sight_counter == 0));

                vxs[current_boid] += (in_sight_counter > 0) * (avg_x - xs[current_boid]) * rules.cohesion_factor;
                vys[current_boid] += (in_sight_counter > 0) * (avg_y - ys[current_boid]) * rules.cohesion_factor;
            }
        }       
    }
}

/*
#define CALC_AVG                       \
        { \
        const auto xs_delta = _mm256_sub_ps(current_xs_vec, nearby_xs_vec);\
        const auto ys_delta = _mm256_sub_ps(current_ys_vec, nearby_ys_vec);\
        const auto ds_vec = _mm256_add_ps(_mm256_mul_ps(xs_delta, xs_delta), _mm256_mul_ps(ys_delta, ys_delta));\
        const auto srs_mask = _mm256_cmp_ps(ds_vec, srs_vec, _CMP_LT_OS);\         
        const auto ads_mask = _mm256_cmp_ps(ds_vec, ads_vec, _CMP_LT_OS);\           
        const auto sna_bitmask = _mm256_andnot_ps(ads_mask, srs_mask); \            
        const auto sna_fpmask = _mm256_and_ps(sna_bitmask, _mm256_set1_ps(1.)); \ 
        const auto ads_take_ds = _mm256_sub_ps(ads_vec, ds_vec); \
        const auto ads_take_ds_sqr = _mm256_mul_ps(ads_take_ds, ads_take_ds); \
        sep_x_vec = _mm256_add_ps(sep_x_vec, _mm256_and_ps(ads_mask, _mm256_mul_ps(xs_delta, ads_take_ds_sqr))); \
        sep_y_vec = _mm256_add_ps(sep_y_vec, _mm256_and_ps(ads_mask, _mm256_mul_ps(ys_delta, ads_take_ds_sqr))); \
        avg_vx_vec = _mm256_fmadd_ps(sna_fpmask, nearby_vxs_vec, avg_vx_vec); \
        avg_vy_vec = _mm256_fmadd_ps(sna_fpmask, nearby_vys_vec, avg_vy_vec); \
        avg_x_vec = _mm256_fmadd_ps(sna_fpmask, nearby_xs_vec, avg_x_vec); \ 
        avg_y_vec = _mm256_fmadd_ps(sna_fpmask, nearby_ys_vec, avg_y_vec); \
        isc = _mm256_add_epi32(isc, _mm256_cvtps_epi32(sna_fpmask)); \
        }
*/

/*
inline void calc_avg(__m256& current_xs_vec, __m256& current_ys_vec, __m256& nearby_xs_vec, __m256& nearby_ys_vec, __m256& srs_vec, __m256& ads_vec, __m256& sep_x_vec, __m256& sep_y_vec) {
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
}
*/
inline void update_cell2(const BoidMap *map, const int x, const int y, const Rules *rules, const BoidList *boid_list) {
    const auto world_height = map->m_cell_size * map->m_ysize;
    const auto world_width = map->m_cell_size * map->m_xsize;

    const Boid cell_to_update = map->get_coord(y, x);
    if (cell_to_update == -1 ) return;

    const auto xs = boid_list->m_boid_store->xs;
    const auto ys = boid_list->m_boid_store->ys;
    const auto vxs = boid_list->m_boid_store->vxs;
    const auto vys = boid_list->m_boid_store->vys;

    const auto ads_vec = _mm256_set1_ps(rules->avoid_distance_squared);
    const auto srs_vec = _mm256_set1_ps(rules->sight_range_squared);
    const auto af_vec  = _mm256_set1_ps(rules->avoid_factor);
    
    Boid cell_begin = cell_to_update;
    Boid cell_end = cell_begin + boid_list->m_boid_store->depth[cell_begin];

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
                Boid current = map->get_coord(y + cy, x + cx);
                if (current != -1) {
                    if (row_begin == -1) row_begin = current;
                    row_end = current + boid_list->m_boid_store->depth[current];
                }
            }

            // For each block of boids in current row
            for (Boid nearby_boid = row_begin; nearby_boid < row_end; nearby_boid += 8) {
                auto nearby_xs_vec = _mm256_loadu_ps(&xs[nearby_boid]);
                auto nearby_ys_vec = _mm256_loadu_ps(&ys[nearby_boid]);

                auto nearby_vxs_vec = _mm256_loadu_ps(&vxs[nearby_boid]);
                auto nearby_vys_vec = _mm256_loadu_ps(&vys[nearby_boid]);

                auto test_vec = _mm256_set_ps(0.f, 1.f, 2.f, 3.f, 4.f, 5.f, 6.f, 7.f);
                //DEBUG("Test started");
                //For each of the 8 permutations do some work TODO unroll this and use the faster in lane permute most of the time
                
                for (int i = 0; i < 8; i++) {
                    //debug_vector<float>(test_vec, "%f");
                    //test_vec = _mm256_permutevar8x32_ps(test_vec, _mm256_set_epi32(7, 6, 5, 4, 3, 2, 1, 0));
                    //test_vec = _mm256_shuffle_ps(test_vec, test_vec, _MM_SHUFFLE(0, 3, 2, 1));
                    
                
                    //Do some stuff                    
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

                    //__m256i test = _mm256_add_epi32(isc, _mm256_cvtps_epi32(sna_fpmask));
                    //isc = _mm256_add_epi32(isc, (__m256i) _mm256_and_ps((__m256) read_mask, (__m256) _mm256_cvtps_epi32(sna_fpmask)));
                    isc = _mm256_add_epi32(isc, _mm256_cvtps_epi32(sna_fpmask));
                    
                    
                    //Then permute
                    //test_vec        = _mm256_permutevar8x32_ps(test_vec, _mm256_set_epi32(0, 7, 6, 5, 4, 3, 2, 1));
                    nearby_xs_vec   = _mm256_permutevar8x32_ps(nearby_xs_vec, _mm256_set_epi32(0, 7, 6, 5, 4, 3, 2, 1));
                    nearby_ys_vec   = _mm256_permutevar8x32_ps(nearby_ys_vec, _mm256_set_epi32(0, 7, 6, 5, 4, 3, 2, 1));
                    nearby_vxs_vec  = _mm256_permutevar8x32_ps(nearby_vxs_vec, _mm256_set_epi32(0, 7, 6, 5, 4, 3, 2, 1));
                    nearby_vys_vec  = _mm256_permutevar8x32_ps(nearby_vys_vec, _mm256_set_epi32(0, 7, 6, 5, 4, 3, 2, 1));
                }
                /*
                
                //Do stuff
                CALC_AVG
                
                //Then (permute + do stuff) x 3
                for (int i = 4; i > 0; i--) {
                    nearby_xs_vec   = _mm256_shuffle_ps(nearby_xs_vec, nearby_xs_vec, _MM_SHUFFLE(0, 3, 2, 1));
                    nearby_ys_vec   = _mm256_shuffle_ps(nearby_ys_vec, nearby_ys_vec, _MM_SHUFFLE(0, 3, 2, 1));
                    nearby_vxs_vec  = _mm256_shuffle_ps(nearby_vxs_vec, nearby_vxs_vec, _MM_SHUFFLE(0, 3, 2, 1));
                    nearby_vys_vec  = _mm256_shuffle_ps(nearby_vys_vec, nearby_vys_vec, _MM_SHUFFLE(0, 3, 2, 1));

                    CALC_AVG
                }
                //Big flip TODO
                nearby_xs_vec   = _mm256_permutevar8x32_ps(nearby_xs_vec, _mm256_set_epi32(3, 2, 1, 0, 7, 6, 5, 4));
                nearby_ys_vec   = _mm256_permutevar8x32_ps(nearby_ys_vec, _mm256_set_epi32(3, 2, 1, 0, 7, 6, 5, 4));
                nearby_vxs_vec  = _mm256_permutevar8x32_ps(nearby_vxs_vec, _mm256_set_epi32(3, 2, 1, 0, 7, 6, 5, 4));
                nearby_vys_vec  = _mm256_permutevar8x32_ps(nearby_vys_vec, _mm256_set_epi32(3, 2, 1, 0, 7, 6, 5, 4));

                
                //Do stuff

                CALC_AVG

                //Then (permute + do stuff) x 3
                for (int i = 4; i > 0; i--) {
                    nearby_xs_vec   = _mm256_shuffle_ps(nearby_xs_vec, nearby_xs_vec, _MM_SHUFFLE(0, 3, 2, 1));
                    nearby_ys_vec   = _mm256_shuffle_ps(nearby_ys_vec, nearby_ys_vec, _MM_SHUFFLE(0, 3, 2, 1));
                    nearby_vxs_vec  = _mm256_shuffle_ps(nearby_vxs_vec, nearby_vxs_vec, _MM_SHUFFLE(0, 3, 2, 1));
                    nearby_vys_vec  = _mm256_shuffle_ps(nearby_vys_vec, nearby_vys_vec, _MM_SHUFFLE(0, 3, 2, 1));

                    CALC_AVG
                }
                */
                
            }
        }

        //Write out the data from tracking vecs
        //I should probably change the type of ISC
        __m256 isc_mask = (__m256) _mm256_cmpgt_epi32(isc, _mm256_set1_epi32(0));

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
        

        //Apply homing

        //TODO

        //Update Xs and Ys
        /*
        float speed = sqrtf((vxs[boid]*vxs[boid]) + (vys[boid]*vys[boid]));
        
        
        //speed = speed + (!speed);
        
        auto const minspeed = 3.0f;
        auto const maxspeed = 4.0f;

        float ispeed = maxspeed/speed;
        float lspeed = minspeed/speed;

        vxs[boid] = (speed <= maxspeed) * vxs[boid] + (speed > maxspeed) * vxs[boid] * ispeed;
        vys[boid] = (speed <= maxspeed) * vys[boid] + (speed > maxspeed) * vys[boid] * ispeed;

        vxs[boid] = (speed >= minspeed) * vxs[boid] + (speed < minspeed) * vxs[boid] * lspeed;
        vys[boid] = (speed >= minspeed) * vys[boid] + (speed < minspeed) * vys[boid] * lspeed;

        //if (boid < 8) DEBUG("xs[b]: %f", ys[boid] + vys[boid]*ispeed);
        
        xs[boid] += vxs[boid];
        ys[boid] += vys[boid];        
        */

        
        auto const minspeed = 3.0f;
        auto const maxspeed = 4.0f;

        auto rspeed_vec = _mm256_rsqrt_ps(_mm256_add_ps(_mm256_mul_ps(vxs_out, vxs_out), _mm256_mul_ps(vys_out, vys_out)));

        auto ispeed_vec = _mm256_mul_ps(rspeed_vec, _mm256_set1_ps(maxspeed));
        auto lspeed_vec = _mm256_mul_ps(rspeed_vec, _mm256_set1_ps(minspeed));

        auto too_slow = _mm256_cmp_ps(rspeed_vec, _mm256_set1_ps(1.0/minspeed), _CMP_GT_OS);
        auto too_fast = _mm256_cmp_ps(rspeed_vec, _mm256_set1_ps(1.0/maxspeed), _CMP_LT_OS);
        auto too_inf = _mm256_cmp_ps(rspeed_vec, _mm256_set1_ps(__FLT_MAX__), _CMP_GT_OS);

        auto multiplers = _mm256_set1_ps(1.0);
        multiplers = _mm256_blendv_ps(multiplers, lspeed_vec, too_slow);
        multiplers = _mm256_blendv_ps(multiplers, ispeed_vec, too_fast);
        multiplers = _mm256_blendv_ps(multiplers, _mm256_set1_ps(1.0), too_inf);

        vxs_out = _mm256_mul_ps(multiplers, vxs_out);
        vys_out = _mm256_mul_ps(multiplers, vys_out);

        //vxs_out = vxs_out_temp;
        //vys_out = vys_out_temp;

        //Maybe we should mask here to avoid messing up data in the next cell.
        //But generating that mask is somewhat expensive
        //current_xs_vec = _mm256_add_ps(current_xs_vec, vxs_out);
        //current_ys_vec = _mm256_add_ps(current_ys_vec, vys_out);
        

        //_mm256_storeu_ps(&xs[current_boid], current_xs_vec);
        //_mm256_storeu_ps(&ys[current_boid], current_ys_vec);

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

inline void update_non_interacting(const BoidMap& boid_map, const Rules& rules, const BoidList& boid_list) {
    const auto xs = boid_list.m_boid_store->xs;
    const auto ys = boid_list.m_boid_store->ys;
    const auto vxs = boid_list.m_boid_store->vxs;
    const auto vys = boid_list.m_boid_store->vys;
    const auto homes = boid_list.m_boid_store->homes;

    const auto world_height = boid_map.m_cell_size * boid_map.m_ysize;
    const auto world_width = boid_map.m_cell_size * boid_map.m_xsize;

    /*
    //Moving rand to it's own loop to avoid having to vectorize rand for now. (Might disable rand)
    for (Boid boid = 0; boid < boid_list.m_size; boid++) { 
        //Apply some randomness
        float rx = static_cast <float> (rand()) / (static_cast <float> (RAND_MAX/2)) - 1;
        float ry = static_cast <float> (rand()) / (static_cast <float> (RAND_MAX/2)) - 1;

        vxs[boid] += rx * rules.rand;
        vys[boid] += ry * rules.rand;
    } 
    */

    //Easy to SIMD but currently only using a few ms -- there are better targets for optimization
    for (Boid boid = 0; boid < boid_list.m_size; boid++) {       
        //Window edges
        vxs[boid] = vxs[boid] + rules.edge_factor * ((xs[boid] < rules.edge_width) - (xs[boid] > (world_width - rules.edge_width)));
        vys[boid] = vys[boid] + rules.edge_factor * ((ys[boid] < rules.edge_width) - (ys[boid] > (world_height - rules.edge_width)));

        //Apply homing
        int home_index_y = homes[boid] / 16;
        int home_index_x = homes[boid] % 16;

        float home_loc_x = home_index_x * ((world_width - rules.edge_width * 2) /  16) + rules.edge_width;
        float home_loc_y = home_index_y * ((world_height - rules.edge_width * 2) /  9) + rules.edge_width;

        float dx = home_loc_x - xs[boid];
        float dy = home_loc_y - ys[boid];

        vxs[boid] += dx * rules.homing;
        vys[boid] += dy * rules.homing;

        float speed = sqrtf((vxs[boid]*vxs[boid]) + (vys[boid]*vys[boid]));
        
        //speed = speed + (!speed);
        
        auto const minspeed = 3.0f;
        auto const maxspeed = 4.0f;

        float ispeed = maxspeed/speed;
        float lspeed = minspeed/speed;

        vxs[boid] = (speed <= maxspeed) * vxs[boid] + (speed > maxspeed) * vxs[boid] * ispeed;
        vys[boid] = (speed <= maxspeed) * vys[boid] + (speed > maxspeed) * vys[boid] * ispeed;

        vxs[boid] = (speed >= minspeed) * vxs[boid] + (speed < minspeed) * vxs[boid] * lspeed;
        vys[boid] = (speed >= minspeed) * vys[boid] + (speed < minspeed) * vys[boid] * lspeed;

        //if (boid < 8) DEBUG("xs[b]: %f", ys[boid] + vys[boid]*ispeed);
        
        xs[boid] += vxs[boid];
        ys[boid] += vys[boid];        
    }

    
    /*
    Inital profiling on SIMD for this showed negligible results.

    const auto rf = _mm256_set1_ps(rules.edge_factor);
    const auto ew = _mm256_set1_ps(rules.edge_width);
    const auto wh = _mm256_set1_ps(world_height);
    const auto ww = _mm256_set1_ps(world_width);

    for (Boid boid = boid_list.m_size; boid < boid_list.m_size; boid += 8) { 
        const auto xs_vec = _mm256_load_ps(&xs[boid]);
        const auto ys_vec = _mm256_load_ps(&ys[boid]);

        auto vxs_vec = _mm256_load_ps(&vxs[boid]);
        auto vys_vec = _mm256_load_ps(&vys[boid]);

        auto cmpx_1 = _mm256_cmp_ps(xs_vec, ew, _CMP_LT_OS);
        auto cmpx_2 = _mm256_cmp_ps(xs_vec, _mm256_sub_ps(ww, - ew), _CMP_GT_OS);
        auto cmpy_1 = _mm256_cmp_ps(ys_vec, ew, _CMP_LT_OS);
        auto cmpy_2 = _mm256_cmp_ps(ys_vec, _mm256_sub_ps(wh, - ew), _CMP_GT_OS);

        vxs_vec = _mm256_add_ps(vxs_vec, _mm256_sub_ps(_mm256_and_ps(cmpx_1, rf), _mm256_and_ps(cmpx_2, rf)));
        vys_vec = _mm256_add_ps(vys_vec, _mm256_sub_ps(_mm256_and_ps(cmpy_1, rf), _mm256_and_ps(cmpy_2, rf)));

        
        //Apply homing
        int home_index_y = homes[boid] / 16;
        int home_index_x = homes[boid] % 16;

        float home_loc_x = home_index_x * ((world_width - rules.edge_width * 2) /  16) + rules.edge_width;
        float home_loc_y = home_index_y * ((world_height - rules.edge_width * 2) /  9) + rules.edge_width;


        float dx = home_loc_x - xs[boid];
        float dy = home_loc_y - ys[boid];

        vxs[boid] += dx * rules.homing;
        vys[boid] += dy * rules.homing;

        float speed = sqrtf((vxs[boid]*vxs[boid]) + (vys[boid]*vys[boid]));

        
        speed = speed + (!speed);

        float ispeed = 3.0f/speed;
        vxs[boid] = vxs[boid]*ispeed;
        vys[boid] = vys[boid]*ispeed;
        
        xs[boid] += vxs[boid];
        ys[boid] += vys[boid];
        

        const auto threes = _mm256_set1_ps(3.0);

        const auto rspeed_vec =  _mm256_rsqrt_ps(_mm256_add_ps(_mm256_mul_ps(vxs_vec, vxs_vec),_mm256_mul_ps(vys_vec, vys_vec)));
        const auto ispeed_vec = _mm256_mul_ps(threes, rspeed_vec);

        vxs_vec = _mm256_mul_ps(vxs_vec, ispeed_vec);
        vys_vec = _mm256_mul_ps(vys_vec, ispeed_vec);

        //if (boid == 0) debug_vector<float>(_mm256_add_ps(ys_vec, vys_vec), "%f");
        _mm256_store_ps(&xs[boid], _mm256_add_ps(xs_vec, vxs_vec));
        _mm256_store_ps(&ys[boid], _mm256_add_ps(ys_vec, vys_vec));  

        _mm256_store_ps(&vxs[boid], vxs_vec);
        _mm256_store_ps(&vys[boid], vys_vec);  
    }
    */
}

inline void update_non_interacting2(const BoidMap& boid_map, const Rules& rules, const BoidList& boid_list) {
    const auto xs = boid_list.m_boid_store->xs;
    const auto ys = boid_list.m_boid_store->ys;
    const auto vxs = boid_list.m_boid_store->vxs;
    const auto vys = boid_list.m_boid_store->vys;
    const auto homes = boid_list.m_boid_store->homes;

    const auto world_height = boid_map.m_cell_size * boid_map.m_ysize;
    const auto world_width = boid_map.m_cell_size * boid_map.m_xsize;

    //Easy to SIMD but currently only using a few ms -- there are better targets for optimization
    for (Boid boid = 0; boid < boid_list.m_size; boid++) {     
        int home_index_y = homes[boid] / 16;
        int home_index_x = homes[boid] % 16;

        float home_loc_x = home_index_x * ((world_width - rules.edge_width * 2) /  16) + rules.edge_width;
        float home_loc_y = home_index_y * ((world_height - rules.edge_width * 2) /  9) + rules.edge_width;

        float dx = home_loc_x - xs[boid];
        float dy = home_loc_y - ys[boid];

        vxs[boid] += dx * rules.homing;
        vys[boid] += dy * rules.homing;

          
        xs[boid] += vxs[boid];
        ys[boid] += vys[boid];        
    }
}

void row_runner(const BoidMap *boid_map, const int y, const Rules *rules, const BoidList *boid_list) {
    for (int x = 0; x < boid_map->m_xsize; x++) {
        update_cell2(boid_map, x, y, rules, boid_list);
    }
}

void full_runner(const BoidMap *boid_map, const Rules *rules, const BoidList *boid_list) {
    for (int y = 0; y < boid_map->m_ysize; y += 2) {
        for (int x = 0; x < boid_map->m_xsize; x++) {
            //update_cell(boid_map, x, y, rules, selected_boid, boid_list);
            update_cell2(boid_map, x, y, rules, boid_list);
            
        }
    }

    for (int y = 1; y < boid_map->m_ysize; y+=2) {
        for (int x = 0; x < boid_map->m_xsize; x++) {
            //update_cell(boid_map, x, y, rules, selected_boid, boid_list);
            update_cell2(boid_map, x, y, rules, boid_list);
        }
    }
}

//Allocates rows in blocks to each thread
void block_runner(int thread_num, bool offset, const BoidMap *boid_map, const Rules *rules, const BoidList *boid_list) {
    for (int y = thread_num * (boid_map->m_ysize / NUM_THREADS) + offset; y < (thread_num + 1) * (boid_map->m_ysize / NUM_THREADS); y += 2) {
        for (int x = 0; x < boid_map->m_xsize; x++) {
            update_cell2(boid_map, x, y, rules, boid_list);
        }
    }
}

//Allocates rows in blocks to each thread
void block_runner_unstable(int y, const BoidMap *boid_map, const Rules *rules, const BoidList *boid_list) {
    for (int x = 0; x < boid_map->m_xsize; x++) {
        update_cell2(boid_map, x, y, rules, boid_list);
    }
}

//Stripes allocations to each thread (hopefully more cache local)
inline void jump_runner(int thread_num, bool offset, const BoidMap *boid_map, const Rules *rules, const BoidList *boid_list) {
    for (int y = thread_num * 2 + offset; y < boid_map->m_ysize; y += 2 * NUM_THREADS) {
        for (int x = 0; x < boid_map->m_xsize; x++) {
            update_cell2(boid_map, x, y, rules, boid_list);
        }
    }
}

inline void async_runner(int y, std::future<void> *pool, const BoidMap *boid_map, const Rules *rules, const BoidList *boid_list) {
    if (y % 2 != 0) {
        pool[y-1].wait();
        pool[y+1].wait();
    }

    for (int x = 0; x < boid_map->m_xsize; x++) {
        update_cell2(boid_map, x, y, rules, boid_list);
    }
}

__thread volatile int you_shall_not_optimize_this;

void work() {
    // This is the simplest way I can think of to prevent the compiler and
    // operating system from doing naughty things
    you_shall_not_optimize_this = 42;
}

void update_boids(const BoidMap& boid_map, const Rules& rules, const BoidList& boid_list) {
    auto t_start = std::chrono::high_resolution_clock::now();

    #ifndef USE_MULTICORE

    for (int y = 0; y < boid_map.m_ysize; y += 2) {
        for (int x = 0; x < boid_map.m_xsize; x++) {
            //update_cell(boid_map, x, y, rules, selected_boid, boid_list);
            update_cell2(&boid_map, x, y, &rules, &boid_list);
            
        }
    }

    for (int y = 1; y < boid_map.m_ysize; y+=2) {
        for (int x = 0; x < boid_map.m_xsize; x++) {
            //update_cell(boid_map, x, y, rules, selected_boid, boid_list);
            update_cell2(&boid_map, x, y, &rules, &boid_list);
        }
    }

    #endif

    #ifdef USE_MULTICORE

    std::thread threads[NUM_THREADS];

    //Async is currently slower than my threading implementation

    #define USE_ASYNC

    #ifndef USE_ASYNC

    for (int i = 0; i < NUM_THREADS; i++) {
        //block_runner(i, false, &boid_map, &rules, &boid_list);
        threads[i] = std::thread(jump_runner, i, false, &boid_map, &rules, &boid_list);
    }

    for (int i = 0; i < NUM_THREADS; i++) {
        threads[i].join();
    }


    for (int i = 0; i < NUM_THREADS; i++) {
        //block_runner(i, true, &boid_map, &rules, &boid_list);
        threads[i] = std::thread(jump_runner, i, true, &boid_map, &rules, &boid_list);
    }

    for (int i = 0; i < NUM_THREADS; i++) {
        threads[i].join();
    }
    #endif

    /*
    for (int i = 0; i < NUM_THREADS; i++) {
        //block_runner(i, true, &boid_map, &rules, &boid_list);
        threads[i] = std::thread(block_runner_unstable, i, &boid_map, &rules, &boid_list);
    }

    for (int i = 0; i < NUM_THREADS; i++) {
        threads[i].join();
    }
    */

    #ifdef USE_ASYNC
    
    std::vector<std::future<void>> pool;
    pool.resize(NUM_THREADS);

    for (int i = 0; i < NUM_THREADS; i++) {
        pool[i] = std::async(std::launch::async, jump_runner, i, false, &boid_map, &rules, &boid_list);
    }

    for (int i = 0; i < NUM_THREADS; i++) {
        pool[i].wait();
    }

    for (int i = 0; i < NUM_THREADS; i++) {
        pool[i] = std::async(std::launch::async, jump_runner, i, true, &boid_map, &rules, &boid_list);
    }

    for (int i = 0; i < NUM_THREADS; i++) {
        pool[i].wait();
    }
    

    #endif


    #endif
    
    auto t_mid = std::chrono::high_resolution_clock::now();

    update_non_interacting2(boid_map, rules, boid_list);

    auto t_end = std::chrono::high_resolution_clock::now();

    TraceLog(LOG_DEBUG, TextFormat("cells :%0.12f, non-inter: %0.12f", std::chrono::duration<double, std::milli>(t_mid-t_start).count(), std::chrono::duration<double, std::milli>(t_end-t_mid).count()));
}

void render(BoidList *boid_list, Ui *ui, Rules &rules, Camera2D cam, Camera3D camera, Mesh *tri, Material *matInstances) {
    BeginDrawing();
        ClearBackground(BLACK);

        auto t_start_3d = std::chrono::high_resolution_clock::now();
        //We could cull here by offsetting the start pointers by some amount
        int offset = 0;
        BeginMode3D(camera);
            DrawMeshInstanced2(*tri, *matInstances, boid_list->m_size - offset, boid_list->m_boid_store->xs + offset, boid_list->m_boid_store->ys + offset, boid_list->m_boid_store->vxs + offset, boid_list->m_boid_store->vys + offset);
        EndMode3D();

        auto t_end_3d = std::chrono::high_resolution_clock::now();

        //DEBUG("3d: %0.6f", std::chrono::duration<double, std::milli>(t_end_3d-t_start_3d).count());
        /*
        BeginMode2D(cam);
            Boid current = boid_map.get_head_from_screen_space(GetScreenToWorld2D(GetMousePosition(), cam));
            while (current != -1) {
                rlPushMatrix();
                    rlTranslatef(xs[current], ys[current], 0);
                    float angle = (atan2(vxs[current], vys[current]) * 360.) / (2 * PI);
                    rlRotatef(angle, 0, 0, -1);
                    DrawTriangle(triangle[2], triangle[1], triangle[0], BLUE);
                rlPopMatrix();

                current = index_nexts[current];
            }        

            if (rules.show_lines) {
                //Draw grid
                for (int y = 0; y < boid_map.m_ysize; y++) {
                    DrawLine(0, y*boid_map.m_cell_size, boid_map.m_xsize*boid_map.m_cell_size, y*boid_map.m_cell_size, GRAY);
                }
                for (int x = 0; x < boid_map.m_xsize; x++) {
                    DrawLine(x*boid_map.m_cell_size, 0, x*boid_map.m_cell_size, boid_map.m_ysize * boid_map.m_cell_size, GRAY);
                }
            }
        EndMode2D();
        */
        ui->Render(cam, camera, rules);

    EndDrawing();
}


int main (int argc, char* argv[]) {
    uint32_t num_boids = atoi(argv[1]);

    InitWindow(0, 0, "RayLib Boids!");
    SetTargetFPS(FRAME_RATE_LIMIT);

    const int screen_width = GetScreenWidth();
    const int screen_height = GetScreenHeight();

    SetTraceLogLevel(LOG_ALL);
    TraceLog(LOG_DEBUG, TextFormat("Boid size is: %d bytes", sizeof(Boid)));

    //SetConfigFlags(FLAG_MSAA_4X_HINT);

    Mesh tri = GenMeshCustom();

    Shader boid_shader = LoadShader(TextFormat("resources/shaders/glsl%i/directional.vs", 330),
                                    TextFormat("resources/shaders/glsl%i/simple.fs", 330));

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

    BoidList boid_list(num_boids);

    Rules rules = {
        .avoid_distance_squared = 1000.f,
        .avoid_factor = 0.00000002f,
        .sight_range = SIGHT_RANGE,
        .sight_range_squared = SIGHT_RANGE * SIGHT_RANGE,
        .alignment_factor = 0.05f,
        .cohesion_factor = 0.0005f,
        .edge_width = 30,
        .edge_factor = 0.05,
        .rand = 0.1,
        .homing = 0.0000051,
        .show_lines = false,
        .min_speed = 2,
        .max_speed = 3,
    };

    Ui ui;

    PerfMonitor perf_monitor;    

    myfile.open("log.log");

    const int world_width = screen_width * WORLD_SIZE_MULT;
    const int world_height = screen_height * WORLD_SIZE_MULT;

    Camera2D cam = { 0 };
    cam.zoom = static_cast<float>(screen_width) / world_width;

    Camera camera = {
        .position = (Vector3){world_width/2.f, 200.f, world_height/2.f},            
        .target = (Vector3){world_width/2.f, 0.0f, world_height/2.f},                
        .up = (Vector3){ 0.0f, 0.0f, -1.0f },                    
        .fovy = (float) world_height,                                          
        .projection = CAMERA_ORTHOGRAPHIC,                      
    };


    const int CELL_WIDTH = 100;
    //assert(CELL_WIDTH >= rules.sight_range);
    BoidMap boid_map(world_height, world_width, CELL_WIDTH);

    std::srand(std::time(nullptr));
    
    std::default_random_engine generator;
    std::uniform_real_distribution<float> width_distribution (rules.edge_width, world_width - rules.edge_width);
    std::uniform_real_distribution<float> height_distribution (rules.edge_width, world_height - rules.edge_width);

    //TODO just clean this up
    std::uniform_real_distribution<float> width_distribution2 (-((world_width - rules.edge_width * 2) /  16) / 2, ((world_width - rules.edge_width * 2) /  16) / 2);
    std::uniform_real_distribution<float> height_distribution2 (-((world_width - rules.edge_width * 2) /  9) / 2, ((world_width - rules.edge_width * 2) /  9) / 2);

    //Populate some test boids
    for (int i = 0; i < boid_list.m_size; i++) {
        boid_list.m_boid_store->index_next[i] = -1;
        boid_list.m_boid_store->vxs[i] = (std::rand() % 3) - 1;
        boid_list.m_boid_store->vys[i] = (std::rand() % 3) - 1;
        boid_list.m_boid_store->homes[i] = rand() % 144;

        int home_index_y = boid_list.m_boid_store->homes[i] / 16;
        int home_index_x = boid_list.m_boid_store->homes[i] % 16;

        boid_list.m_boid_store->xs[i] = home_index_x * ((world_width - rules.edge_width * 2) /  16) + rules.edge_width + width_distribution2(generator);
        boid_list.m_boid_store->ys[i] = home_index_y * ((world_height - rules.edge_width * 2) /  9) + rules.edge_width + height_distribution2(generator);
    }


    populate_map(boid_list, boid_map);

    auto xs = boid_list.m_boid_store->xs;
    auto ys = boid_list.m_boid_store->ys;
    auto vxs = boid_list.m_boid_store->vxs;
    auto vys = boid_list.m_boid_store->vys;
    auto homes = boid_list.m_boid_store->homes;
    auto index_nexts = boid_list.m_boid_store->index_next;
    

    while (WindowShouldClose() == false){  
        auto t_start = std::chrono::high_resolution_clock::now();
        populate_map(boid_list, boid_map);
        auto t_mid = std::chrono::high_resolution_clock::now();
        rebuild_list(boid_list, boid_map);
        auto t_end = std::chrono::high_resolution_clock::now();

        xs = boid_list.m_boid_store->xs;
        ys = boid_list.m_boid_store->ys;
        vxs = boid_list.m_boid_store->vxs;
        vys = boid_list.m_boid_store->vys;
        homes = boid_list.m_boid_store->homes;
        index_nexts = boid_list.m_boid_store->index_next;

        float cameraPos[3] = { camera.position.x, camera.position.y, camera.position.z };
        SetShaderValue(boid_shader, boid_shader.locs[SHADER_LOC_VECTOR_VIEW], cameraPos, SHADER_UNIFORM_VEC3);

        update_boids(boid_map, rules, boid_list);


        //TODO move camera stuff to class
        if (IsMouseButtonDown(MOUSE_BUTTON_RIGHT)){
            Vector2 delta = Vector2Scale(GetMouseDelta(), -1.0f / cam.zoom);

            cam.target = Vector2Add(cam.target, delta);
                
            camera.position = Vector3 {camera.position.x + delta.x, camera.position.y, camera.position.z + delta.y };
            camera.target = Vector3 {camera.target.x + delta.x, camera.target.y, camera.target.z + delta.y };
        }

        float wheel = GetMouseWheelMove();
        if (wheel != 0)
        {
            Vector2 mouseWorldPos = GetScreenToWorld2D(GetMousePosition(), cam);
            
            auto m_pos = GetMousePosition();
            cam.offset = m_pos;

            cam.target = mouseWorldPos;
    
            cam.zoom += wheel * 0.125f;
        
            if (cam.zoom < (1./WORLD_SIZE_MULT))
                cam.zoom = (1./WORLD_SIZE_MULT);

            camera.fovy = screen_height / cam.zoom;

            camera.position = Vector3 {
                .x = (screen_width * 0.5f - cam.offset.x) / cam.zoom + cam.target.x,
                .y = camera.position.y,
                .z = (screen_height * 0.5f - cam.offset.y) / cam.zoom + cam.target.y,
            };

            camera.target = Vector3 {camera.position.x, 0.f, camera.position.z};
        }
        
        /*
        if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT)) {
            auto mouse_pos = GetScreenToWorld2D(GetMousePosition(), cam);
            Boid current = boid_map.get_head_from_screen_space(mouse_pos);
            float min_dist = -1;
            Boid nearest = -1;

            while (current != -1) {
                float dist = Vector2DistanceSqr(mouse_pos, Vector2 {xs[current], ys[current]});
                if (min_dist == -1 || dist < min_dist) {
                    nearest = current;
                    min_dist = dist;
                }
                current = index_nexts[current];
            }

            selected_boid = nearest;
        }                
        */
        auto t_start_drawing = std::chrono::high_resolution_clock::now();

        render(&boid_list, &ui, rules, cam, camera, &tri, &matInstances);

        //auto test = std::async(std::launch::async, render, &boid_list, &ui, rules, cam, camera, &tri, &matInstances);
        //auto test2 = std::thread(render, &boid_list, &ui, rules, cam, camera, &tri, &matInstances);
        
        //test2.join();




        //DEBUG("test");
        /*
        BeginDrawing();
            ClearBackground(BLACK);

            auto t_start_3d = std::chrono::high_resolution_clock::now();
            //We could cull here by offsetting the start pointers by some amount
            int offset = 0;
            BeginMode3D(camera);
                DrawMeshInstanced2(tri, matInstances, num_boids - offset, boid_list.m_boid_store->xs + offset, boid_list.m_boid_store->ys + offset, boid_list.m_boid_store->vxs + offset, boid_list.m_boid_store->vys + offset);
            EndMode3D();

            auto t_end_3d = std::chrono::high_resolution_clock::now();

            //DEBUG("3d: %0.6f", std::chrono::duration<double, std::milli>(t_end_3d-t_start_3d).count());

            BeginMode2D(cam);
                Boid current = boid_map.get_head_from_screen_space(GetScreenToWorld2D(GetMousePosition(), cam));
                while (current != -1) {
                    rlPushMatrix();
                        rlTranslatef(xs[current], ys[current], 0);
                        float angle = (atan2(vxs[current], vys[current]) * 360.) / (2 * PI);
                        rlRotatef(angle, 0, 0, -1);
                        DrawTriangle(triangle[2], triangle[1], triangle[0], BLUE);
                    rlPopMatrix();

                    current = index_nexts[current];
                }        

                if (rules.show_lines) {
                    //Draw grid
                    for (int y = 0; y < boid_map.m_ysize; y++) {
                        DrawLine(0, y*boid_map.m_cell_size, boid_map.m_xsize*boid_map.m_cell_size, y*boid_map.m_cell_size, GRAY);
                    }
                    for (int x = 0; x < boid_map.m_xsize; x++) {
                        DrawLine(x*boid_map.m_cell_size, 0, x*boid_map.m_cell_size, boid_map.m_ysize * boid_map.m_cell_size, GRAY);
                    }
                }
            EndMode2D();

            ui.Render(cam, camera, rules);

        EndDrawing();
        */
        auto t_end_drawing = std::chrono::high_resolution_clock::now();
        DEBUG("Populate :%0.6f, rebuild: %0.6f, render: %0.6f", std::chrono::duration<double, std::milli>(t_mid-t_start).count(), std::chrono::duration<double, std::milli>(t_end-t_mid).count(), std::chrono::duration<double, std::milli>(t_end_drawing-t_start_drawing).count());

    }

    CloseWindow();
    return 0;
}


// Modified version of DrawMeshInstanced from the raylib module "rmodels.c"
// My modifications bind the x, y, vx, and vy of each boids as vertex attributes
// We can then use these to generate the transformation matricies on the GPU instead.
void DrawMeshInstanced2(Mesh mesh, Material material, int instances, float *boid_x, float *boid_y, float *boid_vx, float *boid_vy)
{
    #define MAX_MATERIAL_MAPS 12
    #define MAX_MESH_VERTEX_BUFFERS 7

    // Instancing required variables
    unsigned int instances_boid_x_ID, instances_boid_y_ID, instances_boid_vx_ID, instances_boid_vy_ID;

    // Bind shader program
    rlEnableShader(material.shader.id);

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
    Matrix matView = rlGetMatrixModelview();
    Matrix matModelView = MatrixIdentity();
    Matrix matProjection = rlGetMatrixProjection();

    // Upload view and projection matrices (if locations available)
    if (material.shader.locs[SHADER_LOC_MATRIX_VIEW] != -1) rlSetUniformMatrix(material.shader.locs[SHADER_LOC_MATRIX_VIEW], matView);
    if (material.shader.locs[SHADER_LOC_MATRIX_PROJECTION] != -1) rlSetUniformMatrix(material.shader.locs[SHADER_LOC_MATRIX_PROJECTION], matProjection);

    // Enable mesh VAO to attach new buffer
    rlEnableVertexArray(mesh.vaoId);

    instances_boid_x_ID = rlLoadVertexBuffer(boid_x, instances*sizeof(float), false);
    rlEnableVertexAttribute(material.shader.locs[26]);
    rlSetVertexAttribute(material.shader.locs[26], 1, RL_FLOAT, false, sizeof(float), 0);
    rlSetVertexAttributeDivisor(material.shader.locs[26], 1);

    instances_boid_y_ID = rlLoadVertexBuffer(boid_y, instances*sizeof(float), false);
    rlEnableVertexAttribute(material.shader.locs[27]);
    rlSetVertexAttribute(material.shader.locs[27], 1, RL_FLOAT, false, sizeof(float), 0);
    rlSetVertexAttributeDivisor(material.shader.locs[27], 1);

    instances_boid_vx_ID = rlLoadVertexBuffer(boid_vx, instances*sizeof(float), false);
    rlEnableVertexAttribute(material.shader.locs[28]);
    rlSetVertexAttribute(material.shader.locs[28], 1, RL_FLOAT, false, sizeof(float), 0);
    rlSetVertexAttributeDivisor(material.shader.locs[28], 1);

    instances_boid_vy_ID = rlLoadVertexBuffer(boid_vy, instances*sizeof(float), false);
    rlEnableVertexAttribute(material.shader.locs[29]);
    rlSetVertexAttribute(material.shader.locs[29], 1, RL_FLOAT, false, sizeof(float), 0);
    rlSetVertexAttributeDivisor(material.shader.locs[29], 1);
    
    rlDisableVertexBuffer();
    rlDisableVertexArray();

    // Accumulate internal matrix transform (push/pop) and view matrix
    // NOTE: In this case, model instance transformation must be computed in the shader
    matModelView = MatrixMultiply(rlGetMatrixTransform(), matView);

    // Try binding vertex array objects (VAO)
    rlEnableVertexArray(mesh.vaoId);
       
    // Calculate model-view-projection matrix (MVP)
    Matrix matModelViewProjection = MatrixMultiply(matModelView, matProjection);
    
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
    
    // Disable all possible vertex array objects (or VBOs)
    rlDisableVertexArray();
    rlDisableVertexBuffer();
    rlDisableVertexBufferElement();

    // Disable shader program
    rlDisableShader();

    // Remove instance transforms buffer
    rlUnloadVertexBuffer(instances_boid_x_ID);
    rlUnloadVertexBuffer(instances_boid_y_ID);
    rlUnloadVertexBuffer(instances_boid_vx_ID);
    rlUnloadVertexBuffer(instances_boid_vy_ID);
}


enum TaskType {
    TEST_TASK1,
    TEST_TASK2,
    TEST_TASK3,
    UPDATE_BOIDS,
};

void function1() {
    DEBUG("Function 1 executed");
}

void function2() {
    DEBUG("Function 2 executed");
}

void function3() {
    DEBUG("Function 3 executed");
}

struct ThreadData {
    const BoidMap *boid_map;
    const Rules *rules;
    const BoidList *boid_list;
};

struct Task {
    TaskType task_type;
    void *argument_struct;
    int *task_counter;
};

struct UpdateArgs {
    int thread_num;
    bool offset;
};

typedef volatile int spinlock_t; 

void lock(spinlock_t* lock) {
    while (__sync_lock_test_and_set(lock, 1)) {}
}

void unlock(spinlock_t* lock) {
    __sync_synchronize();
    *lock = 0;
}

struct TaskMaster {
    uint8_t front;
    uint8_t back;

    Task task_buffer[256];

    spinlock_t lock;
};

Task getTask(TaskMaster *task_master) {
    Task task;

    //This should hang until a task is available
    try_lock:

    //Aquire front lock
    lock(&task_master->lock);
        //Critical region
        //If there is a task available
        if(task_master->front != task_master->back) {
            task = task_master->task_buffer[task_master->front];
            task_master->lock += 1;
        } else {
            //Release lock, yeild jump to front of block
            //Yes I'm using goto. -- It reduces the number of expressions being evaluated, and makes this code clearer (IMO).
            unlock(&task_master->lock);
            std::this_thread::yield();
            goto try_lock;
        }

    unlock(&task_master->lock);

    //Return the actual task, because after this point the allocation in the array may be deleted (and they're very small memory wise)
    return task;
}

void runner(TaskMaster *task_master, ThreadData thread_data) {

    //Wait for task
    Task current_task = getTask(task_master);

    // Test case switch and function pointers
    switch (current_task.task_type) {
        case TaskType::TEST_TASK1:
            function1();
            break;
        
        case TaskType::TEST_TASK2:
            function2();
            break;

        case TaskType::TEST_TASK3:
            function3();
            break;

        case TaskType::UPDATE_BOIDS:
            auto cast_args = (UpdateArgs *) current_task.argument_struct;
            jump_runner(cast_args->thread_num, cast_args->offset, thread_data.boid_map, thread_data.rules, thread_data.boid_list);
            break;
    }
}