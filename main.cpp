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

#include "imgui.h"

#define NO_FONT_AWESOME

#include <rlImGui.h>

std::ofstream myfile;

const float TRIANGLE_SIZE = 5.f;
const uint32_t NUM_BOIDS = 75000;
const uint16_t SIGHT_RANGE = 100;

static const Vector2 triangle[3] = {
    Vector2 {0.f, 2 * TRIANGLE_SIZE},
    Vector2 {-TRIANGLE_SIZE, -2 * TRIANGLE_SIZE},
    Vector2 {TRIANGLE_SIZE, -2 * TRIANGLE_SIZE}
};

//#define DEBUG(...) myfile << TextFormat(__VA_ARGS__) << '\n';
//#define DEBUG(...) ;
#define DEBUG(...) TraceLog(LOG_DEBUG, TextFormat(__VA_ARGS__));

//Initial plan
//Hybrid linked list structure with periodic reodering to increase cache locality at expensive of intermittent memory spikes.

struct BoidStore {
    float *xs;            
    float *ys;            
    float *vxs;          
    float *vys;
    int32_t *index_next;  
    int32_t *homes;       
    int32_t *depth;                 
} typedef BoidStore;

typedef int32_t Boid;

const int screen_width = 2560;
const int screen_height = 1440;

struct Rules {
    float avoid_distance_squared = 1000.f;
    float avoid_factor = 0.00000002f;
    float sight_range = SIGHT_RANGE;
    float sight_range_squared = SIGHT_RANGE * SIGHT_RANGE;
    float alignment_factor = 0.05f;
    float cohesion_factor = 0.0005f;
    int edge_width = 30;
    float edge_factor = 0.05;
    float rand = 0.1; 
    float homing = 0.0000001;
    bool show_lines = false;
    int min_speed = 2;
    int max_speed = 3;
} typedef Rules;

struct PerfMonitor {
    int tick_counter = 0;
    float tps;
    float rolling_average;
} typedef PerfMonitor;

void free_boidstore_members(BoidStore *boid_store) {
    free(boid_store->index_next);
    free(boid_store->xs);
    free(boid_store->ys);
    free(boid_store->vxs);
    free(boid_store->vys);
    free(boid_store->homes);
    free(boid_store->depth);
}

BoidStore* new_boidstore(int to_alloc) {
    return new BoidStore {
        .xs = (float *) _aligned_malloc(to_alloc * sizeof(float), 64),
        .ys = (float *) _aligned_malloc(to_alloc * sizeof(float), 64),
        .vxs = (float *) _aligned_malloc(to_alloc * sizeof(float), 64),
        .vys = (float *) _aligned_malloc(to_alloc * sizeof(float), 64),
        .index_next = (int32_t *) _aligned_malloc(to_alloc * sizeof(int32_t), 64),
        .homes = (int32_t *) _aligned_malloc(to_alloc * sizeof(int32_t), 64),
        .depth = (int32_t *) _aligned_malloc(to_alloc * sizeof(int32_t), 64),
    };
}

class BoidList {
    public:
        BoidStore *m_boid_store;
        BoidStore *m_backbuffer; //We're going to use a back buffer as we would need to have double the
                                 //memory allocated at some point and this reduces calls to malloc.
        int m_size;

    public:
        BoidList() = delete;
        BoidList(int size) { 
            //We need to round up the amount of memory we are allocating as 
            //we will be reading these as blocks of 8 floats (__m256)

            //We should allocate at least 28 extra bytes so if we read the last real 
            //float as the begining of an 8 byte block we do not overrun

            int to_alloc = (size + 1);

            m_boid_store = new_boidstore(to_alloc);
            m_backbuffer = new_boidstore(to_alloc);

            m_size = size;

            TraceLog(LOG_DEBUG, TextFormat("Initialized Boid List of size %d", size));
        }
        
        //Todo refactor this to only use new/delete 
        ~BoidList() {
            free_boidstore_members(m_boid_store);
            free_boidstore_members(m_backbuffer);

            delete m_boid_store;
            delete m_backbuffer;

            TraceLog(LOG_DEBUG, "Deallocated boid list");
        }
};

class BoidMap {
    public:
        int m_ysize;
        int m_xsize;
        int m_cell_size; 
        Boid *m_boid_map;

    public:
        BoidMap() = delete;
        BoidMap(const int height, const int width, const int cell_size) { 
            m_ysize = std::ceil(height / static_cast<float>(cell_size));
            m_xsize = std::ceil(width / static_cast<float>(cell_size));
            m_boid_map = new Boid[m_ysize * m_xsize]();
            m_cell_size = cell_size;

            TraceLog(LOG_DEBUG, TextFormat("Initalized map of size (%d x %d)", m_ysize, m_xsize));
        }
        
        ~BoidMap() {
            TraceLog(LOG_DEBUG, "Deallocated boid map");
            delete[] m_boid_map;
        }
        
        Boid get_coord(const int y, const int x) const {
            //Little bit of safety, useful to minimize bounds checking code
            if (x < 0 || y < 0 || x >= m_xsize || y >= m_ysize) {
                return -1;
            }

            return m_boid_map[y * m_xsize + x];
        }

        //Returns the nearest map position (in case the supplied position is out of bounds of the map)
        int get_map_pos_nearest(const int x, const int y) const {
            int col = std::min(std::max(x, 0), m_cell_size * m_xsize - 1) / m_cell_size;
            int row = std::min(std::max(y, 0), m_cell_size * m_ysize - 1) / m_cell_size;

            return row * m_xsize + col;
        }

        Boid get_head_from_screen_space(const Vector2 pos) const {
            return m_boid_map[get_map_pos_nearest(pos.x, pos.y)];
        }

        Boid get_absolute(const int n) const {
            return m_boid_map[n];
        }

        int get_map_pos(const int x, const int y) const {
            int row = y / m_cell_size;
            int col = x / m_cell_size;

            return row * m_xsize + col;
        }
};


Boid rebuild_list(BoidList& boid_list, BoidMap& boid_map, Boid to_track) {
    BoidStore *main_buffer = boid_list.m_boid_store;
    BoidStore *back_buffer = boid_list.m_backbuffer;

    Boid new_pos = -1;

    int index = 0;
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

            if (current == to_track) new_pos = index;
            
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

    return new_pos;
}

void populate_map(BoidList& boid_list, BoidMap& map) {
    for (int i = 0; i < map.m_xsize * map.m_ysize; i++) {
        map.m_boid_map[i] = -1;
    }

    //todo slight problem here is that we are WILL reverse the memory positions in each cell each update.
    //Maybe we can fix this writing into the new list in reverse order in the rebuild list function (Use the depth to work out correct positions).
    //Or we can fix this by building our linked list s.t. the first boid we find in a cell will stay as the head.
    //This would be performance intensive.
    for (int i = 0; i < boid_list.m_size; i++) {
        Boid boid_to_place = i;
        int map_pos = map.get_map_pos_nearest(boid_list.m_boid_store->xs[boid_to_place], boid_list.m_boid_store->ys[boid_to_place]); //No one said that vectorization would be pretty...
        Boid old_head = map.m_boid_map[map_pos];
        map.m_boid_map[map_pos] = boid_to_place;
        boid_list.m_boid_store->index_next[boid_to_place] = old_head;
        boid_list.m_boid_store->depth[boid_to_place] = (old_head != -1) ? boid_list.m_boid_store->depth[old_head] + 1 : 1;
    }
}

/*
void print_boid(Boid *boid) {
    TraceLog(LOG_DEBUG, TextFormat("{next: %p, x: %d, y: %d}", boid->next, boid->x, boid->y));
}
*/
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
    auto temp = _mm256_hadd_epi32(v, v);
    temp = _mm256_hadd_epi32(temp, temp);
    auto fliptemp = (__m256i) _mm256_permute2f128_ps((__m256) temp, (__m256) temp, 1);
    return _mm256_add_epi32(temp, fliptemp);
}

inline __m256 vector_sum(__m256 v) {
    auto temp = _mm256_hadd_ps(v, v);
    temp = _mm256_hadd_ps(temp, temp);
    auto fliptemp = _mm256_permute2f128_ps(temp, temp, 1); 
    return _mm256_add_ps(temp, fliptemp);
}

void update_cell(const BoidMap& map, const int x, const int y, const Rules& rules, Boid selected_boid, const BoidList& boid_list) {
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

        //Todo check if this would be faster branchless
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
                
                //Remember to mask before summation!
                __m256 sep_x_vec = _mm256_set1_ps(0.);
                __m256 sep_y_vec = _mm256_set1_ps(0.);
                __m256 avg_x_vec = _mm256_set1_ps(0.);
                __m256 avg_y_vec = _mm256_set1_ps(0.);
                __m256 avg_vx_vec = _mm256_set1_ps(0.);
                __m256 avg_vy_vec = _mm256_set1_ps(0.);

                uint_fast16_t in_sight_counter = 0; 
                __m256i isc = _mm256_set1_epi32(0);

                __m256i read_mask;
                __m256i read_mask_rev;

                
                //Check against each boid in the row currently being processed
                for (Boid nearby_boid = row_begin; nearby_boid < row_end; nearby_boid++) {
                    int bytes_left = row_end - nearby_boid; 
                    //read_mask       = _mm256_set_epi32(0, (bytes_left < 1) * 0xFFFF, (bytes_left < 2) * 0xFFFF, (bytes_left < 3) * 0xFFFF, (bytes_left < 4) * 0xFFFF, (bytes_left < 5) * 0xFFFF, (bytes_left < 6) * 0xFFFF, (bytes_left < 7) * 0xFFFF);
                    //read_mask_rev   = _mm256_set_epi32((bytes_left < 7) * 0xFFFF, (bytes_left < 6) * 0xFFFF, (bytes_left < 5) * 0xFFFF, (bytes_left < 4) * 0xFFFF, (bytes_left < 3) * 0xFFFF, (bytes_left < 2) * 0xFFFF, (bytes_left < 1) * 0xFFFF, 0);
                    /*
                    read_mask_rev = _mm256_set1_epi32(0);
                    const auto nearby_xs_vec = _mm256_loadu_ps(&xs[nearby_boid]);
                    const auto nearby_ys_vec = _mm256_loadu_ps(&ys[nearby_boid]);

                    const auto nearby_vxs_vec = _mm256_loadu_ps(&vxs[nearby_boid]);
                    const auto nearby_vys_vec = _mm256_loadu_ps(&vys[nearby_boid]);

                    const auto current_xs_vec = _mm256_set1_ps(xs[current_boid]);
                    const auto current_ys_vec = _mm256_set1_ps(xs[current_boid]);              
                    */
                    int_fast32_t dist_squared = (xs[current_boid] - xs[nearby_boid]) * (xs[current_boid] - xs[nearby_boid]) + (ys[current_boid] - ys[nearby_boid]) * (ys[current_boid] - ys[nearby_boid]);
                    
                    /*
                    auto xs_delta = _mm256_sub_ps(current_xs_vec, nearby_xs_vec);
                    auto ys_delta = _mm256_sub_ps(current_xs_vec, nearby_xs_vec);
                    auto ds_vec = _mm256_add_ps(_mm256_mul_ps(xs_delta, xs_delta), _mm256_mul_ps(ys_delta, ys_delta));

                    //this can be moved
                    const auto ads_vec = _mm256_set1_ps(rules.avoid_distance_squared);
                    */
                    const bool pc_srs = dist_squared < rules.sight_range_squared;
                    //const auto srs_mask = _mm256_cmp_ps(ds_vec, _mm256_set1_ps(rules.sight_range_squared), _CMP_LE_OS);

                    const bool pc_ads = dist_squared < rules.avoid_distance_squared;
                    //const auto ads_mask = _mm256_cmp_ps(ds_vec, _mm256_set1_ps(rules.avoid_distance_squared), _CMP_LE_OS);

                    const bool in_sight_but_not_avoid = !pc_ads & pc_srs;
                    //Warning - not a bit mask!
                    //const auto sna_fpmask = _mm256_and_ps(_mm256_andnot_ps(ads_mask, srs_mask), _mm256_set1_ps(1.));

                    //const auto ads_take_ds = _mm256_sub_ps(ads_vec, ds_vec);
                    //const auto ads_take_ds_sqr = _mm256_mul_ps(ads_take_ds, ads_take_ds);
                    
                    sep_x += (pc_ads) * (xs[current_boid] - xs[nearby_boid]) * (rules.avoid_distance_squared - dist_squared) * (rules.avoid_distance_squared - dist_squared);
                    sep_y += (pc_ads) * (ys[current_boid] - ys[nearby_boid]) * (rules.avoid_distance_squared - dist_squared) * (rules.avoid_distance_squared - dist_squared);
                    
                    //sep_x_vec = _mm256_add_ps(sep_x_vec, _mm256_and_ps(ads_mask, _mm256_mul_ps(xs_delta, ads_take_ds_sqr)));
                    //sep_y_vec = _mm256_add_ps(sep_y_vec, _mm256_and_ps(ads_mask, _mm256_mul_ps(ys_delta, ads_take_ds_sqr)));

                    avg_vx += in_sight_but_not_avoid * vxs[nearby_boid];
                    avg_vy += in_sight_but_not_avoid * vys[nearby_boid];

                    //avg_vx_vec = _mm256_fmadd_ps(sna_fpmask, nearby_vxs_vec, avg_vx_vec);
                    //avg_vy_vec = _mm256_fmadd_ps(sna_fpmask, nearby_vys_vec, avg_vy_vec);

                    avg_x  += in_sight_but_not_avoid * xs[nearby_boid];
                    avg_y  += in_sight_but_not_avoid * ys[nearby_boid];

                    //avg_x_vec = _mm256_fmadd_ps(sna_fpmask, nearby_xs_vec, avg_x_vec);
                    //avg_y_vec = _mm256_fmadd_ps(sna_fpmask, nearby_ys_vec, avg_y_vec);

                    //DEBUG("s: %f", xs[nearby_boid]);
                    //DEBUG("s: %f", (pc_ads) * (xs[current_boid] - xs[nearby_boid]) * (rules.avoid_distance_squared - dist_squared) * (rules.avoid_distance_squared - dist_squared));

                    //DEBUG("s: %d", in_sight_but_not_avoid);
                    in_sight_counter += in_sight_but_not_avoid;
                    //in_sight_counter++;
                    //isc = _mm256_add_epi32(isc, _mm256_cvtps_epi32(sna_fpmask));
                }
                

                for (Boid nearby_boid = row_begin; nearby_boid < row_end; nearby_boid += 8) {
                    int bytes_left = row_end - nearby_boid; 
                    read_mask = _mm256_set_epi32((bytes_left > 7) * 0xFFFFFFFF, (bytes_left > 6) * 0xFFFFFFFF, (bytes_left > 5) * 0xFFFFFFFF, (bytes_left > 4) * 0xFFFFFFFF, (bytes_left > 3) * 0xFFFFFFFF, (bytes_left > 2) * 0xFFFFFFFF, (bytes_left > 1) * 0xFFFFFFFF, 0xFFFFFFFF);

                    const auto nearby_xs_vec = _mm256_loadu_ps(&xs[nearby_boid]);
                    const auto nearby_ys_vec = _mm256_loadu_ps(&ys[nearby_boid]);

                    const auto nearby_vxs_vec = _mm256_loadu_ps(&vxs[nearby_boid]);
                    const auto nearby_vys_vec = _mm256_loadu_ps(&vys[nearby_boid]);

                    const auto current_xs_vec = _mm256_set1_ps(xs[current_boid]);
                    const auto current_ys_vec = _mm256_set1_ps(ys[current_boid]);              
                    
                    auto xs_delta = _mm256_sub_ps(current_xs_vec, nearby_xs_vec);
                    auto ys_delta = _mm256_sub_ps(current_ys_vec, nearby_ys_vec);
                    auto ds_vec = _mm256_add_ps(_mm256_mul_ps(xs_delta, xs_delta), _mm256_mul_ps(ys_delta, ys_delta));

                    //this can be moved
                    const auto ads_vec = _mm256_set1_ps(rules.avoid_distance_squared);

                    const auto srs_mask = _mm256_cmp_ps(ds_vec, _mm256_set1_ps(rules.sight_range_squared), _CMP_LT_OS); 

                    const auto ads_mask = _mm256_cmp_ps(ds_vec, _mm256_set1_ps(rules.avoid_distance_squared), _CMP_LT_OS);

                    //Warning - not a bit mask!
                    const auto sna_fpmask = _mm256_and_ps(_mm256_andnot_ps(ads_mask, srs_mask), _mm256_set1_ps(1.));

                    const auto ads_take_ds = _mm256_sub_ps(ads_vec, ds_vec);
                    const auto ads_take_ds_sqr = _mm256_mul_ps(ads_take_ds, ads_take_ds);
                    
                    //sep_x_vec = _mm256_and_ps(_mm256_add_ps(sep_x_vec, _mm256_and_ps(ads_mask, _mm256_mul_ps(xs_delta, ads_take_ds_sqr))), (__m256) read_mask);
                    //sep_y_vec = _mm256_and_ps(_mm256_add_ps(sep_y_vec, _mm256_and_ps(ads_mask, _mm256_mul_ps(ys_delta, ads_take_ds_sqr))), (__m256) read_mask);
                    sep_x_vec = _mm256_add_ps(sep_x_vec, _mm256_and_ps(ads_mask, _mm256_mul_ps(xs_delta, ads_take_ds_sqr)));
                    sep_y_vec = _mm256_add_ps(sep_y_vec, _mm256_and_ps(ads_mask, _mm256_mul_ps(ys_delta, ads_take_ds_sqr)));


                    avg_vx_vec = _mm256_fmadd_ps(sna_fpmask, nearby_vxs_vec, avg_vx_vec);
                    avg_vy_vec = _mm256_fmadd_ps(sna_fpmask, nearby_vys_vec, avg_vy_vec);

                    avg_x_vec = _mm256_fmadd_ps(sna_fpmask, nearby_xs_vec, avg_x_vec);
                    avg_y_vec = _mm256_fmadd_ps(sna_fpmask, nearby_ys_vec, avg_y_vec);

                    isc = _mm256_add_epi32(isc, (__m256i) _mm256_and_ps((__m256) read_mask, (__m256) _mm256_cvtps_epi32(sna_fpmask)));
                    
                    //DEBUG("to read: %d", bytes_left);
                    //DEBUG("RANGE: %f", rules.sight_range_squared);

                    //debug_vector<float, __m256>(_mm256_and_ps((__m256) read_mask, nearby_xs_vec), "v: %f");
                    //isc = _mm256_add_epi32(isc, _mm256_set1_epi32(1));

                    //debug_vector<uint32_t, __m256i>(_mm256_cvtps_epi32(sna_fpmask), "v: %d");

                    /*
                    DEBUG("----");
                    debug_vector<float, __m256>(ds_vec, "ds %f");
                    debug_vector<uint32_t, __m256i>((__m256i) srs_mask, "srs 0x%08x");
                    debug_vector<float, __m256>(nearby_xs_vec, "near_x %f");
                    debug_vector<float, __m256>(avg_x_vec, "avgx %f");
                    DEBUG("----");
                    */
                    /*
                    debug_vector<float, __m256>(_mm256_set1_ps(rules.sight_range_squared), "set %f");
                    */


                    //debug_vector<float, __m256>(ds_vec, "ds %f");
                    //debug_vector<uint32_t, __m256i>((__m256i) srs_mask, "srs 0x%08x");
                    //debug_vector<float, __m256>(sna_fpmask, "sna %f");

                    //debug_vector<float, __m256>(sep_x_vec, "sep_x %f");
                

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

                //uint32_t mask = (in_sight_counter != 0) * 0xffff;

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

void update_non_interacting(const BoidMap& boid_map, const Rules& rules, const BoidList& boid_list) {
    const auto xs = boid_list.m_boid_store->xs;
    const auto ys = boid_list.m_boid_store->ys;
    const auto vxs = boid_list.m_boid_store->vxs;
    const auto vys = boid_list.m_boid_store->vys;
    const auto homes = boid_list.m_boid_store->homes;

    const auto world_height = boid_map.m_cell_size * boid_map.m_ysize;
    const auto world_width = boid_map.m_cell_size * boid_map.m_xsize;

    for (int i = 0; i < boid_list.m_size; i++) {
        Boid boid = i;   
        
        //Window edges, todo replace with old function
        if (xs[boid] > (world_width - rules.edge_width)) {
            vxs[boid] -= rules.edge_factor;
        }
        if (xs[boid] < rules.edge_width) {
            vxs[boid] += rules.edge_factor;
        } 
        if (ys[boid] > (world_height - rules.edge_width)) {
            vys[boid] -= rules.edge_factor;
        } 
        if (ys[boid] < rules.edge_width) {
            vys[boid] += rules.edge_factor;
        } 


        //Apply some randomness
        float rx = static_cast <float> (rand()) / (static_cast <float> (RAND_MAX/2)) - 1;
        float ry = static_cast <float> (rand()) / (static_cast <float> (RAND_MAX/2)) - 1;

        vxs[boid] += rx * rules.rand;
        vys[boid] += ry * rules.rand;


        //Apply homing
        int hy = homes[boid] / 16;
        int hx = homes[boid] % 16;

        float px = hx * ((world_width - rules.edge_width * 2) /  16) + rules.edge_width;
        float py = hy * ((world_height - rules.edge_width * 2) /  9) + rules.edge_width;


        float dx = px - xs[boid];
        float dy = py - ys[boid];

        vxs[boid] += dx * rules.homing;
        vys[boid] += dy * rules.homing;

        float speed = sqrtf((vxs[boid]*vxs[boid]) + (vys[boid]*vys[boid]));

        if (speed > 0)
        {
            float ispeed = 1.0f/speed;
            vxs[boid] = vxs[boid]*ispeed * 3;
            vys[boid] = vys[boid]*ispeed * 3;
        }
        
        xs[boid] += vxs[boid];
        ys[boid] += vys[boid];
    }
}

void update_boids(const BoidMap& boid_map, const Rules& rules, Boid selected_boid, const BoidList& boid_list) {
    auto t_start = std::chrono::high_resolution_clock::now();

    for (int y = 0; y < boid_map.m_ysize; y++) {
        for (int x = 0; x < boid_map.m_xsize; x++) {
            update_cell(boid_map, x, y, rules, selected_boid, boid_list);
        }
    }

    auto t_mid = std::chrono::high_resolution_clock::now();

    update_non_interacting(boid_map, rules, boid_list);

    auto t_end = std::chrono::high_resolution_clock::now();

    TraceLog(LOG_DEBUG, TextFormat("cells :%0.12f, non-inter: %0.12f", std::chrono::duration<double, std::milli>(t_mid-t_start).count(), std::chrono::duration<double, std::milli>(t_end-t_mid).count()));
}

void UpdateRulesWindow(Rules &rules) {
        ImGui::Begin("Boids Settings");
        ImGui::SliderFloat("alignment_factor", &rules.alignment_factor, 0., 1., "%0.9f");
        ImGui::SliderFloat("sight_range", &rules.sight_range, 0., 100., "%0.9f");
        ImGui::SliderFloat("avoid_distance_squared", &rules.avoid_distance_squared, 0., 1000., "%0.9f");
        ImGui::SliderFloat("avoid_factor", &rules.avoid_factor, 0., 1., "%0.9f");
        ImGui::SliderFloat("cohesion_factor", &rules.cohesion_factor, 0., 1., "%0.9f");
        ImGui::SliderFloat("rand", &rules.rand, 0., 1., "%0.9f");
        ImGui::SliderFloat("homing", &rules.homing, 0., 1., "%0.9f");
        ImGui::SliderInt("edge_width", &rules.edge_width, 0, 100, "%d");
        ImGui::SliderFloat("edge_factor", &rules.edge_factor, 0., 1., "%0.9f");
        ImGui::Checkbox("Show Debug Lines", &rules.show_lines);
        ImGui::End();

    rules.sight_range_squared = rules.sight_range * rules.sight_range;
}

void UpdatePerfMonitor(PerfMonitor &perf_monitor, bool *rebuild_scheduled) {
    perf_monitor.tps = GetFPS();
        ImGui::Begin("Performance Monitor");
        ImGui::Text("TPS: %f", perf_monitor.tps);
        if (ImGui::Button("Rebuild")) {
            *rebuild_scheduled = true;
        }
        ImGui::End();
}

int main () {
    InitWindow(screen_width, screen_height, "RayLib Boids!");
    SetTargetFPS(130);

    SetTraceLogLevel(LOG_ALL);
    TraceLog(LOG_DEBUG, TextFormat("Boid size is: %d bytes", sizeof(Boid)));
    
    std::time_t result = std::time(nullptr);
    TraceLog(LOG_DEBUG, TextFormat("Time is: %s", std::asctime(std::localtime(&result)) ));

    BoidList boid_list(NUM_BOIDS);

    Rules rules;
    PerfMonitor perf_monitor;    

    myfile.open("log.log");

    int world_width = screen_width * 8;
    int world_height = screen_height * 8;

    Camera2D cam = { 0 };
    cam.zoom = static_cast<float>(screen_width) / world_width;

    const int CELL_WIDTH = 150;
    assert(CELL_WIDTH > rules.sight_range);
    BoidMap boid_map(world_height, world_width, CELL_WIDTH);

    std::srand(std::time(nullptr));
    
    std::default_random_engine generator;
    std::uniform_real_distribution<float> width_distribution (rules.edge_width, world_width - rules.edge_width);
    std::uniform_real_distribution<float> height_distribution (rules.edge_width, world_height - rules.edge_width);

    //Populate some test boids
    for (int i = 0; i < boid_list.m_size; i++) {
        boid_list.m_boid_store->index_next[i] = -1;
        boid_list.m_boid_store->xs[i] = width_distribution(generator);
        boid_list.m_boid_store->ys[i] = height_distribution(generator);
        boid_list.m_boid_store->vxs[i] = (std::rand() % 3) - 1;
        boid_list.m_boid_store->vys[i] = (std::rand() % 3) - 1;
        boid_list.m_boid_store->homes[i] = rand() % 144;
    }


    populate_map(boid_list, boid_map);

    auto xs = boid_list.m_boid_store->xs;
    auto ys = boid_list.m_boid_store->ys;
    auto vxs = boid_list.m_boid_store->vxs;
    auto vys = boid_list.m_boid_store->vys;
    auto homes = boid_list.m_boid_store->homes;
    auto index_nexts = boid_list.m_boid_store->index_next;

    rlImGuiSetup(true);
    

    Boid selected_boid = -1;
    bool rebuild_scheduled = false;

    while (WindowShouldClose() == false){

        populate_map(boid_list, boid_map);
        selected_boid = rebuild_list(boid_list, boid_map, selected_boid);

        xs = boid_list.m_boid_store->xs;
        ys = boid_list.m_boid_store->ys;
        vxs = boid_list.m_boid_store->vxs;
        vys = boid_list.m_boid_store->vys;
        homes = boid_list.m_boid_store->homes;
        index_nexts = boid_list.m_boid_store->index_next;

        BeginDrawing();
            ClearBackground(BLACK);
            BeginMode2D(cam);

            if (IsMouseButtonDown(MOUSE_BUTTON_RIGHT))
            {
                Vector2 delta = GetMouseDelta();
                delta = Vector2Scale(delta, -1.0f / cam.zoom);

                cam.target = Vector2Add(cam.target, delta);
            }

            float wheel = GetMouseWheelMove();
            if (wheel != 0)
            {
                Vector2 mouseWorldPos = GetScreenToWorld2D(GetMousePosition(), cam);

                cam.offset = GetMousePosition();
                cam.target = mouseWorldPos;

                cam.zoom += wheel * 0.125f;
                if (cam.zoom < 0.125f)
                    cam.zoom = 0.125f;
            }
            
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

            update_boids(boid_map, rules, selected_boid, boid_list);
           
            //TraceLog(LOG_DEBUG, TextFormat("%0.12f", std::chrono::duration<double, std::milli>(t_end-t_start).count()));


            for (int i = 0; i < boid_list.m_size; i++) {
                rlPushMatrix();
                    rlTranslatef(xs[i], ys[i], 0);
                    float angle = (atan2(vxs[i], vys[i]) * 360.) / (2 * PI);
                    rlRotatef(angle, 0, 0, -1);
                    DrawTriangle(triangle[2], triangle[1], triangle[0], WHITE);
                rlPopMatrix();
            }
            
            
            auto trav = boid_map.get_head_from_screen_space(GetScreenToWorld2D(GetMousePosition(), cam));
            while (trav != -1) {
                rlPushMatrix();
                    rlTranslatef(xs[trav], ys[trav], 0);
                    float angle = (atan2(vxs[trav], vys[trav]) * 360.) / (2 * PI);
                    rlRotatef(angle, 0, 0, -1);
                    DrawTriangle(triangle[2], triangle[1], triangle[0], BLUE);
                rlPopMatrix();

                trav = index_nexts[trav];
            }

            /*
            //Highlight selected boid
            if (selected_boid != nullptr) {
                rlPushMatrix();
                    rlTranslatef(selected_boid->x, selected_boid->y, 0);
                    float angle = (atan2(selected_boid->vx , selected_boid->vy) * 360.) / (2 * PI);
                    rlRotatef(angle, 0, 0, -1);
                    DrawTriangle(v3, v2, v1, RED);
                rlPopMatrix();
            }
            */
            //Draw grid
            for (int y = 0; y < boid_map.m_ysize; y++) {
                DrawLine(0, y*boid_map.m_cell_size, boid_map.m_xsize*boid_map.m_cell_size, y*boid_map.m_cell_size, GRAY);
            }
            for (int x = 0; x < boid_map.m_xsize; x++) {
                DrawLine(x*boid_map.m_cell_size, 0, x*boid_map.m_cell_size, boid_map.m_ysize * boid_map.m_cell_size, GRAY);
            }
            

            EndMode2D();
            
            rlImGuiBegin();
                UpdateRulesWindow(rules);
                UpdatePerfMonitor(perf_monitor, &rebuild_scheduled);
            rlImGuiEnd();

        EndDrawing();
    }

    CloseWindow();
    return 0;
}