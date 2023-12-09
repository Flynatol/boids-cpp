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
const uint32_t NUM_BOIDS = 100000;
const uint16_t SIGHT_RANGE = 100;

#define WORLD_SIZE_MULT 10

static const Vector2 triangle[3] = {
    Vector2 {0.f, 2 * TRIANGLE_SIZE},
    Vector2 {-TRIANGLE_SIZE, -2 * TRIANGLE_SIZE},
    Vector2 {TRIANGLE_SIZE, -2 * TRIANGLE_SIZE}
};

//#define DEBUG(...) myfile << TextFormat(__VA_ARGS__) << '\n';
//#define DEBUG(...) ;
#define DEBUG(...) TraceLog(LOG_DEBUG, TextFormat(__VA_ARGS__));

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
    float homing = 0.0000311;
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


static Mesh GenMeshCustom(void)
{
    Mesh mesh = { 0 };
    mesh.triangleCount = 1;
    mesh.vertexCount = mesh.triangleCount*3;
    mesh.vertices = (float *)MemAlloc(mesh.vertexCount*3*sizeof(float));    // 3 vertices, 3 coordinates each (x, y, z)

    // Vertex at (0, 0, 0)
    mesh.vertices[0] = -TRIANGLE_SIZE;
    mesh.vertices[1] = 0;
    mesh.vertices[2] = -2 * TRIANGLE_SIZE;

    // Vertex at (1, 0, 2)
    mesh.vertices[3] = 0;
    mesh.vertices[4] = 0;
    mesh.vertices[5] = 2 * TRIANGLE_SIZE;

    // Vertex at (2, 0, 0)
    mesh.vertices[6] = TRIANGLE_SIZE;
    mesh.vertices[7] = 0;
    mesh.vertices[8] = -2 * TRIANGLE_SIZE;

    // Upload mesh data from CPU (RAM) to GPU (VRAM) memory
    UploadMesh(&mesh, false);

    return mesh;
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
                for (Boid nearby_boid = row_end; nearby_boid < row_end; nearby_boid++) {
                    int bytes_left = row_end - nearby_boid; 

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
                    
                    sep_x_vec = _mm256_add_ps(sep_x_vec, _mm256_and_ps(ads_mask, _mm256_mul_ps(xs_delta, ads_take_ds_sqr)));
                    sep_y_vec = _mm256_add_ps(sep_y_vec, _mm256_and_ps(ads_mask, _mm256_mul_ps(ys_delta, ads_take_ds_sqr)));

                    avg_vx_vec = _mm256_fmadd_ps(sna_fpmask, nearby_vxs_vec, avg_vx_vec);
                    avg_vy_vec = _mm256_fmadd_ps(sna_fpmask, nearby_vys_vec, avg_vy_vec);

                    avg_x_vec = _mm256_fmadd_ps(sna_fpmask, nearby_xs_vec, avg_x_vec);
                    avg_y_vec = _mm256_fmadd_ps(sna_fpmask, nearby_ys_vec, avg_y_vec);

                    isc = _mm256_add_epi32(isc, (__m256i) _mm256_and_ps((__m256) read_mask, (__m256) _mm256_cvtps_epi32(sna_fpmask)));
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

inline Matrix YAxisRotate(float angle) {
    Matrix result = { 0 };

    const float x = 0, y = 1, z = 0;

    const float lengthSquared = 1;

    const float sinres = sinf(angle);
    const float cosres = cosf(angle);
    const float t = 1.0f - cosres;

    result.m0 = cosres;
    result.m1 = 0.0f;
    result.m2 = -sinres;
    result.m3 = 0.0f;

    result.m4 = 0.0f;
    result.m5 = 1.0f;
    result.m6 = 0.0f;
    result.m7 = 0.0f;

    result.m8 = sinres;
    result.m9 = 0.0f;
    result.m10 = cosres;
    result.m11 = 0.0f;

    result.m12 = 0.0f;
    result.m13 = 0.0f;
    result.m14 = 0.0f;
    result.m15 = 1.0f;

    return result;
}

void update_non_interacting(const BoidMap& boid_map, const Rules& rules, const BoidList& boid_list, Matrix* transforms) {
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

        speed = speed + (!speed);

        float ispeed = 1.0f/speed;
        vxs[boid] = vxs[boid]*ispeed * 3;
        vys[boid] = vys[boid]*ispeed * 3;
        
        
        xs[boid] += vxs[boid];
        ys[boid] += vys[boid];

        //TODO optimize this -- currently very expensive
        //In current form doesn't lend itself to SIMD
        Matrix translation = MatrixTranslate(xs[boid], 0., ys[boid]);
        Vector3 axis = (Vector3){ 0., 1., 0.};
        float angle = (atan2(vxs[i], vys[i]));
        Matrix rotation = YAxisRotate(angle);
        
        transforms[i] = MatrixMultiply(rotation, translation);
        
    }
}

void update_boids(const BoidMap& boid_map, const Rules& rules, Boid selected_boid, const BoidList& boid_list, Matrix* transforms) {
    auto t_start = std::chrono::high_resolution_clock::now();

    for (int y = 0; y < boid_map.m_ysize; y++) {
        for (int x = 0; x < boid_map.m_xsize; x++) {
            update_cell(boid_map, x, y, rules, selected_boid, boid_list);
        }
    }

    auto t_mid = std::chrono::high_resolution_clock::now();

    update_non_interacting(boid_map, rules, boid_list, transforms);

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

void UpdateCameraWindow(Camera *camera){
    ImGui::Begin("Camera Settings");
    ImGui::SliderFloat("x", &camera->position.x, -1000., 1000., "%0.9f");
    ImGui::SliderFloat("y", &camera->position.y, -1000., 1000., "%0.9f");
    ImGui::SliderFloat("z", &camera->position.z, -1000., 1000., "%0.9f");
    ImGui::SliderFloat("fovy", &camera->fovy, 0., 32., "%0.9f");

    ImGui::SliderFloat("x_target", &camera->target.x, -1000., 1000., "%0.9f");
    ImGui::SliderFloat("y_target", &camera->target.y, -1000., 1000., "%0.9f");
    ImGui::SliderFloat("z_target", &camera->target.z, -1000., 1000., "%0.9f");

    ImGui::End();

    //camera->target = Vector3Add(camera->position, Vector3 {0., -100., 0.});
}

void UpdateCameraWindow2d(Camera2D *cam) {
    ImGui::Begin("Camera2d Settings");
    ImGui::SliderFloat("x_offset", &cam->offset.x, -1000., 1000., "%0.9f");
    ImGui::SliderFloat("y_offset", &cam->offset.y, -1000., 1000., "%0.9f");
    ImGui::SliderFloat("x_target", &cam->target.x, -1000., 1000., "%0.9f");
    ImGui::SliderFloat("y_target", &cam->target.y, -1000., 1000., "%0.9f");
    ImGui::SliderFloat("zoom", &cam->zoom, 0., 32., "%0.9f");
    ImGui::End();
}

int main () {
    InitWindow(screen_width, screen_height, "RayLib Boids!");
    SetTargetFPS(165);

    SetTraceLogLevel(LOG_ALL);
    TraceLog(LOG_DEBUG, TextFormat("Boid size is: %d bytes", sizeof(Boid)));
    
    std::time_t result = std::time(nullptr);
    TraceLog(LOG_DEBUG, TextFormat("Time is: %s", std::asctime(std::localtime(&result)) ));

   

    Mesh tri = GenMeshCustom();
    Matrix *transforms = (Matrix *)RL_CALLOC(NUM_BOIDS, sizeof(Matrix));

    Shader shader = LoadShader(TextFormat("resources/shaders/glsl%i/lighting_instancing.vs", 330),
                               TextFormat("resources/shaders/glsl%i/lighting.fs", 330));

    shader.locs[SHADER_LOC_VECTOR_VIEW] = GetShaderLocation(shader, "viewPos");
    shader.locs[SHADER_LOC_MATRIX_MODEL] = GetShaderLocationAttrib(shader, "instanceTransform");

    Material matInstances = LoadMaterialDefault();
    matInstances.shader = shader;
    matInstances.maps[MATERIAL_MAP_DIFFUSE].color = WHITE;

    BoidList boid_list(NUM_BOIDS);

    Rules rules;
    PerfMonitor perf_monitor;    

    myfile.open("log.log");

    int world_width = screen_width * WORLD_SIZE_MULT;
    int world_height = screen_height * WORLD_SIZE_MULT;

    Camera2D cam = { 0 };
    cam.zoom = static_cast<float>(screen_width) / world_width;

    Camera camera = {
        .position = (Vector3){world_width/2., 200., world_height/2},            
        .target = (Vector3){world_width/2, 0.0f, world_height/2},                
        .up = (Vector3){ 0.0f, 0.0f, -1.0f },                    
        .fovy = world_height,                                          
        .projection = CAMERA_ORTHOGRAPHIC,                      
    };


    const int CELL_WIDTH = 100;
    assert(CELL_WIDTH >= rules.sight_range);
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
    
    /*
    for (int i = 0; i < NUM_BOIDS; i++)
    {   
        Matrix translation = MatrixTranslate((float)GetRandomValue(-(2560/2), (2560/2)), 0., (float)GetRandomValue(-(1440/2), (1440/2)));
        Vector3 axis = Vector3Normalize((Vector3){ 0., 1., 0. });
        float angle = (float)GetRandomValue(0, 10)*DEG2RAD;
        Matrix rotation = MatrixRotate(axis, angle);
        transforms[i] = MatrixMultiply(rotation, translation);
    }
    */

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

        float cameraPos[3] = { camera.position.x, camera.position.y, camera.position.z };
        SetShaderValue(shader, shader.locs[SHADER_LOC_VECTOR_VIEW], cameraPos, SHADER_UNIFORM_VEC3);
 
        update_boids(boid_map, rules, selected_boid, boid_list, transforms);



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

            auto new_pos = Vector3 {
                .x = (screen_width * 0.5 - cam.offset.x) / cam.zoom + cam.target.x,
                .y = camera.position.y,
                .z = (screen_height * 0.5 - cam.offset.y) / cam.zoom + cam.target.y,
            };
            
            camera.position = new_pos;
            camera.target = Vector3 {camera.position.x, 0.f, camera.position.z};
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
        
        BeginDrawing();
            ClearBackground(BLACK);

            BeginMode3D(camera);
                DrawMeshInstanced(tri, matInstances, transforms, NUM_BOIDS);
            EndMode3D();

            BeginMode2D(cam);
                /*
                for (int i = 0; i < boid_list.m_size; i++) {
                    rlPushMatrix();
                        rlTranslatef(xs[i], ys[i], 0);
                        float angle = (atan2(vxs[i], vys[i]) * 360.) / (2 * PI);
                        rlRotatef(angle, 0, 0, -1);
                        DrawTriangle(triangle[2], triangle[1], triangle[0], WHITE);
                    rlPopMatrix();
                } 
                */
            
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
            
            rlImGuiBegin();
                UpdateRulesWindow(rules);
                UpdatePerfMonitor(perf_monitor, &rebuild_scheduled);
                UpdateCameraWindow(&camera);
                UpdateCameraWindow2d(&cam);
            rlImGuiEnd();

        EndDrawing();
    }

    CloseWindow();
    return 0;
}