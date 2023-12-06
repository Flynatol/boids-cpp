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

#include "imgui.h"

#define NO_FONT_AWESOME

#include <rlImGui.h>

const float TRIANGLE_SIZE = 5.f;
const uint32_t NUM_BOIDS = 75000;
const uint16_t SIGHT_RANGE = 100;



//Initial plan
//Hybrid linked list structure with periodic reodering to increase cache locality at expensive of intermittent memory spikes.

struct boid {   //Minimum size 16 bytes
    struct boid *next;  // 8 bytes  --- 8 remaining
    float x;            // 2 bytes  --- 6 remaining
    float y;            // 2 bytes  --- 4 remaining
    float vx;          // 1 byte   --- 3 remaining //TODO DEFLOAT
    float vy;          // 1 byte   --- 2 remaining
    uint8_t home;                    // What to do with our last two bytes? Maybe different variants
} typedef Boid;

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

class BoidList {
    public:
        Boid *m_boid_buffer;
        int m_size;

    public:
        BoidList() = delete;
        BoidList(int size) {    
            m_boid_buffer = new Boid[size];
            m_size = size;
        }
        
        ~BoidList() {
            TraceLog(LOG_DEBUG, "Deallocated boid list");
            delete[] m_boid_buffer;
        }
};

class BoidMap {
    public:
        int m_ysize;
        int m_xsize;
        int m_cell_size; 
        Boid **m_boid_map;

    public:
        BoidMap() = delete;
        BoidMap(const int height, const int width, const int cell_size) { 
            m_ysize = std::ceil(height / static_cast<float>(cell_size));
            m_xsize = std::ceil(width / static_cast<float>(cell_size));
            m_boid_map = new Boid*[m_ysize * m_xsize]();
            m_cell_size = cell_size;

            TraceLog(LOG_DEBUG, TextFormat("Initalized map of size (%d x %d)", m_ysize, m_xsize));
        }
        
        ~BoidMap() {
            TraceLog(LOG_DEBUG, "Deallocated boid map");
            delete[] m_boid_map;
        }
        
        Boid* get_coord(const int y, const int x) const {
            //Little bit of safety, useful to minimize bounds checking code
            if (x < 0 || y < 0 || x >= m_xsize || y >= m_ysize) {
                return nullptr;
            }

            return m_boid_map[y * m_xsize + x];
        }

        //Returns the nearest map position (in case the supplied position is out of bounds of the map)
        int get_map_pos_nearest(const int x, const int y) const {
            int col = std::min(std::max(x, 0), m_cell_size * m_xsize - 1) / m_cell_size;
            int row = std::min(std::max(y, 0), m_cell_size * m_ysize - 1) / m_cell_size;

            return row * m_xsize + col;
        }

        Boid* get_head_from_screen_space(const Vector2 pos) const {
            return m_boid_map[get_map_pos_nearest(pos.x, pos.y)];
        }

        Boid* get_absolute(const int n) const {
            return m_boid_map[n];
        }

        Boid** get_pointer(const int x, const int y) const {
            return nullptr;
        }

        int get_map_pos(const int x, const int y) const {
            int row = y / m_cell_size;
            int col = x / m_cell_size;

            return row * m_xsize + col;
        }
};


void rebuild_list(BoidList& boid_list, BoidMap& boid_map) {
    //Idea todo:
    //Track how many children (depth) of each boid
    //Then we can rebuild multithreaded.

    Boid *new_boid_buffer = new Boid[boid_list.m_size];
    int index = 0;

    for (int i = 0; i < boid_map.m_xsize * boid_map.m_ysize; i++) {
        Boid *current = boid_map.get_absolute(i);
        
        while (current != nullptr) {
            new_boid_buffer[index] = *current;
            current = current->next;
            index++;
        }
    }

    delete[] boid_list.m_boid_buffer;
    boid_list.m_boid_buffer = new_boid_buffer;
}

void populate_map(BoidList& boid_list, BoidMap& map) {
    for (int i = 0; i < map.m_xsize * map.m_ysize; i++) {
        map.m_boid_map[i] = nullptr;
    }

    for (int i = 0; i < boid_list.m_size; i++) {
        Boid *current = &boid_list.m_boid_buffer[i];
        int map_pos = map.get_map_pos_nearest(current->x, current->y);
        Boid *head = map.m_boid_map[map_pos];
        map.m_boid_map[map_pos] = current;
        current->next = head;

        //cout << "i:" << i << "c: " << current << "\t n:" << current->next << endl;
    }
}


void print_boid(Boid *boid) {
    TraceLog(LOG_DEBUG, TextFormat("{next: %p, x: %d, y: %d}", boid->next, boid->x, boid->y));
}

void do_something_to_cell(BoidMap& map, const int x, const int y, const Rules& rules, Boid* selected_boid) {
    Boid *cell_to_update = map.get_coord(y, x);

    Boid *neighbours[9] =  {
        map.get_coord(y-1, x-1), map.get_coord(y-1, x), map.get_coord(y-1, x+1),
        map.get_coord(y,   x-1), map.get_coord(y,   x), map.get_coord(y,   x+1),
        map.get_coord(y+1, x-1), map.get_coord(y+1, x), map.get_coord(y+1, x+1),
    };

    //int limit = 100;
    //Boid **test = new Boid*[limit];
    std::vector<Boid*> cell_wide_candidates;
    cell_wide_candidates.reserve(100);

    for (Boid *n : neighbours) {
        //if (track >= limit - 1) break;
        Boid *current = n;
        while (current != nullptr) {
            //error_test.push_back(track2);
            cell_wide_candidates.push_back(current);
            //test[track] = current;
            //cout << "\t" << current << "    size" << cell_wide_candidates.size() << endl;
            //track++;

            //This is a reasonable use of goto, need to break out of nested loops.
            //if (track >= limit) goto double_break;

            current = current->next;
            //cout << track << '\n';
        }
    }
    double_break:


    //Now we have most the information we need
    //Do the calculations for each boid    
    Boid *current_boid = cell_to_update;

    while (current_boid != nullptr) {
        //Variables for tracking seperation force
        float sep_x = 0, sep_y = 0;

        //Variables for tracking aligment
        float avg_vx = 0, avg_vy = 0;

        //Variables for tracking cohesion
        float avg_x = 0, avg_y = 05;

        uint_fast16_t in_sight_counter = 0; 

        //for (int i = 0; i < track; i++) {
            //auto nearby_boid = test[i];
            //DrawLine(current_boid->x, current_boid->y, nearby_boid->x, nearby_boid->y, YELLOW);
        for (auto nearby_boid : cell_wide_candidates) {
            int_fast32_t dist_squared = (current_boid->x - nearby_boid->x) * (current_boid->x - nearby_boid->x) + (current_boid->y - nearby_boid->y) * (current_boid->y - nearby_boid->y);
            if (dist_squared < rules.sight_range_squared) {
                if (dist_squared < rules.avoid_distance_squared) {
                    //If too close
                    //We are currently counting this boid. testing required.
                    sep_x += (current_boid->x - nearby_boid->x) * (rules.avoid_distance_squared - dist_squared) * (rules.avoid_distance_squared - dist_squared);
                    sep_y += (current_boid->y - nearby_boid->y) * (rules.avoid_distance_squared - dist_squared) * (rules.avoid_distance_squared - dist_squared);

                    if (rules.show_lines || (current_boid == selected_boid)) DrawLine(current_boid->x, current_boid->y, nearby_boid->x, nearby_boid->y, RED);
                } else {
                    //If in sight
                    avg_vx += nearby_boid->vx;
                    avg_vy += nearby_boid->vy;
                    avg_x  += nearby_boid->x;
                    avg_y  += nearby_boid->y;
                    in_sight_counter++;
                    if (rules.show_lines || (current_boid == selected_boid)) DrawLine(current_boid->x, current_boid->y, nearby_boid->x, nearby_boid->y, GREEN);
                }
            }
        }

        //Avoidance
        current_boid->vx += sep_x * rules.avoid_factor;
        current_boid->vy += sep_y * rules.avoid_factor;

        //TODO see if this is more performant branchless
        if (in_sight_counter) {

            //Alignment
            avg_vx = avg_vx/in_sight_counter;
            avg_vy = avg_vy/in_sight_counter;

            current_boid->vx += (avg_vx - current_boid->vx) * rules.alignment_factor;
            current_boid->vy += (avg_vy - current_boid->vy) * rules.alignment_factor;

            //Cohesion
            avg_x = avg_x/in_sight_counter;
            avg_y = avg_y/in_sight_counter;

            current_boid->vx += (avg_x - current_boid->x) * rules.cohesion_factor;
            current_boid->vy += (avg_y - current_boid->y) * rules.cohesion_factor;
        }


        if ((rules.show_lines || (current_boid == selected_boid)) && current_boid->next != nullptr) {
            DrawLine(current_boid->x, current_boid->y, current_boid->next->x, current_boid->next->y, BLUE);
        }

        if (current_boid == selected_boid) TraceLog(LOG_DEBUG, TextFormat("FORCES: SEP: [%f, %f], ALIGN: [%f, %f], COHESION: [%f, %f]", sep_x * rules.avoid_factor, sep_y * rules.avoid_factor, (avg_vx - current_boid->vx) * rules.alignment_factor, (avg_vy - current_boid->vy) * rules.alignment_factor, (avg_x - current_boid->x) * rules.cohesion_factor, (avg_y - current_boid->y) * rules.cohesion_factor));
        current_boid = current_boid->next;
    }

    //delete[] test;
}

void push_boids(const BoidList& boids) {
    for (int i = 0; i < boids.m_size; i++) {
        boids.m_boid_buffer[i].x += boids.m_boid_buffer[i].vx;
        boids.m_boid_buffer[i].y += boids.m_boid_buffer[i].vy;
    }
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

    //std::uniform_real_distribution<float> distribution(rules.edge_width, world_width - rules.edge_width);

    //std::uniform_real_distribution<float>

    //Populate some test boids
    for (int i = 0; i < boid_list.m_size; i++) {
        Boid b {
            .next = nullptr,
            .x = width_distribution(generator),
            .y = height_distribution(generator),
            .vx = (std::rand() % 3) - 1,
            .vy = (std::rand() % 3) - 1,
            .home = rand() % 144,
        };

        boid_list.m_boid_buffer[i] = b;
    }


    populate_map(boid_list, boid_map);


    rlImGuiSetup(true);
    

    Boid *selected_boid = nullptr;
    bool rebuild_scheduled = false;

    while (WindowShouldClose() == false){
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
            
            if (rebuild_scheduled) {
                rebuild_list(boid_list, boid_map);
                selected_boid = nullptr;
                rebuild_scheduled = false;
            }
            
            populate_map(boid_list, boid_map);

            if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT)) {
                auto mouse_pos = GetScreenToWorld2D(GetMousePosition(), cam);
                Boid* current = boid_map.get_head_from_screen_space(mouse_pos);
                float min_dist = -1;
                Boid* nearest = nullptr;

                while (current != nullptr) {
                    float dist = Vector2DistanceSqr(mouse_pos, Vector2 {current->x, current->y});
                    if (min_dist == -1 || dist < min_dist) {
                        nearest = current;
                        min_dist = dist;
                    }
                    current = current->next;
                }

                selected_boid = nearest;
                //Traverse node that contains mouse
            }

            for (int y = 0; y < boid_map.m_ysize; y++) {
                for (int x = 0; x < boid_map.m_xsize; x++) {
                    do_something_to_cell(boid_map, x, y, rules, selected_boid);
                }
            }
            
            //This shoudl be easy to be simple to vectorize
            for (int i = 0; i < boid_list.m_size; i++) {
                Boid &boid = boid_list.m_boid_buffer[i];   

                //Window edges, todo replace with old function
                if (boid.x > (world_width - rules.edge_width)) {
                    boid.vx -= rules.edge_factor;
                }
                if (boid.x < rules.edge_width) {
                    boid.vx += rules.edge_factor;
                } 
                if (boid.y > (world_height - rules.edge_width)) {
                    boid.vy -= rules.edge_factor;
                } 
                if (boid.y < rules.edge_width) {
                    boid.vy += rules.edge_factor;
                } 


                //Apply some randomness
                float rx = static_cast <float> (rand()) / (static_cast <float> (RAND_MAX/2)) - 1;
                float ry = static_cast <float> (rand()) / (static_cast <float> (RAND_MAX/2)) - 1;

                boid.vx += rx * rules.rand;
                boid.vy += ry * rules.rand;


                //Apply homing
                int hy = boid.home / 16;
                int hx = boid.home % 16;

                float px = hx * ((world_width - rules.edge_width * 2) /  16) + rules.edge_width;
                float py = hy * ((world_height - rules.edge_width * 2) /  9) + rules.edge_width;


                float dx = px - boid.x;
                float dy = py - boid.y;

                boid.vx += dx * rules.homing;
                boid.vy += dy * rules.homing;

                float speed = sqrtf((boid.vx*boid.vx) + (boid.vy*boid.vy));

                if (speed > 0)
                {
                    float ispeed = 1.0f/speed;
                    boid.vx = boid.vx*ispeed * 3;
                    boid.vy = boid.vy*ispeed * 3;
                }
                
                boid.x += boid.vx;
                boid.y += boid.vy;
            }

            Vector2 ball_pos2 {0, 0};

            Vector2 v1 {0.f, 2 * TRIANGLE_SIZE};
            Vector2 v2 {-TRIANGLE_SIZE, -2 * TRIANGLE_SIZE};
            Vector2 v3 {TRIANGLE_SIZE, -2 * TRIANGLE_SIZE};

            
            for (int i = 0; i < boid_list.m_size; i++) {
                rlPushMatrix();
                    rlTranslatef(boid_list.m_boid_buffer[i].x, boid_list.m_boid_buffer[i].y, 0);
                    float angle = (atan2(boid_list.m_boid_buffer[i].vx , boid_list.m_boid_buffer[i].vy) * 360.) / (2 * PI);
                    rlRotatef(angle, 0, 0, -1);
                    DrawTriangle(v3, v2, v1, WHITE);
                rlPopMatrix();
            }

            
            auto trav = boid_map.get_head_from_screen_space(GetScreenToWorld2D(GetMousePosition(), cam));
            while (trav != nullptr) {
                rlPushMatrix();
                    rlTranslatef(trav->x, trav->y, 0);
                    float angle = (atan2(trav->vx , trav->vy) * 360.) / (2 * PI);
                    rlRotatef(angle, 0, 0, -1);
                    DrawTriangle(v3, v2, v1, BLUE);
                rlPopMatrix();

                trav = trav->next;
            }

            //Highlight selected boid
            if (selected_boid != nullptr) {
                rlPushMatrix();
                    rlTranslatef(selected_boid->x, selected_boid->y, 0);
                    float angle = (atan2(selected_boid->vx , selected_boid->vy) * 360.) / (2 * PI);
                    rlRotatef(angle, 0, 0, -1);
                    DrawTriangle(v3, v2, v1, RED);
                rlPopMatrix();
            }

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