#pragma once

#include <raylib.h>

struct Rules {
    float avoid_distance_squared;
    float avoid_factor;
    float sight_range;
    float sight_range_squared;
    float alignment_factor;
    float cohesion_factor;
    int edge_width;
    float edge_factor;
    float rand; 
    float homing;
    bool show_lines;
    int min_speed;
    int max_speed;
} typedef Rules;

class Ui {
    public:

    public:
        Ui();

        void Render(Camera2D &cam, Camera &camera, Rules &rules);

    private:
        void UpdateCameraWindow2d(Camera2D &cam);

        void UpdateCameraWindow(Camera &camera);

        void UpdatePerfMonitor();

        void UpdateRulesWindow(Rules &rules);
};
