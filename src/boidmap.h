#pragma once

#include <cstdint>
#include <raylib.h>
#include ".\task_master\lock.h"

typedef int32_t Boid;

class BoidMap {
    public:
        int m_ysize;
        int m_xsize;
        int m_cell_size; 
        Boid *m_boid_map;
        Boid *m_index_buffer;
        Lock *safety;

    public:
        BoidMap() = delete;
        BoidMap(const int height, const int width, const int cell_size);
        
        ~BoidMap();
        
        Boid get_coord(const int y, const int x) const;

        //Returns the nearest map position (in case the supplied position is out of bounds of the map)
        int get_map_pos_nearest(const int x, const int y) const;

        Boid get_head_from_screen_space(const Vector2 pos) const;

        Boid get_absolute(const int n) const;

        int get_map_pos(const int x, const int y) const;
};
