#pragma once

#include <cstdint>
#include <raylib.h>
#include <algorithm>

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
        
        inline Boid get_coord(const int y, const int x) const {
            //Little bit of safety, useful to minimize bounds checking code
            if (x < 0 || y < 0 || x >= m_xsize || y >= m_ysize) {
                return -1;
            }

            return m_boid_map[y * m_xsize + x];
        }

        //Returns the nearest map position (in case the supplied position is out of bounds of the map)
        //Returns the nearest map position (in case the supplied position is out of bounds of the map)
        inline Boid get_map_pos_nearest(const int32_t x, const int32_t y) const {
            int col = std::min((std::max(x, 0)), m_cell_size * m_xsize - 1) / m_cell_size;
            int row = std::min(std::max(y, 0), m_cell_size * m_ysize - 1) / m_cell_size;
            
            return row * m_xsize + col;
        }

        
        inline Boid get_map_pos_nearest2(const int32_t x, const int32_t y) const {
            int32_t min_x_0 = x - ((x >> 31) & x);
            int32_t x_size = m_cell_size * m_xsize - 1 - min_x_0;
            int32_t col = (min_x_0 + ((x_size >> 31) & x_size)) / m_cell_size;

            int32_t min_y_0 = y - ((y >> 31) & y);
            int32_t y_size = m_cell_size * m_ysize - 1 - min_y_0;
            int32_t row = (min_y_0 + ((y_size >> 31) & y_size)) / m_cell_size;

            return row * m_xsize + col;
        }
        

        inline Boid get_head_from_screen_space(const Vector2 pos) const {
            return m_boid_map[get_map_pos_nearest(pos.x, pos.y)];
        }

        inline Boid get_absolute(const int n) const {
            return m_boid_map[n];
        }

        inline int get_map_pos(const int x, const int y) const {
            int row = y / m_cell_size;
            int col = x / m_cell_size;

            return row * m_xsize + col;
        }
};
