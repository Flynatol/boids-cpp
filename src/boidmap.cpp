#include <raylib.h>
#include <raymath.h>
#include <algorithm>

#include "boidmap.h"


BoidMap::BoidMap(const int height, const int width, const int cell_size) {
    m_ysize = std::ceil(height / static_cast<float>(cell_size));
    m_xsize = std::ceil(width / static_cast<float>(cell_size));
    m_boid_map = new Boid[m_ysize * m_xsize]();
    m_index_buffer = new Boid[m_ysize * m_xsize];
    m_cell_size = cell_size;
    safety = new Lock[m_ysize * m_xsize];

    TraceLog(LOG_DEBUG, TextFormat("Initalized map of size (%d x %d)", m_xsize, m_ysize));
}

BoidMap::~BoidMap() {
    TraceLog(LOG_DEBUG, "Deallocated boid map");
    delete[] m_boid_map;
}

Boid BoidMap::get_coord(const int y, const int x) const {
    //Little bit of safety, useful to minimize bounds checking code
    if (x < 0 || y < 0 || x >= m_xsize || y >= m_ysize) {
        return -1;
    }

    return m_boid_map[y * m_xsize + x];
}

//Returns the nearest map position (in case the supplied position is out of bounds of the map)
Boid BoidMap::get_map_pos_nearest(const int x, const int y) const {
    int col = std::min(std::max(x, 0), m_cell_size * m_xsize - 1) / m_cell_size;
    int row = std::min(std::max(y, 0), m_cell_size * m_ysize - 1) / m_cell_size;

    return row * m_xsize + col;
}

Boid BoidMap::get_head_from_screen_space(const Vector2 pos) const {
    return m_boid_map[get_map_pos_nearest(pos.x, pos.y)];
}

Boid BoidMap::get_absolute(const int n) const {
    return m_boid_map[n];
}

int BoidMap::get_map_pos(const int x, const int y) const {
    int row = y / m_cell_size;
    int col = x / m_cell_size;

    return row * m_xsize + col;
}
