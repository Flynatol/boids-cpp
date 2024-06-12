#include <raylib.h>
#include <raymath.h>
#include <algorithm>
#include <cmath>

#include "boidmap.h"


BoidMap::BoidMap(const int height, const int width, const int cell_size) {
    m_ysize        = std::ceil(height / static_cast<float>(cell_size));
    m_xsize        = std::ceil(width / static_cast<float>(cell_size));
    m_boid_map     = new Boid[m_ysize * m_xsize]();
    m_index_buffer = new Boid[m_ysize * m_xsize];
    m_cell_size    = cell_size;
    safety         = new Lock[m_ysize * m_xsize];

    TraceLog(LOG_DEBUG, TextFormat("Initalized map of size (%d x %d)", m_xsize, m_ysize));
}

BoidMap::~BoidMap() {
    TraceLog(LOG_DEBUG, "Deallocated boid map");
    delete[] m_boid_map;
}
