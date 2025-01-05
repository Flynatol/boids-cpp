#include <stdlib.h>
#include <raylib.h>
#include <array>

#include "boidlist.h"

BoidList::BoidList(int size) {
    // We need to round up the amount of memory we are allocating as 
    // we will be reading these as blocks of 8 floats (__m256)

    // We should allocate at least 56 extra bytes so if we read the last real 
    // float as the begining of an 8 byte block we do not overrun

    
    int to_alloc = size + 7;

    m_boid_store = new BoidStore(to_alloc);
    m_backbuffer = new BoidStore(to_alloc);

    m_size = size;

    TraceLog(LOG_DEBUG, TextFormat("Initialized Boid List of size %d", size));
}

BoidList::~BoidList() {
    delete m_boid_store;
    delete m_backbuffer;

    TraceLog(LOG_DEBUG, "Deallocated boid list");
}

BoidStore::BoidStore(std::size_t to_alloc) 
    :   xs          ((float *)   _aligned_malloc(to_alloc * sizeof(float), 64)),
        ys          ((float *)   _aligned_malloc(to_alloc * sizeof(float), 64)),
        vxs         ((float *)   _aligned_malloc(to_alloc * sizeof(float), 64)),
        vys         ((float *)   _aligned_malloc(to_alloc * sizeof(float), 64)),
        index_next  ((int32_t *) _aligned_malloc(to_alloc * sizeof(int32_t), 64)),
        homes       ((int32_t *) _aligned_malloc(to_alloc * sizeof(int32_t), 64)),
        depth       ((int32_t *) _aligned_malloc(to_alloc * sizeof(int32_t), 64)) 
{
    memset(xs, 0, to_alloc * sizeof(float));
    memset(ys, 0, to_alloc * sizeof(float));
    memset(vxs, 0, to_alloc * sizeof(float));
    memset(vys, 0, to_alloc * sizeof(float));
}

BoidStore::~BoidStore() {
    _aligned_free(this->xs);
    _aligned_free(this->ys);
    _aligned_free(this->vxs);
    _aligned_free(this->vys);
    _aligned_free(this->index_next);
    _aligned_free(this->homes);
    _aligned_free(this->depth);
    
    TraceLog(LOG_DEBUG, "Deallocated aligned memory");
}