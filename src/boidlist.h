#pragma once
#include <cstdint>

struct BoidStore {
    float *xs;            
    float *ys;            
    float *vxs;          
    float *vys;
    int32_t *index_next;  
    int32_t *homes;       
    int32_t *depth;

    BoidStore(size_t to_alloc);
    ~BoidStore();      
} typedef BoidStore;

BoidStore* new_boidstore(size_t to_alloc);

void free_boidstore_members(BoidStore *boid_store);

class BoidList {
    public:
        BoidStore *m_boid_store;
        BoidStore *m_backbuffer;
                             
        int m_size;

    public:
        BoidList(int size);
        ~BoidList();
};