#include "app/sim_dump.h"

#include <cstdlib>
#include <cstdint>
#include <fstream>

#include <raylib.h>

#include "sim/boidlist.h"
#include "sim/boidmap.h"

namespace {

struct DumpHeader {
    uint32_t magic;
    uint32_t version;
    uint32_t boid_count;
    uint32_t cell_count;
    uint32_t map_xsize;
    uint32_t map_ysize;
    uint32_t cell_size;
    uint32_t frame_index;
    double sim_time_seconds;
    float world_width;
    float world_height;
};

constexpr uint32_t BOID_DUMP_MAGIC = 0x42444942; // "BIDB"
constexpr uint32_t BOID_DUMP_VERSION = 1;

}

SimDumpWriter::SimDumpWriter(int argc, char* argv[]) {
    if (argc > 2) {
        m_trigger_time_seconds = std::atof(argv[2]);
    }

    if (argc > 3) {
        m_path = argv[3];
    }
}

void SimDumpWriter::log_configuration() const {
    if (m_trigger_time_seconds >= 0.0) {
        TraceLog(LOG_INFO, TextFormat("Sim dump armed: t=%0.3fs -> %s", m_trigger_time_seconds, m_path.c_str()));
    }
}

void SimDumpWriter::tick(
    const double frame_time_seconds,
    const BoidList& boid_list,
    const BoidMap& boid_map,
    const float world_width,
    const float world_height
) {
    m_elapsed_seconds += frame_time_seconds;

    if (!m_written && m_trigger_time_seconds >= 0.0 && m_elapsed_seconds >= m_trigger_time_seconds) {
        if (write_dump(boid_list, boid_map, world_width, world_height)) {
            TraceLog(LOG_INFO, TextFormat("Wrote sim dump at t=%0.3fs to %s", m_elapsed_seconds, m_path.c_str()));
            m_written = true;
        } else {
            TraceLog(LOG_ERROR, TextFormat("Failed to write sim dump to %s", m_path.c_str()));
        }
    }

    m_frame_index++;
}

bool SimDumpWriter::write_dump(
    const BoidList& boid_list,
    const BoidMap& boid_map,
    const float world_width,
    const float world_height
) const {
    std::ofstream out(m_path, std::ios::binary);
    if (!out) {
        return false;
    }

    const BoidStore* store = boid_list.m_boid_store;
    const uint32_t boid_count = boid_list.m_size;
    const uint32_t cell_count = static_cast<uint32_t>(boid_map.m_xsize * boid_map.m_ysize);

    DumpHeader header {
        .magic = BOID_DUMP_MAGIC,
        .version = BOID_DUMP_VERSION,
        .boid_count = boid_count,
        .cell_count = cell_count,
        .map_xsize = static_cast<uint32_t>(boid_map.m_xsize),
        .map_ysize = static_cast<uint32_t>(boid_map.m_ysize),
        .cell_size = static_cast<uint32_t>(boid_map.m_cell_size),
        .frame_index = m_frame_index,
        .sim_time_seconds = m_elapsed_seconds,
        .world_width = world_width,
        .world_height = world_height,
    };

    out.write(reinterpret_cast<const char*>(&header), sizeof(header));
    out.write(reinterpret_cast<const char*>(boid_map.m_boid_map), sizeof(Boid) * cell_count);

    for (uint32_t cell = 0; cell < cell_count; cell++) {
        const Boid head = boid_map.m_boid_map[cell];
        const int32_t count = (head == -1) ? 0 : store->depth[head];
        out.write(reinterpret_cast<const char*>(&count), sizeof(count));
    }

    out.write(reinterpret_cast<const char*>(store->xs), sizeof(float) * boid_count);
    out.write(reinterpret_cast<const char*>(store->ys), sizeof(float) * boid_count);
    out.write(reinterpret_cast<const char*>(store->vxs), sizeof(float) * boid_count);
    out.write(reinterpret_cast<const char*>(store->vys), sizeof(float) * boid_count);
    out.write(reinterpret_cast<const char*>(store->homes), sizeof(int32_t) * boid_count);
    out.write(reinterpret_cast<const char*>(store->depth), sizeof(int32_t) * boid_count);
    out.write(reinterpret_cast<const char*>(store->index_next), sizeof(int32_t) * boid_count);

    return out.good();
}
