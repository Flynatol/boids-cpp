#pragma once

#include <cstdint>
#include <string>

class BoidList;
class BoidMap;

class SimDumpWriter {
    public:
        SimDumpWriter(int argc, char* argv[]);

        void log_configuration() const;
        void tick(double frame_time_seconds, const BoidList& boid_list, const BoidMap& boid_map, float world_width, float world_height);

    private:
        bool write_dump(const BoidList& boid_list, const BoidMap& boid_map, float world_width, float world_height) const;

        double m_trigger_time_seconds = -1.0;
        double m_elapsed_seconds = 0.0;
        uint32_t m_frame_index = 0;
        bool m_written = false;
        std::string m_path = "boid_dump.bin";
};
