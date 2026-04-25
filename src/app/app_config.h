#pragma once

#include <chrono>
#include <cstdint>

#include <raylib.h>

#ifdef TRACY_ENABLE
    #ifndef TRACY_CALLSTACK
        #define TRACY_CALLSTACK 32
    #endif
#endif

#include <tracy/Tracy.hpp>

constexpr float TRIANGLE_SIZE = 5.f;
constexpr uint16_t SIGHT_RANGE = 100;

constexpr int FRAME_RATE_LIMIT = 165;
constexpr double BOID_DENSITY_MAGIC_NUMBER = 2304.0;
constexpr int CELL_WIDTH = 100;

constexpr int SHADER_LOC_BOID_X = 26;
constexpr int SHADER_LOC_BOID_Y = 27;
constexpr int SHADER_LOC_BOID_VX = 28;
constexpr int SHADER_LOC_BOID_VY = 29;

//#define DEBUG_ENABLED
//#define RUNNER_STORE
#define APPROXIMATE_NEIGHBOR_AVERAGES

#ifdef DEBUG_ENABLED
    #define TIME_NOW std::chrono::high_resolution_clock::now()
    #define DEBUG(...) TraceLog(LOG_DEBUG, TextFormat(__VA_ARGS__));
#endif
#ifndef DEBUG_ENABLED
    #define TIME_NOW (0.f)
    #define DEBUG(...)
#endif
