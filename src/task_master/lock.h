#pragma once

#ifndef NOMINMAX
    #define NOMINMAX
#endif

#include <Windows.h>
#include <thread>
#include <cstdio>

typedef volatile long spinlock_t;

struct Lock {
    spinlock_t lock_internal = 0;

    inline void lock() {
        while (_interlockedbittestandset(&lock_internal, 0)) {}
    }

    inline void unlock() {
        _interlockedbittestandreset(&lock_internal, 0);
    }

    void lock_debug(uint32_t debug) {
        while (_interlockedbittestandset(&lock_internal, 0)) {
            char buffer[64];
            sprintf_s(buffer, "Failed to lock %u\n", debug);
            OutputDebugStringA(buffer);
            std::this_thread::yield();
        }
    }

    void unlock_debug(uint32_t debug) {
        _interlockedbittestandreset(&lock_internal, 0);
        char buffer[64];
        sprintf_s(buffer, "Unlocked %u\n", debug);
        OutputDebugStringA(buffer);
    }
};
