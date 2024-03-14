#pragma once

#ifndef NOMINMAX
    #define NOMINMAX
#endif
#include <Windows.h>

#include <synchapi.h>
#include <mutex>
#include <thread>

#include <raylib.h>

/*
struct Lock {
    CRITICAL_SECTION cs;

    void lock() {
        EnterCriticalSection(&cs);
    }

    void unlock() {
        LeaveCriticalSection(&cs);
    }

    Lock() {
        InitializeCriticalSection(&cs);
    }

    ~Lock() {
        DeleteCriticalSection(&cs);
    }
};
*/

typedef volatile long spinlock_t;


struct Lock {
    spinlock_t lock_internal = 0;

    void lock() {
        while (_interlockedbittestandset(&lock_internal, 0)) {}
    }

    void lock_debug(uint32_t debug) {
        while (_interlockedbittestandset(&lock_internal, 0)) {
            TraceLog(LOG_DEBUG, TextFormat("Failed to lock %d", debug));
            std::this_thread::yield();
        }
    }

    void unlock() {
        _interlockedbittestandreset(&lock_internal, 0);
    }

    void unlock_debug(uint32_t debug) {
        _interlockedbittestandreset(&lock_internal, 0);
        TraceLog(LOG_DEBUG, TextFormat("Unlocked %d", debug));
    }
};


//typedef std::mutex Lock;