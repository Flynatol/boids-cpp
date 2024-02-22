#include <cstdint>
#include "tm_shared.h"

void doLotsOfWork () {
    volatile unsigned long long i;
    for (i = 0; i < 1000000ULL; ++i);
}

void function3(uint8_t thread_id, const char *string) {
    doLotsOfWork();
}

