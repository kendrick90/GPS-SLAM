#pragma once

// Cross-platform timing utilities
// Provides clock_gettime/CLOCK_MONOTONIC compatibility for Windows

#ifdef _WIN32
// Prevent Windows.h from defining min/max macros that conflict with std::min/std::max
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#include <time.h>  // For struct timespec on Windows (already defined in Windows SDK)

#ifndef CLOCK_MONOTONIC
#define CLOCK_MONOTONIC 1
#endif

// Windows doesn't have clock_gettime, so we implement it
inline int clock_gettime(int /*clock_id*/, struct timespec* tp) {
    static LARGE_INTEGER frequency;
    static BOOL initialized = FALSE;

    if (!initialized) {
        QueryPerformanceFrequency(&frequency);
        initialized = TRUE;
    }

    LARGE_INTEGER counter;
    QueryPerformanceCounter(&counter);

    tp->tv_sec = static_cast<time_t>(counter.QuadPart / frequency.QuadPart);
    tp->tv_nsec = static_cast<long>(
        ((counter.QuadPart % frequency.QuadPart) * 1000000000LL) / frequency.QuadPart
    );

    return 0;
}

#else
// POSIX systems - use native clock_gettime
#include <time.h>
#endif
