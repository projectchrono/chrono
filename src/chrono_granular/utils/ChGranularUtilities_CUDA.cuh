
#pragma once

#include <cuda_runtime.h>
#include <cstdio>
#include <cstdlib>

/** Set up some error checking mechanism to ensure CUDA didn't complain about things.
 *   This approach suggested <a
 * href="https://stackoverflow.com/questions/14038589/what-is-the-canonical-way-to-check-for-errors-using-the-cuda-runtime-api">elsewhere</a>.
 *   Some nice suggestions for how to use the mechanism are provided at the above link.
 */
#define gpuErrchk(ans) \
    { gpuAssert((ans), __FILE__, __LINE__); }
inline void gpuAssert(cudaError_t code, const char* file, int line, bool abort = true) {
    if (code != cudaSuccess) {
        fprintf(stderr, "GPUassert: %s %s %d\n", cudaGetErrorString(code), file, line);
        if (abort)
            exit(code);
    }
}

// Add verbose checks easily
#define VERBOSE_PRINTF(...)  \
    if (verbose_runtime) {   \
        printf(__VA_ARGS__); \
    }

// Print a user-given error message and crash
#define ABORTABORTABORT(...) \
    {                        \
        printf(__VA_ARGS__); \
        __threadfence();     \
        cub::ThreadTrap();   \
    }
