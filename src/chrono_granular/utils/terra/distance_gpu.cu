// Written by Colin Vanden Heuvel and Conlain Kelly as part of the
// Chrono granular project. I hope someome that I don't know reads
// this code someday.

#include <cuda.h>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <iostream>
#include "timer.h"

#define THOUSAND 1000ul
#define MILLION 1000000ul
#define BILLION 1000000000ul
#define RADIUS .1f

// Number of points to compute
#define N (2u << 25)
#define THREADS_PER_BLOCK 256
// Reference point (index in list of points)
#define REF 200

// Amount of times to run to compute averages
#define NRUNS 40ul

__global__ void testIntrinsic(double* x, double* y, double* z, double* distances) {
    int i = threadIdx.x + blockIdx.x * blockDim.x;
    double x1 = x[i];
    double y1 = y[i];
    double z1 = z[i];
    if (i >= N) {
        return;
    }
    distances[i] = norm3d(x1, y1, z1);
}

__global__ void testRegular(double* x, double* y, double* z, double* distances) {
    int i = threadIdx.x + blockIdx.x * blockDim.x;
    double x1 = x[i];
    double y1 = y[i];
    double z1 = z[i];
    if (i >= N) {
        return;
    }
    distances[i] = sqrt(x1 * x1 + y1 * y1 + z1 * z1);
}

// __global__ void findDiff(double* distances, double* distances2) {
//     int i = threadIdx.x + blockIdx.x * blockDim.x;
//     if (i >= N) {
//         return;
//     }
//     distances[i] = sqrt(x[i] * x[i] + y[i] * y[i] + z[i] * z[i]);
// }

// Get me a random float between 0 and 1
inline double getRand() {
    int r = rand();  // returns a pseudo-random integer between 0 and RAND_MAX
    return (double)r / RAND_MAX;
}

// Distance for struct of arrays
inline float getDistance(float x1, float x2, float y1, float y2, float z1, float z2) {
    float dx = x2 - x1;
    float dy = y2 - y1;
    float dz = z2 - z1;
    return sqrt(dx * dx + dy * dy + dz * dz);
}

// Nice for legibility
void printDelim() {
    printf("--------------------------------------------\n");
}

// Print an error for the validation
void err(int i1, int i2, int id) {
    std::cerr << "ERROR! test " << i1 << " vs test " << i2 << ",  index" << id << std::endl;
}

// Test a bunch of cases
int main(int argc, char** argv) {
    // Allocate x, y, z for struct of arrays
    double* x;
    double* y;
    double* z;

    cudaMallocManaged(&x, N * sizeof(double), cudaMemAttachGlobal);
    cudaMallocManaged(&y, N * sizeof(double), cudaMemAttachGlobal);
    cudaMallocManaged(&z, N * sizeof(double), cudaMemAttachGlobal);

    // Initialize points
    printf("Initializing points\n");
    for (size_t i = 0; i < N; i++) {
        x[i] = getRand();
        y[i] = getRand();
        z[i] = getRand();
    }
    printf("%lu points to compute\n", N);
    srand(time(NULL));  // Seed random number generator
    double intrinsic = 0, regular = 0;
    double* distance;
    double* distance2;
    cudaMallocManaged(&distance, N * sizeof(double), cudaMemAttachGlobal);
    cudaMallocManaged(&distance2, N * sizeof(double), cudaMemAttachGlobal);

    milliTimer timer;
    int nblocks = N / THREADS_PER_BLOCK;
    double timeIntrinsic;
    double timeRegular;

    for (size_t i = 0; i < NRUNS; i++) {
        // assume this is a whole number
        // printDelim();
        timer.start();
        testIntrinsic<<<nblocks, THREADS_PER_BLOCK>>>(x, y, z, distance);
        timer.stop();
        timeIntrinsic = timer.count();
        // Skip first 8 runs of each
        if (i > 8) {
            intrinsic += timeIntrinsic;
        }

        timer.start();
        testRegular<<<nblocks, THREADS_PER_BLOCK>>>(x, y, z, distance2);
        timer.stop();
        timeRegular = timer.count();
        // Skip first 8 runs of each
        if (i > 8) {
            regular += timeRegular;
        }
    }
    // for (int i = 0; i < N; i++) {
    //     printf("dist[%d] is %f, dist2[%d] is %f \n", i, distance[i], i, distance2[i]);
    // }

    printf("Intrinsic took %f ms total, %f average\n", intrinsic, intrinsic / NRUNS);
    printf("Regular took %f ms total, %f average\n", regular, regular / NRUNS);

    cudaFree(x);
    cudaFree(y);
    cudaFree(z);
    cudaFree(distance);
    cudaFree(distance2);
}
