// Written by Colin Vanden Heuvel and Conlain Kelly as part of the
// Chrono granular project. I hope someome that I don't know reads
// this code someday.

#include <omp.h>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <iostream>
#include "timer.h"

#include <immintrin.h>
#include <pmmintrin.h>
#include <smmintrin.h>
#include <xmmintrin.h>

#define THOUSAND 1000ul
#define MILLION 1000000ul
#define BILLION 1000000000ul
#define RADIUS .1f

// Number of points to compute
#define N (100 * MILLION)
// Reference point (index in list of points)
#define REF 200

// Amount of times to run to compute averages
#define NRUNS 3ul

// Used for array of structs, w is a padding
struct vec3 {
    float x, y, z, w;
};

// Object-oriented box
// Used for terramechanics stuff and wheel alignment
struct OOB {
    vec3 dimensions;  // e0,e1,e2 half-width
    vec3 basis[3];    // 3 basis vectors in its frame, allows me to skip frames for now
    vec3 center;
};

// Axis-aligned box
struct AAB {
    vec3 min, max;  // min x,y,z and max x,y,z
};

// Returns b - a
inline vec3 sub(vec3* a, vec3* b) {
    float x = b->x - a->x;
    float y = b->y - a->y;
    float z = b->z - a->z;
    // Could definitely be vectorized
    return {x, y, z, 0.0f};  // w term is always 0 anyways
}

// Returns the inner product of b and a
inline float dot(float* a, float* b) {
    float d0 = a[0] * b[0];
    float d1 = a[1] * b[1];
    float d2 = a[2] * b[2];
    return d0 + d1 + d2;  // Could almost certainly be vectorized
}

// Returns the inner product of b and a
inline float dotabs(float* a, float* b) {
    float d0 = a[0] * b[0];
    float d1 = a[1] * b[1];
    float d2 = a[2] * b[2];
    return std::fabs(d0 + d1 + d2);  // Could almost certainly be vectorized
}

// Vectorized dot product
inline float dotVabs(float* p1, float* p2) {
    __m128 v1 = _mm_loadu_ps(p1);
    __m128 v2 = _mm_loadu_ps(p2);
    __m128 res = _mm_dp_ps(v2, v1, 0xff);
    return std::fabs(res[0]);
}

// Get me a random float between 0 and 1
inline float getRand() {
    int r = rand();  // returns a pseudo-random integer between 0 and RAND_MAX
    return (float)r / RAND_MAX;
}

// Distance for struct of arrays
inline float getDistance(float x1, float x2, float y1, float y2, float z1, float z2) {
    float dx = x2 - x1;
    float dy = y2 - y1;
    float dz = z2 - z1;
    return sqrt(dx * dx + dy * dy + dz * dz);
}

// Uses array of structs for now, return 1 if inside, 0 if outside
inline int insideBox(vec3* p1, OOB* box) {
    vec3 d = sub(&box->center, p1);  // Relative offset
    float r = RADIUS;                // Ball radius
    // Project onto basis
    float* d_ptr = &(d.x);
    float* basis0 = &(box->basis[0].x);
    float d0 = dotabs(d_ptr, basis0);
    if (d0 < box->dimensions.x + r) {
        return 1;
    }
    float* basis1 = &(box->basis[1].x);
    float d1 = dotabs(d_ptr, basis1);
    if (d1 < box->dimensions.y + r) {
        return 1;
    }
    float* basis2 = &(box->basis[2].x);
    float d2 = dotabs(d_ptr, basis2);
    if (d2 < box->dimensions.z + r) {
        return 1;
    }

    return 0;  // Outside
}

// Uses array of structs for now, return 1 if inside, 0 if outside
inline int insideBoxAAB(vec3* p1, AAB* box) {
    float r = RADIUS;  // Ball radius

    if ((p1->x > box->min.x - r) && (p1->x < box->max.x + r) && (p1->y > box->min.y - r) && (p1->y < box->max.y + r) &&
        (p1->z > box->min.z - r) && (p1->z < box->max.z + r)) {
        return 1;  // Inside
    }

    return 0;  // Outside
}

// Uses array of structs for now, return 1 if inside, 0 if outside, vectorized
inline int insideBoxV(vec3* p1, OOB* box) {
    vec3 d = sub(&box->center, p1);  // Relative offset
    float r = RADIUS;                // Ball radius
    // Project onto basis
    float* d_ptr = &(d.x);
    float* basis0 = &(box->basis[0].x);
    float* basis1 = &(box->basis[1].x);
    float* basis2 = &(box->basis[2].x);

    float dv0 = dotVabs(d_ptr, basis0);
    float dv1 = dotVabs(d_ptr, basis1);
    float dv2 = dotVabs(d_ptr, basis2);

    if ((dv0 < box->dimensions.x + r) && (dv1 < box->dimensions.y + r) && (dv2 < box->dimensions.z + r)) {
        return 1;  // Inside
    }

    return 0;  // Outside
}

// Test if a each point is inside a box
inline double testBox(vec3* points, int* results) {
    milliTimer timer;
    timer.start();

    OOB box;
    box.dimensions = {.25, .25, .25, 0};
    box.basis[0] = {1, 0, 0, 0};
    box.basis[1] = {0, 1, 0, 0};
    box.basis[2] = {0, 0, 1, 0};
    box.center = {0, 0, 0, 0};
#pragma omp parallel for
    for (size_t i = 0; i < N; i++) {
        results[i] = insideBox(points + i, &box);
    }
    timer.stop();
    return timer.count();
}

// Test if a each point is inside a box, vectorized
double testBoxV(vec3* points, int* results) {
    milliTimer timer;
    timer.start();

    OOB box;
    box.dimensions = {.25, .25, .25, 0};
    box.basis[0] = {1, 0, 0, 0};
    box.basis[1] = {0, 1, 0, 0};
    box.basis[2] = {0, 0, 1, 0};
    box.center = {0, 0, 0, 0};
#pragma omp parallel for
    for (size_t i = 0; i < N; i++) {
        results[i] = insideBoxV(points + i, &box);
    }
    timer.stop();
    return timer.count();
}

double testBoxAAB(vec3* points, int* results) {
    milliTimer timer;
    timer.start();

    AAB box;
    box.min = {0, 0, 0, 0};
    box.max = {.5, .5, .5, 0};

#pragma omp parallel for
    for (size_t i = 0; i < N; i++) {
        results[i] = insideBoxAAB(points + i, &box);
    }
    timer.stop();
    return timer.count();
}

// Regular distance for array of structs
inline float getDistanceStruct(vec3* p1, vec3* p2) {
    float dx = p2->x - p1->x;
    float dy = p2->y - p1->y;
    float dz = p2->z - p1->z;
    return sqrt(dx * dx + dy * dy + dz * dz);
}

// Vectorized distance for array of structs
inline float getDistanceV(vec3* p1, vec3* p2) {
    __m128 v1 = _mm_loadu_ps(&(p1->x));
    __m128 v2 = _mm_loadu_ps(&(p2->x));
    __m128 dv = _mm_sub_ps(v2, v1);
    dv = _mm_mul_ps(dv, dv);
    // dv.x = dv.x + dv.y
    // dv.y = dv.z + (dv.w = 0)
    // ... upper bits don't matter
    dv = _mm_hadd_ps(dv, dv);
    // dv.x = dv.x + dv.y
    // the rest doesn't matter
    dv = _mm_hadd_ps(dv, dv);
    // sqrt(dv.x);
    dv = _mm_sqrt_ss(dv);
    return _mm_cvtss_f32(dv);
}

// Big freaking vector
void getDistanceBFV(float* distances, vec3* p1, vec3* p2) {
    __m256 v1 = _mm256_loadu_ps(&(p1->x));
    __m256 v2 = _mm256_loadu_ps(&(p2->x));
    __m256 dv = _mm256_sub_ps(v2, v1);
    dv = _mm256_mul_ps(dv, dv);
    // dv.x = dv.x + dv.y
    // dv.y = dv.z + (dv.w = 0)
    // ... upper bits don't matter
    dv = _mm256_hadd_ps(dv, dv);
    // dv.x = dv.x + dv.y
    // the rest doesn't matter
    dv = _mm256_hadd_ps(dv, dv);
    // sqrt(dv.x);
    dv = _mm256_sqrt_ps(dv);

    *distances = _mm256_cvtss_f32(dv);
    *(distances + 1) = _mm_cvtss_f32(_mm256_extractf128_ps(dv, 1));
}

// Test array of structs with vectorization
double testStructsBFV(vec3* points, float* distances) {
    printf("testing structs, BFvectorized!\n");

    // Reference point, twice as wide as necessary
    vec3 ref[2] = {points[REF], points[REF]};
    // Allocate distances
    milliTimer timer;
    timer.start();
#pragma omp parallel for
    for (size_t i = 0; i < N; i += 2) {
        getDistanceBFV(distances + i, points + i, ref);
    }
    timer.stop();
    double elapsed = timer.count();
    printf("distance time is %f ms\n", elapsed);
    return elapsed;
}

// Test array of structs with vectorization
double testStructsVectorized(vec3* points, float* distances) {
    printf("testing structs, vectorized!\n");

    // Reference point
    vec3 pr = points[REF];
    // Allocate distances
    milliTimer timer;
    timer.start();
#pragma omp parallel for
    for (size_t i = 0; i < N; i++) {
        distances[i] = getDistanceV(points + i, &pr);
    }
    timer.stop();
    double elapsed = timer.count();
    printf("distance time is %f ms\n", elapsed);
    return elapsed;
}

// Test Calculation with structures
double testStructs(vec3* points, float* distances) {
    printf("testing structs!\n");

    // Reference point
    vec3 pr = points[REF];
    // Allocate distances
    milliTimer timer;
    timer.start();
#pragma omp parallel for
    for (size_t i = 0; i < N; i++) {
        distances[i] = getDistanceStruct(&points[i], &pr);
    }
    timer.stop();
    double elapsed = timer.count();
    printf("distance time is %f ms\n", elapsed);
    return elapsed;
}

// Test struct of arrays
double testArrays(float* x, float* y, float* z, float* distances) {
    printf("testing arrays\n");

    // Reference point
    float xr = x[REF];
    float yr = y[REF];
    float zr = z[REF];

    milliTimer timer;
    timer.start();
#pragma omp parallel for
    for (size_t i = 0; i < N; i++) {
        distances[i] = getDistance(x[i], xr, y[i], yr, z[i], zr);
        // printf("x is %f, y is %f, z is %f, dist is %f\n", x[i], y[i], z[i],
        //        distances[i]);
    }
    timer.stop();
    double elapsed = timer.count();
    printf("distance time is %f ms\n", elapsed);

    return elapsed;
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
    int nthreads = 1;
    // Can pass in num threads
    if (argc == 2) {
        nthreads = atoi(argv[1]);
    }
    omp_set_num_threads(nthreads);
    nthreads = omp_get_max_threads();
    printf("running on %d threads\n", nthreads);

    // Allocate x, y, z for struct of arrays
    float* x = new float[N];
    float* y = new float[N];
    float* z = new float[N];

    // Allocate points for array of structs
    vec3* points = new vec3[N];

    // Initialize points
    printf("Initializing points\n");
    for (size_t i = 0; i < N; i++) {
        x[i] = getRand();
        points[i].x = x[i];
        y[i] = getRand();
        points[i].y = y[i];
        z[i] = getRand();
        points[i].z = z[i];
        points[i].w = 0;
    }
    printf("%lu points to compute\n", N);
    srand(time(NULL));  // Seed random number generator
    double array = 0, strucs = 0, vectorized = 0, bfv = 0;
    float* distance = new float[N];
    // float *distance2 = new float[N];
    // float *distance3 = new float[N];
    // float *distance4 = new float[N];
    int* results = new int[N];
    int* results2 = new int[N];
    for (size_t i = 0; i < NRUNS; i++) {
        printDelim();
        array += testArrays(x, y, z, distance);
        strucs += testStructs(points, distance);
        vectorized += testStructsVectorized(points, distance);
        bfv += testStructsBFV(points, distance);
        // Can use this code for validation -- it passes
        // for (size_t i = 0; i < N; i++) {
        //   // printf("%f %f %f %f\n", distance1[i], distance2[i], distance3[i],
        //   //        distance4[i]);
        //   if (distance1[i] != distance2[i]) {
        //     err(1, 2, i);
        //   }
        //   if (distance2[i] != distance3[i]) {
        //     err(2, 3, i);
        //   }
        //   if (distance3[i] != distance4[i]) {
        //     err(3, 4, i);distance4
        //   }
        // }
        double timeBox = testBox(points, results);
        double timeBoxV = testBoxV(points, results2);
        double timeBoxAAB = testBoxAAB(points, results);
        printf("box took %f ms\n", timeBox);
        printf("box took %f ms vectorized\n", timeBoxV);
        printf("AAB took %f ms\n", timeBoxAAB);
    }
    // This is a long print statement
    printf(
        "Over %lu runs, struct of arrays took %f ms, array of structs took %f ms, vectorized  array of structs took %f "
        "ms, big vectorization took %f ms on average\n",
        NRUNS, array / NRUNS, strucs / NRUNS, vectorized / NRUNS, bfv / NRUNS);
    for (size_t i = 0; i < N; i++) {
        if (results[i] == 1) {
            // printf("point (%f, %f, %f) is inside the box\n", points[i].x, points[i].y, points[i].z);
        }
    }
    delete[] x;
    delete[] y;
    delete[] z;
    delete[] points;
    delete[] distance;
    delete[] results;
    delete[] results2;
}
