// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================

#ifndef CHC_MATVEC_H
#define CHC_MATVEC_H

#include <cmath>
#include <cstdio>

#include "chrono/collision/edgetempest/ChCCompile.h"

namespace chrono {
namespace collision {

#ifndef M_PI
const PQP_REAL M_PI = (PQP_REAL)3.14159265359;
#endif

#define myfabs(x) ((x < 0) ? -x : x)

inline void Mprintg(const PQP_REAL M[3][3]) {
    printf("%g %g %g\n%g %g %g\n%g %g %g\n", M[0][0], M[0][1], M[0][2], M[1][0], M[1][1], M[1][2], M[2][0], M[2][1],
           M[2][2]);
}

inline void Mfprint(FILE* f, const PQP_REAL M[3][3]) {
    fprintf(f, "%g %g %g\n%g %g %g\n%g %g %g\n", M[0][0], M[0][1], M[0][2], M[1][0], M[1][1], M[1][2], M[2][0], M[2][1],
            M[2][2]);
}

inline void Mprint(const PQP_REAL M[3][3]) {
    printf("%g %g %g\n%g %g %g\n%g %g %g\n", M[0][0], M[0][1], M[0][2], M[1][0], M[1][1], M[1][2], M[2][0], M[2][1],
           M[2][2]);
}

inline void Vprintg(const PQP_REAL V[3]) {
    printf("%g %g %g\n", V[0], V[1], V[2]);
}

inline void Vfprint(FILE* f, const PQP_REAL V[3]) {
    fprintf(f, "%g %g %g\n", V[0], V[1], V[2]);
}

inline void Vprint(const PQP_REAL V[3]) {
    printf("%g %g %g\n", V[0], V[1], V[2]);
}

inline void Midentity(PQP_REAL M[3][3]) {
    M[0][0] = M[1][1] = M[2][2] = 1.0;
    M[0][1] = M[1][2] = M[2][0] = 0.0;
    M[0][2] = M[1][0] = M[2][1] = 0.0;
}

inline void Videntity(PQP_REAL T[3]) {
    T[0] = T[1] = T[2] = 0.0;
}

inline void McM(PQP_REAL Mr[3][3], const PQP_REAL M[3][3]) {
    Mr[0][0] = M[0][0];
    Mr[0][1] = M[0][1];
    Mr[0][2] = M[0][2];
    Mr[1][0] = M[1][0];
    Mr[1][1] = M[1][1];
    Mr[1][2] = M[1][2];
    Mr[2][0] = M[2][0];
    Mr[2][1] = M[2][1];
    Mr[2][2] = M[2][2];
}

inline void MTcM(PQP_REAL Mr[3][3], const PQP_REAL M[3][3]) {
    Mr[0][0] = M[0][0];
    Mr[1][0] = M[0][1];
    Mr[2][0] = M[0][2];
    Mr[0][1] = M[1][0];
    Mr[1][1] = M[1][1];
    Mr[2][1] = M[1][2];
    Mr[0][2] = M[2][0];
    Mr[1][2] = M[2][1];
    Mr[2][2] = M[2][2];
}

inline void VcV(PQP_REAL Vr[3], const PQP_REAL V[3]) {
    Vr[0] = V[0];
    Vr[1] = V[1];
    Vr[2] = V[2];
}

inline void McolcV(PQP_REAL Vr[3], const PQP_REAL M[3][3], int c) {
    Vr[0] = M[0][c];
    Vr[1] = M[1][c];
    Vr[2] = M[2][c];
}

inline void McolcMcol(PQP_REAL Mr[3][3], int cr, const PQP_REAL M[3][3], int c) {
    Mr[0][cr] = M[0][c];
    Mr[1][cr] = M[1][c];
    Mr[2][cr] = M[2][c];
}

inline void MxMpV(PQP_REAL Mr[3][3], const PQP_REAL M1[3][3], const PQP_REAL M2[3][3], const PQP_REAL T[3]) {
    Mr[0][0] = (M1[0][0] * M2[0][0] + M1[0][1] * M2[1][0] + M1[0][2] * M2[2][0] + T[0]);
    Mr[1][0] = (M1[1][0] * M2[0][0] + M1[1][1] * M2[1][0] + M1[1][2] * M2[2][0] + T[1]);
    Mr[2][0] = (M1[2][0] * M2[0][0] + M1[2][1] * M2[1][0] + M1[2][2] * M2[2][0] + T[2]);
    Mr[0][1] = (M1[0][0] * M2[0][1] + M1[0][1] * M2[1][1] + M1[0][2] * M2[2][1] + T[0]);
    Mr[1][1] = (M1[1][0] * M2[0][1] + M1[1][1] * M2[1][1] + M1[1][2] * M2[2][1] + T[1]);
    Mr[2][1] = (M1[2][0] * M2[0][1] + M1[2][1] * M2[1][1] + M1[2][2] * M2[2][1] + T[2]);
    Mr[0][2] = (M1[0][0] * M2[0][2] + M1[0][1] * M2[1][2] + M1[0][2] * M2[2][2] + T[0]);
    Mr[1][2] = (M1[1][0] * M2[0][2] + M1[1][1] * M2[1][2] + M1[1][2] * M2[2][2] + T[1]);
    Mr[2][2] = (M1[2][0] * M2[0][2] + M1[2][1] * M2[1][2] + M1[2][2] * M2[2][2] + T[2]);
}

inline void MxM(PQP_REAL Mr[3][3], const PQP_REAL M1[3][3], const PQP_REAL M2[3][3]) {
    Mr[0][0] = (M1[0][0] * M2[0][0] + M1[0][1] * M2[1][0] + M1[0][2] * M2[2][0]);
    Mr[1][0] = (M1[1][0] * M2[0][0] + M1[1][1] * M2[1][0] + M1[1][2] * M2[2][0]);
    Mr[2][0] = (M1[2][0] * M2[0][0] + M1[2][1] * M2[1][0] + M1[2][2] * M2[2][0]);
    Mr[0][1] = (M1[0][0] * M2[0][1] + M1[0][1] * M2[1][1] + M1[0][2] * M2[2][1]);
    Mr[1][1] = (M1[1][0] * M2[0][1] + M1[1][1] * M2[1][1] + M1[1][2] * M2[2][1]);
    Mr[2][1] = (M1[2][0] * M2[0][1] + M1[2][1] * M2[1][1] + M1[2][2] * M2[2][1]);
    Mr[0][2] = (M1[0][0] * M2[0][2] + M1[0][1] * M2[1][2] + M1[0][2] * M2[2][2]);
    Mr[1][2] = (M1[1][0] * M2[0][2] + M1[1][1] * M2[1][2] + M1[1][2] * M2[2][2]);
    Mr[2][2] = (M1[2][0] * M2[0][2] + M1[2][1] * M2[1][2] + M1[2][2] * M2[2][2]);
}

inline void MxMT(PQP_REAL Mr[3][3], const PQP_REAL M1[3][3], const PQP_REAL M2[3][3]) {
    Mr[0][0] = (M1[0][0] * M2[0][0] + M1[0][1] * M2[0][1] + M1[0][2] * M2[0][2]);
    Mr[1][0] = (M1[1][0] * M2[0][0] + M1[1][1] * M2[0][1] + M1[1][2] * M2[0][2]);
    Mr[2][0] = (M1[2][0] * M2[0][0] + M1[2][1] * M2[0][1] + M1[2][2] * M2[0][2]);
    Mr[0][1] = (M1[0][0] * M2[1][0] + M1[0][1] * M2[1][1] + M1[0][2] * M2[1][2]);
    Mr[1][1] = (M1[1][0] * M2[1][0] + M1[1][1] * M2[1][1] + M1[1][2] * M2[1][2]);
    Mr[2][1] = (M1[2][0] * M2[1][0] + M1[2][1] * M2[1][1] + M1[2][2] * M2[1][2]);
    Mr[0][2] = (M1[0][0] * M2[2][0] + M1[0][1] * M2[2][1] + M1[0][2] * M2[2][2]);
    Mr[1][2] = (M1[1][0] * M2[2][0] + M1[1][1] * M2[2][1] + M1[1][2] * M2[2][2]);
    Mr[2][2] = (M1[2][0] * M2[2][0] + M1[2][1] * M2[2][1] + M1[2][2] * M2[2][2]);
}

inline void MTxM(PQP_REAL Mr[3][3], const PQP_REAL M1[3][3], const PQP_REAL M2[3][3]) {
    Mr[0][0] = (M1[0][0] * M2[0][0] + M1[1][0] * M2[1][0] + M1[2][0] * M2[2][0]);
    Mr[1][0] = (M1[0][1] * M2[0][0] + M1[1][1] * M2[1][0] + M1[2][1] * M2[2][0]);
    Mr[2][0] = (M1[0][2] * M2[0][0] + M1[1][2] * M2[1][0] + M1[2][2] * M2[2][0]);
    Mr[0][1] = (M1[0][0] * M2[0][1] + M1[1][0] * M2[1][1] + M1[2][0] * M2[2][1]);
    Mr[1][1] = (M1[0][1] * M2[0][1] + M1[1][1] * M2[1][1] + M1[2][1] * M2[2][1]);
    Mr[2][1] = (M1[0][2] * M2[0][1] + M1[1][2] * M2[1][1] + M1[2][2] * M2[2][1]);
    Mr[0][2] = (M1[0][0] * M2[0][2] + M1[1][0] * M2[1][2] + M1[2][0] * M2[2][2]);
    Mr[1][2] = (M1[0][1] * M2[0][2] + M1[1][1] * M2[1][2] + M1[2][1] * M2[2][2]);
    Mr[2][2] = (M1[0][2] * M2[0][2] + M1[1][2] * M2[1][2] + M1[2][2] * M2[2][2]);
}

inline void MxV(PQP_REAL Vr[3], const PQP_REAL M1[3][3], const PQP_REAL V1[3]) {
    Vr[0] = (M1[0][0] * V1[0] + M1[0][1] * V1[1] + M1[0][2] * V1[2]);
    Vr[1] = (M1[1][0] * V1[0] + M1[1][1] * V1[1] + M1[1][2] * V1[2]);
    Vr[2] = (M1[2][0] * V1[0] + M1[2][1] * V1[1] + M1[2][2] * V1[2]);
}

inline void MxVpV(PQP_REAL Vr[3], const PQP_REAL M1[3][3], const PQP_REAL V1[3], const PQP_REAL V2[3]) {
    Vr[0] = (M1[0][0] * V1[0] + M1[0][1] * V1[1] + M1[0][2] * V1[2] + V2[0]);
    Vr[1] = (M1[1][0] * V1[0] + M1[1][1] * V1[1] + M1[1][2] * V1[2] + V2[1]);
    Vr[2] = (M1[2][0] * V1[0] + M1[2][1] * V1[1] + M1[2][2] * V1[2] + V2[2]);
}

inline void sMxVpV(PQP_REAL Vr[3], PQP_REAL s1, const PQP_REAL M1[3][3], const PQP_REAL V1[3], const PQP_REAL V2[3]) {
    Vr[0] = s1 * (M1[0][0] * V1[0] + M1[0][1] * V1[1] + M1[0][2] * V1[2]) + V2[0];
    Vr[1] = s1 * (M1[1][0] * V1[0] + M1[1][1] * V1[1] + M1[1][2] * V1[2]) + V2[1];
    Vr[2] = s1 * (M1[2][0] * V1[0] + M1[2][1] * V1[1] + M1[2][2] * V1[2]) + V2[2];
}

inline void MTxV(PQP_REAL Vr[3], const PQP_REAL M1[3][3], const PQP_REAL V1[3]) {
    Vr[0] = (M1[0][0] * V1[0] + M1[1][0] * V1[1] + M1[2][0] * V1[2]);
    Vr[1] = (M1[0][1] * V1[0] + M1[1][1] * V1[1] + M1[2][1] * V1[2]);
    Vr[2] = (M1[0][2] * V1[0] + M1[1][2] * V1[1] + M1[2][2] * V1[2]);
}

inline void sMTxV(PQP_REAL Vr[3], PQP_REAL s1, const PQP_REAL M1[3][3], const PQP_REAL V1[3]) {
    Vr[0] = s1 * (M1[0][0] * V1[0] + M1[1][0] * V1[1] + M1[2][0] * V1[2]);
    Vr[1] = s1 * (M1[0][1] * V1[0] + M1[1][1] * V1[1] + M1[2][1] * V1[2]);
    Vr[2] = s1 * (M1[0][2] * V1[0] + M1[1][2] * V1[1] + M1[2][2] * V1[2]);
}

inline void sMxV(PQP_REAL Vr[3], PQP_REAL s1, const PQP_REAL M1[3][3], const PQP_REAL V1[3]) {
    Vr[0] = s1 * (M1[0][0] * V1[0] + M1[0][1] * V1[1] + M1[0][2] * V1[2]);
    Vr[1] = s1 * (M1[1][0] * V1[0] + M1[1][1] * V1[1] + M1[1][2] * V1[2]);
    Vr[2] = s1 * (M1[2][0] * V1[0] + M1[2][1] * V1[1] + M1[2][2] * V1[2]);
}

inline void VmV(PQP_REAL Vr[3], const PQP_REAL V1[3], const PQP_REAL V2[3]) {
    Vr[0] = V1[0] - V2[0];
    Vr[1] = V1[1] - V2[1];
    Vr[2] = V1[2] - V2[2];
}

inline void VpV(PQP_REAL Vr[3], const PQP_REAL V1[3], const PQP_REAL V2[3]) {
    Vr[0] = V1[0] + V2[0];
    Vr[1] = V1[1] + V2[1];
    Vr[2] = V1[2] + V2[2];
}

inline void VpVxS(PQP_REAL Vr[3], const PQP_REAL V1[3], const PQP_REAL V2[3], PQP_REAL s) {
    Vr[0] = V1[0] + V2[0] * s;
    Vr[1] = V1[1] + V2[1] * s;
    Vr[2] = V1[2] + V2[2] * s;
}

inline void MskewV(PQP_REAL M[3][3], const PQP_REAL v[3]) {
    M[0][0] = M[1][1] = M[2][2] = 0.0;
    M[1][0] = v[2];
    M[0][1] = -v[2];
    M[0][2] = v[1];
    M[2][0] = -v[1];
    M[1][2] = -v[0];
    M[2][1] = v[0];
}

inline void VcrossV(PQP_REAL Vr[3], const PQP_REAL V1[3], const PQP_REAL V2[3]) {
    Vr[0] = V1[1] * V2[2] - V1[2] * V2[1];
    Vr[1] = V1[2] * V2[0] - V1[0] * V2[2];
    Vr[2] = V1[0] * V2[1] - V1[1] * V2[0];
}

inline PQP_REAL Vlength(PQP_REAL V[3]) {
    return sqrt(V[0] * V[0] + V[1] * V[1] + V[2] * V[2]);
}

inline void Vnormalize(PQP_REAL V[3]) {
    PQP_REAL d = (PQP_REAL)1.0 / sqrt(V[0] * V[0] + V[1] * V[1] + V[2] * V[2]);
    V[0] *= d;
    V[1] *= d;
    V[2] *= d;
}

inline PQP_REAL VdotV(const PQP_REAL V1[3], const PQP_REAL V2[3]) {
    return (V1[0] * V2[0] + V1[1] * V2[1] + V1[2] * V2[2]);
}

inline PQP_REAL VdistV2(const PQP_REAL V1[3], const PQP_REAL V2[3]) {
    return ((V1[0] - V2[0]) * (V1[0] - V2[0]) + (V1[1] - V2[1]) * (V1[1] - V2[1]) + (V1[2] - V2[2]) * (V1[2] - V2[2]));
}

inline void VxS(PQP_REAL Vr[3], const PQP_REAL V[3], PQP_REAL s) {
    Vr[0] = V[0] * s;
    Vr[1] = V[1] * s;
    Vr[2] = V[2] * s;
}

inline void MRotZ(PQP_REAL Mr[3][3], PQP_REAL t) {
    Mr[0][0] = cos(t);
    Mr[1][0] = sin(t);
    Mr[0][1] = -Mr[1][0];
    Mr[1][1] = Mr[0][0];
    Mr[2][0] = Mr[2][1] = 0.0;
    Mr[0][2] = Mr[1][2] = 0.0;
    Mr[2][2] = 1.0;
}

inline void MRotX(PQP_REAL Mr[3][3], PQP_REAL t) {
    Mr[1][1] = cos(t);
    Mr[2][1] = sin(t);
    Mr[1][2] = -Mr[2][1];
    Mr[2][2] = Mr[1][1];
    Mr[0][1] = Mr[0][2] = 0.0;
    Mr[1][0] = Mr[2][0] = 0.0;
    Mr[0][0] = 1.0;
}

inline void MRotY(PQP_REAL Mr[3][3], PQP_REAL t) {
    Mr[2][2] = cos(t);
    Mr[0][2] = sin(t);
    Mr[2][0] = -Mr[0][2];
    Mr[0][0] = Mr[2][2];
    Mr[1][2] = Mr[1][0] = 0.0;
    Mr[2][1] = Mr[0][1] = 0.0;
    Mr[1][1] = 1.0;
}

inline void MVtoOGL(double oglm[16], const PQP_REAL R[3][3], const PQP_REAL T[3]) {
    oglm[0] = (double)R[0][0];
    oglm[1] = (double)R[1][0];
    oglm[2] = (double)R[2][0];
    oglm[3] = 0.0;
    oglm[4] = (double)R[0][1];
    oglm[5] = (double)R[1][1];
    oglm[6] = (double)R[2][1];
    oglm[7] = 0.0;
    oglm[8] = (double)R[0][2];
    oglm[9] = (double)R[1][2];
    oglm[10] = (double)R[2][2];
    oglm[11] = 0.0;
    oglm[12] = (double)T[0];
    oglm[13] = (double)T[1];
    oglm[14] = (double)T[2];
    oglm[15] = 1.0;
}

inline void OGLtoMV(PQP_REAL R[3][3], PQP_REAL T[3], const double oglm[16]) {
    R[0][0] = (PQP_REAL)oglm[0];
    R[1][0] = (PQP_REAL)oglm[1];
    R[2][0] = (PQP_REAL)oglm[2];

    R[0][1] = (PQP_REAL)oglm[4];
    R[1][1] = (PQP_REAL)oglm[5];
    R[2][1] = (PQP_REAL)oglm[6];

    R[0][2] = (PQP_REAL)oglm[8];
    R[1][2] = (PQP_REAL)oglm[9];
    R[2][2] = (PQP_REAL)oglm[10];

    T[0] = (PQP_REAL)oglm[12];
    T[1] = (PQP_REAL)oglm[13];
    T[2] = (PQP_REAL)oglm[14];
}

// taken from quatlib, written by Richard Holloway
const int QX = 0;
const int QY = 1;
const int QZ = 2;
const int QW = 3;

inline void MRotQ(PQP_REAL destMatrix[3][3], PQP_REAL srcQuat[4]) {
    PQP_REAL s;
    PQP_REAL xs, ys, zs, wx, wy, wz, xx, xy, xz, yy, yz, zz;

    /*
     * For unit srcQuat, just set s = 2.0; or set xs = srcQuat[QX] +
     *   srcQuat[QX], etc.
     */

    s = (PQP_REAL)2.0 /
        (srcQuat[QX] * srcQuat[QX] + srcQuat[QY] * srcQuat[QY] + srcQuat[QZ] * srcQuat[QZ] + srcQuat[QW] * srcQuat[QW]);

    xs = srcQuat[QX] * s;
    ys = srcQuat[QY] * s;
    zs = srcQuat[QZ] * s;
    wx = srcQuat[QW] * xs;
    wy = srcQuat[QW] * ys;
    wz = srcQuat[QW] * zs;
    xx = srcQuat[QX] * xs;
    xy = srcQuat[QX] * ys;
    xz = srcQuat[QX] * zs;
    yy = srcQuat[QY] * ys;
    yz = srcQuat[QY] * zs;
    zz = srcQuat[QZ] * zs;

    destMatrix[QX][QX] = (PQP_REAL)1.0 - (yy + zz);
    destMatrix[QX][QY] = xy + wz;
    destMatrix[QX][QZ] = xz - wy;

    destMatrix[QY][QX] = xy - wz;
    destMatrix[QY][QY] = (PQP_REAL)1.0 - (xx + zz);
    destMatrix[QY][QZ] = yz + wx;

    destMatrix[QZ][QX] = xz + wy;
    destMatrix[QZ][QY] = yz - wx;
    destMatrix[QZ][QZ] = (PQP_REAL)1.0 - (xx + yy);
}

inline void Mqinverse(PQP_REAL Mr[3][3], PQP_REAL m[3][3]) {
    int i, j;

    for (i = 0; i < 3; i++)
        for (j = 0; j < 3; j++) {
            int i1 = (i + 1) % 3;
            int i2 = (i + 2) % 3;
            int j1 = (j + 1) % 3;
            int j2 = (j + 2) % 3;
            Mr[i][j] = (m[j1][i1] * m[j2][i2] - m[j1][i2] * m[j2][i1]);
        }
}

inline double M33FastInvert(PQP_REAL matra[3][3], PQP_REAL mthis[3][3]) {
    double det;
    double sdet0, sdet1, sdet2;

    sdet0 = +(mthis[1][1] * mthis[2][2]) - (mthis[2][1] * mthis[1][2]);
    sdet1 = -(mthis[1][0] * mthis[2][2]) + (mthis[2][0] * mthis[1][2]);
    sdet2 = +(mthis[1][0] * mthis[2][1]) - (mthis[2][0] * mthis[1][1]);

    det = sdet0 * mthis[0][0] + sdet1 * mthis[0][1] + sdet2 * mthis[0][2];

    matra[0][0] = sdet0 / det;
    matra[1][0] = sdet1 / det;
    matra[2][0] = sdet2 / det;
    matra[0][1] = (-(mthis[0][1] * mthis[2][2]) + (mthis[2][1] * mthis[0][2])) / det;
    matra[1][1] = (+(mthis[0][0] * mthis[2][2]) - (mthis[2][0] * mthis[0][2])) / det;
    matra[2][1] = (-(mthis[0][0] * mthis[2][1]) + (mthis[2][0] * mthis[0][1])) / det;
    matra[0][2] = (+(mthis[0][1] * mthis[1][2]) - (mthis[1][1] * mthis[0][2])) / det;
    matra[1][2] = (-(mthis[0][0] * mthis[1][2]) + (mthis[1][0] * mthis[0][2])) / det;
    matra[2][2] = (+(mthis[0][0] * mthis[1][1]) - (mthis[1][0] * mthis[0][1])) / det;

    return (det);
}
// Meigen from Numerical Recipes in C

#if 0

#define rfabs(x) ((x < 0) ? -x : x)

#define ROT(a, i, j, k, l)           \
    g = a[i][j];                     \
    h = a[k][l];                     \
    a[i][j] = g - s * (h + g * tau); \
    a[k][l] = h + s * (g - h * tau);

int
inline
Meigen(PQP_REAL vout[3][3], PQP_REAL dout[3], PQP_REAL a[3][3])
{
  int i;
  PQP_REAL tresh,theta,tau,t,sm,s,h,g,c;
  int nrot;
  PQP_REAL b[3];
  PQP_REAL z[3];
  PQP_REAL v[3][3];
  PQP_REAL d[3];

  v[0][0] = v[1][1] = v[2][2] = 1.0;
  v[0][1] = v[1][2] = v[2][0] = 0.0;
  v[0][2] = v[1][0] = v[2][1] = 0.0;
  
  b[0] = a[0][0]; d[0] = a[0][0]; z[0] = 0.0;
  b[1] = a[1][1]; d[1] = a[1][1]; z[1] = 0.0;
  b[2] = a[2][2]; d[2] = a[2][2]; z[2] = 0.0;

  nrot = 0;

  
  for(i=0; i<50; i++)
    {

      printf("2\n");

      sm=0.0; sm+=fabs(a[0][1]); sm+=fabs(a[0][2]); sm+=fabs(a[1][2]);
      if (sm == 0.0) { McM(vout,v); VcV(dout,d); return i; }
      
      if (i < 3) tresh=0.2*sm/(3*3); else tresh=0.0;
      
      {
	g = 100.0*rfabs(a[0][1]);  
	if (i>3 && rfabs(d[0])+g==rfabs(d[0]) && rfabs(d[1])+g==rfabs(d[1]))
	  a[0][1]=0.0;
	else if (rfabs(a[0][1])>tresh)
	  {
	    h = d[1]-d[0];
	    if (rfabs(h)+g == rfabs(h)) t=(a[0][1])/h;
	    else
	      {
		theta=0.5*h/(a[0][1]);
		t=1.0/(rfabs(theta)+sqrt(1.0+theta*theta));
		if (theta < 0.0) t = -t;
	      }
	    c=1.0/sqrt(1+t*t); s=t*c; tau=s/(1.0+c); h=t*a[0][1];
	    z[0] -= h; z[1] += h; d[0] -= h; d[1] += h;
	    a[0][1]=0.0;
	    ROT(a,0,2,1,2); ROT(v,0,0,0,1); ROT(v,1,0,1,1); ROT(v,2,0,2,1); 
	    nrot++;
	  }
      }

      {
	g = 100.0*rfabs(a[0][2]);
	if (i>3 && rfabs(d[0])+g==rfabs(d[0]) && rfabs(d[2])+g==rfabs(d[2]))
	  a[0][2]=0.0;
	else if (rfabs(a[0][2])>tresh)
	  {
	    h = d[2]-d[0];
	    if (rfabs(h)+g == rfabs(h)) t=(a[0][2])/h;
	    else
	      {
		theta=0.5*h/(a[0][2]);
		t=1.0/(rfabs(theta)+sqrt(1.0+theta*theta));
		if (theta < 0.0) t = -t;
	      }
	    c=1.0/sqrt(1+t*t); s=t*c; tau=s/(1.0+c); h=t*a[0][2];
	    z[0] -= h; z[2] += h; d[0] -= h; d[2] += h;
	    a[0][2]=0.0;
	    ROT(a,0,1,1,2); ROT(v,0,0,0,2); ROT(v,1,0,1,2); ROT(v,2,0,2,2); 
	    nrot++;
	  }
      }

      {
	g = 100.0*rfabs(a[1][2]);
	if (i>3 && rfabs(d[1])+g==rfabs(d[1]) && rfabs(d[2])+g==rfabs(d[2]))
	  a[1][2]=0.0;
	else if (rfabs(a[1][2])>tresh)
	  {
	    h = d[2]-d[1];
	    if (rfabs(h)+g == rfabs(h)) t=(a[1][2])/h;
	    else
	      {
		theta=0.5*h/(a[1][2]);
		t=1.0/(rfabs(theta)+sqrt(1.0+theta*theta));
		if (theta < 0.0) t = -t;
	      }
	    c=1.0/sqrt(1+t*t); s=t*c; tau=s/(1.0+c); h=t*a[1][2];
	    z[1] -= h; z[2] += h; d[1] -= h; d[2] += h;
	    a[1][2]=0.0;
	    ROT(a,0,1,0,2); ROT(v,0,1,0,2); ROT(v,1,1,1,2); ROT(v,2,1,2,2); 
	    nrot++;
	  }
      }

      b[0] += z[0]; d[0] = b[0]; z[0] = 0.0;
      b[1] += z[1]; d[1] = b[1]; z[1] = 0.0;
      b[2] += z[2]; d[2] = b[2]; z[2] = 0.0;
      
    }

  fprintf(stderr, "eigen: too many iterations in Jacobi transform (%d).\n", i);

  return i;
}

#else

#define ROTATE(a, i, j, k, l)        \
    g = a[i][j];                     \
    h = a[k][l];                     \
    a[i][j] = g - s * (h + g * tau); \
    a[k][l] = h + s * (g - h * tau);

void inline Meigen(PQP_REAL vout[3][3], PQP_REAL dout[3], PQP_REAL a[3][3]) {
    int n = 3;
    int j, iq, ip, i;
    PQP_REAL tresh, theta, tau, t, sm, s, h, g, c;
    int nrot;
    PQP_REAL b[3];
    PQP_REAL z[3];
    PQP_REAL v[3][3];
    PQP_REAL d[3];

    Midentity(v);
    for (ip = 0; ip < n; ip++) {
        b[ip] = a[ip][ip];
        d[ip] = a[ip][ip];
        z[ip] = 0.0;
    }

    nrot = 0;

    for (i = 0; i < 50; i++) {
        sm = 0.0;
        for (ip = 0; ip < n; ip++)
            for (iq = ip + 1; iq < n; iq++)
                sm += fabs(a[ip][iq]);
        if (sm == 0.0) {
            McM(vout, v);
            VcV(dout, d);
            return;
        }

        if (i < 3)
            tresh = (PQP_REAL)0.2 * sm / (n * n);
        else
            tresh = 0.0;

        for (ip = 0; ip < n; ip++)
            for (iq = ip + 1; iq < n; iq++) {
                g = (PQP_REAL)100.0 * fabs(a[ip][iq]);
                if (i > 3 && fabs(d[ip]) + g == fabs(d[ip]) && fabs(d[iq]) + g == fabs(d[iq]))
                    a[ip][iq] = 0.0;
                else if (fabs(a[ip][iq]) > tresh) {
                    h = d[iq] - d[ip];
                    if (fabs(h) + g == fabs(h))
                        t = (a[ip][iq]) / h;
                    else {
                        theta = (PQP_REAL)0.5 * h / (a[ip][iq]);
                        t = (PQP_REAL)(1.0 / (fabs(theta) + sqrt(1.0 + theta * theta)));
                        if (theta < 0.0)
                            t = -t;
                    }
                    c = (PQP_REAL)1.0 / sqrt(1 + t * t);
                    s = t * c;
                    tau = s / ((PQP_REAL)1.0 + c);
                    h = t * a[ip][iq];
                    z[ip] -= h;
                    z[iq] += h;
                    d[ip] -= h;
                    d[iq] += h;
                    a[ip][iq] = 0.0;
                    for (j = 0; j < ip; j++) {
                        ROTATE(a, j, ip, j, iq);
                    }
                    for (j = ip + 1; j < iq; j++) {
                        ROTATE(a, ip, j, j, iq);
                    }
                    for (j = iq + 1; j < n; j++) {
                        ROTATE(a, ip, j, iq, j);
                    }
                    for (j = 0; j < n; j++) {
                        ROTATE(v, j, ip, j, iq);
                    }
                    nrot++;
                }
            }
        for (ip = 0; ip < n; ip++) {
            b[ip] += z[ip];
            d[ip] = b[ip];
            z[ip] = 0.0;
        }
    }

    fprintf(stderr, "eigen: too many iterations in Jacobi transform.\n");

    return;
}

#endif  // end if meigen

}  // end namespace collision
}  // end namespace chrono

#endif
