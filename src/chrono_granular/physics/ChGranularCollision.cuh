// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2018 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// CUDA wrapper for collision-detection utilities for chrono_granular
//
// =============================================================================
// Authors: Dan Negrut, Conlain Kelly
// =============================================================================

/// test whether a box and a triangle are overlapping; used to check if a triangle touches an SD.
__device__ bool check_TriangleBoxOverlap(float boxcenter[3],
                                         float boxhalfsize[3],
                                         const float3& vA,
                                         const float3& vB,
                                         const float3& vC);

/// test whether a triangle and a sphere are in contact and set the normal and penetration
__device__ bool face_sphere_cd(const double3& A1,
                               const double3& B1,
                               const double3& C1,
                               const double3& pos2,
                               const double& radius2,
                               double3& norm,
                               double& depth,
                               double3& pt1,
                               double3& pt2,
                               double& eff_radius);
