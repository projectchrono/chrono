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

// test whether a box and a triangle are overlapping, used to check if a triangle is in an SD
__device__ int triBoxOverlap(const float (&boxcenter)[3], const float (&boxhalfsize)[3], const float (&triverts)[3][3]);

// test whether a triangle and a sphere are in contact and set the normal and penetration
__device__ bool face_sphere(const float3& A1,
                            const float3& B1,
                            const float3& C1,
                            const float3& pos2,
                            const float& radius2,
                            const float& separation,
                            float3& norm,
                            float& depth,
                            float3& pt1,
                            float3& pt2,
                            float& eff_radius);