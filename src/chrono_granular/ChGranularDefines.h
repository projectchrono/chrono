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
// =============================================================================
// Authors: Dan Negrut
// =============================================================================
/*! \file */
#include <climits>

#pragma once

///< At most 8 domains are touched by a sphere
#define MAX_SDs_TOUCHED_BY_SPHERE 8
/// The L-size of an SD should contain, on average, about these many spheres
#define AVERAGE_SPHERES_PER_SD_L_DIR 3.5
/// The D-size of an SD should contain, on average, about these many spheres
#define AVERAGE_SPHERES_PER_SD_D_DIR 3.5
/// The H-size of an SD should contain, on average, about these many spheres
#define AVERAGE_SPHERES_PER_SD_H_DIR 3.5
/// Anticipated max number of DEs in an SD; used for setting aside memory ahead of time
#define MAX_COUNT_OF_DEs_PER_SD 256
/// Anticipated max number of mesh triangles in an SD; used for setting aside memory ahead of time
#define MAX_COUNT_OF_Triangles_PER_SD 64
/// Value that indicates non-valid ID. The assumption is that an ID is always a positive integer
#define NULL_GRANULAR_ID UINT_MAX
/// Value that indicates non-valid ID in long size_t form. This allows > 4 billion entries into an array
#define NULL_GRANULAR_ID_LONG SIZE_MAX
/// Value that indicates a non-valid (ILLegal) physical attribute for integer attributes
#define ILL_GRANULAR_VAL INT_MAX
/// The number of average contacts per DE. High values are safe but might also translate into wasted memory
#define AVERAGE_COUNT_CONTACTS_PER_DE 8
/// Value that controls the length unit. It is this many simulation length units that a sphere deforms under its own
/// weight.
#define PSI_L 16
/// Value that controls the time unit. It is this many simulation time units that it will take to clear a deformation of
/// a sphere
#define PSI_h 4
/// Value that controls the time unit. It is like a safety factor.
#define PSI_T 8
/// Max number of SDs that a mesh triangle can touch
#define MAX_SDs_TOUCHED_BY_TRIANGLE 8
/// The number of buckets used to host the triangles that touch a subset of SDs. If memory was not an issue, we'd
/// have as many an buckets as SD and one bucket would handle one SD. Yet this would lead to a lot of wasted memory
/// since most SDs have no triangles touching them. As such, the idea is to group a number of SDs associated with a
/// bucket and have that bucket store all the triangles touching this subset of SDs. A prime number for the bucket count
/// is good
#define TRIANGLEBUCKET_COUNT 4111
/// MAX_TRIANGLE_COUNT_PER_BUCKET indicates how much memory we set aside to store mesh triangles that belong to a
/// bucket. This number is not the amount of memory, rather the max number of triangles we anticipate to see in a
/// bucket. Note that if TRIANGLE_BUCKET_COUNT is small, a lot of SDs will send their triangles to the same bucket,
/// which means that MAX_TRIANGLE_COUNT_PER_BUCKET should be cranked up.
#define MAX_TRIANGLE_COUNT_PER_BUCKET 256

//////////////////////////////////////////////////////////////////////////////
/// COLLISION DETECTION RELATED QUANTITIES
//////////////////////////////////////////////////////////////////////////////
/// Fictitious radius of curvature for collision with a corner or an edge.
#define EDGE_RADIUS 0.1
