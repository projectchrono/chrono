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

#include <climits>

#pragma once

///< At most 8 domains are touched by a sphere
#define MAX_SDs_TOUCHED_BY_SPHERE 8
/// The L-size of an SD should contain, on average, about these many spheres
#define AVERAGE_SPHERES_PER_SD_L_DIR 4.5
/// The D-size of an SD should contain, on average, about these many spheres
#define AVERAGE_SPHERES_PER_SD_D_DIR 4.5
/// The H-size of an SD should contain, on average, about these many spheres
#define AVERAGE_SPHERES_PER_SD_H_DIR 4.5
/// Anticipated max number of DEs in an SD; used for setting aside memory ahead of time
#define MAX_COUNT_OF_DEs_PER_SD 250
/// 2^LENGTH_UNIT_FACTOR is used in the process of AD-ing the length for monodisperse spheres
#define SPHERE_LENGTH_UNIT_FACTOR 12
/// 2^SPHERE_TIME_UNIT_FACTOR is used in the process of AD-ing the time for monodisperse spheres
#define SPHERE_TIME_UNIT_FACTOR 4
/// Value that indicates non-valid ID. The assumption is that an ID is always a positive integer
#define NULL_GRANULAR_ID UINT_MAX - 1
/// Value that indicates a non-valid (ILLegal) physical attribute. The assumption is that this attribute is always an integer
#define ILL_GRANULAR_VAL INT_MAX - 1
/// The number of average contacts per DE. High values are safe but might also translate into wasted memory
#define AVERAGE_COUNT_CONTACTS_PER_DE 8

