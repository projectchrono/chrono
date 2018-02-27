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
#define MAX_COUNT_OF_DEs_PER_SD 255
/// 2^LENGTH_UNIT_FACTOR is used in the process of AD-ing the length for monodisperse spheres
#define SPHERE_LENGTH_UNIT_FACTOR 12
/// 2^SPHERE_TIME_UNIT_FACTOR is used in the process of AD-ing the time for monodisperse spheres
#define SPHERE_TIME_UNIT_FACTOR 4
