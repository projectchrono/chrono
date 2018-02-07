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

#define MAX_SDs_TOUCHED_BY_SPHERE 8         ///< At most 8 domains are touched by a sphere
#define MAX_COUNT_OF_DEs_PER_SD 150       /// Provides a rough count of max number of DEs per SD; helpful when setting aside memory

#define SPHERE_LENGTH_UNIT_FACTOR 12      /// 2^LENGTH_UNIT_FACTOR is used in the process of AD-ing the length for monodisperse spheres
#define SPHERE_TIME_UNIT_FACTOR    4      /// 2^SPHERE_TIME_UNIT_FACTOR is used in the process of AD-ing the time for monodisperse spheres

__constant__ unsigned int d_monoDisperseSphRadius_AD; //!< Ad-ed dimension of the spere radius

__constant__ unsigned int d_SD_Ldim_AD;    //!< Ad-ed L-dimension of the SD box
__constant__ unsigned int d_SD_Ddim_AD;    //!< Ad-ed D-dimension of the SD box
__constant__ unsigned int d_SD_Hdim_AD;    //!< Ad-ed H-dimension of the SD box

