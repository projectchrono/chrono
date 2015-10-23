// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All right reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban
// =============================================================================
//
// Utility functions for generating parameterized 2D sprocket gear profiles.
// These are used as 2D collision shapes (against similar profiles for track
// shoe bodies). They are composed of a number of sub-paths of type ChLineArc
// or ChLineSegment.
//
// =============================================================================

#ifndef CH_SPROCKET_PROFILES_H
#define CH_SPROCKET_PROFILES_H

#include "chrono/geometry/ChCLinePath.h"

#include "chrono_vehicle/ChApiVehicle.h"

namespace chrono {
namespace vehicle {

/// Create profile of circular teeth.
CH_VEHICLE_API ChSharedPtr<geometry::ChLinePath> ChCircularProfile(int num_teeth,  ///< number of teeth
                                                                   double R_T,     ///< radius at top of teeth
                                                                   double R_C,     ///< location of tooth arc centers
                                                                   double R        ///< radius of tooth arcs
                                                                   );

}  // end namespace vehicle
}  // end namespace chrono

#endif
