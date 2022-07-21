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

#ifndef CH_CASCADE_TRIANGULATE_H
#define CH_CASCADE_TRIANGULATE_H

#include "chrono/core/ChTypes.h"

namespace chrono {
namespace cascade {

/// @addtogroup cascade_module
/// @{

/// Class for storing settings on OpenCASCADE tesselation of shapes.
class ChCascadeTriangulate {
  public:
    ChCascadeTriangulate(
        double defl = 0.05,   ///< maximum allowed chordal deflection
        bool is_rel = false,  ///< chordal deflection is assumed relative to triangle chord (default false)
        double ang = 0.5      ///< angular deflection
        )
        : deflection_is_relative(is_rel), deflection(defl), angular_deflection(ang) {}

    bool deflection_is_relative;
    double deflection;
    double angular_deflection;
};

/// @} cascade_module

}  // end namespace cascade
}  // end namespace chrono

#endif
