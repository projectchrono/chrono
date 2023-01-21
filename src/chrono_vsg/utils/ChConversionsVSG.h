// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2023 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Radu Serban
// =============================================================================

#ifndef CH_CONVERSIONS_VSG_H
#define CH_CONVERSIONS_VSG_H

#include <vsg/maths/mat4.h>
#include <vsg/maths/vec3.h>

#include "chrono/core/ChFrame.h"

#include "chrono_vsg/ChApiVSG.h"

namespace vsg {
class CH_VSG_API dvec3CH : public dvec3 {
  public:
    dvec3CH(const chrono::ChVector<>& vec);
};

class CH_VSG_API dmat4CH : public dmat4 {
  public:
    dmat4CH(const chrono::ChFrame<>& frame, const chrono::ChVector<>& scale);
    dmat4CH(const chrono::ChFrame<>& frame, double scale);
};

}  // namespace vsg

#endif
