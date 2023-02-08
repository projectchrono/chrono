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
// Authors: Alessandro Tasora
// =============================================================================
// =============================================================================


#include "chrono/geometry/ChProperty.h"

namespace chrono {
namespace geometry {

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChPropertyScalar)
CH_FACTORY_REGISTER(ChPropertyColor)
CH_FACTORY_REGISTER(ChPropertyVector)
CH_FACTORY_REGISTER(ChPropertyQuaternion)




}  // end namespace geometry
}  // end namespace chrono
