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
// Authors: Andrea Favali, Alessandro Tasora
// =============================================================================

#ifndef CHELEMENT3D_H
#define CHELEMENT3D_H

#include "chrono/fea/ChElementGeneric.h"
#include "chrono/fea/ChGaussIntegrationRule.h"
#include "chrono/fea/ChPolarDecomposition.h"
#include "chrono/fea/ChMatrixCorotation.h"

namespace chrono {
namespace fea {

/// @addtogroup fea_elements
/// @{

/// Class for all 3-Dimensional elements.
class ChApi ChElement3D : public ChElementGeneric {
  protected:
    double Volume;

  public:
    double GetVolume() { return Volume; }
};

/// @} fea_elements

}  // end namespace fea
}  // end namespace chrono

#endif