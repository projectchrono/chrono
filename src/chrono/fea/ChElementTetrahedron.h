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

#ifndef CHELEMENTTETRAHEDRON_H
#define CHELEMENTTETRAHEDRON_H

#include "chrono/fea/ChElementGeneric.h"
#include "chrono/fea/ChElementCorotational.h"

namespace chrono {
namespace fea {

/// @addtogroup fea_elements
/// @{

/// Class for tetrahedral elements.
class ChApi ChElementTetrahedron : public ChElementGeneric, public ChElementCorotational {
  public:
    ChElementTetrahedron() : Volume(0) {}

    double GetVolume() { return Volume; }

    virtual void Update() {
        // parent class update:
        ChElementGeneric::Update();
        // always keep updated the rotation matrix A:
        this->UpdateRotation();
    }

  protected:
    double Volume;
};

/// @} fea_elements

}  // end namespace fea
}  // end namespace chrono

#endif
