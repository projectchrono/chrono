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

#ifndef CHELEMENTHEXAHEDRON_H
#define CHELEMENTHEXAHEDRON_H

#include "chrono/fea/ChElementGeneric.h"
#include "chrono/fea/ChElementCorotational.h"
#include "chrono/fea/ChGaussIntegrationRule.h"

namespace chrono {
namespace fea {

/// @addtogroup fea_elements
/// @{

/// Class for hexahedral elements.
class ChApi ChElementHexahedron : public ChElementGeneric, public ChElementCorotational {
  public:
    ChElementHexahedron() : ir(nullptr), Volume(0) {}

    virtual ~ChElementHexahedron() {
        delete ir;
        for (auto gpoint : GpVector)
            delete gpoint;
        GpVector.clear();
    }

    double GetVolume() { return Volume; }

    virtual void Update() {
        // parent class update:
        ChElementGeneric::Update();
        // always keep updated the rotation matrix A:
        this->UpdateRotation();
    }

  protected:
    ChGaussIntegrationRule* ir;
    std::vector<ChGaussPoint*> GpVector;
    double Volume;
};

/// @} fea_elements

}  // end namespace fea
}  // end namespace chrono

#endif
