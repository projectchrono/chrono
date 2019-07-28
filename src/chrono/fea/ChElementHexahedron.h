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

#include "chrono/fea/ChElement3D.h"
#include "chrono/fea/ChElementCorotational.h"

namespace chrono {
namespace fea {

/// Class for hexahedral elements.
class ChApi ChElementHexahedron : public ChElement3D,
                                  public ChElementCorotational
//                                        //       __ __ __ __      //
{                                         //      /           /|    //
  protected:                              //     /_ __ __ __ / |    //
    ChGaussIntegrationRule* ir;           //    |           |  |    //
    std::vector<ChGaussPoint*> GpVector;  //    |           |  |    //
                                          //    |           |  |    //
                                          //    |           | /     //
                                          //    |__ __ __ __|/      //
  public:
    int ID;

    ChElementHexahedron() : ir(nullptr) {}
    
    virtual ~ChElementHexahedron() {
        delete ir;
        for (auto gpoint : GpVector)
            delete gpoint;
        GpVector.clear();
    }

    virtual void Update() {
        // parent class update:
        ChElement3D::Update();
        // always keep updated the rotation matrix A:
        this->UpdateRotation();
    }
};

}  // end namespace fea
}  // end namespace chrono

#endif
