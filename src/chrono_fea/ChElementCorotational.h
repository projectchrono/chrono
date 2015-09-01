//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2013 Project Chrono
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//
// File authors: Andrea Favali, Alessandro Tasora

#ifndef CHCOROTATIONAL_H
#define CHCOROTATIONAL_H

#include "ChPolarDecomposition.h"
#include "ChMatrixCorotation.h"

namespace chrono {
namespace fea {

/// Class for corotational elements (elements with rotation
/// matrices that follow the global motion of the element)
class ChApiFea ChElementCorotational {
  protected:
    ChMatrix33<> A;  // rotation matrix

  public:
    ChElementCorotational() {
        A(0, 0) = 1;
        A(1, 1) = 1;
        A(2, 2) = 1;
    }

    /// Access the cumulative rotation matrix of the element,
    /// The rotation is expressed relative to initial reference
    /// position of element.
    ChMatrix33<>& Rotation() { return A; }

    /// Given the actual position of the nodes, recompute
    /// the cumulative rotation matrix A.
    /// CHLDREN CLASSES MUST IMPLEMENT THIS!!!
    virtual void UpdateRotation() = 0;
};

}  //___end of namespace fea___
}  //___end of namespace chrono___

#endif