//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2010 Alessandro Tasora
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//
// File author: A.Tasora

#ifndef CHRANDOMPARTICLEPOSITION_H
#define CHRANDOMPARTICLEPOSITION_H

#include "core/ChMathematics.h"
#include "core/ChVector.h"
#include "core/ChMatrix.h"
#include "core/ChDistribution.h"
#include "core/ChSmartpointers.h"

namespace chrono {
namespace particlefactory {

/// BASE class for generators of random particle positions.
/// By default it simply always returns Pxyz={0,0,0}, so it
/// it is up to sub-classes to implement more sophisticated randomizations.
class ChRandomParticlePosition : public ChShared {
  public:
    ChRandomParticlePosition() {}

    /// Function that creates a random position each
    /// time it is called.
    /// Note: it must be implemented by children classes!
    virtual ChVector<> RandomPosition() { return VNULL; }
};

/// Class for generator of random particle positions
/// scattered over a rectangle outlet in 3D space
class ChRandomParticlePositionRectangleOutlet : public ChRandomParticlePosition {
  public:
    ChRandomParticlePositionRectangleOutlet() {
        // defaults
        outlet = CSYSNORM;
        width = 0.1;
        height = 0.1;
    }

    /// Function that creates a random position each
    /// time it is called.
    virtual ChVector<> RandomPosition() {
        ChVector<> localp = ChVector<>(ChRandom() * width - 0.5 * width, ChRandom() * height - 0.5 * height, 0);
        return outlet.TransformLocalToParent(localp);
    }
    /// Access the coordinate system of the rectangular outlet.
    /// The outlet width is on the X direction of this csys, and the
    /// outled height is on the Y direction of this csys.
    ChCoordsys<>& Outlet() { return outlet; }

    /// Access the width of the rectangular outlet, that is on the X axis of the coordinate
    double& OutletWidth() { return width; }

    /// Access the height of the rectangular outlet, that is on the Y axis of the coordinate
    double& OutletHeight() { return height; }

  private:
    ChCoordsys<> outlet;
    double width;
    double height;
};

/// Class for generator of random particle positions
/// scattered over a parametric surface
class ChRandomParticlePositionOnGeometry : public ChRandomParticlePosition {
  public:
    ChRandomParticlePositionOnGeometry() {
        // defaults
        geometry = ChSmartPtr<geometry::ChGeometry>(
            new geometry::ChBox(VNULL, ChMatrix33<>(QUNIT), ChVector<>(0.1, 0.1, 0.1)));
    }

    /// Function that creates a random position each
    /// time it is called.
    virtual ChVector<> RandomPosition() {
        ChVector<> mpos;
        geometry->Evaluate(mpos, ChRandom(), ChRandom(), ChRandom());
        return mpos;
    }

    /// Set the parametric surface used for this outlet.
    /// The surface will be sampled uniformly over its U,V parametric
    /// coordinaters. In cas of lines, oly U is used, in case of parametric volumes, U,V,W.
    void SetGeometry(ChSmartPtr<geometry::ChGeometry> mgeometry) { this->geometry = mgeometry; }

  private:
    ChSmartPtr<geometry::ChGeometry> geometry;
};

}  // end of namespace particlefactory
}  // end of namespace chrono

#endif
