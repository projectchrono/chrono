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

#ifndef CHRANDOMPARTICLEPOSITION_H
#define CHRANDOMPARTICLEPOSITION_H

#include <memory>

#include "chrono/core/ChMathematics.h"
#include "chrono/core/ChVector.h"
#include "chrono/core/ChMatrix.h"
#include "chrono/core/ChDistribution.h"
#include "chrono/geometry/ChSurface.h"
#include "chrono/geometry/ChVolume.h"

namespace chrono {
namespace particlefactory {

/// BASE class for generators of random particle positions.
/// By default it simply always returns Pxyz={0,0,0}, so it
/// it is up to sub-classes to implement more sophisticated randomizations.
class ChRandomParticlePosition {
  public:
    ChRandomParticlePosition() {}
    virtual ~ChRandomParticlePosition() {}

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
    virtual ChVector<> RandomPosition() override {
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
        m_geometry = chrono_types::make_shared<geometry::ChBox>(ChVector<>(0.1, 0.1, 0.1));
    }

    /// Function that creates a random position each
    /// time it is called.
    virtual ChVector<> RandomPosition() override {
        if (auto mline = std::dynamic_pointer_cast<geometry::ChLine>(m_geometry)) {
            mline->Evaluate(m_frame.GetPos(), ChRandom());
        }
        else if (auto msurface = std::dynamic_pointer_cast<geometry::ChSurface>(m_geometry)) {
            msurface->Evaluate(m_frame.GetPos(), ChRandom(), ChRandom());
        }
        else if (auto mvolume = std::dynamic_pointer_cast<geometry::ChVolume>(m_geometry)) {
            mvolume->Evaluate(m_frame.GetPos(), ChRandom(), ChRandom(), ChRandom());
        }
        return m_frame.GetPos();
    }

    /// Set the parametric surface used for this outlet.
    /// The surface will be sampled uniformly over its U,V parametric coordinates. In cas of lines, oly U is used, in
    /// case of parametric volumes, U,V,W.
    void SetGeometry(std::shared_ptr<geometry::ChGeometry> geometry, const ChFrame<>& frame) {
        m_geometry = geometry;
        m_frame = frame;
    }

  private:
    std::shared_ptr<geometry::ChGeometry> m_geometry;
    ChFrame<> m_frame;
};

}  // end of namespace particlefactory
}  // end of namespace chrono

#endif
