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
// Authors: Alessandro Tasora, Radu Serban
// =============================================================================

#ifndef CHC_VOLUME_H
#define CHC_VOLUME_H

#include <cmath>

#include "chrono/core/ChFilePS.h"
#include "chrono/geometry/ChGeometry.h"

namespace chrono {
namespace geometry {

/// Base class for all geometric objects representing tri-parametric surfaces in 3D space.
/// This is the base for all U,V,W-parametric object, implementing Evaluate() that returns a point as a function of
/// three U,V,W parameters.
class ChApi ChVolume : public ChGeometry {
  public:
    ChVolume() {}
    ChVolume(const ChVolume& source){};
    virtual ~ChVolume() {}

    /// "Virtual" copy constructor (covariant return type).
    // virtual ChVolume* Clone() const override { };

    /// Return the volume of this solid.
    virtual double GetVolume() const = 0;

    /// Return the gyration matrix for this solid.
    virtual ChMatrix33<> GetGyration() const = 0;

    /// Return a point in the volume, given parametric coordinates U,V,W.
    /// Parameters U V W always work in 0..1 range.
    /// The default implementation always returns the volume center.
    virtual ChVector<> Evaluate(double parU, double parV, double parW) const { return VNULL; }

    /// Tell if the volume is closed (periodic) in parametric coordinate
    virtual bool Get_closed_U() const { return false; }

    /// Tell if the volume is closed (periodic) in parametric coordinate
    virtual bool Get_closed_V() const { return false; }

    /// Tell if the volume is closed (periodic) in parametric coordinate
    virtual bool Get_closed_W() const { return false; }

    /// This is a volume.
    virtual int GetManifoldDimension() const override final { return 3; }

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOut(ChArchiveOut& marchive) override;

    /// Method to allow de-serialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& marchive) override;
};

}  // end namespace geometry

CH_CLASS_VERSION(geometry::ChVolume, 0)

}  // end namespace chrono

#endif
