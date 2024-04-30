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

#ifndef CH_VOLUME_H
#define CH_VOLUME_H

#include "chrono/geometry/ChGeometry.h"

namespace chrono {

/// @addtogroup chrono_geometry
/// @{

/// Base class for all geometric objects representing tri-parametric surfaces in 3D space.
/// This is the base for all U,V,W-parametric object, implementing Evaluate() that returns a point as a function of
/// three U,V,W parameters.
class ChApi ChVolume : public ChGeometry {
  public:
    ChVolume() {}
    ChVolume(const ChVolume& other) : ChGeometry(other) {}
    virtual ~ChVolume() {}

    /// "Virtual" copy constructor (covariant return type).
    // virtual ChVolume* Clone() const override { };

    /// Return the volume of this solid.
    virtual double GetVolume() const = 0;

    /// Return the gyration matrix for this solid.
    virtual ChMatrix33<> GetGyration() const = 0;

    /// Return a point in the volume, given parametric coordinates U,V,W.
    /// Parameters U, V, and W are always in the [0,1] range.
    /// The default implementation always returns the volume center.
    virtual ChVector3d Evaluate(double parU, double parV, double parW) const { return VNULL; }

    /// Return true if the volume is closed (periodic) in the 1st parametric coordinate.
    virtual bool IsClosedU() const { return false; }

    /// Return true if the volume is closed (periodic) in the 2nd parametric coordinate.
    virtual bool IslosedV() const { return false; }

    /// Return true if the volume is closed (periodic) in the 3rd parametric coordinate.
    virtual bool IsClosedW() const { return false; }

    /// Return dimensionality of this object.
    /// For a volume, this is always 3.
    virtual int GetManifoldDimension() const override final { return 3; }

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOut(ChArchiveOut& archive_out) override;

    /// Method to allow de-serialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& archive_in) override;
};

/// @} chrono_geometry

CH_CLASS_VERSION(ChVolume, 0)

}  // end namespace chrono

#endif
