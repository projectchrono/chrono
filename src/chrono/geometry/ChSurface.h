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

#ifndef CH_SURFACE_H
#define CH_SURFACE_H

#include "chrono/geometry/ChGeometry.h"

namespace chrono {

/// @addtogroup chrono_geometry
/// @{

/// Base class for all geometric objects representing bi-parametric surfaces in 3D space.
/// This is the base for all U,V-parametric object, implementing Evaluate()
/// that returns a point as a function of two U,V parameters.
class ChApi ChSurface : public ChGeometry {
  public:
    ChSurface() : wireframe(false) {}
    ChSurface(const ChSurface& other);
    virtual ~ChSurface() {}

    /// "Virtual" copy constructor (covariant return type).
    // virtual ChSurface* Clone() const override { }

    /// Return a point on the surface, given parametric coordinates U,V.
    /// Parameters U and V are always in the [0,1] range.
    /// The default implementation always returns the origin of the surface frame.
    virtual ChVector3d Evaluate(double parU, double parV) const { return VNULL; }

    /// Return the normal unit vector at the parametric coordinates (U,V).
    /// Parameters U and V are always in the [0,1] range.
    virtual ChVector3d GetNormal(double parU, double parV) const;

    /// Return true if the surface is closed (periodic) in the 1st parametric coordinate.
    virtual bool IsClosedU() const { return false; }

    /// Return true if the surface is closed (periodic) in the 2nd parametric coordinate.
    virtual bool IslosedV() const { return false; }

    /// Return dimensionality of this object.
    /// For a surface, this is always 2.
    virtual int GetManifoldDimension() const override final { return 2; }

    /// Return true if visualization is done only as UV isolines.
    bool IsWireframe() { return wireframe; }

    /// Enable/disable visualization as wireframe.
    void SetWireframe(bool mw) { wireframe = mw; }

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOut(ChArchiveOut& archive_out) override;

    /// Method to allow de-serialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& archive_in) override;

  protected:
    // bool closed_U;
    bool wireframe;
};

/// @} chrono_geometry

CH_CLASS_VERSION(ChSurface, 0)

}  // end namespace chrono

#endif
