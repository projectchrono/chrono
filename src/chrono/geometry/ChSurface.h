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

#ifndef CHC_SURFACE_H
#define CHC_SURFACE_H

#include <cmath>

#include "chrono/core/ChFilePS.h"
#include "chrono/geometry/ChGeometry.h"

namespace chrono {
namespace geometry {

/// Base class for all geometric objects representing bi-parametric surfaces in 3D space.
/// This is the base for all U,V-parametric object, implementing Evaluate()
/// that returns a point as a function of two U,V parameters.
class ChApi ChSurface : public ChGeometry {
  protected:
    // bool closed_U;
    bool wireframe;

  public:
    ChSurface() : wireframe(false) {}
    ChSurface(const ChSurface& source);
    virtual ~ChSurface() {}

    /// "Virtual" copy constructor (covariant return type).
    // virtual ChSurface* Clone() const override { }

    /// Return a point on the surface, given parametric coordinates U,V.
    /// Parameters U and V always work in 0..1 range.
    /// The default implementation always returns the origin of the surface frame.
    virtual ChVector<> Evaluate(double parU, double parV) const { return VNULL; }

    /// Return the normal unit vector at the parametric coordinates U,V (in the range [0,1]).
    /// Computed value (normalized) goes into the 'pos' reference.
    /// This default implementation uses finite differences.
    virtual ChVector<> GetNormal(double parU, double parV) const;

    /// Tell if the surface is closed (periodic) on U
    virtual bool Get_closed_U() const { return false; }

    /// Tell if the surface is closed (periodic) on V
    virtual bool Get_closed_V() const { return false; }

    /// This is a surface, so manifold dimension=2
    virtual int GetManifoldDimension() const override { return 2; }

    /// Tell if the visualization is done only as UV isolines
    bool IsWireframe() { return wireframe; }
    /// Set if the visualization is done only as UV isolines
    void SetWireframe(bool mw) { wireframe = mw; }

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOut(ChArchiveOut& marchive) override;

    /// Method to allow de-serialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& marchive) override;
};

}  // end namespace geometry

CH_CLASS_VERSION(geometry::ChSurface, 0)

}  // end namespace chrono

#endif
