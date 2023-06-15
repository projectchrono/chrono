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

    /// Evaluates a point on the surface, given parametric coordinates U,V.
    /// Parameters U and V always work in 0..1 range.
    /// Computed value goes into the 'pos' reference.
    /// It must be implemented by inherited classes.
    virtual void Evaluate(ChVector<>& pos, const double parU, const double parV) const = 0;

    /// Evaluates a normal versor, given parametric coordinates U,V.
    /// Parameters U,V always work in 0..1 range.
    /// Computed value (normalized) goes into the 'pos' reference.
    /// It could be overridden by inherited classes if a precise solution is
    /// known (otherwise it defaults to numerical BDF using the Evaluate()
    /// function).
    virtual void Normal(ChVector<>& dir, const double parU, const double parV) const;

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
