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

#ifndef CH_VISUAL_SHAPE_SURFACE_H
#define CH_VISUAL_SHAPE_SURFACE_H

#include "chrono/assets/ChVisualShape.h"
#include "chrono/geometry/ChSurface.h"

namespace chrono {

/// @addtogroup chrono_assets
/// @{

/// Class for referencing a ChSurface u,v, parametric surface that can be visualized in some way.
class ChApi ChVisualShapeSurface : public ChVisualShape {
  public:
    ChVisualShapeSurface() : wireframe(false), resolution_U(5), resolution_V(5) {}

    ChVisualShapeSurface(std::shared_ptr<geometry::ChSurface> msurf)
        : gsurface(msurf), wireframe(false), resolution_U(5), resolution_V(5) {}

    ~ChVisualShapeSurface() {}

    /// Get the surface  geometry.
    std::shared_ptr<geometry::ChSurface> GetSurfaceGeometry() { return gsurface; }

    /// Set the surface geometry.
    void SetSurfaceGeometry(std::shared_ptr<geometry::ChSurface> ms) { gsurface = ms; }

    /// Tell if showing only UV isolines.
    bool IsWireframe() const { return wireframe; }

    /// Set if using only isolines.
    void SetWireframe(bool mw) { wireframe = mw; }

    /// Subdivision density for tesselation.
    int GetResolutionU() { return resolution_U; }

    /// Subdivision density for tesselation.
    int GetResolutionV() { return resolution_V; }

    /// Set u- subdivision density for tesselation.
    void SetResolutionU(int mr) { resolution_U = mr; }

    /// Set v- subdivision density for tesselation.
    void SetResolutionV(int mr) { resolution_V = mr; }

    /// Set subdivision density for tesselation (both for u and v).
    void SetResolution(int mr) {
        resolution_U = mr;
        resolution_V = mr;
    }

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOut(ChArchiveOut& marchive) override;

    /// Method to allow de-serialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& marchive) override;

  private:
    std::shared_ptr<geometry::ChSurface> gsurface;

    bool wireframe;
    int resolution_U;
    int resolution_V;
};

/// @} chrono_assets

CH_CLASS_VERSION(ChVisualShapeSurface, 0)

}  // end namespace chrono

#endif
