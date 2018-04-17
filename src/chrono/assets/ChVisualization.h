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

#ifndef CHVISUALIZATION_H
#define CHVISUALIZATION_H

#include "chrono/assets/ChAsset.h"
#include "chrono/assets/ChColor.h"
#include "chrono/core/ChMath.h"

namespace chrono {

/// Base class for assets that define something about visualization (rendering,
/// post processing, etc.)
/// It contains basic information about position, color, and visibility.

class ChApi ChVisualization : public ChAsset {
  public:
    virtual ~ChVisualization() {}

    /// Set this visualization asset as visible.
    void SetVisible(bool mv) { visible = mv; }

    /// Return true if the asset is set as visible.
    bool IsVisible() const { return visible; }

    /// Set the color of the surface (default: white).
    /// This information can optionally be used by a visualization system.
    void SetColor(const ChColor& mc) { color = mc; }

    /// Return the color assigned to this asset.
    const ChColor& GetColor() const { return color; }

    /// Set the fading level, a value in [0,1] (default: 0).
    /// If fading = 0, the surface is completely opaque.
    /// If fading = 1, the surface is completely transparent.
    void SetFading(const float mc) { fading = mc; }

    /// Get the fading level.
    float GetFading() const { return fading; }

    /// Set this visualization asset as static (default: false).
    /// Set to true to indicate that the asset never changes and therefore does not require updates
    /// (e.g. for a non-deformable triangular mesh).
    /// A particular visualization system may take advantage of this setting to accelerate rendering.
    void SetStatic(bool val) { is_static = val; }

    /// Return true if the visualization asset is marked as static.
    bool IsStatic() const { return is_static; }

    ChVector<> Pos;    ///< Position of Asset
    ChMatrix33<> Rot;  ///< Rotation of Asset

  protected:
    ChVisualization();

    virtual void ArchiveOUT(ChArchiveOut& marchive) override;
    virtual void ArchiveIN(ChArchiveIn& marchive) override;

    bool visible;
    bool is_static;
    ChColor color;
    float fading;
};

CH_CLASS_VERSION(ChVisualization, 0)

}  // end namespace chrono

#endif
