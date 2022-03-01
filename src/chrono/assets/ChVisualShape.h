// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2022 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================

#ifndef CH_VISUAL_SHAPE_H
#define CH_VISUAL_SHAPE_H

#include "chrono/core/ChMath.h"
#include "chrono/core/ChFrame.h"

#include "chrono/assets/ChAsset.h"
#include "chrono/assets/ChColor.h"
#include "chrono/assets/ChVisualMaterial.h"

namespace chrono {

/// Base class for a visualization asset for rendering (run-time or post processing).
/// Encapsulates basic information about the shape position, materials, and visibility.
class ChApi ChVisualShape : public ChAsset {
  public:
    virtual ~ChVisualShape() {}

    /// Set this visualization asset as visible.
    void SetVisible(bool mv) { visible = mv; }

    /// Return true if the asset is set as visible.
    bool IsVisible() const { return visible; }

    /// Set the diffuse color for this shape.
    /// This changes the color of the first material in the list of materials for this shape.
    /// If no materials are defined for a shape, one is first created by duplicating the default material.
    void SetColor(const ChColor& col);

    /// Return the diffuse color of the first material in the list of materials for this shape.
    /// If no materials are defined, return the color of the default material.
    ChColor GetColor() const;

    /// Set the diffuse texture map for this shape.
    /// This changes the texture of the first material in the list of materials for this shape.
    /// If no materials are defined for a shape, one is first created by duplicating the default material.
    void SetTexture(const std::string& filename);

    /// Return the diffuse texture map of the first material in the list of materials for this shape.
    /// If no materials are defined, return an empty string (no texture for the default material).
    const std::string& GetTxture() const;

    /// Set the fading level, a value in [0,1] (default: 0).
    /// If fading = 0, the surface is completely opaque.
    /// If fading = 1, the surface is completely transparent.
    void SetFading(const float mc) { fading = mc; }

    /// Get the fading level.
    float GetFading() const { return fading; }

    /// Set this visualization shape as static (default: false).
    /// Set to true to indicate that the asset never changes and therefore does not require updates
    /// (e.g. for a non-deformable triangular mesh).
    /// A particular visualization system may take advantage of this setting to accelerate rendering.
    void SetStatic(bool val) { is_static = val; }

    /// Return true if the visualization shape is marked as static.
    bool IsStatic() const { return is_static; }

    /// Add a visualization material and return its index in the list of materials.
    int AddMaterial(std::shared_ptr<ChVisualMaterial> material);

    /// Get the list of visualization materials.
    std::vector<std::shared_ptr<ChVisualMaterial>>& GetMaterials() { return material_list; }

    /// Get the specified material in the list.
    std::shared_ptr<ChVisualMaterial> GetMaterial(int i) { return material_list[i]; }

    /// Get the number of visualization materials.
    int GetNumMaterials() const { return (int)material_list.size(); }

    ChVector<> Pos;    ///< asset position
    ChMatrix33<> Rot;  ///< asset orientation

  protected:
    ChVisualShape();

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOUT(ChArchiveOut& marchive) override;

    /// Method to allow de-serialization of transient data from archives.
    virtual void ArchiveIN(ChArchiveIn& marchive) override;

    ChFrame<> frame; ///< shape position relative to containing model

    bool visible;
    bool is_static;
    float fading;

    std::vector<std::shared_ptr<ChVisualMaterial>> material_list;  ///< list of visualization materials

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    friend class ChVisualModel;
};

CH_CLASS_VERSION(ChVisualShape, 0)

}  // end namespace chrono

#endif
