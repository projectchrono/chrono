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

#include "chrono/assets/ChColor.h"
#include "chrono/assets/ChVisualMaterial.h"

namespace chrono {

class ChPhysicsItem;

/// Base class for a visualization asset for rendering (run-time or post processing).
/// Encapsulates basic information about the shape position, materials, and visibility.
class ChApi ChVisualShape {
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

    /// Set opacity for this shape (0: fully transparent; 1: fully opaque).
    /// This changes the first material in the list of materials for this shape.
    /// If no materials are defined for a shape, one is first created by duplicating the default material.
    void SetOpacity(float val);

    /// Get opacity of the first material in the list of materials for this shape.
    /// If no materials are defined, return the color of the default material.
    float GetOpacity() const;

    /// Set the diffuse texture map for this shape.
    /// This changes the texture of the first material in the list of materials for this shape.
    /// If no materials are defined for a shape, one is first created by duplicating the default material.
    void SetTexture(const std::string& filename, float scale_x = 1, float scale_y = 1);

    /// Return the diffuse texture map of the first material in the list of materials for this shape.
    /// If no materials are defined, return an empty string (no texture for the default material).
    std::string GetTexture() const;

    /// Set this visualization shape as modifiable (default: false for primitive shapes, true otherwise).
    /// Set to false to indicate that the asset never changes and therefore does not require updates
    /// (e.g. for a non-deformable triangular mesh). Note that this also includes changes in materials.
    /// A particular visualization system may take advantage of this setting to accelerate rendering.
    void SetMutable(bool val) { is_mutable = val; }

    /// Return true if the visualization shape is marked as modifiable.
    bool IsMutable() const { return is_mutable; }

    /// Add a visualization material and return its index in the list of materials.
    int AddMaterial(std::shared_ptr<ChVisualMaterial> material);

    /// Replace the material with specified index.
    /// No-op if there is no material with given index.
    void SetMaterial(int i, std::shared_ptr<ChVisualMaterial> material);

    /// Get the list of visualization materials.
    std::vector<std::shared_ptr<ChVisualMaterial>>& GetMaterials() { return material_list; }

    /// Get the specified material in the list.
    std::shared_ptr<ChVisualMaterial> GetMaterial(int i) { return material_list[i]; }

    /// Get the number of visualization materials.
    int GetNumMaterials() const { return (int)material_list.size(); }

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOut(ChArchiveOut& marchive);

    /// Method to allow de-serialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& marchive);

  protected:
    ChVisualShape();

    /// Update this visual shape with information for the owning physical object.
    /// Since a visual shape can be shared in multiple instances, this function may be called with different updaters.
    virtual void Update(ChPhysicsItem* updater, const ChFrame<>& frame) {}

    bool visible;     ///< shape visibility flag
    bool is_mutable;  ///< flag indicating whether the shape is rigid or deformable

    std::vector<std::shared_ptr<ChVisualMaterial>> material_list;  ///< list of visualization materials

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    friend class ChVisualModel;
};

CH_CLASS_VERSION(ChVisualShape, 0)

}  // end namespace chrono

#endif
