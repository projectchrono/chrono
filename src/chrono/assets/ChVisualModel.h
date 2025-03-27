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
// Authors: Radu Serban
// =============================================================================

#ifndef CH_VISUAL_MODEL_H
#define CH_VISUAL_MODEL_H

#include <vector>

#include "chrono/core/ChApiCE.h"
#include "chrono/core/ChFrame.h"
#include "chrono/assets/ChVisualShape.h"
#include "chrono/assets/ChVisualShapeFEA.h"

namespace chrono {

/// @addtogroup chrono_assets
/// @{

// Forward declaration
class ChObj;

/// Definition of an instance of a visual shape in a visual model.
struct ChApi ChVisualShapeInstance {
    std::shared_ptr<ChVisualShape> shape;  /// visual shape (possibly shared)
    ChFramed frame;                        ///< shape position relative to containing model
    bool wireframe;                        ///< wireframe rendering for this instance

    void ArchiveOut(ChArchiveOut& archive_out);
    void ArchiveIn(ChArchiveIn& archive_in);
};

/// Definition of a visual model which encapsulates a set of visual shapes.
/// Visual models can be instantiated and shared among different Chrono objects.
class ChApi ChVisualModel {
  public:
    ChVisualModel() {}
    ~ChVisualModel() {}

    /// Add a visual shape with specified position within the model.
    void AddShape(std::shared_ptr<ChVisualShape> shape,  ///< visualization shape
                  const ChFramed& frame = ChFrame<>(),   ///< shape frame in model
                  bool wireframe = false                 ///< solid rendering by default
    );

    /// Add visual shapes for an FEA mesh to this model.
    void AddShapeFEA(std::shared_ptr<ChVisualShapeFEA> shapeFEA);

    /// Get the number of visual shapes in the model.
    unsigned int GetNumShapes() const { return (unsigned int)m_shapes.size(); }

    /// Get the number of FEA visual shapes in the model.
    unsigned int GetNumShapesFEA() const { return (unsigned int)m_shapesFEA.size(); }

    /// Get the visual shapes in the model.
    const std::vector<ChVisualShapeInstance>& GetShapeInstances() const { return m_shapes; }

    /// Get the specified visual shape in the model.
    std::shared_ptr<ChVisualShape> GetShape(unsigned int i) const { return m_shapes[i].shape; }

    /// Get the coordinate frame of the specified visual shape in the model (relative to the model frame).
    const ChFrame<>& GetShapeFrame(unsigned int i) const { return m_shapes[i].frame; }

    /// Enable wireframe mode for all shapes in the model.
    void EnableWireframe(bool val = true);

    /// Enable/disable wireframe rendering mode for the specified shape in the model (default: false).
    void EnableWireframe(unsigned int i, bool val = true) { m_shapes[i].wireframe = val; }

    /// Get the rendering mode of the specifiedc shape instance in the model.
    bool UseWireframe(unsigned int i) const { return m_shapes[i].wireframe; }

    /// Get the FEA visualization shapes in the model.
    const std::vector<std::shared_ptr<ChVisualShapeFEA>>& GetShapesFEA() const { return m_shapesFEA; }

    /// Get the specified FEA visualization object in the model.
    std::shared_ptr<ChVisualShapeFEA> GetShapeFEA(unsigned int i) const { return m_shapesFEA[i]; }

    /// Erase all shapes in this model.
    void Clear();

    /// Erase the specified visual shape from this model.
    void Erase(std::shared_ptr<ChVisualShape> shape);

    /// Return the axis aligned bounding box (AABB) of the visual model.
    ChAABB GetBoundingBox() const;

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOut(ChArchiveOut& archive_out);

    /// Method to allow deserialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& archive_in);

  private:
    /// Update this visual model with information for the owning physical object.
    /// Since a visual model can be shared in multiple instances, this function may be called with different owners.
    void Update(ChObj* owner, const ChFrame<>& frame);

    std::vector<ChVisualShapeInstance> m_shapes;
    std::vector<std::shared_ptr<ChVisualShapeFEA>> m_shapesFEA;

    friend class ChVisualModelInstance;
};

/// Definition of a visual model instance which contains a potentially shared visual model and its owning object.
/// Multiple instances of a visual model can be associated with the same object or the same visual model can be
/// instanced in multiple objects.
class ChApi ChVisualModelInstance {
  public:
    ~ChVisualModelInstance() {}

    /// Get the associated visual model.
    std::shared_ptr<ChVisualModel> GetModel() const { return m_model; }

    /// Update this visual model with information for the owning physical object.
    void Update(const ChFrame<>& frame);

  private:
    ChVisualModelInstance(std::shared_ptr<ChVisualModel> model);

    std::shared_ptr<ChVisualModel> m_model;
    ChObj* m_owner;

    friend class ChObj;
};

/// @} chrono_assets

}  // namespace chrono

#endif
