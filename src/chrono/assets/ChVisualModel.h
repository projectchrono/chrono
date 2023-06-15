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

// Forward declaration
class ChPhysicsItem;

/// Base class for a visual model which encapsulates a set of visual shapes.
/// Visual models can be instantiated and shared among different Chrono physics items.
class ChApi ChVisualModel {
  public:
    /// A ShapeInstance is a pair of a (shared) shape and its position in the model.
    typedef std::pair<std::shared_ptr<ChVisualShape>, ChFrame<>> ShapeInstance;

    ChVisualModel() {}
    ~ChVisualModel() {}

    /// Add a visual shape with specified position within the model.
    void AddShape(std::shared_ptr<ChVisualShape> shape,  ///< visualization shape
                  const ChFrame<>& frame = ChFrame<>()   ///< shape frame in model
    );

    /// Add visual shapes for an FEA mesh to this model.
    void AddShapeFEA(std::shared_ptr<ChVisualShapeFEA> shapeFEA);

    /// Get the number of visual shapes in the model.
    int GetNumShapes() const { return (int)m_shapes.size(); }

    /// Get the visual shapes in the model.
    const std::vector<ShapeInstance>& GetShapes() const { return m_shapes; }

    /// Get the specified visual shape in the model.
    std::shared_ptr<ChVisualShape> GetShape(unsigned int i) const { return m_shapes[i].first; }

    /// Get the number of visual shapes in the model.
    int GetNumShapesFEA() const { return (int)m_shapesFEA.size(); }

    /// Get the FEA visualization shapes in the model.
    const std::vector<std::shared_ptr<ChVisualShapeFEA>>& GetShapesFEA() const { return m_shapesFEA; }

    /// Get the specified FEA visualization object in the model.
    std::shared_ptr<ChVisualShapeFEA> GetShapeFEA(unsigned int i) const { return m_shapesFEA[i]; }

    /// Get the coordinate frame of the specified visual shape in the model (relative to the model frame).
    const ChFrame<>& GetShapeFrame(unsigned int i) const { return m_shapes[i].second; }

    /// Erase all shapes in this model.
    void Clear();

    /// Erase the specified visual shape from this model.
    void Erase(std::shared_ptr<ChVisualShape> shape);

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOut(ChArchiveOut& marchive);

    /// Method to allow deserialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& marchive);

  private:
    /// Update this visual model with information for the owning physical object.
    /// Since a visual model can be shared in multiple instances, this function may be called with different owners.
    void Update(ChPhysicsItem* owner, const ChFrame<>& frame);

    std::vector<ShapeInstance> m_shapes;
    std::vector<std::shared_ptr<ChVisualShapeFEA>> m_shapesFEA;

    friend class ChVisualModelInstance;
};

/// A visual model instance encodes a potentially shared visual model and its owning physics item.
/// Multiple instances of a visual model can be associated with the same physics item or the same visual model can be
/// instanced in multiple physics items.
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
    ChPhysicsItem* m_owner;

    friend class ChPhysicsItem;
};

}  // namespace chrono

#endif
