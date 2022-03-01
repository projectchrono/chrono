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
// Radu Serban
// =============================================================================

#ifndef CH_VISUAL_MODEL_H
#define CH_VISUAL_MODEL_H

#include <vector>

#include "chrono/core/ChApiCE.h"
#include "chrono/core/ChFrame.h"
#include "chrono/assets/ChVisualShape.h"

namespace chrono {

// Forward declaration
class ChPhysicsItem;

/// Base class for a visual model which encapsulates a set of visual shapes.
/// Visual models can be instantiated and shared among different Chrono physics items.
class ChApi ChVisualModel {
  public:
    ChVisualModel() {}
    ~ChVisualModel() {}

    /// Add a visual shape with specified position within the model.
    void AddShape(std::shared_ptr<ChVisualShape> shape,  ///< visualization shape
                  const ChFrame<>& frame = ChFrame<>()   ///< shape frame in model
    );

    /// Get the visual shapes in the model.
    const std::vector<std::shared_ptr<ChVisualShape>>& GetShapes() const { return m_shapes; }

    /// Get the specified visual shape in the model.
    std::shared_ptr<ChVisualShape> GetShape(unsigned int i) const { return m_shapes[i]; }

    /// Update this visual model with information for the owning physical object.
    void Update(const ChFrame<>& frame) {}

  private:
    std::vector<std::shared_ptr<ChVisualShape>> m_shapes;
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
