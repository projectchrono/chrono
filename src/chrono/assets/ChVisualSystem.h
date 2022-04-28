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

#ifndef CH_VISUAL_SYSTEM_H
#define CH_VISUAL_SYSTEM_H

#include <vector>

#include "chrono/core/ChApiCE.h"
#include "chrono/physics/ChSystem.h"
#include "chrono/physics/ChPhysicsItem.h"

namespace chrono {

/// Vertical direction
enum class CameraVerticalDir { Y, Z };

/// Base class for a Chrono run-time visualization system.
class ChApi ChVisualSystem {
  public:
    virtual ~ChVisualSystem() {}

    /// Process all visual assets in the associated ChSystem.
    /// This function is called by default when the visualization system is attached to a Chrono system (using
    /// ChSystem::SetVisualSystem()), but can also be called later if further modifications to visualization assets
    /// occur.
    virtual void BindAll() {}

    /// Process the visual assets for the spcified physics item.
    /// This function must be called if a new physics item is added to the system or if changes to its visual model
    /// occur after the visualization system was attached to the Chrono system.
    virtual void BindItem(std::shared_ptr<ChPhysicsItem> item) {}

    /// Create a snapshot of the last rendered frame and save it to the provided file.
    /// The file extension determines the image format.
    virtual void WriteImageToFile(const std::string& filename) {}

    /// Enable modal analysis visualization.
    /// If supported, visualize an oscillatory motion of the n-th mode (if the associated system contains a
    /// ChModalAssembly).
    virtual void EnableModalAnalysis(bool val) {}

    /// Set the mode to be shown (only if some ChModalAssembly is found).
    virtual void SetModalModeNumber(int val) {}

    /// Set the amplitude of the shown mode (only if some ChModalAssembly is found).
    virtual void SetModalAmplitude(double val) {}

    /// Set the speed of the shown mode (only if some ChModalAssembly is found).
    virtual void SetModalSpeed(double val) {}

    /// Get the associated Chrono system.
    const ChSystem& GetSystem() const { return *m_system; }

  protected:
    ChVisualSystem() : m_system(nullptr) {}

    /// Perform any necessary operations when the visualization system is attached to a ChSystem.
    /// Called by the associated ChSystem after it sets m_system.
    virtual void OnAttach() {}

    /// Perform any necessary setup operations at the beginning of a time step.
    /// Called by the associated ChSystem.
    virtual void OnSetup() {}

    /// Perform any necessary update operations at the end of a time step.
    /// Called by the associated ChSystem.
    virtual void OnUpdate() {}

    /// Remove all visualization objects from this visualization system.
    /// Called by the associated ChSystem.
    virtual void OnClear() {}

    ChSystem* m_system;  ///< associated Chrono system

    friend class ChSystem;
};

}  // namespace chrono

#endif
