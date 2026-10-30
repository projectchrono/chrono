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
// Authors: Andrea Favali, Alessandro Tasora, Radu Serban
// =============================================================================

#ifndef CHNODEFEABASE_H
#define CHNODEFEABASE_H

#include "chrono/physics/ChNodeBase.h"

namespace chrono {

class ChCheckpointWriter;
class ChCheckpointReader;

namespace fea {

/// @addtogroup fea_nodes
/// @{

// Forward
class ChMesh;

/// Base class for a generic finite element node that can be stored in ChMesh containers.
/// Derived classes must implement specialized versions.
class ChApi ChNodeFEAbase : public virtual ChNodeBase {
  public:
    ChNodeFEAbase() {}

    /// Set the rest position as the actual position.
    virtual void Relax() = 0;

    /// Reset to no speed and acceleration.
    virtual void ForceToRest() = 0;

    /// Fix/release this node.
    /// If fixed, its state variables are not changed by the solver.
    virtual void SetFixed(bool fixed) = 0;

    /// Return true if the node is fixed (i.e., its state variables are not changed by the solver).
    virtual bool IsFixed() const = 0;

    /// Sets the global index of the node
    virtual void SetIndex(unsigned int mindex) { g_index = mindex; }

    /// Gets the global index of the node
    virtual unsigned int GetIndex() { return g_index; }

    /// Initial setup.
    /// Called once by the containing mesh, before the node is first used in a simulation.
    virtual void SetupInitial(ChSystem* system) {}

    /// Update the number of active coordinates from the current fixed state of the node.
    /// Called by the containing mesh every time it counts coordinates. Only needed for nodes whose number of active
    /// coordinates can change.
    virtual void UpdateActiveCoords() {}

    /// Return true if the internal state of this node can be saved in a checkpoint (see ChPhysicsItem).
    virtual bool SupportsCheckpoint() const { return true; }

    /// Save the internal state of this node: data not stored in the system state vectors that it carries from step to
    /// step (for example, plastic strains). Derived classes must first call the base class implementation.
    virtual void SaveInternalState(ChCheckpointWriter& writer) const {}

    /// Load the internal state of this node, as saved by SaveInternalState.
    virtual void LoadInternalState(ChCheckpointReader& reader) {}

    double m_TotalMass;  ///< Nodal mass obtained from element mass matrix

  protected:
    unsigned int g_index;  ///< global node index

  private:
    bool m_initialized = false;  ///< true once SetupInitial was called by the containing mesh

    friend class ChMesh;
};

/// @} fea_nodes

}  // end namespace fea
}  // end namespace chrono

#endif
