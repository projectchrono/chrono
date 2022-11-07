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
    virtual void SetNoSpeedNoAcceleration() = 0;

    /// Fix/release this node.
    /// If fixed, its state variables are not changed by the solver.
    virtual void SetFixed(bool mev) = 0;

    /// Return true if the node is fixed (i.e., its state variables are not changed by the solver).
    virtual bool IsFixed() const = 0;

    /// Sets the global index of the node
    virtual void SetIndex(unsigned int mindex) { g_index = mindex; }

    /// Gets the global index of the node
    virtual unsigned int GetIndex() { return g_index; }

    double m_TotalMass;  ///< Nodal mass obtained from element masss matrix

  protected:
    unsigned int g_index;  ///< global node index

  private:
    /// Initial setup.
    virtual void SetupInitial(ChSystem* system) {}

    friend class ChMesh;
};

/// @} fea_nodes

}  // end namespace fea
}  // end namespace chrono

#endif
