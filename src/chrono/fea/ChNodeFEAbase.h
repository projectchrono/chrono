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
#include "chrono_fea/ChApiFEA.h"

namespace chrono {
namespace fea {

/// @addtogroup fea_nodes
/// @{

// Forward
class ChMesh;

/// Base class for a generic finite element node that can be stored in ChMesh containers.
/// Derived classes must implement specialized versions.
class ChApiFea ChNodeFEAbase : public virtual ChNodeBase {
  public:
    ChNodeFEAbase() {}

    /// Set the rest position as the actual position.
    virtual void Relax() = 0;

    /// Reset to no speed and acceleration.
    virtual void SetNoSpeedNoAcceleration() = 0;

    /// Set the 'fixed' state of the node.
    /// If true, its current field value is not changed by solver.
    virtual void SetFixed(bool mev) = 0;

    /// Get the 'fixed' state of the node.
    /// If true, its current field value is not changed by solver.
    virtual bool GetFixed() = 0;

    /// Sets the global index of the node
    virtual void SetIndex(unsigned int mindex) { g_index = mindex; }

    /// Gets the global index of the node
    virtual unsigned int GetIndex() { return g_index; }

    double m_TotalMass;  ///< Nodal mass obtained from element masss matrix

  protected:
    //
    // DATA
    //

    unsigned int g_index;  ///< global node index
};

/// @} fea_nodes

}  // end namespace fea
}  // end namespace chrono

#endif
