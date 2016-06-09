// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All right reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Alessandro Tasora, Radu Serban
// =============================================================================

#ifndef CHCONTROLS_H
#define CHCONTROLS_H

#include <cmath>

#include "chrono/physics/ChObject.h"

namespace chrono {

/// Basic interface class for 'controls', that are objects that change parameters
/// during the simulation, ex. to simulate PIDs, etc.
/// Must be inherited and implemented by user.

class ChApi ChControls : public ChObj {
    // Chrono simulation of RTTI, needed for serialization
    CH_RTTI(ChControls, ChObj);

  public:
    ChControls() {}
    ChControls(const ChControls& other) : ChObj(other) {}
    virtual ~ChControls() {}

    virtual bool ExecuteForStart() = 0;
    virtual bool ExecuteForUpdate() = 0;
    virtual bool ExecuteForStep() = 0;
    virtual bool ExecuteFor3DStep() = 0;

    /// Method to allow serialization of transient data in archives.
    virtual void ArchiveOUT(ChArchiveOut& marchive) override;

    /// Method to allow de serialization of transient data from archives.
    virtual void ArchiveIN(ChArchiveIn& marchive) override;
};

}  // end namespace chrono

#endif
