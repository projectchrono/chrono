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

    // Tag needed for class factory in archive (de)serialization:
    CH_FACTORY_TAG(ChControls)

  public:
    ChControls() {}
    ChControls(const ChControls& other) : ChObj(other) {}
    virtual ~ChControls() {}

    /// "Virtual" copy constructor.
    virtual ChControls* Clone() const override { return new ChControls(*this); }

    virtual bool ExecuteForUpdate() { return true; }
    virtual bool ExecuteForStep() { return true; }

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOUT(ChArchiveOut& marchive) override;

    /// Method to allow de serialization of transient data from archives.
    virtual void ArchiveIN(ChArchiveIn& marchive) override;
};

CH_CLASS_VERSION(ChControls,0)

}  // end namespace chrono

#endif
