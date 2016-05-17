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

#ifndef CHPROBE_H
#define CHPROBE_H

#include <cmath>

#include "chrono/physics/ChObject.h"

namespace chrono {

/// Base class for probe objects, used to record data during simulations.

class ChApi ChProbe : public ChObj {
  public:
    ChProbe();
    ChProbe(const ChProbe& other);
    virtual ~ChProbe() {}

    void Copy(ChProbe* source);

    /// Record the value.
    /// Note that X and Y variables must be already set, using the probe data.
    /// Usually mtime is used for X var (default), while a script is used to
    /// specify the Y  variable. Record() is called for each integration step,
    /// by the system object which contains all probes.
    virtual void Record(double mtime) {}

    // If some data is recorded, delete.
    virtual void Reset() {}
};

}  // end namespace chrono

#endif
