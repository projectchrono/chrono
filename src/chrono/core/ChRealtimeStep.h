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
// Authors: Alessandro Tasora, Radu Serban
// =============================================================================

#ifndef CHREALTIMESTEP_H
#define CHREALTIMESTEP_H

#include <limits>

#include "chrono/core/ChTimer.h"

namespace chrono {

/// Class for a timer which attempts to enforce soft real-time.
class ChRealtimeStepTimer : public ChTimer {
  public:
    /// Create the timer (outside the simulation loop, preferably just before beginning the loop)
    ChRealtimeStepTimer() { start(); }

    /// Call this function INSIDE the simulation loop, just ONCE per loop (preferably as the last call in the loop),
    /// passing it the integration step size used at this step. If the time elapsed over the last step (i.e., from
    /// the last call to Spin) is small than the integration step size, this function will spin in place until real time
    /// catches up with the simulation time, thus providing soft real-time capabilities.
    void Spin(double step) {
        while (GetTimeSecondsIntermediate() < step) {
        }
        reset();
        start();
    }
};

}  // end namespace chrono

#endif
