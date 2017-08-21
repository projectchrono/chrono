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
// Authors: Alessandro Tasora
// =============================================================================

#ifndef CHREALTIMESTEP_H
#define CHREALTIMESTEP_H

#include "chrono/core/ChTimer.h"
#include "chrono/core/ChMath.h"

namespace chrono {

/// Class for a timer which measure the time spent
/// in VR or game-like simulation loops, and suggests
/// a dt integration step for the physical simulation
/// for the next step. Uses high-resolution timer.
/// Note that the suggested integrations step tries
/// to keep a true real-time pace in advancing the
/// simulation, that is the simulated time should
/// match the real time. The suggested step may
/// be non-constant because the overhead of the
/// simulation loop may vary because of varying
/// overhead in visualization, collision or simulation.

class ChRealtimeStepTimer : public ChTimer<double> {
  public:
    /// Create the timer (outside the simulation loop, preferably
    /// just before beginning the while{} loop)
    ChRealtimeStepTimer() { this->start(); }

    /// Call this function INSIDE the simulation loop, just ONCE
    /// per loop, to get the suggested time for the next integration
    /// time step. If the ChRealtimeStepTimer measured that
    /// previous simulation step required few real-time, it will
    /// suggest a corresponding small value for advancing the simulated
    /// time, and viceversa will give higher values (up to a maximum
    /// 'max_step' limit, however) if the simulation goes slow because
    /// of high CPU overhead. If the clamping value of 'max_step' is
    /// not reached, the real-time and simulated time should always match.
    /// There is also an optional 'min_step' value, which avoids too small
    /// integration steps.

    double SuggestSimulationStep(double max_step = 0.02,        ///< upper limit for step
                                 double min_step = CH_NANOTOL)  ///< lower limit for step
    {
        this->stop();
        double mstep = this->GetTimeSeconds();
        this->reset();
        this->start();

        if (mstep < min_step)
            return min_step;
        if (mstep > max_step)
            return max_step;
        return mstep;
    }
};

}  // end namespace chrono

#endif
