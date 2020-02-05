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
#include "chrono/core/ChMath.h"

namespace chrono {

/// <pre>
/// Class for a timer which attempts to enforce soft real-time.
/// Two different mechanisms are provided:
///    1) Spin(), spins in place waiting for real time to catch up with the simulation time.
///    2) SuggestSimulationStep(): returns a suggested value for the integration step size based on the wall clock time
///    required for completing the previous iteration of the simulation loop.
/// The first mechanism is the standard way of enforcing soft real time. The mechanism implemented through
/// SuggestSimulationStep has the drawback of changing the integration step size in an uncontrollable manner thus
/// resulting in slightly different results on different platforms or at different runs. It is kept for backward
/// compatibility.
/// </pre>
class ChRealtimeStepTimer : public ChTimer<double> {
  public:
    /// Create the timer (outside the simulation loop, preferably just before beginning the loop)
    ChRealtimeStepTimer() { this->start(); }

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

    /// Call this function INSIDE the simulation loop, just ONCE per loop, to get the suggested time for the next
    /// integration time step. If the previous simulation step was completed fast enough, this function will suggest a
    /// corresponding small value for advancing the simulated time; vice-versa, it will return higher values (up to a
    /// maximum 'max_step' limit) if the simulation goes slow because of high CPU overhead. If the clamping value of
    /// 'max_step' is not reached, the real-time and simulated time should always match. There is also an optional
    /// 'min_step' value, to avoid too small of an integration step.
    double SuggestSimulationStep(double max_step = 0.02,                                    ///< upper limit for step
                                 double min_step = std::numeric_limits<double>::epsilon())  ///< lower limit for step
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
