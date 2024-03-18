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

#ifndef CHCONTROLLER_H
#define CHCONTROLLER_H

#include <cmath>
#include "chrono/core/ChApiCE.h"
#include "chrono/core/ChFrame.h"
#include "chrono/physics/ChObject.h"

namespace chrono {

/// Simple PID controller
///
/// out = P*in + D*d(in)/dt + I*int(in*dt)
class ChApi ChControllerPID : public ChObj {
  private:
    double m_input;      ///< internal, last input set into controller, at time 'last_t'
    double m_input_integral;  ///< internal, last integral, for integrative part of controller
    double m_input_dt;   ///< internal, last derivative, for derivative part of controller
    double m_last_t;  ///< internal, last time
    double m_output;     ///< internal, last output value

  public:
    ChControllerPID();
    ChControllerPID(const ChControllerPID& other);
    ~ChControllerPID() {}

    /// "Virtual" copy constructor (covariant return type).
    virtual ChControllerPID* Clone() const override { return new ChControllerPID(*this); }

    double P;  ///< proportional coefficient
    double I;  ///< integrative coefficient
    double D;  ///< derivative coefficient

    /// Returns the output of the controller.
    /// Calls to GetOutput must be done in (possibly uniform) time steps,
    /// otherwise remember to call Reset() before other sequences of calls.
    double GetOutput(double input, double time);

    /// Same, but just returns last computed output
    double GetOutput() const { return m_output; }

    /// Use Reset to set accumulator to zero, at beginning. For integrative part.
    void Reset();
};

}  // end namespace chrono

#endif
