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

#ifndef CH_CONTROLLERS_H
#define CH_CONTROLLERS_H

#include "chrono/core/ChApiCE.h"
#include "chrono/physics/ChObject.h"

namespace chrono {
namespace utils {

/// @addtogroup chrono_utils
/// @{

/// Simple PID controller.
/// Produces: out = P*in + D*d(in)/dt + I*int(in*dt)
class ChApi ChControllerPID : public ChObj {
  public:
    ChControllerPID();
    ChControllerPID(double P, double I, double D);
    ChControllerPID(const ChControllerPID& other);
    ~ChControllerPID() {}

    /// "Virtual" copy constructor (covariant return type).
    virtual ChControllerPID* Clone() const override { return new ChControllerPID(*this); }

    /// Set the controller gains.
    void SetGains(double P, double I, double D);

    /// Return the output of the controller.
    /// Calls to GetOutput must be done in (ideally) uniform time steps.
    double GetOutput(double input, double time);

    /// Return last computed output.
    double GetOutput() const { return m_output; }

    /// Use Reset to set accumulator to zero, at beginning. For integrative part.
    void Reset();

  private:
    double m_P;  ///< proportional coefficient
    double m_I;  ///< integrative coefficient
    double m_D;  ///< derivative coefficient

    double m_input;           ///< last input set into controller, at time 'last_t'
    double m_input_integral;  ///< last integral, for integrative part of controller
    double m_input_dt;        ///< last derivative, for derivative part of controller
    double m_last_t;          ///< last time
    double m_output;          ///< last output value
};

/// @} chrono_utils

}  // end namespace utils
}  // end namespace chrono

#endif
