// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2026 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban
// =============================================================================
//
// Rotational actuator wrapping an RSDA.
//
// =============================================================================

#ifndef CH_ACTUATOR_RSDA_H
#define CH_ACTUATOR_RSDA_H

#include "chrono/physics/ChActuator.h"
#include "chrono/physics/ChLinkRSDA.h"

namespace chrono {

/// RSDA rotational actuator.
/// The actuator force includes elastic and damping components, a constant actuation torque, and a component modulated by the input function:
/// T = t0 + t * u(time) + k * (a = a0) + c * ad
class ChActuatorRSDA : public ChRotationalActuator {
  public:
    ChActuatorRSDA() : m_k(0), m_r(0), m_t(0), m_rest_angle(0) {}
    ~ChActuatorRSDA() {}

    /// Set spring coefficient (default: 0).
    void SetSpringCoefficient(double k) { m_k = k; }

    /// Set damping coefficient (default: 0).
    void SetDampingCoefficient(double r) { m_r = r; }

    /// Set constant torque component (default: 0).
    void SetConstantTorque(double t) { m_t0 = t; }

    /// Set base actuation torque (default: 0).
    /// The RSDA actuation torque is modulated by the actuator time-dependent input function.
    /// The value specified here corresponds to the maximum input value of 1.
    void SetBaseTorque(double t) { m_t = t; }

    /// Set spring rest angle (in radians).
    void SetRestAngle(double rest_angle);

    /// Get the actuator torque at the specified time.
    virtual double GetActuatorTorque(double time) {
        double U = GetInput(time);
        auto torque = m_t * U - m_k * (a - m_rest_angle) - m_r * ad;
        return torque;
    }

  private:
    double m_k;           ///< spring coefficient
    double m_r;           ///< damping coefficient
    double m_t;           ///< base actuation torque
    double m_t0;          ///< constant actuation torque
    double m_rest_angle;  ///< undeformed length
};

}  // end namespace chrono

#endif
