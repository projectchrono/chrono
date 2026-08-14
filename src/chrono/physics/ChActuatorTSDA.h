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
// Linear actuator wrapping .
//
// =============================================================================

#ifndef CH_ACTUATOR_TSDA_H
#define CH_ACTUATOR_TSDA_H

#include "chrono/physics/ChActuator.h"

namespace chrono {

/// TSDA linear actuator.
/// The actuator force includes elastic and damping components, a constant actuation force, and a component modulated by the input function:
/// F = f0 + f * u(time) + k * (l = l0) + c * ld
class ChActuatorTSDA : public ChLinearActuator {
  public:
    ChActuatorTSDA() : m_k(0), m_r(0), m_f(0), m_rest_length(0) {}
    ~ChActuatorTSDA() {}

    /// Set spring coefficient (default: 0).
    void SetSpringCoefficient(double k) { m_k = k; }

    /// Set damping coefficient (default: 0).
    void SetDampingCoefficient(double r) { m_r = r; }

    /// Set constant force component (default: 0).
    void SetConstantForce(double f) { m_f0 = f; }

    /// Set base actuation force (default: 0).
    /// The TSDA actuation force is modulated by the actuator time-dependent input function.
    /// The value specified here corresponds to the maximum input value of 1.
    void SetBaseForce(double f) { m_f = f; }

    /// Set spring rest (free) length.
    void SetRestLength(double len) { m_rest_length = len; }

    /// Get the actuator force at the specified time.
    virtual double GetActuatorForce(double time) override {
        double U = GetInput(time);
        auto force = m_f0 + m_f * U - m_k * (s - m_rest_length) - m_r * sd;
        return force;
    }

  private:
    double m_k;            ///< spring coefficient
    double m_r;            ///< damping coefficient
    double m_f;            ///< base actuation force
    double m_f0;           ///< constant actuation force
    double m_rest_length;  ///< undeformed length
};

}  // end namespace chrono

#endif
