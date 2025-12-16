// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2025 projectchrono.org
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
// Electric actuator model based on the paper:
// "Dynamic simulation of constrained multibody systems withelectromechanical
// actuators using a monolithic approach,"
// A.G. Agúndez, A. Mikkola, D. García-Vallejo, R. Serban, A. Mattsson, P. Peltoniemi,
// Multibody System Dynamics, DOI: 10.1007/s11044-025-10130-9, 2025
// =============================================================================

#ifndef CH_ELECTRIC_ACTUATOR_H
#define CH_ELECTRIC_ACTUATOR_H

#include "chrono/physics/ChActuator.h"

namespace chrono {

/// Base class for an electric actuator.
class ChApi ChElectricActuator : public ChActuator {
  public:
    ChElectricActuator();
    ~ChElectricActuator() {}

  private:
    /// Declare the EOM of this physics item as stiff or non-stiff.
    virtual bool IsStiff() const override { return true; }

    // Interface to ChExternalDynamicsODE

    virtual unsigned int GetNumStates() const override { return 5; }

    virtual void SetInitialConditions(ChVectorDynamic<>& y0) override;

    virtual void CalculateRHS(double time,                 ///< current time
                              const ChVectorDynamic<>& y,  ///< current ODE states
                              ChVectorDynamic<>& rhs       ///< output ODE right-hand side vector
                              ) override;

    virtual bool CalculateJac(double time,                   ///< current time
                              const ChVectorDynamic<>& y,    ///< current ODE states
                              const ChVectorDynamic<>& rhs,  ///< current ODE right-hand side vector
                              ChMatrixDynamic<>& J           ///< output Jacobian matrix
                              ) override;

    // Coupling term

    virtual bool EnableCouplingKRM() override { return false; }

    virtual void ComputeCouplingKRM(ChMatrixRef H, double Kfactor, double Rfactor, double Mfactor) override;
};

}  // end namespace chrono

#endif
