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

#include "chrono/utils/ChUtils.h"
#include "chrono/physics/ChElectricActuator.h"

namespace chrono {

ChElectricActuator::ChElectricActuator() {}

void ChElectricActuator::SetInitialConditions(ChVectorDynamic<>& y0) {
    //// TODO
}

void ChElectricActuator::CalculateRHS(double time,                 // current time
                                      const ChVectorDynamic<>& y,  // current ODE states
                                      ChVectorDynamic<>& rhs       // output ODE right-hand side vector
) {
    //// TODO
}

bool ChElectricActuator::CalculateJac(double time,                   // current time
                                      const ChVectorDynamic<>& y,    // current ODE states
                                      const ChVectorDynamic<>& rhs,  // current ODE right-hand side vector
                                      ChMatrixDynamic<>& J           // output Jacobian matrix
) {
    // Do not provide Jacobian information if problem not stiff
    if (!IsStiff())
        return false;

    //// TODO: analytical Jacobian
    return false;
}

void ChElectricActuator::ComputeCouplingKRM(ChMatrixRef H, double Kfactor, double Rfactor, double Mfactor) {
    //// TODO
}

}  // end namespace chrono
