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
// Authors: Simone Benatti
// =============================================================================
//
// Structures for links, contacts, body properties in PBD systems and their lists
//
// =============================================================================

#include <algorithm>

#include "chrono/physics/ChPBDShaftsCouple.h"
#include "chrono/physics/ChSystemPBD.h"
#include <Eigen/Core>


namespace chrono {

	ChPBDShaftsCoupleGear::ChPBDShaftsCoupleGear(ChSystemPBD* sys, ChShaftsGear* gear)
    : shaftGear(gear), ChPBDShaftsCouple(sys, gear->GetShaft1(), gear->GetShaft2()) {}
	void ChPBDShaftsCoupleGear::SolveShaftCoupling() {
	
	    double C = shaftGear->GetTransmissionRatio() * (shaft1->GetPos()) - (this->shaft2->GetPos());
        if (abs(C)>0) {
            double alpha_hat = alpha / pow(PBDsys->Get_h(), 2);
            double w1 = 1 / shaft1->GetInertia();
            double w2 = 1 / shaft2->GetInertia();
            
            double delta_lambda = -(C + alpha_hat * lambda) / (w1 + w2 + alpha_hat);
            lambda += delta_lambda;

            shaft1->SetPos(shaft1->GetPos() + delta_lambda * w1);
            shaft1->SetPos(shaft1->GetPos() - delta_lambda * w2);

        }
	}


}

