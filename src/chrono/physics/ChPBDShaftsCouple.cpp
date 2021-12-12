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
    if (abs(C) > 0) {
        double alpha_hat = alpha / pow(PBDsys->Get_h(), 2);
        double w1 = 1 / shaft1->GetInertia();
        double w2 = 1 / shaft2->GetInertia();

        double delta_lambda = -(C + alpha_hat * lambda) / (w1 + w2 + alpha_hat);
        lambda += delta_lambda;

        shaft1->SetPos(shaft1->GetPos() + delta_lambda * w1);
        shaft2->SetPos(shaft2->GetPos() - delta_lambda * w2);
    }
}

ChPBDShaftsCoupleClutch::ChPBDShaftsCoupleClutch(ChSystemPBD* sys, ChShaftsClutch* clutch)
    : clutchptr(clutch), ChPBDShaftsCouple(sys, clutch->GetShaft1(), clutch->GetShaft2()) {}

void ChPBDShaftsCoupleClutch::SolveShaftCoupling() {
    double h = PBDsys->Get_h();
    double C = (shaft1->GetPos_dt() - shaft2->GetPos_dt()) * h;
    if (abs(C) > 1E-5) {
        double h_sqrd = pow(h, 2);
        double modulation = clutchptr->GetModulation();
        double minT = clutchptr->GetTorqueLimitB();
        double maxT = clutchptr->GetTorqueLimitF();
        double alpha_hat = alpha / h_sqrd;
        double w1 = 1 / shaft1->GetInertia();
        double w2 = 1 / shaft2->GetInertia();

        double delta_lambda = -(C + alpha_hat * lambda) / (w1 + w2 + alpha_hat);
        delta_lambda = ChClamp(delta_lambda, h_sqrd * minT * modulation, h_sqrd * maxT * modulation);
        lambda += delta_lambda;

        shaft1->SetPos(shaft1->GetPos() + delta_lambda * w1);
        shaft2->SetPos(shaft2->GetPos() - delta_lambda * w2);
    }
}

ChPBDShaftsCouplePlanetary::ChPBDShaftsCouplePlanetary(ChSystemPBD* sys, ChShaftsPlanetary* planetary)
    : planetaryptr(planetary),
      ChPBDShaftsCouple(sys, planetary->GetShaft1(), planetary->GetShaft2()),
      shaft3(planetary->GetShaft3()) {}

void ChPBDShaftsCouplePlanetary::SolveShaftCoupling() {
    // double C = shaftGear->GetTransmissionRatio() * (shaft1->GetPos()) - (this->shaft2->GetPos());
    double C = planetaryptr->GetTransmissionR1() * (shaft1->GetPos()) +
               planetaryptr->GetTransmissionR2() * (shaft2->GetPos()) +
               planetaryptr->GetTransmissionR3() * (shaft3->GetPos());
    if (abs(C) > 0) {
        double alpha_hat = alpha / pow(PBDsys->Get_h(), 2);
        double w1 = 1 / shaft1->GetInertia();
        double w2 = 1 / shaft2->GetInertia();
        double w3 = 1 / shaft3->GetInertia();

        double delta_lambda = -(C + alpha_hat * lambda) / (w1 + w2 + w3 + alpha_hat);
        lambda += delta_lambda;

        shaft1->SetPos(shaft1->GetPos() + delta_lambda * w1);
        shaft2->SetPos(shaft2->GetPos() - delta_lambda * w2);
        shaft3->SetPos(shaft3->GetPos() - delta_lambda * w3);
    }
}

void PopulateShaftCouplingPBD(std::vector<std::shared_ptr<ChPBDShaftsCouple>>& listPBD,
                              const std::vector<std::shared_ptr<ChPhysicsItem>>& otherlist,
                              ChSystemPBD* sys) {
    for (auto& physobj : otherlist) {
        if (dynamic_cast<const ChShaftsGear*>(physobj.get()) != nullptr) {
            ChShaftsGear* gear = dynamic_cast<ChShaftsGear*>(physobj.get());
            auto gearPBD = chrono_types::make_shared<ChPBDShaftsCoupleGear>(sys, gear);
            listPBD.push_back(gearPBD);
        }
        if (dynamic_cast<const ChShaftsClutch*>(physobj.get()) != nullptr) {
            ChShaftsClutch* clutch = dynamic_cast<ChShaftsClutch*>(physobj.get());
            auto clutchPBD = chrono_types::make_shared<ChPBDShaftsCoupleClutch>(sys, clutch);
            listPBD.push_back(clutchPBD);
        }
        if (dynamic_cast<const ChShaftsPlanetary*>(physobj.get()) != nullptr) {
            ChShaftsPlanetary* planetary = dynamic_cast<ChShaftsPlanetary*>(physobj.get());
            auto planetaryPBD = chrono_types::make_shared<ChPBDShaftsCouplePlanetary>(sys, planetary);
            listPBD.push_back(planetaryPBD);
        }
    }
}

}  // namespace chrono
