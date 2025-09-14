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

#include <cstdlib>
#include <algorithm>

#include "chrono_peridynamics/ChPeridynamics.h"
#include "chrono_peridynamics/ChMatterPeriLinearElastic.h"

namespace chrono {

using namespace fea;
using namespace peridynamics;

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChMatterPeriLinearElastic)

void ChMatterPeriLinearElastic::SetYoungModulusShearModulus(double mE, double mG) {
    k_bulk = (mE * mG / (3. * (3. * mG - mE)));
    G = mG;
}

void ChMatterPeriLinearElastic::SetYoungModulusPoisson(double mE, double mu) {
    k_bulk = (mE / (3. * (1. - 2. * mu)));
    G = (mE / (2. * (1. + mu)));
}

double ChMatterPeriLinearElastic::InfluenceFunction(double zeta, double horizon) {
    if (zeta > horizon)
        return 0.0;

    // inv.linear decrease
    return horizon / zeta;
}

void ChMatterPeriLinearElastic::SetupInitial() {
    /*
    // loop on nodes
    for (auto& node : this->nodes) {
        ChMatterDataPerNodeLinearElastic& mnodedata = node.second;
        mnodedata.m = 0;
        mnodedata.theta = 0;
    }
    // loop on bonds
    for (auto& bond : this->bonds) {
        ChMatterDataPerBondLinearElastic& mbond = bond.second;
        ChVector3d old_vdist = mbond.nodeB->GetX0() - mbond.nodeA->GetX0();
        double     old_sdist = old_vdist.Length();
        double horizon = mbond.nodeA->GetHorizonRadius();
        double omega = this->InfluenceFunction(old_sdist, horizon);
        ChMatterDataPerNodeLinearElastic& mnodedataA = this->nodes[mbond.nodeA];
        ChMatterDataPerNodeLinearElastic& mnodedataB = this->nodes[mbond.nodeB];
        mnodedataA.m += omega * old_sdist * old_sdist * mbond.nodeB->volume;
        mnodedataB.m += omega * old_sdist * old_sdist * mbond.nodeA->volume;
    }
    */
}

void ChMatterPeriLinearElastic::ComputeForces() {
    // loop on nodes for resetting dilation
    for (auto& node : this->nodes) {
        ChMatterDataPerNodeLinearElastic& mnodedata = node.second;
        mnodedata.m = 0;
        mnodedata.theta = 0;
    }
    // loop on bonds for weighted mass
    for (auto& bond : this->bonds) {
        ChMatterDataPerBondLinearElastic& mbond = bond.second;
        if (!mbond.broken) {
            ChVector3d old_vdist = mbond.nodeB->GetX0() - mbond.nodeA->GetX0();
            double old_sdist = old_vdist.Length();

            double horizon = mbond.nodeA->GetHorizonRadius();
            double omega = this->InfluenceFunction(old_sdist, horizon);

            ChMatterDataPerNodeLinearElastic& mnodedataA = this->nodes[mbond.nodeA];
            ChMatterDataPerNodeLinearElastic& mnodedataB = this->nodes[mbond.nodeB];

            mnodedataA.m += omega * (old_sdist * old_sdist) * mbond.nodeB->volume;
            mnodedataB.m += omega * (old_sdist * old_sdist) * mbond.nodeA->volume;
        }
    }

    // loop on bonds for dilation
    for (auto& bond : this->bonds) {
        ChMatterDataPerBondLinearElastic& mbond = bond.second;
        if (!mbond.broken) {
            ChVector3d old_vdist = mbond.nodeB->GetX0() - mbond.nodeA->GetX0();
            ChVector3d vdist = mbond.nodeB->GetPos() - mbond.nodeA->GetPos();
            double old_sdist = old_vdist.Length();
            double sdist = vdist.Length();

            double horizon = mbond.nodeA->GetHorizonRadius();
            double omega = this->InfluenceFunction(old_sdist, horizon);

            ChMatterDataPerNodeLinearElastic& mnodedataA = this->nodes[mbond.nodeA];
            ChMatterDataPerNodeLinearElastic& mnodedataB = this->nodes[mbond.nodeB];

            mnodedataA.theta += (3. / mnodedataA.m) * omega * old_sdist * (sdist - old_sdist) * mbond.nodeB->volume;
            mnodedataB.theta += (3. / mnodedataB.m) * omega * old_sdist * (sdist - old_sdist) * mbond.nodeA->volume;
        } else {
            if (mbond.broken)
                bonds.erase(bond.first);
        }
    }

    // loop on bonds for force computation
    for (auto& bond : this->bonds) {
        ChMatterDataPerBondLinearElastic& mbond = bond.second;
        ChVector3d old_vdist = mbond.nodeB->GetX0() - mbond.nodeA->GetX0();
        ChVector3d vdist = mbond.nodeB->GetPos() - mbond.nodeA->GetPos();
        double old_sdist = old_vdist.Length();
        double sdist = vdist.Length();
        ChVector3d vdir = vdist / sdist;
        ChVector3d old_vdir = old_vdist / old_sdist;
        // double         svel = Vdot(vdir, mbond.nodeB->GetPosDt() - mbond.nodeA->GetPosDt());

        double e = sdist - old_sdist;

        double horizon = mbond.nodeA->GetHorizonRadius();
        double omega = this->InfluenceFunction(old_sdist, horizon);

        ChMatterDataPerNodeLinearElastic& mnodedataA = this->nodes[mbond.nodeA];
        ChMatterDataPerNodeLinearElastic& mnodedataB = this->nodes[mbond.nodeB];

        // Silling:
        double e_i_A = mnodedataA.theta * old_sdist / 3.0;  // isotropic
        double e_i_B = mnodedataB.theta * old_sdist / 3.0;  // isotropic
        double e_d_A = e - e_i_A;                           // deviatoric
        double e_d_B = e - e_i_B;                           // deviatoric
        double t_A = 3. * this->k_bulk * mnodedataA.theta * omega * old_sdist / mnodedataA.m +
                     15. * this->G / mnodedataA.m * omega * e_d_A;
        double t_B = 3. * this->k_bulk * mnodedataB.theta * omega * old_sdist / mnodedataB.m +
                     15. * this->G / mnodedataB.m * omega * e_d_B;

        //***TODO***  optimize and simplify computations - many are redundant

        //***TODO***  implement damping
        /*
        if (this->r_bulk > 0) {
            double pih4 = chrono::CH_PI * horizon * horizon * horizon * horizon;
            double viscforce = 0.5 * (18.0 * r_bulk / pih4) * svel;
            t_A += viscforce;
            t_B += viscforce;
        }
        */

        mbond.nodeB->F_peridyn += -vdir * 0.5 * (t_B + t_A) * mbond.nodeA->volume * mbond.nodeB->volume;
        mbond.nodeA->F_peridyn += vdir * 0.5 * (t_B + t_A) * mbond.nodeB->volume * mbond.nodeA->volume;
    }
}

}  // end namespace chrono
