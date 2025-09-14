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
// Authors: Alessandro Tasora
// =============================================================================

#include <cstdlib>
#include <algorithm>

#include "chrono_peridynamics/ChPeridynamics.h"
#include "chrono_peridynamics/ChMatterPeriLiquid.h"

namespace chrono {

using namespace fea;
using namespace peridynamics;

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChMatterPeriLiquid)

void ChMatterPeriLiquid::ComputeForces() {
    // 1- Per-node initialization
    for (auto& node : this->nodes) {
        node.second.density = 0;
        node.second.pressure = 0;
        node.first->is_fluid = true;  // this forces continuous collision proximity search
    }

    // 2- Per-edge initialization and accumulation of particles's density
    for (auto& bond : this->bonds) {
        ChMatterDataPerBond& mbond = bond.second;
        ChMatterDataLiquid& nodeA_data = nodes[mbond.nodeA];
        ChMatterDataLiquid& nodeB_data = nodes[mbond.nodeB];

        ChVector3d x_A = mbond.nodeA->GetPos();
        ChVector3d x_B = mbond.nodeB->GetPos();

        ChVector3d r_BA = x_B - x_A;
        double dist_BA = r_BA.Length();

        double W_k_poly6 = W_poly6(dist_BA, mbond.nodeA->GetHorizonRadius());

        nodeA_data.density += mbond.nodeA->GetMass() * W_k_poly6;
        nodeB_data.density += mbond.nodeB->GetMass() * W_k_poly6;
    }

    // 3- Per-node volume and pressure computation

    for (auto& node : this->nodes) {
        // node volume is v=mass/density
        if (node.second.density)
            node.first->volume = node.first->GetMass() / node.second.density;
        else
            node.first->volume = 0;

        // node pressure = k(dens - dens_0);
        node.second.pressure = pressure_stiffness * (node.second.density - this->density);
    }

    // 4- Per-edge forces computation and accumulation
    for (auto& bond : this->bonds) {
        ChMatterDataPerBond& mbond = bond.second;
        ChMatterDataLiquid& nodeA_data = nodes[mbond.nodeA];
        ChMatterDataLiquid& nodeB_data = nodes[mbond.nodeB];

        ChVector3d x_A = mbond.nodeA->GetPos();
        ChVector3d x_B = mbond.nodeB->GetPos();

        ChVector3d r_BA = x_B - x_A;
        double dist_BA = r_BA.Length();

        // increment pressure forces
        ChVector3d W_k_press;
        W_gr_press(W_k_press, r_BA, dist_BA, mbond.nodeA->GetHorizonRadius());

        double avg_press = 0.5 * (nodeA_data.pressure + nodeB_data.pressure);

        ChVector3d pressureForceA = W_k_press * mbond.nodeA->volume * avg_press * mbond.nodeB->volume;
        mbond.nodeA->F_peridyn += pressureForceA / mbond.nodeA->volume;
        mbond.nodeB->F_peridyn -= pressureForceA / mbond.nodeB->volume;

        // increment viscous forces..
        double W_k_visc = W_sq_visco(dist_BA, mbond.nodeA->GetHorizonRadius());
        ChVector3d velBA = mbond.nodeA->GetPosDt() - mbond.nodeB->GetPosDt();

        ChVector3d viscforceBA = velBA * (mbond.nodeA->volume * this->viscosity * mbond.nodeA->volume * W_k_visc);
        mbond.nodeA->F_peridyn += viscforceBA / mbond.nodeA->volume;
        mbond.nodeB->F_peridyn -= viscforceBA / mbond.nodeB->volume;
    }
}

}  // end namespace chrono
