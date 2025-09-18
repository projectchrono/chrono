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
#include "chrono_peridynamics/ChMatterPeriBB.h"

namespace chrono {

using namespace fea;
using namespace peridynamics;

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChMatterPeriBB)

void ChMatterPeriBB::SetYoungModulus(double mE) {
    // bulk stiffness  K=E/(3(1-2mu)) , with mu=1/4 -> K=E*(2/3)
    k_bulk = (mE * (2. / 3.));
}

double ChMatterPeriBB::VolumeCorrection(double dist, double horizon, double vol_size) {
    if (dist < (horizon - vol_size))
        return 1.0;
    else if (dist < horizon)
        return 1.0 - (dist - (horizon - vol_size)) / (vol_size);
    else
        return 0.0;
}

void ChMatterPeriBB::ComputeForces() {
    // loop on nodes for resetting volume accumulator   //// TODO should be faster in SetupInitial - but see
    // notes below
    for (auto& node : this->nodes) {
        node.first->vol_accumulator = node.first->volume;
    }
    // loop on bonds for sum of connected volumes to nodes   //// TODO should be faster in SetupInitial - but see
    // notes below
    for (auto& bond : this->bonds) {
        ChMatterDataPerBondBB& mbond = bond.second;
        if (!mbond.broken) {
            mbond.nodeA->vol_accumulator += mbond.nodeB->volume;  // * VolumeCorrection(old_sdist, horizon, vol_size);
                                                                  // vol corr. not relevant in Ganzenmueller?
            mbond.nodeB->vol_accumulator += mbond.nodeA->volume;  // * VolumeCorrection(old_sdist, horizon, vol_size);
                                                                  // vol corr. not relevant in Ganzenmueller?
        }
    }

    // loop on bonds
    for (auto& bond : this->bonds) {
        ChMatterDataPerBondBB& mbond = bond.second;
        if (!mbond.broken) {
            ChVector3d old_vdist = mbond.nodeB->GetX0() - mbond.nodeA->GetX0();
            ChVector3d vdist = mbond.nodeB->GetPos() - mbond.nodeA->GetPos();
            double old_sdist = old_vdist.Length();
            double sdist = vdist.Length();
            ChVector3d vdir = vdist.GetNormalized();
            double svel = Vdot(vdir, mbond.nodeB->GetPosDt() - mbond.nodeA->GetPosDt());

            double stretch = (sdist - old_sdist) / old_sdist;

            // Original PMB in Silling
            // double vol_size = mbond.nodeA->GetVolumeSize();
            // double horizon = mbond.nodeA->GetHorizonRadius();
            // double pih4 = chrono::CH_PI * horizon * horizon * horizon * horizon;
            // double force_density_val =  (18.0 * k_bulk / pih4) * stretch; // original in Silling

            // Modified PMB, see [Ganzenmueller et al. "Improvements to the Prototype Micro - Brittle Linear
            // Elasticity Model of Peridynamics"]
            double c_gmuller =
                0.5 * ((18.0 * k_bulk / mbond.nodeA->vol_accumulator) + (18.0 * k_bulk / mbond.nodeB->vol_accumulator));
            double force_density_val = (c_gmuller / old_sdist) * stretch;

            if (this->damping > 0)
                force_density_val += this->damping * (c_gmuller / (old_sdist * old_sdist)) * svel;

            mbond.F_density = force_density_val;
            double wVV_ji =
                mbond.nodeA->volume * mbond.nodeB->volume;  // *VolumeCorrection(old_sdist, horizon, vol_size); //
                                                            // vol corr. not relevant in Ganzenmueller?

            mbond.nodeB->F_peridyn += -vdir * force_density_val * wVV_ji;
            mbond.nodeA->F_peridyn += vdir * force_density_val * wVV_ji;

            if (stretch > max_stretch) {
                mbond.F_density = 0;
                mbond.broken = true;
                // the following will propagate the fracture geometry so that broken parts can collide
                mbond.nodeA->is_boundary = true;
                mbond.nodeB->is_boundary = true;
            }
        } else {
            bonds.erase(bond.first);
        }
    }
}

void ChMatterPeriBB::SetupInitial() {
    /*
    // Not here, still in Computeforces() at each step because it could be necessary to update in runtime
    // node->vol_accumulator if bonds are deleted/fractured/..
    // TODO: still can be put here, and in ComputeForces just subtract from accumulator as
    //    mbond.nodeA->vol_accumulator -= mbond.nodeB->volume;
    //    mbond.nodeB->vol_accumulator -= mbond.nodeA->volume;
    // when the bond is erased in  bonds.erase(bond.first);  ? To be tested!
    // see
    // Ganzenmueller et al. "Improvements to the Prototype Micro - Brittle Linear Elasticity Model of Peridynamics"
    // loop on nodes for resetting volume accumulator note! could be necessary to update in runtime
    // if bonds are deleted/fractured/..
    for (auto& node : this->nodes) {
        node.first->vol_accumulator = node.first->volume;
    }
    // loop on bonds for sum of connected volumes to nodes
    // note! could be necessary to update in runtime if bonds are deleted/fractured/..
    for (auto& bond : this->bonds) {
        ChMatterDataPerBondBB& mbond = bond.second;
        if (!mbond.broken) {
            mbond.nodeA->vol_accumulator += mbond.nodeB->volume;
            // * VolumeCorrection(old_sdist, horizon, vol_size); // vol corr. not relevant in Ganzenmueller?
            mbond.nodeB->vol_accumulator += mbond.nodeA->volume;
            // * VolumeCorrection(old_sdist, horizon, vol_size); // vol corr. not relevant in Ganzenmueller?
        }
    }
    */
}

}  // end namespace chrono
