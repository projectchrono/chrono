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
#include "chrono_peridynamics/ChMatterPeriSprings.h"

namespace chrono {

using namespace fea;
using namespace peridynamics;

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChMatterPeriSprings)

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChMatterPeriSpringsBreakable)

void ChMatterPeriSprings::ComputeForces() {
    // loop on bonds
    for (auto& bond : this->bonds) {
        ChMatterDataPerBond& mbond = bond.second;
        ChVector3d old_vdist = mbond.nodeB->GetX0() - mbond.nodeA->GetX0();
        ChVector3d vdist = mbond.nodeB->GetPos() - mbond.nodeA->GetPos();
        ChVector3d vdir = vdist.GetNormalized();
        double vel = Vdot(vdir, mbond.nodeB->GetPosDt() - mbond.nodeA->GetPosDt());
        ChVector3d force_val = (vdist.Length() - old_vdist.Length()) * this->k + vel * this->r;
        mbond.nodeB->F_peridyn += -vdir * force_val;
        mbond.nodeA->F_peridyn += vdir * force_val;
    }
}

void ChMatterPeriSpringsBreakable::ComputeForces() {
    // loop on bonds
    for (auto& bond : this->bonds) {
        ChMatterDataPerBondBreakable& mbond = bond.second;
        if (!mbond.broken) {
            ChVector3d old_vdist = mbond.nodeB->GetX0() - mbond.nodeA->GetX0();
            ChVector3d vdist = mbond.nodeB->GetPos() - mbond.nodeA->GetPos();
            ChVector3d vdir = vdist.GetNormalized();
            double vel = Vdot(vdir, mbond.nodeB->GetPosDt() - mbond.nodeA->GetPosDt());
            ChVector3d force_val = (vdist.Length() - old_vdist.Length()) * this->k + vel * this->r;
            mbond.nodeB->F_peridyn +=
                -vdir * force_val / mbond.nodeB->volume;  // divide by volumes because F_peridyn are force _densities_
            mbond.nodeA->F_peridyn += vdir * force_val / mbond.nodeA->volume;

            double stretch = (vdist.Length() - old_vdist.Length()) / old_vdist.Length();
            if (stretch > max_stretch) {
                mbond.nodeA->F_peridyn = 0;
                mbond.nodeB->F_peridyn = 0;
                mbond.broken = true;
                // the following will propagate the fracture geometry so that broken parts can collide
                mbond.nodeA->is_boundary = true;
                mbond.nodeB->is_boundary = true;
            }
        } else {
            if (mbond.broken)
                bonds.erase(bond.first);
        }
    }
}

}  // end namespace chrono
