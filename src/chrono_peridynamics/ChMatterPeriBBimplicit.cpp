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
#include "chrono_peridynamics/ChMatterPeriBBimplicit.h"

namespace chrono {

using namespace fea;
using namespace peridynamics;

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChMatterPeriBBimplicit)

void ChMatterDataPerBondBBimplicit::Initialize(ChNodePeri* mA, ChNodePeri* mB) {
    ChMatterDataPerBond::Initialize(mA, mB);
    constraint.SetVariables(&mA->Variables(), &mB->Variables());
    constraint.SetBoxedMinMax(-1e30, 1e30);  // unlimited forces by default
}

void ChMatterPeriBBimplicit::SetYoungModulus(double mE) {
    // bulk stiffness  K=E/(3(1-2mu)) , with mu=1/4 -> K=E*(2/3)
    this->k_bulk = (mE * (2. / 3.));
}

double ChMatterPeriBBimplicit::VolumeCorrection(double dist, double horizon, double vol_size) {
    if (dist < (horizon - vol_size))
        return 1.0;
    else if (dist < horizon)
        return 1.0 - (dist - (horizon - vol_size)) / (vol_size);
    else
        return 0.0;
}

void ChMatterPeriBBimplicit::ComputeForces() {
    // see [Ganzenmueller et al. "Improvements to the Prototype Micro - Brittle Linear Elasticity Model of
    // Peridynamics"] loop on nodes for resetting volume accumulator   //***TODO*** maybe faster in SetupInitial
    for (auto& node : this->nodes) {
        node.first->vol_accumulator = node.first->volume;
    }
    // loop on bonds for sum of connected volumes to nodes   //***TODO*** maybe faster in SetupInitial
    for (auto& bond : this->bonds) {
        ChMatterDataPerBondBBimplicit& mbond = bond.second;
        if (mbond.state == ChMatterDataPerBondBBimplicit::bond_state::ACTIVE) {
            mbond.nodeA->vol_accumulator += mbond.nodeB->volume;
            mbond.nodeB->vol_accumulator += mbond.nodeA->volume;
        }
    }

    // loop on bonds
    for (auto& bond : this->bonds) {
        ChMatterDataPerBondBBimplicit& mbond = bond.second;
        if (mbond.state != ChMatterDataPerBondBBimplicit::bond_state::BROKEN) {
            ChVector3d old_vdist = mbond.nodeB->GetX0() - mbond.nodeA->GetX0();
            ChVector3d vdist = mbond.nodeB->GetPos() - mbond.nodeA->GetPos();
            double old_sdist = old_vdist.Length();
            double sdist = vdist.Length();
            ChVector3d vdir = vdist.GetNormalized();
            double svel = Vdot(vdir, mbond.nodeB->GetPosDt() - mbond.nodeA->GetPosDt());

            double stretch = (sdist - old_sdist) / old_sdist;

            // Original PMB in Silling
            // double horizon = mbond.nodeA->GetHorizonRadius();
            // double vol_size = mbond.nodeA->GetVolumeSize();
            // double pih4 = chrono::CH_PI * horizon * horizon * horizon * horizon;
            // double c_silling = (18.0 * k_bulk /  (chrono::CH_PI * horizon * horizon * horizon * horizon) )
            // double force_density_val =  c_silling * stretch; // original in Silling

            // Modified PMB in Ganzenmueller et.al.
            double c_gmuller =
                0.5 * ((18.0 * k_bulk / mbond.nodeA->vol_accumulator) + (18.0 * k_bulk / mbond.nodeB->vol_accumulator));
            double force_density_val = (c_gmuller / old_sdist) * stretch;  // in Ganzenmueller et.al.

            if (this->damping > 0)
                force_density_val += this->damping * (c_gmuller / (old_sdist * old_sdist)) * svel;

            // constraint elongation
            mbond.d_zeta = sdist - old_sdist;

            // tangent stiffness,   Km = c * 1/zeta * w_jk
            double wVV_jk =
                mbond.nodeA->volume * mbond.nodeB->volume;  // * VolumeCorrection(old_sdist, horizon, vol_size); //
                                                            // vol corr. not relevant in Ganzenmueller
            bond.second.Km = (c_gmuller / (old_sdist * old_sdist)) *
                             wVV_jk;  // that is..   Km= (c_gmuller / old_sdist) * (1.0/old_sdist) * wVV_jk;

            // compute here the jacobians, no need to move in LoadConstraintJacobians()
            mbond.constraint.Get_Cq_a()(0) = -vdir.x();  // / old_sdist;
            mbond.constraint.Get_Cq_a()(1) = -vdir.y();  // / old_sdist;
            mbond.constraint.Get_Cq_a()(2) = -vdir.z();  // / old_sdist;
            mbond.constraint.Get_Cq_b()(0) = vdir.x();   // / old_sdist;
            mbond.constraint.Get_Cq_b()(1) = vdir.y();   // / old_sdist;
            mbond.constraint.Get_Cq_b()(2) = vdir.z();   // / old_sdist;

            if (stretch > max_stretch_fracture) {
                mbond.force_density_val = 0;
                mbond.state = ChMatterDataPerBondBBimplicit::bond_state::FRACTURED;
                mbond.constraint.SetBoxedMinMax(0, 1e30);  // enable complementarity, reaction>0.
            }

            if (stretch > max_stretch_break) {
                mbond.force_density_val = 0;
                mbond.state = ChMatterDataPerBondBBimplicit::bond_state::BROKEN;
                // the following will propagate the fracture geometry so that broken parts can collide
                mbond.nodeA->is_boundary = true;
                mbond.nodeB->is_boundary = true;
            }
        }
    }
}

void ChMatterPeriBBimplicit::Setup() {
    // cleanup bonds that are broken
    for (auto& bond : this->bonds) {
        if (bond.second.state == ChMatterDataPerBondBBimplicit::bond_state::BROKEN)
            bonds.erase(bond.first);
    }
}

void ChMatterPeriBBimplicit::IntLoadResidual_CqL(const unsigned int off_L,
                                                 ChVectorDynamic<>& R,
                                                 const ChVectorDynamic<>& L,
                                                 const double c) {
    int boff = 0;
    for (auto& bond : this->bonds) {
        bond.second.constraint.AddJacobianTransposedTimesScalarInto(R, L(off_L + boff) * c);
        ++boff;
    }
}

void ChMatterPeriBBimplicit::IntLoadConstraint_C(const unsigned int off,
                                                 ChVectorDynamic<>& Qc,
                                                 const double c,
                                                 bool do_clamp,
                                                 double recovery_clamp) {
    int boff = 0;

    double h = 1.0 / c;  // was: this->container->GetSystem()->GetStep(); note not all steppers have c = 1/h

    double alpha = this->damping;
    double inv_hpa = 1.0 / (h + alpha);         // 1/(h+a)
    double inv_hhpa = 1.0 / (h * (h + alpha));  // 1/(h*(h+a))

    for (auto& bond : this->bonds) {
        // TODO  move to LoadKRMMatrices() the following
        // or move in a future IntLoadConstraint_Compliance, or right in IntToDescriptor
        // set compliance 1/h^2 * K^-1, here assuming c=1/h
        bond.second.constraint.SetComplianceTerm(inv_hhpa / bond.second.Km);

        double qc = inv_hpa * bond.second.d_zeta;

        // Note: clamping of Qc in case of compliance is questionable: it does not limit only the outbond
        // speed, but also the reaction, so it might allow longer 'sinking' not related to the real compliance.
        // For this reason, do not do any clamping.
        // if (do_clamp) {
        //    qc = std::min(std::max(qc, -recovery_clamp), recovery_clamp); //=std::max(qc, -recovery_clamp);
        //}

        Qc(off + boff) += qc;

        ++boff;
    }
}

void ChMatterPeriBBimplicit::InjectConstraints(ChSystemDescriptor& descriptor) {
    for (auto& bond : this->bonds) {
        descriptor.InsertConstraint(&bond.second.constraint);
    }
}

void ChMatterPeriBBimplicit::LoadConstraintJacobians() {
    // not needed because already done in ComputeForces() for simplicity
}

void ChMatterPeriBBimplicit::IntToDescriptor(const unsigned int off_v,
                                             const ChStateDelta& v,
                                             const ChVectorDynamic<>& R,
                                             const unsigned int off_L,
                                             const ChVectorDynamic<>& L,
                                             const ChVectorDynamic<>& Qc) {
    int boff = 0;
    for (auto& bond : this->bonds) {
        bond.second.constraint.SetLagrangeMultiplier(L(off_L + boff));
        bond.second.constraint.SetRightHandSide(Qc(off_L + boff));
        ++boff;
    }
}

void ChMatterPeriBBimplicit::IntFromDescriptor(const unsigned int off_v,
                                               ChStateDelta& v,
                                               const unsigned int off_L,
                                               ChVectorDynamic<>& L) {
    int boff = 0;
    for (auto& bond : this->bonds) {
        L(off_L + boff) = bond.second.constraint.GetLagrangeMultiplier();
        ++boff;
    }
}

}  // end namespace chrono
