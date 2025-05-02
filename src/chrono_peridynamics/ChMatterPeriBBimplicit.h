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

#ifndef CHMATTERPERIBBIMPLICIT_H
#define CHMATTERPERIBBIMPLICIT_H


#include "chrono_peridynamics/ChApiPeridynamics.h"
#include "chrono_peridynamics/ChMatterPeridynamics.h"
#include "chrono/solver/ChConstraintTwoGenericBoxed.h"

namespace chrono {
namespace peridynamics {

class ChPeridynamics;

/// @addtogroup chrono_peridynamics
/// @{



/// Helper class: the per-bond auxialiary data for ChMatterPeriBBimplicit

class  ChApiPeridynamics ChMatterDataPerBondBBimplicit : public ChMatterDataPerBond {
public:
    enum class bond_state {
        ACTIVE,     ///< Regular bond, push-pull
        FRACTURED,  ///< Fractured - with small dislocation, collision via box constraint
        BROKEN      ///< Broken - far apart, full collision surfaces generated
    };
    bond_state state = bond_state::ACTIVE;
    double force_density_val = 0;  // force density per vol squared in this bond, for postprocessing
    double d_zeta;   // elongation   (d-zeta)= (sdist - old_sdist),  residual for implicit form
    double Km;  // tangent stiffness matrix, for implicit form

    ChConstraintTwoGenericBoxed constraint;
    //ChConstraintTwoGeneric constraint;

    void Initialize(ChNodePeri* mA, ChNodePeri* mB) override {
        ChMatterDataPerBond::Initialize(mA, mB);
        constraint.SetVariables(mA->GetVariables1(), mB->GetVariables1());
        constraint.SetBoxedMinMax(-1e30, 1e30); // unlimited forces by default
    };
};

/// An implicit form of the ChMatterPeriBBimplicit material, where instead of tangent 
/// stiffness matrices we use a compliant-constraint formulation. Moreover, this adds the ability of 
/// having two stages for breaking: an intermediate fractured state where bonds are still in place but unilateral,
/// and a fully broken state where bonds are removed. 
/// Simple bond-based peridynamic material whose elasticity depends on a single
/// parameter, that is K, the bulk modulus.  
/// The Poisson ratio is always 1/4 and cannot be set otherwise, as in general for bond-based
/// elasticity models. Having a fixed Poisson ration can be a limitation, but the positive
/// note is that this material is computationally-efficient.

class ChApiPeridynamics ChMatterPeriBBimplicit : public ChMatterPeri<ChMatterDataPerNode, ChMatterDataPerBondBBimplicit> {
public:

    /// Set the material Young modulus. The unique bulk modulus will be automatically computed,
    /// since in this material Poisson is always =1/4
    void SetYoungModulus(double mE) {
        // bulk stiffness  K=E/(3(1-2mu)) , with mu=1/4 -> K=E*(2/3)
        this->k_bulk = (mE * (2. / 3.));  
    }

    /// bulk modulus, unit  Pa, i.e. N/m^2
    double k_bulk = 100;

    /// damping, as Rayleigh beta (bulk stiffness-proportional)
    double damping = 0.001;

    /// maximum stretch for fracure - after this, bonds will become unilateral. Default no break.
    double max_stretch_fracture = 1e30;

    /// maximum stretch for full breaking - after this, bonds will break. Default no break.
    double max_stretch_break = 1e30;

    ChMatterPeriBBimplicit() {};

    /// When doing quadrature, particle volumes that overlap with the horizon should be scaled
    /// proportionally to how much of their volume is really inside the horizon sphere. A simple
    /// and very often used approximation is the following 'fading' function.
    double VolumeCorrection(double dist, double horizon, double vol_size) {
        if (dist < (horizon - vol_size))
            return 1.0;
        else
            if (dist < horizon)
                return 1.0 - (dist - (horizon - vol_size)) / (vol_size);
            else
                return 0.0;
    }

    // Implement the function that adds the peridynamics force to each node, as a 
    // summation of all the effects of neighbouring nodes.
    virtual void ComputeForces() {

        // see [Ganzenmueller et al. "Improvements to the Prototype Micro - Brittle Linear Elasticity Model of Peridynamics"]
        // loop on nodes for resetting volume accumulator   //***TODO*** maybe faster in SetupInitial
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
                ChVector3d     vdist = mbond.nodeB->GetPos() - mbond.nodeA->GetPos();
                double     old_sdist = old_vdist.Length();
                double         sdist = vdist.Length();
                ChVector3d     vdir = vdist.GetNormalized();
                double         svel = Vdot(vdir, mbond.nodeB->GetPosDt() - mbond.nodeA->GetPosDt());

                double     stretch = (sdist - old_sdist) / old_sdist;

                // Original PMB in Silling
                // double horizon = mbond.nodeA->GetHorizonRadius();
                // double vol_size = mbond.nodeA->GetVolumeSize();
                // double pih4 = chrono::CH_PI * horizon * horizon * horizon * horizon;
                // double c_silling = (18.0 * k_bulk /  (chrono::CH_PI * horizon * horizon * horizon * horizon) )
                // double force_density_val =  c_silling * stretch; // original in Silling

                // Modified PMB in Ganzenmueller et.al.
                double c_gmuller = 0.5 * ((18.0 * k_bulk / mbond.nodeA->vol_accumulator) + (18.0 * k_bulk / mbond.nodeB->vol_accumulator));
                double force_density_val = (c_gmuller / old_sdist) * stretch;  // in Ganzenmueller et.al.

                if (this->damping > 0)
                    force_density_val += this->damping * (c_gmuller / (old_sdist * old_sdist) ) * svel;

                
                // constraint elongation
                mbond.d_zeta = sdist - old_sdist;
                
                // tangent stiffness,   Km = c * 1/zeta * w_jk
                double wVV_jk = mbond.nodeA->volume * mbond.nodeB->volume; // * VolumeCorrection(old_sdist, horizon, vol_size); // vol corr. not relevant in Ganzenmueller 
                bond.second.Km = (c_gmuller / (old_sdist * old_sdist) ) * wVV_jk;  // that is..   Km= (c_gmuller / old_sdist) * (1.0/old_sdist) * wVV_jk;

                // compute here the jacobians, no need to move in LoadConstraintJacobians()
                mbond.constraint.Get_Cq_a()(0) = -vdir.x();// / old_sdist;
                mbond.constraint.Get_Cq_a()(1) = -vdir.y();// / old_sdist;
                mbond.constraint.Get_Cq_a()(2) = -vdir.z();// / old_sdist;
                mbond.constraint.Get_Cq_b()(0) = vdir.x();// / old_sdist;
                mbond.constraint.Get_Cq_b()(1) = vdir.y();// / old_sdist;
                mbond.constraint.Get_Cq_b()(2) = vdir.z();// / old_sdist;

                if (stretch > max_stretch_fracture) {
                    mbond.force_density_val = 0;
                    mbond.state = ChMatterDataPerBondBBimplicit::bond_state::FRACTURED;
                    mbond.constraint.SetBoxedMinMax(0, 1e30); // enable complementarity, reaction>0.
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

    virtual void Setup() override {
        // cleanup bonds that are broken 
        for (auto& bond : this->bonds) {
            if (bond.second.state == ChMatterDataPerBondBBimplicit::bond_state::BROKEN)
                bonds.erase(bond.first);
        }
    }

    virtual unsigned int GetNumConstraints() override { 
        return bonds.size(); 
    }

    virtual void IntLoadResidual_CqL(const unsigned int off_L,    ///< offset in L multipliers
        ChVectorDynamic<>& R,        ///< result: the R residual, R += c*Cq'*L
        const ChVectorDynamic<>& L,  ///< the L vector
        const double c               ///< a scaling factor
    ) override {

        int boff = 0;
        for (auto& bond : this->bonds) {
            bond.second.constraint.AddJacobianTransposedTimesScalarInto(R, L(off_L + boff) * c);
            ++boff;
        }
    }

    /// Takes the term C, scale and adds to Qc at given offset:
    ///    Qc += c*C
    virtual void IntLoadConstraint_C(const unsigned int off,  ///< offset in Qc residual
        ChVectorDynamic<>& Qc,   ///< result: the Qc residual, Qc += c*C
        const double c,          ///< a scaling factor
        bool do_clamp,           ///< apply clamping to c*C?
        double recovery_clamp    ///< value for min/max clamping of c*C
    ) override {

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

            double qc = inv_hpa* bond.second.d_zeta;

            // Note: clamping of Qc in case of compliance is questionable: it does not limit only the outbond
            // speed, but also the reaction, so it might allow longer 'sinking' not related to the real compliance.
            // For this reason, do not do any clamping.
            //if (do_clamp) {
            //    qc = std::min(std::max(qc, -recovery_clamp), recovery_clamp); //=std::max(qc, -recovery_clamp);
            //}

            Qc(off + boff) += qc;       
            
            ++boff;
        }
    }

    /// Register with the given system descriptor any ChConstraint objects associated with this item.
    virtual void InjectConstraints(ChSystemDescriptor& descriptor) override {
        for (auto& bond : this->bonds) {
            descriptor.InsertConstraint(&bond.second.constraint);
        }
    }

    /// Compute and load current Jacobians in encapsulated ChConstraint objects.
    virtual void LoadConstraintJacobians() override {
        // not needed because already done in ComputeForces() for simplicity
    }

    virtual void IntToDescriptor(const unsigned int off_v,
        const ChStateDelta& v,
        const ChVectorDynamic<>& R,
        const unsigned int off_L,
        const ChVectorDynamic<>& L,
        const ChVectorDynamic<>& Qc) {

        int boff = 0;
        for (auto& bond : this->bonds) {
            bond.second.constraint.SetLagrangeMultiplier(L(off_L+boff));
            bond.second.constraint.SetRightHandSide(Qc(off_L+boff));
            ++boff;
        }
    }

    virtual void IntFromDescriptor(const unsigned int off_v,
        ChStateDelta& v,
        const unsigned int off_L,
        ChVectorDynamic<>& L) {

        int boff = 0;
        for (auto& bond : this->bonds) {
            L(off_L + boff) = bond.second.constraint.GetLagrangeMultiplier();
            ++boff;
        }
    }

};




/// Class for visualization of ChMatterPeriBBimplicit  nodes
/// This can be attached to ChPeridynamics with my_peridynamics->AddVisualShape(my_visual);

class /*ChApiPeridynamics*/ ChVisualPeriBBimplicit : public ChGlyphs {
public:
    ChVisualPeriBBimplicit(std::shared_ptr<ChMatterPeriBBimplicit> amatter) : mmatter(amatter) { is_mutable = true; };
    virtual ~ChVisualPeriBBimplicit() {}

    // Attach velocity property. (ex for postprocessing in falsecolor or with vectors with the Blender add-on)
    void AttachVelocity(double min = 0, double max = 1, std::string mname = "Velocity") {
        vel_property = new ChPropertyVector;
        vel_property->min = min; vel_property->max = max;  vel_property->name = mname;
        this->m_properties.push_back(vel_property);
    }

    // Attach acceleration property. (ex for postprocessing in falsecolor or with vectors with the Blender add-on)
    void AttachAcceleration(double min = 0, double max = 1, std::string mname = "Acceleration") {
        acc_property = new ChPropertyVector;
        acc_property->min = min; acc_property->max = max;  acc_property->name = mname;
        this->m_properties.push_back(acc_property);
    }

    bool draw_colliding = true;
    bool draw_noncolliding = true;

protected:
    ChPropertyVector* vel_property = 0;
    ChPropertyVector* acc_property = 0;

    virtual void Update(ChObj* updater, const ChFrame<>& frame) override {
        if (!mmatter)
            return;

        int totglyphs = 0;
        for (const auto& anode : mmatter->GetMapOfNodes()) {
            if ((anode.first->is_colliding && this->draw_colliding) ||
                (!anode.first->is_colliding && this->draw_noncolliding))
                    ++totglyphs;
        }
        this->Reserve(totglyphs);

        auto mcolor = this->GetColor();

        unsigned int i = 0;
        for (const auto& anode : mmatter->GetMapOfNodes()) {
            if ((anode.first->is_colliding && this->draw_colliding) ||
                (!anode.first->is_colliding && this->draw_noncolliding))  {
                    this->SetGlyphPoint(i, anode.first->GetPos(), mcolor);
                    if (vel_property)
                        vel_property->data[i] = anode.first->GetPosDt();
                    if (acc_property)
                        acc_property->data[i] = anode.first->GetPosDt2();
                    ++i;
            }
        }

    }

    std::shared_ptr<ChMatterPeriBBimplicit> mmatter;
};



/// Class for visualization of ChMatterPeriBBimplicit  bonds
/// This can be attached to ChPeridynamics with my_peridynamics->AddVisualShape(my_visual);

class /*ChApiPeridynamics*/ ChVisualPeriBBimplicitBonds : public ChGlyphs {
public:
    ChVisualPeriBBimplicitBonds(std::shared_ptr<ChMatterPeriBBimplicit> amatter) : mmatter(amatter) { is_mutable = true; };
    virtual ~ChVisualPeriBBimplicitBonds() {}

    bool draw_broken = true;
    bool draw_active = false;
    bool draw_fractured = false;

protected:
    virtual void Update(ChObj* updater, const ChFrame<>& frame) override {
        if (!mmatter)
            return;

        unsigned int count = 0;
        for (const auto& abond : mmatter->GetMapOfBonds()) {
            if ((abond.second.state == ChMatterDataPerBondBBimplicit::bond_state::ACTIVE) && draw_active)
                ++count;
            if ((abond.second.state == ChMatterDataPerBondBBimplicit::bond_state::BROKEN) && draw_broken)
                ++count;
            if ((abond.second.state == ChMatterDataPerBondBBimplicit::bond_state::FRACTURED) && draw_fractured)
                ++count;
        }
        this->Reserve(count);

        unsigned int i = 0;
        for (const auto& abond : mmatter->GetMapOfBonds()) {
            if ((abond.second.state == ChMatterDataPerBondBBimplicit::bond_state::ACTIVE) && draw_active) {
                this->SetGlyphVector(i, abond.second.nodeA->GetPos(), abond.second.nodeB->GetPos() - abond.second.nodeA->GetPos(), ChColor(0.15f, 0.84f, 1.f));
                ++i;
            }
            if ((abond.second.state == ChMatterDataPerBondBBimplicit::bond_state::BROKEN) && draw_broken) {
                this->SetGlyphVector(i, abond.second.nodeA->GetPos(), abond.second.nodeB->GetPos() - abond.second.nodeA->GetPos(), ChColor(1.f, 0.f, 0.f));
                ++i;
            }
            if ((abond.second.state == ChMatterDataPerBondBBimplicit::bond_state::FRACTURED) && draw_fractured) {
                this->SetGlyphVector(i, abond.second.nodeA->GetPos(), abond.second.nodeB->GetPos() - abond.second.nodeA->GetPos(), ChColor(1.f, 0.5f, 0.f));
                ++i;
            }

        }
    }

    std::shared_ptr<ChMatterPeriBBimplicit> mmatter;
};





/// @} chrono_peridynamics

}  // end namespace peridynamics
}  // end namespace chrono

#endif
