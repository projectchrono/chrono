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

#ifndef CHMATTERPERIBULKELASTIC_H
#define CHMATTERPERIBULKELASTIC_H


#include "chrono_peridynamics/ChApiPeridynamics.h"
#include "chrono_peridynamics/ChMatterPeridynamics.h"
#include "chrono/solver/ChConstraintTwoGenericBoxed.h"

namespace chrono {
namespace peridynamics {

class ChPeridynamics;

/// @addtogroup chrono_peridynamics
/// @{



/// Helper class: the per-bond auxialiary data for ChMatterPeriBulkElastic

class  ChApiPeridynamics ChMatterDataPerBoundBulk : public ChMatterDataPerBound { 
public: 
    bool broken = false;
    double F_density = 0;  // force density per volume squared in this bound
};


/// Simple bond-based peridynamic material whose elasticity depends on a single
/// parameter, that is K, the bulk modulus.  
/// The Poisson ratio is always 1/4 and cannot be set otherwise, as in general for bond-based
/// elasticity models. Having a fixed Poisson ration can be a limitation, but the positive
/// note is that this material is very computationally-efficient.

class ChApiPeridynamics ChMatterPeriBulkElastic : public ChMatterPeri<ChMatterDataPerNode, ChMatterDataPerBoundBulk> {
public:
    /// bulk modulus, unit  Pa, i.e. N/m^2
    double k_bulk = 100; 

    /// bulk damping, unit  Pa*s/m, i.e. Ns/m^3
    double r_bulk = 10;
    
    /// maximum stretch - after this, bonds will break. Default no break.
    double max_stretch = 1e30;

    ChMatterPeriBulkElastic() {};

    // Implement the function that adds the peridynamics force to each node, as a 
    // summation of all the effects of neighbouring nodes.
    virtual void ComputeForces() {
        // loop on bounds
        for (auto& bound : this->bounds) {
            ChMatterDataPerBoundBulk& mbound = bound.second;
            if (!mbound.broken) {
                ChVector3d old_vdist = mbound.nodeB->GetX0() - mbound.nodeA->GetX0();
                ChVector3d     vdist = mbound.nodeB->GetPos() - mbound.nodeA->GetPos();
                double     old_sdist = old_vdist.Length();
                double         sdist = vdist.Length();
                ChVector3d     vdir = vdist.GetNormalized();
                double         svel = Vdot(vdir, mbound.nodeB->GetPosDt() - mbound.nodeA->GetPosDt());

                double     stretch = (sdist - old_sdist) / old_sdist;

                double horizon = mbound.nodeA->GetHorizonRadius();
                double pih4 = chrono::CH_PI * horizon * horizon * horizon * horizon;

                double force_density_val =  (18.0 * k_bulk / pih4) * stretch;
                
                if (this->r_bulk > 0)
                    force_density_val +=  (18.0 * r_bulk / pih4) * svel;

                mbound.F_density = force_density_val;
                mbound.nodeB->F_peridyn += -vdir * force_density_val * mbound.nodeA->volume * mbound.nodeB->volume;
                mbound.nodeA->F_peridyn += vdir * force_density_val * mbound.nodeB->volume * mbound.nodeA->volume;

                if (stretch > max_stretch) {
                    mbound.F_density = 0;
                    mbound.broken = true;
                    // the following will propagate the fracture geometry so that broken parts can collide
                    mbound.nodeA->is_boundary = true; 
                    mbound.nodeB->is_boundary = true;
                }
            }
            else {
                if ((mbound.nodeB->GetPos() - mbound.nodeA->GetPos()).Length() > mbound.nodeA->GetHorizonRadius())
                    bounds.erase(bound.first);
            }

        }
    }
};




/// Class for visualization of ChMatterPeriBulkElastic  nodes
/// This can be attached to ChPeridynamics with my_peridynamics->AddVisualShape(my_visual);

class /*ChApiPeridynamics*/ ChVisualPeriBulkElastic : public ChGlyphs {
public:
    ChVisualPeriBulkElastic(std::shared_ptr<ChMatterPeriBulkElastic> amatter) : mmatter(amatter) { is_mutable = true; };
    virtual ~ChVisualPeriBulkElastic() {}

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
    

protected:
    ChPropertyVector* vel_property =0;
    ChPropertyVector* acc_property =0;

    virtual void Update(ChPhysicsItem* updater, const ChFrame<>& frame) {
        if (!mmatter)
            return;
        this->Reserve(mmatter->GetNnodes());

        unsigned int i = 0;
        for (const auto& anode : mmatter->GetMapOfNodes()) {
            this->SetGlyphPoint(i, anode.first->GetPos());
            if (vel_property) 
                vel_property->data[i] = anode.first->GetPosDt();
            if (acc_property) 
                acc_property->data[i] = anode.first->GetPosDt2();
            ++i;
        }

    }

    std::shared_ptr<ChMatterPeriBulkElastic> mmatter;
};



/// Class for visualization of ChMatterPeriBulkElastic  bounds
/// This can be attached to ChPeridynamics with my_peridynamics->AddVisualShape(my_visual);

class /*ChApiPeridynamics*/ ChVisualPeriBulkElasticBounds : public ChGlyphs {
public:
    ChVisualPeriBulkElasticBounds(std::shared_ptr<ChMatterPeriBulkElastic> amatter) : mmatter(amatter) { is_mutable = true; };
    virtual ~ChVisualPeriBulkElasticBounds() {}

    bool draw_broken = true;
    bool draw_unbroken = false;

protected:
    virtual void Update(ChPhysicsItem* updater, const ChFrame<>& frame) {
        if (!mmatter)
            return;

        unsigned int count = 0;
        for (const auto& abound : mmatter->GetMapOfBounds()) {
            if (abound.second.broken && draw_broken)
                ++count;
            if (!abound.second.broken && draw_unbroken)
                ++count;
        }
        this->Reserve(count);

        unsigned int i = 0;
        for (const auto& abound : mmatter->GetMapOfBounds()) {
            if (abound.second.broken && draw_broken) {
                this->SetGlyphVector(i, abound.second.nodeA->GetPos(), abound.second.nodeB->GetPos() - abound.second.nodeA->GetPos(), ChColor(1, 0, 0));
                ++i;
            }
            if (!abound.second.broken && draw_unbroken) {
                this->SetGlyphVector(i, abound.second.nodeA->GetPos(), abound.second.nodeB->GetPos() - abound.second.nodeA->GetPos(), ChColor(0, 0, 1));
                ++i;
            }
        }
    }

    std::shared_ptr<ChMatterPeriBulkElastic> mmatter;
};



////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


/// Helper class: the per-bond auxialiary data for ChMatterPeriBulkElasticImplicit

class  ChApiPeridynamics ChMatterDataPerBoundBulkImplicit : public ChMatterDataPerBound {
public:
    enum class bond_state {
        ACTIVE,     ///< Regular bond, push-pull
        FRACTURED,  ///< Fractured - with small dislocation, collision via box constraint
        BROKEN      ///< Broken - far apart, full collision surfaces generated
    };
    bond_state state = bond_state::ACTIVE;
    double force_density_val = 0;  // force density per vol squared in this bond, for postprocessing
    double Phi; // residual   (d-zeta)= (sdist - old_sdist),  for implicit form
    double Km;  // tangent stiffness matrix, for implicit form

    ChConstraintTwoGenericBoxed constraint;
    //ChConstraintTwoGeneric constraint;

    void Initialize(ChNodePeri* mA, ChNodePeri* mB) override {
        ChMatterDataPerBound::Initialize(mA, mB);
        constraint.SetVariables(mA->GetVariables1(), mB->GetVariables1());
        constraint.SetBoxedMinMax(-1e30, 1e30); // unlimited forces by default
    };
};

/// An implicit form of the ChMatterPeriBulkImplicit material, where instead of tangent 
/// stiffness matrices we use a compliant-constraint formulation. Moreover, this adds the ability of 
/// having two stages for breaking: an intermediate fractured state where bonds are still in place but unilateral,
/// and a fully broken state where bonds are removed. 
/// Simple bond-based peridynamic material whose elasticity depends on a single
/// parameter, that is K, the bulk modulus.  
/// The Poisson ratio is always 1/4 and cannot be set otherwise, as in general for bond-based
/// elasticity models. Having a fixed Poisson ration can be a limitation, but the positive
/// note is that this material is computationally-efficient.

class ChApiPeridynamics ChMatterPeriBulkImplicit : public ChMatterPeri<ChMatterDataPerNode, ChMatterDataPerBoundBulkImplicit> {
public:
    /// bulk modulus, unit  Pa, i.e. N/m^2
    double k_bulk = 100;

    /// bulk damping, unit  Pa*s/m, i.e. Ns/m^3
    double r_bulk = 10;

    /// maximum stretch for fracure - after this, bonds will become unilateral. Default no break.
    double max_stretch_fracture = 1e30;

    /// maximum stretch for full breaking - after this, bonds will break. Default no break.
    double max_stretch_break = 1e30;

    ChMatterPeriBulkImplicit() {};

    // Implement the function that adds the peridynamics force to each node, as a 
    // summation of all the effects of neighbouring nodes.
    virtual void ComputeForces() {
        // loop on bounds
        for (auto& bound : this->bounds) {
            ChMatterDataPerBoundBulkImplicit& mbound = bound.second;
            if (mbound.state != ChMatterDataPerBoundBulkImplicit::bond_state::BROKEN) {
                ChVector3d old_vdist = mbound.nodeB->GetX0() - mbound.nodeA->GetX0();
                ChVector3d     vdist = mbound.nodeB->GetPos() - mbound.nodeA->GetPos();
                double     old_sdist = old_vdist.Length();
                double         sdist = vdist.Length();
                ChVector3d     vdir = vdist.GetNormalized();
                double         svel = Vdot(vdir, mbound.nodeB->GetPosDt() - mbound.nodeA->GetPosDt());

                double     stretch = (sdist - old_sdist) / old_sdist;

                double horizon = mbound.nodeA->GetHorizonRadius();
                double pih4 = chrono::CH_PI * horizon * horizon * horizon * horizon;
                
                mbound.force_density_val = (18.0 * k_bulk / pih4) * stretch;

                if (this->r_bulk > 0)
                    mbound.force_density_val += (18.0 * r_bulk / pih4) * svel;

                // constraint elongation
                mbound.Phi = sdist - old_sdist;
                
                // tangent stiffness,   Km = c * 1/zeta * w_jk
                bound.second.Km = ((18.0 * k_bulk / pih4) * (1.0/old_sdist) * mbound.nodeA->volume * mbound.nodeB->volume);

                // compute here the jacobians, no need to move in LoadConstraintJacobians()
                mbound.constraint.Get_Cq_a()(0) = -vdir.x() / old_sdist;
                mbound.constraint.Get_Cq_a()(1) = -vdir.y() / old_sdist;
                mbound.constraint.Get_Cq_a()(2) = -vdir.z() / old_sdist;
                mbound.constraint.Get_Cq_b()(0) = vdir.x() / old_sdist;
                mbound.constraint.Get_Cq_b()(1) = vdir.y() / old_sdist;
                mbound.constraint.Get_Cq_b()(2) = vdir.z() / old_sdist;

                if (stretch > max_stretch_fracture) {
                    mbound.force_density_val = 0;
                    mbound.state = ChMatterDataPerBoundBulkImplicit::bond_state::FRACTURED;
                    mbound.constraint.SetBoxedMinMax(0, 1e30); // enable complementarity, reaction>0.
                }

                if (stretch > max_stretch_break) {
                    mbound.force_density_val = 0;
                    mbound.state = ChMatterDataPerBoundBulkImplicit::bond_state::BROKEN;
                    // the following will propagate the fracture geometry so that broken parts can collide
                    mbound.nodeA->is_boundary = true;
                    mbound.nodeB->is_boundary = true;
                }
                  
            }
        }
    }

    virtual void Setup() override {
        // cleanup bounds that are broken and far apart 
        for (auto& bound : this->bounds) {
            if ((bound.second.nodeB->GetPos() - bound.second.nodeA->GetPos()).Length() > bound.second.nodeA->GetHorizonRadius())
                bounds.erase(bound.first);
        }
    }

    virtual unsigned int GetNumConstraints() override { 
        return bounds.size(); 
    }

    virtual void IntLoadResidual_CqL(const unsigned int off_L,    ///< offset in L multipliers
        ChVectorDynamic<>& R,        ///< result: the R residual, R += c*Cq'*L
        const ChVectorDynamic<>& L,  ///< the L vector
        const double c               ///< a scaling factor
    ) override {

        int boff = 0;
        for (auto& bound : this->bounds) {
            bound.second.constraint.AddJacobianTransposedTimesScalarInto(R, L(off_L + boff) * c);
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

        double alpha = this->r_bulk / this->k_bulk; // [R]=alpha*[K]
        double inv_hpa = 1.0 / (h + alpha);         // 1/(h+a)
        double inv_hhpa = 1.0 / (h * (h + alpha));  // 1/(h*(h+a))

        for (auto& bound : this->bounds) {
            

            // TODO  move to LoadKRMMatrices() the following
            // or move in a future IntLoadConstraint_Compliance, or right in IntToDescriptor
            // set compliance 1/h^2 * K^-1, here assuming c=1/h
            bound.second.constraint.SetComplianceTerm((inv_hhpa) / bound.second.Km);

            double qc = inv_hpa * bound.second.Phi;

            // Note: clamping of Qc in case of compliance is questionable: it does not limit only the outbound
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
        for (auto& bound : this->bounds) {
            descriptor.InsertConstraint(&bound.second.constraint);
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
        for (auto& bound : this->bounds) {
            bound.second.constraint.SetLagrangeMultiplier(L(off_L+boff));
            bound.second.constraint.SetRightHandSide(Qc(off_L+boff));
            ++boff;
        }
    }

    virtual void IntFromDescriptor(const unsigned int off_v,
        ChStateDelta& v,
        const unsigned int off_L,
        ChVectorDynamic<>& L) {

        int boff = 0;
        for (auto& bound : this->bounds) {
            L(off_L + boff) = bound.second.constraint.GetLagrangeMultiplier();
            ++boff;
        }
    }

};




/// Class for visualization of ChMatterPeriBulkImplicit  nodes
/// This can be attached to ChPeridynamics with my_peridynamics->AddVisualShape(my_visual);

class /*ChApiPeridynamics*/ ChVisualPeriBulkImplicit : public ChGlyphs {
public:
    ChVisualPeriBulkImplicit(std::shared_ptr<ChMatterPeriBulkImplicit> amatter) : mmatter(amatter) { is_mutable = true; };
    virtual ~ChVisualPeriBulkImplicit() {}

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


protected:
    ChPropertyVector* vel_property = 0;
    ChPropertyVector* acc_property = 0;

    virtual void Update(ChPhysicsItem* updater, const ChFrame<>& frame) {
        if (!mmatter)
            return;
        this->Reserve(mmatter->GetNnodes());

        unsigned int i = 0;
        for (const auto& anode : mmatter->GetMapOfNodes()) {
            this->SetGlyphPoint(i, anode.first->GetPos());
            if (vel_property)
                vel_property->data[i] = anode.first->GetPosDt();
            if (acc_property)
                acc_property->data[i] = anode.first->GetPosDt2();
            ++i;
        }

    }

    std::shared_ptr<ChMatterPeriBulkImplicit> mmatter;
};



/// Class for visualization of ChMatterPeriBulkImplicit  bonds
/// This can be attached to ChPeridynamics with my_peridynamics->AddVisualShape(my_visual);

class /*ChApiPeridynamics*/ ChVisualPeriBulkImplicitBounds : public ChGlyphs {
public:
    ChVisualPeriBulkImplicitBounds(std::shared_ptr<ChMatterPeriBulkImplicit> amatter) : mmatter(amatter) { is_mutable = true; };
    virtual ~ChVisualPeriBulkImplicitBounds() {}

    bool draw_broken = true;
    bool draw_active = false;
    bool draw_fractured = false;

protected:
    virtual void Update(ChPhysicsItem* updater, const ChFrame<>& frame) {
        if (!mmatter)
            return;

        unsigned int count = 0;
        for (const auto& abound : mmatter->GetMapOfBounds()) {
            if ((abound.second.state == ChMatterDataPerBoundBulkImplicit::bond_state::ACTIVE) && draw_active)
                ++count;
            if ((abound.second.state == ChMatterDataPerBoundBulkImplicit::bond_state::BROKEN) && draw_broken)
                ++count;
            if ((abound.second.state == ChMatterDataPerBoundBulkImplicit::bond_state::FRACTURED) && draw_fractured)
                ++count;
        }
        this->Reserve(count);

        unsigned int i = 0;
        for (const auto& abound : mmatter->GetMapOfBounds()) {
            if ((abound.second.state == ChMatterDataPerBoundBulkImplicit::bond_state::ACTIVE) && draw_active) {
                this->SetGlyphVector(i, abound.second.nodeA->GetPos(), abound.second.nodeB->GetPos() - abound.second.nodeA->GetPos(), ChColor(0, 0, 1));
                ++i;
            }
            if ((abound.second.state == ChMatterDataPerBoundBulkImplicit::bond_state::BROKEN) && draw_broken) {
                this->SetGlyphVector(i, abound.second.nodeA->GetPos(), abound.second.nodeB->GetPos() - abound.second.nodeA->GetPos(), ChColor(1, 0, 0));
                ++i;
            }
            if ((abound.second.state == ChMatterDataPerBoundBulkImplicit::bond_state::FRACTURED) && draw_fractured) {
                this->SetGlyphVector(i, abound.second.nodeA->GetPos(), abound.second.nodeB->GetPos() - abound.second.nodeA->GetPos(), ChColor(1, 1, 0));
                ++i;
            }

        }
    }

    std::shared_ptr<ChMatterPeriBulkImplicit> mmatter;
};



////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////




/// Helper class: the per-node auxiliary data for ChMatterDataPerBoundLinearElastic

class  ChApiPeridynamics ChMatterDataPerNodeLinearElastic : public ChMatterDataPerNode {
public:
    double m = 0;  // weighted volume
    double theta = 0; // dilation
};

/// Helper class: the per-bound auxiliary data for ChMatterDataPerBoundLinearElastic

class  ChApiPeridynamics ChMatterDataPerBoundLinearElastic : public ChMatterDataPerBound {
public:
    bool   broken = false;
    double F_per_bond = 0;  // force density in this bound
};


/// Simple state-based peridynamic material whose elasticity depends on two
/// parameters, as in a 3D linear elastic Hookean material, that is:
///  - K, the bulk modulus.  
///  - mu, the Poisson ratio.
/// This comes at a cost of slower performance compared to the ChMatterPeriBulkElastic 
/// bond-based elasticity model, that has Poisson fixed to 1/4. 

class ChApiPeridynamics ChMatterPeriLinearElastic : public ChMatterPeri<ChMatterDataPerNodeLinearElastic, ChMatterDataPerBoundLinearElastic> {
public:
    /// bulk modulus, unit  Pa, i.e. N/m^2
    double k_bulk = 100;

    /// poisson coefficient (ex. -0.5 incompressible)
    double poisson = 0.25;

    /// bulk damping, unit  Pa*s/m, i.e. Ns/m^3
    double r_bulk = 10;

    /// maximum stretch - after this, bonds will break. Default no break.
    double max_stretch = 1e30;

    ChMatterPeriLinearElastic() {};

    double InfluenceFunction(double zeta, double horizon) {
        if (zeta > horizon)
            return 0.0;
        
        // linear decrease
        //return horizon / zeta;

        // constant one:
        return 1.0;

        // piecewise decay
        /*
        double normdist = zeta / horizon;
        double value = normdist < 0.5 ? 1.0 : -4.0 * normdist * normdist + 4.0 * normdist;
        return value;
        */
    }


    // Initialize material with weighted volume
    virtual void SetupInitial() {
        /*
        // loop on nodes
        for (auto& node : this->nodes) {
            ChMatterDataPerNodeLinearElastic& mnodedata = node.second;
            mnodedata.m = 0;
            mnodedata.theta = 0;
        }
        // loop on bounds
        for (auto& bound : this->bounds) {
            ChMatterDataPerBoundLinearElastic& mbound = bound.second;
            ChVector3d old_vdist = mbound.nodeB->GetX0() - mbound.nodeA->GetX0();
            double     old_sdist = old_vdist.Length();
            double horizon = mbound.nodeA->GetHorizonRadius();
            double omega = this->InfluenceFunction(old_sdist, horizon);
            ChMatterDataPerNodeLinearElastic& mnodedataA = this->nodes[mbound.nodeA];
            ChMatterDataPerNodeLinearElastic& mnodedataB = this->nodes[mbound.nodeB];
            mnodedataA.m += omega * old_sdist * old_sdist * mbound.nodeB->volume;
            mnodedataB.m += omega * old_sdist * old_sdist * mbound.nodeA->volume;
        }
        */
    }


    // Implement the function that adds the peridynamics force to each node, as a 
    // summation of all the effects of neighbouring nodes.
    // Formulas based on "Peridynamics-Based Fracture Animation for Elastoplastic
    // Solids", W.Chen, F.Zhu, J.Zhao, S.Li, G.Wang, CG Forum 2017
    virtual void ComputeForces() {
        
        // loop on nodes for resetting dilation 
        for (auto& node : this->nodes) {
            ChMatterDataPerNodeLinearElastic& mnodedata = node.second;
            mnodedata.theta = 0;
        }

        // loop on bounds for dilation 
        for (auto& bound : this->bounds) {
            ChMatterDataPerBoundLinearElastic& mbound = bound.second;
            if (!mbound.broken) {
                ChVector3d old_vdist = mbound.nodeB->GetX0() - mbound.nodeA->GetX0();
                ChVector3d     vdist = mbound.nodeB->GetPos() - mbound.nodeA->GetPos();
                double     old_sdist = old_vdist.Length();
                double         sdist = vdist.Length();
                ChVector3d     vdir  = vdist/sdist;

                double horizon = mbound.nodeA->GetHorizonRadius();
                double omega = this->InfluenceFunction(old_sdist, horizon);

                ChMatterDataPerNodeLinearElastic& mnodedataA = this->nodes[mbound.nodeA];
                ChMatterDataPerNodeLinearElastic& mnodedataB = this->nodes[mbound.nodeB];

                double s = (sdist/old_sdist) -1.0; // stretch
                double form = s * omega * 9.0 / (4.0 * CH_PI * horizon * horizon * horizon * horizon) * Vdot(vdir,old_vdist);
                mnodedataA.theta += form * mbound.nodeB->volume;
                mnodedataB.theta += form * mbound.nodeA->volume;
            }
            else {
                if ((mbound.nodeB->GetPos() - mbound.nodeA->GetPos()).Length() > mbound.nodeA->GetHorizonRadius())
                    bounds.erase(bound.first);
            }
        }

        // loop on bounds for force computation 
        for (auto& bound : this->bounds) {
            ChMatterDataPerBoundLinearElastic& mbound = bound.second;
            ChVector3d old_vdist = mbound.nodeB->GetX0() - mbound.nodeA->GetX0();
            ChVector3d     vdist = mbound.nodeB->GetPos() - mbound.nodeA->GetPos();
            double     old_sdist = old_vdist.Length();
            double         sdist = vdist.Length();
            ChVector3d     vdir = vdist / sdist;
            ChVector3d old_vdir = old_vdist / old_sdist;
            double         svel = Vdot(vdir, mbound.nodeB->GetPosDt() - mbound.nodeA->GetPosDt());
            
            double e = sdist - old_sdist;

            double horizon = mbound.nodeA->GetHorizonRadius();
            double omega = this->InfluenceFunction(old_sdist, horizon);

            ChMatterDataPerNodeLinearElastic& mnodedataA = this->nodes[mbound.nodeA];
            ChMatterDataPerNodeLinearElastic& mnodedataB = this->nodes[mbound.nodeB];

            double a = 9.0 * this->k_bulk / (8.0 * CH_PI * pow(horizon, 4.));
            double b = 15.0 * this->poisson / (2.0 * CH_PI * pow(horizon, 5.));
            //double a_d = 9.0 / (4.0 * CH_PI * pow(horizon, 4.)) * 0.5*(this->k_bulk - (5./3.)*this->poisson);

            double A_dil_A = 4.0 * omega * a * Vdot(vdir, old_vdir) * mnodedataA.theta;
            double A_dil_B = 4.0 * omega * a * Vdot(vdir, old_vdir) * mnodedataB.theta;

            double A_dev_A = 4.0 * omega * b *  (e - (horizon / 4.0) * Vdot(vdir, old_vdir) * mnodedataA.theta);
            double A_dev_B = 4.0 * omega * b *  (e - (horizon / 4.0) * Vdot(vdir, old_vdir) * mnodedataB.theta);
           
            double t_A = (A_dil_A + A_dev_A);
            double t_B = (A_dil_B + A_dev_B);

            if (this->r_bulk > 0) {
                double pih4 = chrono::CH_PI * horizon * horizon * horizon * horizon;
                double viscforce = 0.5 * (18.0 * r_bulk / pih4) * svel;
                t_A += viscforce;
                t_B += viscforce;
            }
            
            mbound.nodeB->F_peridyn += -vdir * (t_B + t_A) * mbound.nodeA->volume  * mbound.nodeB->volume;
            mbound.nodeA->F_peridyn +=  vdir * (t_B + t_A) * mbound.nodeB->volume  * mbound.nodeA->volume;
        }

    }
};




/// Class for visualization of ChMatterDataPerBoundLinearElastic  nodes
/// This can be attached to ChPeridynamics with my_peridynamics->AddVisualShape(my_visual);

class /*ChApiPeridynamics*/ ChVisualPeriLinearElastic : public ChGlyphs {
public:
    ChVisualPeriLinearElastic(std::shared_ptr<ChMatterPeriLinearElastic> amatter) : mmatter(amatter) { is_mutable = true; };
    virtual ~ChVisualPeriLinearElastic() {}

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

    // Attach dilation (theta) property. (ex for postprocessing in falsecolor or with vectors with the Blender add-on)
    void AttachDilation(double min = 0, double max = 1, std::string mname = "Dilation") {
        theta_property = new ChPropertyScalar;
        theta_property->min = min; theta_property->max = max;  theta_property->name = mname;
        this->m_properties.push_back(theta_property);
    }


protected:
    ChPropertyVector* vel_property = 0;
    ChPropertyVector* acc_property = 0;
    ChPropertyScalar* theta_property = 0;

    virtual void Update(ChPhysicsItem* updater, const ChFrame<>& frame) {
        if (!mmatter)
            return;
        this->Reserve(mmatter->GetNnodes());

        unsigned int i = 0;
        for (const auto& anode : mmatter->GetMapOfNodes()) {
            this->SetGlyphPoint(i, anode.first->GetPos());
            if (vel_property)
                vel_property->data[i] = anode.first->GetPosDt();
            if (acc_property)
                acc_property->data[i] = anode.first->GetPosDt2();
            if (theta_property)
                theta_property->data[i]  = mmatter->GetMapOfNodes().at(anode.first).theta;
            ++i;
        }

    }

    std::shared_ptr<ChMatterPeriLinearElastic> mmatter;
};



/// Class for visualization of ChMatterPeriLinearElastic  bounds
/// This can be attached to ChPeridynamics with my_peridynamics->AddVisualShape(my_visual);

class /*ChApiPeridynamics*/ ChVisualPeriLinearElasticBounds : public ChGlyphs {
public:
    ChVisualPeriLinearElasticBounds(std::shared_ptr<ChMatterPeriLinearElastic> amatter) : mmatter(amatter) { is_mutable = true; };
    virtual ~ChVisualPeriLinearElasticBounds() {}

    bool draw_broken = true;
    bool draw_unbroken = false;

protected:
    virtual void Update(ChPhysicsItem* updater, const ChFrame<>& frame) {
        if (!mmatter)
            return;

        unsigned int count = 0;
        for (const auto& abound : mmatter->GetMapOfBounds()) {
            if (abound.second.broken && draw_broken)
                ++count;
            if (!abound.second.broken && draw_unbroken)
                ++count;
        }
        this->Reserve(count);

        unsigned int i = 0;
        for (const auto& abound : mmatter->GetMapOfBounds()) {
            if (abound.second.broken && draw_broken) {
                this->SetGlyphVector(i, abound.second.nodeA->GetPos(), abound.second.nodeB->GetPos() - abound.second.nodeA->GetPos(), ChColor(1, 0, 0));
                ++i;
            }
            if (!abound.second.broken && draw_unbroken) {
                this->SetGlyphVector(i, abound.second.nodeA->GetPos(), abound.second.nodeB->GetPos() - abound.second.nodeA->GetPos(), ChColor(0, 0, 1));
                ++i;
            }
        }
    }

    std::shared_ptr<ChMatterPeriLinearElastic> mmatter;
};















/// @} chrono_peridynamics

}  // end namespace peridynamics
}  // end namespace chrono

#endif
