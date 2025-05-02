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

#ifndef CHMATTERPERILINEARELASTIC_H
#define CHMATTERPERILINEARELASTIC_H


#include "chrono_peridynamics/ChApiPeridynamics.h"
#include "chrono_peridynamics/ChMatterPeridynamics.h"
#include "chrono/solver/ChConstraintTwoGenericBoxed.h"

namespace chrono {
namespace peridynamics {

class ChPeridynamics;

/// @addtogroup chrono_peridynamics
/// @{



/// Helper class: the per-node auxiliary data for ChMatterDataPerBondLinearElastic

class  ChApiPeridynamics ChMatterDataPerNodeLinearElastic : public ChMatterDataPerNode {
public:
    double m = 0;  // weighted volume
    double theta = 0; // dilation
};

/// Helper class: the per-bond auxiliary data for ChMatterDataPerBondLinearElastic

class  ChApiPeridynamics ChMatterDataPerBondLinearElastic : public ChMatterDataPerBond {
public:
    bool   broken = false;
    double F_per_bond = 0;  // force density in this bond
};


/// Simple state-based peridynamic material whose elasticity depends on two
/// parameters, as in a 3D linear elastic Hookean material, that is:
///  - K, the bulk modulus.  
///  - G, the shear modulus.
/// This comes at a cost of slower performance compared to the ChMatterPeriBB 
/// bond-based elasticity model, that has Poisson fixed to 1/4. 
/// For very high stiffness, time integration might diverge because it does not introduce any tangent 
/// stiffness matrix (even if using implicit integrators like HHT, this will behave like in explicit anyway); 
/// use ChMatterPeriBBimplicit in these cases.

class ChApiPeridynamics ChMatterPeriLinearElastic : public ChMatterPeri<ChMatterDataPerNodeLinearElastic, ChMatterDataPerBondLinearElastic> {
public:
    void SetYoungModulusShearModulus(double mE, double mG) {
        this->k_bulk = (mE * mG / (3. * (3. * mG - mE)));
        this->G = mG;
    }
    void SetYoungModulusPoisson(double mE, double mu) {
        this->k_bulk = (mE / (3. * (1. - 2.* mu)));
        this->G      = (mE / (2. * (1. +  mu)));
    }

    /// bulk modulus, unit  Pa, i.e. N/m^2
    double k_bulk = 100;

    /// shear modulus 
    double G = 100;

    /// bulk damping, unit  Pa*s/m, i.e. Ns/m^3 ***NOT USED***
    double r_bulk = 0;

    /// maximum stretch - after this, bonds will break. Default no break.
    double max_stretch = 1e30;

    ChMatterPeriLinearElastic() {
        assert(false); // Material not ready to use. TO DO
    };

    double InfluenceFunction(double zeta, double horizon) {
        if (zeta > horizon)
            return 0.0;

        // inv.linear decrease
        return horizon / zeta;
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


    // Implement the function that adds the peridynamics force to each node, as a 
    // summation of all the effects of neighbouring nodes.
    // Formulas based on Silling work
    virtual void ComputeForces() {
        
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
                double     old_sdist = old_vdist.Length();

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
                ChVector3d     vdist = mbond.nodeB->GetPos() - mbond.nodeA->GetPos();
                double     old_sdist = old_vdist.Length();
                double         sdist = vdist.Length();

                double horizon = mbond.nodeA->GetHorizonRadius();
                double omega = this->InfluenceFunction(old_sdist, horizon);

                ChMatterDataPerNodeLinearElastic& mnodedataA = this->nodes[mbond.nodeA];
                ChMatterDataPerNodeLinearElastic& mnodedataB = this->nodes[mbond.nodeB];

                mnodedataA.theta += (3. / mnodedataA.m) * omega * old_sdist * (sdist - old_sdist) * mbond.nodeB->volume;
                mnodedataB.theta += (3. / mnodedataB.m) * omega * old_sdist * (sdist - old_sdist) * mbond.nodeA->volume;
            }
            else {
                if (mbond.broken)
                    bonds.erase(bond.first);
            }
        }

        // loop on bonds for force computation 
        for (auto& bond : this->bonds) {
            ChMatterDataPerBondLinearElastic& mbond = bond.second;
            ChVector3d old_vdist = mbond.nodeB->GetX0() - mbond.nodeA->GetX0();
            ChVector3d     vdist = mbond.nodeB->GetPos() - mbond.nodeA->GetPos();
            double     old_sdist = old_vdist.Length();
            double         sdist = vdist.Length();
            ChVector3d     vdir = vdist / sdist;
            ChVector3d old_vdir = old_vdist / old_sdist;
            //double         svel = Vdot(vdir, mbond.nodeB->GetPosDt() - mbond.nodeA->GetPosDt());
            
            double e = sdist - old_sdist;

            double horizon = mbond.nodeA->GetHorizonRadius();
            double omega = this->InfluenceFunction(old_sdist, horizon);

            ChMatterDataPerNodeLinearElastic& mnodedataA = this->nodes[mbond.nodeA];
            ChMatterDataPerNodeLinearElastic& mnodedataB = this->nodes[mbond.nodeB];

            // Silling:
            double e_i_A = mnodedataA.theta * old_sdist / 3.0; // isotropic
            double e_i_B = mnodedataB.theta * old_sdist / 3.0; // isotropic
            double e_d_A = e - e_i_A; // deviatoric
            double e_d_B = e - e_i_B; // deviatoric
            double t_A = 3. * this->k_bulk * mnodedataA.theta * omega * old_sdist / mnodedataA.m + 15. * this->G / mnodedataA.m * omega * e_d_A;
            double t_B = 3. * this->k_bulk * mnodedataB.theta * omega * old_sdist / mnodedataB.m + 15. * this->G / mnodedataB.m * omega * e_d_B;

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

            mbond.nodeB->F_peridyn += -vdir * 0.5*( t_B + t_A) * mbond.nodeA->volume  * mbond.nodeB->volume;
            mbond.nodeA->F_peridyn +=  vdir * 0.5*( t_B + t_A) * mbond.nodeB->volume  * mbond.nodeA->volume;
        }

    }
};




/// Class for visualization of ChMatterDataPerBondLinearElastic  nodes
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

    virtual void Update(ChObj* updater, const ChFrame<>& frame) override {
        if (!mmatter)
            return;
        this->Reserve(mmatter->GetNnodes());

        auto mcolor = this->GetColor();
        unsigned int i = 0;
        for (const auto& anode : mmatter->GetMapOfNodes()) {
            this->SetGlyphPoint(i, anode.first->GetPos(), mcolor);
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



/// Class for visualization of ChMatterPeriLinearElastic  bonds
/// This can be attached to ChPeridynamics with my_peridynamics->AddVisualShape(my_visual);

class /*ChApiPeridynamics*/ ChVisualPeriLinearElasticBonds : public ChGlyphs {
public:
    ChVisualPeriLinearElasticBonds(std::shared_ptr<ChMatterPeriLinearElastic> amatter) : mmatter(amatter) { is_mutable = true; };
    virtual ~ChVisualPeriLinearElasticBonds() {}

    bool draw_broken = true;
    bool draw_unbroken = false;

protected:
    virtual void Update(ChObj* updater, const ChFrame<>& frame) override {
        if (!mmatter)
            return;

        unsigned int count = 0;
        for (const auto& abond : mmatter->GetMapOfBonds()) {
            if (abond.second.broken && draw_broken)
                ++count;
            if (!abond.second.broken && draw_unbroken)
                ++count;
        }
        this->Reserve(count);

        unsigned int i = 0;
        for (const auto& abond : mmatter->GetMapOfBonds()) {
            if (abond.second.broken && draw_broken) {
                this->SetGlyphVector(i, abond.second.nodeA->GetPos(), abond.second.nodeB->GetPos() - abond.second.nodeA->GetPos(), ChColor(1, 0, 0));
                ++i;
            }
            if (!abond.second.broken && draw_unbroken) {
                this->SetGlyphVector(i, abond.second.nodeA->GetPos(), abond.second.nodeB->GetPos() - abond.second.nodeA->GetPos(), ChColor(0, 0, 1));
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
