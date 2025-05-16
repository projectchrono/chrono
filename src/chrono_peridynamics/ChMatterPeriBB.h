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

#ifndef CHMATTERPERIBB_H
#define CHMATTERPERIBB_H


#include "chrono_peridynamics/ChApiPeridynamics.h"
#include "chrono_peridynamics/ChMatterPeridynamics.h"
#include "chrono/solver/ChConstraintTwoGenericBoxed.h"

namespace chrono {
namespace peridynamics {

class ChPeridynamics;

/// @addtogroup chrono_peridynamics
/// @{



/// Helper class: the per-bond auxialiary data for ChMatterPeriBB

class  ChApiPeridynamics ChMatterDataPerBondBB : public ChMatterDataPerBond { 
public: 
    bool broken = false;
    double F_density = 0;  // force density per volume squared in this bond
};


/// Simple bond-based peridynamic material whose elasticity depends on a single
/// parameter, that is K, the bulk modulus.  
/// The Poisson ratio is always 1/4 and cannot be set otherwise, as in general for bond-based
/// elasticity models. Having a fixed Poisson ration can be a limitation, but the positive
/// note is that this material is very computationally-efficient.
/// For very high stiffness, time integration might diverge because it does not introduce any tangent 
/// stiffness matrix (even if using implicit integrators like HHT, this will behave like in explicit anyway); 
/// use ChMatterPeriBBimplicit in these cases.

class ChApiPeridynamics ChMatterPeriBB : public ChMatterPeri<ChMatterDataPerNode, ChMatterDataPerBondBB> {
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
    
    /// maximum stretch - after this, bonds will break. Default no break.
    double max_stretch = 1e30;

    ChMatterPeriBB() {};

    /// When doing quadrature, particle volumes that overlap with the horizon should be scaled
    /// proportionally to how much of their volume is really inside the horizon sphere. A simple
    /// and very often used approximation is the following 'fading' function.
    double VolumeCorrection(double dist, double horizon, double vol_size) {
        if (dist < (horizon - vol_size))
            return 1.0;
        else
            if (dist < horizon)
                return 1.0 - (dist- (horizon- vol_size))/(vol_size);
            else
                return 0.0;
    }

    // Implement the function that adds the peridynamics force to each node, as a 
    // summation of all the effects of neighbouring nodes.
    virtual void ComputeForces() {

        // loop on nodes for resetting volume accumulator   //***TODO*** should be faster in SetupInitial - but see notes below
        for (auto& node : this->nodes) {
            node.first->vol_accumulator = node.first->volume;
        }
        // loop on bonds for sum of connected volumes to nodes   //***TODO*** should be faster in SetupInitial - but see notes below
        for (auto& bond : this->bonds) {
            ChMatterDataPerBondBB& mbond = bond.second;
            if (!mbond.broken) {
                mbond.nodeA->vol_accumulator += mbond.nodeB->volume; //*VolumeCorrection(old_sdist, horizon, vol_size); // vol corr. not relevant in Ganzenmueller? 
                mbond.nodeB->vol_accumulator += mbond.nodeA->volume; //*VolumeCorrection(old_sdist, horizon, vol_size); // vol corr. not relevant in Ganzenmueller? 
            }
        }
     
        // loop on bonds
        for (auto& bond : this->bonds) {
            ChMatterDataPerBondBB& mbond = bond.second;
            if (!mbond.broken) {
                ChVector3d old_vdist = mbond.nodeB->GetX0() - mbond.nodeA->GetX0();
                ChVector3d     vdist = mbond.nodeB->GetPos() - mbond.nodeA->GetPos();
                double     old_sdist = old_vdist.Length();
                double         sdist = vdist.Length();
                ChVector3d     vdir = vdist.GetNormalized();
                double         svel = Vdot(vdir, mbond.nodeB->GetPosDt() - mbond.nodeA->GetPosDt());

                double     stretch = (sdist - old_sdist) / old_sdist;

                // Original PMB in Silling
                // double vol_size = mbond.nodeA->GetVolumeSize();
                // double horizon = mbond.nodeA->GetHorizonRadius();
                // double pih4 = chrono::CH_PI * horizon * horizon * horizon * horizon;
                // double force_density_val =  (18.0 * k_bulk / pih4) * stretch; // original in Silling

                // Modified PMB, see [Ganzenmueller et al. "Improvements to the Prototype Micro - Brittle Linear Elasticity Model of Peridynamics"]
                double c_gmuller = 0.5 * ((18.0 * k_bulk / mbond.nodeA->vol_accumulator) + (18.0 * k_bulk / mbond.nodeB->vol_accumulator));
                double force_density_val = (c_gmuller / old_sdist) * stretch;  
                
                if (this->damping > 0)
                    force_density_val += this->damping * (c_gmuller / (old_sdist * old_sdist) ) * svel;

                mbond.F_density = force_density_val;
                double wVV_ji = mbond.nodeA->volume * mbond.nodeB->volume;// *VolumeCorrection(old_sdist, horizon, vol_size); // vol corr. not relevant in Ganzenmueller?

                mbond.nodeB->F_peridyn += -vdir * force_density_val * wVV_ji;
                mbond.nodeA->F_peridyn += vdir * force_density_val * wVV_ji;

                if (stretch > max_stretch) {
                    mbond.F_density = 0;
                    mbond.broken = true;
                    // the following will propagate the fracture geometry so that broken parts can collide
                    mbond.nodeA->is_boundary = true; 
                    mbond.nodeB->is_boundary = true;
                }
            }
            else {
                bonds.erase(bond.first);
            }

        }
    }

    void SetupInitial() override {
        // Not here, still in Computeforces() at each step because it could be necessary to update in runtime node->vol_accumulator if bonds are deleted/fractured/..
        // TODO: still can be put here, and in ComputeForces just subtract from accumulator as 
        //    mbond.nodeA->vol_accumulator -= mbond.nodeB->volume; 
        //    mbond.nodeB->vol_accumulator -= mbond.nodeA->volume;
        // when the bond is erased in  bonds.erase(bond.first);  ? To be tested!
        /*
        // see [Ganzenmueller et al. "Improvements to the Prototype Micro - Brittle Linear Elasticity Model of Peridynamics"]
        // loop on nodes for resetting volume accumulator   // note! could be necessary to update in runtime if bonds are deleted/fractured/..
        for (auto& node : this->nodes) {
            node.first->vol_accumulator = node.first->volume;
        }
        // loop on bonds for sum of connected volumes to nodes   // note! could be necessary to update in runtime if bonds are deleted/fractured/..
        for (auto& bond : this->bonds) {
            ChMatterDataPerBondBB& mbond = bond.second;
            if (!mbond.broken) {
                mbond.nodeA->vol_accumulator += mbond.nodeB->volume; //*VolumeCorrection(old_sdist, horizon, vol_size); // vol corr. not relevant in Ganzenmueller? 
                mbond.nodeB->vol_accumulator += mbond.nodeA->volume; //*VolumeCorrection(old_sdist, horizon, vol_size); // vol corr. not relevant in Ganzenmueller? 
            }
        }
        */
    }
};




/// Class for visualization of ChMatterPeriBB  nodes
/// This can be attached to ChPeridynamics with my_peridynamics->AddVisualShape(my_visual);

class /*ChApiPeridynamics*/ ChVisualPeriBB : public ChGlyphs {
public:
    ChVisualPeriBB(std::shared_ptr<ChMatterPeriBB> amatter) : mmatter(amatter) { is_mutable = true; };
    virtual ~ChVisualPeriBB() {}

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
            ++i;
        }

    }

    std::shared_ptr<ChMatterPeriBB> mmatter;
};



/// Class for visualization of ChMatterPeriBB  bonds
/// This can be attached to ChPeridynamics with my_peridynamics->AddVisualShape(my_visual);

class /*ChApiPeridynamics*/ ChVisualPeriBBBonds : public ChGlyphs {
public:
    ChVisualPeriBBBonds(std::shared_ptr<ChMatterPeriBB> amatter) : mmatter(amatter) { is_mutable = true; };
    virtual ~ChVisualPeriBBBonds() {}

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

    std::shared_ptr<ChMatterPeriBB> mmatter;
};









/// @} chrono_peridynamics

}  // end namespace peridynamics
}  // end namespace chrono

#endif
