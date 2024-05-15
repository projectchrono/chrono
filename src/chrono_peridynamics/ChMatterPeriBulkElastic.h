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


namespace chrono {
namespace peridynamics {

class ChPeridynamics;

/// @addtogroup chrono_peridynamics
/// @{



/// Helper class: the per-bond auxialiary data for ChMatterPeriBulkElastic

class  ChApiPeridynamics ChMatterDataPerBoundBulk : public ChMatterDataPerBound { 
public: 
    bool broken = false;
    double F_per_bond = 0;  // force density in this bound
};


/// Simple bond-based peridynamic material whose elasticity depends on a single
/// parameter, that is K, the bulk modulus.  
/// The Poisson ratio is always 1/4 and cannot be set otherwise, as in general for bond-based
/// elasticity models. Having a fixed Poission ration can be a limitation, but the positive
/// note is that this material is very computationally-efficient.

class ChApiPeridynamics ChMatterPeriBulkElastic : public ChMatterPeri<ChMatterDataPerNode, ChMatterDataPerBoundBulk> {
public:
    /// bulk modulus, unit  Pa, i.e. N/m^2
    double k_bulk = 100; 

    /// bulk damping, unit  Pa*s/m, i.e. Ns/m^3
    double r_bulk = 10;
    
    /// maximum stretch - after this, bonds will break
    double max_stretch = 0.08;

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

                double force_val = 0.5 * (18.0 * k_bulk / pih4) * stretch;
                
                if (this->r_bulk > 0)
                    force_val += 0.5 * (18.0 * r_bulk / pih4) * svel;

                mbound.F_per_bond = force_val;
                mbound.nodeB->F_peridyn += -vdir * force_val * mbound.nodeA->volume;
                mbound.nodeA->F_peridyn += vdir * force_val * mbound.nodeB->volume;

                if (stretch > max_stretch) {
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
    ChPropertyVector* vel_property;
    ChPropertyVector* acc_property;

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
            if (abound.second.broken && draw_broken)
                this->SetGlyphVector(i, abound.second.nodeA->GetPos(), abound.second.nodeB->GetPos() - abound.second.nodeA->GetPos(),ChColor(1,0,0));
            if (!abound.second.broken && draw_unbroken)
                this->SetGlyphVector(i, abound.second.nodeA->GetPos(), abound.second.nodeB->GetPos() - abound.second.nodeA->GetPos(),ChColor(0, 0, 1));
            ++i;
        }
    }

    std::shared_ptr<ChMatterPeriBulkElastic> mmatter;
};


/// @} chrono_peridynamics

}  // end namespace peridynamics
}  // end namespace chrono

#endif
