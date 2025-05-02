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

#ifndef CHMATTERPERISPRINGS_H
#define CHMATTERPERISPRINGS_H


#include "chrono_peridynamics/ChApiPeridynamics.h"
#include "chrono_peridynamics/ChMatterPeridynamics.h"


namespace chrono {
namespace peridynamics {

class ChPeridynamics;

/// @addtogroup chrono_peridynamics
/// @{


//------------------------------------------------------------------------------------
// ChMatterPeriSprings    -  for didactical purpose


/// The simplest peridynamic material: a bond-based material based on a 
/// network of springs, each with the same stiffness k regardless of length, etc. 
/// Just for didactical purposes - do not use it for serious applications.
/// Also use a damping coefficient r. 

class ChApiPeridynamics ChMatterPeriSprings : public ChMatterPeri<> {
public:
    double k = 100;
    double r = 10;

    ChMatterPeriSprings() {};

    // Implement the function that adds the peridynamics force to each node, as a 
    // summation of all the effects of neighbouring nodes.
    virtual void ComputeForces() {
        // loop on bonds
        for (auto& bond : this->bonds) {
            ChMatterDataPerBond& mbond = bond.second;
            ChVector3d old_vdist = mbond.nodeB->GetX0() - mbond.nodeA->GetX0();
            ChVector3d     vdist = mbond.nodeB->GetPos() - mbond.nodeA->GetPos();
            ChVector3d     vdir = vdist.GetNormalized();
            double         vel = Vdot(vdir, mbond.nodeB->GetPosDt() - mbond.nodeA->GetPosDt());
            ChVector3d force_val = (vdist.Length() - old_vdist.Length()) * this->k  + vel * this->r;
            mbond.nodeB->F_peridyn += -vdir * force_val;
            mbond.nodeA->F_peridyn += vdir * force_val;
        }
    };
};



//------------------------------------------------------------------------------------
// ChMatterPeriSpringsBreakable   -  for didactical purpose



class  ChApiPeridynamics ChMatterDataPerBondBreakable : public ChMatterDataPerBond { 
public: 
    bool broken = false;
};

class ChApiPeridynamics ChMatterPeriSpringsBreakable : public ChMatterPeri<ChMatterDataPerNode, ChMatterDataPerBondBreakable> {
public:
    double k = 100;
    double r = 10;
    double max_stretch = 0.08;

    ChMatterPeriSpringsBreakable() {};

    // Implement the function that adds the peridynamics force to each node, as a 
    // summation of all the effects of neighbouring nodes.
    virtual void ComputeForces() {
        // loop on bonds
        for (auto& bond : this->bonds) {
            ChMatterDataPerBondBreakable& mbond = bond.second;
            if (!mbond.broken) {
                ChVector3d old_vdist = mbond.nodeB->GetX0() - mbond.nodeA->GetX0();
                ChVector3d     vdist = mbond.nodeB->GetPos() - mbond.nodeA->GetPos();
                ChVector3d     vdir = vdist.GetNormalized();
                double         vel = Vdot(vdir, mbond.nodeB->GetPosDt() - mbond.nodeA->GetPosDt());
                ChVector3d force_val = (vdist.Length() - old_vdist.Length()) * this->k + vel * this->r;
                mbond.nodeB->F_peridyn += -vdir * force_val / mbond.nodeB->volume; //divide by volumes because F_peridyn are force _densities_
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
            }
            else {
                if (mbond.broken)
                    bonds.erase(bond.first);
            }

        }
    }
};



class /*ChApiPeridynamics*/ ChVisualPeriSpringsBreakable : public ChGlyphs {
public:
    ChVisualPeriSpringsBreakable(std::shared_ptr<ChMatterPeriSpringsBreakable> amatter) : mmatter(amatter) { is_mutable = true; };
    virtual ~ChVisualPeriSpringsBreakable() {}

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

    std::shared_ptr<ChMatterPeriSpringsBreakable> mmatter;
};


class /*ChApiPeridynamics*/ ChVisualPeriSpringsBreakableBonds : public ChGlyphs {
public:
    ChVisualPeriSpringsBreakableBonds(std::shared_ptr<ChMatterPeriSpringsBreakable> amatter) : mmatter(amatter) { is_mutable = true; };
    virtual ~ChVisualPeriSpringsBreakableBonds() {}

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
        for (const auto& anode : mmatter->GetMapOfBonds()) {
            if (anode.second.broken && draw_broken) {
                this->SetGlyphVector(i, anode.second.nodeA->GetPos(), anode.second.nodeB->GetPos() - anode.second.nodeA->GetPos(), ChColor(1, 0, 0));
                ++i;
            }
            if (!anode.second.broken && draw_unbroken) {
                this->SetGlyphVector(i, anode.second.nodeA->GetPos(), anode.second.nodeB->GetPos() - anode.second.nodeA->GetPos(), ChColor(0, 0, 1));
                ++i;
            }
        }
    }

    std::shared_ptr<ChMatterPeriSpringsBreakable> mmatter;
};


/// @} chrono_peridynamics

}  // end namespace peridynamics
}  // end namespace chrono

#endif
