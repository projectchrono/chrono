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
class ChApiPeridynamics ChMatterDataPerNodeLinearElastic : public ChMatterDataPerNode {
  public:
    double m = 0;      ///< weighted volume
    double theta = 0;  ///< dilation
};

/// Helper class: the per-bond auxiliary data for ChMatterDataPerBondLinearElastic

class ChApiPeridynamics ChMatterDataPerBondLinearElastic : public ChMatterDataPerBond {
  public:
    bool broken = false;    ///< is broken?
    double F_per_bond = 0;  ///< force density in this bond
};

/// Simple state-based peridynamic material whose elasticity depends on two parameters.
/// Example: a 3D linear elastic Hookean material, that is:
///  - K, the bulk modulus.
///  - G, the shear modulus.
/// This comes at a cost of slower performance compared to the ChMatterPeriBB bond-based elasticity model, that has
/// Poisson fixed to 1/4. For very high stiffness, time integration might diverge because it does not introduce any
/// tangent stiffness matrix (even if using implicit integrators like HHT, this will behave like in explicit anyway);
/// use ChMatterPeriBBimplicit in these cases.
class ChApiPeridynamics ChMatterPeriLinearElastic
    : public ChMatterPeri<ChMatterDataPerNodeLinearElastic, ChMatterDataPerBondLinearElastic> {
  public:
    void SetYoungModulusShearModulus(double mE, double mG);
    void SetYoungModulusPoisson(double mE, double mu);

    /// bulk modulus, unit  Pa, i.e. N/m^2
    double k_bulk = 100;

    /// shear modulus
    double G = 100;

    /// bulk damping, unit  Pa*s/m, i.e. Ns/m^3 ***NOT USED***
    double r_bulk = 0;

    /// maximum stretch - after this, bonds will break. Default no break.
    double max_stretch = 1e30;

    ChMatterPeriLinearElastic() {
        assert(false);  // Material not ready to use. TO DO
    };

    double InfluenceFunction(double zeta, double horizon);

    /// Initialize material with weighted volume.
    virtual void SetupInitial() override;

    /// Adds the peridynamics force to each node, as a summation of all the effects of neighbouring nodes.
    /// Formulas based on Silling work
    virtual void ComputeForces() override;
};

// -----------------------------------------------------------------------------

/// Class for visualization of ChMatterDataPerBondLinearElastic nodes
/// This can be attached to ChPeridynamics with my_peridynamics->AddVisualShape(my_visual);
class /*ChApiPeridynamics*/ ChVisualPeriLinearElastic : public ChGlyphs {
  public:
    ChVisualPeriLinearElastic(std::shared_ptr<ChMatterPeriLinearElastic> amatter) : mmatter(amatter) {
        SetMutable(true);
    }

    virtual ~ChVisualPeriLinearElastic() {}

    // Attach velocity property. (ex for postprocessing in falsecolor or with vectors with the Blender add-on)
    void AttachVelocity(double min = 0, double max = 1, std::string mname = "Velocity") {
        vel_property = new ChPropertyVector;
        vel_property->min = min;
        vel_property->max = max;
        vel_property->name = mname;
        this->m_properties.push_back(vel_property);
    }

    // Attach acceleration property. (ex for postprocessing in falsecolor or with vectors with the Blender add-on)
    void AttachAcceleration(double min = 0, double max = 1, std::string mname = "Acceleration") {
        acc_property = new ChPropertyVector;
        acc_property->min = min;
        acc_property->max = max;
        acc_property->name = mname;
        this->m_properties.push_back(acc_property);
    }

    // Attach dilation (theta) property. (ex for postprocessing in falsecolor or with vectors with the Blender add-on)
    void AttachDilation(double min = 0, double max = 1, std::string mname = "Dilation") {
        theta_property = new ChPropertyScalar;
        theta_property->min = min;
        theta_property->max = max;
        theta_property->name = mname;
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
                theta_property->data[i] = mmatter->GetMapOfNodes().at(anode.first).theta;
            ++i;
        }
    }

    std::shared_ptr<ChMatterPeriLinearElastic> mmatter;
};

/// Class for visualization of ChMatterPeriLinearElastic bonds
/// This can be attached to ChPeridynamics with my_peridynamics->AddVisualShape(my_visual);
class /*ChApiPeridynamics*/ ChVisualPeriLinearElasticBonds : public ChGlyphs {
  public:
    ChVisualPeriLinearElasticBonds(std::shared_ptr<ChMatterPeriLinearElastic> amatter) : mmatter(amatter) {
        SetMutable(true);
    }
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
                this->SetGlyphVector(i, abond.second.nodeA->GetPos(),
                                     abond.second.nodeB->GetPos() - abond.second.nodeA->GetPos(), ChColor(1, 0, 0));
                ++i;
            }
            if (!abond.second.broken && draw_unbroken) {
                this->SetGlyphVector(i, abond.second.nodeA->GetPos(),
                                     abond.second.nodeB->GetPos() - abond.second.nodeA->GetPos(), ChColor(0, 0, 1));
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
