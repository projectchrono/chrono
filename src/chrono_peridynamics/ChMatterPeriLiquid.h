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

#ifndef CHMATTERPERILIQUID_H
#define CHMATTERPERILIQUID_H

#include "chrono_peridynamics/ChApiPeridynamics.h"
#include "chrono_peridynamics/ChMatterPeridynamics.h"

namespace chrono {
namespace peridynamics {

class ChPeridynamics;

/// @addtogroup chrono_peridynamics
/// @{

static double W_poly6(double r, double h) {
    if (r < h) {
        return (315.0 / (64.0 * CH_PI * pow(h, 9))) * pow((h * h - r * r), 3);
    } else
        return 0;
}

static double W_sq_visco(double r, double h) {
    if (r < h) {
        return (45.0 / (CH_PI * pow(h, 6))) * (h - r);
    } else
        return 0;
}

static void W_gr_press(ChVector3d& Wresult, const ChVector3d& r, const double r_length, const double h) {
    if (r_length < h) {
        Wresult = r;
        Wresult *= -(45.0 / (CH_PI * pow(h, 6))) * pow((h - r_length), 2.0);
    } else
        Wresult = VNULL;
}

/// Helper class: the per-node auxiliary data for ChMatterPeriLiquid

class ChApiPeridynamics ChMatterDataLiquid : public ChMatterDataPerNode {
  public:
    double density = 0;  // node density
    double pressure = 0;
};

/// Simple SPH-like material, with viscosity and compressibility.
/// This material can interact with collisioon shapes and other peridynamic materials.
/// TODO: NOT READY. TO BE IMPROVED / REWRITTEN
class ChApiPeridynamics ChMatterPeriLiquid : public ChMatterPeri<ChMatterDataLiquid, ChMatterDataPerBond> {
  public:
    /// fluid viscosity
    double viscosity = 0.01;

    /// pressure stiffness
    double pressure_stiffness = 100;

    /// material density
    double density = 1000;

    ChMatterPeriLiquid() {}

    /// Add the peridynamics force to each node, as a summation of all the effects of neighbouring nodes.
    virtual void ComputeForces() override;
};

// -----------------------------------------------------------------------------

/// Class for visualization of ChMatterPeriLiquid  nodes
/// This can be attached to ChPeridynamics with my_peridynamics->AddVisualShape(my_visual);
class /*ChApiPeridynamics*/ ChVisualPeriLiquid : public ChGlyphs {
  public:
    ChVisualPeriLiquid(std::shared_ptr<ChMatterPeriLiquid> amatter) : mmatter(amatter) { is_mutable = true; };
    virtual ~ChVisualPeriLiquid() {}

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

    // Attach density property. (ex for postprocessing in falsecolor or with vectors with the Blender add-on)
    void AttachDensity(double min = 0, double max = 1000, std::string mname = "Density") {
        density_property = new ChPropertyScalar;
        density_property->min = min;
        acc_property->max = max;
        acc_property->name = mname;
        this->m_properties.push_back(density_property);
    }

  protected:
    ChPropertyVector* vel_property = 0;
    ChPropertyVector* acc_property = 0;
    ChPropertyScalar* density_property = 0;

    virtual void Update(ChObj* updater, const ChFrame<>& frame) override {
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
            if (density_property)
                density_property->data[i] = anode.second.density;
            ++i;
        }
    }

    std::shared_ptr<ChMatterPeriLiquid> mmatter;
};

/// @} chrono_peridynamics

}  // end namespace peridynamics
}  // end namespace chrono

#endif
