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
class ChApiPeridynamics ChMatterDataPerBondBBimplicit : public ChMatterDataPerBond {
  public:
    enum class bond_state {
        ACTIVE,     ///< Regular bond, push-pull
        FRACTURED,  ///< Fractured - with small dislocation, collision via box constraint
        BROKEN      ///< Broken - far apart, full collision surfaces generated
    };

    bond_state state = bond_state::ACTIVE;
    double force_density_val = 0;  ///< force density per vol squared in this bond, for postprocessing
    double d_zeta;                 ///< elongation   (d-zeta)= (sdist - old_sdist),  residual for implicit form
    double Km;                     ///< tangent stiffness matrix, for implicit form

    ChConstraintTwoGenericBoxed constraint;
    // ChConstraintTwoGeneric constraint;

    virtual void Initialize(ChNodePeri* mA, ChNodePeri* mB) override;
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
class ChApiPeridynamics ChMatterPeriBBimplicit
    : public ChMatterPeri<ChMatterDataPerNode, ChMatterDataPerBondBBimplicit> {
  public:
    /// Set the material Young modulus. The unique bulk modulus will be automatically computed,
    /// since in this material Poisson is always =1/4
    void SetYoungModulus(double mE);

    /// bulk modulus, unit  Pa, i.e. N/m^2
    double k_bulk = 100;

    /// damping, as Rayleigh beta (bulk stiffness-proportional)
    double damping = 0.001;

    /// maximum stretch for fracure - after this, bonds will become unilateral. Default no break.
    double max_stretch_fracture = 1e30;

    /// maximum stretch for full breaking - after this, bonds will break. Default no break.
    double max_stretch_break = 1e30;

    ChMatterPeriBBimplicit() {}

    /// When doing quadrature, particle volumes that overlap with the horizon should be scaled
    /// proportionally to how much of their volume is really inside the horizon sphere. A simple
    /// and very often used approximation is the following 'fading' function.
    double VolumeCorrection(double dist, double horizon, double vol_size);

    // Implement the function that adds the peridynamics force to each node, as a
    // summation of all the effects of neighbouring nodes.
    virtual void ComputeForces() override;

    virtual unsigned int GetNumConstraints() override { return (unsigned int)bonds.size(); }

    virtual void Setup() override;

    virtual void IntLoadResidual_CqL(const unsigned int off_L,    ///< offset in L multipliers
                                     ChVectorDynamic<>& R,        ///< result: the R residual, R += c*Cq'*L
                                     const ChVectorDynamic<>& L,  ///< the L vector
                                     const double c               ///< a scaling factor
                                     ) override;

    /// Takes the term C, scale and adds to Qc at given offset:
    ///    Qc += c*C
    virtual void IntLoadConstraint_C(const unsigned int off,  ///< offset in Qc residual
                                     ChVectorDynamic<>& Qc,   ///< result: the Qc residual, Qc += c*C
                                     const double c,          ///< a scaling factor
                                     bool do_clamp,           ///< apply clamping to c*C?
                                     double recovery_clamp    ///< value for min/max clamping of c*C
                                     ) override;

    /// Register with the given system descriptor any ChConstraint objects associated with this item.
    virtual void InjectConstraints(ChSystemDescriptor& descriptor) override;

    /// Compute and load current Jacobians in encapsulated ChConstraint objects.
    virtual void LoadConstraintJacobians() override;

    virtual void IntToDescriptor(const unsigned int off_v,
                                 const ChStateDelta& v,
                                 const ChVectorDynamic<>& R,
                                 const unsigned int off_L,
                                 const ChVectorDynamic<>& L,
                                 const ChVectorDynamic<>& Qc) override;

    virtual void IntFromDescriptor(const unsigned int off_v,
                                   ChStateDelta& v,
                                   const unsigned int off_L,
                                   ChVectorDynamic<>& L) override;
};

// -----------------------------------------------------------------------------

/// Class for visualization of ChMatterPeriBBimplicit  nodes
/// This can be attached to ChPeridynamics with my_peridynamics->AddVisualShape(my_visual);
class /*ChApiPeridynamics*/ ChVisualPeriBBimplicit : public ChGlyphs {
  public:
    ChVisualPeriBBimplicit(std::shared_ptr<ChMatterPeriBBimplicit> amatter) : mmatter(amatter) { SetMutable(true); }
    virtual ~ChVisualPeriBBimplicit() {}

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
                (!anode.first->is_colliding && this->draw_noncolliding)) {
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
    ChVisualPeriBBimplicitBonds(std::shared_ptr<ChMatterPeriBBimplicit> amatter) : mmatter(amatter) {
        SetMutable(true);
    }
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
                this->SetGlyphVector(i, abond.second.nodeA->GetPos(),
                                     abond.second.nodeB->GetPos() - abond.second.nodeA->GetPos(),
                                     ChColor(0.15f, 0.84f, 1.f));
                ++i;
            }
            if ((abond.second.state == ChMatterDataPerBondBBimplicit::bond_state::BROKEN) && draw_broken) {
                this->SetGlyphVector(i, abond.second.nodeA->GetPos(),
                                     abond.second.nodeB->GetPos() - abond.second.nodeA->GetPos(),
                                     ChColor(1.f, 0.f, 0.f));
                ++i;
            }
            if ((abond.second.state == ChMatterDataPerBondBBimplicit::bond_state::FRACTURED) && draw_fractured) {
                this->SetGlyphVector(i, abond.second.nodeA->GetPos(),
                                     abond.second.nodeB->GetPos() - abond.second.nodeA->GetPos(),
                                     ChColor(1.f, 0.5f, 0.f));
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
