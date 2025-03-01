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

#ifndef CHSHAFTSGEARBOX_H
#define CHSHAFTSGEARBOX_H

#include "chrono/physics/ChBodyFrame.h"
#include "chrono/physics/ChPhysicsItem.h"
#include "chrono/physics/ChShaft.h"
#include "chrono/solver/ChConstraintThreeGeneric.h"

namespace chrono {

// Forward references (for parent hierarchy pointer)
class ChShaft;
class ChBodyFrame;

/// Class for defining a gearbox.
/// It defines a transmission ratio between two 1D entities of ChShaft type, and transmits the reaction of the gearbox
/// to a 3D body that acts as the support truss. Note that the more basic ChShaftsGear can do the same, except that it
/// does not provide a way to transmit reaction to a truss body. Also note that this can also be seen as a
/// ChShaftPlanetary where one has joined the carrier shaft to a fixed body via a ChShaftBodyRotation constraint.
class ChApi ChShaftsGearbox : public ChPhysicsItem {
  public:
    ChShaftsGearbox();
    ChShaftsGearbox(const ChShaftsGearbox& other);
    ~ChShaftsGearbox() {}

    /// "Virtual" copy constructor (covariant return type).
    virtual ChShaftsGearbox* Clone() const override { return new ChShaftsGearbox(*this); }

    /// Get the first shaft (carrier wheel).
    ChShaft* GetShaft1() const { return shaft1; }

    /// Get the second shaft.
    ChShaft* GetShaft2() const { return shaft2; }

    /// Get the third shaft.
    ChBodyFrame* GetBodyTruss() const { return body; }

    /// Set the transmission ratio t, as in w2=t*w1, or t=w2/w1 , or  t*w1 - w2 = 0.
    /// For example, t=1 for a rigid joint; t=-0.5 for representing
    /// a couple of spur gears with teeth z1=20 & z2=40; t=0.1 for
    /// a gear with inner teeth (or epicycloidal reducer), etc.
    /// Also the t0 ordinary equivalent ratio of the inverted planetary,
    /// if the 3D body is rotating as in planetary gears.
    void SetTransmissionRatio(double t0);

    /// Get the transmission ratio t, as in w2=t*w1, or t=w2/w1.
    /// Also the t0 ordinary equivalent ratio of the inverted planetary,
    /// if the 3D body is rotating as in planetary gears.
    double GetTransmissionRatio() const { return -r1 / r2; }

    /// Set the direction of the shaft with respect to 3D body, as a
    /// normalized vector expressed in the coordinates of the body.
    /// The shaft applies only torque, about this axis.
    void SetShaftDirection(ChVector3d md) { shaft_dir = Vnorm(md); }

    /// Get the direction of the shaft with respect to 3D body, as a
    /// normalized vector expressed in the coordinates of the body.
    const ChVector3d& GetShaftDirection() const { return shaft_dir; }

    /// Get the reaction torque considered as applied to the 1st axis.
    double GetReaction1() const { return (r1 * torque_react); }

    /// Get the reaction torque considered as applied to the 2nd axis.
    double GetReaction2() const { return (r2 * torque_react); }

    /// Get the reaction torque considered as applied to the body
    /// (the truss of the gearbox), expressed in the coordinates of the body.
    ChVector3d GetTorqueReactionOnBody() const { return (shaft_dir * (r3 * torque_react)); }

    /// Initialize this gearbox, given two shafts to join, and the 3D truss body.
    /// The truss receives the reaction torque of the gearbox.
    /// Direction is expressed in the local coordinates of the body.
    /// The shaft and body must belong to the same ChSystem.
    bool Initialize(
        std::shared_ptr<ChShaft> shaft_1,    ///< first (input) shaft to join
        std::shared_ptr<ChShaft> shaft_2,    ///< second  (output) shaft to join
        std::shared_ptr<ChBodyFrame> truss,  ///< truss body (also carrier, if rotating as in planetary gearboxes)
        ChVector3d& dir                      ///< the direction of the shaft on 3D body (applied on COM: pure torque)
    );

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOut(ChArchiveOut& archive_out) override;

    /// Method to allow deserialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& archive_in) override;

  private:
    double r1;  ///< transmission ratios  as in   r1*w1 + r2*w2 + r3*w3 = 0
    double r2;
    double r3;

    double torque_react;  ///< reaction torque

    ChConstraintThreeGeneric constraint;  ///< used as an interface to the solver

    ChShaft* shaft1;    ///< first shaft
    ChShaft* shaft2;    ///< second shaft
    ChBodyFrame* body;  ///< support truss

    ChVector3d shaft_dir;  ///< shaft direction

    /// Get the number of scalar variables affected by constraints in this link
    virtual unsigned int GetNumAffectedCoords() const { return 6 + 1 + 1; }

    /// Number of scalar constraints
    virtual unsigned int GetNumConstraintsBilateral() override { return 1; }

    virtual void Update(double time, bool update_assets) override;

    virtual void IntStateGatherReactions(const unsigned int off_L, ChVectorDynamic<>& L) override;
    virtual void IntStateScatterReactions(const unsigned int off_L, const ChVectorDynamic<>& L) override;
    virtual void IntLoadResidual_CqL(const unsigned int off_L,
                                     ChVectorDynamic<>& R,
                                     const ChVectorDynamic<>& L,
                                     const double c) override;
    virtual void IntLoadConstraint_C(const unsigned int off,
                                     ChVectorDynamic<>& Qc,
                                     const double c,
                                     bool do_clamp,
                                     double recovery_clamp) override;
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

    virtual void InjectConstraints(ChSystemDescriptor& descriptor) override;
    virtual void LoadConstraintJacobians() override;

    virtual void ConstraintsBiReset() override;
    virtual void ConstraintsBiLoad_C(double factor = 1, double recovery_clamp = 0.1, bool do_clamp = false) override;
    virtual void ConstraintsFetch_react(double factor = 1) override;
};

CH_CLASS_VERSION(ChShaftsGearbox, 0)

}  // end namespace chrono

#endif
