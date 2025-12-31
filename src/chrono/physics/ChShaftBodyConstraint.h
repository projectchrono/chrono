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

#ifndef CH_SHAFT_BODY_CONSTRAINT_H
#define CH_SHAFT_BODY_CONSTRAINT_H

#include "chrono/physics/ChBodyFrame.h"
#include "chrono/physics/ChShaft.h"
#include "chrono/solver/ChConstraintTwoGeneric.h"

namespace chrono {

// Forward references (for parent hierarchy pointer)
class ChShaft;
class ChBodyFrame;

/// Constraint between a 3D ChBody object and a ChShaft object that represents a 1D rotational DOF.
/// A rotation axis must be specified (to tell along which direction the shaft inertia and rotation affects the body).
/// This constraint is useful, for example, when modeling a vehicle using ChBody items and a 1D powertrain (gears,
/// differential, etc.) using ChShaft objects: wheel bodies can be connected to the driveline using this constraint.
class ChApi ChShaftBodyRotation : public ChPhysicsItem {
  public:
    ChShaftBodyRotation();
    ChShaftBodyRotation(const ChShaftBodyRotation& other);
    ~ChShaftBodyRotation() {}

    /// "Virtual" copy constructor (covariant return type).
    virtual ChShaftBodyRotation* Clone() const override { return new ChShaftBodyRotation(*this); }

    /// Initialize the constraint, given the 1D shaft and 3D body to join.
    /// Direction is expressed in the local coordinates of the body.
    /// Both items must belong to the same ChSystem.
    bool Initialize(std::shared_ptr<ChShaft> mshaft,     ///< shaft to join
                    std::shared_ptr<ChBodyFrame> mbody,  ///< body to join
                    const ChVector3d& mdir               ///< direction of the shaft on 3D body
    );

    /// Get the shaft.
    ChShaft* GetShaft() { return shaft; }

    /// Get the body.
    ChBodyFrame* GetBody() { return body; }

    /// Set the direction of the shaft respect to 3D body, as a normalized vector expressed in the coordinates of the
    /// body. The shaft applies only torque, about this axis.
    void SetShaftDirection(ChVector3d md) { shaft_dir = Vnorm(md); }

    /// Get the direction of the shaft respect to 3D body, as a normalized vector expressed in the coordinates of the
    /// body.
    const ChVector3d& GetShaftDirection() const { return shaft_dir; }

    /// Get the reaction torque considered as applied to ChShaft.
    double GetTorqueReactionOnShaft() const { return -(torque_react); }

    /// Get the reaction torque considered as applied to ChBody, expressed in the coordinates of the body.
    ChVector3d GetTorqueReactionOnBody() const { return (shaft_dir * torque_react); }

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOut(ChArchiveOut& archive_out) override;

    /// Method to allow deserialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& archive_in) override;

  private:
    double torque_react;                ///< reaction torque
    ChConstraintTwoGeneric constraint;  ///< used as an interface to the solver
    ChShaft* shaft;                     ///< connected shaft
    ChBodyFrame* body;                  ///< connected body
    ChVector3d shaft_dir;               ///< shaft direction

    /// Get the number of scalar variables affected by constraints in this link.
    virtual unsigned int GetNumAffectedCoords() const { return 6 + 1; }

    /// Number of scalar constraints.
    virtual unsigned int GetNumConstraintsBilateral() override { return 1; }

    /// Update all auxiliary data of the gear transmission at given time.
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
    virtual void IntLoadConstraint_Ct(const unsigned int off, ChVectorDynamic<>& Qc, const double c) override {}
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
    virtual void ConstraintsBiLoad_Ct(double factor = 1) override;
    virtual void ConstraintsFetch_react(double factor = 1) override;

    friend class ChLinkMotorRotationDriveline;
    friend class ChLinkMotorLinearDriveline;
};

CH_CLASS_VERSION(ChShaftBodyRotation, 0)

/// Constraint between a 3D ChBody object and a ChShaft object that represents a 1D translational DOF.
/// Note that this is different from the ChShaftBodyRotation constraint  which connects to a rotational DOF.
/// A translation axis must be specified (to tell along which direction the 1D shaft inertia rotation affects the body).
class ChApi ChShaftBodyTranslation : public ChPhysicsItem {
  public:
    ChShaftBodyTranslation();
    ChShaftBodyTranslation(const ChShaftBodyTranslation& other);
    ~ChShaftBodyTranslation() {}

    /// "Virtual" copy constructor (covariant return type).
    virtual ChShaftBodyTranslation* Clone() const override { return new ChShaftBodyTranslation(*this); }

    /// Initialize the constraint, given the 1D shaft and 3D body to join.
    ///  Direction is expressed in the local coordinates of the body.
    ///  Both items must belong to the same ChSystem.
    bool Initialize(std::shared_ptr<ChShaft> mshaft,     ///< shaft to join, representing translational dof
                    std::shared_ptr<ChBodyFrame> mbody,  ///< body to join
                    const ChVector3d& mdir,              ///< the direction of the shaft on 3D body, in body coords
                    const ChVector3d& mpos  ///< the anchro position of the shaft on 3D body, in body coords
    );

    /// Get the shaft.
    ChShaft* GetShaft() { return shaft; }

    /// Get the body.
    ChBodyFrame* GetBody() { return body; }

    /// Set the direction of the shaft respect to 3D body, as a normalized vector expressed in the coordinates of the
    /// body.
    void SetShaftDirection(ChVector3d md) { shaft_dir = Vnorm(md); }

    /// Get the direction of the shaft respect to 3D body, as a normalized vector expressed in the coordinates of the
    /// body.
    const ChVector3d& GetShaftDirection() const { return shaft_dir; }

    /// Set the anchor point of the shaft respect to 3D body, as a vector expressed in the coordinates of the body.
    void SetShaftPos(ChVector3d md) { shaft_pos = md; }

    /// Get the anchor point of the shaft respect to 3D body, as a vector expressed in the coordinates of the body.
    const ChVector3d& GetShaftPos() const { return shaft_pos; }

    /// Get the reaction force considered as applied to ChShaft.
    double GetForceReactionOnShaft() const { return -(force_react); }

    /// Get the reaction torque considered as applied to ChBody, expressed in the coordinates of the body.
    ChVector3d GetForceReactionOnBody() const { return (shaft_dir * force_react); }

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOut(ChArchiveOut& archive_out) override;

    /// Method to allow deserialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& archive_in) override;

  private:
    double force_react;                 ///< reaction force
    ChConstraintTwoGeneric constraint;  ///< used as an interface to the solver
    ChShaft* shaft;                     ///< connected shaft (translation dof)
    ChBodyFrame* body;                  ///< connected body
    ChVector3d shaft_dir;               ///< shaft direction
    ChVector3d shaft_pos;               ///< shaft anchor to body

    /// Get the number of scalar variables affected by constraints in this link.
    virtual unsigned int GetNumAffectedCoords() const { return 6 + 1; }

    /// Number of scalar constraints.
    virtual unsigned int GetNumConstraintsBilateral() override { return 1; }

    /// Update all auxiliary data of the gear transmission at given time.
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
    virtual void IntLoadConstraint_Ct(const unsigned int off, ChVectorDynamic<>& Qc, const double c) override {}
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

    virtual void InjectConstraints(ChSystemDescriptor& mdescriptor) override;
    virtual void ConstraintsBiReset() override;
    virtual void ConstraintsBiLoad_C(double factor = 1, double recovery_clamp = 0.1, bool do_clamp = false) override;
    virtual void ConstraintsBiLoad_Ct(double factor = 1) override;
    virtual void LoadConstraintJacobians() override;
    virtual void ConstraintsFetch_react(double factor = 1) override;

    friend class ChLinkMotorRotationDriveline;
    friend class ChLinkMotorLinearDriveline;
};

CH_CLASS_VERSION(ChShaftBodyTranslation, 0)

}  // end namespace chrono

#endif
