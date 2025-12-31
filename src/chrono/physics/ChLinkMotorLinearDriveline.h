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

#ifndef CHLINKMOTORLINEARDRIVELINE_H
#define CHLINKMOTORLINEARDRIVELINE_H

#include "chrono/physics/ChLinkMotorLinear.h"
#include "chrono/physics/ChShaft.h"
#include "chrono/physics/ChShaftBodyConstraint.h"

namespace chrono {

/// Couples the relative translation of two bodies (along Z direction of the link frames) with the rotation of a 1D
/// shaft.
///
/// This link adds three additional ChShaft objects:
/// - two of them on Body 2, representing a rotational (Shaft2Rot) and a translational (Shfat2Lin) inertia.
/// - one ChShaft object to Body 1, representing a translational inertia (Shaft1Lin).
/// Each _translational_ shaft is connected to the Z degree of freedom of its body (through a ChShaftBodyTranslation).
/// The _rotational_ shaft is connected to Body 2 (through a ChShaftBodyRotation) along a direction (default: Z axis)
/// that can be later set through SetInnerShaft2RotDirection(). Any action applied to the shafts is then reflected back
/// to the respective bodies along their given directions.
///
///                  [************ ChLinkMotorLinearDriveline ********]
///     [ Body2 ]----[----(ChShaftBodyRotation)-------[Shaft2Rot]----]---->
///     [ Body2 ]----[----(ChShaftBodyTranslation)----[Shaft2Lin]----]---->
///     [ Body1 ]----[----(ChShaftBodyTranslation)----[Shaft1Lin]----]---->

/// Typical usage is to model a linear actuator between two 3D bodies by taking into account also the actuator driveline
/// (e.g. inertia, friction, ...).
///
///                  [************ ChLinkMotorLinearDriveline ********]
///     [ Body2 ]----[----(ChShaftBodyRotation)-------[Shaft2Rot]----]--->
///     [ Body2 ]----[----(ChShaftBodyTranslation)----[Shaft2Lin]----]--->
///     [ Body1 ]----[----(ChShaftBodyTranslation)----[Shaft1Lin]----]--->
///
///                                        [***** ChShaftsPlanetary *****]
///     >-[ChShaftsMotor]----[ChShaft] -----[----[shaft2]                 ]
///     >-----------------------------------[----[shaft1]                 ]
///     >-----------------------------------[----[shaft3]                 ]
///

class ChApi ChLinkMotorLinearDriveline : public ChLinkMotorLinear {
  public:
    ChLinkMotorLinearDriveline();
    ChLinkMotorLinearDriveline(const ChLinkMotorLinearDriveline& other);
    virtual ~ChLinkMotorLinearDriveline();

    /// "Virtual" copy constructor (covariant return type).
    virtual ChLinkMotorLinearDriveline* Clone() const override { return new ChLinkMotorLinearDriveline(*this); }

    virtual void SetSystem(ChSystem* m_system) override {
        ChPhysicsItem::SetSystem(m_system);
        innershaft1lin->SetSystem(m_system);
        innershaft2lin->SetSystem(m_system);
        innershaft2rot->SetSystem(m_system);
    }

    /// Access the inner 1D shaft connected to the translation of body 1 about dir of linear guide.
    /// The shaft can be connected to other shafts with ChShaftsMotor or similar items.
    std::shared_ptr<ChShaft> GetInnerShaft1Lin() const { return innershaft1lin; }

    /// Access the inner 1D shaft connected to the translation of body 2 about dir of linear guide.
    /// The shaft can be connected to other shafts with ChShaftsMotor or similar items.
    std::shared_ptr<ChShaft> GetInnerShaft2Lin() const { return innershaft2lin; }

    /// Access the inner 1D shaft connected to the rotation of body 2 about dir of linear guide.
    /// This is needed because one might need to design a driveline with rotational 1D components
    /// such as ChShaftsMotor, that require an anchoring to a rotational shaft.
    /// The shaft can be connected to other shafts with ChShaftsMotor or similar items.
    std::shared_ptr<ChShaft> GetInnerShaft2Rot() const { return innershaft2rot; }

    /// Set the direction of the inner rotation axis for body 2, expressed in link coordinates
    /// Default is VECT_Z, same dir of guide, i.e. useful when anchoring drives with screw transmission.
    void SetInnerShaft2RotDirection(ChVector3d md) { shaft2_rotation_dir = md; }

    /// Get the direction of the inner rotation axis for body 2, expressed in link coordinates
    /// Default is VECT_Z, same dir of guide, i.e. useful when anchoring drives with screw transmission.
    ChVector3d GetInnerShaft2RotDirection() const { return shaft2_rotation_dir; }

    /// Get the force between body 1 and inner shaft 1
    /// Note: coincident with GetMotorForce() of this motor.
    double GetInnerForce1() const { return innerconstraint1lin->GetForceReactionOnShaft(); }

    /// Get the force between body 2 and inner translational shaft 2
    double GetInnerForce2() const { return innerconstraint2lin->GetForceReactionOnShaft(); }

    /// Get the torque between body 2 and inner rotational shaft 2 (ex. might be caused by the
    /// inertia reaction of an internal rotation motor that is accelerating)
    double GetInnerTorque2() const { return innerconstraint2rot->GetTorqueReactionOnShaft(); }

    /// Get the current actuator reaction torque [Nm]
    virtual double GetMotorForce() const override { return GetInnerForce1(); }

    /// Initialize the generic mate, given the two bodies to be connected, and the absolute position of
    /// the mate (the two frames to connect on the bodies will be initially coincindent to that frame).
    virtual void Initialize(std::shared_ptr<ChBodyFrame> mbody1,  ///< first body to link
                            std::shared_ptr<ChBodyFrame> mbody2,  ///< second body to link
                            ChFrame<> mabsframe                   ///< mate frame, in abs. coordinate
                            ) override;

    /// Specialized initialization for LinkMotorLinearDriveline, given the two bodies to be connected, the positions of
    /// the two frames to connect on the bodies (each expressed in body or abs. coordinates).
    virtual void Initialize(std::shared_ptr<ChBodyFrame> mbody1,  ///< first body to link
                            std::shared_ptr<ChBodyFrame> mbody2,  ///< second body to link
                            bool pos_are_relative,                ///< true: following pos. are relative to bodies
                            ChFrame<> mframe1,                    ///< slave frame 1 (rel. or abs.)
                            ChFrame<> mframe2                     ///< master frame 2 (rel. or abs.)
                            ) override;

    /// Specialized initialization for LinkMotorLinearDriveline based on passing two vectors (point + dir) on the two
    /// bodies, which will represent the Z axes of the two frames (X and Y will be built from the Z vector via Gram
    /// Schmidt orthonormalization).
    virtual void Initialize(std::shared_ptr<ChBodyFrame> mbody1,  ///< first body to link
                            std::shared_ptr<ChBodyFrame> mbody2,  ///< second body to link
                            bool pos_are_relative,                ///< true: following pos. are relative to bodies
                            const ChVector3d& mpt1,               ///< origin of slave frame 1 (rel. or abs.)
                            const ChVector3d& mpt2,               ///< origin of master frame 2 (rel. or abs.)
                            const ChVector3d& mnorm1,             ///< X axis of slave plane 1 (rel. or abs.)
                            const ChVector3d& mnorm2              ///< X axis of master plane 2 (rel. or abs.)
                            ) override;

    /// Compute offsets of sub-objects, offsetting all the contained sub objects (the inner shafts)
    virtual void Setup() override;

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOut(ChArchiveOut& archive_out) override;

    /// Method to allow deserialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& archive_in) override;

  private:
    std::shared_ptr<ChShaft> innershaft1lin;
    std::shared_ptr<ChShaft> innershaft2lin;
    std::shared_ptr<ChShaft> innershaft2rot;
    std::shared_ptr<ChShaftBodyTranslation> innerconstraint1lin;
    std::shared_ptr<ChShaftBodyTranslation> innerconstraint2lin;
    std::shared_ptr<ChShaftBodyRotation> innerconstraint2rot;
    ChVector3d shaft2_rotation_dir;

    /// Update this object. Also relinks the innerconstraints.
    virtual void Update(double time, bool update_assets) override;

    virtual unsigned int GetNumCoordsPosLevel() override;
    virtual unsigned int GetNumConstraints() override;
    virtual unsigned int GetNumConstraintsBilateral() override;

    virtual void IntStateGather(const unsigned int off_x,
                                ChState& x,
                                const unsigned int off_v,
                                ChStateDelta& v,
                                double& T) override;
    virtual void IntStateScatter(const unsigned int off_x,
                                 const ChState& x,
                                 const unsigned int off_v,
                                 const ChStateDelta& v,
                                 const double T,
                                 bool full_update) override;
    virtual void IntStateGatherAcceleration(const unsigned int off_a, ChStateDelta& a) override;
    virtual void IntStateScatterAcceleration(const unsigned int off_a, const ChStateDelta& a) override;
    virtual void IntStateIncrement(const unsigned int off_x,
                                   ChState& x_new,
                                   const ChState& x,
                                   const unsigned int off_v,
                                   const ChStateDelta& Dv) override;
    virtual void IntStateGetIncrement(const unsigned int off_x,
                                      const ChState& x_new,
                                      const ChState& x,
                                      const unsigned int off_v,
                                      ChStateDelta& Dv) override;
    virtual void IntStateGatherReactions(const unsigned int off_L, ChVectorDynamic<>& L) override;
    virtual void IntStateScatterReactions(const unsigned int off_L, const ChVectorDynamic<>& L) override;
    virtual void IntLoadResidual_F(const unsigned int off, ChVectorDynamic<>& R, const double c) override;
    virtual void IntLoadResidual_Mv(const unsigned int off,
                                    ChVectorDynamic<>& R,
                                    const ChVectorDynamic<>& w,
                                    const double c) override;
    virtual void IntLoadLumpedMass_Md(const unsigned int off,
                                      ChVectorDynamic<>& Md,
                                      double& err,
                                      const double c) override;
    virtual void IntLoadResidual_CqL(const unsigned int off_L,
                                     ChVectorDynamic<>& R,
                                     const ChVectorDynamic<>& L,
                                     const double c) override;
    virtual void IntLoadConstraint_C(const unsigned int off,
                                     ChVectorDynamic<>& Qc,
                                     const double c,
                                     bool do_clamp,
                                     double recovery_clamp) override;
    virtual void IntLoadConstraint_Ct(const unsigned int off, ChVectorDynamic<>& Qc, const double c) override;
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
    virtual void ConstraintsBiReset() override;
    virtual void ConstraintsBiLoad_C(double factor = 1, double recovery_clamp = 0.1, bool do_clamp = false) override;
    virtual void ConstraintsBiLoad_Ct(double factor = 1) override;
    virtual void LoadConstraintJacobians() override;
    virtual void ConstraintsFetch_react(double factor = 1) override;
    virtual void InjectVariables(ChSystemDescriptor& descriptor) override;
    virtual void VariablesFbReset() override;
    virtual void VariablesFbLoadForces(double factor = 1) override;
    virtual void VariablesQbLoadSpeed() override;
    virtual void VariablesFbIncrementMq() override;
    virtual void VariablesQbSetSpeed(double step = 0) override;
    virtual void VariablesQbIncrementPosition(double step) override;
};

CH_CLASS_VERSION(ChLinkMotorLinearDriveline, 0)

}  // end namespace chrono

#endif
