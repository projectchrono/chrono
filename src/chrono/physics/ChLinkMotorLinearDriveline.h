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
#include "chrono/physics/ChShaftsBody.h"

namespace chrono {


/// This is an "interface" from 3D to a powertrain/powertrain that is modeled via
/// 1D elements such as ChShaft, ChShaftsMotor, ChShaftsGearbox, ChShaftsClutch, etc. 
///
/// This is the most avanced type of "linear motor" because using many of those 1D
/// elements one can build very complex drivelines, for example use
/// this ChLinkMotorLinearDriveline to represent a drive+reducer,
/// where usually the drive moves a recirculating screw or a pulley or a rack-pinion.
/// Hence this takes into account of the inertia of the motor shaft (as in
/// many cases of robotic actuators, that has electric drives+reducers.)
/// At the same time, using 1D elements avoids the unnecessary complication 
/// of using complete 3D parts to make screws, spindles, 3D rack-pinions, etc. 
///
///  The 1D driveline is "interfaced" to the two connected threedimensional
/// parts using two "inner" 1D shafts, each connected to 3D part translation;
/// it is up to the user to build the driveline that connects those two shafts.
///
///  Most often the driveline is a graph starting at inner shaft 2 (consider 
/// it to be the truss for holding the motor drive, also the support for reducers 
/// if any) and ending at inner shaft 1 (consider it to be the output, i.e. the 
/// slow-moving slider).

class ChApi ChLinkMotorLinearDriveline : public ChLinkMotorLinear {

    std::shared_ptr<ChShaft> innershaft1lin;            
    std::shared_ptr<ChShaft> innershaft2lin;  
    std::shared_ptr<ChShaft> innershaft2rot;  
    std::shared_ptr<ChShaftsBodyTranslation> innerconstraint1lin;  
    std::shared_ptr<ChShaftsBodyTranslation> innerconstraint2lin; 
    std::shared_ptr<ChShaftsBody> innerconstraint2rot; 
    ChVector<> shaft2_rotation_dir; 

  public:
    ChLinkMotorLinearDriveline();
    ChLinkMotorLinearDriveline(const ChLinkMotorLinearDriveline& other);
    virtual ~ChLinkMotorLinearDriveline();

    /// "Virtual" copy constructor (covariant return type).
    virtual ChLinkMotorLinearDriveline* Clone() const override { return new ChLinkMotorLinearDriveline(*this); }

	void SetSystem(ChSystem* m_system) override {
		ChPhysicsItem::SetSystem(m_system);
		innershaft1lin->SetSystem(m_system);
		innershaft2lin->SetSystem(m_system);
		innershaft2rot->SetSystem(m_system);
	}

    /// Access the inner 1D shaft connected to the translation of body1 about dir of linear guide.
    /// The shaft can be connected to other shafts with ChShaftsMotor or similar items.
    std::shared_ptr<ChShaft> GetInnerShaft1lin() const { return innershaft1lin; }

    /// Access the inner 1D shaft connected to the translation of body2 about dir of linear guide.
    /// The shaft can be connected to other shafts with ChShaftsMotor or similar items.
    std::shared_ptr<ChShaft> GetInnerShaft2lin() const { return innershaft2lin; }

    /// Access the inner 1D shaft connected to the rotation of body2 about dir of linear guide.
    /// This is needed because one might need to design a driveline with rotational 1D components
    /// such as ChShaftsMotor, that require an anchoring to a rotational shaft.
    /// The shaft can be connected to other shafts with ChShaftsMotor or similar items.
    std::shared_ptr<ChShaft> GetInnerShaft2rot() const { return innershaft2rot; }

    /// Set the direction of the inner rotation axis for body2, expressed in link coordinates 
    /// Default is VECT_X, same dir of guide, i.e. useful when anchoring drives with screw transmission.
    void SetInnerShaft2RotDirection(ChVector<> md) { shaft2_rotation_dir = md; }

    /// Get the direction of the inner rotation axis for body2, expressed in link coordinates 
    /// Default is VECT_X, same dir of guide, i.e. useful when anchoring drives with screw transmission.
    ChVector<> GetInnerShaft2RotDirection() const { return shaft2_rotation_dir; }


    /// Get the force between body 1 and inner shaft 1 
    /// Note: cohincident with GetMotorForce() of this motor.
    double GetInnerForce1() const { return innerconstraint1lin->GetForceReactionOnShaft(); }

    /// Get the force between body 2 and inner translational shaft 2 
    double GetInnerForce2() const { return innerconstraint2lin->GetForceReactionOnShaft(); }

    /// Get the torque between body 2 and inner rotational shaft 2 (ex. might be caused by the
    /// inertia reaction of an internal rotation motor that is accelerating)
    double GetInnerTorque2() const { return innerconstraint2rot->GetTorqueReactionOnShaft(); }


    /// Get the current actuator reaction torque [Nm]
    virtual double GetMotorForce() const override { return GetInnerForce1();}

	// Setup. Compute offsets of sub-objects, offsetting all the contained sub objects (the inner shafts)
    virtual void Setup() override;

    // Update. Also relinks the innerconstraints.
    void Update(double mytime, bool update_assets) override;

    //
    // STATE FUNCTIONS
    //
    virtual int GetDOF() override;
    virtual int GetDOC() override;
    virtual int GetDOC_c() override;

    virtual void IntStateGather(const unsigned int off_x, ChState& x, const unsigned int off_v, ChStateDelta& v, double& T) override;
    virtual void IntStateScatter(const unsigned int off_x, const ChState& x, const unsigned int off_v, const ChStateDelta& v, const double T) override;
    virtual void IntStateGatherAcceleration(const unsigned int off_a, ChStateDelta& a) override;
    virtual void IntStateScatterAcceleration(const unsigned int off_a, const ChStateDelta& a) override;
    virtual void IntStateIncrement(const unsigned int off_x, ChState& x_new, const ChState& x, const unsigned int off_v, const ChStateDelta& Dv) override;
    virtual void IntStateGatherReactions(const unsigned int off_L, ChVectorDynamic<>& L) override;
    virtual void IntStateScatterReactions(const unsigned int off_L, const ChVectorDynamic<>& L) override;
    virtual void IntLoadResidual_F(const unsigned int off, ChVectorDynamic<>& R, const double c) override;
    virtual void IntLoadResidual_Mv(const unsigned int off, ChVectorDynamic<>& R, const ChVectorDynamic<>& w, const double c) override;
    virtual void IntLoadResidual_CqL(const unsigned int off_L, ChVectorDynamic<>& R, const ChVectorDynamic<>& L, const double c) override;
    virtual void IntLoadConstraint_C(const unsigned int off, ChVectorDynamic<>& Qc, const double c, bool do_clamp, double recovery_clamp) override;
    virtual void IntLoadConstraint_Ct(const unsigned int off, ChVectorDynamic<>& Qc, const double c) override;
    virtual void IntToDescriptor(const unsigned int off_v, const ChStateDelta& v, const ChVectorDynamic<>& R, const unsigned int off_L, const ChVectorDynamic<>& L, const ChVectorDynamic<>& Qc) override;
    virtual void IntFromDescriptor(const unsigned int off_v, ChStateDelta& v, const unsigned int off_L, ChVectorDynamic<>& L) override;

    //
    // SOLVER INTERFACE (old)
    //

    virtual void InjectConstraints(ChSystemDescriptor& mdescriptor) override;
    virtual void ConstraintsBiReset() override;
    virtual void ConstraintsBiLoad_C(double factor = 1, double recovery_clamp = 0.1, bool do_clamp = false) override;
    virtual void ConstraintsBiLoad_Ct(double factor = 1) override;
    virtual void ConstraintsLoadJacobians() override;
    virtual void ConstraintsFetch_react(double factor = 1) override;
    virtual void InjectVariables(ChSystemDescriptor& mdescriptor) override;
    virtual void VariablesFbReset() override;
    virtual void VariablesFbLoadForces(double factor = 1) override;
    virtual void VariablesQbLoadSpeed() override;
    virtual void VariablesFbIncrementMq() override;
    virtual void VariablesQbSetSpeed(double step = 0) override;
    virtual void VariablesQbIncrementPosition(double step) override;

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOUT(ChArchiveOut& marchive) override;

    /// Method to allow deserialization of transient data from archives.
    virtual void ArchiveIN(ChArchiveIn& marchive) override;

};

CH_CLASS_VERSION(ChLinkMotorLinearDriveline,0)



}  // end namespace chrono

#endif
