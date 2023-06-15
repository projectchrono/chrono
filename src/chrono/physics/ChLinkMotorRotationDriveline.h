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

#ifndef CHLINKMOTORROTATIONDRIVELINE_H
#define CHLINKMOTORROTATIONDRIVELINE_H


#include "chrono/physics/ChLinkMotorRotation.h"
#include "chrono/physics/ChShaft.h"
#include "chrono/physics/ChShaftsBody.h"

namespace chrono {


/// This is an "interface" from 3D to a powertrain/powertrain that is modeled via
/// 1D elements such as ChShaft, ChShaftsMotor, ChShaftsGearbox, ChShaftsClutch, etc. 
///
///  This is the most avanced type of "motor" because using many of those 1D
/// elements one can build very complex drivelines, for example use
/// this ChLinkMotorRotationDriveline to represent a drive+reducer,
/// hence taking into account of the inertia of the motor shaft (as in
/// many cases of robotic actuators, that has electric drives+reducers.)
/// At the same time, using 1D elements avoids the unnecessary complication 
/// of using complete 3D parts to make fast spindles, 3D gears etc. 
///
///  The 1D driveline is "interfaced" to the two connected threedimensional
/// parts using two "inner" 1D shafts, each rotating as the connected 3D part;
/// it is up to the user to build the driveline that connects those two shafts.
///
///  Most often the driveline is a graph starting at inner shaft 2 (consider 
/// it to be the truss for holding the motor drive, also the support for reducers 
/// if any) and ending at inner shaft 1 (consider it to be the output, i.e. the 
/// slow-rotation spindle).
///  Note that it is up to the user to create a driveline where all torques are
/// balanced action/reactions: in this case, 
///    GetMotorTorque() = GetInnerTorque1() = - GetInnerTorque2(). 
/// This is not true for example, for an unbalanced driveline where one of the 
/// two inner shafts is connected to some external ChShaft. 

class ChApi ChLinkMotorRotationDriveline : public ChLinkMotorRotation {

    std::shared_ptr<ChShaft> innershaft1;            
    std::shared_ptr<ChShaft> innershaft2;            
    std::shared_ptr<ChShaftsBody> innerconstraint1;  
    std::shared_ptr<ChShaftsBody> innerconstraint2;  

  public:
    ChLinkMotorRotationDriveline();
    ChLinkMotorRotationDriveline(const ChLinkMotorRotationDriveline& other);
    virtual ~ChLinkMotorRotationDriveline();

    /// "Virtual" copy constructor (covariant return type).
    virtual ChLinkMotorRotationDriveline* Clone() const override { return new ChLinkMotorRotationDriveline(*this); }


	virtual void SetSystem(ChSystem* m_system) override {
		ChPhysicsItem::SetSystem(m_system);
		innershaft1->SetSystem(m_system);
		innershaft2->SetSystem(m_system);
	}

    /// Access the inner 1D shaft connected to the rotation of body1 about dir of motor shaft.
    /// The shaft can be connected to other shafts with ChShaftsClutch or similar items.
    std::shared_ptr<ChShaft> GetInnerShaft1() const { return innershaft1; }

    /// Access the inner 1D shaft connected to the rotation of body2 about dir of motor shaft,
    /// The shaft can be connected to other shafts with ChShaftsClutch or similar items.
    std::shared_ptr<ChShaft> GetInnerShaft2() const { return innershaft2; }

    /// Get the torque between body 1 and inner shaft 1. 
    /// Note: cohincident with GetMotorTorque() of this motor.
    /// Note: if driveline is not connected to outer 1D shafts, it should be GetInnerTorque2() = - GetInnerTorque1()
    double GetInnerTorque1() const { return innerconstraint1->GetTorqueReactionOnShaft(); }

    /// Get the torque between body 2 and inner shaft 2 
    /// Note, if driveline is not connected to outer 1D shafts, it should be GetInnerTorque2() = - GetInnerTorque1()
    double GetInnerTorque2() const { return innerconstraint2->GetTorqueReactionOnShaft(); }

    /// Get the current actuator reaction torque [Nm]
    virtual double GetMotorTorque() const override { return GetInnerTorque1();}

    /// Initialize the generic mate, given the two bodies to be connected, and the absolute position of
    /// the mate (the two frames to connect on the bodies will be initially coincindent to that frame).
    virtual void Initialize(std::shared_ptr<ChBodyFrame> mbody1,  ///< first body to link
                            std::shared_ptr<ChBodyFrame> mbody2,  ///< second body to link
                            ChFrame<> mabsframe                   ///< mate frame, in abs. coordinate
                            ) override;

    /// Specialized initialization for LinkMotorRotationDriveline, given the two bodies to be connected, the positions
    /// of the two frames to connect on the bodies (each expressed in body or abs. coordinates).
    virtual void Initialize(std::shared_ptr<ChBodyFrame> mbody1,  ///< first body to link
                            std::shared_ptr<ChBodyFrame> mbody2,  ///< second body to link
                            bool pos_are_relative,                ///< true: following pos. are relative to bodies
                            ChFrame<> mframe1,                    ///< slave frame 1 (rel. or abs.)
                            ChFrame<> mframe2                     ///< master frame 2 (rel. or abs.)
                            ) override;

    /// Specialized initialization for LinkMotorRotationDriveline based on passing two vectors (point + dir) on the two
    /// bodies, which will represent the X axes of the two frames (Y and Z will be built from the X vector via Gram
    /// Schmidt orthonormalization).
    virtual void Initialize(std::shared_ptr<ChBodyFrame> mbody1,  ///< first body to link
                            std::shared_ptr<ChBodyFrame> mbody2,  ///< second body to link
                            bool pos_are_relative,                ///< true: following pos. are relative to bodies
                            ChVector<> mpt1,                      ///< origin of slave frame 1 (rel. or abs.)
                            ChVector<> mpt2,                      ///< origin of master frame 2 (rel. or abs.)
                            ChVector<> mnorm1,                    ///< X axis of slave plane 1 (rel. or abs.)
                            ChVector<> mnorm2                     ///< X axis of master plane 2 (rel. or abs.)
                            ) override;

    // Compute offsets of sub-objects, offsetting all the contained sub objects (the inner shafts)
	virtual void Setup() override;

    // Update this object. Also relinks the innerconstraints.
    virtual void Update(double mytime, bool update_assets = true) override;

    //
    // STATE FUNCTIONS
    //
    virtual int GetDOF() override;
    virtual int GetDOC() override;
    virtual int GetDOC_c() override;

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
    virtual void ArchiveOut(ChArchiveOut& marchive) override;

    /// Method to allow deserialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& marchive) override;

};

CH_CLASS_VERSION(ChLinkMotorRotationDriveline,0)





}  // end namespace chrono

#endif
