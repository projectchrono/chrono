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

#ifndef CHSHAFTSMOTOR_H
#define CHSHAFTSMOTOR_H

#include "chrono/physics/ChShaftsCouple.h"
#include "chrono/solver/ChConstraintTwoGeneric.h"

namespace chrono {



/// Base class for all "motors" between two 1D elements of ChShaft class.
/// You can consider the usage of its inherited  classes such as ChShaftsMotor,
/// ChShaftsMotorAngle, ChShaftsMotorSpeed or ChShaftsMotorTorque.

class ChApi ChShaftsMotorBase : public ChShaftsCouple {
  public:
    ChShaftsMotorBase(){};
    ChShaftsMotorBase(const ChShaftsMotorBase& other){};
    virtual ~ChShaftsMotorBase(){};

    /// Get the actual angle rotation [rad] of the motor, in terms of phase of shaft 1 respect to 2.
    virtual double GetMotorRot() const { return (shaft1->GetPos() - shaft2->GetPos()); }
    /// Get the actual speed [rad/s] of the motor, in terms of speed of shaft 1 respect to 2.
    virtual double GetMotorRot_dt() const { return (shaft1->GetPos_dt() - shaft2->GetPos_dt()); }
    /// Get the actual acceleration [rad/s^2] of the motor, in terms of accel. of shaft 1 respect to 2.
    virtual double GetMotorRot_dtdt() const { return (shaft1->GetPos_dtdt() - shaft2->GetPos_dtdt()); }

    /// In case of multi-turns, gets the current actuator number of (integer) rotations:
    virtual int GetMotorRotTurns() const { return int(GetMotorRot() / CH_C_2PI); }

    /// In case of multi-turns, gets the current actuator rotation angle [rad], in periodic -PI..+PI.
    virtual double GetMotorRotPeriodic() const { return fmod(GetMotorRot(), CH_C_2PI); }

    /// Get the current motor torque between shaft2 and shaft1, expressed as applied to shaft1
    virtual double GetMotorTorque() const = 0;

    /// Get the reaction torque exchanged between the two shafts,
    /// considered as applied to the 1st axis.
    virtual double GetTorqueReactionOn1() const override { return (GetMotorTorque()); }

    /// Get the reaction torque exchanged between the two shafts,
    /// considered as applied to the 2nd axis.
    virtual double GetTorqueReactionOn2() const override { return -(GetMotorTorque()); }

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOut(ChArchiveOut& marchive) override;

    /// Method to allow deserialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& marchive) override;
};

CH_CLASS_VERSION(ChShaftsMotorBase, 0)




///  Class for a multipurpose motor (a 1D model of 'imposed torque'
///  or 'imposed velocity' or 'imposed rotation')
///  between two one-degree-of-freedom parts, that is,
///  shafts that can be used to build 1D models
///  of power trains. This is more efficient than
///  simulating power trains modeled with full 3D ChBody
///  objects.
///  Consider also the more specific ChShaftsMotorAngle, 
///  ChShaftsMotorSpeed or ChShaftsMotorTorque  for more advanced 
///  models.

class ChApi ChShaftsMotor : public ChShaftsMotorBase {

  private:
    double motor_torque;

    double motor_set_rot;
    double motor_set_rot_dt;

    ChConstraintTwoGeneric constraint;  ///< used as an interface to the solver

  public:
    ChShaftsMotor();
    ChShaftsMotor(const ChShaftsMotor& other);
    ~ChShaftsMotor() {}

    /// "Virtual" copy constructor (covariant return type).
    virtual ChShaftsMotor* Clone() const override { return new ChShaftsMotor(*this); }

    /// Number of scalar constraints
    virtual int GetDOC_c() override { return (motor_mode == MOT_MODE_TORQUE) ? 0 : 1; }

    //
    // STATE FUNCTIONS
    //

    // (override/implement interfaces for global state vectors, see ChPhysicsItem for comments.)
    virtual void IntStateGatherReactions(const unsigned int off_L, ChVectorDynamic<>& L) override;
    virtual void IntStateScatterReactions(const unsigned int off_L, const ChVectorDynamic<>& L) override;
    virtual void IntLoadResidual_F(const unsigned int off, ChVectorDynamic<>& R, const double c) override;
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

    // Override/implement system functions of ChShaftsCouple
    // (to assemble/manage data for system solver)

    virtual void InjectConstraints(ChSystemDescriptor& mdescriptor) override;
    virtual void ConstraintsBiReset() override;
    virtual void ConstraintsBiLoad_C(double factor = 1, double recovery_clamp = 0.1, bool do_clamp = false) override;
    virtual void ConstraintsBiLoad_Ct(double factor = 1) override;
    virtual void ConstraintsFbLoadForces(double factor = 1) override;
    virtual void ConstraintsLoadJacobians() override;
    virtual void ConstraintsFetch_react(double factor = 1) override;

    // Other functions

    /// Use this function after gear creation, to initialize it, given
    /// two shafts to join. The first shaft is the 'output' shaft of the motor,
    /// the second is the 'truss', often fixed and not rotating.
    /// The torque is applied to the output shaft, while the truss shafts
    /// gets the same torque but with opposite sign.
    /// Each shaft must belong to the same ChSystem.
    bool Initialize(std::shared_ptr<ChShaft> mshaft1,  ///< first  shaft to join (motor output shaft)
                    std::shared_ptr<ChShaft> mshaft2   ///< second shaft to join (motor truss)
                    ) override;

    enum eCh_shaftsmotor_mode { MOT_MODE_ROTATION = 0, MOT_MODE_SPEED, MOT_MODE_TORQUE } motor_mode;

    /// Se the motor mode. The options are that you impose
    /// the relative torque between the two shafts,
    /// or their relative rotation phase, or
    /// their relative speed, but one mode excludes the others.
    void SetMotorMode(eCh_shaftsmotor_mode mmode) { motor_mode = mmode; }

    /// Set the motor torque applied between shaft2 and shaft1.
    /// So, if fixed, shaft1 can be considered the reference, or the 'truss'.
    /// (The torque is applied with opposite sign to shaft 1).
    /// Note: use this only when in MOT_MODE_TORQUE !!
    void SetMotorTorque(double mt) {
        assert(motor_mode == MOT_MODE_TORQUE);
        motor_torque = mt;
    }

    /// Get the currnet motor torque between shaft2 and shaft1, expressed as applied to shaft1
    virtual double GetMotorTorque() const override { return motor_torque; }


    /// Set the motor rotation phase between shaft2 and shaft1.
    /// If the rotation is not constant, you also must use SetMotorRot_dt()
    /// Note: use this only when in MOT_MODE_ROTATION !
    void SetMotorRot(double mt) {
        assert(motor_mode == MOT_MODE_ROTATION);
        motor_set_rot = mt;
    }

    /// Set the motor rotation speed between shaft2 and shaft1.
    /// Note: use this only when in MOT_MODE_ROTATION or MOT_MODE_SPEED !
    void SetMotorRot_dt(double mt) {
        assert((motor_mode == MOT_MODE_ROTATION) || (motor_mode == MOT_MODE_SPEED));
        motor_set_rot_dt = mt;
    }



    /// Update all auxiliary data 
    virtual void Update(double mytime, bool update_assets = true) override;

    //
    // SERIALIZATION
    //

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOut(ChArchiveOut& marchive) override;

    /// Method to allow deserialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& marchive) override;
};

CH_CLASS_VERSION(ChShaftsMotor,0)









}  // end namespace chrono

#endif
