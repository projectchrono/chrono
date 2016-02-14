//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2010-2012 Alessandro Tasora
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//

#ifndef CHSHAFTSMOTOR_H
#define CHSHAFTSMOTOR_H

//////////////////////////////////////////////////
//
//   ChShaftsMotor.h
//
//   Class for defining a motor (a torque) between
//   two one-degree-of-freedom parts, that is,
//   shafts that can be used to build 1D models
//   of power trains. This is more efficient than
//   simulating power trains modeled full 3D ChBody
//   objects.
//
//   HEADER file for CHRONO,
//	 Multibody dynamics engine
//
// ------------------------------------------------
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////

#include "physics/ChShaftsCouple.h"
#include "lcp/ChLcpConstraintTwoGeneric.h"

namespace chrono {

///  Class for defining a 'motor' (a 1D model of 'imposed torque'
///  or 'imposed velocity' or 'imposed rotation')
///  between two one-degree-of-freedom parts, that is,
///  shafts that can be used to build 1D models
///  of power trains. This is more efficient than
///  simulating power trains modeled with full 3D ChBody
///  objects.
///  Note, it is not inherited from ChShaftsTorqueBase because
///  it might introduce also a constraint, in case it is working in
///  MOT_MODE_ROTATION or MOT_MODE_SPEED.

class ChApi ChShaftsMotor : public ChShaftsCouple {
    // Chrono simulation of RTTI, needed for serialization
    CH_RTTI(ChShaftsMotor, ChShaftsCouple);

  private:
    //
    // DATA
    //

    double motor_torque;

    double motor_set_rot;
    double motor_set_rot_dt;

    double torque_react1;
    double torque_react2;

    // used as an interface to the LCP solver.
    ChLcpConstraintTwoGeneric constraint;
    float cache_li_speed;  // used to cache the last computed value of multiplier (solver warm starting)
    float cache_li_pos;    // used to cache the last computed value of multiplier (solver warm starting)

  public:
    //
    // CONSTRUCTORS
    //

    /// Constructor.
    ChShaftsMotor();
    /// Destructor
    ~ChShaftsMotor();

    /// Copy from another ChShaftsMotor.
    void Copy(ChShaftsMotor* source);

    //
    // FLAGS
    //

    //
    // FUNCTIONS
    //

    /// Number of scalar constraints
    virtual int GetDOC_c() {
        if (motor_mode == MOT_MODE_TORQUE)
            return 0;
        else
            return 1;
    }

    //
    // STATE FUNCTIONS
    //

    // (override/implement interfaces for global state vectors, see ChPhysicsItem for comments.)
    virtual void IntStateGatherReactions(const unsigned int off_L, ChVectorDynamic<>& L);
    virtual void IntStateScatterReactions(const unsigned int off_L, const ChVectorDynamic<>& L);
    virtual void IntLoadResidual_F(const unsigned int off, ChVectorDynamic<>& R, const double c);
    virtual void IntLoadResidual_CqL(const unsigned int off_L,
                                     ChVectorDynamic<>& R,
                                     const ChVectorDynamic<>& L,
                                     const double c);
    virtual void IntLoadConstraint_C(const unsigned int off,
                                     ChVectorDynamic<>& Qc,
                                     const double c,
                                     bool do_clamp,
                                     double recovery_clamp);
    virtual void IntLoadConstraint_Ct(const unsigned int off, ChVectorDynamic<>& Qc, const double c);
    virtual void IntToLCP(const unsigned int off_v,
                          const ChStateDelta& v,
                          const ChVectorDynamic<>& R,
                          const unsigned int off_L,
                          const ChVectorDynamic<>& L,
                          const ChVectorDynamic<>& Qc);
    virtual void IntFromLCP(const unsigned int off_v, ChStateDelta& v, const unsigned int off_L, ChVectorDynamic<>& L);

    // Override/implement LCP system functions of ChShaftsCouple
    // (to assembly/manage data for LCP system solver

    virtual void InjectConstraints(ChLcpSystemDescriptor& mdescriptor);
    virtual void ConstraintsBiReset();
    virtual void ConstraintsBiLoad_C(double factor = 1., double recovery_clamp = 0.1, bool do_clamp = false);
    virtual void ConstraintsBiLoad_Ct(double factor = 1.);
    virtual void ConstraintsFbLoadForces(double factor = 1.);
    virtual void ConstraintsLoadJacobians();
    virtual void ConstraintsLiLoadSuggestedSpeedSolution();
    virtual void ConstraintsLiLoadSuggestedPositionSolution();
    virtual void ConstraintsLiFetchSuggestedSpeedSolution();
    virtual void ConstraintsLiFetchSuggestedPositionSolution();
    virtual void ConstraintsFetch_react(double factor = 1.);

    // Other functions

    /// Use this function after gear creation, to initialize it, given
    /// two shafts to join. The first shaft is the 'output' shaft of the motor,
    /// the second is the 'truss', often fixed and not rotating.
    /// The torque is applied to the output shaft, while the truss shafts
    /// gets the same torque but with opposite sign.
    /// Each shaft must belong to the same ChSystem.
    virtual bool Initialize(std::shared_ptr<ChShaft> mshaft1,  ///< first  shaft to join (motor output shaft)
                            std::shared_ptr<ChShaft> mshaft2   ///< second shaft to join (motor truss)
                            );

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
        this->motor_torque = mt;
    }

    /// Get the motor torque applied between shaft2 and shaft1.
    double GetMotorTorque() const { return this->motor_torque; }

    /// Get the reaction torque exchanged between the two shafts,
    /// considered as applied to the 1st axis.
    double GetTorqueReactionOn1() const { return (GetMotorTorque()); }

    /// Get the reaction torque exchanged between the two shafts,
    /// considered as applied to the 2nd axis.
    double GetTorqueReactionOn2() const { return -(GetMotorTorque()); }

    /// Set the motor rotation phase between shaft2 and shaft1.
    /// If the rotation is not constant, you also must use SetMotorRot_dt()
    /// Note: use this only when in MOT_MODE_ROTATION !
    void SetMotorRot(double mt) {
        assert(motor_mode == MOT_MODE_ROTATION);
        this->motor_set_rot = mt;
    }

    /// Set the motor rotation speed between shaft2 and shaft1.
    /// Note: use this only when in MOT_MODE_ROTATION or MOT_MODE_SPEED !
    void SetMotorRot_dt(double mt) {
        assert((motor_mode == MOT_MODE_ROTATION) || (motor_mode == MOT_MODE_SPEED));
        this->motor_set_rot_dt = mt;
    }

    /// Get the actual angle rotation of the motor, in terms of phase of shaft 1 respect to 2.
    double GetMotorRot() const { return (this->shaft1->GetPos() - this->shaft2->GetPos()); }
    /// Get the actual speed of the motor, in terms of speed of shaft 1 respect to 2.
    double GetMotorRot_dt() const { return (this->shaft1->GetPos_dt() - this->shaft2->GetPos_dt()); }
    /// Get the actual acceleration of the motor, in terms of accel. of shaft 1 respect to 2.
    double GetMotorRot_dtdt() const { return (this->shaft1->GetPos_dtdt() - this->shaft2->GetPos_dtdt()); }

    //
    // UPDATE FUNCTIONS
    //

    /// Update all auxiliary data of the gear transmission at given time
    virtual void Update(double mytime, bool update_assets = true);

    //
    // SERIALIZATION
    //

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOUT(ChArchiveOut& marchive);

    /// Method to allow deserialization of transient data from archives.
    virtual void ArchiveIN(ChArchiveIn& marchive);
};

}  // END_OF_NAMESPACE____

#endif
