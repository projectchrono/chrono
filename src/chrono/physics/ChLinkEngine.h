//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2010 Alessandro Tasora
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//

#ifndef CHLINKENGINE_H
#define CHLINKENGINE_H

#include "physics/ChLinkLock.h"
#include "physics/ChShaft.h"
#include "physics/ChShaftsBody.h"
#include "lcp/ChLcpConstraintTwoGeneric.h"

namespace chrono {
///
/// Class for links representing engines between two rigid bodies.
/// Note that the engine can be in 'impose relative rotation' mode,
/// as well as in 'impose speed' etc. It can also be used to represent
/// an engine with a torque/speed custom curve. etc.
///

class ChApi ChLinkEngine : public ChLinkLock {
    CH_RTTI(ChLinkEngine, ChLinkLock);

  protected:
    ChSharedPtr<ChFunction> rot_funct;  // rotation(t) function
    ChSharedPtr<ChFunction> spe_funct;  // speed(t) function
    ChSharedPtr<ChFunction> tor_funct;  // torque(t) function
    ChSharedPtr<ChFunction> torque_w;   // torque(w) function

    int learn;  // if TRUE, the actuator does not apply constraint, just
                // records the motion into its rot_function.

    int impose_reducer;  // if true, speed torque or rotation are imposed to the fast (motor) shaft, before reducer!

    double mot_rot;         // current rotation (read only)
    double mot_rot_dt;      // current ang speed (read only)
    double mot_rot_dtdt;    // current ang acc  (read only)
    double mot_torque;      // current motor torque (read only)
    double mot_rerot;       // current rotation (read only)  before reducer
    double mot_rerot_dt;    // current ang speed (read only) before reducer
    double mot_rerot_dtdt;  // current ang acc  (read only)  before reducer
    double mot_retorque;    // current motor torque (read only) before reducer

    double mot_tau;      // motor: transmission ratio
    double mot_eta;      // motor: transmission efficiency
    double mot_inertia;  // motor: inertia (added to system)

    int eng_mode;  // mode of controlling the motor (by rotation, speed etc.)

    int shaft_mode;  // mode of imposing constraints on extra (non-z) degrees of freedom

    ChSharedPtr<ChFunction> rot_funct_x;  // rotation(t) function for keyframe polar motor
    ChSharedPtr<ChFunction> rot_funct_y;  // rotation(t) function for keyframe polar motor
    double last_r3time;                   // internal:for backward differentiation to compute speed in keyframe mode
    double last_r3mot_rot;                // internal:for backward differentiation to compute speed in keyframe mode
    double last_r3mot_rot_dt;             // internal:for backward differentiation to compute speed in keyframe mode
    Quaternion last_r3relm_rot;           // internal:for backward differentiation to compute speed in keyframe mode
    Quaternion last_r3relm_rot_dt;        // internal
    Quaternion keyed_polar_rotation;      // internal

    ChSharedPtr<ChShaft> innershaft1;            // used in ENG_MODE_TO_POWERTRAIN_SHAFT
    ChSharedPtr<ChShaft> innershaft2;            // ''      ''
    ChSharedPtr<ChShaftsBody> innerconstraint1;  // ''     ''
    ChSharedPtr<ChShaftsBody> innerconstraint2;  // ''     ''
    double cache_li_speed1;
    double cache_li_pos1;
    double torque_react1;
    double cache_li_speed2;
    double cache_li_pos2;
    double torque_react2;

  public:
    // builders and destroyers
    ChLinkEngine();
    virtual ~ChLinkEngine();
    virtual void Copy(ChLinkEngine* source);
    virtual ChLink* new_Duplicate();  // always return base link class pointer

    // UPDATING FUNCTIONS - "lin.act. link" custom implementations

    /// Updates motion laws, etc. for the impose rotation / impose speed modes
    virtual void UpdateTime(double mytime);
    /// Updates torque for the impose torque mode
    virtual void UpdateForces(double mytime);
    /// Updates the r3d time, so perform differentiation for computing speed in case of keyframed motion

    virtual void UpdatedExternalTime(double prevtime, double time);

    /// Sets up the markers associated with the engine link
    virtual void SetUpMarkers(ChMarker* mark1, ChMarker* mark2);

    // data get/set
    ChSharedPtr<ChFunction> Get_rot_funct() const { return rot_funct; }
    ChSharedPtr<ChFunction> Get_spe_funct() const { return spe_funct; }
    ChSharedPtr<ChFunction> Get_tor_funct() const { return tor_funct; }
    ChSharedPtr<ChFunction> Get_torque_w_funct() const { return torque_w; }

    void Set_rot_funct(ChSharedPtr<ChFunction> mf) { rot_funct = mf; }
    void Set_spe_funct(ChSharedPtr<ChFunction> mf) { spe_funct = mf; }
    void Set_tor_funct(ChSharedPtr<ChFunction> mf) { tor_funct = mf; }
    void Set_torque_w_funct(ChSharedPtr<ChFunction> mf) { torque_w = mf; }

    ChSharedPtr<ChFunction> Get_rot_funct_x() const { return rot_funct_x; }
    ChSharedPtr<ChFunction> Get_rot_funct_y() const { return rot_funct_y; }
    const Quaternion& GetKeyedPolarRotation() const { return keyed_polar_rotation; }

    void Set_rot_funct_x(ChSharedPtr<ChFunction> mf) { rot_funct_x = mf; }
    void Set_rot_funct_y(ChSharedPtr<ChFunction> mf) { rot_funct_y = mf; }
    void SetKeyedPolarRotation(const Quaternion& mq) { keyed_polar_rotation = mq; }

    int Get_learn() const { return learn; }
    int Get_impose_reducer() const { return impose_reducer; }
    int Get_eng_mode() const { return eng_mode; }

    void Set_learn(int mset);
    void Set_impose_reducer(int mset) { impose_reducer = mset; }
    void Set_eng_mode(int mset);

    enum eCh_eng_mode {
        ENG_MODE_ROTATION = 0,
        ENG_MODE_SPEED,
        ENG_MODE_TORQUE,
        ENG_MODE_KEY_ROTATION,
        ENG_MODE_KEY_POLAR,
        ENG_MODE_TO_POWERTRAIN_SHAFT
    };

    int Get_shaft_mode() const { return shaft_mode; }
    void Set_shaft_mode(int mset);

    enum eCh_shaft_mode {
        ENG_SHAFT_LOCK = 0,   // shafts of motor and user (markers 1 and 2) are stiffly joined
        ENG_SHAFT_PRISM,      // shafts of motor and user (markers 1 and 2) can shift along shaft (Z axis)
        ENG_SHAFT_OLDHAM,     // shafts of motor and user (markers 1 and 2) may be parallel shifting on X and Y
        ENG_SHAFT_UNIVERSAL,  // not yet used
        ENG_SHAFT_CARDANO     // not yet used
    };

    double Get_mot_rot() const { return mot_rot; }
    double Get_mot_rot_dt() const { return mot_rot_dt; }
    double Get_mot_rot_dtdt() const { return mot_rot_dtdt; }
    double Get_mot_torque() const { return mot_torque; }
    double Get_mot_rerot() const { return mot_rerot; }
    double Get_mot_rerot_dt() const { return mot_rerot_dt; }
    double Get_mot_rerot_dtdt() const { return mot_rerot_dtdt; }
    double Get_mot_retorque() const { return mot_retorque; }
    double Get_mot_tau() const { return mot_tau; }
    double Get_mot_eta() const { return mot_eta; }
    double Get_mot_inertia() const { return mot_inertia; }

    void Set_mot_tau(double mtau) { mot_tau = mtau; }
    void Set_mot_eta(double meta) { mot_eta = meta; }
    void Set_mot_inertia(double min) { mot_inertia = min; }

    // Access the inner 1D shaft connected to the rotation of body1 about dir of motor shaft,
    // if in CH_ENG_MODE_TO_POWERTRAIN_SHAFT. The shaft can be
    // connected to other shafts with ChShaftsClutch or similar items.
    ChSharedPtr<ChShaft> GetInnerShaft1() { return innershaft1; }
    // Access the inner 1D shaft connected to the rotation of body2 about dir of motor shaft,
    // if in CH_ENG_MODE_TO_POWERTRAIN_SHAFT. The shaft can be
    // connected to other shafts with ChShaftsClutch or similar items.
    ChSharedPtr<ChShaft> GetInnerShaft2() { return innershaft2; }
    // Get the torque between body 1 and inner shaft 1.
    // Note: use only if in CH_ENG_MODE_TO_POWERTRAIN_SHAFT.
    double GetInnerTorque1() const { return torque_react1; }
    // Get the torque between body 2 and inner shaft 2.
    // Note: use only if in CH_ENG_MODE_TO_POWERTRAIN_SHAFT.
    double GetInnerTorque2() const { return torque_react2; }

    //
    // STATE FUNCTIONS
    //
    virtual int GetDOF();
    virtual int GetDOC_c();

    // (override/implement interfaces for global state vectors, see ChPhysicsItem for comments.)
    // (beyond the base link implementations, it also have to
    // add the constraint coming from the inner shaft etc.)
    virtual void IntStateGather(const unsigned int off_x,
                                ChState& x,
                                const unsigned int off_v,
                                ChStateDelta& v,
                                double& T);
    virtual void IntStateScatter(const unsigned int off_x,
                                 const ChState& x,
                                 const unsigned int off_v,
                                 const ChStateDelta& v,
                                 const double T);
    virtual void IntStateGatherAcceleration(const unsigned int off_a, ChStateDelta& a);
    virtual void IntStateScatterAcceleration(const unsigned int off_a, const ChStateDelta& a);
    virtual void IntStateIncrement(const unsigned int off_x,
                                   ChState& x_new,
                                   const ChState& x,
                                   const unsigned int off_v,
                                   const ChStateDelta& Dv);
    virtual void IntStateGatherReactions(const unsigned int off_L, ChVectorDynamic<>& L);
    virtual void IntStateScatterReactions(const unsigned int off_L, const ChVectorDynamic<>& L);
    virtual void IntLoadResidual_F(const unsigned int off, ChVectorDynamic<>& R, const double c);
    virtual void IntLoadResidual_Mv(const unsigned int off,
                                    ChVectorDynamic<>& R,
                                    const ChVectorDynamic<>& w,
                                    const double c);
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

    //
    // LCP INTERFACE
    //

    // Overload LCP system functions of ChPhysicsItem
    // (beyond the base link implementations, it also have to
    // add the constraint coming from the inner shaft etc.)
    virtual void InjectConstraints(ChLcpSystemDescriptor& mdescriptor);
    virtual void ConstraintsBiReset();
    virtual void ConstraintsBiLoad_C(double factor = 1., double recovery_clamp = 0.1, bool do_clamp = false);
    virtual void ConstraintsBiLoad_Ct(double factor = 1.);
    virtual void ConstraintsLoadJacobians();
    virtual void ConstraintsLiLoadSuggestedSpeedSolution();
    virtual void ConstraintsLiLoadSuggestedPositionSolution();
    virtual void ConstraintsLiFetchSuggestedSpeedSolution();
    virtual void ConstraintsLiFetchSuggestedPositionSolution();
    virtual void ConstraintsFetch_react(double factor = 1.);
    virtual void InjectVariables(ChLcpSystemDescriptor& mdescriptor);
    virtual void VariablesFbReset();
    virtual void VariablesFbLoadForces(double factor = 1.);
    virtual void VariablesQbLoadSpeed();
    virtual void VariablesFbIncrementMq();
    virtual void VariablesQbSetSpeed(double step = 0.);
    virtual void VariablesQbIncrementPosition(double step);

    // STREAMING
    virtual void StreamIN(ChStreamInBinary& mstream);
    virtual void StreamOUT(ChStreamOutBinary& mstream);
};

//////////////////////////////////////////////////////
//////////////////////////////////////////////////////

}  // END_OF_NAMESPACE____

#endif
