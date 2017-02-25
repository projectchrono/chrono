// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All right reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Alessandro Tasora, Radu Serban
// =============================================================================

#ifndef CHSHAFT_H
#define CHSHAFT_H

#include "chrono/physics/ChPhysicsItem.h"
#include "chrono/solver/ChVariablesShaft.h"

namespace chrono {

// Forward references (for parent hierarchy pointer)
class ChSystem;

///  Class for one-degree-of-freedom mechanical parts with associated
///  inertia (mass, or J moment of intertial for rotating parts).
///  In most cases these represent shafts that can be used to build 1D models
///  of power trains. This is more efficient than simulating power trains
///  modeled with full 3D ChBody objects.

class ChApi ChShaft : public ChPhysicsItem {

    // Tag needed for class factory in archive (de)serialization:
    CH_FACTORY_TAG(ChShaft)

  private:
    double torque;  ///< The torque acting on shaft (force, if used as linear DOF)

    double pos;       //< shaft angle
    double pos_dt;    //< shaft angular velocity
    double pos_dtdt;  //< shaft angular acceleration

    double inertia;  ///< shaft J moment of inertia (or mass, if used as linear DOF)

    ChVariablesShaft variables;  ///< used as an interface to the solver

    float max_speed;  ///< limit on linear speed

    float sleep_time;
    float sleep_minspeed;
    float sleep_minwvel;
    float sleep_starttime;

    bool fixed;
    bool limitspeed;
    bool sleeping;
    bool use_sleeping;

    unsigned int id;  ///< shaft id used for internal indexing

  public:
    ChShaft();
    ChShaft(const ChShaft& other);
    ~ChShaft() {}

    /// "Virtual" copy constructor (covariant return type).
    virtual ChShaft* Clone() const override { return new ChShaft(*this); }

    //
    // FLAGS
    //

    /// Sets the 'fixed' state of the shaft. If true, it does not rotate
    /// despite constraints, forces, etc.
    void SetShaftFixed(bool mev) {
        fixed = mev;
        variables.SetDisabled(mev);
    }
    bool GetShaftFixed() const { return fixed; }
    /// Trick. Set the maximum shaft velocity (beyond this limit it will
    /// be clamped). This is useful in virtual reality and real-time
    /// simulations.
    /// The realism is limited, but the simulation is more stable.
    void SetLimitSpeed(bool mlimit) { limitspeed = mlimit; }
    bool GetLimitSpeed() const { return limitspeed; }

    /// Trick. If use sleeping= true, shafts which do not rotate
    /// for too long time will be deactivated, for optimization.
    /// The realism is limited, but the simulation is faster.
    void SetUseSleeping(bool ms) { use_sleeping = ms; }
    bool GetUseSleeping() const { return use_sleeping; }

    /// Force the shaft in sleeping mode or not (usually this state change is not
    /// handled by users, anyway, because it is mostly automatic).
    void SetSleeping(bool ms) { sleeping = ms; }
    /// Tell if the shaft is actually in sleeping state.
    bool GetSleeping() const { return sleeping; }

    /// Put the shaft in sleeping state if requirements are satisfied.
    bool TrySleeping();

    /// Tell if the body is active, i.e. it is neither fixed to ground nor
    /// it is in sleep mode.
    bool IsActive() const { return !(sleeping || fixed); }

    //
    // FUNCTIONS
    //

    /// Set the shaft id for indexing (only used internally)
    void SetId(unsigned int identifier) { id = identifier; }

    /// Get the shaft id for indexing (only used internally)
    unsigned int GetId() const { return id; }

    /// Number of coordinates of the shaft
    virtual int GetDOF() override { return 1; }

    /// Returns reference to the encapsulated ChVariables,
    ChVariablesShaft& Variables() { return variables; }

    //
    // STATE FUNCTIONS
    //

    // (override/implement interfaces for global state vectors, see ChPhysicsItem for comments.)
    virtual void IntStateGather(const unsigned int off_x,
                                ChState& x,
                                const unsigned int off_v,
                                ChStateDelta& v,
                                double& T) override;
    virtual void IntStateScatter(const unsigned int off_x,
                                 const ChState& x,
                                 const unsigned int off_v,
                                 const ChStateDelta& v,
                                 const double T) override;
    virtual void IntStateGatherAcceleration(const unsigned int off_a, ChStateDelta& a) override;
    virtual void IntStateScatterAcceleration(const unsigned int off_a, const ChStateDelta& a) override;
    virtual void IntLoadResidual_F(const unsigned int off, ChVectorDynamic<>& R, const double c) override;
    virtual void IntLoadResidual_Mv(const unsigned int off,
                                    ChVectorDynamic<>& R,
                                    const ChVectorDynamic<>& w,
                                    const double c) override;
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
    // SOLVER FUNCTIONS
    //

    // Override/implement system functions of ChPhysicsItem
    // (to assemble/manage data for system solver)

    /// Sets the 'fb' part of the encapsulated ChVariables to zero.
    virtual void VariablesFbReset() override;

    /// Adds the current torques in the 'fb' part: qf+=torques*factor
    virtual void VariablesFbLoadForces(double factor = 1) override;

    /// Initialize the 'qb' part of the ChVariables with the
    /// current value of shaft speed. Note: since 'qb' is the unknown , this
    /// function seems unnecessary, unless used before VariablesFbIncrementMq()
    virtual void VariablesQbLoadSpeed() override;

    /// Adds M*q (masses multiplied current 'qb') to Fb, ex. if qb is initialized
    /// with v_old using VariablesQbLoadSpeed, this method can be used in
    /// timestepping schemes that do: M*v_new = M*v_old + forces*dt
    virtual void VariablesFbIncrementMq() override;

    /// Fetches the shaft speed from the 'qb' part of the ChVariables (does not
    /// updates the full shaft state) and sets it as the current shaft speed.
    /// If 'step' is not 0, also computes the approximate acceleration of
    /// the shaft using backward differences, that is  accel=(new_speed-old_speed)/step.
    virtual void VariablesQbSetSpeed(double step = 0) override;

    /// Increment shaft position by the 'qb' part of the ChVariables,
    /// multiplied by a 'step' factor.
    ///     pos+=qb*step
    virtual void VariablesQbIncrementPosition(double step) override;

    /// Tell to a system descriptor that there are variables of type
    /// ChVariables in this object (for further passing it to a solver)
    virtual void InjectVariables(ChSystemDescriptor& mdescriptor) override;

    // Other functions

    /// Set no speed and no accelerations (but does not change the position)
    void SetNoSpeedNoAcceleration() override;

    /// Set the torque applied to the shaft
    void SetAppliedTorque(double mtorque) { torque = mtorque; }
    /// Get the torque applied to the shaft
    double GetAppliedTorque() const { return torque; }

    /// Set the angular position
    void SetPos(double mp) { pos = mp; }
    /// Get the angular position
    double GetPos() const { return pos; }

    /// Set the angular velocity
    void SetPos_dt(double mp) { pos_dt = mp; }
    /// Get the angular velocity
    double GetPos_dt() const { return pos_dt; }

    /// Set the angular acceleration
    void SetPos_dtdt(double mp) { pos_dtdt = mp; }
    /// Get the angular acceleration
    double GetPos_dtdt() const { return pos_dtdt; }

    /// Inertia of the shaft. Must be positive.
    /// Try not to mix bodies with too high/too low values of mass, for numerical stability.
    void SetInertia(double newJ);
    double GetInertia() const { return inertia; }

    /// Trick. Set the maximum velocity (beyond this limit it will
    /// be clamped). This is useful in virtual reality and real-time
    /// simulations, to increase robustness at the cost of realism.
    /// This limit is active only if you set  SetLimitSpeed(true);
    void SetMaxSpeed(float m_max_speed) { max_speed = m_max_speed; }
    float GetMaxSpeed() const { return max_speed; }

    /// When this function is called, the speed of the shaft is clamped
    /// into limits posed by max_speed and max_wvel  - but remember to
    /// put the shaft in the SetLimitSpeed(true) mode.
    void ClampSpeed();

    /// Set the amount of time which must pass before going automatically in
    /// sleep mode when the shaft has very small movements.
    void SetSleepTime(float m_t) { sleep_time = m_t; }
    float GetSleepTime() const { return sleep_time; }

    /// Set the max linear speed to be kept for 'sleep_time' before freezing.
    void SetSleepMinSpeed(float m_t) { sleep_minspeed = m_t; }
    float GetSleepMinSpeed() const { return sleep_minspeed; }

    /// Set the max linear speed to be kept for 'sleep_time' before freezing.
    void SetSleepMinWvel(float m_t) { sleep_minwvel = m_t; }
    float GetSleepMinWvel() const { return sleep_minwvel; }

    //
    // UPDATE FUNCTIONS
    //

    /// Update all auxiliary data of the shaft at given time
    virtual void Update(double mytime, bool update_assets = true) override;

    //
    // SERIALIZATION
    //

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOUT(ChArchiveOut& marchive) override;

    /// Method to allow deserialization of transient data from archives.
    virtual void ArchiveIN(ChArchiveIn& marchive) override;
};

CH_CLASS_VERSION(ChShaft,0)


}  // end namespace chrono

#endif
