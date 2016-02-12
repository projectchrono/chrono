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

#ifndef CHSHAFT_H
#define CHSHAFT_H

//////////////////////////////////////////////////
//
//   ChShaft.h
//
//   Class for one-degree-of-freedom part, that is
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

#include "physics/ChPhysicsItem.h"
#include "lcp/ChLcpVariablesShaft.h"

namespace chrono {

// Forward references (for parent hierarchy pointer)

class ChSystem;

///
///  Class for one-degree-of-freedom mechanical parts with associated
///  inertia (mass, or J moment of intertial for rotating parts).
///  In most cases these represent shafts that can be used to build 1D models
///  of power trains. This is more efficient than simulating power trains
///  modeled with full 3D ChBody objects.
///

class ChApi ChShaft : public ChPhysicsItem {
    // Chrono simulation of RTTI, needed for serialization
    CH_RTTI(ChShaft, ChPhysicsItem);

  private:
    //
    // DATA
    //

    double torque;  // The torque acting on shaft (force, if used as linear DOF)

    double pos;       // angle
    double pos_dt;    // angular velocity
    double pos_dtdt;  // angular acceleration

    double inertia;  // the J moment of inertia (or mass, if used as linear DOF)

    // used as an interface to the LCP solver.
    ChLcpVariablesShaft variables;

    float max_speed;  // limit on linear speed (useful for VR & videagames)

    float sleep_time;
    float sleep_minspeed;
    float sleep_minwvel;
    float sleep_starttime;

    bool fixed;
    bool limitspeed;
    bool sleeping;
    bool use_sleeping;

    unsigned int id;  // shaft id used for indexing

  public:
    //
    // CONSTRUCTORS
    //

    /// Build a shaft.
    ChShaft();
    /// Destructor
    ~ChShaft();

    /// Copy from another ChShaft.
    void Copy(ChShaft* source);

    //
    // FLAGS
    //

    /// Sets the 'fixed' state of the shaft. If true, it does not rotate
    /// despite constraints, forces, etc.
    void SetShaftFixed(bool mev) {
        this->fixed = mev;
        variables.SetDisabled(mev);
    }
    bool GetShaftFixed() const { return this->fixed; }
    /// Trick. Set the maximum shaft velocity (beyond this limit it will
    /// be clamped). This is useful in virtual reality and real-time
    /// simulations.
    /// The realism is limited, but the simulation is more stable.
    void SetLimitSpeed(bool mlimit) { this->limitspeed = mlimit; }
    bool GetLimitSpeed() const { return this->limitspeed; };

    /// Trick. If use sleeping= true, shafts which do not rotate
    /// for too long time will be deactivated, for optimization.
    /// The realism is limited, but the simulation is faster.
    void SetUseSleeping(bool ms) { this->use_sleeping = ms; }
    bool GetUseSleeping() const { return this->use_sleeping; }

    /// Force the shaft in sleeping mode or not (usually this state change is not
    /// handled by users, anyway, because it is mostly automatic).
    void SetSleeping(bool ms) { this->sleeping = ms; }
    /// Tell if the shaft is actually in sleeping state.
    bool GetSleeping() const { return this->sleeping; }

    /// Put the shaft in sleeping state if requirements are satisfied.
    bool TrySleeping();

    /// Tell if the body is active, i.e. it is neither fixed to ground nor
    /// it is in sleep mode.
    bool IsActive() const { return !(sleeping || fixed); }

    //
    // FUNCTIONS
    //

    /// Set the shaft id
    void SetId(unsigned int identifier) { id = identifier; }

    /// Get the shaft id
    unsigned int GetId() const { return id; }

    /// Number of coordinates of the shaft
    virtual int GetDOF() { return 1; }

    /// Returns reference to the encapsulated ChLcpVariables,
    ChLcpVariablesShaft& Variables() { return variables; }

    //
    // STATE FUNCTIONS
    //

    // (override/implement interfaces for global state vectors, see ChPhysicsItem for comments.)
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
    virtual void IntLoadResidual_F(const unsigned int off, ChVectorDynamic<>& R, const double c);
    virtual void IntLoadResidual_Mv(const unsigned int off,
                                    ChVectorDynamic<>& R,
                                    const ChVectorDynamic<>& w,
                                    const double c);
    virtual void IntToLCP(const unsigned int off_v,
                          const ChStateDelta& v,
                          const ChVectorDynamic<>& R,
                          const unsigned int off_L,
                          const ChVectorDynamic<>& L,
                          const ChVectorDynamic<>& Qc);
    virtual void IntFromLCP(const unsigned int off_v, ChStateDelta& v, const unsigned int off_L, ChVectorDynamic<>& L);

    //
    // LCP FUNCTIONS
    //

    // Override/implement LCP system functions of ChPhysicsItem
    // (to assembly/manage data for LCP system solver)

    /// Sets the 'fb' part of the encapsulated ChLcpVariables to zero.
    void VariablesFbReset();

    /// Adds the current torques in the 'fb' part: qf+=torques*factor
    void VariablesFbLoadForces(double factor = 1.);

    /// Initialize the 'qb' part of the ChLcpVariables with the
    /// current value of shaft speed. Note: since 'qb' is the unknown of the LCP, this
    /// function seems unuseful, unless used before VariablesFbIncrementMq()
    void VariablesQbLoadSpeed();

    /// Adds M*q (masses multiplied current 'qb') to Fb, ex. if qb is initialized
    /// with v_old using VariablesQbLoadSpeed, this method can be used in
    /// timestepping schemes that do: M*v_new = M*v_old + forces*dt
    void VariablesFbIncrementMq();

    /// Fetches the shaft speed from the 'qb' part of the ChLcpVariables (does not
    /// updates the full shaft state) and sets it as the current shaft speed.
    /// If 'step' is not 0, also computes the approximate acceleration of
    /// the shaft using backward differences, that is  accel=(new_speed-old_speed)/step.
    void VariablesQbSetSpeed(double step = 0.);

    /// Increment shaft position by the 'qb' part of the ChLcpVariables,
    /// multiplied by a 'step' factor.
    ///     pos+=qb*step
    void VariablesQbIncrementPosition(double step);

    /// Tell to a system descriptor that there are variables of type
    /// ChLcpVariables in this object (for further passing it to a LCP solver)
    virtual void InjectVariables(ChLcpSystemDescriptor& mdescriptor);

    // Other functions

    /// Set no speed and no accelerations (but does not change the position)
    void SetNoSpeedNoAcceleration();

    /// Set the torque applied to the shaft
    void SetAppliedTorque(double mtorque) { this->torque = mtorque; }
    /// Get the torque applied to the shaft
    double GetAppliedTorque() const { return torque; }

    /// Set the angular position
    void SetPos(double mp) { this->pos = mp; }
    /// Get the angular position
    double GetPos() const { return this->pos; }

    /// Set the angular velocity
    void SetPos_dt(double mp) { this->pos_dt = mp; }
    /// Get the angular velocity
    double GetPos_dt() const { return this->pos_dt; }

    /// Set the angular acceleration
    void SetPos_dtdt(double mp) { this->pos_dtdt = mp; }
    /// Get the angular acceleration
    double GetPos_dtdt() const { return this->pos_dtdt; }

    /// Inertia of the shaft. Must be positive.
    /// Try not to mix bodies with too high/too low values of mass, for numerical stability.
    void SetInertia(double newJ);
    double GetInertia() const { return this->inertia; }

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
