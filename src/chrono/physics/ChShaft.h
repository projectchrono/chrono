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

#ifndef CHSHAFT_H
#define CHSHAFT_H

#include "chrono/physics/ChPhysicsItem.h"
#include "chrono/physics/ChLoadable.h"
#include "chrono/solver/ChVariablesShaft.h"

namespace chrono {

// Forward references (for parent hierarchy pointer)
class ChSystem;

/// Class for one-degree-of-freedom mechanical parts with associated  inertia (mass or moment of rotational inertia).
/// In most cases these represent shafts that can be used to build 1D models of power trains. This is more efficient
/// than simulating power trains  modeled with full 3D ChBody objects.
class ChApi ChShaft : public ChPhysicsItem, public ChLoadable {
  public:
    ChShaft();
    ChShaft(const ChShaft& other);
    ~ChShaft() {}

    /// "Virtual" copy constructor (covariant return type).
    virtual ChShaft* Clone() const override { return new ChShaft(*this); }

    /// Set no speed and no accelerations (but does not change the position)
    void ForceToRest() override;

    /// Set the load applied to the shaft.
    /// For shafts with a rotational degree of freedom, this represents the applied torque.
    /// For shafts with a translational degree of freedom, this is the applied force.
    void SetAppliedLoad(double applied_load) { load = applied_load; }

    /// Get the load applied to the shaft.
    /// For shafts with a rotational degree of freedom, this represents the applied torque.
    /// For shafts with a translational degree of freedom, this is the applied force.
    double GetAppliedLoad() const { return load; }

    /// Set the shaft position.
    /// For shafts with a rotational degree of freedom, this represents an angle.
    /// For shafts with a translational degree of freedom, this represents a displacement.
    void SetPos(double mp) { pos = mp; }

    /// Get the shaft position.
    /// For shafts with a rotational degree of freedom, this represents an angle.
    /// For shafts with a translational degree of freedom, this represents a displacement.
    double GetPos() const { return pos; }

    /// Set the shaft velocity.
    /// For shafts with a rotational degree of freedom, this represents an angular velocity.
    /// For shafts with a translational degree of freedom, this represents a linear velocity.
    void SetPosDt(double mp) { pos_dt = mp; }

    /// Get the shaft velocity.
    /// For shafts with a rotational degree of freedom, this represents an angular velocity.
    /// For shafts with a translational degree of freedom, this represents a linear velocity.
    double GetPosDt() const { return pos_dt; }

    /// Set the shaft acceleration.
    /// For shafts with a rotational degree of freedom, this represents an angular acceleration.
    /// For shafts with a translational degree of freedom, this represents a linear acceleration.
    void SetPosDt2(double mp) { pos_dtdt = mp; }

    /// Get the shaft acceleration.
    /// For shafts with a rotational degree of freedom, this represents an angular acceleration.
    /// For shafts with a translational degree of freedom, this represents a linear acceleration.
    double GetPosDt2() const { return pos_dtdt; }

    /// Inertia property of the shaft (must be a positive value).
    /// For rotational shafts, this is a moment of inertia. For translational shafts, this is a mass.
    /// Try not to mix physics items with too widely different inertia values, for numerical stability.
    void SetInertia(double newJ);

    /// Get the shaft inertia property.
    double GetInertia() const { return inertia; }

    /// Set the maximum shaft speed (beyond this limit, the speed it will be clamped).
    void SetMaxSpeed(float m_max_speed) { max_speed = m_max_speed; }

    /// Clamp the shaft speed shaft is clamped to the range specified by SetMaxSpeed.
    /// The speed limit must be enabled (i.e., call first SetLimitSpeed(true)).
    void ClampSpeed();

    /// Set the duration before automatically placing the shaft in sleep mode when shaft movement is small.
    void SetSleepTime(float time) { sleep_time = time; }

    /// Set the max linear speed to be kept for 'sleep_time' before freezing.
    void SetSleepMinSpeed(float speed) { sleep_minspeed = speed; }

    /// Fix or release this shaft.
    /// If set to true, the shaft does not move, regardless of applied loads.
    void SetFixed(bool state);

    /// Return true if the shaft is set to fixed.
    bool IsFixed() const { return fixed; }

    /// Enable limit on shaft speed (default: false).
    void SetLimitSpeed(bool limit) { limitspeed = limit; }

    /// If sleeping is allowed, shafts which do not rotate for too long time will be deactivated.
    void SetSleepingAllowed(bool state) { use_sleeping = state; }

    ///  Return true if this shaft is allowed to go to sleep.
    bool IsSleepingAllowed() const { return use_sleeping; }

    /// Force the shaft in sleeping mode or not.
    /// Note: Usually this state change is not handled by users, because it is mostly automatic.
    void SetSleeping(bool state) { sleeping = state; }

    /// Tell if the shaft is actually in sleeping state.
    bool IsSleeping() const { return sleeping; }

    /// Put the shaft in sleeping state if requirements are satisfied.
    bool TrySleeping();

    /// Return true if the shaft is currently active and thereofre included into the system solver.
    /// A shaft is inactive if it is fixed to ground or is in sleep mode.
    virtual bool IsActive() const override { return !(sleeping || fixed); }

    /// Returns reference to the encapsulated ChVariables,
    ChVariablesShaft& Variables() { return variables; }

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOut(ChArchiveOut& archive_out) override;

    /// Method to allow deserialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& archive_in) override;

  public:
    /// Get the unique sequential shaft index (internal use only).
    unsigned int GetIndex() const { return index; }

    /// Number of coordinates of the shaft
    virtual unsigned int GetNumCoordsPosLevel() override { return 1; }

    // Solver, integrator, and loadable interfaces

    virtual void InjectVariables(ChSystemDescriptor& descriptor) override;

    virtual void Update(double time, bool update_assets) override;

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
    virtual void IntLoadResidual_F(const unsigned int off, ChVectorDynamic<>& R, const double c) override;
    virtual void IntLoadResidual_Mv(const unsigned int off,
                                    ChVectorDynamic<>& R,
                                    const ChVectorDynamic<>& w,
                                    const double c) override;
    virtual void IntLoadLumpedMass_Md(const unsigned int off,
                                      ChVectorDynamic<>& Md,
                                      double& err,
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

    virtual void VariablesFbReset() override;
    virtual void VariablesFbLoadForces(double factor = 1) override;
    virtual void VariablesQbLoadSpeed() override;
    virtual void VariablesFbIncrementMq() override;
    virtual void VariablesQbSetSpeed(double step = 0) override;
    virtual void VariablesQbIncrementPosition(double step) override;

    virtual unsigned int GetLoadableNumCoordsPosLevel() override { return 1; }
    virtual unsigned int GetLoadableNumCoordsVelLevel() override { return 1; }
    virtual void LoadableGetStateBlockPosLevel(int block_offset, ChState& mD) override { mD(block_offset) = GetPos(); }
    virtual void LoadableGetStateBlockVelLevel(int block_offset, ChStateDelta& mD) override {
        mD(block_offset) = GetPosDt();
    }
    virtual void LoadableStateIncrement(const unsigned int off_x,
                                        ChState& x_new,
                                        const ChState& x,
                                        const unsigned int off_v,
                                        const ChStateDelta& Dv) override {
        x_new(off_x) = x(off_x) + Dv(off_v);
    }
    virtual unsigned int GetNumFieldCoords() override { return 1; }
    virtual unsigned int GetNumSubBlocks() override { return 1; }
    virtual unsigned int GetSubBlockOffset(unsigned int nblock) override { return GetOffset_w(); }
    virtual unsigned int GetSubBlockSize(unsigned int nblock) override { return 1; }
    virtual bool IsSubBlockActive(unsigned int nblock) const override { return true; }
    virtual void LoadableGetVariables(std::vector<ChVariables*>& mvars) override { mvars.push_back(&Variables()); };

  private:
    double load;  ///< load acting on shaft (torque for rotational DOF, force for linear DOF)

    double pos;       //< shaft position (angle or displacement)
    double pos_dt;    //< shaft velocity (angular velocity or linear velocity)
    double pos_dtdt;  //< shaft acceleration (angular acceleration or linear acceleration)

    double inertia;  ///< shaft inertia property (moment of inertia for rotational DOF, mass for linear DOF)

    ChVariablesShaft variables;  ///< used as an interface to the solver

    float max_speed;  ///< limit on shaft speed

    float sleep_time;
    float sleep_minspeed;
    float sleep_starttime;

    bool fixed;
    bool limitspeed;
    bool sleeping;
    bool use_sleeping;

    unsigned int index;  ///< unique sequential body identifier, used for indexing (internal use only)

    // Friend classes with private access
    friend class ChSystem;
    friend class ChSystemMulticore;
};

CH_CLASS_VERSION(ChShaft, 0)

}  // end namespace chrono

#endif
