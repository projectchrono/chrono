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

#ifndef CHBODY_H
#define CHBODY_H

#include <cmath>

#include "chrono/physics/ChBodyFrame.h"
#include "chrono/physics/ChContactable.h"
#include "chrono/physics/ChForce.h"
#include "chrono/physics/ChLoadable.h"
#include "chrono/physics/ChMarker.h"
#include "chrono/physics/ChPhysicsItem.h"
#include "chrono/collision/ChCollisionSystem.h"
#include "chrono/solver/ChConstraint.h"
#include "chrono/solver/ChVariablesBodyOwnMass.h"

namespace chrono {

// Forward references (for parent hierarchy pointer)

class ChSystem;

/// Class for Rigid Bodies
///
/// A rigid body is an entity with mass and inertia properties moving in the 3D space.
/// Optionally, an object of the ChBody class (or derived) can be:
/// - involved in collision, if a collision model is provided (@ref collisions) and the collision is enabled;
/// - visualized, if a visual model is provided and a visualization system is available (@ref visualization_system);
/// - be constrained by means of ChLink objects (@ref links);
/// - be loaded by ChLoad objects (@ref loads);
/// - be used in coordinate transformation, being itself inherited from ChFrameMoving;
///
/// Location and orientation of the ChBody refer to its Center of Mass (CoM).
/// Since no additional frame is available, also visual and collision shapes refer to the same frame.
/// Derived classes might offer additional frames (e.g. @ref ChBodyAuxRef).
///
/// Further info at the @ref rigid_bodies manual page.

class ChApi ChBody : public ChPhysicsItem, public ChBodyFrame, public ChContactable, public ChLoadableUVW {
  public:
    ChBody();
    ChBody(const ChBody& other);

    virtual ~ChBody();

    /// "Virtual" copy constructor (covariant return type).
    virtual ChBody* Clone() const override { return new ChBody(*this); }

    /// Sets the 'fixed' state of the body.
    /// If true, the body does not move with respect to the absolute reference frame.
    /// Its state is also removed by the system, thus reducing the global size of the system.
    /// If this is not desired, use ChLinkMateFix or ChLinkLockLock instead.
    void SetFixed(bool state);

    /// Return true if this body is fixed to ground.
    bool IsFixed() const;

    /// Enable/disable the collision for this rigid body.
    void EnableCollision(bool state);

    /// Return true if collision is enabled for this body.
    virtual bool IsCollisionEnabled() const override;

    /// Enable the maximum linear speed limit (default: false).
    void SetLimitSpeed(bool state);

    /// Enable/disable the gyroscopic torque (quadratic term).
    /// This is useful in virtual reality and real-time simulations, where objects that spin too fast with non-uniform
    /// inertia tensors (e.g., thin cylinders) might cause the integration to diverge quickly. The realism is limited,
    /// but the simulation is more stable.
    /// By default the gyroscopic torque is enabled.
    void SetUseGyroTorque(bool state);

    /// Return true if gyroscopic torque is used (default=true).
    bool IsUsingGyroTorque() const;

    /// Enable/disable option for setting bodies to 'sleep'.
    /// If sleeping is allowed, bodies which stay in same place for long enough time will be deactivated.
    /// By default the sleeping is enabled.
    void SetSleepingAllowed(bool state);

    /// Return true if 'sleep' mode is allowed for this specific body.
    bool IsSleepingAllowed() const;

    /// Force the body in sleeping mode or not.
    /// Usually this state change is handled internally.
    void SetSleeping(bool state);

    /// Return true if this body is currently in 'sleep' mode.
    bool IsSleeping() const;

    /// Test if a body could go in sleeping state if requirements are satisfied.
    /// Return true if state could be changed from no sleep to sleep.
    bool TrySleeping();

    /// Return true if the body is currently active and therefore included into the system solver.
    /// A body is inactive if it is fixed to ground or in sleep mode.
    virtual bool IsActive() const override;

    /// Get the unique sequential body index (internal use only).
    unsigned int GetIndex() { return index; }

    /// Number of coordinates of body: 7 because uses quaternions for rotation.
    virtual unsigned int GetNumCoordsPosLevel() override { return 7; }

    /// Number of coordinates of body: 6 because derivatives use angular velocity.
    virtual unsigned int GetNumCoordsVelLevel() override { return 6; }

    /// Return a reference to the encapsulated ChVariablesBody, representing states (pos, speed, or accel.) and forces.
    /// The ChVariablesBodyOwnMass is the interface to the system solver.
    virtual ChVariables& Variables() override { return variables; }

    /// Set no speed and no accelerations (but does not change the position).
    void ForceToRest() override;

    /// Add the body collision model (if any) to the provided collision system.
    virtual void AddCollisionModelsToSystem(ChCollisionSystem* coll_sys) const override;

    /// Remove the body collision model (if any) from the provided collision system.
    virtual void RemoveCollisionModelsFromSystem(ChCollisionSystem* coll_sys) const override;

    /// Synchronize the position and bounding box of the body collision model (if any).
    virtual void SyncCollisionModels() override;

    /// Get the rigid body coordinate system that represents the GOG (Center of Gravity).
    /// The mass and inertia tensor are defined with respect to this coordinate system, which is also assumed to be the
    /// default coordinates of the body. By default, a call to body.GetPos() is equivalent to
    /// body.GetFrame_COG_abs().GetPos().
    virtual const ChFrameMoving<>& GetFrameCOMToAbs() const { return *this; }

    /// Get the rigid body coordinate system that is used for defining the collision shapes and the ChMarker objects.
    /// For the base ChBody, this is always the same reference of the COG.
    virtual const ChFrameMoving<>& GetFrameRefToAbs() const { return *this; }

    /// Get the reference frame (expressed in and relative to the absolute frame) of the visual model.
    /// For a ChBody, this is the main coordinate system of the rigid body.
    virtual ChFrame<> GetVisualModelFrame(unsigned int nclone = 0) const override { return GetFrameRefToAbs(); }

    /// Get the axis-aligned bounding (AABB) box of the object.
    /// The body AABB is the AABB of its collision model (if any).
    virtual ChAABB GetTotalAABB() const override;

    // DATABASE HANDLING

    /// Attach a marker to this body.
    void AddMarker(std::shared_ptr<ChMarker> amarker);

    /// Attach a force to this body.
    void AddForce(std::shared_ptr<ChForce> aforce);

    /// Remove a specific marker from this body.
    /// Warning: linear time search.
    void RemoveMarker(std::shared_ptr<ChMarker> amarker);

    /// Remove a specific force from this body.
    /// Warning: linear time search.
    void RemoveForce(std::shared_ptr<ChForce> aforce);

    /// Remove all markers at once.
    /// Faster than doing multiple RemoveForce().
    void RemoveAllForces();

    /// Remove all markers at once.
    /// Faster than doing multiple RemoveForce().
    void RemoveAllMarkers();

    /// Find a marker by its name.
    std::shared_ptr<ChMarker> SearchMarker(const std::string& name) const;

    /// Find a marker by its identifier.
    std::shared_ptr<ChMarker> SearchMarker(int id) const;

    /// Find a force by its name.
    std::shared_ptr<ChForce> SearchForce(const std::string& name) const;

    /// Gets the list of children markers.
    /// NOTE: to modify this list, use the appropriate Remove..
    /// and Add.. functions.
    const std::vector<std::shared_ptr<ChMarker>>& GetMarkers() const { return marklist; }

    /// Gets the list of children forces.
    /// NOTE: to modify this list, use the appropriate Remove..
    /// and Add.. functions.
    const std::vector<std::shared_ptr<ChForce>>& GetForces() const { return forcelist; }

    /// Set the body mass.
    /// Try not to mix bodies with too high/too low values of mass, for numerical stability.
    void SetMass(double newmass) { variables.SetBodyMass(newmass); }

    /// Get the body mass.
    double GetMass() { return variables.GetBodyMass(); }

    /// Set the inertia tensor of the body.
    /// The provided 3x3 matrix should be symmetric and contain the inertia tensor, expressed in the local coordinate
    /// system:
    /// <pre>
    ///               [ int{y^2+z^2}dm    -int{xy}dm    -int{xz}dm    ]
    /// newXInertia = [                  int{x^2+z^2}   -int{yz}dm    ]
    ///               [                                int{x^2+y^2}dm ]
    /// </pre>
    void SetInertia(const ChMatrix33<>& newXInertia);

    /// Get the inertia tensor, expressed in the local coordinate system.
    /// The return 3x3 symmetric matrix contains the following values:
    /// <pre>
    ///  [ int{y^2+z^2}dm    -int{xy}dm    -int{xz}dm    ]
    ///  [                  int{x^2+z^2}   -int{yz}dm    ]
    ///  [                                int{x^2+y^2}dm ]
    /// </pre>
    const ChMatrix33<>& GetInertia() const { return variables.GetBodyInertia(); }

    /// Get the inverse of the inertia matrix.
    const ChMatrix33<>& GetInvInertia() const { return variables.GetBodyInvInertia(); }

    /// Set the diagonal part of the inertia tensor (Ixx, Iyy, Izz values).
    /// The provided 3x1 vector should contain the moments of inertia, expressed in the local coordinate frame:
    /// <pre>
    /// iner = [  int{y^2+z^2}dm   int{x^2+z^2}   int{x^2+y^2}dm ]
    /// </pre>
    void SetInertiaXX(const ChVector3d& iner);

    /// Get the diagonal part of the inertia tensor (Ixx, Iyy, Izz values).
    /// The return 3x1 vector contains the following values:
    /// <pre>
    /// [  int{y^2+z^2}dm   int{x^2+z^2}   int{x^2+y^2}dm ]
    /// </pre>
    ChVector3d GetInertiaXX() const;

    /// Set the off-diagonal part of the inertia tensor (Ixy, Ixz, Iyz values).
    /// The provided 3x1 vector should contain the products of inertia,
    /// expressed in the local coordinate frame:
    /// <pre>
    /// iner = [ -int{xy}dm   -int{xz}dm   -int{yz}dm ]
    /// </pre>
    void SetInertiaXY(const ChVector3d& iner);

    /// Get the extra-diagonal part of the inertia tensor (Ixy, Ixz, Iyz values).
    /// The return 3x1 vector contains the following values:
    /// <pre>
    /// [ -int{xy}dm   -int{xz}dm   -int{yz}dm ]
    /// </pre>
    ChVector3d GetInertiaXY() const;

    /// Set the maximum linear speed (beyond this limit it will be clamped).
    /// This speed limit is active only if you set  SetLimitSpeed(true);
    void SetMaxLinVel(float m_max_speed) { max_speed = m_max_speed; }
    float GetMaxLinVel() const { return max_speed; }

    /// Set the maximum angular speed (beyond this limit it will be clamped).
    /// This speed limit is active only if you set  SetLimitSpeed(true);
    void SetMaxAngVel(float m_max_wvel) { max_wvel = m_max_wvel; }
    float GetMaxAngVel() const { return max_wvel; }

    /// Clamp the body speed to the provided limits.
    /// When this function is called, the speed of the body is clamped
    /// to the range specified by max_speed and max_wvel. Remember to
    /// put the body in the SetLimitSpeed(true) mode.
    void ClampSpeed();

    /// Set the amount of time which must pass before going automatically in
    /// sleep mode when the body has very small movements.
    void SetSleepTime(float m_t) { sleep_time = m_t; }
    float GetSleepTime() const { return sleep_time; }

    /// Set the max linear speed to be kept for 'sleep_time' before freezing.
    void SetSleepMinLinVel(float m_t) { sleep_minspeed = m_t; }
    float GetSleepMinLinVel() const { return sleep_minspeed; }

    /// Set the max linear speed to be kept for 'sleep_time' before freezing.
    void SetSleepMinAngVel(float m_t) { sleep_minwvel = m_t; }
    float GetSleepMinAngVel() const { return sleep_minwvel; }

    /// Computes the 4x4 inertia tensor in quaternion space, if needed.
    void ComputeQInertia(ChMatrix44<>& mQInertia);

    /// Computes the gyroscopic torque. In fact, in sake of highest
    /// speed, the gyroscopic torque isn't automatically updated each time a
    /// SetCoordsys() or SetCoordsysDt() etc. is called, but only if necessary,
    /// for each UpdateState().
    void ComputeGyro();

    // UTILITIES FOR FORCES/TORQUES:

    /// Add a new force and torque accumulator.
    /// An arbitrary number of accumulators can be specified for a body, differentiated by their index `idx` (as
    /// returned by this function). At each step in a simulation loop, a typical use of these accumulators is: 
    /// <pre>
    ///    EmptyAccumulator(idx);
    ///    AccumulateForce(idx, ...);
    ///    AccumulateTorque(idx, ...);
    ///    ...
    ///    AccumulateForce(idx, ...);
    ///    ...
    /// </pre>
    unsigned int AddAccumulator();

    /// Clear the accumulator with specififed index.
    void EmptyAccumulator(unsigned int idx);

    /// Include a concentrated body force in the specified accumulator.
    /// The accumulator force and torque are incremented with the specified applied force and induced moment at the body
    /// COM. It is the caller's responsibility to clear the force and torque accumulator before reuse (e.g., at each
    /// integration step). If local = true, the provided applied force and application point are assumed to be expressed
    /// in body coordinates; otherwise (local = false), these quantities are assumed to be expressed in absolute
    /// coordinates.
    void AccumulateForce(unsigned int idx,              ///< index of the accumulator
                         const ChVector3d& force,       ///< applied force
                         const ChVector3d& appl_point,  ///< application point
                         bool local                     ///< force and point expressed in body local frame?
    );

    /// Include a body torque in the specified accumulator.
    /// The accumulator torque is incremented with the specified torque. It is the caller's responsibility to clear the
    /// force and torque accumulator before reuse (e.g., at each integration step). If local = true, the provided
    /// applied torque is assumed to be expressed in body coordinates; otherwise (local = false), it is assumed to be
    /// expressed in absolute coordinates.
    void AccumulateTorque(unsigned int idx,          ///< index of the accumulator
                          const ChVector3d& torque,  ///< applied torque
                          bool local                 ///< torque expressed in body local frame?
    );

    /// Return the current value of the accumulated force from the specified accumulator.
    /// Note that this is a resultant force as applied to the COM and expressed in the absolute frame.
    const ChVector3d& GetAccumulatedForce(unsigned int idx) const;

    /// Return the current value of the accumulated torque from the specified accumulator.
    /// Note that this is a resultant torque expressed in the body local frame.
    const ChVector3d& GetAccumulatedTorque(unsigned int idx) const;

    // This function should *not* exist. It's here only because of the asinine implementation of ChModalAssembly!
    // Collects the resultant wrench across all accumulators.
    const ChWrenchd GetAccumulatorWrench() const;

    // UPDATE FUNCTIONS

    /// Update all children markers of the rigid body, at current body state
    void UpdateMarkers(double time, bool update_assets);

    /// Update all children forces of the rigid body, at current body state.
    void UpdateForces(double time, bool update_assets);

    /// Update all auxiliary data of the rigid body and of its children (markers, forces..), at given time
    virtual void Update(double time, bool update_assets) override;

    /// Return the resultant applied force on the body.
    /// This resultant force includes all external applied loads acting on this body (from gravity, loads, springs,
    /// etc). However, this does *not* include any constraint forces. In particular, contact forces are not included if
    /// using the NSC formulation, but are included when using the SMC formulation.
    ChVector3d GetAppliedForce();

    /// Return the resultant applied torque on the body.
    /// This resultant torque includes all external applied loads acting on this body (from gravity, loads, springs,
    /// etc). However, this does *not* include any constraint forces. In particular, contact torques are not included if
    /// using the NSC formulation, but are included when using the SMC formulation.
    ChVector3d GetAppliedTorque();

    /// Get the resultant contact force acting on this body.
    ChVector3d GetContactForce();

    /// Get the resultant contact torque acting on this body.
    ChVector3d GetContactTorque();

    /// This is only for backward compatibility
    virtual ChPhysicsItem* GetPhysicsItem() override { return this; }

    // SERIALIZATION

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOut(ChArchiveOut& archive_out) override;

    /// Method to allow deserialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& archive_in) override;

  public:
    // Public functions for ADVANCED use.
    // For example, access to these methods may be needed in implementing custom loads

    /// Get the pointers to the contained ChVariables, appending to the mvars vector.
    virtual void LoadableGetVariables(std::vector<ChVariables*>& mvars) override;

    /// Increment all DOFs using a delta.
    virtual void LoadableStateIncrement(const unsigned int off_x,
                                        ChState& x_new,
                                        const ChState& x,
                                        const unsigned int off_v,
                                        const ChStateDelta& Dv) override;

    /// Gets all the DOFs packed in a single vector (position part)
    virtual void LoadableGetStateBlockPosLevel(int block_offset, ChState& mD) override;

    /// Gets all the DOFs packed in a single vector (speed part)
    virtual void LoadableGetStateBlockVelLevel(int block_offset, ChStateDelta& mD) override;

    /// Evaluate Q=N'*F, for Q generalized lagrangian load, where N is some type of matrix evaluated at point P(U,V,W)
    /// assumed in absolute coordinates, and F is a load assumed in absolute coordinates. det[J] is unused.
    virtual void ComputeNF(
        const double U,              ///< x coordinate of application point in absolute space
        const double V,              ///< y coordinate of application point in absolute space
        const double W,              ///< z coordinate of application point in absolute space
        ChVectorDynamic<>& Qi,       ///< Return result of N'*F  here, maybe with offset block_offset
        double& detJ,                ///< Return det[J] here
        const ChVectorDynamic<>& F,  ///< Input F vector, size is 6, it is {Force,Torque} in absolute coords.
        ChVectorDynamic<>* state_x,  ///< if != 0, update state (pos. part) to this, then evaluate Q
        ChVectorDynamic<>* state_w   ///< if != 0, update state (speed part) to this, then evaluate Q
        ) override;

  protected:
    struct WrenchAccumulator {
        ChVector3d force;   ///< force accumulator, applied to COM (in absolute coords)
        ChVector3d torque;  ///< torque accumulator (in body local coords)
    };

    std::vector<std::shared_ptr<ChMarker>> marklist;  ///< list of markers
    std::vector<std::shared_ptr<ChForce>> forcelist;  ///< list of forces

    ChVector3d gyro;  ///< gyroscopic torque, i.e. Qm = Wvel x (XInertia*Wvel)

    ChVector3d Xforce;   ///< force  acting on body, applied to COM (in absolute coords)
    ChVector3d Xtorque;  ///< torque acting on body  (in body local coords)

    std::vector<WrenchAccumulator> accumulators;

    ChVariablesBodyOwnMass variables;  ///< interface to solver (store inertia and coordinates)

    float max_speed;  ///< limit on linear speed
    float max_wvel;   ///< limit on angular velocity

    float sleep_time;
    float sleep_minspeed;
    float sleep_minwvel;
    float sleep_starttime;

    unsigned int index;  ///< unique sequential body identifier, used for indexing (internal use only)

  private:
    // STATE FUNCTIONS

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

    // SOLVER FUNCTIONS

    // Override/implement solver system functions of ChPhysicsItem
    // (to assemble/manage data for system solver)

    /// Sets the 'fb' part of the encapsulated ChVariablesBodyOwnMass to zero.
    virtual void VariablesFbReset() override;

    /// Adds the current forces applied to body (including gyroscopic torque) in
    /// encapsulated ChVariablesBody, in the 'fb' part: qf+=forces*factor
    virtual void VariablesFbLoadForces(double factor = 1) override;

    /// Initialize the 'qb' part of the ChVariablesBody with the
    /// current value of body speeds. Note: since 'qb' is the unknown, this
    /// function seems unnecessary, unless used before VariablesFbIncrementMq()
    virtual void VariablesQbLoadSpeed() override;

    /// Adds M*q (masses multiplied current 'qb') to Fb, ex. if qb is initialized
    /// with v_old using VariablesQbLoadSpeed, this method can be used in
    /// timestepping schemes that do: M*v_new = M*v_old + forces*dt
    virtual void VariablesFbIncrementMq() override;

    /// Fetches the body speed (both linear and angular) from the
    /// 'qb' part of the ChVariablesBody (does not updates the full body&markers state)
    /// and sets it as the current body speed.
    /// If 'step' is not 0, also computes the approximate acceleration of
    /// the body using backward differences, that is  accel=(new_speed-old_speed)/step.
    /// Mostly used after the solver provided the solution in ChVariablesBody .
    virtual void VariablesQbSetSpeed(double step = 0) override;

    /// Increment body position by the 'qb' part of the ChVariablesBody,
    /// multiplied by a 'step' factor.
    ///     pos+=qb*step
    /// If qb is a speed, this behaves like a single step of 1-st order
    /// numerical integration (Euler integration).
    /// Does not automatically update markers & forces.
    virtual void VariablesQbIncrementPosition(double step) override;

    /// Register with the given system descriptor any ChVariable objects associated with this item.
    virtual void InjectVariables(ChSystemDescriptor& descriptor) override;

    // INTERFACE TO ChContactable

    virtual ChContactable::Type GetContactableType() const override { return ChContactable::Type::ONE_6; }

    virtual ChConstraintTuple* CreateConstraintTuple() override { return new ChConstraintTuple_1vars<6>(&variables); }

    /// Indicate whether or not the object must be considered in collision detection.
    virtual bool IsContactActive() override { return this->IsActive(); }

    /// Get the number of DOFs affected by this object (position part)
    virtual int GetContactableNumCoordsPosLevel() override { return 7; }

    /// Get the number of DOFs affected by this object (speed part)
    virtual int GetContactableNumCoordsVelLevel() override { return 6; }

    /// Get all the DOFs packed in a single vector (position part)
    virtual void ContactableGetStateBlockPosLevel(ChState& x) override;

    /// Get all the DOFs packed in a single vector (speed part)
    virtual void ContactableGetStateBlockVelLevel(ChStateDelta& w) override;

    /// Increment the provided state of this object by the given state-delta increment.
    /// Compute: x_new = x + dw.
    virtual void ContactableIncrementState(const ChState& x, const ChStateDelta& dw, ChState& x_new) override;

    /// Express the local point in absolute frame, for the given state position.
    virtual ChVector3d GetContactPoint(const ChVector3d& loc_point, const ChState& state_x) override;

    /// Get the absolute speed of a local point attached to the contactable.
    /// The given point is assumed to be expressed in the local frame of this object.
    /// This function must use the provided states.
    virtual ChVector3d GetContactPointSpeed(const ChVector3d& loc_point,
                                            const ChState& state_x,
                                            const ChStateDelta& state_w) override;

    /// Get the absolute speed of point abs_point if attached to the surface.
    virtual ChVector3d GetContactPointSpeed(const ChVector3d& abs_point) override;

    /// Return the frame of the associated collision model relative to the contactable object.
    /// ChCollisionModel might call this to get the position of the contact model (when rigid) and sync it.
    virtual ChFrame<> GetCollisionModelFrame() override;

    /// Apply the force & torque expressed in absolute reference, applied in pos, to the
    /// coordinates of the variables. Force for example could come from a penalty model.
    virtual void ContactForceLoadResidual_F(const ChVector3d& F,
                                            const ChVector3d& T,
                                            const ChVector3d& abs_point,
                                            ChVectorDynamic<>& R) override;

    /// Compute a contiguous vector of generalized forces Q from a given force & torque at the given point.
    /// Used for computing stiffness matrix (square force jacobian) by backward differentiation.
    /// The force and its application point are specified in the global frame.
    /// Each object must set the entries in Q corresponding to its variables, starting at the specified offset.
    /// If needed, the object states must be extracted from the provided state position.
    virtual void ContactComputeQ(const ChVector3d& F,
                                 const ChVector3d& T,
                                 const ChVector3d& point,
                                 const ChState& state_x,
                                 ChVectorDynamic<>& Q,
                                 int offset) override;

    /// Compute the jacobian(s) part(s) for this contactable item.
    /// For a ChBody, this updates the corresponding 1x6 jacobian.
    virtual void ComputeJacobianForContactPart(const ChVector3d& abs_point,
                                               ChMatrix33<>& contact_plane,
                                               ChConstraintTuple* jacobian_tuple_N,
                                               ChConstraintTuple* jacobian_tuple_U,
                                               ChConstraintTuple* jacobian_tuple_V,
                                               bool second) override;

    /// Compute the jacobian(s) part(s) for this contactable item, for rolling about N,u,v.
    /// Used only for rolling friction NSC contacts.
    virtual void ComputeJacobianForRollingContactPart(const ChVector3d& abs_point,
                                                      ChMatrix33<>& contact_plane,
                                                      ChConstraintTuple* jacobian_tuple_N,
                                                      ChConstraintTuple* jacobian_tuple_U,
                                                      ChConstraintTuple* jacobian_tuple_V,
                                                      bool second) override;

    /// Used by some SMC code
    virtual double GetContactableMass() override { return this->GetMass(); }

    // INTERFACE to ChLoadable

    /// Gets the number of DOFs affected by this element (position part)
    virtual unsigned int GetLoadableNumCoordsPosLevel() override { return 7; }

    /// Gets the number of DOFs affected by this element (speed part)
    virtual unsigned int GetLoadableNumCoordsVelLevel() override { return 6; }

    /// Number of coordinates in the interpolated field. Here, 6: = xyz displ + xyz rots.
    virtual unsigned int GetNumFieldCoords() override { return 6; }

    /// Tell the number of DOFs blocks.
    virtual unsigned int GetNumSubBlocks() override { return 1; }

    /// Get the offset of the specified sub-block of DOFs in global vector.
    virtual unsigned int GetSubBlockOffset(unsigned int nblock) override { return GetOffset_w(); }

    /// Get the size of the specified sub-block of DOFs in global vector.
    virtual unsigned int GetSubBlockSize(unsigned int nblock) override { return 6; }

    /// Check if the specified sub-block of DOFs is active.
    virtual bool IsSubBlockActive(unsigned int nblock) const override { return variables.IsActive(); }

    /// This is not needed because not used in quadrature.
    virtual double GetDensity() override { return 0; }

    bool fixed;    ///< flag indicating whether or not the body is fixed to global frame
    bool collide;  ///< flag indicating whether or not the body participates in collisions

    bool limit_speed;         ///< enable the clamping on body angular and linear speed
    bool disable_gyrotorque;  ///< disable the gyroscopic (quadratic) term, help the stability of the simulation but
                              ///< reduces the accuracy
    bool is_sleeping;         ///< flag indicating whether or not the body is currently in sleep mode
    bool allow_sleeping;      ///< flag indicating whether or not the body can go to sleep mode
    bool candidate_sleeping;  ///< flag indicating whether or not the body is a candidate for sleep mode in the current
                              ///< simulation

    // Friend classes with private access
    friend class ChSystem;
    friend class ChSystemMulticore;
    friend class ChSystemMulticoreNSC;
    friend class ChAssembly;
    friend class modal::ChModalAssembly;
    friend class ChConveyor;
};

CH_CLASS_VERSION(ChBody, 0)

const int BODY_DOF = 6;   ///< degrees of freedom of body in 3d space
const int BODY_QDOF = 7;  ///< degrees of freedom with quaternion rotation state
const int BODY_ROT = 3;   ///< rotational dof in Newton dynamics

}  // end namespace chrono

#endif
