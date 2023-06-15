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

/// Class for rigid bodies. A rigid body is an entity which
/// can move in 3D space, and can be constrained to other rigid
/// bodies using ChLink objects. Rigid bodies can contain auxiliary
/// references (the ChMarker objects) and forces (the ChForce objects).
/// These objects have mass and inertia properties. A shape can also
/// be associated to the body, for collision detection.
///
/// Further info at the @ref rigid_bodies  manual page.

class ChApi ChBody : public ChPhysicsItem, public ChBodyFrame, public ChContactable_1vars<6>, public ChLoadableUVW {
  public:
    /// Build a rigid body.
    ChBody(collision::ChCollisionSystemType collision_type = collision::ChCollisionSystemType::BULLET);

    /// Build a rigid body with a different collision model.
    ChBody(std::shared_ptr<collision::ChCollisionModel> new_collision_model);

    ChBody(const ChBody& other);

    /// Destructor
    virtual ~ChBody();

    /// "Virtual" copy constructor (covariant return type).
    virtual ChBody* Clone() const override { return new ChBody(*this); }

    // FLAGS

    /// Sets the 'fixed' state of the body. If true, it does not move
    /// respect to the absolute world, despite constraints, forces, etc.
    void SetBodyFixed(bool state);
    /// Return true if this body is fixed to ground.
    bool GetBodyFixed() const;

    /// If true, the normal restitution coefficient is evaluated from painted material channel.
    void SetEvalContactCn(bool state);
    bool GetEvalContactCn() const;

    /// If true, the tangential restitution coefficient is evaluated from painted material channel.
    void SetEvalContactCt(bool state);
    bool GetEvalContactCt() const;

    /// If true, the kinetic friction coefficient is evaluated from painted material channel.
    void SetEvalContactKf(bool state);
    bool GetEvalContactKf() const;

    /// If true, the static friction coefficient is evaluated
    /// from painted material channel.
    void SetEvalContactSf(bool state);
    bool GetEvalContactSf() const;

    /// Enable/disable the collision for this rigid body.
    /// before anim starts, if you added an external object
    /// that implements onAddCollisionGeometries(), ex. in a plug-in for a CAD)
    void SetCollide(bool state);

    /// Return true if collision is enabled for this body.
    virtual bool GetCollide() const override;

    /// Show collision mesh in 3D views.
    void SetShowCollisionMesh(bool state);

    /// Return true if collision mesh is shown in 3D views.
    bool GetShowCollisionMesh() const;

    /// Enable the maximum linear speed limit (beyond this limit it will be clamped).
    /// This is useful in virtual reality and real-time simulations, because
    /// it reduces the risk of bad collision detection.
    /// The realism is limited, but the simulation is more stable.
    void SetLimitSpeed(bool state);

    /// Return true if maximum linear speed is limited.
    bool GetLimitSpeed() const;

    /// Deactivate the gyroscopic torque (quadratic term).
    /// This is useful in virtual reality and real-time
    /// simulations, where objects that spin too fast with non-uniform inertia
    /// tensors (ex thin cylinders) might cause the integration to diverge quickly.
    /// The realism is limited, but the simulation is more stable.
    void SetNoGyroTorque(bool state);

    /// Return true if gyroscopic torque is deactivated.
    bool GetNoGyroTorque() const;

    /// Enable/disable option for setting bodies to "sleep".
    /// If use sleeping = true, bodies which stay in same place
    /// for long enough time will be deactivated, for optimization.
    /// The realism is limited, but the simulation is faster.
    void SetUseSleeping(bool state);

    /// Return true if 'sleep' mode is activated.
    bool GetUseSleeping() const;

    /// Force the body in sleeping mode or not (usually this state change is not
    /// handled by users, anyway, because it is mostly automatic).
    void SetSleeping(bool state);

    /// Return true if this body is currently in 'sleep' mode.
    bool GetSleeping() const;

    /// Test if a body could go in sleeping state if requirements are satisfied.
    /// Return true if state could be changed from no sleep to sleep.
    bool TrySleeping();

    /// Return true if the body is currently active and thereofre included into the system solver.
    /// A body is inactive if it is fixed to ground or in sleep mode.
    virtual bool IsActive() const override;

    /// Set body id for indexing (internal use only)
    void SetId(int id) { body_id = id; }

    /// Set body id for indexing (internal use only)
    unsigned int GetId() { return body_id; }

    /// Set global body index (internal use only)
    void SetGid(unsigned int id) { body_gid = id; }

    /// Get the global body index (internal use only)
    unsigned int GetGid() const { return body_gid; }

    // FUNCTIONS

    /// Number of coordinates of body: 7 because uses quaternions for rotation.
    virtual int GetDOF() override { return 7; }

    /// Number of coordinates of body: 6 because derivatives use angular velocity.
    virtual int GetDOF_w() override { return 6; }

    /// Return a reference to the encapsulated ChVariablesBody, representing states (pos, speed, or accel.) and forces.
    /// The ChVariablesBodyOwnMass is the interface to the system solver.
    virtual ChVariables& Variables() override { return variables; }

    /// Set no speed and no accelerations (but does not change the position)
    void SetNoSpeedNoAcceleration() override;

    /// Change the collision model.
    void SetCollisionModel(std::shared_ptr<collision::ChCollisionModel> new_collision_model);

    /// Access the collision model for the collision engine.
    /// To get a non-null pointer, remember to SetCollide(true), before.
    std::shared_ptr<collision::ChCollisionModel> GetCollisionModel() { return collision_model; }

    /// Synchronize coll.model coordinate and bounding box to the position of the body.
    virtual void SyncCollisionModels() override;
    virtual void AddCollisionModelsToSystem() override;
    virtual void RemoveCollisionModelsFromSystem() override;

    /// Get the rigid body coordinate system that represents
    /// the GOG (Center of Gravity). The mass and inertia tensor
    /// are defined respect to this coordinate system, that is also
    /// assumed the default main coordinates of the body.
    /// By default, doing mybody.GetPos() etc. is like mybody.GetFrame_COG_abs().GetPos() etc.
    virtual const ChFrameMoving<>& GetFrame_COG_to_abs() const { return *this; }

    /// Get the rigid body coordinate system that is used for
    /// defining the collision shapes and the ChMarker objects.
    /// For the base ChBody, this is always the same reference of the COG.
    virtual const ChFrameMoving<>& GetFrame_REF_to_abs() const { return *this; }

    /// Get the reference frame (expressed in and relative to the absolute frame) of the visual model.
    /// For a ChBody, this is the main coordinate system of the rigid body.
    virtual ChFrame<> GetVisualModelFrame(unsigned int nclone = 0) override { return (GetFrame_REF_to_abs()); }

    /// Get the entire AABB axis-aligned bounding box of the object,
    /// as defined by the collision model (if any).
    virtual void GetTotalAABB(ChVector<>& bbmin, ChVector<>& bbmax) override;

    /// Method to deserialize only the state (position, speed)
    virtual void StreamInstate(ChStreamInBinary& mstream) override;

    /// Method to serialize only the state (position, speed)
    virtual void StreamOutstate(ChStreamOutBinary& mstream) override;

    /// The density of the rigid body, as [mass]/[unit volume]. Used just if
    /// the inertia tensor and mass are automatically recomputed from the
    /// geometry (in case a CAD plugin for example provides the surfaces.)
    // float GetDensity() const { return density; } //  obsolete, use the base ChLoadable::GetDensity()
    void SetDensity(float mdensity) { density = mdensity; }

    // DATABASE HANDLING.
    //
    // Do not add the same item multiple times
    // Do not remove items that have not been added.

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
    const std::vector<std::shared_ptr<ChMarker>>& GetMarkerList() const { return marklist; }

    /// Gets the list of children forces.
    /// NOTE: to modify this list, use the appropriate Remove..
    /// and Add.. functions.
    const std::vector<std::shared_ptr<ChForce>>& GetForceList() const { return forcelist; }

    // Point/vector transf.(NOTE! you may also use operators of ChMovingFrame)

    ChVector<> Point_World2Body(const ChVector<>& mpoint);
    ChVector<> Point_Body2World(const ChVector<>& mpoint);
    ChVector<> Dir_World2Body(const ChVector<>& dir);
    ChVector<> Dir_Body2World(const ChVector<>& dir);
    ChVector<> RelPoint_AbsSpeed(const ChVector<>& mrelpoint);
    ChVector<> RelPoint_AbsAcc(const ChVector<>& mrelpoint);

    /// Set the body mass.
    /// Try not to mix bodies with too high/too low values of mass, for numerical stability.
    void SetMass(double newmass) {
         variables.SetBodyMass(newmass);
    }

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
    void SetInertiaXX(const ChVector<>& iner);

    /// Get the diagonal part of the inertia tensor (Ixx, Iyy, Izz values).
    /// The return 3x1 vector contains the following values:
    /// <pre>
    /// [  int{y^2+z^2}dm   int{x^2+z^2}   int{x^2+y^2}dm ]
    /// </pre>
    ChVector<> GetInertiaXX() const;

    /// Set the off-diagonal part of the inertia tensor (Ixy, Ixz, Iyz values).
    /// The provided 3x1 vector should contain the products of inertia,
    /// expressed in the local coordinate frame:
    /// <pre>
    /// iner = [ -int{xy}dm   -int{xz}dm   -int{yz}dm ]
    /// </pre>
    void SetInertiaXY(const ChVector<>& iner);

    /// Get the extra-diagonal part of the inertia tensor (Ixy, Ixz, Iyz values).
    /// The return 3x1 vector contains the following values:
    /// <pre>
    /// [ -int{xy}dm   -int{xz}dm   -int{yz}dm ]
    /// </pre>
    ChVector<> GetInertiaXY() const;

    /// Set the maximum linear speed (beyond this limit it will be clamped).
    /// This is useful in virtual reality and real-time simulations, because
    /// it reduces the risk of bad collision detection.
    /// This speed limit is active only if you set  SetLimitSpeed(true);
    void SetMaxSpeed(float m_max_speed) { max_speed = m_max_speed; }
    float GetMaxSpeed() const { return max_speed; }

    /// Set the maximum angular speed (beyond this limit it will be clamped).
    /// This is useful in virtual reality and real-time simulations, because
    /// it reduces the risk of bad collision detection.
    /// This speed limit is active only if you set  SetLimitSpeed(true);
    void SetMaxWvel(float m_max_wvel) { max_wvel = m_max_wvel; }
    float GetMaxWvel() const { return max_wvel; }

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
    void SetSleepMinSpeed(float m_t) { sleep_minspeed = m_t; }
    float GetSleepMinSpeed() const { return sleep_minspeed; }

    /// Set the max linear speed to be kept for 'sleep_time' before freezing.
    void SetSleepMinWvel(float m_t) { sleep_minwvel = m_t; }
    float GetSleepMinWvel() const { return sleep_minwvel; }

    /// Computes the 4x4 inertia tensor in quaternion space, if needed.
    void ComputeQInertia(ChMatrix44<>& mQInertia);

    /// Computes the gyroscopic torque. In fact, in sake of highest
    /// speed, the gyroscopic torque isn't automatically updated each time a
    /// SetCoord() or SetCoord_dt() etc. is called, but only if necessary,
    /// for each UpdateState().
    void ComputeGyro();

    // UTILITIES FOR FORCES/TORQUES:

    /// Add an applied force to the body's accumulator (as an increment).
    /// It is the caller's responsibility to clear the force and torque accumulators at each integration step.
    /// If local = true, the provided applied force is assumed to be expressed in body coordinates.
    /// If local = false, the provided applied force is assumed to be expressed in absolute coordinates.
    void Accumulate_force(const ChVector<>& force,       ///< applied force
                          const ChVector<>& appl_point,  ///< application point
                          bool local                     ///< force and point expressed in body local frame?
    );

    /// Add an applied torque to the body's accumulator (as an increment).
    /// It is the caller's responsibility to clear the force and torque accumulators at each integration step.
    /// If local = true, the provided applied torque is assumed to be expressed in body coordinates.
    /// If local = false, the provided applied torque is assumed to be expressed in absolute coordinates.
    void Accumulate_torque(const ChVector<>& torque,  ///< applied torque
                           bool local                 ///< torque expressed in body local frame?
    );

    /// Clear the force and torque accumulators.
    void Empty_forces_accumulators();

    /// Return the current value of the accumulator force.
    /// Note that this is a resultant force as applied to the COM and expressed in the absolute frame.
    const ChVector<>& Get_accumulated_force() const { return Force_acc; }

    /// Return the current value of the accumulator torque.
    /// Note that this is a resultant torque expressed in the body local frame.
    const ChVector<>& Get_accumulated_torque() const { return Torque_acc; }

    // UPDATE FUNCTIONS

    /// Update all children markers of the rigid body, at current body state
    void UpdateMarkers(double mytime);
    /// Update all children forces of the rigid body, at current body state.
    void UpdateForces(double mytime);
    /// Update local time of rigid body, and time-dependent data
    void UpdateTime(double mytime);

    /// Update all auxiliary data of the rigid body and of
    /// its children (markers, forces..), at given time
    virtual void Update(double mytime, bool update_assets = true) override;
    /// Update all auxiliary data of the rigid body and of
    /// its children (markers, forces..)
    virtual void Update(bool update_assets = true) override;

    /// Return the resultant applied force on the body.
    /// This resultant force includes all external applied loads acting on this body (from gravity, loads, springs,
    /// etc). However, this does *not* include any constraint forces. In particular, contact forces are not included if
    /// using the NSC formulation, but are included when using the SMC formulation.
    ChVector<> GetAppliedForce();

    /// Return the resultant applied torque on the body.
    /// This resultant torque includes all external applied loads acting on this body (from gravity, loads, springs,
    /// etc). However, this does *not* include any constraint forces. In particular, contact torques are not included if
    /// using the NSC formulation, but are included when using the SMC formulation.
    ChVector<> GetAppliedTorque();

    /// Get the resultant contact force acting on this body.
    ChVector<> GetContactForce();

    /// Get the resultant contact torque acting on this body.
    ChVector<> GetContactTorque();

    /// This is only for backward compatibility
    virtual ChPhysicsItem* GetPhysicsItem() override { return this; }

    // SERIALIZATION

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOut(ChArchiveOut& marchive) override;

    /// Method to allow deserialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& marchive) override;

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
    virtual void LoadableGetStateBlock_x(int block_offset, ChState& mD) override;

    /// Gets all the DOFs packed in a single vector (speed part)
    virtual void LoadableGetStateBlock_w(int block_offset, ChStateDelta& mD) override;

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
    std::shared_ptr<collision::ChCollisionModel> collision_model;  ///< pointer to the collision model

    unsigned int body_id;   ///< body-specific identifier, used for indexing (internal use only)
    unsigned int body_gid;  ///< body-specific identifier, used for global indexing (internal use only)

    std::vector<std::shared_ptr<ChMarker>> marklist;  ///< list of markers
    std::vector<std::shared_ptr<ChForce>> forcelist;  ///< list of forces

    ChVector<> gyro;  ///< gyroscopic torque, i.e. Qm = Wvel x (XInertia*Wvel)

    ChVector<> Xforce;   ///< force  acting on body, applied to COM (in absolute coords)
    ChVector<> Xtorque;  ///< torque acting on body  (in body local coords)

    ChVector<> Force_acc;   ///< force accumulator, applied to COM (in absolute coords)
    ChVector<> Torque_acc;  ///< torque accumulator (in body local coords)

    float density;  ///< used when doing the 'recompute mass' operation.

    ChVariablesBodyOwnMass variables;  ///< interface to solver (store inertia and coordinates)

    float max_speed;  ///< limit on linear speed
    float max_wvel;   ///< limit on angular velocity

    float sleep_time;
    float sleep_minspeed;
    float sleep_minwvel;
    float sleep_starttime;

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
    /// numerical integration (Eulero integration).
    /// Does not automatically update markers & forces.
    virtual void VariablesQbIncrementPosition(double step) override;

    /// Tell to a system descriptor that there are variables of type
    /// ChVariables in this object (for further passing it to a solver)
    virtual void InjectVariables(ChSystemDescriptor& mdescriptor) override;

    // INTERFACE TO ChContactable

    virtual ChContactable::eChContactableType GetContactableType() const override { return CONTACTABLE_6; }

    virtual ChVariables* GetVariables1() override { return &this->variables; }

    /// Indicate whether or not the object must be considered in collision detection.
    virtual bool IsContactActive() override { return this->IsActive(); }

    /// Get the number of DOFs affected by this object (position part)
    virtual int ContactableGet_ndof_x() override { return 7; }

    /// Get the number of DOFs affected by this object (speed part)
    virtual int ContactableGet_ndof_w() override { return 6; }

    /// Get all the DOFs packed in a single vector (position part)
    virtual void ContactableGetStateBlock_x(ChState& x) override;

    /// Get all the DOFs packed in a single vector (speed part)
    virtual void ContactableGetStateBlock_w(ChStateDelta& w) override;

    /// Increment the provided state of this object by the given state-delta increment.
    /// Compute: x_new = x + dw.
    virtual void ContactableIncrementState(const ChState& x, const ChStateDelta& dw, ChState& x_new) override;

    /// Express the local point in absolute frame, for the given state position.
    virtual ChVector<> GetContactPoint(const ChVector<>& loc_point, const ChState& state_x) override;

    /// Get the absolute speed of a local point attached to the contactable.
    /// The given point is assumed to be expressed in the local frame of this object.
    /// This function must use the provided states.
    virtual ChVector<> GetContactPointSpeed(const ChVector<>& loc_point,
                                            const ChState& state_x,
                                            const ChStateDelta& state_w) override;

    /// Get the absolute speed of point abs_point if attached to the surface.
    virtual ChVector<> GetContactPointSpeed(const ChVector<>& abs_point) override;

    /// Return the coordinate system for the associated collision model.
    /// ChCollisionModel might call this to get the position of the
    /// contact model (when rigid) and sync it.
    virtual ChCoordsys<> GetCsysForCollisionModel() override;

    /// Apply the force, expressed in absolute reference, applied in pos, to the
    /// coordinates of the variables. Force for example could come from a penalty model.
    virtual void ContactForceLoadResidual_F(const ChVector<>& F,
                                            const ChVector<>& abs_point,
                                            ChVectorDynamic<>& R) override;

    /// Apply the given force at the given point and load the generalized force array.
    /// The force and its application point are specified in the global frame.
    /// Each object must set the entries in Q corresponding to its variables, starting at the specified offset.
    /// If needed, the object states must be extracted from the provided state position.
    virtual void ContactForceLoadQ(const ChVector<>& F,
                                   const ChVector<>& point,
                                   const ChState& state_x,
                                   ChVectorDynamic<>& Q,
                                   int offset) override;

    /// Compute the jacobian(s) part(s) for this contactable item.
    /// For a ChBody, this updates the corresponding 1x6 jacobian.
    virtual void ComputeJacobianForContactPart(const ChVector<>& abs_point,
                                               ChMatrix33<>& contact_plane,
                                               ChVariableTupleCarrier_1vars<6>::type_constraint_tuple& jacobian_tuple_N,
                                               ChVariableTupleCarrier_1vars<6>::type_constraint_tuple& jacobian_tuple_U,
                                               ChVariableTupleCarrier_1vars<6>::type_constraint_tuple& jacobian_tuple_V,
                                               bool second) override;

    /// Compute the jacobian(s) part(s) for this contactable item, for rolling about N,u,v.
    /// Used only for rolling friction NSC contacts.
    virtual void ComputeJacobianForRollingContactPart(
        const ChVector<>& abs_point,
        ChMatrix33<>& contact_plane,
        ChVariableTupleCarrier_1vars<6>::type_constraint_tuple& jacobian_tuple_N,
        ChVariableTupleCarrier_1vars<6>::type_constraint_tuple& jacobian_tuple_U,
        ChVariableTupleCarrier_1vars<6>::type_constraint_tuple& jacobian_tuple_V,
        bool second) override;

    /// Used by some SMC code
    virtual double GetContactableMass() override { return this->GetMass(); }

    // INTERFACE to ChLoadable

    /// Gets the number of DOFs affected by this element (position part)
    virtual int LoadableGet_ndof_x() override { return 7; }

    /// Gets the number of DOFs affected by this element (speed part)
    virtual int LoadableGet_ndof_w() override { return 6; }

    /// Number of coordinates in the interpolated field. Here, 6: = xyz displ + xyz rots.
    virtual int Get_field_ncoords() override { return 6; }

    /// Tell the number of DOFs blocks.
    virtual int GetSubBlocks() override { return 1; }

    /// Get the offset of the specified sub-block of DOFs in global vector.
    virtual unsigned int GetSubBlockOffset(int nblock) override { return GetOffset_w(); }

    /// Get the size of the specified sub-block of DOFs in global vector.
    virtual unsigned int GetSubBlockSize(int nblock) override { return 6; }

    /// Check if the specified sub-block of DOFs is active.
    virtual bool IsSubBlockActive(int nblock) const override { return variables.IsActive(); }

    /// This is not needed because not used in quadrature.
    virtual double GetDensity() override { return density; }

    /// Bit flags
    enum BodyFlag {
        COLLIDE = (1L << 0),          // detects collisions
        CDINVISIBLE = (1L << 1),      // collision detection invisible
        EVAL_CONTACT_CN = (1L << 2),  // evaluate CONTACT_CN channel (normal restitution)
        EVAL_CONTACT_CT = (1L << 3),  // evaluate CONTACT_CT channel (tangential rest.)
        EVAL_CONTACT_KF = (1L << 4),  // evaluate CONTACT_KF channel (kinetic friction coeff)
        EVAL_CONTACT_SF = (1L << 5),  // evaluate CONTACT_SF channel (static friction coeff)
        SHOW_COLLMESH = (1L << 6),    // show collision mesh - obsolete
        FIXED = (1L << 7),            // body is fixed to ground
        LIMITSPEED = (1L << 8),       // body angular and linear speed is limited (clamped)
        SLEEPING = (1L << 9),         // body is sleeping [internal]
        USESLEEPING = (1L << 10),     // if body remains in same place for too long time, it will be frozen
        NOGYROTORQUE = (1L << 11),    // do not get the gyroscopic (quadratic) term, for low-fi but stable simulation
        COULDSLEEP = (1L << 12)       // if body remains in same place for too long time, it will be frozen
    };

    int bflags;  ///< encoding for all body flags

    /// Flags handling functions
    void BFlagsSetAllOFF();
    void BFlagsSetAllON();
    void BFlagSetON(BodyFlag mask);
    void BFlagSetOFF(BodyFlag mask);
    void BFlagSet(BodyFlag mask, bool state);
    bool BFlagGet(BodyFlag mask) const;

    // Give private access
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
