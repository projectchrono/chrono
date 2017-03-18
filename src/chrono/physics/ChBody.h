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

#ifndef CHBODY_H
#define CHBODY_H

#include <cmath>

#include "chrono/physics/ChBodyFrame.h"
#include "chrono/physics/ChContactable.h"
#include "chrono/physics/ChForce.h"
#include "chrono/physics/ChLoadable.h"
#include "chrono/physics/ChMarker.h"
#include "chrono/physics/ChMaterialSurface.h"
#include "chrono/physics/ChMaterialSurfaceDEM.h"
#include "chrono/physics/ChPhysicsItem.h"
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
    
    // Tag needed for class factory in archive (de)serialization:
    CH_FACTORY_TAG(ChBody)

  protected:
    std::shared_ptr<collision::ChCollisionModel> collision_model;  ///< Pointer to the collision model

    unsigned int body_id;  ///< body specific identifier, used for indexing (internal use only)

    std::vector<std::shared_ptr<ChMarker> > marklist;  ///< list of child markers
    std::vector<std::shared_ptr<ChForce> > forcelist;  ///< list of child forces

    ChVector<> gyro;  ///< gyroscopic torque, i.e. Qm = Wvel x (XInertia*Wvel)

    ChVector<> Xforce;   ///< force  acting on body, applied to COG -in abs. coords-
    ChVector<> Xtorque;  ///< torque acting on body  -in body rel. coords-

    ChVector<> Force_acc;   ///< force accumulator; (in abs space, applied to COG)
    ChVector<> Torque_acc;  ///< torque accumulator;(in abs space)

    ChVector<> Scr_force;   ///< script force accumulator; (in abs space, applied to COG)
    ChVector<> Scr_torque;  ///< script torque accumulator;(in abs space)

    std::shared_ptr<ChMaterialSurfaceBase> matsurface;  ///< data for surface contact and impact

    // Auxiliary, stores position/rotation once a while when collision detection
    // routines require to know the last time that coll. detect. was satisfied
    ChCoordsys<> last_coll_pos;  ///< cached position at last collision

    float density;  ///< used when doing the 'recompute mass' operation.

    ChVariablesBodyOwnMass variables;  ///< interface to solver (store inertia and coordinates)

    float max_speed;  ///< limit on linear speed (useful for VR & videogames)
    float max_wvel;   ///< limit on angular vel. (useful for VR & videogames)

    float sleep_time;
    float sleep_minspeed;
    float sleep_minwvel;
    float sleep_starttime;

  public:
    /// Build a rigid body.
    ChBody(ChMaterialSurfaceBase::ContactMethod contact_method = ChMaterialSurfaceBase::DVI);

    /// Build a rigid body with a different collision model.
    ChBody(std::shared_ptr<collision::ChCollisionModel> new_collision_model,
           ChMaterialSurfaceBase::ContactMethod contact_method = ChMaterialSurfaceBase::DVI);

    ChBody(const ChBody& other);

    /// Destructor
    virtual ~ChBody();

    /// "Virtual" copy constructor (covariant return type).
    virtual ChBody* Clone() const override { return new ChBody(*this); }

    //
    // FLAGS
    //

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
    /// (After setting ON, you may need RecomputeCollisionModel()
    /// before anim starts, if you added an external object
    /// that implements onAddCollisionGeometries(), ex. in a plugin for a CAD)
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

    /// Return true if the body is active; i.e. it is neither fixed to ground
    /// nor is it in "sleep" mode. Return false otherwise.
    bool IsActive();

    /// Set body id for indexing (used only internally)
    void SetId(int id) { body_id = id; }

    /// Set body id for indexing (used only internally)
    unsigned int GetId() { return body_id; }

    //
    // FUNCTIONS
    //

    /// Number of coordinates of body: 7 because uses quaternions for rotation
    virtual int GetDOF() override { return 7; }
    /// Number of coordinates of body: 6 because derivatives use angular velocity
    virtual int GetDOF_w() override { return 6; }

    /// Returns reference to the encapsulated ChVariablesBody, representing
    /// body variables (pos, speed, or accel.) and forces.
    /// The ChVariablesBodyOwnMass is the interface to the system solver.
    ChVariablesBodyOwnMass& VariablesBody() override { return variables; }
    ChVariables& Variables() override { return variables; }

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
    virtual void IntStateIncrement(const unsigned int off_x,
                                   ChState& x_new,
                                   const ChState& x,
                                   const unsigned int off_v,
                                   const ChStateDelta& Dv) override;
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

    // Other functions

    /// Set no speed and no accelerations (but does not change the position)
    void SetNoSpeedNoAcceleration() override;

    /// Change the collision model.
    void SetCollisionModel(std::shared_ptr<collision::ChCollisionModel> new_collision_model);

    /// Acess the collision model for the collision engine.
    /// To get a non-null pointer, remember to SetCollide(true), before.
    std::shared_ptr<collision::ChCollisionModel> GetCollisionModel() { return collision_model; }

    /// Synchronize coll.model coordinate and bounding box to the position of the body.
    virtual void SyncCollisionModels() override;
    virtual void AddCollisionModelsToSystem() override;
    virtual void RemoveCollisionModelsFromSystem() override;

    /// Update the optimization structures (OOBB, ABB, etc.)
    /// of the collision model, from the associated geometry in some external object (es.CAD).
    bool RecomputeCollisionModel();

    /// Gets the last position when the collision detection was
    /// performed last time (i.e. last time SynchronizeLastCollPos() was used)
    const ChCoordsys<>& GetLastCollPos() const { return last_coll_pos; }
    /// Stores the current position in the last-collision-position buffer.
    void SynchronizeLastCollPos() { last_coll_pos = this->coord; }

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

    /// Get the master coordinate system for the assets (this will return the
    /// main coordinate system of the rigid body)
    virtual ChFrame<> GetAssetsFrame(unsigned int nclone = 0) override { return (GetFrame_REF_to_abs()); }

    /// Get the entire AABB axis-aligned bounding box of the object,
    /// as defined by the collision model (if any).
    virtual void GetTotalAABB(ChVector<>& bbmin, ChVector<>& bbmax) override;

    /// Method to deserialize only the state (position, speed)
    virtual void StreamINstate(ChStreamInBinary& mstream) override;
    /// Method to serialize only the state (position, speed)
    virtual void StreamOUTstate(ChStreamOutBinary& mstream) override;

    /// Infer the contact method from the underlying material properties object.
    ChMaterialSurfaceBase::ContactMethod GetContactMethod() { return matsurface->GetContactMethod(); }

    /// Access the DVI material surface properties associated with this body.
    /// This function performs a dynamic cast (and returns an empty pointer
    /// if matsurface is in fact of DEM type).  As such, it must return a copy
    /// of the shared pointer and is therefore NOT thread safe.
    std::shared_ptr<ChMaterialSurface> GetMaterialSurface() {
        return std::dynamic_pointer_cast<ChMaterialSurface>(matsurface);
    }

    /// Access the DEM material surface properties associated with this body.
    /// This function performs a dynamic cast (and returns an empty pointer
    /// if matsurface is in fact of DVI type).  As such, it must return a copy
    /// of the shared pointer and is therefore NOT thread safe.
    std::shared_ptr<ChMaterialSurfaceDEM> GetMaterialSurfaceDEM() {
        return std::dynamic_pointer_cast<ChMaterialSurfaceDEM>(matsurface);
    }

    /// Set the material surface properties by passing a ChMaterialSurface or
    /// ChMaterialSurfaceDEM object.
    void SetMaterialSurface(const std::shared_ptr<ChMaterialSurfaceBase>& mnewsurf) { matsurface = mnewsurf; }

    /// The density of the rigid body, as [mass]/[unit volume]. Used just if
    /// the inertia tensor and mass are automatically recomputed from the
    /// geometry (in case a CAD plugin for example provides the surfaces.)
    //float GetDensity() const { return density; } //  obsolete, use the base ChLoadable::GetDensity()
    void SetDensity(float mdensity) { density = mdensity; }

    //
    // DATABASE HANDLING.
    //
    // To attach/remove items (rigid bodies, links, etc.) you must use
    // shared pointer, so that you don't need to care about item deletion,
    // which will be automatic when needed.
    // Please don't add the same item multiple times; also, don't remove
    // items which haven't ever been added.
    // NOTE! After adding/removing items to the system, you should call Update() !

    /// Attach a marker to this body.
    void AddMarker(std::shared_ptr<ChMarker> amarker);
    /// Attach a force to this body.
    void AddForce(std::shared_ptr<ChForce> aforce);

    /// Remove a specific marker from this body. Warning: linear time search.
    void RemoveMarker(std::shared_ptr<ChMarker> amarker);
    /// Remove a specific force from this body. Warning: linear time search.
    void RemoveForce(std::shared_ptr<ChForce> aforce);

    /// Remove all markers at once. Faster than doing multiple RemoveForce()
    /// Don't care about deletion: it is automatic, only when needed.
    void RemoveAllForces();
    /// Remove all markers at once. Faster than doing multiple RemoveForce()
    /// Don't care about deletion: it is automatic, only when needed.
    void RemoveAllMarkers();

    /// Finds a marker from its ChObject name
    std::shared_ptr<ChMarker> SearchMarker(const char* m_name);
    /// Finds a force from its ChObject name
    std::shared_ptr<ChForce> SearchForce(const char* m_name);

    /// Gets the list of children markers.
    /// NOTE: to modify this list, use the appropriate Remove..
    /// and Add.. functions.
    const std::vector<std::shared_ptr<ChMarker> >& GetMarkerList() const { return marklist; }

    /// Gets the list of children forces.
    /// NOTE: to modify this list, use the appropriate Remove..
    /// and Add.. functions.
    const std::vector<std::shared_ptr<ChForce> >& GetForceList() const { return forcelist; }

    //
    // Point/vector transf.(NOTE! you may also use operators of ChMovingFrame)
    //

    ChVector<> Point_World2Body(const ChVector<>& mpoint);
    ChVector<> Point_Body2World(const ChVector<>& mpoint);
    ChVector<> Dir_World2Body(const ChVector<>& mpoint);
    ChVector<> Dir_Body2World(const ChVector<>& mpoint);
    ChVector<> RelPoint_AbsSpeed(const ChVector<>& mrelpoint);
    ChVector<> RelPoint_AbsAcc(const ChVector<>& mrelpoint);

    /// Mass of the rigid body. Must be positive.
    /// Try not to mix bodies with too high/too low values of mass, for numerical stability.
    void SetMass(double newmass) {
        if (newmass > 0.)
            variables.SetBodyMass(newmass);
    }
    double GetMass() { return variables.GetBodyMass(); }

    /// Set the inertia tensor of the body.
    /// The provided 3x3 matrix should be symmetric and contain the inertia
    /// tensor, epxressed in the local coordinate system:
    /// <pre>
    ///               [ int{x^2+z^2}dm    -int{xy}dm    -int{xz}dm    ]
    /// newXInertia = [                  int{x^2+z^2}   -int{yz}dm    ]
    ///               [                                int{x^2+y^2}dm ]
    /// </pre>
    void SetInertia(const ChMatrix33<>& newXInertia);
    /// Get a reference to the inertia tensor, expressed in local coordinate system.
    /// The return 3xe3 symmetric matrix contains the following values:
    /// <pre>
    ///  [ int{x^2+z^2}dm    -int{xy}dm    -int{xz}dm    ]
    ///  [                  int{x^2+z^2}   -int{yz}dm    ]
    ///  [                                int{x^2+y^2}dm ]
    /// </pre>
    const ChMatrix33<>& GetInertia() { return variables.GetBodyInertia(); }
    /// Set the diagonal part of the inertia tensor (Ixx, Iyy, Izz values).
    /// The provided 3x1 vector should contain the moments of inertia,
    /// expressed in the local coordinate frame:
    /// <pre>
    /// iner = [  int{x^2+z^2}dm   int{x^2+z^2}   int{x^2+y^2}dm ]
    /// </pre>
    void SetInertiaXX(const ChVector<>& iner);
    /// Get the diagonal part of the inertia tensor (Ixx, Iyy, Izz values).
    /// The return 3x1 vector contains the following values:
    /// <pre>
    /// [  int{x^2+z^2}dm   int{x^2+z^2}   int{x^2+y^2}dm ]
    /// </pre>   
    ChVector<> GetInertiaXX();
    /// Set the off-diagonal part of the inertia tensor (Ixy, Ixz, Iyz values).
    /// Warning about sign: in some books they write the inertia tensor as
    /// I=[Ixx, -Ixy, -Ixz; etc.] but here is I=[Ixx, Ixy, Ixz; ...].
    /// The provided 3x1 vector should contain the products of inertia,
    /// expressed in the local coordinate frame:
    /// <pre>
    /// iner = [ -int{xy}dm   -int{xz}dm   -int{yz}dm ]
    /// </pre>
    void SetInertiaXY(const ChVector<>& iner);
    /// Get the extradiagonal part of the inertia tensor (Ixy, Ixz, Iyz values)
    /// Warning about sign: in some books they write the inertia tensor as
    /// I=[Ixx, -Ixy, -Ixz; etc.] but here is I=[Ixx, Ixy, Ixz; ...].
    /// The return 3x1 vector contains the following values:
    /// <pre>
    /// [ -int{xy}dm   -int{xz}dm   -int{yz}dm ]
    /// </pre>  
    ChVector<> GetInertiaXY();

    /// Set the maximum linear speed (beyond this limit it will be clamped).
    /// This is useful in virtual reality and real-time simulations, because
    /// it reduces the risk of bad collision detection.
    /// This speed limit is active only if you set  SetLimitSpeed(true);
    void SetMaxSpeed(float m_max_speed) { max_speed = m_max_speed; }
    float GetMaxSpeed() const { return max_speed; }

    /// Set the maximum angualar speed (beyond this limit it will be clamped).
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
    void ComputeQInertia(ChMatrixNM<double, 4, 4>* mQInertia);

    /// Computes the gyroscopic torque. In fact, in sake of highest
    /// speed, the gyroscopic torque isn't automatically updated each time a
    /// SetCoord() or SetCoord_dt() etc. is called, but only if necessary,
    /// for each UpdateState().
    void ComputeGyro();

    /// Transform and adds a cartesian force to a generic 7x1 vector of body lagrangian forces mQf .
    /// The carthesian force must be passed as vector and application point, and vcan be either in local
    /// (local = true) or absolute reference (local = false)
    void Add_as_lagrangian_force(const ChVector<>& force,
                                 const ChVector<>& appl_point,
                                 bool local,
                                 ChMatrixNM<double, 7, 1>* mQf);
    void Add_as_lagrangian_torque(const ChVector<>& torque, bool local, ChMatrixNM<double, 7, 1>* mQf);

    //
    // UTILITIES FOR FORCES/TORQUES:
    //

    /// Add forces and torques into the "accumulators", as increment.
    /// Forces and torques currently in accumulators will affect the body.
    /// It's up to the user to remember to empty them and/or set again at each
    /// integration step. Useful to apply forces to bodies without needing to
    /// add ChForce() objects. If local=true, force,appl.point or torque are considered
    /// expressed in body coordinates, otherwise are considered in absolute coordinates.
    void Accumulate_force(const ChVector<>& force, const ChVector<>& appl_point, bool local);
    void Accumulate_torque(const ChVector<>& torque, bool local);
    void Empty_forces_accumulators() {
        Force_acc = VNULL;
        Torque_acc = VNULL;
    }
    const ChVector<>& Get_accumulated_force() const { return Force_acc; }
    const ChVector<>& Get_accumulated_torque() const { return Torque_acc; }

    /// To get & set the 'script' force buffers(only accessed by external scripts, so
    /// It's up to the script to remember to set& reset them -link class just add them to
    /// all other forces. Script forces&torques are considered applied to COG, in abs csys.
    const ChVector<>& Get_Scr_force() const { return Scr_force; }
    const ChVector<>& Get_Scr_torque() const { return Scr_torque; }
    void Set_Scr_force(const ChVector<>& mf) { Scr_force = mf; }
    void Set_Scr_torque(const ChVector<>& mf) { Scr_torque = mf; }
    void Accumulate_script_force(const ChVector<>& force, const ChVector<>& appl_point, bool local);
    void Accumulate_script_torque(const ChVector<>& torque, bool local);

    /// Return the gyroscopic torque.
    const ChVector<>& Get_gyro() const { return gyro; }

    /// Get the total force applied to the rigid body (applied at center of mass.
    /// expressed in absolute coordinates).
    const ChVector<>& Get_Xforce() const { return Xforce; }
    /// Get the total torque applied to the rigid body (expressed in body coordinates).
    /// This does not include the gyroscopic torque.
    const ChVector<>& Get_Xtorque() const { return Xtorque; }


    //
    // UPDATE FUNCTIONS
    //

    /// Update all children markers of the rigid body, at current body state
    void UpdateMarkers(double mytime);
    /// Update all children forces of the rigid body, at current body state.
    void UpdateForces(double mytime);
    /// Update local time of rigid body, and time-dependant data
    void UpdateTime(double mytime);

    /// Update all auxiliary data of the rigid body and of
    /// its children (markers, forces..), at given time
    virtual void Update(double mytime, bool update_assets = true) override;
    /// Update all auxiliary data of the rigid body and of
    /// its children (markers, forces..)
    virtual void Update(bool update_assets = true) override;

    //
    // INTERFACE TO ChContactable
    //

    virtual ChVariables* GetVariables1() override { return &this->variables; }

    /// Tell if the object must be considered in collision detection
    virtual bool IsContactActive() override { return this->IsActive(); }

    /// Get the number of DOFs affected by this object (position part)
    virtual int ContactableGet_ndof_x() override { return 7; }

    /// Get the number of DOFs affected by this object (speed part)
    virtual int ContactableGet_ndof_w() override { return 6; }

    /// Get all the DOFs packed in a single vector (position part)
    virtual void ContactableGetStateBlock_x(ChState& x) override { x.PasteCoordsys(this->GetCoord(), 0, 0); }

    /// Get all the DOFs packed in a single vector (speed part)
    virtual void ContactableGetStateBlock_w(ChStateDelta& w) override {
        w.PasteVector(this->GetPos_dt(), 0, 0);
        w.PasteVector(this->GetWvel_loc(), 3, 0);
    }

    /// Increment the provided state of this object by the given state-delta increment.
    /// Compute: x_new = x + dw.
    virtual void ContactableIncrementState(const ChState& x, const ChStateDelta& dw, ChState& x_new) override {
        IntStateIncrement(0, x_new, x, 0, dw);
    }

    /// Return the pointer to the surface material.
    /// Use dynamic cast to understand if this is a ChMaterialSurfaceDEM, ChMaterialSurfaceDVI or others.
    /// This function returns a reference to the shared pointer member variable and is therefore THREAD SAFE.
    virtual std::shared_ptr<ChMaterialSurfaceBase>& GetMaterialSurfaceBase() override { return matsurface; }

    /// Get the resultant contact force acting on this body.
    ChVector<> GetContactForce();

    /// Get the resultant contact torque acting on this body.
    ChVector<> GetContactTorque();

    /// Express the local point in absolute frame, for the given state position.
    virtual ChVector<> GetContactPoint(const ChVector<>& loc_point, const ChState& state_x) override {
        ChCoordsys<> csys = state_x.ClipCoordsys(0, 0);
        return csys.TransformPointLocalToParent(loc_point);
    }

    /// Get the absolute speed of a local point attached to the contactable.
    /// The given point is assumed to be expressed in the local frame of this object.
    /// This function must use the provided states.
    virtual ChVector<> GetContactPointSpeed(const ChVector<>& loc_point,
                                            const ChState& state_x,
                                            const ChStateDelta& state_w) override {
        ChCoordsys<> csys = state_x.ClipCoordsys(0, 0);
        ChVector<> abs_vel = state_w.ClipVector(0, 0);
        ChVector<> loc_omg = state_w.ClipVector(3, 0);
        ChVector<> abs_omg = csys.TransformDirectionLocalToParent(loc_omg);

        return abs_vel + Vcross(abs_omg, loc_point);
    }

    /// Get the absolute speed of point abs_point if attached to the surface.
    virtual ChVector<> GetContactPointSpeed(const ChVector<>& abs_point) override;

    /// Return the coordinate system for the associated collision model.
    /// ChCollisionModel might call this to get the position of the
    /// contact model (when rigid) and sync it.
    virtual ChCoordsys<> GetCsysForCollisionModel() override { return ChCoordsys<>(this->GetFrame_REF_to_abs().coord); }

    /// Apply the force, expressed in absolute reference, applied in pos, to the
    /// coordinates of the variables. Force for example could come from a penalty model.
    virtual void ContactForceLoadResidual_F(const ChVector<>& F,
                                            const ChVector<>& abs_point,
                                            ChVectorDynamic<>& R) override;

    /// Apply the given force at the given point and load the generalized force array.
    /// The force and its application point are specified in the gloabl frame.
    /// Each object must set the entries in Q corresponding to its variables, starting at the specified offset.
    /// If needed, the object states must be extracted from the provided state position.
    virtual void ContactForceLoadQ(const ChVector<>& F,
                                   const ChVector<>& point,
                                   const ChState& state_x,
                                   ChVectorDynamic<>& Q,
                                   int offset) override {
        ChCoordsys<> csys = state_x.ClipCoordsys(0, 0);
        ChVector<> point_loc = csys.TransformPointParentToLocal(point);
        ChVector<> force_loc = csys.TransformDirectionParentToLocal(F);
        ChVector<> torque_loc = Vcross(point_loc, force_loc);
        Q.PasteVector(F, offset + 0, 0);
        Q.PasteVector(torque_loc, offset + 3, 0);
    }

    /// Compute the jacobian(s) part(s) for this contactable item.
    /// For a ChBody, this updates the corresponding 1x6 jacobian.
    virtual void ComputeJacobianForContactPart(const ChVector<>& abs_point,
                                               ChMatrix33<>& contact_plane,
                                               ChVariableTupleCarrier_1vars<6>::type_constraint_tuple& jacobian_tuple_N,
                                               ChVariableTupleCarrier_1vars<6>::type_constraint_tuple& jacobian_tuple_U,
                                               ChVariableTupleCarrier_1vars<6>::type_constraint_tuple& jacobian_tuple_V,
                                               bool second) override;

    /// Compute the jacobian(s) part(s) for this contactable item, for rolling about N,u,v.
    /// Used only for rolling friction DVI contacts.
    virtual void ComputeJacobianForRollingContactPart(
        const ChVector<>& abs_point,
        ChMatrix33<>& contact_plane,
        ChVariableTupleCarrier_1vars<6>::type_constraint_tuple& jacobian_tuple_N,
        ChVariableTupleCarrier_1vars<6>::type_constraint_tuple& jacobian_tuple_U,
        ChVariableTupleCarrier_1vars<6>::type_constraint_tuple& jacobian_tuple_V,
        bool second) override;

    /// Used by some DEM code
    virtual double GetContactableMass() override { return this->GetMass(); }

    /// This is only for backward compatibility
    virtual ChPhysicsItem* GetPhysicsItem() override { return this; }

    //
    // INTERFACE to ChLoadable
    //

    /// Gets the number of DOFs affected by this element (position part)
    virtual int LoadableGet_ndof_x() override { return 7; }

    /// Gets the number of DOFs affected by this element (speed part)
    virtual int LoadableGet_ndof_w() override { return 6; }

    /// Gets all the DOFs packed in a single vector (position part)
    virtual void LoadableGetStateBlock_x(int block_offset, ChState& mD) override {
        mD.PasteCoordsys(this->GetCoord(), block_offset, 0);
    }

    /// Gets all the DOFs packed in a single vector (speed part)
    virtual void LoadableGetStateBlock_w(int block_offset, ChStateDelta& mD) override {
        mD.PasteVector(this->GetPos_dt(), block_offset, 0);
        mD.PasteVector(this->GetWvel_loc(), block_offset + 3, 0);
    }

    /// Increment all DOFs using a delta.
    virtual void LoadableStateIncrement(const unsigned int off_x, ChState& x_new, const ChState& x, const unsigned int off_v, const ChStateDelta& Dv) override {
        IntStateIncrement(off_x, x_new, x, off_v, Dv);
    }


    /// Number of coordinates in the interpolated field, ex=3 for a
    /// tetrahedron finite element or a cable, etc. Here is 6: xyz displ + xyz rots
    virtual int Get_field_ncoords() override { return 6; }

    /// Tell the number of DOFs blocks (ex. =1 for a body, =4 for a tetrahedron, etc.)
    virtual int GetSubBlocks() override { return 1; }

    /// Get the offset of the i-th sub-block of DOFs in global vector
    virtual unsigned int GetSubBlockOffset(int nblock) override { return this->GetOffset_w(); }

    /// Get the size of the i-th sub-block of DOFs in global vector
    virtual unsigned int GetSubBlockSize(int nblock) override { return 6; }

    /// Get the pointers to the contained ChVariables, appending to the mvars vector.
    virtual void LoadableGetVariables(std::vector<ChVariables*>& mvars) override {
        mvars.push_back(&this->Variables());
    }

    /// Evaluate Q=N'*F , for Q generalized lagrangian load, where N is some type of matrix
    /// evaluated at point P(U,V,W) assumed in absolute coordinates, and
    /// F is a load assumed in absolute coordinates.
    /// The det[J] is unused.
    virtual void ComputeNF(
        const double U,              ///< x coordinate of application point in absolute space
        const double V,              ///< y coordinate of application point in absolute space
        const double W,              ///< z coordinate of application point in absolute space
        ChVectorDynamic<>& Qi,       ///< Return result of N'*F  here, maybe with offset block_offset
        double& detJ,                ///< Return det[J] here
        const ChVectorDynamic<>& F,  ///< Input F vector, size is 6, it is {Force,Torque} in absolute coords.
        ChVectorDynamic<>* state_x,  ///< if != 0, update state (pos. part) to this, then evaluate Q
        ChVectorDynamic<>* state_w   ///< if != 0, update state (speed part) to this, then evaluate Q
        ) override {
        ChVector<> abs_pos(U, V, W);
        ChVector<> absF = F.ClipVector(0, 0);
        ChVector<> absT = F.ClipVector(3, 0);
        ChVector<> body_absF;
        ChVector<> body_locT;
        ChCoordsys<> bodycoord;
        if (state_x)
            bodycoord = state_x->ClipCoordsys(0, 0);  // the numerical jacobian algo might change state_x
        else
            bodycoord = this->coord;
        // compute Q components F,T, given current state of body 'bodycoord'. Note T in Q is in local csys, F is an abs
        // csys
        body_absF = absF;
        body_locT = bodycoord.rot.RotateBack(absT + ((abs_pos - bodycoord.pos) % absF));
        Qi.PasteVector(body_absF, 0, 0);
        Qi.PasteVector(body_locT, 3, 0);
        detJ = 1;  // not needed because not used in quadrature.
    }

    /// This is not needed because not used in quadrature.
    virtual double GetDensity() override { return density; }

    //
    // SERIALIZATION
    //

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOUT(ChArchiveOut& marchive) override;

    /// Method to allow deserialization of transient data from archives.
    virtual void ArchiveIN(ChArchiveIn& marchive) override;

  private:
    /// Instantiate the collision model
    virtual std::shared_ptr<collision::ChCollisionModel> InstanceCollisionModel();

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
        LIMITSPEED = (1L << 8),       // body angular and linar speed is limited (clamped)
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
};

CH_CLASS_VERSION(ChBody,0)

const int BODY_DOF = 6;   ///< degrees of freedom of body in 3d space
const int BODY_QDOF = 7;  ///< degrees of freedom with quaternion rotation state
const int BODY_ROT = 3;   ///< rotational dof in Newton dynamics

}  // end namespace chrono


#endif
