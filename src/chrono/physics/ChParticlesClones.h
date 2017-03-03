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

#ifndef CHPARTICLESCLONES_H
#define CHPARTICLESCLONES_H

#include <cmath>

#include "chrono/collision/ChCCollisionModel.h"
#include "chrono/physics/ChContactable.h"
#include "chrono/physics/ChIndexedParticles.h"
#include "chrono/physics/ChMaterialSurface.h"
#include "chrono/solver/ChVariablesBodySharedMass.h"

namespace chrono {

// Forward references (for parent hierarchy pointer)
class ChSystem;
class ChParticlesClones;

/// Class for a single particle clone in the ChParticlesClones cluster.
/// It does not define mass, inertia and shape because those are _shared_ among them.
class ChApi ChAparticle : public ChParticleBase, public ChContactable_1vars<6> {
  public:
    ChAparticle();
    ChAparticle(const ChAparticle& other);
    ~ChAparticle();

    ChAparticle& operator=(const ChAparticle& other);

    // Access the variables of the node
    virtual ChVariables& Variables() override { return variables; }

    // Get the container
    ChParticlesClones* GetContainer() const { return container; }
    // Set the container
    void SetContainer(ChParticlesClones* mc) { container = mc; }

    //
    // INTERFACE TO ChContactable
    //

    /// Access variables.
    virtual ChVariables* GetVariables1() override { return &Variables(); }

    /// Tell if the object must be considered in collision detection.
    virtual bool IsContactActive() override { return true; }

    /// Get the number of DOFs affected by this object (position part).
    virtual int ContactableGet_ndof_x() override { return 7; }

    /// Get the number of DOFs affected by this object (speed part).
    virtual int ContactableGet_ndof_w() override { return 6; }

    /// Get all the DOFs packed in a single vector (position part)
    virtual void ContactableGetStateBlock_x(ChState& x) override { x.PasteCoordsys(this->GetCoord(), 0, 0); }

    /// Get all the DOFs packed in a single vector (speed part)
    virtual void ContactableGetStateBlock_w(ChStateDelta& w) override;

    /// Increment the provided state of this object by the given state-delta increment.
    /// Compute: x_new = x + dw.
    virtual void ContactableIncrementState(const ChState& x, const ChStateDelta& dw, ChState& x_new) override;

    /// Return the pointer to the contact surface material.
    virtual std::shared_ptr<ChMaterialSurfaceBase>& GetMaterialSurfaceBase() override;

    /// Express the local point in absolute frame, for the given state position.
    virtual ChVector<> GetContactPoint(const ChVector<>& loc_point, const ChState& state_x) override;

    /// Get the absolute speed of a local point attached to the contactable.
    /// The given point is assumed to be expressed in the local frame of this object.
    /// This function must use the provided states.
    virtual ChVector<> GetContactPointSpeed(const ChVector<>& loc_point,
                                            const ChState& state_x,
                                            const ChStateDelta& state_w) override;

    /// Get the absolute speed of point abs_point if attached to the surface.
    /// Easy in this case because there are no roations..
    virtual ChVector<> GetContactPointSpeed(const ChVector<>& abs_point) override;

    /// Return the coordinate system for the associated collision model.
    /// ChCollisionModel might call this to get the position of the
    /// contact model (when rigid) and sync it.
    virtual ChCoordsys<> GetCsysForCollisionModel() override { return this->coord; }

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
                                   int offset) override;

    /// Compute the jacobian(s) part(s) for this contactable item. For example,
    /// if the contactable is a ChBody, this should update the corresponding 1x6 jacobian.
    virtual void ComputeJacobianForContactPart(const ChVector<>& abs_point,
                                               ChMatrix33<>& contact_plane,
                                               ChVariableTupleCarrier_1vars<6>::type_constraint_tuple& jacobian_tuple_N,
                                               ChVariableTupleCarrier_1vars<6>::type_constraint_tuple& jacobian_tuple_U,
                                               ChVariableTupleCarrier_1vars<6>::type_constraint_tuple& jacobian_tuple_V,
                                               bool second) override;

    /// Compute the jacobian(s) part(s) for this contactable item, for rolling about N,u,v
    /// (used only for rolling friction DVI contacts)
    virtual void ComputeJacobianForRollingContactPart(
        const ChVector<>& abs_point,
        ChMatrix33<>& contact_plane,
        ChVariableTupleCarrier_1vars<6>::type_constraint_tuple& jacobian_tuple_N,
        ChVariableTupleCarrier_1vars<6>::type_constraint_tuple& jacobian_tuple_U,
        ChVariableTupleCarrier_1vars<6>::type_constraint_tuple& jacobian_tuple_V,
        bool second) override;

    /// used by some DEM code
    virtual double GetContactableMass() override { return variables.GetBodyMass(); }

    /// This is only for backward compatibility
    virtual ChPhysicsItem* GetPhysicsItem() override;

    // SERIALIZATION

    virtual void ArchiveOUT(ChArchiveOut& marchive) override;
    virtual void ArchiveIN(ChArchiveIn& marchive) override;

    //
    // DATA
    //

    ChParticlesClones* container;
    ChVariablesBodySharedMass variables;
    collision::ChCollisionModel* collision_model;
    ChVector<> UserForce;
    ChVector<> UserTorque;
};

/// Class for clusters of 'clone' particles, that is many
/// rigid objects with the same shape and mass.
/// This can be used to make granular flows, where
/// you have thousands of objects with the same shape.
/// In fact, a single ChParticlesClones object can
/// be more memory-efficient than many ChBody objects,
/// because they share many features, such as mass and
/// collision shape.
/// If you have N different families of shapes in your
/// granular simulations (ex. 50% of particles are large
/// spheres, 25% are small spheres and 25% are polihedrons)
/// you can simply add three ChParticlesClones objects to the
/// ChSystem. This would be more efficient anyway than
/// creating all shapes as ChBody.

class ChApi ChParticlesClones : public ChIndexedParticles {

    // Tag needed for class factory in archive (de)serialization:
    CH_FACTORY_TAG(ChParticlesClones)

  private:
    std::vector<ChAparticle*> particles;  ///< the parricles

    ChSharedMassBody particle_mass;  ///< shared mass of particles

    collision::ChCollisionModel* particle_collision_model;  ///< sample collision model

    std::shared_ptr<ChMaterialSurfaceBase> matsurface;  ///< data for surface contact and impact

    bool do_collide;
    bool do_limit_speed;
    bool do_sleep;

    float max_speed;  ///< limit on linear speed (useful for increased simulation speed)
    float max_wvel;   ///< limit on angular vel. (useful for increased simulation speed)

    float sleep_time;
    float sleep_minspeed;
    float sleep_minwvel;
    float sleep_starttime;

  public:
    ChParticlesClones();
    ChParticlesClones(const ChParticlesClones& other);
    ~ChParticlesClones();

    /// "Virtual" copy constructor (covariant return type).
    virtual ChParticlesClones* Clone() const override { return new ChParticlesClones(*this); }

    /// Enable/disable the collision for this cluster of particles.
    /// After setting ON, remember RecomputeCollisionModel()
    /// before anim starts (it is not automatically
    /// recomputed here because of performance issues.)
    void SetCollide(bool mcoll);
    virtual bool GetCollide() const override { return do_collide; }

    /// Trick. Set the maximum linear speed (beyond this limit it will
    /// be clamped). This is useful in virtual reality and real-time
    /// simulations, because it reduces the risk of bad collision detection.
    /// The realism is limited, but the simulation is more stable.
    void SetLimitSpeed(bool mlimit) { do_limit_speed = mlimit; };
    bool GetLimitSpeed() const { return do_limit_speed; };

    /// Get the number of particles
    size_t GetNparticles() const override { return particles.size(); }

    /// Access the N-th particle
    ChParticleBase& GetParticle(unsigned int n) override {
        assert(n < particles.size());
        return *particles[n];
    }

    /// Resize the particle cluster. Also clear the state of
    /// previously created particles, if any.
    /// NOTE! Define the sample collision shape using GetCollisionModel()->...
    /// before adding particles!
    void ResizeNparticles(int newsize) override;

    /// Add a new particle to the particle cluster, passing a
    /// coordinate system as initial state.
    /// NOTE! Define the sample collision shape using GetCollisionModel()->...
    /// before adding particles!
    void AddParticle(ChCoordsys<double> initial_state = CSYSNORM) override;

    /// Set the material surface for contacts
    void SetMaterialSurface(const std::shared_ptr<ChMaterialSurfaceBase>& mnewsurf) { matsurface = mnewsurf; }

    /// Set the material surface for contacts
    std::shared_ptr<ChMaterialSurfaceBase>& GetMaterialSurfaceBase() { return matsurface; }

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

    // Override/implement system functions of ChPhysicsItem
    // (to assemble/manage data for system solver)

    virtual void VariablesFbReset() override;
    virtual void VariablesFbLoadForces(double factor = 1) override;
    virtual void VariablesQbLoadSpeed() override;
    virtual void VariablesFbIncrementMq() override;
    virtual void VariablesQbSetSpeed(double step = 0) override;
    virtual void VariablesQbIncrementPosition(double step) override;
    virtual void InjectVariables(ChSystemDescriptor& mdescriptor) override;

    // Other functions

    /// Set no speed and no accelerations (but does not change the position)
    void SetNoSpeedNoAcceleration() override;

    /// Acess the collision model for the collision engine: this is the 'sample'
    /// collision model that is used by all particles.
    /// To get a non-null pointer, remember to SetCollide(true), before.
    collision::ChCollisionModel* GetCollisionModel() { return particle_collision_model; }

    /// Synchronize coll.models coordinates and bounding boxes to the positions of the particles.
    virtual void SyncCollisionModels() override;
    virtual void AddCollisionModelsToSystem() override;
    virtual void RemoveCollisionModelsFromSystem() override;

    /// After you added collision shapes to the sample coll.model (the one
    /// that you access with GetCollisionModel() ) you need to call this
    /// function so that all collision models of particles will reference the sample coll.model.
    void UpdateParticleCollisionModels();

    /// Mass of each particle. Must be positive.
    void SetMass(double newmass) {
        if (newmass > 0)
            particle_mass.SetBodyMass(newmass);
    }
    double GetMass() const { return particle_mass.GetBodyMass(); }

    /// Set the inertia tensor of each particle
    void SetInertia(const ChMatrix33<>& newXInertia);
    /// Set the diagonal part of the inertia tensor of each particle
    void SetInertiaXX(const ChVector<>& iner);
    /// Get the diagonal part of the inertia tensor of each particle
    ChVector<> GetInertiaXX() const;
    /// Set the extradiagonal part of the inertia tensor of each particle
    /// (xy, yz, zx values, the rest is symmetric)
    void SetInertiaXY(const ChVector<>& iner);
    /// Get the extradiagonal part of the inertia tensor of each particle
    /// (xy, yz, zx values, the rest is symmetric)
    ChVector<> GetInertiaXY() const;

    /// Trick. Set the maximum linear speed (beyond this limit it will
    /// be clamped). This is useful in virtual reality and real-time
    /// simulations, because it reduces the risk of bad collision detection.
    /// This speed limit is active only if you set  SetLimitSpeed(true);
    void SetMaxSpeed(float m_max_speed) { max_speed = m_max_speed; }
    float GetMaxSpeed() const { return max_speed; }

    /// Trick. Set the maximum angular speed (beyond this limit it will
    /// be clamped). This is useful in virtual reality and real-time
    /// simulations, because it reduces the risk of bad collision detection.
    /// This speed limit is active only if you set  SetLimitSpeed(true);
    void SetMaxWvel(float m_max_wvel) { max_wvel = m_max_wvel; }
    float GetMaxWvel() const { return max_wvel; }

    /// When this function is called, the speed of particles is clamped
    /// into limits posed by max_speed and max_wvel  - but remember to
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

    //
    // UPDATE FUNCTIONS
    //

    /// Update all auxiliary data of the particles
    virtual void Update(double mytime, bool update_assets = true) override;
    /// Update all auxiliary data of the particles
    virtual void Update(bool update_assets = true) override;

    //
    // SERIALIZATION
    //

    virtual void ArchiveOUT(ChArchiveOut& marchive) override;
    virtual void ArchiveIN(ChArchiveIn& marchive) override;
};

CH_CLASS_VERSION(ChParticlesClones,0)

}  // end namespace chrono

#endif
