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

#ifndef CH_PARTICLE_CLOUD_H
#define CH_PARTICLE_CLOUD_H

#include <cmath>

#include "chrono/collision/ChCollisionModel.h"
#include "chrono/physics/ChContactable.h"
#include "chrono/physics/ChIndexedParticles.h"
#include "chrono/solver/ChVariablesBodySharedMass.h"

namespace chrono {

// Forward references (for parent hierarchy pointer)
class ChSystem;
class ChParticleCloud;

/// Class for a single particle clone in the ChParticleCloud cluster.
/// It does not define mass, inertia and shape because those are _shared_ among them.
class ChApi ChParticle : public ChParticleBase, public ChContactable_1vars<6> {
  public:
    ChParticle();
    ChParticle(const ChParticle& other);
    ~ChParticle();

    ChParticle& operator=(const ChParticle& other);

    // Access the variables of the node
    virtual ChVariables& Variables() override { return variables; }

    // Get the container
    ChParticleCloud* GetContainer() const { return container; }
    // Set the container
    void SetContainer(ChParticleCloud* mc) { container = mc; }

    // INTERFACE TO ChContactable

    virtual ChContactable::eChContactableType GetContactableType() const override { return CONTACTABLE_6; }

    /// Access variables.
    virtual ChVariables* GetVariables1() override { return &Variables(); }

    /// Tell if the object must be considered in collision detection.
    virtual bool IsContactActive() override { return true; }

    /// Get the number of DOFs affected by this object (position part).
    virtual int GetContactableNumCoordsPosLevel() override { return 7; }

    /// Get the number of DOFs affected by this object (speed part).
    virtual int GetContactableNumCoordsVelLevel() override { return 6; }

    /// Get all the DOFs packed in a single vector (position part)
    virtual void ContactableGetStateBlockPosLevel(ChState& x) override {
        x.segment(0, 3) = GetCoordsys().pos.eigen();
        x.segment(3, 4) = GetCoordsys().rot.eigen();
    }

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
    /// Easy in this case because there are no rotations..
    virtual ChVector3d GetContactPointSpeed(const ChVector3d& abs_point) override;

    /// Return the coordinate system for the associated collision model.
    /// ChCollisionModel might call this to get the position of the
    /// contact model (when rigid) and sync it.
    virtual ChFrame<> GetCollisionModelFrame() override { return ChFrame<>(m_csys); }

    /// Apply the force & torque, expressed in absolute reference, applied in pos, to the
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

    /// Compute the jacobian(s) part(s) for this contactable item. For example,
    /// if the contactable is a ChBody, this should update the corresponding 1x6 jacobian.
    virtual void ComputeJacobianForContactPart(const ChVector3d& abs_point,
                                               ChMatrix33<>& contact_plane,
                                               ChVariableTupleCarrier_1vars<6>::type_constraint_tuple& jacobian_tuple_N,
                                               ChVariableTupleCarrier_1vars<6>::type_constraint_tuple& jacobian_tuple_U,
                                               ChVariableTupleCarrier_1vars<6>::type_constraint_tuple& jacobian_tuple_V,
                                               bool second) override;

    /// Compute the jacobian(s) part(s) for this contactable item, for rolling about N,u,v
    /// (used only for rolling friction NSC contacts)
    virtual void ComputeJacobianForRollingContactPart(
        const ChVector3d& abs_point,
        ChMatrix33<>& contact_plane,
        ChVariableTupleCarrier_1vars<6>::type_constraint_tuple& jacobian_tuple_N,
        ChVariableTupleCarrier_1vars<6>::type_constraint_tuple& jacobian_tuple_U,
        ChVariableTupleCarrier_1vars<6>::type_constraint_tuple& jacobian_tuple_V,
        bool second) override;

    /// used by some SMC code
    virtual double GetContactableMass() override { return variables.GetBodyMass(); }

    /// This is only for backward compatibility
    virtual ChPhysicsItem* GetPhysicsItem() override;

    // SERIALIZATION
    virtual void ArchiveOut(ChArchiveOut& archive_out) override;
    virtual void ArchiveIn(ChArchiveIn& archive_in) override;

    // DATA
    ChParticleCloud* container;
    ChVariablesBodySharedMass variables;
    ChVector3d UserForce;
    ChVector3d UserTorque;
};

/// Class for clusters of 'clone' particles, that is many rigid objects with the same shape and mass.
/// This can be used to make granular flows, where you have thousands of objects with the same shape. In fact, a single
/// ChParticleCloud object can be more memory-efficient than many ChBody objects, because they share many features,
/// such as mass and collision shape. If you have N different families of shapes in your granular simulations (ex. 50%
/// of particles are large spheres, 25% are small spheres and 25% are polyhedrons) you can simply add three
/// ChParticleCloud objects to the ChSystem. This would be more efficient anyway than creating all shapes as ChBody.
class ChApi ChParticleCloud : public ChIndexedParticles {
  public:
    ChParticleCloud();
    ChParticleCloud(const ChParticleCloud& other);
    ~ChParticleCloud();

    /// "Virtual" copy constructor (covariant return type).
    virtual ChParticleCloud* Clone() const override { return new ChParticleCloud(*this); }

    /// Enable/disable the collision for this cluster of particles.
    void EnableCollision(bool state);
    virtual bool IsCollisionEnabled() const override { return collide; }

    /// Set the state of all particles in the cluster to 'fixed' (default: false).
    /// If true, the particles do not move.
    void SetFixed(bool state) { fixed = state; }

    /// Return true if the particle cluster is currently active and therefore included into the system solver.
    /// A cluster is inactive if it is fixed to ground.
    virtual bool IsActive() const override { return !fixed; }

    /// Enable limiting the linear speed (default: false).
    void SetLimitSpeed(bool state) { limit_speed = state; }

    /// Get the number of particles.
    size_t GetNumParticles() const override { return particles.size(); }

    /// Get all particles in the cluster.
    std::vector<ChParticle*> GetParticles() const { return particles; }

    /// Get particle position.
    const ChVector3d& GetParticlePos(unsigned int n) const { return particles[n]->GetPos(); }

    /// Get particle linear velocity.
    const ChVector3d& GetParticleVel(unsigned int n) const { return particles[n]->GetPosDt(); }

    /// Access the N-th particle.
    ChParticleBase& Particle(unsigned int n) override {
        assert(n < particles.size());
        return *particles[n];
    }

    /// Access the N-th particle.
    const ChParticleBase& Particle(unsigned int n) const override {
        assert(n < particles.size());
        return *particles[n];
    }

    /// Add a collision model for particles in this cloud.
    /// This is the "template" collision model that is used by all particles.
    void AddCollisionModel(std::shared_ptr<ChCollisionModel> model) { particle_collision_model = model; }

    /// Add a collision shape for particles in this cloud.
    /// If a collision model does not already exist, it is first created.
    /// The resulting model witll be the "template" collision model that is used by all particles.
    void AddCollisionShape(std::shared_ptr<ChCollisionShape> shape, const ChFrame<>& frame = ChFrame<>());

    /// Resize the particle cluster.
    /// This first deletes all existing particles, if any.
    void ResizeNparticles(int newsize) override;

    /// Add a new particle to the particle cluster, passing a coordinate system as initial state.
    void AddParticle(ChCoordsys<double> initial_state = CSYSNORM) override;

    /// Class to be used as a callback interface for dynamic coloring of particles in a cloud.
    class ChApi ColorCallback {
      public:
        virtual ~ColorCallback() {}

        /// Return the color for the given particle.
        virtual ChColor get(unsigned int n, const ChParticleCloud& cloud) const = 0;
    };

    /// Set callback to dynamically set visualization color (default: none).
    /// If enabled, a visualization system could use this for color-coding of the particles in a cloud.
    void RegisterColorCallback(std::shared_ptr<ColorCallback> callback) { m_color_fun = callback; }

    /// Return true if using dynamic coloring.
    /// This is the case if a ColorCallback was specified.
    bool UseDynamicColors() const;

    /// Get the visualization color for the specified particle.
    /// Return the color given by a ColorCallback, if one was provided. Otherwise return a default color.
    ChColor GetVisualColor(unsigned int n) const;

    /// Class to be used as a callback interface for dynamic visibility of particles in a cloud.
    class ChApi VisibilityCallback {
      public:
        virtual ~VisibilityCallback() {}

        /// Return a boolean indicating whether or not the given particle is visible or not.
        virtual bool get(unsigned int n, const ChParticleCloud& cloud) const = 0;
    };

    /// Set callback to dynamically set particle visibility (default: none).
    /// If enabled, a visualization system could use this for conditionally rendering particles in a cloud.
    void RegisterVisibilityCallback(std::shared_ptr<VisibilityCallback> callback) { m_vis_fun = callback; }

    /// Return a boolean indicating whether or not the specified particle is visible.
    /// Return the result given by a VisibilityCallback, if one was provided. Otherwise return true.
    bool IsVisible(unsigned int n) const;

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
    void ForceToRest() override;

    /// Add collision models (if any) for all particles to the provided collision system.
    virtual void AddCollisionModelsToSystem(ChCollisionSystem* coll_sys) const override;

    /// Remove the collision models (if any) for all particles from the provided collision system.
    virtual void RemoveCollisionModelsFromSystem(ChCollisionSystem* coll_sys) const override;

    /// Synchronize the position and bounding box of all particle collision models (if any).
    virtual void SyncCollisionModels() override;

    /// Mass of each particle. Must be positive.
    void SetMass(double newmass) {
        if (newmass > 0)
            particle_mass.SetBodyMass(newmass);
    }
    double GetMass() const { return particle_mass.GetBodyMass(); }

    /// Set the inertia tensor of each particle
    void SetInertia(const ChMatrix33<>& newXInertia);
    /// Set the diagonal part of the inertia tensor of each particle
    void SetInertiaXX(const ChVector3d& iner);
    /// Get the diagonal part of the inertia tensor of each particle
    ChVector3d GetInertiaXX() const;
    /// Set the extra-diagonal part of the inertia tensor of each particle
    /// (xy, yz, zx values, the rest is symmetric)
    void SetInertiaXY(const ChVector3d& iner);
    /// Get the extra-diagonal part of the inertia tensor of each particle
    /// (xy, yz, zx values, the rest is symmetric)
    ChVector3d GetInertiaXY() const;

    /// Set the maximum linear speed (beyond this limit it willbe clamped).
    /// This speed limit is active only if SetLimitSpeed(true) was called.
    void SetMaxLinVel(float m_max_speed) { max_speed = m_max_speed; }
    float GetMaxLinVel() const { return max_speed; }

    /// Set the maximum angular speed (beyond this limit it willbe clamped). 
    /// This speed limit is active only if  SetLimitSpeed(true) was called.
    void SetMaxAngVel(float m_max_wvel) { max_wvel = m_max_wvel; }
    float GetMaxAngVel() const { return max_wvel; }

    /// When this function is called, the speed of particles is clamped
    /// into limits posed by max_speed and max_wvel  - but remember to
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

    /// Update all auxiliary data of the particles
    virtual void Update(double time, bool update_assets) override;

    virtual void ArchiveOut(ChArchiveOut& archive_out) override;
    virtual void ArchiveIn(ChArchiveIn& archive_in) override;

  private:
    std::vector<ChParticle*> particles;  ///< the particles
    ChSharedMassBody particle_mass;      ///< shared mass of particles

    std::shared_ptr<ColorCallback> m_color_fun;     ///< callback for dynamic coloring
    std::shared_ptr<VisibilityCallback> m_vis_fun;  ///< callback for particle visibility

    std::shared_ptr<ChCollisionModel> particle_collision_model;  ///< sample collision model

    bool fixed;
    bool collide;
    bool limit_speed;

    float max_speed;  ///< limit on linear speed (useful for increased simulation speed)
    float max_wvel;   ///< limit on angular vel. (useful for increased simulation speed)

    float sleep_time;
    float sleep_minspeed;
    float sleep_minwvel;
    float sleep_starttime;
};

/// Predefined particle cloud dynamic coloring based on particle height.
class ChApi HeightColorCallback : public ChParticleCloud::ColorCallback {
  public:
    HeightColorCallback(double hmin, double hmax, const ChVector3d& up = ChVector3d(0, 0, 1))
        : m_monochrome(false), m_hmin(hmin), m_hmax(hmax), m_up(up) {}
    HeightColorCallback(const ChColor& base_color, double hmin, double hmax, const ChVector3d& up = ChVector3d(0, 0, 1))
        : m_monochrome(true), m_base_color(base_color), m_hmin(hmin), m_hmax(hmax), m_up(up) {}

    virtual ChColor get(unsigned int n, const ChParticleCloud& cloud) const override {
        double height = Vdot(cloud.GetParticlePos(n), m_up);  // particle height
        if (m_monochrome) {
            float factor = (float)((height - m_hmin) / (m_hmax - m_hmin));  // color scaling factor (0,1)
            return ChColor(factor * m_base_color.R, factor * m_base_color.G, factor * m_base_color.B);
        } else
            return ChColor::ComputeFalseColor(height, m_hmin, m_hmax);
    }

  private:
    bool m_monochrome;
    ChColor m_base_color;
    double m_hmin;
    double m_hmax;
    ChVector3d m_up;
};

class ChApi VelocityColorCallback : public ChParticleCloud::ColorCallback {
  public:
    enum class Component { X, Y, Z, NORM };

    VelocityColorCallback(double vmin, double vmax, Component component = Component::NORM)
        : m_monochrome(false), m_vmin(vmin), m_vmax(vmax), m_component(component) {}
    VelocityColorCallback(const ChColor& base_color, double vmin, double vmax, Component component = Component::NORM)
        : m_monochrome(true), m_base_color(base_color), m_vmin(vmin), m_vmax(vmax), m_component(component) {}

    virtual ChColor get(unsigned int n, const ChParticleCloud& cloud) const override {
        double vel = 0;
        switch (m_component) {
            case Component::NORM:
                vel = cloud.GetParticleVel(n).Length();
                break;
            case Component::X:
                vel = std::abs(cloud.GetParticleVel(n).x());
                break;
            case Component::Y:
                vel = std::abs(cloud.GetParticleVel(n).y());
                break;
            case Component::Z:
                vel = std::abs(cloud.GetParticleVel(n).z());
                break;
        }

        if (m_monochrome) {
            float factor = (float)((vel - m_vmin) / (m_vmax - m_vmin));  // color scaling factor (0,1)
            return ChColor(factor * m_base_color.R, factor * m_base_color.G, factor * m_base_color.B);
        } else
            return ChColor::ComputeFalseColor(vel, m_vmin, m_vmax);
    }

  private:
    Component m_component;
    bool m_monochrome;
    ChColor m_base_color;
    double m_vmin;
    double m_vmax;
};

CH_CLASS_VERSION(ChParticleCloud, 0)

}  // end namespace chrono

#endif
