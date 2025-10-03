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

#ifndef CH_PHYSICSITEM_H
#define CH_PHYSICSITEM_H

#include "chrono/core/ChFrame.h"
#include "chrono/core/ChRotation.h"

#include "chrono/geometry/ChGeometry.h"
#include "chrono/physics/ChObject.h"
#include "chrono/solver/ChSystemDescriptor.h"
#include "chrono/timestepper/ChState.h"

namespace chrono {

// Forward references
class ChSystem;
class ChCollisionSystem;
namespace modal {
class ChModalAssembly;
}

/// Base class for physics items that are part of a simulation.
/// Such items (e.g., rigid bodies, joints, FEM meshes, etc.) can contain ChVariables or ChConstraints objects.
class ChApi ChPhysicsItem : public ChObj {
  public:
    ChPhysicsItem() : system(NULL), offset_x(0), offset_w(0), offset_L(0) {}
    ChPhysicsItem(const ChPhysicsItem& other);
    virtual ~ChPhysicsItem();

    /// "Virtual" copy constructor (covariant return type).
    virtual ChPhysicsItem* Clone() const override { return new ChPhysicsItem(*this); }

    /// Get the pointer to the parent ChSystem().
    ChSystem* GetSystem() const { return system; }

    /// Set the pointer to the parent ChSystem().
    /// Also add to new collision system / remove from old collision system.
    virtual void SetSystem(ChSystem* m_system);

    // INTERFACES
    // inherited classes might/should implement some of the following functions

    /// Return true if the object is active and included in dynamics.
    virtual bool IsActive() const { return true; }

    // Collisions - override these in child classes if needed

    /// Tell if the object is subject to collision.
    /// Only for interface; child classes may override this, using internal flags.
    virtual bool IsCollisionEnabled() const { return false; }

    /// Add to the provided collision system any collision models managed by this physics item.
    /// A derived calss should invoke ChCollisionSystem::Add for each of its collision models.
    virtual void AddCollisionModelsToSystem(ChCollisionSystem* coll_sys) const {}

    /// Remove from the provided collision system any collision models managed by this physics item.
    /// A derived class should invoke ChCollisionSystem::Remove for each of its collision models.
    virtual void RemoveCollisionModelsFromSystem(ChCollisionSystem* coll_sys) const {}

    /// Synchronize the position and bounding box of any collision models managed by this physics item.
    virtual void SyncCollisionModels() {}

    /// Get the axis-aligned bounding box (AABB) of this object.
    /// The AABB must enclose all collision models, if any.
    /// By default, returns an infinite AABB.
    virtual ChAABB GetTotalAABB() const;

    /// Get a symbolic 'center' of the object. 
    /// By default this function returns the center of the AABB.
    /// Derived classes may override this, but must always return a point inside the AABB.
    virtual ChVector3d GetCenter() const;

    // UPDATE OPERATIONS

    /// Perform setup operations.
    /// This function is called at the beginning of a step, to recalculate quantities that may have changed (e.g.,
    /// number of coordinates, DOFs, constraints), as well as offsets in system-wide state vectors.
    virtual void Setup() {}

    /// Perform any updates necessary at the current phase during the solution process.
    /// This function is called at least once per step to update auxiliary data, internal states, etc.
    /// The default implementation updates the item's time stamp and its visualization assets (if any are defined anf
    /// only if requested).
    virtual void Update(double time, bool update_assets) override;

    /// Set zero speed (and zero accelerations) in state, without changing the position.
    /// Child classes should implement this function if GetNumCoordsPosLevel() > 0.
    /// It is used by owner ChSystem for some static analysis.
    virtual void ForceToRest() {}

    /// Get the number of coordinates at the position level.
    /// Might differ from coordinates at velocity level if quaternions are used for rotations.
    virtual unsigned int GetNumCoordsPosLevel() { return 0; }

    /// Get the number of coordinates at the velocity level.
    /// Might differ from coordinates at position level if quaternions are used for rotations.
    virtual unsigned int GetNumCoordsVelLevel() { return GetNumCoordsPosLevel(); }

    /// Get the number of scalar constraints.
    virtual unsigned int GetNumConstraints() { return GetNumConstraintsBilateral() + GetNumConstraintsUnilateral(); }

    /// Get the number of bilateral scalar constraints.
    virtual unsigned int GetNumConstraintsBilateral() { return 0; }

    /// Get the number of unilateral scalar constraints.
    virtual unsigned int GetNumConstraintsUnilateral() { return 0; }

    /// Get offset in the state vector (position part)
    unsigned int GetOffset_x() { return offset_x; }
    /// Get offset in the state vector (speed part)
    unsigned int GetOffset_w() { return offset_w; }
    /// Get offset in the lagrangian multipliers
    unsigned int GetOffset_L() { return offset_L; }

    /// Set offset in the state vector (position part)
    /// Note: only the ChSystem::Setup function should use this
    void SetOffset_x(const unsigned int moff) { offset_x = moff; }
    /// Set offset in the state vector (speed part)
    /// Note: only the ChSystem::Setup function should use this
    void SetOffset_w(const unsigned int moff) { offset_w = moff; }
    /// Set offset in the lagrangian multipliers
    /// Note: only the ChSystem::Setup function should use this
    void SetOffset_L(const unsigned int moff) { offset_L = moff; }

    /// From item's state to global state vectors y={x,v} pasting the states at the specified offsets.
    virtual void IntStateGather(const unsigned int off_x,  ///< offset in x state vector
                                ChState& x,                ///< state vector, position part
                                const unsigned int off_v,  ///< offset in v state vector
                                ChStateDelta& v,           ///< state vector, speed part
                                double& T                  ///< time
    ) {}

    /// From global state vectors y={x,v} to  item's state (and update) fetching the states at the specified offsets.
    virtual void IntStateScatter(const unsigned int off_x,  ///< offset in x state vector
                                 const ChState& x,          ///< state vector, position part
                                 const unsigned int off_v,  ///< offset in v state vector
                                 const ChStateDelta& v,     ///< state vector, speed part
                                 const double T,            ///< time
                                 bool full_update           ///< perform complete update
    ) {
        // Default behavior: even if no state is used, at least call Update()
        Update(T, full_update);
    }

    /// From item's state acceleration to global acceleration vector
    virtual void IntStateGatherAcceleration(const unsigned int off_a,  ///< offset in a accel. vector
                                            ChStateDelta& a            ///< acceleration part of state vector derivative
    ) {}

    /// From global acceleration vector to item's state acceleration
    virtual void IntStateScatterAcceleration(const unsigned int off_a,  ///< offset in a accel. vector
                                             const ChStateDelta& a  ///< acceleration part of state vector derivative
    ) {}

    /// From item's reaction forces to global reaction vector
    virtual void IntStateGatherReactions(const unsigned int off_L,  ///< offset in L vector
                                         ChVectorDynamic<>& L       ///< L vector of reaction forces
    ) {}

    /// From global reaction vector to item's reaction forces
    virtual void IntStateScatterReactions(const unsigned int off_L,   ///< offset in L vector
                                          const ChVectorDynamic<>& L  ///< L vector of reaction forces
    ) {}

    /// Computes x_new = x + Dt , using vectors at specified offsets.
    /// By default, when DOF = DOF_w, it does just the sum, but in some cases (ex when using quaternions
    /// for rotations) it could do more complex stuff, and children classes might overload it.
    virtual void IntStateIncrement(const unsigned int off_x,  ///< offset in x state vector
                                   ChState& x_new,            ///< state vector, position part, incremented result
                                   const ChState& x,          ///< state vector, initial position part
                                   const unsigned int off_v,  ///< offset in v state vector
                                   const ChStateDelta& Dv     ///< state vector, increment
    ) {
        for (unsigned int i = 0; i < GetNumCoordsPosLevel(); ++i) {
            x_new(off_x + i) = x(off_x + i) + Dv(off_v + i);
        }
    }

    /// Computes Dt = x_new - x, using vectors at specified offsets.
    /// By default, when DOF = DOF_w, it does just the difference of two state vectors, but in some cases (ex when using
    /// quaternions for rotations) it could do more complex stuff, and children classes might overload it.
    virtual void IntStateGetIncrement(const unsigned int off_x,  ///< offset in x state vector
                                      const ChState& x_new,      ///< state vector, final position part
                                      const ChState& x,          ///< state vector, initial position part
                                      const unsigned int off_v,  ///< offset in v state vector
                                      ChStateDelta& Dv           ///< state vector, increment. Here gets the result
    ) {
        for (unsigned int i = 0; i < GetNumCoordsPosLevel(); ++i) {
            Dv(off_v + i) = x_new(off_x + i) - x(off_x + i);
        }
    }

    /// Takes the F force term, scale and adds to R at given offset:
    ///    R += c*F
    virtual void IntLoadResidual_F(const unsigned int off,  ///< offset in R residual
                                   ChVectorDynamic<>& R,    ///< result: the R residual, R += c*F
                                   const double c           ///< a scaling factor
    ) {}

    /// Takes the M*v  term,  multiplying mass by a vector, scale and adds to R at given offset:
    ///    R += c*M*w
    virtual void IntLoadResidual_Mv(const unsigned int off,      ///< offset in R residual
                                    ChVectorDynamic<>& R,        ///< result: the R residual, R += c*M*v
                                    const ChVectorDynamic<>& w,  ///< the w vector
                                    const double c               ///< a scaling factor
    ) {}

    /// Adds the lumped mass to a Md vector, representing a mass diagonal matrix. Used by lumped explicit integrators.
    /// If mass lumping is impossible or approximate, adds scalar error to "error" parameter.
    ///    Md += c*diag(M)
    virtual void IntLoadLumpedMass_Md(const unsigned int off,  ///< offset in Md vector
                                      ChVectorDynamic<>& Md,  ///< result: Md vector, diagonal of the lumped mass matrix
                                      double& err,    ///< result: not touched if lumping does not introduce errors
                                      const double c  ///< a scaling factor
    ) {}

    /// Takes the term Cq'*L, scale and adds to R at given offset:
    ///    R += c*Cq'*L
    virtual void IntLoadResidual_CqL(const unsigned int off_L,    ///< offset in L multipliers
                                     ChVectorDynamic<>& R,        ///< result: the R residual, R += c*Cq'*L
                                     const ChVectorDynamic<>& L,  ///< the L vector
                                     const double c               ///< a scaling factor
    ) {}

    /// Takes the term C, scale and adds to Qc at given offset:
    ///    Qc += c*C
    virtual void IntLoadConstraint_C(const unsigned int off,  ///< offset in Qc residual
                                     ChVectorDynamic<>& Qc,   ///< result: the Qc residual, Qc += c*C
                                     const double c,          ///< a scaling factor
                                     bool do_clamp,           ///< apply clamping to c*C?
                                     double recovery_clamp    ///< value for min/max clamping of c*C
    ) {}

    /// Takes the term Ct, scale and adds to Qc at given offset:
    ///    Qc += c*Ct
    virtual void IntLoadConstraint_Ct(const unsigned int off,  ///< offset in Qc residual
                                      ChVectorDynamic<>& Qc,   ///< result: the Qc residual, Qc += c*Ct
                                      const double c           ///< a scaling factor
    ) {}

    /// Prepare variables and constraints to accommodate a solution:
    virtual void IntToDescriptor(
        const unsigned int off_v,    ///< offset for \e v and \e R
        const ChStateDelta& v,       ///< vector copied into the \e q 'unknowns' term of the variables
        const ChVectorDynamic<>& R,  ///< vector copied into the \e F 'force' term of the variables
        const unsigned int off_L,    ///< offset for \e L and \e Qc
        const ChVectorDynamic<>& L,  ///< vector copied into the \e L 'lagrangian ' term of the constraints
        const ChVectorDynamic<>& Qc  ///< vector copied into the \e Qb 'constraint' term of the constraints
    ) {}

    /// After a solver solution, fetch values from variables and constraints into vectors:
    virtual void IntFromDescriptor(
        const unsigned int off_v,  ///< offset for \e v
        ChStateDelta& v,           ///< vector to where the \e q 'unknowns' term of the variables will be copied
        const unsigned int off_L,  ///< offset for \e L
        ChVectorDynamic<>& L       ///< vector to where \e L 'lagrangian ' term of the constraints will be copied
    ) {}

    // SOLVER SYSTEM FUNCTIONS
    //
    // These are the functions that are used to manage ChConstraint and/or ChVariable
    // objects that are sent to the system solver.
    // The children classes, inherited from ChPhysicsItem, can implement them (by default,
    // the base ChPhysicsItem does not introduce any variable nor any constraint).

    /// Register with the given system descriptor any ChVariable objects associated with this item.
    virtual void InjectVariables(ChSystemDescriptor& descriptor) {}

    /// Register with the given system descriptor any ChConstraint objects associated with this item.
    virtual void InjectConstraints(ChSystemDescriptor& descriptor) {}

    /// Compute and load current Jacobians in encapsulated ChConstraint objects.
    virtual void LoadConstraintJacobians() {}

    /// Register with the given system descriptor any ChKRMBlock objects associated with this item.
    virtual void InjectKRMMatrices(ChSystemDescriptor& descriptor) {}

    /// Compute and load current stiffnes (K), damping (R), and mass (M) matrices in encapsulated ChKRMBlock objects.
    /// The resulting KRM blocks represent linear combinations of the K, R, and M matrices, with the specified
    /// coefficients Kfactor, Rfactor,and Mfactor, respectively.
    /// Note: signs are flipped from the term dF/dx in the integrator: K = -dF/dq and R = -dF/dv.
    virtual void LoadKRMMatrices(double Kfactor, double Rfactor, double Mfactor) {}

    // OLD BOOKKEEPING MECHANISM (marked for elimination)

    /// Sets the 'fb' part (the known term) of the encapsulated ChVariables to zero.
    virtual void VariablesFbReset() {}

    /// Adds the current forces (applied to item) into the
    /// encapsulated ChVariables, in the 'fb' part: qf+=forces*factor
    virtual void VariablesFbLoadForces(double factor = 1) {}

    /// Initialize the 'qb' part of the ChVariables with the
    /// current value of speeds. Note: since 'qb' is the unknown, this
    /// function seems unnecessary, unless used before VariablesFbIncrementMq()
    virtual void VariablesQbLoadSpeed() {}

    /// Adds M*q (masses multiplied current 'qb') to Fb, ex. if qb is initialized
    /// with v_old using VariablesQbLoadSpeed, this method can be used in
    /// timestepping schemes that do: M*v_new = M*v_old + forces*dt
    virtual void VariablesFbIncrementMq() {}

    /// Fetches the item speed (ex. linear and angular vel.in rigid bodies) from the
    /// 'qb' part of the ChVariables and sets it as the current item speed.
    /// If 'step' is not 0, also should compute the approximate acceleration of
    /// the item using backward differences, that is  accel=(new_speed-old_speed)/step.
    /// Mostly used after the solver provided the solution in ChVariables.
    virtual void VariablesQbSetSpeed(double step = 0) {}

    /// Increment item positions by the 'qb' part of the ChVariables,
    /// multiplied by a 'step' factor.
    ///     pos+=qb*step
    /// If qb is a speed, this behaves like a single step of 1-st order
    /// numerical integration (Euler integration).
    virtual void VariablesQbIncrementPosition(double step) {}

    /// Sets to zero the known term (b_i) of encapsulated ChConstraints
    virtual void ConstraintsBiReset() {}

    /// Adds the current C (constraint violation) to the known term (b_i) of
    /// encapsulated ChConstraints
    virtual void ConstraintsBiLoad_C(double factor = 1, double recovery_clamp = 0.1, bool do_clamp = false) {}

    /// Adds the current Ct (partial t-derivative, as in C_dt=0-> [Cq]*q_dt=-Ct)
    /// to the known term (b_i) of encapsulated ChConstraints
    virtual void ConstraintsBiLoad_Ct(double factor = 1) {}

    /// Adds the current Qc (the vector of C_dtdt=0 -> [Cq]*q_dtdt=Qc )
    /// to the known term (b_i) of encapsulated ChConstraints
    virtual void ConstraintsBiLoad_Qc(double factor = 1) {}

    /// Adds the current link-forces, if any, (caused by springs, etc.) to the 'fb' vectors
    /// of the ChVariables referenced by encapsulated ChConstraints
    virtual void ConstraintsFbLoadForces(double factor = 1) {}

    /// Fetches the reactions from the lagrangian multiplier (l_i)
    /// of encapsulated ChConstraints.
    /// Mostly used after the solver provided the solution in ChConstraints.
    /// Also, should convert the reactions obtained from dynamical simulation,
    /// from link space to intuitive react_force and react_torque.
    virtual void ConstraintsFetch_react(double factor = 1) {}

    // SERIALIZATION

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOut(ChArchiveOut& archive_out) override;

    /// Method to allow deserialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& archive_in) override;

  protected:
    ChSystem* system;  ///< parent system

    unsigned int offset_x;  ///< offset in vector of state (position part)
    unsigned int offset_w;  ///< offset in vector of state (speed part)
    unsigned int offset_L;  ///< offset in vector of lagrangian multipliers

  private:
    virtual void SetupInitial() {}

    friend class ChSystem;
    friend class ChAssembly;
    friend class modal::ChModalAssembly;
};

CH_CLASS_VERSION(ChPhysicsItem, 0)

}  // end namespace chrono

#endif
