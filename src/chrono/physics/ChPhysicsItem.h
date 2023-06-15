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
#include "chrono/physics/ChObject.h"
#include "chrono/assets/ChCamera.h"
#include "chrono/assets/ChVisualModel.h"
#include "chrono/collision/ChCollisionModel.h"
#include "chrono/solver/ChSystemDescriptor.h"
#include "chrono/timestepper/ChState.h"

namespace chrono {

// Forward references
class ChSystem;
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

    /// Add an (optional) visualization model.
    /// Not that an instance of the given visual model is associated with this physics ite, thus allowing sharing the
    /// same model among multiple items.
    void AddVisualModel(std::shared_ptr<ChVisualModel> model);

    /// Access the visualization model (if any).
    /// Note that this model may be shared with other physics items that may instance it.
    /// Returns nullptr if no visual model is present.
    std::shared_ptr<ChVisualModel> GetVisualModel() const;

    /// Add the specified visual shape to the visualization model.
    /// If this item does not have a visual model, one is created.
    void AddVisualShape(std::shared_ptr<ChVisualShape> shape, const ChFrame<>& frame = ChFrame<>());

    /// Access the specified visualization shape in the visualization model (if any).
    /// Note that no range check is performed.
    std::shared_ptr<ChVisualShape> GetVisualShape(unsigned int i) const;

    /// Add the specified FEA visualization object to the visualization model.
    /// If this item does not have a visual model, one is created.
    void AddVisualShapeFEA(std::shared_ptr<ChVisualShapeFEA> shapeFEA);

    /// Access the specified FEA visualization object in the visualization model (if any).
    /// Note that no range check is performed.
    std::shared_ptr<ChVisualShapeFEA> GetVisualShapeFEA(unsigned int i) const;

    /// Get the reference frame (expressed in and relative to the absolute frame) of the visual model.
    /// If the visual model is cloned (for example for a physics item modeling a particle system), this function returns
    /// the coordinate system of the specified clone.
    virtual ChFrame<> GetVisualModelFrame(unsigned int nclone = 0) { return ChFrame<>(); }

    /// Return the number of clones of the visual model associated with this physics item.
    /// If the visual model is cloned (for example for a physics item modeling a particle system), this function should
    /// return the total number of copies of the visual model, including the "original".  The current coordinate frame
    /// of a given clone can be obtained by calling GetVisualModelFrame() with the corresponding clone identifier.
    virtual unsigned int GetNumVisualModelClones() const { return 0; }

    /// Attach a ChCamera to this physical item.
    /// Multiple cameras can be attached to the same physics item.
    void AddCamera(std::shared_ptr<ChCamera> camera);

    /// Get the set of cameras attached to this physics item.
    std::vector<std::shared_ptr<ChCamera>> GetCameras() const { return cameras; }

    // INTERFACES
    // inherited classes might/should implement some of the following functions

    /// Return true if the object is active and included in dynamics.
    virtual bool IsActive() const { return true; }

    // Collisions - override these in child classes if needed

    /// Tell if the object is subject to collision.
    /// Only for interface; child classes may override this, using internal flags.
    virtual bool GetCollide() const { return false; }

    /// If this physical item contains one or more collision models,
    /// synchronize their coordinates and bounding boxes to the state of the item.
    virtual void SyncCollisionModels() {}

    /// If this physical item contains one or more collision models,
    /// add them to the system's collision engine.
    virtual void AddCollisionModelsToSystem() {}

    /// If this physical item contains one or more collision models,
    /// remove them from the system's collision engine.
    virtual void RemoveCollisionModelsFromSystem() {}

    // Functions used by domain decomposition

    /// Get the entire AABB axis-aligned bounding box of the object.
    /// The AABB must enclose the collision models, if any.
    /// By default is infinite AABB.
    /// Should be overridden by child classes.
    virtual void GetTotalAABB(ChVector<>& bbmin, ChVector<>& bbmax);

    /// Get a symbolic 'center' of the object. By default this
    /// function returns the center of the AABB.
    /// It could be overridden by child classes, anyway it must
    /// always get a point that must be inside AABB.
    virtual void GetCenter(ChVector<>& mcenter);

    /// Method to deserialize only the state (position, speed)
    /// Must be implemented by child classes.
    virtual void StreamInstate(ChStreamInBinary& mstream) {}
    /// Method to serialize only the state (position, speed)
    /// Must be implemented by child classes.
    virtual void StreamOutstate(ChStreamOutBinary& mstream) {}

    // UPDATING  - child classes may implement these functions

    /// This might recompute the number of coordinates, DOFs, constraints,
    /// in case this might change (ex in ChAssembly), as well as state offsets
    /// of contained items (ex in ChMesh)
    virtual void Setup() {}

    /// This is an important function, which is called by the
    /// owner ChSystem at least once per integration step.
    /// It may update all auxiliary data of the item, such as
    /// matrices if any, etc., depending on the current coordinates.
    /// The inherited classes, for example the ChLinkMask, often
    /// implement specialized versions of this Update(time) function,
    /// because they might need to update inner states, forces, springs, etc.
    /// This base version, by default, simply updates the item's time,
    /// and update the asset tree, if any.
    virtual void Update(double mytime, bool update_assets = true);

    /// As above, but does not require updating of time-dependent
    /// data. By default, calls Update(mytime) using item's current time.
    virtual void Update(bool update_assets = true) { Update(ChTime, update_assets); }

    /// Set zero speed (and zero accelerations) in state, without changing the position.
    /// Child classes should implement this function if GetDOF() > 0.
    /// It is used by owner ChSystem for some static analysis.
    virtual void SetNoSpeedNoAcceleration() {}

    // STATE FUNCTIONS
    //
    // These functions are used for bookkeeping in ChSystem, so that states (position, speeds)
    // of multiple physics items can be mapped in a single system state vector.
    // These will be used to interface to time integrators.
    // Note: these are not 'pure virtual' interfaces to avoid the burden of implementing all them
    // when just few are needed, so here is a default fallback that represent a 0 DOF, 0 DOC item, but
    // the children classes should override them.

    /// Get the number of scalar coordinates (variables), if any, in this item.
    /// Children classes must override this.
    virtual int GetDOF() { return 0; }
    /// Get the number of scalar coordinates of variables derivatives (usually = DOF, but might be
    /// different than DOF, ex. DOF=4 for quaternions, but DOF_w = 3 for its Lie algebra, ex angular velocity)
    /// Children classes might override this.
    virtual int GetDOF_w() { return GetDOF(); }
    /// Get the number of scalar constraints, if any, in this item
    virtual int GetDOC() { return GetDOC_c() + GetDOC_d(); }
    /// Get the number of scalar constraints, if any, in this item (only bilateral constr.)
    /// Children classes might override this.
    virtual int GetDOC_c() { return 0; }
    /// Get the number of scalar constraints, if any, in this item (only unilateral constr.)
    /// Children classes might override this.
    virtual int GetDOC_d() { return 0; }

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
        for (int i = 0; i < GetDOF(); ++i) {
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
        for (int i = 0; i < GetDOF(); ++i) {
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
    /// numerical integration (Eulero integration).
    virtual void VariablesQbIncrementPosition(double step) {}

    /// Tell to a system descriptor that there are variables of type
    /// ChVariables in this object (for further passing it to a solver)
    /// Basically does nothing, but maybe that inherited classes may specialize this.
    virtual void InjectVariables(ChSystemDescriptor& mdescriptor) {}

    /// Tell to a system descriptor that there are constraints of type
    /// ChConstraint in this object (for further passing it to a solver)
    /// Basically does nothing, but maybe that inherited classes may specialize this.
    virtual void InjectConstraints(ChSystemDescriptor& mdescriptor) {}

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

    /// Adds the current jacobians in encapsulated ChConstraints
    virtual void ConstraintsLoadJacobians() {}

    /// Fetches the reactions from the lagrangian multiplier (l_i)
    /// of encapsulated ChConstraints.
    /// Mostly used after the solver provided the solution in ChConstraints.
    /// Also, should convert the reactions obtained from dynamical simulation,
    /// from link space to intuitive react_force and react_torque.
    virtual void ConstraintsFetch_react(double factor = 1) {}

    /// Tell to a system descriptor that there are items of type
    /// ChKblock in this object (for further passing it to a solver)
    /// Basically does nothing, but maybe that inherited classes may specialize this.
    virtual void InjectKRMmatrices(ChSystemDescriptor& mdescriptor) {}

    /// Adds the current stiffness K and damping R and mass M matrices in encapsulated
    /// ChKblock item(s), if any. The K, R, M matrices are added with scaling
    /// values Kfactor, Rfactor, Mfactor.
    /// NOTE: signs are flipped respect to the ChTimestepper dF/dx terms:  K = -dF/dq, R = -dF/dv
    virtual void KRMmatricesLoad(double Kfactor, double Rfactor, double Mfactor) {}

    // SERIALIZATION

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOut(ChArchiveOut& marchive) override;

    /// Method to allow deserialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& marchive) override;

  protected:
    ChSystem* system;  ///< parent system

    std::shared_ptr<ChVisualModelInstance> vis_model_instance;  ///< instantiated visualization model
    std::vector<std::shared_ptr<ChCamera>> cameras;             ///< set of cameras

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
