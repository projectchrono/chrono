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

#ifndef CHPHYSICSITEM_H
#define CHPHYSICSITEM_H

#include "chrono/assets/ChAsset.h"
#include "chrono/collision/ChCCollisionModel.h"
#include "chrono/core/ChFrame.h"
#include "chrono/physics/ChObject.h"
#include "chrono/solver/ChSystemDescriptor.h"
#include "chrono/timestepper/ChState.h"

namespace chrono {

// Forward references
class ChSystem;

/// Base class for items that can contain objects of ChVariables or ChConstraints,
/// such as rigid bodies, mechanical joints, etc.

class ChApi ChPhysicsItem : public ChObj {

    // Tag needed for class factory in archive (de)serialization:
    CH_FACTORY_TAG(ChPhysicsItem)

    friend class ChSystem;

  protected:
    ChSystem* system;  ///< parent system

    std::vector<std::shared_ptr<ChAsset> > assets;  ///< set of assets

    unsigned int offset_x;  ///< offset in vector of state (position part)
    unsigned int offset_w;  ///< offset in vector of state (speed part)
    unsigned int offset_L;  ///< offset in vector of lagrangian multipliers

  private:
    virtual void SetupInitial() {}

  public:
    ChPhysicsItem() : system(NULL), offset_x(0), offset_w(0), offset_L(0) {}
    ChPhysicsItem(const ChPhysicsItem& other);
    virtual ~ChPhysicsItem();

    /// "Virtual" copy constructor (covariant return type).
    virtual ChPhysicsItem* Clone() const override { return new ChPhysicsItem(*this); }

    /// Get the pointer to the parent ChSystem()
    ChSystem* GetSystem() const { return system; }

    /// Set the pointer to the parent ChSystem() and
    /// also add to new collision system / remove from old coll.system
    virtual void SetSystem(ChSystem* m_system);

    /// Add an optional asset (it can be used to define visualization shapes, es ChSphereShape,
    /// or textures, or custom attached properties that the user can define by
    /// creating his class inherited from ChAsset)
    void AddAsset(std::shared_ptr<ChAsset> masset) { assets.push_back(masset); }

    /// Access to the list of optional assets.
    std::vector<std::shared_ptr<ChAsset> >& GetAssets() { return assets; }

    /// Access the Nth asset in the list of optional assets.
    std::shared_ptr<ChAsset> GetAssetN(unsigned int num);

    /// Get the master coordinate system for assets that have some geometric meaning.
    /// It could be used, for example, by a visualization system to show a 3d shape of this item.
    /// Children classes might override this (for example, for a ChBody, this will
    /// return the coordinate system of the rigid body).
    /// Optional parameter 'nclone' can be used for items that contain 'clones' (ex. lot
    /// of particles with the same visualization shape), so the corresponding coordinate frame
    /// can be returned.
    virtual ChFrame<> GetAssetsFrame(unsigned int nclone = 0) { return ChFrame<>(); }

    /// Optionally, a ChPhysicsItem can return multiple asset coordinate systems;
    /// this can be helpful if, for example, when a ChPhysicsItem contains 'clones'
    /// with the same assets (ex. lot of particle with the same visualization shape).
    /// If so, returns Nclones >0 , the number of clones including the original.
    /// Then use GetAssetsFrame(n), n=0...Nclones-1, to access the corresponding coord.frame.
    virtual unsigned int GetAssetsFrameNclones() { return 0; }

    //                   --- INTERFACES ---
    // inherited classes might/should implement some of the following functions.

    // Collisions - override these in child classes if needed

    /// Tell if the object is subject to collision.
    /// Only for interface; child classes may override this, using internal flags.
    virtual bool GetCollide() const { return false; }

    /// If this physical item contains one or more collision models,
    /// sinchronize their coordinates and bounding boxes to the state of the item.
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
    virtual void StreamINstate(ChStreamInBinary& mstream) {}
    /// Method to serialize only the state (position, speed)
    /// Must be implemented by child classes.
    virtual void StreamOUTstate(ChStreamOutBinary& mstream) {}

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
    /// Child classes should impement this function if GetDOF() > 0.
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

    /// From item's state to global state vectors y={x,v}
    /// pasting the states at the specified offsets.
    virtual void IntStateGather(const unsigned int off_x,  ///< offset in x state vector
                                ChState& x,                ///< state vector, position part
                                const unsigned int off_v,  ///< offset in v state vector
                                ChStateDelta& v,           ///< state vector, speed part
                                double& T                  ///< time
                                ) {}

    /// From global state vectors y={x,v} to  item's state (and update)
    /// fetching the states at the specified offsets.
    virtual void IntStateScatter(const unsigned int off_x,  ///< offset in x state vector
                                 const ChState& x,          ///< state vector, position part
                                 const unsigned int off_v,  ///< offset in v state vector
                                 const ChStateDelta& v,     ///< state vector, speed part
                                 const double T             ///< time
                                 ) {
        // Default behavior: even if no state is used, at least call Update()
        Update(T); 
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

    /// Prepare variables and constraints for a solution:
    /// From a vector R  into the F 'force' term of the variables
    /// From a vector Qc into the Qb 'constraint' term of the constraints
    /// From a vector v  into the q 'unknowns' term of the variables (for warm starting)
    /// From a vector L  into the L 'lagrangian ' term of the constraints (for warm starting)
    virtual void IntToDescriptor(const unsigned int off_v,  ///< offset in v, R
                                 const ChStateDelta& v,
                                 const ChVectorDynamic<>& R,
                                 const unsigned int off_L,  ///< offset in L, Qc
                                 const ChVectorDynamic<>& L,
                                 const ChVectorDynamic<>& Qc) {}

    /// After a solver solution, fetch values from variables and constraints:
    /// To a vector v  from the q 'unknowns' term of the variables
    /// To a vector L  from the L 'lagrangian ' term of the constraints
    virtual void IntFromDescriptor(const unsigned int off_v,  ///< offset in v
                                   ChStateDelta& v,
                                   const unsigned int off_L,  ///< offset in L
                                   ChVectorDynamic<>& L) {}

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

    /// Tell to a system descriptor that there are contraints of type
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

    //
    // SERIALIZATION
    //

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOUT(ChArchiveOut& marchive) override;

    /// Method to allow deserialization of transient data from archives.
    virtual void ArchiveIN(ChArchiveIn& marchive) override;
};

CH_CLASS_VERSION(ChPhysicsItem,0)

}  // end namespace chrono

#endif
