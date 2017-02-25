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

#ifndef CHLINKMASKED_H
#define CHLINKMASKED_H

#include "chrono/physics/ChLinkForce.h"
#include "chrono/physics/ChLinkMarkers.h"
#include "chrono/physics/ChLinkMask.h"

namespace chrono {

// Forward references
class ChSystem;

/// Base class for all link joints which use a 'six degrees of freedom' mask
/// and auxiliary matrices to store jacobians. The management of jacobian matrices,
/// as well as other auxiliary matrices, is automatic as soon as the mask is
/// changed.
/// Also, links inherited from this class inherit six ChLinkForce objects, to
/// set inner spring-damper features for each degree of freedom.

class ChApi ChLinkMasked : public ChLinkMarkers {

    // Tag needed for class factory in archive (de)serialization:
    CH_FACTORY_TAG(ChLinkMasked)

  protected:
    // The mask of the locked coords, with the status of the
    // scalar constraints. This encapsulated object also
    // contains the jacobians and residuals for the solver.
    ChLinkMask* mask;  ///< scalar constraints

    // the following counters are cached here for optimization purposes
    int ndoc;    ///< number of DOC, degrees of costraint
    int ndoc_c;  ///< number of DOC, degrees of costraint (only bilaterals)
    int ndoc_d;  ///< number of DOC, degrees of costraint (only unilaterals)

    // internal forces
    ChLinkForce* force_D;   ///< the force acting on the straight line m1-m2 (distance)
    ChLinkForce* force_R;   ///< the torque acting about rotation axis
    ChLinkForce* force_X;   ///< the force acting along X dof
    ChLinkForce* force_Y;   ///< the force acting along Y dof
    ChLinkForce* force_Z;   ///< the force acting along Z dof
    ChLinkForce* force_Rx;  ///< the torque acting about Rx dof
    ChLinkForce* force_Ry;  ///< the torque acting about Ry dof
    ChLinkForce* force_Rz;  ///< the torque acting about Rz dof
    double d_restlength;    ///< the rest length of the "d_spring" spring

    ChMatrix<>* C;       ///< {C(q,q_dt,t)}, <<<<!!!   that is
    ChMatrix<>* C_dt;    ///< the violation= relC = C(q,qdt,t)
    ChMatrix<>* C_dtdt;  ///< also speed violations. and acc violations

    ChMatrix<>* Cq1;  ///< [Cq1], the jacobian of the constraint, for coords1, [ndoc,7]
    ChMatrix<>* Cq2;  ///< [Cq2], the jacobian of the constraint, for coords2. [ndoc,7]

    ChMatrix<>* Cqw1;  ///< [Cqw1], the jacobian [ndoc,6] for 3 Wl rot.coordinates instead of quaternions
    ChMatrix<>* Cqw2;  ///< [Cqw2], the jacobian [ndoc,6] for 3 Wl rot.coordinates instead of quaternions

    ChMatrix<>* Qc;  ///< {Qc}, the known part, {Qc}=-{C_dtdt}-([Cq]{q_dt})q-2[Cq_dt]{q_dt}

    ChMatrix<>* Ct;  ///< partial derivative of the link kin. equation wrt to time

    ChMatrix<>* react;  ///< {l}, the lagrangians forces in the constraints

  public:
    ChLinkMasked();
    ChLinkMasked(const ChLinkMasked& other);
    virtual ~ChLinkMasked();

    /// "Virtual" copy constructor (covariant return type).
    virtual ChLinkMasked* Clone() const override { return new ChLinkMasked(*this); }

  protected:
    // [mostly internal], allocates matrices and, in general, initializes all stuff
    // which is mask-dependant.  Sets n. DOF and n .DOC.
    // Copies the mask from new_mask.
    void BuildLink(ChLinkMask* new_mask);
    // [mostly internal]. as before, but uses the current (just changed) mask
    void BuildLink();
    // [mostly internal], frees matrices allocated by BuildLink
    void DestroyLink();

  public:
    /// Must be called after whatever change the mask of the link,
    /// in order to update auxiliary matrices sizes...
    void ChangeLinkMask(ChLinkMask* new_mask);

    /// Must be called after whatever change the mask of the link,
    /// in order to update auxiliary matrices sizes...
    void ChangedLinkMask();

    /// If some constraint is redundant, return to normal state
    int RestoreRedundant() override;  ///< \return number of changed states

    /// User can use this to enable/disable all the constraint of
    /// the link as desired.
    virtual void SetDisabled(bool mdis) override;

    /// Ex:3rd party software can set the 'broken' status via this method
    virtual void SetBroken(bool mon) override;

    /// Get the pointer to the link mask, ie. a ChLinkMask (sort of
    /// array containing a set of ChConstraint items).
    ChLinkMask* GetMask() { return mask; }

    /// overwrites inherited implementation of this method
    virtual void SetUpMarkers(ChMarker* mark1, ChMarker* mark2) override;

    //
    // STATE FUNCTIONS
    //

    virtual int GetDOC() override { return ndoc; }
    virtual int GetDOC_c() override { return ndoc_c; }
    virtual int GetDOC_d() override { return ndoc_d; }

    // (override/implement interfaces for global state vectors, see ChPhysicsItem for comments.)
    virtual void IntStateGatherReactions(const unsigned int off_L, ChVectorDynamic<>& L) override;
    virtual void IntStateScatterReactions(const unsigned int off_L, const ChVectorDynamic<>& L) override;
    virtual void IntLoadResidual_CqL(const unsigned int off_L,
                                     ChVectorDynamic<>& R,
                                     const ChVectorDynamic<>& L,
                                     const double c) override;
    virtual void IntLoadConstraint_C(const unsigned int off,
                                     ChVectorDynamic<>& Qc,
                                     const double c,
                                     bool do_clamp,
                                     double recovery_clamp) override;
    virtual void IntLoadConstraint_Ct(const unsigned int off, ChVectorDynamic<>& Qc, const double c) override;
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
    // SOLVER INTERFACE
    //

    virtual void InjectConstraints(ChSystemDescriptor& mdescriptor) override;
    virtual void ConstraintsBiReset() override;
    virtual void ConstraintsBiLoad_C(double factor = 1, double recovery_clamp = 0.1, bool do_clamp = false) override;
    virtual void ConstraintsBiLoad_Ct(double factor = 1) override;
    virtual void ConstraintsBiLoad_Qc(double factor = 1) override;
    virtual void ConstraintsLoadJacobians() override;
    virtual void ConstraintsFetch_react(double factor = 1) override;

    //
    // UPDATING FUNCTIONS
    //

    /// This is expected to update the values in C, C_dt, C_dtdt, in
    /// jacobians Cq1 and Cq2, in Qc and in Ct.
    /// By default, this does NOTHING, so it is up to the inherited
    /// classes to fill these vectors/matrices depending on how they
    /// describe the constraint equations.
    virtual void UpdateState() {}

    /// Updates Cqw1 and Cqw2  given updated  Cq1 and Cq2, i.e. computes the
    /// jacobians with 'Wl' rotational coordinates knowing the jacobians
    /// for body rotations in quaternion coordinates.
    virtual void UpdateCqw();

    /// Inherits, and updates the C_force and C_torque 'intuitive' forces,
    /// adding the effects of the contained ChLinkForce objects.
    /// (Default: inherits parent UpdateForces(), then C_force and C_torque are
    /// incremented with the Link::ChLinkForces objects)
    virtual void UpdateForces(double mytime) override;

    // -----------COMPLETE UPDATE.
    // sequence:
    //			UpdateTime;
    //          UpdateRelMarkerCoords;
    //			UpdateState;
    //          UpdateCqw
    //			UpdateForces;

    /// This following "complete" update functions actually fill all the
    /// matrices of the link, and does all calculus, by
    /// calling all the previous Update functions in sequence.
    virtual void Update(double mytime, bool update_assets = true) override;

    // LINK VIOLATIONS
    //
    // to get the constraint violations,
    // i.e. the residual of the constraint equations and their time derivatives

    /// Link violation (residuals of the link constraint equations)
    ChMatrix<>* GetC() { return C; }
    /// Time derivatives of link violations
    ChMatrix<>* GetC_dt() { return C_dt; }
    /// Double time derivatives of link violations
    ChMatrix<>* GetC_dtdt() { return C_dtdt; }

    // LINK STATE MATRICES
    //
    // Here follow the functions used by simulation engines to
    // fetch the system state matrices (the jacobians, the Q vector, etc.)
    // for building the state system matrices
    // Note that these function does not compute/update such matrices,
    // because all the updating/computing must be implemented in the Update...
    // functions above.

    /// The jacobian (body n.1 part, i.e. columns= 7 ,  rows= ndoc)
    ChMatrix<>* GetCq1() { return Cq1; }
    /// The jacobian (body n.2 part, i.e. columns= 7 ,  rows= ndoc)
    ChMatrix<>* GetCq2() { return Cq2; }

    /// The jacobian for Wl (col 6, rows= ndoc), as [Cqw1_rot]=[Cq_rot]*[Gl_1]'
    ChMatrix<>* GetCqw1() { return Cqw1; }
    /// The jacobian for Wl (col 6, rows= ndoc)	as [Cqw2_rot]=[Cq_rot]*[Gl_2]'
    ChMatrix<>* GetCqw2() { return Cqw2; }

    /// The gamma vector used in dynamics,  [Cq]x''=Qc
    ChMatrix<>* GetQc() { return Qc; }

    /// The Ct vector used in kinematics,  [Cq]x'=Ct
    ChMatrix<>* GetCt() { return Ct; }

    /// Access the reaction vector, after dynamics computations
    ChMatrix<>* GetReact() { return react; }

    //
    // OTHER DATA
    //

    // for the internal forces
    ChLinkForce* GetForce_D() { return force_D; }
    ChLinkForce* GetForce_R() { return force_R; }
    ChLinkForce* GetForce_X() { return force_X; }
    ChLinkForce* GetForce_Y() { return force_Y; }
    ChLinkForce* GetForce_Z() { return force_Z; }
    ChLinkForce* GetForce_Rx() { return force_Rx; }
    ChLinkForce* GetForce_Ry() { return force_Ry; }
    ChLinkForce* GetForce_Rz() { return force_Rz; }

    void SetForce_D(ChLinkForce* m_for) {
        if (force_D)
            delete force_D;
        force_D = m_for;
    }

    void SetForce_R(ChLinkForce* m_for) {
        if (force_R)
            delete force_R;
        force_R = m_for;
    }

    void SetForce_X(ChLinkForce* m_for) {
        if (force_X)
            delete force_X;
        force_X = m_for;
    }

    void SetForce_Y(ChLinkForce* m_for) {
        if (force_Y)
            delete force_Y;
        force_Y = m_for;
    }

    void SetForce_Z(ChLinkForce* m_for) {
        if (force_Z)
            delete force_Z;
        force_Z = m_for;
    }

    void SetForce_Rx(ChLinkForce* m_for) {
        if (force_Rx)
            delete force_Rx;
        force_Rx = m_for;
    }

    void SetForce_Ry(ChLinkForce* m_for) {
        if (force_Ry)
            delete force_Ry;
        force_Ry = m_for;
    }

    void SetForce_Rz(ChLinkForce* m_for) {
        if (force_Rz)
            delete force_Rz;
        force_Rz = m_for;
    }

    //
    // SERIALIZATION
    //

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOUT(ChArchiveOut& marchive) override;

    /// Method to allow deserialization of transient data from archives.
    virtual void ArchiveIN(ChArchiveIn& marchive) override;

  protected:
    // Internal use only - transforms a Nx7 jacobian matrix for a
    // body with rotations expressed as quaternions, into
    // a Nx6 jacobian matrix for a body with 'w' rotations.
    static void Transform_Cq_to_Cqw(ChMatrix<>* mCq, ChMatrix<>* mCqw, ChBodyFrame* mbody);
};

CH_CLASS_VERSION(ChLinkMasked,0)

}  // end namespace chrono

#endif
