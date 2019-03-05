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

#ifndef CHLINKLOCK_H
#define CHLINKLOCK_H

#include "chrono/physics/ChLimit.h"
#include "chrono/physics/ChLinkForce.h"
#include "chrono/physics/ChLinkMarkers.h"
#include "chrono/physics/ChLinkMask.h"

namespace chrono {

/// Base class for joints implemented using the "lock formulation".
/// Derived classes provide models for revolute, prismatic, spherical, etc.
/// Note that certain kinematic joints (e.g., Universal) cannot be modeled using the lock formulation.
/// Joints of this type can optionally specify 'limits' over upper-lower motions of their respective
/// degrees of freedom, using the ChLinkLimit objects.
class ChApi ChLinkLock : public ChLinkMarkers {
  public:
    /// Default constructor. Builds a ChLinkLockSpherical.
    ChLinkLock();

    /// Copy constructor.
    ChLinkLock(const ChLinkLock& other);

    virtual ~ChLinkLock();

    /// "Virtual" copy constructor (covariant return type).
    virtual ChLinkLock* Clone() const override { return new ChLinkLock(*this); }

    /// Must be called after whatever change the mask of the link,
    /// in order to update auxiliary matrices sizes.
    void ChangeLinkMask(ChLinkMask* new_mask);

    /// Must be called after whatever change the mask of the link,
    /// in order to update auxiliary matrices sizes.
    void ChangedLinkMask();

    /// If some constraint is redundant, return to normal state.
    int RestoreRedundant() override;

    /// Enable/disable all constraints of the link.
    virtual void SetDisabled(bool mdis) override;

    /// For example, a 3rd party software can set the 'broken' status via this method
    virtual void SetBroken(bool mon) override;

    /// Get the pointer to the link mask, ie. a ChLinkMask (sort of
    /// array containing a set of ChConstraint items).
    ChLinkMask* GetMask() { return mask; }

    /// overwrites inherited implementation of this method
    virtual void SetUpMarkers(ChMarker* mark1, ChMarker* mark2) override;

    //@{
    /// Accessors for internal forces on free degrees of freedom.
    /// These functions provide access to initialize and set parameters for link
    /// forces acting on different degrees of freedom of the joint.
    /// Note that they use "lazy initialization"; an internal link force object is
    /// created on the first invocation of the corresponding accessor function.
    ChLinkForce& GetForce_D();
    ChLinkForce& GetForce_R();
    ChLinkForce& GetForce_X();
    ChLinkForce& GetForce_Y();
    ChLinkForce& GetForce_Z();
    ChLinkForce& GetForce_Rx();
    ChLinkForce& GetForce_Ry();
    ChLinkForce& GetForce_Rz();
    //@}

    //@{
    /// Accessors for limits on free degrees of freedom.
    /// These functions provide access to initialize and set parameters for link
    /// limits on different free degrees of freedom of the joint.
    /// Note that they use "lazy initialization"; an internal link limit object is
    /// created on the first invocation of the corresponding accessor function.
    ChLinkLimit& GetLimit_X();
    ChLinkLimit& GetLimit_Y();
    ChLinkLimit& GetLimit_Z();
    ChLinkLimit& GetLimit_Rx();
    ChLinkLimit& GetLimit_Ry();
    ChLinkLimit& GetLimit_Rz();
    ChLinkLimit& GetLimit_Rp();
    ChLinkLimit& GetLimit_D();
    //@}

    /// Get the number of scalar constraints for this link.
    virtual int GetDOC() override { return GetDOC_c() + GetDOC_d(); }

    /// Get the number of bilateral constraints for this link.
    virtual int GetDOC_c() override { return ndoc_c; }

    /// Get the number of unilateral constraints for this link.
    virtual int GetDOC_d() override;

    // LINK VIOLATIONS

    // Get the constraint violations, i.e. the residual of the constraint equations
    // and their time derivatives

    /// Link violation (residuals of the link constraint equations).
    ChMatrix<>* GetC() { return C; }
    /// Time derivatives of link violations.
    ChMatrix<>* GetC_dt() { return C_dt; }
    /// Double time derivatives of link violations.
    ChMatrix<>* GetC_dtdt() { return C_dtdt; }

    // LINK STATE MATRICES

    // Functions used by simulation engines to fetch the system state matrices
    // (the jacobians, the Q vector, etc.) for building the state system matrices
    // Note that these functions do not compute/update such matrices; this happens
    // in the Update functions.

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

    // UPDATE FUNCTIONS

    /// Given current time and body state, computes the constraint differentiation to get the
    /// the state matrices Cq1,  Cq2,  Qc,  Ct , and also C, C_dt, C_dtd.
    virtual void UpdateState();

    /// Updates the local F, M forces adding penalties from ChLinkLimit objects, if any.
    virtual void UpdateForces(double mytime) override;

    /// Updates Cqw1 and Cqw2  given updated  Cq1 and Cq2, i.e. computes the
    /// jacobians with 'Wl' rotational coordinates knowing the jacobians
    /// for body rotations in quaternion coordinates.
    void UpdateCqw();

    /// Full update. Fills-in all the matrices of the link, and does all required calculations
    /// by calling specific Update functions in sequence:
    /// <pre>
    ///     UpdateTime;
    ///     UpdateRelMarkerCoords;
    ///     UpdateState;
    ///     UpdateCqw
    ///     UpdateForces;
    /// </pre>
    virtual void Update(double mytime, bool update_assets = true) override;

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOUT(ChArchiveOut& marchive) override;

    /// Method to allow deserialization of transient data from archives.
    virtual void ArchiveIN(ChArchiveIn& marchive) override;

  protected:
    /// Type of link-lock
    enum class LinkType {
        LOCK,
        SPHERICAL,
        POINTPLANE,
        POINTLINE,
        CYLINDRICAL,
        PRISMATIC,
        PLANEPLANE,
        OLDHAM,
        REVOLUTE,
        FREE,
        ALIGN,
        PARALLEL,
        PERPEND,
        TRAJECTORY,
        CLEARANCE,
        REVOLUTEPRISMATIC
    };

    LinkType type;  ///< type of link_lock joint

    // The mask of the locked coords, with the status of the scalar constraints.
    // This object also encapsulates the jacobians and residuals for the solver.
    ChLinkMask* mask;  ///< scalar constraints

    // Degrees of constraint (excluding constraints from joint limits)
    int ndoc;    ///< number of degrees of constraint
    int ndoc_c;  ///< number of degrees of constraint (bilateral constraintss)
    int ndoc_d;  ///< number of degrees of constraint (unilateral constraints, excluding joint limits)

    std::unique_ptr<ChLinkForce> force_D;   ///< the force acting on the straight line m1-m2 (distance)
    std::unique_ptr<ChLinkForce> force_R;   ///< the torque acting about rotation axis
    std::unique_ptr<ChLinkForce> force_X;   ///< the force acting along X dof
    std::unique_ptr<ChLinkForce> force_Y;   ///< the force acting along Y dof
    std::unique_ptr<ChLinkForce> force_Z;   ///< the force acting along Z dof
    std::unique_ptr<ChLinkForce> force_Rx;  ///< the torque acting about Rx dof
    std::unique_ptr<ChLinkForce> force_Ry;  ///< the torque acting about Ry dof
    std::unique_ptr<ChLinkForce> force_Rz;  ///< the torque acting about Rz dof
    double d_restlength;                    ///< the rest length of the "d_spring" spring

    std::unique_ptr<ChLinkLimit> limit_X;   ///< the upper/lower limits for X dof
    std::unique_ptr<ChLinkLimit> limit_Y;   ///< the upper/lower limits for Y dof
    std::unique_ptr<ChLinkLimit> limit_Z;   ///< the upper/lower limits for Z dof
    std::unique_ptr<ChLinkLimit> limit_Rx;  ///< the upper/lower limits for Rx dof
    std::unique_ptr<ChLinkLimit> limit_Ry;  ///< the upper/lower limits for Ry dof
    std::unique_ptr<ChLinkLimit> limit_Rz;  ///< the upper/lower limits for Rz dof
    std::unique_ptr<ChLinkLimit> limit_Rp;  ///< the polar (conical) limit for "shoulder"rotation
    std::unique_ptr<ChLinkLimit> limit_D;   ///< the polar (conical) limit for "shoulder"rotation

    ChMatrix<>* C;       ///< C(q,q_dt,t), the constraint violations
    ChMatrix<>* C_dt;    ///< Speed constraint violations
    ChMatrix<>* C_dtdt;  ///< Acceleration constraint violations

    ChMatrix<>* Cq1;  ///< [Cq1], the jacobian of the constraint, for coords1, [ndoc,7]
    ChMatrix<>* Cq2;  ///< [Cq2], the jacobian of the constraint, for coords2. [ndoc,7]

    ChMatrix<>* Cqw1;  ///< [Cqw1], the jacobian [ndoc,6] for 3 Wl rot.coordinates instead of quaternions
    ChMatrix<>* Cqw2;  ///< [Cqw2], the jacobian [ndoc,6] for 3 Wl rot.coordinates instead of quaternions

    ChMatrix<>* Qc;  ///< {Qc}, the known part, {Qc}=-{C_dtdt}-([Cq]{q_dt})q-2[Cq_dt]{q_dt}

    ChMatrix<>* Ct;  ///< partial derivative of the link kin. equation wrt to time

    ChMatrix<>* react;  ///< {l}, the lagrangians forces in the constraints

    // Only for intermediate calculus
    ChMatrix<>* Cq1_temp;  //
    ChMatrix<>* Cq2_temp;  //   the temporary "lock" jacobians,
    ChMatrix<>* Qc_temp;   //   i.e. the full x,y,z,r0,r1,r2,r3 joint
    Coordsys Ct_temp;      //

  protected:
    /// Allocates matrices and initializes all mask-dependent quantities.
    /// Sets number of DOF and number DOC. Copies the mask from new_mask.
    void BuildLink(ChLinkMask* new_mask);

    /// Allocates matrices and initializes all mask-dependent quantities.
    /// Sets number of DOF and number DOC. Uses the current mask.
    void BuildLink();

    /// Frees matrices allocated by BuildLink.
    void DestroyLink();

    void ChangeLinkType(LinkType new_link_type);
    void BuildLinkType(LinkType link_type);

    // Extend parent functions to account for any ChLinkLimit objects.
    ////virtual void IntLoadResidual_F(const unsigned int off,	ChVectorDynamic<>& R, const double c );
    virtual void IntStateScatterReactions(const unsigned int off_L, const ChVectorDynamic<>& L) override;

    virtual void IntStateGatherReactions(const unsigned int off_L, ChVectorDynamic<>& L) override;

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

    // Extend parent constraint functions to consider constraints possibly induced by 'limits'.
    virtual void InjectConstraints(ChSystemDescriptor& mdescriptor) override;
    virtual void ConstraintsBiReset() override;
    virtual void ConstraintsBiLoad_C(double factor = 1, double recovery_clamp = 0.1, bool do_clamp = false) override;
    virtual void ConstraintsBiLoad_Ct(double factor = 1) override;
    virtual void ConstraintsBiLoad_Qc(double factor = 1) override;
    virtual void ConstraintsLoadJacobians() override;
    virtual void ConstraintsFetch_react(double factor = 1) override;

    friend class ChConveyor;
};

CH_CLASS_VERSION(ChLinkLock, 0)

// ---------------------------------------------------------------------------------------
// Specialization of a "weld" joint, in the link-lock formulation
// ---------------------------------------------------------------------------------------

/// 6-dof locked joint, with the link-lock formulation.
/// This specialization allows for specification of prescribed motion in the direction of
/// any of the 6 directions (3 translations and 3 rotations).
class ChApi ChLinkLockLock : public ChLinkLock {
  public:
    ChLinkLockLock();
    ChLinkLockLock(const ChLinkLockLock& other);

    /// "Virtual" copy constructor (covariant return type).
    virtual ChLinkLockLock* Clone() const override { return new ChLinkLockLock(*this); }

    // Imposed motion functions

    void SetMotion_X(std::shared_ptr<ChFunction> m_funct);
    void SetMotion_Y(std::shared_ptr<ChFunction> m_funct);
    void SetMotion_Z(std::shared_ptr<ChFunction> m_funct);
    void SetMotion_ang(std::shared_ptr<ChFunction> m_funct);
    void SetMotion_ang2(std::shared_ptr<ChFunction> m_funct);
    void SetMotion_ang3(std::shared_ptr<ChFunction> m_funct);
    void SetMotion_axis(Vector m_axis);
    void Set_angleset(AngleSet mset) { angleset = mset; }

    std::shared_ptr<ChFunction> GetMotion_X() const { return motion_X; }
    std::shared_ptr<ChFunction> GetMotion_Y() const { return motion_Y; }
    std::shared_ptr<ChFunction> GetMotion_Z() const { return motion_Z; }
    std::shared_ptr<ChFunction> GetMotion_ang() const { return motion_ang; }
    std::shared_ptr<ChFunction> GetMotion_ang2() const { return motion_ang2; }
    std::shared_ptr<ChFunction> GetMotion_ang3() const { return motion_ang3; }
    const ChVector<>& GetMotion_axis() const { return motion_axis; }
    AngleSet Get_angleset() const { return angleset; }

    /// Get constraint violations in pos/rot coordinates.
    const Coordsys& GetRelC() const { return relC; }
    /// Get first time derivative of constraint violations in pos/rot coordinates.
    const Coordsys& GetRelC_dt() const { return relC_dt; }
    /// Get second time derivative of constraint violations in pos/rot coordinates.
    const Coordsys& GetRelC_dtdt() const { return relC_dtdt; }

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOUT(ChArchiveOut& marchive) override;

    /// Method to allow deserialization of transient data from archives.
    virtual void ArchiveIN(ChArchiveIn& marchive) override;

  protected:
    std::shared_ptr<ChFunction> motion_X;     ///< user imposed motion for X coord, marker relative
    std::shared_ptr<ChFunction> motion_Y;     ///< user imposed motion for Y coord, marker relative
    std::shared_ptr<ChFunction> motion_Z;     ///< user imposed motion for Z coord, marker relative
    std::shared_ptr<ChFunction> motion_ang;   ///< user imposed angle rotation about axis
    std::shared_ptr<ChFunction> motion_ang2;  ///< user imposed angle rotation if three-angles rot.
    std::shared_ptr<ChFunction> motion_ang3;  ///< user imposed angle rotation if three-angles rot.
    Vector motion_axis;                       ///< this is the axis for the user imposed rotation
    AngleSet angleset;                        ///< type of rotation (3 Eul angles, angle/axis, etc.)

    Coordsys relC;       ///< relative constraint position: relC = (relM-deltaC)
    Coordsys relC_dt;    ///< relative constraint speed
    Coordsys relC_dtdt;  ///< relative constraint acceleration

    Coordsys deltaC;       ///< user-imposed rel. position
    Coordsys deltaC_dt;    ///< user-imposed rel. speed
    Coordsys deltaC_dtdt;  ///< user-imposed rel. acceleration

    /// Inherits, and also updates motion laws: deltaC, deltaC_dt, deltaC_dtdt
    virtual void UpdateTime(double mytime) override;

    /// Given current time and body state, computes the constraint differentiation to get the
    /// the state matrices Cq1,  Cq2,  Qc,  Ct , and also C, C_dt, C_dtd.
    virtual void UpdateState() override;
};

CH_CLASS_VERSION(ChLinkLockLock, 0)

// ---------------------------------------------------------------------------------------
// Concrete joints using the link-lock formulation
// ---------------------------------------------------------------------------------------

/// Revolute joint, with the 'ChLinkLock' formulation.
/// (allows a simpler creation of a link as a sub-type of ChLinkLock).
class ChApi ChLinkLockRevolute : public ChLinkLock {
  public:
    ChLinkLockRevolute() { ChangeLinkType(LinkType::REVOLUTE); }

    /// "Virtual" copy constructor (covariant return type).
    virtual ChLinkLockRevolute* Clone() const override { return new ChLinkLockRevolute(*this); }
};

/// Spherical joint, with the 'ChLinkLock' formulation.
/// (allows a simpler creation of a link as a sub-type of ChLinkLock).
class ChApi ChLinkLockSpherical : public ChLinkLock {
  public:
    ChLinkLockSpherical() { ChangeLinkType(LinkType::SPHERICAL); }

    /// "Virtual" copy constructor (covariant return type).
    virtual ChLinkLockSpherical* Clone() const override { return new ChLinkLockSpherical(*this); }
};

/// Cylindrical joint, with the 'ChLinkLock' formulation.
/// (allows a simpler creation of a link as a sub-type of ChLinkLock).
class ChApi ChLinkLockCylindrical : public ChLinkLock {
  public:
    ChLinkLockCylindrical() { ChangeLinkType(LinkType::CYLINDRICAL); }

    /// "Virtual" copy constructor (covariant return type).
    virtual ChLinkLockCylindrical* Clone() const override { return new ChLinkLockCylindrical(*this); }
};

/// Prismatic joint, with the 'ChLinkLock' formulation.
/// Default axis along +z
class ChApi ChLinkLockPrismatic : public ChLinkLock {
  public:
    ChLinkLockPrismatic() { ChangeLinkType(LinkType::PRISMATIC); }

    /// "Virtual" copy constructor (covariant return type).
    virtual ChLinkLockPrismatic* Clone() const override { return new ChLinkLockPrismatic(*this); }
};

/// Point-plane joint, with the 'ChLinkLock' formulation.
/// (allows a simpler creation of a link as a sub-type of ChLinkLock).
class ChApi ChLinkLockPointPlane : public ChLinkLock {
  public:
    ChLinkLockPointPlane() { ChangeLinkType(LinkType::POINTPLANE); }

    /// "Virtual" copy constructor (covariant return type).
    virtual ChLinkLockPointPlane* Clone() const override { return new ChLinkLockPointPlane(*this); }
};

/// Point-line joint, with the 'ChLinkLock' formulation.
/// (allows a simpler creation of a link as a sub-type of ChLinkLock).
class ChApi ChLinkLockPointLine : public ChLinkLock {
  public:
    ChLinkLockPointLine() { ChangeLinkType(LinkType::POINTLINE); }

    /// "Virtual" copy constructor (covariant return type).
    virtual ChLinkLockPointLine* Clone() const override { return new ChLinkLockPointLine(*this); }
};

/// Plane-plane joint, with the 'ChLinkLock' formulation.
/// (allows a simpler creation of a link as a sub-type of ChLinkLock).
class ChApi ChLinkLockPlanePlane : public ChLinkLock {
  public:
    ChLinkLockPlanePlane() { ChangeLinkType(LinkType::PLANEPLANE); }

    /// "Virtual" copy constructor (covariant return type).
    virtual ChLinkLockPlanePlane* Clone() const override { return new ChLinkLockPlanePlane(*this); }
};

/// Oldham joint, with the 'ChLinkLock' formulation.
/// (allows a simpler creation of a link as a sub-type of ChLinkLock).
class ChApi ChLinkLockOldham : public ChLinkLock {
  public:
    ChLinkLockOldham() { ChangeLinkType(LinkType::OLDHAM); }

    /// "Virtual" copy constructor (covariant return type).
    virtual ChLinkLockOldham* Clone() const override { return new ChLinkLockOldham(*this); }
};

/// Free joint, with the 'ChLinkLock' formulation.
/// (allows a simpler creation of a link as a sub-type of ChLinkLock).
class ChApi ChLinkLockFree : public ChLinkLock {
  public:
    ChLinkLockFree() { ChangeLinkType(LinkType::FREE); }

    /// "Virtual" copy constructor (covariant return type).
    virtual ChLinkLockFree* Clone() const override { return new ChLinkLockFree(*this); }
};

/// Align joint, with the 'ChLinkLock' formulation.
/// (allows a simpler creation of a link as a sub-type of ChLinkLock).
class ChApi ChLinkLockAlign : public ChLinkLock {
  public:
    ChLinkLockAlign() { ChangeLinkType(LinkType::ALIGN); }

    /// "Virtual" copy constructor (covariant return type).
    virtual ChLinkLockAlign* Clone() const override { return new ChLinkLockAlign(*this); }
};

/// Parallel joint, with the 'ChLinkLock' formulation.
/// (allows a simpler creation of a link as a sub-type of ChLinkLock).
class ChApi ChLinkLockParallel : public ChLinkLock {
  public:
    ChLinkLockParallel() { ChangeLinkType(LinkType::PARALLEL); }

    /// "Virtual" copy constructor (covariant return type).
    virtual ChLinkLockParallel* Clone() const override { return new ChLinkLockParallel(*this); }
};

/// Perpendicularity joint, with the 'ChLinkLock' formulation.
/// (allows a simpler creation of a link as a sub-type of ChLinkLock).
class ChApi ChLinkLockPerpend : public ChLinkLock {
  public:
    ChLinkLockPerpend() { ChangeLinkType(LinkType::PERPEND); }

    /// "Virtual" copy constructor (covariant return type).
    virtual ChLinkLockPerpend* Clone() const override { return new ChLinkLockPerpend(*this); }
};

/// RevolutePrismatic joint, with the 'ChLinkLock' formulation.
/// Translates along x-dir, rotates about z-axis
class ChApi ChLinkLockRevolutePrismatic : public ChLinkLock {
  public:
    ChLinkLockRevolutePrismatic() { ChangeLinkType(LinkType::REVOLUTEPRISMATIC); }

    /// "Virtual" copy constructor (covariant return type).
    virtual ChLinkLockRevolutePrismatic* Clone() const override { return new ChLinkLockRevolutePrismatic(*this); }
};

}  // end namespace chrono

#endif
