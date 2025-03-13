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

#include "chrono/physics/ChLinkLimit.h"
#include "chrono/physics/ChLinkForce.h"
#include "chrono/physics/ChLinkMarkers.h"
#include "chrono/physics/ChLinkMask.h"

namespace chrono {

/// Base class for joints implemented using the "lock" formulation.
/// This implementation of joints allow the constrained frame (ChMarker) to move with respect to the constrained body.
/// Moreover, a ChLinkLock joint can:
/// - limit the range of motion of those unconstrained degrees of freedom by means of ChLinkLimit objects.
/// - impose a relative force or a torque on the constrained marker by means of ChLinkForce objects.
class ChApi ChLinkLock : public ChLinkMarkers {
  public:
    using ChConstraintVectorX = Eigen::Matrix<double, Eigen::Dynamic, 1, Eigen::ColMajor, 7, 1>;
    using ChConstraintMatrixX6 = Eigen::Matrix<double, Eigen::Dynamic, BODY_DOF, Eigen::RowMajor, 7, BODY_DOF>;
    using ChConstraintMatrixX7 = Eigen::Matrix<double, Eigen::Dynamic, BODY_QDOF, Eigen::RowMajor, 7, BODY_QDOF>;

    /// Default constructor. Builds a ChLinkLockSpherical.
    ChLinkLock();

    /// Copy constructor.
    ChLinkLock(const ChLinkLock& other);

    virtual ~ChLinkLock();

    /// "Virtual" copy constructor (covariant return type).
    virtual ChLinkLock* Clone() const override { return new ChLinkLock(*this); }

    /// Enable/disable all constraints of the link.
    virtual void SetDisabled(bool mdis) override;

    /// For example, a 3rd party software can set the 'broken' status via this method
    virtual void SetBroken(bool mon) override;

    /// Get the link mask (a container for the ChConstraint items).
    ChLinkMask& GetMask() { return mask; }

    /// Set the two markers associated with this link.
    virtual void SetupMarkers(ChMarker* mark1, ChMarker* mark2) override;

    //@{
    /// Accessors for internal forces on free degrees of freedom.
    /// These functions provide access to initialize and set parameters for link
    /// forces acting on different degrees of freedom of the joint.
    /// Note that they use "lazy initialization"; an internal link force object is
    /// created on the first invocation of the corresponding accessor function.
    ChLinkForce& ForceX();
    ChLinkForce& ForceY();
    ChLinkForce& ForceZ();
    ChLinkForce& ForceRx();
    ChLinkForce& ForceRy();
    ChLinkForce& ForceRz();
    ChLinkForce& ForceD();
    ChLinkForce& ForceRp();
    //@}

    //@{
    /// Accessors for limits on free degrees of freedom.
    /// These functions provide access to initialize and set parameters for link
    /// limits on different free degrees of freedom of the joint.
    /// Note that they use "lazy initialization"; an internal link limit object is
    /// created on the first invocation of the corresponding accessor function.
    ChLinkLimit& LimitX();
    ChLinkLimit& LimitY();
    ChLinkLimit& LimitZ();
    ChLinkLimit& LimitRx();
    ChLinkLimit& LimitRy();
    ChLinkLimit& LimitRz();
    ChLinkLimit& LimitD();
    ChLinkLimit& LimitRp();
    //@}

    /// Get the number of scalar constraints for this link.
    virtual unsigned int GetNumConstraints() override {
        return GetNumConstraintsBilateral() + GetNumConstraintsUnilateral();
    }

    /// Get the number of bilateral constraints for this link.
    virtual unsigned int GetNumConstraintsBilateral() override { return m_num_constr_bil; }

    /// Get the number of unilateral constraints for this link.
    virtual unsigned int GetNumConstraintsUnilateral() override;

    // LINK VIOLATIONS

    /// Link violation (residuals of the link constraint equations).
    virtual ChVectorDynamic<> GetConstraintViolation() const override { return C; }

    /// Time derivatives of link violation.
    const ChConstraintVectorX& GetConstraintViolationDt() const { return C_dt; }

    /// Second time derivatives of link violation.
    const ChConstraintVectorX& GetConstraintViolationDt2() const { return C_dtdt; }

    // LINK STATE MATRICES

    // Functions used by simulation engines to fetch the system state matrices
    // (the jacobians, the Q vector, etc.) for building the state system matrices
    // Note that these functions do not compute/update such matrices; this happens
    // in the Update functions.

    /// The jacobian (body n.1 part, i.e. columns= 7 ,  rows= m_num_constr)
    const ChConstraintMatrixX7& GetCq1() const { return Cq1; }
    /// The jacobian (body n.2 part, i.e. columns= 7 ,  rows= m_num_constr)
    const ChConstraintMatrixX7& GetCq2() const { return Cq2; }

    /// The jacobian for Wl (col 6, rows= m_num_constr), as [Cqw1_rot]=[Cq_rot]*[Gl_1]'
    const ChConstraintMatrixX6& GetCqw1() const { return Cqw1; }
    /// The jacobian for Wl (col 6, rows= m_num_constr)	as [Cqw2_rot]=[Cq_rot]*[Gl_2]'
    const ChConstraintMatrixX6& GetCqw2() const { return Cqw2; }

    /// The gamma vector used in dynamics,  [Cq]x''=Qc
    const ChConstraintVectorX& GetQc() const { return Q_c; }

    /// The Ct vector used in kinematics,  [Cq]x'=Ct
    const ChConstraintVectorX& GetCt() const { return Ct; }

    /// Access the reaction vector, after dynamics computations
    const ChConstraintVectorX& GetReact() const { return react; }

    // UPDATE FUNCTIONS

    /// Update time-dependent quantities in link state (e.g., motion laws, moving markers, etc.).
    virtual void UpdateTime(double mytime);

    /// Given current time and body state, computes the constraint differentiation to get the the state matrices Cq1,
    /// Cq2,  Qc,  Ct , and also C, C_dt, C_dtd.
    virtual void UpdateState();

    /// Updates the local F, M forces adding penalties from ChLinkLimit objects, if any.
    virtual void UpdateForces(double time) override;

    /// Updates Cqw1 and Cqw2  given updated  Cq1 and Cq2, i.e. computes the jacobians with 'Wl' rotational coordinates
    /// knowing the jacobians for body rotations in quaternion coordinates.
    void UpdateCqw();

    /// Full update. Fills-in all the matrices of the link, and does all required calculations by calling specific
    /// Update functions in sequence:
    /// - UpdateTime;
    /// - UpdateRelMarkerCoords;
    /// - UpdateState;
    /// - UpdateCqw
    /// - UpdateForces;
    virtual void Update(double time, bool update_assets) override;

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOut(ChArchiveOut& archive_out) override;

    /// Method to allow deserialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& archive_in) override;

  protected:
    /// Type of link-lock
    enum class Type {
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

    Type type;  ///< type of link_lock joint

    // The mask of the locked coords, with the status of the scalar constraints.
    // This object also encapsulates the jacobians and residuals for the solver.
    ChLinkMaskLF mask;  ///< scalar constraints

    // Degrees of constraint (excluding constraints from joint limits)
    int m_num_constr;      ///< number of degrees of constraint
    int m_num_constr_bil;  ///< number of degrees of constraint (bilateral constraintss)
    int m_num_constr_uni;  ///< number of degrees of constraint (unilateral constraints, excluding joint limits)

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

    ChConstraintVectorX C;       ///< C(q,q_dt,t), the constraint violations
    ChConstraintVectorX C_dt;    ///< Speed constraint violations
    ChConstraintVectorX C_dtdt;  ///< Acceleration constraint violations

    ChConstraintMatrixX7 Cq1;  ///< [Cq1], the jacobian of the constraint, for coords1, [m_num_constr,7]
    ChConstraintMatrixX7 Cq2;  ///< [Cq2], the jacobian of the constraint, for coords2. [m_num_constr,7]

    ChConstraintMatrixX6 Cqw1;  ///< Jacobian [m_num_constr,6] for 3 Wl rot.coordinates instead of quaternions
    ChConstraintMatrixX6 Cqw2;  ///< Jacobian [m_num_constr,6] for 3 Wl rot.coordinates instead of quaternions

    ChConstraintVectorX Q_c;     ///< {Qc}, the known part, {Qc}=-{C_dtdt}-([Cq]{q_dt})q-2[Cq_dt]{q_dt}
    ChConstraintVectorX Ct;     ///< partial derivative of the link kin. equation wrt to time
    ChConstraintVectorX react;  ///< {l}, the lagrangians forces in the constraints

    // Only for intermediate calculus
    ChMatrixNM<double, 7, BODY_QDOF> Cq1_temp;  //
    ChMatrixNM<double, 7, BODY_QDOF> Cq2_temp;  //   the temporary "lock" jacobians,
    ChVectorN<double, 7> Q_c_temp;              //   i.e. the full x,y,z,r0,r1,r2,r3 joint
    ChCoordsysd Ct_temp;                        //

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  protected:
    /// Resize matrices and initializes all mask-dependent quantities.
    /// Sets number of constraints based on current mask information.
    void BuildLink();

    /// Set the mask and then resize matrices.
    void BuildLinkType(Type link_type);
    void BuildLink(bool x, bool y, bool z, bool e0, bool e1, bool e2, bool e3);

    void ChangeType(Type new_link_type);

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
    virtual void InjectConstraints(ChSystemDescriptor& descriptor) override;
    virtual void ConstraintsBiReset() override;
    virtual void ConstraintsBiLoad_C(double factor = 1, double recovery_clamp = 0.1, bool do_clamp = false) override;
    virtual void ConstraintsBiLoad_Ct(double factor = 1) override;
    virtual void ConstraintsBiLoad_Qc(double factor = 1) override;
    virtual void LoadConstraintJacobians() override;
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

    void SetMotionX(std::shared_ptr<ChFunction> m_funct);
    void SetMotionY(std::shared_ptr<ChFunction> m_funct);
    void SetMotionZ(std::shared_ptr<ChFunction> m_funct);
    void SetMotionAng1(std::shared_ptr<ChFunction> m_funct);
    void SetMotionAng2(std::shared_ptr<ChFunction> m_funct);
    void SetMotionAng3(std::shared_ptr<ChFunction> m_funct);
    void SetMotionAxis(ChVector3d m_axis);

    void SetRotationRepresentation(RotRepresentation rot_rep);

    std::shared_ptr<ChFunction> GetMotionX() const { return motion_X; }
    std::shared_ptr<ChFunction> GetMotionY() const { return motion_Y; }
    std::shared_ptr<ChFunction> GetMotionZ() const { return motion_Z; }
    std::shared_ptr<ChFunction> GetMotionAng1() const { return motion_ang; }
    std::shared_ptr<ChFunction> GetMotionAng2() const { return motion_ang2; }
    std::shared_ptr<ChFunction> GetMotionAng3() const { return motion_ang3; }
    const ChVector3d& GetMotionAxis() const { return motion_axis; }

    RotRepresentation GetRotationRepresentation() const { return angleset; }

    /// Get constraint violations in pos/rot coordinates.
    const ChCoordsysd& GetRelCoordsysViolation() const { return relC; }

    /// Get first time derivative of constraint violations in pos/rot coordinates.
    const ChCoordsysd& GetRelCoordsysViolationDt() const { return relC_dt; }

    /// Get second time derivative of constraint violations in pos/rot coordinates.
    const ChCoordsysd& GetRelCoordsysViolationDt2() const { return relC_dtdt; }

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOut(ChArchiveOut& archive_out) override;

    /// Method to allow deserialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& archive_in) override;

  protected:
    std::shared_ptr<ChFunction> motion_X;     ///< user imposed motion for X coord, marker relative
    std::shared_ptr<ChFunction> motion_Y;     ///< user imposed motion for Y coord, marker relative
    std::shared_ptr<ChFunction> motion_Z;     ///< user imposed motion for Z coord, marker relative
    std::shared_ptr<ChFunction> motion_ang;   ///< user imposed angle rotation about axis
    std::shared_ptr<ChFunction> motion_ang2;  ///< user imposed angle rotation if three-angles rot.
    std::shared_ptr<ChFunction> motion_ang3;  ///< user imposed angle rotation if three-angles rot.
    ChVector3d motion_axis;                   ///< this is the axis for the user imposed rotation
    RotRepresentation angleset;               ///< type of rotation (3 Eul angles, angle/axis, etc.)

    ChCoordsysd relC;       ///< relative constraint position: relC = (relM-deltaC)
    ChCoordsysd relC_dt;    ///< relative constraint speed
    ChCoordsysd relC_dtdt;  ///< relative constraint acceleration

    ChCoordsysd deltaC;       ///< user-imposed rel. position
    ChCoordsysd deltaC_dt;    ///< user-imposed rel. speed
    ChCoordsysd deltaC_dtdt;  ///< user-imposed rel. acceleration

    /// Update time-dependent quantities in link state.
    virtual void UpdateTime(double time) override;

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
    ChLinkLockRevolute() { ChangeType(Type::REVOLUTE); }

    /// "Virtual" copy constructor (covariant return type).
    virtual ChLinkLockRevolute* Clone() const override { return new ChLinkLockRevolute(*this); }

    /// Lock the joint.
    /// If enabled (lock = true) this effectively converts this joint into a weld joint.
    /// If lock = false, the joint reverts to its original degrees of freedom.
    void Lock(bool lock);
};

/// Spherical joint, with the 'ChLinkLock' formulation.
/// (allows a simpler creation of a link as a sub-type of ChLinkLock).
class ChApi ChLinkLockSpherical : public ChLinkLock {
  public:
    ChLinkLockSpherical() { ChangeType(Type::SPHERICAL); }

    /// "Virtual" copy constructor (covariant return type).
    virtual ChLinkLockSpherical* Clone() const override { return new ChLinkLockSpherical(*this); }

    /// Lock the joint.
    /// If enabled (lock = true) this effectively converts this joint into a weld joint.
    /// If lock = false, the joint reverts to its original degrees of freedom.
    void Lock(bool lock);
};

/// Cylindrical joint, with the 'ChLinkLock' formulation.
/// (allows a simpler creation of a link as a sub-type of ChLinkLock).
class ChApi ChLinkLockCylindrical : public ChLinkLock {
  public:
    ChLinkLockCylindrical() { ChangeType(Type::CYLINDRICAL); }

    /// "Virtual" copy constructor (covariant return type).
    virtual ChLinkLockCylindrical* Clone() const override { return new ChLinkLockCylindrical(*this); }

    /// Lock the joint.
    /// If enabled (lock = true) this effectively converts this joint into a weld joint.
    /// If lock = false, the joint reverts to its original degrees of freedom.
    void Lock(bool lock);
};

/// Prismatic joint, with the 'ChLinkLock' formulation.
/// Default axis along +z
class ChApi ChLinkLockPrismatic : public ChLinkLock {
  public:
    ChLinkLockPrismatic() { ChangeType(Type::PRISMATIC); }

    /// "Virtual" copy constructor (covariant return type).
    virtual ChLinkLockPrismatic* Clone() const override { return new ChLinkLockPrismatic(*this); }

    /// Lock the joint.
    /// If enabled (lock = true) this effectively converts this joint into a weld joint.
    /// If lock = false, the joint reverts to its original degrees of freedom.
    void Lock(bool lock);
};

/// Point-plane joint, with the 'ChLinkLock' formulation.
/// (allows a simpler creation of a link as a sub-type of ChLinkLock).
class ChApi ChLinkLockPointPlane : public ChLinkLock {
  public:
    ChLinkLockPointPlane() { ChangeType(Type::POINTPLANE); }

    /// "Virtual" copy constructor (covariant return type).
    virtual ChLinkLockPointPlane* Clone() const override { return new ChLinkLockPointPlane(*this); }

    /// Lock the joint.
    /// If enabled (lock = true) this effectively converts this joint into a weld joint.
    /// If lock = false, the joint reverts to its original degrees of freedom.
    void Lock(bool lock);
};

/// Point-line joint, with the 'ChLinkLock' formulation.
/// (allows a simpler creation of a link as a sub-type of ChLinkLock).
class ChApi ChLinkLockPointLine : public ChLinkLock {
  public:
    ChLinkLockPointLine() { ChangeType(Type::POINTLINE); }

    /// "Virtual" copy constructor (covariant return type).
    virtual ChLinkLockPointLine* Clone() const override { return new ChLinkLockPointLine(*this); }

    /// Lock the joint.
    /// If enabled (lock = true) this effectively converts this joint into a weld joint.
    /// If lock = false, the joint reverts to its original degrees of freedom.
    void Lock(bool lock);
};

/// Plane-plane joint, with the 'ChLinkLock' formulation.
/// (allows a simpler creation of a link as a sub-type of ChLinkLock).
class ChApi ChLinkLockPlanar : public ChLinkLock {
  public:
    ChLinkLockPlanar() { ChangeType(Type::PLANEPLANE); }

    /// "Virtual" copy constructor (covariant return type).
    virtual ChLinkLockPlanar* Clone() const override { return new ChLinkLockPlanar(*this); }

    /// Lock the joint.
    /// If enabled (lock = true) this effectively converts this joint into a weld joint.
    /// If lock = false, the joint reverts to its original degrees of freedom.
    void Lock(bool lock);
};

/// Oldham joint, with the 'ChLinkLock' formulation.
/// (allows a simpler creation of a link as a sub-type of ChLinkLock).
class ChApi ChLinkLockOldham : public ChLinkLock {
  public:
    ChLinkLockOldham() { ChangeType(Type::OLDHAM); }

    /// "Virtual" copy constructor (covariant return type).
    virtual ChLinkLockOldham* Clone() const override { return new ChLinkLockOldham(*this); }

    /// Lock the joint.
    /// If enabled (lock = true) this effectively converts this joint into a weld joint.
    /// If lock = false, the joint reverts to its original degrees of freedom.
    void Lock(bool lock);
};

/// Free joint, with the 'ChLinkLock' formulation.
/// (allows a simpler creation of a link as a sub-type of ChLinkLock).
class ChApi ChLinkLockFree : public ChLinkLock {
  public:
    ChLinkLockFree() { ChangeType(Type::FREE); }

    /// "Virtual" copy constructor (covariant return type).
    virtual ChLinkLockFree* Clone() const override { return new ChLinkLockFree(*this); }

    /// Lock the joint.
    /// If enabled (lock = true) this effectively converts this joint into a weld joint.
    /// If lock = false, the joint reverts to its original degrees of freedom.
    void Lock(bool lock);
};

/// Align joint, with the 'ChLinkLock' formulation.
/// (allows a simpler creation of a link as a sub-type of ChLinkLock).
class ChApi ChLinkLockAlign : public ChLinkLock {
  public:
    ChLinkLockAlign() { ChangeType(Type::ALIGN); }

    /// "Virtual" copy constructor (covariant return type).
    virtual ChLinkLockAlign* Clone() const override { return new ChLinkLockAlign(*this); }

    /// Lock the joint.
    /// If enabled (lock = true) this effectively converts this joint into a weld joint.
    /// If lock = false, the joint reverts to its original degrees of freedom.
    void Lock(bool lock);
};

/// Parallel joint, with the 'ChLinkLock' formulation.
/// (allows a simpler creation of a link as a sub-type of ChLinkLock).
class ChApi ChLinkLockParallel : public ChLinkLock {
  public:
    ChLinkLockParallel() { ChangeType(Type::PARALLEL); }

    /// "Virtual" copy constructor (covariant return type).
    virtual ChLinkLockParallel* Clone() const override { return new ChLinkLockParallel(*this); }

    /// Lock the joint.
    /// If enabled (lock = true) this effectively converts this joint into a weld joint.
    /// If lock = false, the joint reverts to its original degrees of freedom.
    void Lock(bool lock);
};

/// Perpendicularity joint, with the 'ChLinkLock' formulation.
/// (allows a simpler creation of a link as a sub-type of ChLinkLock).
class ChApi ChLinkLockPerpend : public ChLinkLock {
  public:
    ChLinkLockPerpend() { ChangeType(Type::PERPEND); }

    /// "Virtual" copy constructor (covariant return type).
    virtual ChLinkLockPerpend* Clone() const override { return new ChLinkLockPerpend(*this); }

    /// Lock the joint.
    /// If enabled (lock = true) this effectively converts this joint into a weld joint.
    /// If lock = false, the joint reverts to its original degrees of freedom.
    void Lock(bool lock);
};

/// RevolutePrismatic joint, with the 'ChLinkLock' formulation.
/// Translates along x-dir, rotates about z-axis
class ChApi ChLinkLockRevolutePrismatic : public ChLinkLock {
  public:
    ChLinkLockRevolutePrismatic() { ChangeType(Type::REVOLUTEPRISMATIC); }

    /// "Virtual" copy constructor (covariant return type).
    virtual ChLinkLockRevolutePrismatic* Clone() const override { return new ChLinkLockRevolutePrismatic(*this); }

    /// Lock the joint.
    /// If enabled (lock = true) this effectively converts this joint into a weld joint.
    /// If lock = false, the joint reverts to its original degrees of freedom.
    void Lock(bool lock);
};

}  // end namespace chrono

#endif
