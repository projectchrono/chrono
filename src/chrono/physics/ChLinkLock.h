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

#ifndef CHLINKLOCK_H
#define CHLINKLOCK_H

#include "chrono/physics/ChLimit.h"
#include "chrono/physics/ChLinkMasked.h"

namespace chrono {

/// ChLinkLock class.
/// This class implements lot of sub types like the revolute
/// joint, the linear guide, the spherical joint, etc. using
/// the 'lock formulation'.
/// Also, it optionally allows the adoption of 'limits' over
/// upper-lower motions on all the 6 degrees of freedom,
/// thank to the ChLinkLimit objects.

class ChApi ChLinkLock : public ChLinkMasked {
    // Tag needed for class factory in archive (de)serialization:
    CH_FACTORY_TAG(ChLinkLock)

  protected:
    Coordsys relC;       ///< relative costraint position: relC = (relM-deltaC)
    Coordsys relC_dt;    ///< relative costraint speed
    Coordsys relC_dtdt;  ///< relative costraint acceleration

    Coordsys deltaC;       ///< user-imposed rel. position
    Coordsys deltaC_dt;    ///< user-imposed rel. speed
    Coordsys deltaC_dtdt;  ///< user-imposed rel. acceleration

    //(only for intermediate calculus)
    ChMatrix<>* Cq1_temp;  ///<
    ChMatrix<>* Cq2_temp;  ///<   the temporary "lock" jacobians,
    ChMatrix<>* Qc_temp;   ///<   i.e. the full x,y,z,r0,r1,r2,r3 joint
    Coordsys Ct_temp;      ///<

    Vector PQw;  ///< for intermediate calculus (here, for speed reasons)
    Vector PQw_dt;
    Vector PQw_dtdt;
    Quaternion q_AD;
    Quaternion q_BC;
    Quaternion q_8;
    Vector q_4;

    // imposed motion
    std::shared_ptr<ChFunction> motion_X;     ///< user imposed motion for X coord, marker relative
    std::shared_ptr<ChFunction> motion_Y;     ///< user imposed motion for Y coord, marker relative
    std::shared_ptr<ChFunction> motion_Z;     ///< user imposed motion for Z coord, marker relative
    std::shared_ptr<ChFunction> motion_ang;   ///< user imposed angle rotation about axis
    std::shared_ptr<ChFunction> motion_ang2;  ///< user imposed angle rotation if three-angles rot.
    std::shared_ptr<ChFunction> motion_ang3;  ///< user imposed angle rotation if three-angles rot.
    Vector motion_axis;       ///< this is the axis for the user imposed rotation
    AngleSet angleset;             ///< type of rotation (3 Eul angles, angle/axis, etc.)

    // limits
    ChLinkLimit* limit_X;   ///< the upper/lower limits for X dof
    ChLinkLimit* limit_Y;   ///< the upper/lower limits for Y dof
    ChLinkLimit* limit_Z;   ///< the upper/lower limits for Z dof
    ChLinkLimit* limit_Rx;  ///< the upper/lower limits for Rx dof
    ChLinkLimit* limit_Ry;  ///< the upper/lower limits for Ry dof
    ChLinkLimit* limit_Rz;  ///< the upper/lower limits for Rz dof
    ChLinkLimit* limit_Rp;  ///< the polar (conical) limit for "shoulder"rotation
    ChLinkLimit* limit_D;   ///< the polar (conical) limit for "shoulder"rotation

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

  public:
    ChLinkLock();
    ChLinkLock(const ChLinkLock& other);
    virtual ~ChLinkLock();

    /// "Virtual" copy constructor (covariant return type).
    virtual ChLinkLock* Clone() const override { return new ChLinkLock(*this); }

    //
    // UPDATING FUNCTIONS
    //

    // Inherits, and also updates motion laws: deltaC, deltaC_dt, deltaC_dtdt
    virtual void UpdateTime(double mytime) override;

    // Updates coords relM, relM_dt, relM_dtdt;
    // dist, dist_dt et similia, just like in parent class, but
    // overrides parent implementation of ChLinkMarkers because it can save some
    // temporary vectors (q_4, q_8 etc.) which can be useful in UpdateState(),
    // for speed reasons.
    virtual void UpdateRelMarkerCoords() override;

    // Given current time and body state, computes
    // the constraint differentiation to get the
    // the state matrices     Cq1,  Cq2,  Qc,  Ct , and also
    // C, C_dt, C_dtd.   ie. the JACOBIAN matrices and friends.
    //  NOTE!! this function uses the fast analytical approach
    // of the "lock formulation".
    virtual void UpdateState() override;

    // Inherits, and also updates the local F,M forces adding penalties from
    // the contained link ChLinkLimit objects, if any.
    virtual void UpdateForces(double mytime) override;

    //
    // OTHER FUNCTIONS
    //

    // constraint violations in pos/rot coordinates
    Coordsys GetRelC() { return relC; }
    Coordsys GetRelC_dt() { return relC_dt; }
    Coordsys GetRelC_dtdt() { return relC_dtdt; }

    // to get the imposed clearances
    Coordsys GetDeltaC() { return deltaC; }
    Coordsys GetDeltaC_dt() { return deltaC_dt; }
    Coordsys GetDeltaC_dtdt() { return deltaC_dtdt; }
    // to set the imposed clearances (best use SetMotion() if you can..)
    void SetDeltaC(Coordsys mc) { deltaC = mc; }
    void SetDeltaC_dt(Coordsys mc) { deltaC_dt = mc; }
    void SetDeltaC_dtdt(Coordsys mc) { deltaC_dtdt = mc; }

    // for the imposed motion functions
    std::shared_ptr<ChFunction> GetMotion_X() { return motion_X; };
    std::shared_ptr<ChFunction> GetMotion_Y() { return motion_Y; };
    std::shared_ptr<ChFunction> GetMotion_Z() { return motion_Z; };
    std::shared_ptr<ChFunction> GetMotion_ang() { return motion_ang; };
    std::shared_ptr<ChFunction> GetMotion_ang2() { return motion_ang2; };
    std::shared_ptr<ChFunction> GetMotion_ang3() { return motion_ang3; };
    Vector GetMotion_axis() { return motion_axis; };
    void SetMotion_X(std::shared_ptr<ChFunction> m_funct);
    void SetMotion_Y(std::shared_ptr<ChFunction> m_funct);
    void SetMotion_Z(std::shared_ptr<ChFunction> m_funct);
    void SetMotion_ang(std::shared_ptr<ChFunction> m_funct);
    void SetMotion_ang2(std::shared_ptr<ChFunction> m_funct);
    void SetMotion_ang3(std::shared_ptr<ChFunction> m_funct);
    void SetMotion_axis(Vector m_axis);
    AngleSet Get_angleset() { return angleset; };
    void Set_angleset(AngleSet mset) { angleset = mset; }

    // for the limits on free degrees
    ChLinkLimit* GetLimit_X() { return limit_X; }
    ChLinkLimit* GetLimit_Y() { return limit_Y; }
    ChLinkLimit* GetLimit_Z() { return limit_Z; }
    ChLinkLimit* GetLimit_Rx() { return limit_Rx; }
    ChLinkLimit* GetLimit_Ry() { return limit_Ry; }
    ChLinkLimit* GetLimit_Rz() { return limit_Rz; }
    ChLinkLimit* GetLimit_Rp() { return limit_Rp; }
    ChLinkLimit* GetLimit_D() { return limit_D; }
    void SetLimit_X(ChLinkLimit* m_limit_X) {
        if (limit_X)
            delete limit_X;
        limit_X = m_limit_X;
    }
    void SetLimit_Y(ChLinkLimit* m_limit_Y) {
        if (limit_Y)
            delete limit_Y;
        limit_Y = m_limit_Y;
    }
    void SetLimit_Z(ChLinkLimit* m_limit_Z) {
        if (limit_Z)
            delete limit_Z;
        limit_Z = m_limit_Z;
    }
    void SetLimit_Rx(ChLinkLimit* m_limit_Rx) {
        if (limit_Rx)
            delete limit_Rx;
        limit_Rx = m_limit_Rx;
    }
    void SetLimit_Ry(ChLinkLimit* m_limit_Ry) {
        if (limit_Ry)
            delete limit_Ry;
        limit_Ry = m_limit_Ry;
    }
    void SetLimit_Rz(ChLinkLimit* m_limit_Rz) {
        if (limit_Rz)
            delete limit_Rz;
        limit_Rz = m_limit_Rz;
    }
    void SetLimit_Rp(ChLinkLimit* m_limit_Rp) {
        if (limit_Rp)
            delete limit_Rp;
        limit_Rp = m_limit_Rp;
    }
    void SetLimit_D(ChLinkLimit* m_limit_D) {
        if (limit_D)
            delete limit_D;
        limit_D = m_limit_D;
    }

    //
    // STATE FUNCTIONS
    //

    /// Get the number of scalar constraints, if any, in this item
    virtual int GetDOC() override { return GetDOC_c() + GetDOC_d(); }
    /// Get the number of scalar constraints, if any, in this item (only bilateral constr.)
    // virtual int GetDOC_c  () {return 0;} // use parent ChLinkMasked ndof
    /// Get the number of scalar constraints, if any, in this item (only unilateral constr.)
    virtual int GetDOC_d() override;  // customized because there might be some active ChLinkLimit

    /// Specialize the following respect to ChLinkMasked base ,in order to update intuitive react_torque and react_force
    virtual void IntStateScatterReactions(const unsigned int off_L, const ChVectorDynamic<>& L) override;

    /// Specialize the following respect to ChLinkMasked base because there might be some active ChLinkLimit
    // virtual void IntLoadResidual_F(const unsigned int off,	ChVectorDynamic<>& R, const double c );

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

    //
    // SOLVER SYSTEM FUNCTIONS
    //

    // expand parent constraint stuff from ChLinkMasked because here
    // it may also consider the	constraints caused by 'limits'..
    virtual void InjectConstraints(ChSystemDescriptor& mdescriptor) override;
    virtual void ConstraintsBiReset() override;
    virtual void ConstraintsBiLoad_C(double factor = 1, double recovery_clamp = 0.1, bool do_clamp = false) override;
    virtual void ConstraintsBiLoad_Ct(double factor = 1) override;
    virtual void ConstraintsBiLoad_Qc(double factor = 1) override;
    virtual void ConstraintsLoadJacobians() override;
    virtual void ConstraintsFetch_react(double factor = 1) override;

    //
    // SERIALIZATION
    //

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOUT(ChArchiveOut& marchive) override;

    /// Method to allow deserialization of transient data from archives.
    virtual void ArchiveIN(ChArchiveIn& marchive) override;

  protected:
    void ChangeLinkType(LinkType new_link_type);

  private:
    void BuildLinkType(LinkType link_type);
};

CH_CLASS_VERSION(ChLinkLock,0)



// ---------------------------------------------------------------------------------------
// SOME WRAPPER CLASSES, TO MAKE 'LINK LOCK' CREATION EASIER...
// ---------------------------------------------------------------------------------------

/// Revolute joint , with the 'ChLinkLock' formulation.
/// (allows a simplier creation of a link as a sub-type of ChLinkLock).

class ChApi ChLinkLockRevolute : public ChLinkLock {
    // Tag needed for class factory in archive (de)serialization:
    CH_FACTORY_TAG(ChLinkLockRevolute)

  public:
    ChLinkLockRevolute() { ChangeLinkType(LinkType::REVOLUTE); }

    /// "Virtual" copy constructor (covariant return type).
    virtual ChLinkLockRevolute* Clone() const override { return new ChLinkLockRevolute(*this); }
};

/// 6-dof locked joint , with the 'ChLinkLock' formulation.
/// (allows a simplier creation of a link as a sub-type of ChLinkLock).

class ChApi ChLinkLockLock : public ChLinkLock {
    // Tag needed for class factory in archive (de)serialization:
    CH_FACTORY_TAG(ChLinkLockLock)

  public:
    ChLinkLockLock() { ChangeLinkType(LinkType::LOCK); }

    /// "Virtual" copy constructor (covariant return type).
    virtual ChLinkLockLock* Clone() const override { return new ChLinkLockLock(*this); }
};

/// Spherical joint , with the 'ChLinkLock' formulation.
/// (allows a simplier creation of a link as a sub-type of ChLinkLock).

class ChApi ChLinkLockSpherical : public ChLinkLock {
    // Tag needed for class factory in archive (de)serialization:
    CH_FACTORY_TAG(ChLinkLockSpherical)

  public:
    ChLinkLockSpherical() { ChangeLinkType(LinkType::SPHERICAL); }

    /// "Virtual" copy constructor (covariant return type).
    virtual ChLinkLockSpherical* Clone() const override { return new ChLinkLockSpherical(*this); }
};

/// Cylindrical joint , with the 'ChLinkLock' formulation.
/// (allows a simplier creation of a link as a sub-type of ChLinkLock).

class ChApi ChLinkLockCylindrical : public ChLinkLock {
    // Tag needed for class factory in archive (de)serialization:
    CH_FACTORY_TAG(ChLinkLockCylindrical)

  public:
    ChLinkLockCylindrical() { ChangeLinkType(LinkType::CYLINDRICAL); }

    /// "Virtual" copy constructor (covariant return type).
    virtual ChLinkLockCylindrical* Clone() const override { return new ChLinkLockCylindrical(*this); }
};

/// Prismatic joint , with the 'ChLinkLock' formulation.
/// Default axis along +z

class ChApi ChLinkLockPrismatic : public ChLinkLock {
    // Tag needed for class factory in archive (de)serialization:
    CH_FACTORY_TAG(ChLinkLockPrismatic)

  public:
    ChLinkLockPrismatic() { ChangeLinkType(LinkType::PRISMATIC); }

    /// "Virtual" copy constructor (covariant return type).
    virtual ChLinkLockPrismatic* Clone() const override { return new ChLinkLockPrismatic(*this); }
};

/// Point-plane joint , with the 'ChLinkLock' formulation.
/// (allows a simplier creation of a link as a sub-type of ChLinkLock).

class ChApi ChLinkLockPointPlane : public ChLinkLock {
    // Tag needed for class factory in archive (de)serialization:
    CH_FACTORY_TAG(ChLinkLockPointPlane)

  public:
    ChLinkLockPointPlane() { ChangeLinkType(LinkType::POINTPLANE); }

    /// "Virtual" copy constructor (covariant return type).
    virtual ChLinkLockPointPlane* Clone() const override { return new ChLinkLockPointPlane(*this); }
};

/// Point-line joint , with the 'ChLinkLock' formulation.
/// (allows a simplier creation of a link as a sub-type of ChLinkLock).

class ChApi ChLinkLockPointLine : public ChLinkLock {
    // Tag needed for class factory in archive (de)serialization:
    CH_FACTORY_TAG(ChLinkLockPointLine)

  public:
    ChLinkLockPointLine() { ChangeLinkType(LinkType::POINTLINE); }

    /// "Virtual" copy constructor (covariant return type).
    virtual ChLinkLockPointLine* Clone() const override { return new ChLinkLockPointLine(*this); }
};

/// Plane-plane joint , with the 'ChLinkLock' formulation.
/// (allows a simplier creation of a link as a sub-type of ChLinkLock).

class ChApi ChLinkLockPlanePlane : public ChLinkLock {
    // Tag needed for class factory in archive (de)serialization:
    CH_FACTORY_TAG(ChLinkLockPlanePlane)

  public:
    ChLinkLockPlanePlane() { ChangeLinkType(LinkType::PLANEPLANE); }

    /// "Virtual" copy constructor (covariant return type).
    virtual ChLinkLockPlanePlane* Clone() const override { return new ChLinkLockPlanePlane(*this); }
};

/// Oldham joint , with the 'ChLinkLock' formulation.
/// (allows a simplier creation of a link as a sub-type of ChLinkLock).

class ChApi ChLinkLockOldham : public ChLinkLock {
    // Tag needed for class factory in archive (de)serialization:
    CH_FACTORY_TAG(ChLinkLockOldham)

  public:
    ChLinkLockOldham() { ChangeLinkType(LinkType::OLDHAM); }

    /// "Virtual" copy constructor (covariant return type).
    virtual ChLinkLockOldham* Clone() const override { return new ChLinkLockOldham(*this); }
};

/// Free joint , with the 'ChLinkLock' formulation.
/// (allows a simplier creation of a link as a sub-type of ChLinkLock).

class ChApi ChLinkLockFree : public ChLinkLock {
    // Tag needed for class factory in archive (de)serialization:
    CH_FACTORY_TAG(ChLinkLockFree)

  public:
    ChLinkLockFree() { ChangeLinkType(LinkType::FREE); }

    /// "Virtual" copy constructor (covariant return type).
    virtual ChLinkLockFree* Clone() const override { return new ChLinkLockFree(*this); }
};

/// Align joint , with the 'ChLinkLock' formulation.
/// (allows a simplier creation of a link as a sub-type of ChLinkLock).

class ChApi ChLinkLockAlign : public ChLinkLock {
    // Tag needed for class factory in archive (de)serialization:
    CH_FACTORY_TAG(ChLinkLockAlign)

  public:
    ChLinkLockAlign() { ChangeLinkType(LinkType::ALIGN); }

    /// "Virtual" copy constructor (covariant return type).
    virtual ChLinkLockAlign* Clone() const override { return new ChLinkLockAlign(*this); }
};

/// Parallel joint , with the 'ChLinkLock' formulation.
/// (allows a simplier creation of a link as a sub-type of ChLinkLock).

class ChApi ChLinkLockParallel : public ChLinkLock {
    // Tag needed for class factory in archive (de)serialization:
    CH_FACTORY_TAG(ChLinkLockParallel)

  public:
    ChLinkLockParallel() { ChangeLinkType(LinkType::PARALLEL); }

    /// "Virtual" copy constructor (covariant return type).
    virtual ChLinkLockParallel* Clone() const override { return new ChLinkLockParallel(*this); }
};

/// Perpendicularity joint , with the 'ChLinkLock' formulation.
/// (allows a simplier creation of a link as a sub-type of ChLinkLock).

class ChApi ChLinkLockPerpend : public ChLinkLock {
    // Tag needed for class factory in archive (de)serialization:
    CH_FACTORY_TAG(ChLinkLockPerpend)

  public:
    ChLinkLockPerpend() { ChangeLinkType(LinkType::PERPEND); }

    /// "Virtual" copy constructor (covariant return type).
    virtual ChLinkLockPerpend* Clone() const override { return new ChLinkLockPerpend(*this); }
};

/// RevolutePrismatic joint , with the 'ChLinkLock' formulation.
/// Translates along x-dir, rotates about z-axis
class ChApi ChLinkLockRevolutePrismatic : public ChLinkLock {
    // Tag needed for class factory in archive (de)serialization:
    CH_FACTORY_TAG(ChLinkLockRevolutePrismatic)

  public:
    ChLinkLockRevolutePrismatic() { ChangeLinkType(LinkType::REVOLUTEPRISMATIC); }

    /// "Virtual" copy constructor (covariant return type).
    virtual ChLinkLockRevolutePrismatic* Clone() const override { return new ChLinkLockRevolutePrismatic(*this); }
};

}  // end namespace chrono

#endif
