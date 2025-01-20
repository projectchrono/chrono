// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2024 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban
// =============================================================================
//
// This file contains the definition of the ChMobilizedBody class and a derived
// class, ChGroundBody. The still abstract ChMobilizedBody, derived from ChBody,
// implements an articulated rigid body which is part of a multibody mechanical
// system. This object encapsulates all quantities required for recursive
// operations on a multibody tree that are not dependent on the actual number of
// degrees of freedom.
// Classes derived from ChMobilizedBody include:
//	- ChMobilizedBodyT, a class templatized by the number of DOFs (i.e. the
//    number of generalized velocities), used as base class for the various
//    concrete mobilized body classes with number of DOFs from 1 to 6.
//	- ChWeldBody, the special case of an articulated body with zero DFOs.
//	- ChGroundBody, the special body used as root of a multibody system. An
//		mbGroundBody exists simply as an initiation point for the recursive
//		traversals of the multibody tree and as a holder of the base bodies
//		(those bodies that have ground as their parent).
//
// =============================================================================

#ifndef CH_MOBILIZED_BODY_H
#define CH_MOBILIZED_BODY_H

#include "chrono/core/ChFrame.h"

#include "chrono/soa/ChSpatial.h"
#include "chrono/soa/ChMassProps.h"
#include "chrono/soa/ChMobilityForce.h"

namespace chrono {
namespace soa {

class ChSoaAssembly;

/// @addtogroup chrono_soa
/// @{

/// Base class for mobilized bodies.
/// This abstract class implements an articulated rigid body which is part of a multibody mechanical SOA assembly. This
/// object encapsulates all quantities required for recursive operations on a multibody tree that are not dependent on
/// the actual number of degrees of freedom.
class ChApi ChMobilizedBody {
  public:
    virtual ~ChMobilizedBody();

    virtual int getNumQ() const = 0;
    virtual int getNumU() const = 0;

    std::shared_ptr<ChMobilizedBody> getParent() const { return m_parent; }

    int getNumChildren() const { return (int)m_children.size(); }
    ChMobilizedBody* getChild(int i) const { return m_children[i]; }

    virtual bool isGround() const { return false; }
    bool isTerminal() const { return m_children.empty(); }

    void lock(bool val);

    const std::string& getName() const { return m_name; }

    const ChFramed& getRelPos() const { return m_X_FM; }
    const ChVector3d& getRelLoc() const { return m_X_FM.GetPos(); }
    const ChQuaterniond& getRelQuat() const { return m_X_FM.GetRot(); }
    const ChMatrix33d& getRelRot() const { return m_X_FM.GetRotMat(); }
    const ChSpatialVec& getRelVel() const { return m_V_FM; }
    const ChVector3d& getRelLinVel() const { return m_V_FM.lin(); }
    const ChVector3d& getRelAngVel() const { return m_V_FM.ang(); }

    const ChFramed& getInboardFrame() const { return m_X_PF; }
    const ChFramed& getOutboardFrame() const { return m_X_BM; }

    virtual void setInboardFrame(const ChFramed& X_PF) { m_X_PF = X_PF; }
    virtual void setOutboardFrame(const ChFramed& X_BM) { m_X_BM = X_BM; }

    const ChFramed& getAbsPos() const { return m_absPos; }
    const ChVector3d& getAbsLoc() const { return m_absPos.GetPos(); }
    const ChQuaterniond& getAbsQuat() const { return m_absPos.GetRot(); }
    const ChMatrix33d& getAbsRot() const { return m_absPos.GetRotMat(); }
    const ChSpatialVec& getAbsVel() const { return m_absVel; }
    const ChVector3d& getAbsLinVel() const { return m_absVel.lin(); }
    const ChVector3d& getAbsAngVel() const { return m_absVel.ang(); }
    const ChSpatialVec& getAbsAcc() const { return m_absAcc; }
    const ChVector3d& getAbsLinAcc() const { return m_absAcc.lin(); }
    const ChVector3d& getAbsAngAcc() const { return m_absAcc.ang(); }

    ChVector3d getAbsLoc(const ChVector3d& p_B) const;
    ChVector3d getAbsVel(const ChVector3d& p_B) const;
    ChVector3d getAbsAcc(const ChVector3d& p_B) const;

    ChVector3d getAbsCOMLoc() const;
    ChVector3d getAbsCOMVel() const;
    ChVector3d getAbsCOMAcc() const;

    // These functions set the relative positions, velocities, and accelerations for this mobilized body, respectively.
    // They are the default implementations of these virtual methods: they use the rotational and translational quantity
    // routines and assume that the rotational and translational coordinates are independent, with rotation handled
    // first and then left alone. If a mobilizer type needs to deal with rotation and translation simultaneously, it
    // should provide a specific implementation for these two routines.
    //
    // Notes:
    //	- accelerations can meaningfully be set only while performing an Inverse Dynamics Analysis.
    //	- a concrete mobilizer must implement setRelRot() and setRelLoc(), setRelAngVel() and setRelLinVel(),
    //    setRelAngAcc() and setRelLinAcc(); these functions do not have default implementations.
    //	- these functions write directly into the SOA assembly's state vectors.

    /// Set the relative position of the mobilized body.
    /// This default implementation uses the rotational and translational quantity setter functions and assume that the
    /// rotational and translational coordinates are independent, with rotation handled first and then left alone. If a
    /// concrete mobilizer needs to deal with rotation and translation simultaneously, it should provide a specific
    /// implementation for this function.
    virtual void setRelPos(const ChFramed& relPos);

    /// Set the relative velocity of the mobilized body.
    /// This default implementation uses the rotational and translational quantity setter functions and assume that the
    /// rotational and translational coordinates are independent, with rotation handled first and then left alone. If a
    /// concrete mobilizer needs to deal with rotation and translation simultaneously, it should provide a specific
    /// implementation for this function.
    virtual void setRelVel(const ChSpatialVec& relVel);

    /// Set the relative acceleration of the mobilized body.
    /// This default implementation uses the rotational and translational quantity setter functions and assume that the
    /// rotational and translational coordinates are independent, with rotation handled first and then left alone. If a
    /// concrete mobilizer needs to deal with rotation and translation simultaneously, it should provide a specific
    /// implementation for this function.
    virtual void setRelAcc(const ChSpatialVec& relAcc);

    // Mobilizer-specific functions to set relative rotation and translation, relative angular and linear
    // velocity, and relative angular and linear acceleration.
    // If the body is associated with an SOA assembly that was already initialized, these functions should write in the
    // components of assembly-wide state vectors corresponding to this body. Otherwise, these should be treated as
    // initial conditions and cached (so that they can be returned through getQ0() and getU0()).

    virtual void setRelRot(const ChMatrix33d& relRot) = 0;
    virtual void setRelLoc(const ChVector3d& relLoc) = 0;
    virtual void setRelAngVel(const ChVector3d& relAngVel) = 0;
    virtual void setRelLinVel(const ChVector3d& relLinVel) = 0;
    virtual void setRelAngAcc(const ChVector3d& relAngAcc) = 0;
    virtual void setRelLinAcc(const ChVector3d& relLinAcc) = 0;

    void AddMobilityForce(int dof, std::shared_ptr<ChMobilityForce> force);

    void applyBodyForce(const ChSpatialVec& force) { m_bodyForce += force; }
    const ChSpatialVec& getBodyForce() const { return m_bodyForce; }

  protected:
    struct MobilityForce {
        int dof;
        std::shared_ptr<ChMobilityForce> force;
    };

    ChMobilizedBody(std::shared_ptr<ChMobilizedBody> parent,
                    const ChMassProps& mpropsB,
                    const ChFramed& X_PF,
                    const ChFramed& X_BM,
                    const std::string& name = "");

    void ApplyMobilityForces();

    double getQ(int dof) const;
    double getU(int dof) const;
    double getUdot(int dof) const;

    void setQ(int dof, double val) const;
    void setU(int dof, double val) const;
    void setUdot(int dof, double val) const;

    // Outward recursive functions
    // ---------------------------
    // These are the default implementations of these virtual functions which, in
    // the case of ChMobilizedBodies, are recursive **base-to-tip** tree traversals of
    // the multibody tree. They simply implement the basic mechanism for deferring
    // to the children. A concrete mobilizer is supposed to implement its own
    // versions of these methods which first perform the appropriate operations for
    // that mobilizer, after which they call the appropriate generic method (one of
    // these) in order to allow its children to perform the same calculations.
    // The only exception is ChGroundBody for which these default methods are
    // sufficient (as Ground has no states).

    /// This recursive function, called in an outward, base-to-tip traversal of the multibody tree, calculates the
    /// absolute position and all position-dependent kinematic quantities. It assumes that the same function has already
    /// been called for the parent body.
    /// A concrete mobilizer is supposed to implement its own version of this method which first performs the
    /// appropriate operations for that mobilizer, *after* which it muct call this generic method in order to allow its
    /// children to perform the same calculations.
    virtual void orProcPosFD(const ChVectorDynamic<>& y);

    /// This recursive function, called in an outward, base-to-tip traversal of the multibody tree, calculates the
    /// absolute body velocity. It assumes that a tree traversal to calculate position-level quantities has been
    /// performed and that this same function has already been called for the parent body.
    /// A concrete mobilizer is supposed to implement its own version of this method which first performs the
    /// appropriate operations for that mobilizer, *after* which it muct call this generic method in order to allow its
    /// children to perform the same calculations.
    virtual void orProcVelFD(const ChVectorDynamic<>& yd);

    /// This recursive function, part of the "Forward Dynamics Analysis" chain, is  called in an outward, base-to-tip
    /// traversal of the multibody tree, to  calculate the absolute body position and velocity, as well as any position-
    /// and velocity-related quantities required for further dynamic analysis. It  assumes that the same function has
    /// already been called for the parent body.
    /// A concrete mobilizer is supposed to implement its own version of this method which first performs the
    /// appropriate operations for that mobilizer, *after* which it muct call this generic method in order to allow its
    /// children to perform the same calculations.
    virtual void orProcPosAndVelFD(const ChVectorDynamic<>& y, const ChVectorDynamic<>& yd);

    /// This recursive function, part of the "Forward Dynamics Analysis" chain, is called in an outward, base-to-tip
    /// traversal of the multibody tree, to calculate the absolute body acceleration and set the derivative of the state
    /// vector.
    /// A concrete mobilizer is supposed to implement its own version of this method which first performs the
    /// appropriate operations for that mobilizer, *after* which it muct call this generic method in order to allow its
    /// children to perform the same calculations.
    virtual void orProcAccFD(const ChVectorDynamic<>& y, const ChVectorDynamic<>& yd, ChVectorDynamic<>& ydd);

    virtual void orProcMiF_passTwo(double* ud);

    /// This recursive function, part of the "Inverse Dynamics Analysis" chain, is called in an outward, base-to-tip
    /// traversal of the multibody tree, to calculate the absolute body position, velocity, and acceleration. It assumes
    /// that the same function has already been called for the parent body.
    /// A concrete mobilizer is supposed to implement its own version of this method which first performs the
    /// appropriate operations for that mobilizer, *after* which it muct call this generic method in order to allow its
    /// children to perform the same calculations.
    virtual void orProcPosVelAccID(const ChVectorDynamic<>& y,
                                   const ChVectorDynamic<>& yd,
                                   const ChVectorDynamic<>& ydd);

    // Inward recursive functions
    // --------------------------
    // These are the default implementations of these virtual functions which, in
    // the case of ChMobilizedBodies, are recursive **tip-to-base** tree traversals of
    // the multibody tree. They simply implement the basic mechanism for deferring
    // to the children. A concrete mobilizer is supposed to implement its own
    // versions of these methods which first call the appropriate generic method
    // (one of these) in order to allow its children to perform the same
    // calculations after which they perform the appropriate operations for that
    // mobilizer.
    // The only exception is ChGroundBody for which these default methods are
    // sufficient (as Ground has no states).

    /// This recursive function, part of the "Forward Dynamics Analysis" chain, is called in an inward, tip-to-base
    /// traversal of the multibody tree, to calculate the body articulated inertia and the generalized force due to
    /// external forces. It is called only if the multibody system has no active constraints and, as such, all CS forces
    /// are set to zero.
    /// A concrete mobilizer is supposed to implement its own version of this method which *first* calls this generic
    /// method in order to allow its children to perform the same calculations after which it performs the appropriate
    /// operations for that mobilizer.
    virtual void irProcInertiasAndForcesFD(const ChVectorDynamic<>& y, const ChVectorDynamic<>& yd);

    /// This recursive function, part of the "Forward Dynamics Analysis" chain, is called in an inward, tip-to-base
    /// traversal of the multibody tree, to calculate the body articulated inertia. It is called only if the multibody
    /// system has active constraints.
    /// A concrete mobilizer is supposed to implement its own version of this method which *first* calls this generic
    /// method in order to allow its children to perform the same calculations after which it performs the appropriate
    /// operations for that mobilizer.
    virtual void irProcInertiasFD(const ChVectorDynamic<>& y, const ChVectorDynamic<>& yd);

    /// This recursive function, part of the "Forward Dynamics Analysis" chain, is called in an inward, tip-to-base
    /// traversal of the multibody tree, to calculate the generalized body force due to external and possibly constraint
    /// forces. It is called only if the multibody system has active constraints, once for evaluating the open-loop
    /// dynamics (in which case all CS forces are set to zero) and a second time for the closed-loop dynamics (in which
    /// case the CS forces are non-zero).
    /// A concrete mobilizer is supposed to implement its own version of this method which *first* calls this generic
    /// method in order to allow its children to perform the same calculations after which it performs the appropriate
    /// operations for that mobilizer.
    virtual void irProcForcesFD(const ChVectorDynamic<>& y, const ChVectorDynamic<>& yd);

    /// This recursive function, part of the "Forward Dynamics Analysis" chain, is called in an inward, tip-to-base
    /// traversal of the multibody tree, to calculate a column of the constraint Jacobian. Inertial effects are *not*
    /// included in this calculation. This function is called only if the multibody system has active constraints.
    /// A concrete mobilizer is supposed to implement its own version of this method which *first* calls this generic
    /// method in order to allow its children to perform the same calculations after which it performs the appropriate
    /// operations for that mobilizer.
    virtual void irProcConstraintJac(double* vec);

    virtual void irProcMiF_passOne();

    /// This recursive function, part of the "Inverse Dynamics Analysis" chain, is called in an inward, tip-to-base
    /// traversal of the multibody tree, to calculate the body and mobility forces at the given body configuration (i.e.
    /// the current body position, velocity, and acceleration).
    /// A concrete mobilizer is supposed to implement its own version of this method which *first* calls this generic
    /// method in order to allow its children to perform the same calculations after which it performs the appropriate
    /// operations for that mobilizer.
    virtual void irProcForcesID(const ChVectorDynamic<>& y, const ChVectorDynamic<>& yd, const ChVectorDynamic<>& ydd);

    /// Allow the body to perform any operations at the beginning of a simulation step.
    virtual void prepSim() {}

    virtual void applyMobilityForce(int which, double force) {}

    virtual void applyCSMobilityForce(int which, double force) {}
    virtual void resetForcesCS() {}

    /// Set the first derivative of the generalized coordinates in the assembly-wide state vector.
    /// In other words, calculate and set:
    /// <pre>
    ///     q' = N(q) * u
    /// </pre>
    ///
    /// Each body is responsible to access an set its own entries in the provided system-level state and state
    /// derivative vectors.
    ///
    /// This is the default implementations of this virtual method. It can be used by any body for which the generalized
    /// speeds are the derivatives of the generalized coordinates. A body for which this is not true (e.g., those using
    /// quaternions) must overwrite this default.
    virtual void setQDot(const ChVectorDynamic<>& y, ChVectorDynamic<>& yd) const;

    /// Set the second derivative of the generalized coordinates in the assembly-wide state vector.
    /// In other words, calculate and set:
    /// <pre>
    ///     q'' = N(q) * u' + N_q(q) * q' * u = N(q) * u' + N_q(q) * N(q) * u * u
    /// </pre>
    ///
    /// Each body is responsible to access an set its own entries in the provided system-level state and state
    /// derivative vectors.
    ///
    /// This is the default implementations of this virtual method. It can be used by any body for which the generalized
    /// speeds are the derivatives of the generalized coordinates. A body for which this is not true (e.g. those using
    /// quaternions) must overwrite this default.
    virtual void setQDotDot(const ChVectorDynamic<>& y, const ChVectorDynamic<>& yd, ChVectorDynamic<>& ydd) const;

    // Functions to get initial conditions
    // -----------------------------------

    virtual double getQ0(int dof) const = 0;
    virtual double getU0(int dof) const = 0;

    // Body properties

    std::string m_name;

    ChMassProps m_mpropsB;
    ChMatrix33d m_inertiaOB_G;
    ChVector3d m_CB_G;
    ChVector3d m_com_G;

    ChFramed m_absPos;
    ChSpatialVec m_absVel;
    ChSpatialVec m_absAcc;

    ChSpatialVec m_cntrfForce;
    ChSpatialVec m_bodyForce;

    bool m_locked;

    // Indices in assembly-wide vectors

    int m_qIdx;
    int m_uIdx;

    ChSoaAssembly* m_assembly;

    // SOA quantities

    ChFramed m_X_PF;
    ChFramed m_X_BM;

    ChFramed m_X_FM;
    ChFramed m_X_PB;

    ChShiftMat m_Phi;

    ChSpatialMat m_Mk;
    ChSpatialMat m_Pplus;

    ChSpatialVec m_V_FM;

    ChSpatialVec m_corAcc;
    ChSpatialVec m_corAccT;

    ChSpatialVec m_gyrForce;

    ChSpatialVec m_Zplus;

    ChSpatialVec m_bodyForceCS;

    std::shared_ptr<ChMobilizedBody> m_parent;
    std::vector<ChMobilizedBody*> m_children;

    std::vector<MobilityForce> m_mobility_forces;

    static const double m_angleClampLimit;

    friend class ChSoaAssembly;
    friend class ChMobilityForce;
    template <int>
    friend class ChMobilizedBodyT;
    friend class mbWeldBody;
    friend class mbConstraint;
};

// -----------------------------------------------------------------------------

/// Special body used as root of a multibody system.
/// A ChGroundBody exists simply as an initiation point for the recursive traversals of the multibody tree and as a
/// holder of the base bodies (those bodies that have ground as their parent).
class ChApi ChGroundBody : public ChMobilizedBody {
  public:
    ChGroundBody();
    ~ChGroundBody() {}

    virtual bool isGround() const override { return true; }

  private:
    virtual int getNumQ() const override { return 0; }
    virtual int getNumU() const override { return 0; }

    virtual void setRelRot(const ChMatrix33d& relRot) override {}
    virtual void setRelLoc(const ChVector3d& relLoc) override {}
    virtual void setRelAngVel(const ChVector3d& relAngVel) override {}
    virtual void setRelLinVel(const ChVector3d& relLinVel) override {}
    virtual void setRelAngAcc(const ChVector3d& relAngAcc) override {}
    virtual void setRelLinAcc(const ChVector3d& relLinAcc) override {}

    virtual void setInboardFrame(const ChFramed& X_PF) override {}
    virtual void setOutboardFrame(const ChFramed& X_BM) override {}

    virtual double getQ0(int dof) const override { return 0.0; }
    virtual double getU0(int dof) const override { return 0.0; }

    friend class ChSoaAssembly;
};

/// @} chrono_soa

}  // namespace soa
}  // namespace chrono

#endif
