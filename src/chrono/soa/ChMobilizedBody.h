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

#include <stdexcept>

#include "chrono/core/ChFrame.h"
#include "chrono/physics/ChObject.h"
#include "chrono/physics/ChContactable.h"
#include "chrono/solver/ChVariablesBodyOwnMass.h"

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
class ChApi ChMobilizedBody : public ChObj, public ChContactable_1vars<6> {
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

    virtual void setInboardFrame(const ChFramed& X_PF) { m_X_PF = X_PF; }
    virtual void setOutboardFrame(const ChFramed& X_BM) { m_X_BM = X_BM; }
    const ChFramed& getInboardFrame() const { return m_X_PF; }
    const ChFramed& getOutboardFrame() const { return m_X_BM; }

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

    ChFramed getAbsCOMPos() const { return m_X_GC; }
    ChVector3d getAbsCOMLoc() const;
    ChVector3d getAbsCOMVel() const;
    ChVector3d getAbsCOMAcc() const;

    ChFramed getAbsInboardFrame() const;
    ChFramed getAbsOutboardFrame() const;

    ChVector3d getAbsLoc(const ChVector3d& p_B) const;
    ChVector3d getAbsVel(const ChVector3d& p_B) const;
    ChVector3d getAbsAcc(const ChVector3d& p_B) const;

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

    void ApplyGravitationalForce(const ChVector3d& g);
    void ApplyBodyForce(const ChSpatialVec& force);
    void ApplyAllMobilityForces();

    const ChSpatialVec& getBodyForce() const { return m_bodyForce; }

    /// Get the reference frame (expressed in and relative to the absolute frame) of the visual model.
    /// A visual model attached to this mobilized body is expected relative to the inboard frame.
    virtual ChFrame<> GetVisualModelFrame(unsigned int nclone = 0) const override { return m_absPos; }

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

    ChMobilizedBody(const ChMobilizedBody& other);

    // Direct access to state vectors

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

    virtual void ApplyMobilityForce(int which, double force) {}
    virtual void ApplyCSMobilityForce(int which, double force) {}

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

    ChFramed m_X_GC;    ///< centroidal frame (relative to global)
    ChVector3d m_CB_G;  ///< body COM location (expressed in global)
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

    ChFramed m_X_PF;  ///< inboard (fixed) frame expressed in parent frame
    ChFramed m_X_BM;  ///< inboard (moving) frame expressed in body frame

    ChFramed m_X_FM;  ///< inboard joint transition frame (includes DOFs)
    ChFramed m_X_PB;  ///< body frame expressed in parent frame

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

  private:
    // Interface to ChContactable
    //// TODO

    ChVariablesBodyOwnMass m_variables;

    virtual ChContactable::eChContactableType GetContactableType() const override { return CONTACTABLE_6; }

    virtual ChVariables* GetVariables1() override { return &m_variables; }

    virtual bool IsContactActive() override { return true; }

    virtual int GetContactableNumCoordsPosLevel() override { return 7; }

    virtual int GetContactableNumCoordsVelLevel() override { return 6; }

    virtual ChVector3d GetContactPointSpeed(const ChVector3d& abs_point) override {
        return getAbsLinVel() + Vcross(getAbsAngVel(), abs_point - m_absPos.GetPos());
    }

    virtual double GetContactableMass() override { return m_mpropsB.mass(); }

    //// TODO: what can we return here?
    virtual ChPhysicsItem* GetPhysicsItem() override { return nullptr; }

    virtual void ContactableGetStateBlockPosLevel(ChState& x) override {
        std::cerr << "ContactableGetStateBlockPosLevel not yet implemented" << std::endl;
        throw std::runtime_error("Not yet implemented");
    }

    virtual void ContactableGetStateBlockVelLevel(ChStateDelta& w) override {
        std::cerr << "ContactableGetStateBlockVelLevel not yet implemented" << std::endl;
        throw std::runtime_error("Not yet implemented");
    }

    virtual void ContactableIncrementState(const ChState& x, const ChStateDelta& dw, ChState& x_new) override {
        std::cerr << "ContactableIncrementState not yet implemented" << std::endl;
        throw std::runtime_error("Not yet implemented");
    }

    /// Express the local point in absolute frame, for the given state position.
    virtual ChVector3d GetContactPoint(const ChVector3d& loc_point, const ChState& state_x) override {
        std::cerr << "GetContactPoint not yet implemented" << std::endl;
        throw std::runtime_error("Not yet implemented");
    }

    /// Get the absolute speed of a local point attached to the contactable.
    /// The given point is assumed to be expressed in the local frame of this object.
    /// This function must use the provided states.
    virtual ChVector3d GetContactPointSpeed(const ChVector3d& loc_point,
                                            const ChState& state_x,
                                            const ChStateDelta& state_w) override {
        std::cerr << "GetContactPointSpeed not yet implemented" << std::endl;
        throw std::runtime_error("Not yet implemented");
    }

    /// Return the frame of the associated collision model relative to the contactable object.
    /// A collision model attached to this mobilized body is expected relative to the inboard frame.
    virtual ChFrame<> GetCollisionModelFrame() override { return m_absPos; }

    /// Apply the force & torque expressed in absolute reference, applied in pos, to the
    /// coordinates of the variables. Force for example could come from a penalty model.
    virtual void ContactForceLoadResidual_F(const ChVector3d& F,
                                            const ChVector3d& T,
                                            const ChVector3d& abs_point,
                                            ChVectorDynamic<>& R) override {
        std::cerr << "ContactForceLoadResidual_F not yet implemented" << std::endl;
        throw std::runtime_error("Not yet implemented");
    }

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
                                 int offset) override {
        std::cerr << "ContactComputeQ not yet implemented" << std::endl;
        throw std::runtime_error("Not yet implemented");
    }

    /// Compute the jacobian(s) part(s) for this contactable item.
    /// For a ChBody, this updates the corresponding 1x6 jacobian.
    virtual void ComputeJacobianForContactPart(const ChVector3d& abs_point,
                                               ChMatrix33<>& contact_plane,
                                               ChVariableTupleCarrier_1vars<6>::type_constraint_tuple& jacobian_tuple_N,
                                               ChVariableTupleCarrier_1vars<6>::type_constraint_tuple& jacobian_tuple_U,
                                               ChVariableTupleCarrier_1vars<6>::type_constraint_tuple& jacobian_tuple_V,
                                               bool second) override {
        std::cerr << "ComputeJacobianForContactPart not yet implemented" << std::endl;
        throw std::runtime_error("Not yet implemented");
    }

    /// Compute the jacobian(s) part(s) for this contactable item, for rolling about N,u,v.
    /// Used only for rolling friction NSC contacts.
    virtual void ComputeJacobianForRollingContactPart(
        const ChVector3d& abs_point,
        ChMatrix33<>& contact_plane,
        ChVariableTupleCarrier_1vars<6>::type_constraint_tuple& jacobian_tuple_N,
        ChVariableTupleCarrier_1vars<6>::type_constraint_tuple& jacobian_tuple_U,
        ChVariableTupleCarrier_1vars<6>::type_constraint_tuple& jacobian_tuple_V,
        bool second) override {
        std::cerr << "ComputeJacobianForRollingContactPart not yet implemented" << std::endl;
        throw std::runtime_error("Not yet implemented");
    }

    friend class ChSoaAssembly;
    friend class ChMobilityForce;
    template <int>
    friend class ChMobilizedBodyT;
    friend class mbWeldBody;
    friend class mbConstraint;
};

// =============================================================================

/// Special body used as root of a multibody system.
/// A ChGroundBody exists simply as an initiation point for the recursive traversals of the multibody tree and as a
/// holder of the base bodies (those bodies that have ground as their parent).
class ChApi ChGroundBody : public ChMobilizedBody {
  public:
    ChGroundBody();
    ChGroundBody(const ChGroundBody& other);

    ~ChGroundBody() {}

    /// "Virtual" copy constructor (covariant return type).
    virtual ChGroundBody* Clone() const override { return new ChGroundBody(*this); }

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
