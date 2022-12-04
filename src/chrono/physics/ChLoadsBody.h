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
//
// This file contains a number of ready-to-use loads (ChLoad inherited classes and
// their embedded ChLoader classes) that can be applied to objects of ChBody (and
// inherited classes) type.
// These are 'simplified' tools, that save you from inheriting your custom loads.
// Or just look at these classes and learn how to implement some special type of load.
//
// Example:
//    std::shared_ptr<ChBodyEasyBox> body_test(new ChBodyEasyBox(8,4,4,1000));
//    sys.Add(body_test);
//
//    std::shared_ptr<ChLoadContainer> mforcecontainer (new ChLoadContainer);
//    sys.Add(mforcecontainer);
//
//    std::shared_ptr<ChLoadBodyForce> mforce (new ChLoadBodyForce(body_test, ChVector<>(0,80000,0), false,
//    ChVector<>(8,0,0),true)); mforcecontainer->Add(mforce);
//
//    std::shared_ptr<ChLoadBodyTorque> mtorque (new ChLoadBodyTorque(body_test, ChVector<>(0,0,-80000*8), true));
//    mforcecontainer->Add(mtorque);
//
// =============================================================================

#ifndef CHLOADSBODY_H
#define CHLOADSBODY_H

#include "chrono/motion_functions/ChFunction.h"
#include "chrono/physics/ChBody.h"
#include "chrono/physics/ChLoad.h"

namespace chrono {

/// Load representing a concentrated force acting on a rigid body.
/// The force can rotate together with the body (if in body local coordinates) or not.
/// The application point can follow the body (if in body local coordinates) or not.
/// The magnitude of the applied force can be optionally modulated by a function of time.
/// The default modulation is the constant function with value 1.
class ChApi ChLoadBodyForce : public ChLoadCustom {
  public:
    ChLoadBodyForce(std::shared_ptr<ChBody> body,  ///< object to apply load to
                    const ChVector<>& force,       ///< force to apply
                    bool local_force,              ///< force is in body local coords
                    const ChVector<>& point,       ///< application point for the force
                    bool local_point = true        ///< application point is in body local coords
    );

    /// "Virtual" copy constructor (covariant return type).
    virtual ChLoadBodyForce* Clone() const override { return new ChLoadBodyForce(*this); }

    /// Set the constant force vector.
    /// It can be expressed in absolute coordinates or body local coordinates.
    /// This value is optionally modulated by a function of time.
    void SetForce(const ChVector<>& force, bool is_local);

    /// Return the current force vector (scaled by the current modulation value).
    ChVector<> GetForce() const;

    /// Set the application point of force, assumed to be constant.
    /// It can be expressed in absolute coordinates or body local coordinates
    void SetApplicationPoint(const ChVector<>& point, const bool is_local);

    /// Return the location of the application point.
    ChVector<> GetApplicationPoint() const { return m_point; }

    /// Set modulation function.
    /// This is a function of time which (optionally) modulates the specified applied force.
    /// By default the modulation is a constant function, always returning a value of 1.
    void SetModulationFunction(std::shared_ptr<ChFunction> modulation) { m_modulation = modulation; }

    /// Compute Q, the generalized load.
    /// Called automatically at each Update().
    virtual void ComputeQ(ChState* state_x,      ///< state position to evaluate Q
                          ChStateDelta* state_w  ///< state speed to evaluate Q
                          ) override;

  private:
    ChVector<> m_force;  ///< base force value
    ChVector<> m_point;  ///< application point
    bool m_local_force;  ///< is force expressed in local frame?
    bool m_local_point;  ///< is application expressed in local frame?

    std::shared_ptr<ChFunction> m_modulation;  ///< modulation function of time
    double m_scale;                            ///< scaling factor (current modulation value)

    virtual bool IsStiff() override { return false; }

    virtual void Update(double time) override;
};

//------------------------------------------------------------------------------------------------

/// Load representing a torque applied to a rigid body.
/// Torque direction does not rotate with the body.
/// The magnitude of the applied torque can be optionally modulated by a function of time.
/// The default modulation is the constant function with value 1.
class ChApi ChLoadBodyTorque : public ChLoadCustom {
  public:
    ChLoadBodyTorque(std::shared_ptr<ChBody> body,  ///< object to apply load to
                     const ChVector<>& torque,      ///< torque to apply
                     bool local_torque              ///< torque is in body local coords
    );

    /// "Virtual" copy constructor (covariant return type).
    virtual ChLoadBodyTorque* Clone() const override { return new ChLoadBodyTorque(*this); }

    /// Set the constant torque vector.
    /// This value is optionally modulated by a function of time.
    void SetTorque(const ChVector<>& torque, bool is_local);

    /// Return the current torque vector (scaled by the current modulation value).
    ChVector<> GetTorque() const;

    /// Set modulation function.
    /// This is a function of time which (optionally) modulates the specified applied force.
    /// By default the modulation is a constant function, always returning a value of 1.
    void SetModulationFunction(std::shared_ptr<ChFunction> modulation) { m_modulation = modulation; }

    /// Compute Q, the generalized load.
    /// Called automatically at each Update().
    virtual void ComputeQ(ChState* state_x,      ///< state position to evaluate Q
                          ChStateDelta* state_w  ///< state speed to evaluate Q
                          ) override;

  private:
    ChVector<> m_torque;  ///< base torque value
    bool m_local_torque;  ///< is torque expressed in local frame?

    std::shared_ptr<ChFunction> m_modulation;  ///< modulation function of time
    double m_scale;                            ///< scaling factor (current modulation value)

    virtual bool IsStiff() override { return false; }

    virtual void Update(double time) override;
};

//------------------------------------------------------------------------------------------------

/// Load for adding mass and inertia to a body. Note that a ChBody already has a mass an inertia
/// tensor, however for speed optimization reasons the ChBody mass is limited to zero offset of center of mass,
/// moreover it does not generate a gyroscopical damping matrix. Therefore, you can use this load if you need:
///   - an offset between the body and its center of mass
///   - a gyroscopic damping matrix (and inertial stiffness matrix), ex. it may affect for advanced aeroelastic modal analysis.
/// Note that you must use the solvers for finite elements in order to exploit this feature.
class ChApi ChLoadBodyInertia : public ChLoadCustom {
  public:
      ChLoadBodyInertia(std::shared_ptr<ChBody> body,  ///< object to apply additional inertia to
          const ChVector<>& m_offset,     ///< offset of the center of mass, in body coordinate system
          const double m_mass,            ///< added mass [kg]
          const ChVector<>& m_IXX=VNULL,  ///< added diag. inertia values Ixx, Iyy, Izz (in body coordinate system, centered in body)
          const ChVector<>& m_IXY=VNULL   ///< added off.diag. inertia values Ixy, Ixz, Iyz including the "-"sign (in body coordinate system, centered in body)
    );

    /// "Virtual" copy constructor (covariant return type).
    virtual ChLoadBodyInertia* Clone() const override { return new ChLoadBodyInertia(*this); }

    /// Set the inertia tensor of the body, assumed in the body reference basis, with body reference as center.
    /// The provided 3x3 matrix should be symmetric and contain the inertia tensor as:
    /// system: 
    /// <pre>
    ///               [ int{y^2+z^2}dm    -int{xy}dm    -int{xz}dm    ]
    /// newXInertia = [                  int{x^2+z^2}   -int{yz}dm    ]
    ///               [     (symm.)                    int{x^2+y^2}dm ]
    /// </pre>
    void SetInertia(const ChMatrix33<>& newXInertia);

    /// Set the inertia tensor of the body, assumed in the body reference basis, with body reference as center.
    /// The return 3x3 symmetric matrix contains the following values:
    /// <pre>
    ///  [ int{y^2+z^2}dm    -int{xy}dm    -int{xz}dm    ]
    ///  [                  int{x^2+z^2}   -int{yz}dm    ]
    ///  [       (symm.)                  int{x^2+y^2}dm ]
    /// </pre>
    const ChMatrix33<>& GetInertia() const { return this->I; }

    /// Set the diagonal part of the inertia tensor (Ixx, Iyy, Izz values). 
    /// The vector should contain these moments of inertia, assumed in the body reference basis, with body reference as center:
    /// <pre>
    /// iner = [  int{y^2+z^2}dm   int{x^2+z^2}   int{x^2+y^2}dm ]
    /// </pre>
    void SetInertiaXX(const ChVector<>& iner);

    /// Get the diagonal part of the inertia tensor (Ixx, Iyy, Izz values). 
    /// The vector contains these values, assumed in the body reference basis, with body reference as center:
    /// <pre>
    /// [  int{y^2+z^2}dm   int{x^2+z^2}   int{x^2+y^2}dm ]
    /// </pre>
    ChVector<> GetInertiaXX() const;

    /// Set the off-diagonal part of the inertia tensor (Ixy, Ixz, Iyz values).
    /// The vector contains these values, assumed in the body reference basis, with body reference as center:
    /// <pre>
    /// iner = [ -int{xy}dm   -int{xz}dm   -int{yz}dm ]
    /// </pre>
    void SetInertiaXY(const ChVector<>& iner);

    /// Get the extra-diagonal part of the inertia tensor (Ixy, Ixz, Iyz values).
    /// The vector contains these values, assumed in the body reference basis, with body reference as center:
    /// <pre>
    /// [ -int{xy}dm   -int{xz}dm   -int{yz}dm ]
    /// </pre>
    ChVector<> GetInertiaXY() const;

    /// Compute Q, the generalized load. 
    /// In this case, it computes the quadratic (centrifugal, gyroscopic) terms. 
    /// Signs are negative as Q assumed at right hand side, so Q= -Fgyro -Fcentrifugal
    /// Called automatically at each Update().
    /// The M*a term is not added: to this end one could use LoadIntLoadResidual_Mv afterward.
    virtual void ComputeQ(ChState* state_x,      ///< state position to evaluate Q
                          ChStateDelta* state_w  ///< state speed to evaluate Q
                          ) override;

    /// For efficiency reasons, do not let the parent class do automatic differentiation
    /// to compute the R, K matrices. Use analytic expressions instead. For example, R is 
    /// the well known gyroscopic damping matrix. Also, compute the M matrix.
    virtual void ComputeJacobian(ChState* state_x,       ///< state position to evaluate jacobians
                                 ChStateDelta* state_w,  ///< state speed to evaluate jacobians
                                 ChMatrixRef mK,         ///< result -dQ/dx
                                 ChMatrixRef mR,         ///< result -dQ/dv
                                 ChMatrixRef mM          ///< result -dQ/da
                                 ) override;

    /// Just for efficiency, override the default LoadIntLoadResidual_Mv, because we can do this in a simplified way.
    virtual void LoadIntLoadResidual_Mv(ChVectorDynamic<>& R,        ///< result: the R residual, R += c*M*w
                                        const ChVectorDynamic<>& w,  ///< the w vector
                                        const double c               ///< scaling factor
                                        ) override;

  private:
    ChVector<> c_m;  ///< offset of center of mass
    double mass;     ///< added mass
    ChMatrix33<> I;  ///< added inertia tensor, in body coordinates

    virtual bool IsStiff() override { return true; }  // this to force the use of the inertial M, R and K matrices

    static bool use_inertial_damping_matrix_R;  // default true. Can be disabled globally, for testing or optimization
    static bool use_inertial_stiffness_matrix_K;// default true. Can be disabled globally, for testing or optimization
    static bool use_gyroscopic_torque;          // default true. Can be disabled globally, for testing or optimization
};



//------------------------------------------------------------------------------------------------

/// Base class for wrench loads (a force + a torque) acting between two bodies.
/// See children classes for concrete implementations.
class ChApi ChLoadBodyBody : public ChLoadCustomMultiple {
  public:
    ChLoadBodyBody(std::shared_ptr<ChBody> bodyA,    ///< body A
                   std::shared_ptr<ChBody> bodyB,    ///< body B
                   const ChFrame<>& abs_application  ///< location of load element (in abs. coordinates)
    );

    /// Compute the force between the two bodies, in local reference loc_application_B,
    /// given rel_AB, i.e. the position and speed of loc_application_A respect to loc_application_B.
    /// Force is assumed applied to loc_application_B of body B, and its opposite reaction to A.
    /// Inherited classes MUST IMPLEMENT THIS.
    virtual void ComputeBodyBodyForceTorque(const ChFrameMoving<>& rel_AB,
                                            ChVector<>& loc_force,
                                            ChVector<>& loc_torque) = 0;

    // Optional: inherited classes could implement this to avoid the
    // default numerical computation of jacobians:
    //   virtual void ComputeJacobian(...) // see ChLoad

    /// For diagnosis purposes, this can return the actual last computed value of
    /// the applied force, expressed in coordinate system of loc_application_B, assumed applied to body B
    ChVector<> GetForce() const { return locB_force; }

    /// For diagnosis purposes, this can return the actual last computed value of
    /// the applied torque, expressed in coordinate system of loc_application_B, assumed applied to body B
    ChVector<> GetTorque() const { return locB_torque; }

    /// Set the application frame of bushing on bodyA
    void SetApplicationFrameA(const ChFrame<>& mpA) { loc_application_A = mpA; }
    ChFrame<> GetApplicationFrameA() const { return loc_application_A; }

    /// Set the application frame of bushing on bodyB
    void SetApplicationFrameB(const ChFrame<>& mpB) { loc_application_B = mpB; }
    ChFrame<> GetApplicationFrameB() const { return loc_application_B; }

    /// Get absolute coordinate of frame A (last computed)
    ChFrameMoving<> GetAbsoluteFrameA() const { return frame_Aw; }

    /// Get absolute coordinate of frame B (last computed)
    ChFrameMoving<> GetAbsoluteFrameB() const { return frame_Bw; }

    std::shared_ptr<ChBody> GetBodyA() const;
    std::shared_ptr<ChBody> GetBodyB() const;

  protected:
    ChFrame<> loc_application_A;  ///< application point on body A (local)
    ChFrame<> loc_application_B;  ///< application point on body B (local)
    ChVector<> locB_force;        ///< store computed values here
    ChVector<> locB_torque;       ///< store computed values here
    ChFrameMoving<> frame_Aw;     ///< for results
    ChFrameMoving<> frame_Bw;     ///< for results

    /// Compute Q, the generalized load. It calls ComputeBodyBodyForceTorque, so in
    /// children classes you do not need to implement it.
    /// Called automatically at each Update().
    virtual void ComputeQ(ChState* state_x,      ///< state position to evaluate Q
                          ChStateDelta* state_w  ///< state speed to evaluate Q
                          ) override;
};

//------------------------------------------------------------------------------------------------

/// Load representing a torque applied between two bodies.
/// The magnitude of the applied torque can be optionally modulated by a function of time.
/// The default modulation is the constant function with value 1.
class ChApi ChLoadBodyBodyTorque : public ChLoadBodyBody {
  public:
    ChLoadBodyBodyTorque(std::shared_ptr<ChBody> bodyA,  ///< first body
                         std::shared_ptr<ChBody> bodyB,  ///< second body
                         const ChVector<> torque,        ///< applied torque
                         bool local_torque               ///< torque is in bodyB local coords
    );

    /// "Virtual" copy constructor (covariant return type).
    virtual ChLoadBodyBodyTorque* Clone() const override { return new ChLoadBodyBodyTorque(*this); }

    /// Set modulation function.
    /// This is a function of time which (optionally) modulates the specified applied torque.
    /// By default the modulation is a constant function, always returning a value of 1.
    void SetModulationFunction(std::shared_ptr<ChFunction> modulation) { m_modulation = modulation; }

  private:
    ChVector<> m_torque;  ///< base torque value
    bool m_local_torque;  ///< is torque expressed in local frame (bodyB)?

    std::shared_ptr<ChFunction> m_modulation;  ///< modulation function of time
    double m_scale;                            ///< scaling factor (current modulation value)

    virtual bool IsStiff() override { return false; }

    virtual void Update(double time) override;

    /// Implement the computation of the body-body force, in local
    /// coordinates of the loc_application_B.
    /// Force is assumed applied to loc_application_B of body B, and its opposite reaction to A.
    virtual void ComputeBodyBodyForceTorque(const ChFrameMoving<>& rel_AB,
                                            ChVector<>& loc_force,
                                            ChVector<>& loc_torque) override;
};

//------------------------------------------------------------------------------------------------

/// Load for a visco-elastic bushing acting between two bodies.
/// It uses three values for stiffness along the X Y Z axes of a coordinate system attached
/// to the second body. This is equivalent to having a bushing with 3x3 diagonal local stiffness matrix.
class ChApi ChLoadBodyBodyBushingSpherical : public ChLoadBodyBody {
  public:
    ChLoadBodyBodyBushingSpherical(
        std::shared_ptr<ChBody> mbodyA,    ///< body A
        std::shared_ptr<ChBody> mbodyB,    ///< body B
        const ChFrame<>& abs_application,  ///< bushing location, in abs. coordinates.
        const ChVector<>& mstiffness,      ///< stiffness, along x y z axes of the abs_application
        const ChVector<>& mdamping         ///< damping, along x y z axes of the abs_application
    );

    /// "Virtual" copy constructor (covariant return type).
    virtual ChLoadBodyBodyBushingSpherical* Clone() const override { return new ChLoadBodyBodyBushingSpherical(*this); }

    /// Set stiffness, along the x y z axes of loc_application_B, es [N/m]
    void SetStiffness(const ChVector<> mstiffness) { stiffness = mstiffness; }
    ChVector<> GetStiffness() const { return stiffness; }

    /// Set damping, along the x y z axes of loc_application_B, es [Ns/m]
    void SetDamping(const ChVector<> mdamping) { damping = mdamping; }
    ChVector<> GetDamping() const { return damping; }

  protected:
    ChVector<> stiffness;
    ChVector<> damping;

    virtual bool IsStiff() override { return true; }

    /// Implement the computation of the bushing force, in local
    /// coordinates of the loc_application_B.
    /// Force is assumed applied to loc_application_B of body B, and its opposite reaction to A.
    virtual void ComputeBodyBodyForceTorque(const ChFrameMoving<>& rel_AB,
                                            ChVector<>& loc_force,
                                            ChVector<>& loc_torque) override;
};

//------------------------------------------------------------------------------------------------

/// Load for a visco-elasto-plastic bushing acting between two bodies.
/// It uses three values for stiffness along the X Y Z axes of a coordinate system attached
/// to the second body. This is equivalent to having a bushing with 3x3 diagonal local stiffness matrix.
/// Also, it allows a very simple plasticity model, to cap the plastic force on x,y,z given three yelds.
class ChApi ChLoadBodyBodyBushingPlastic : public ChLoadBodyBodyBushingSpherical {
  public:
    ChLoadBodyBodyBushingPlastic(
        std::shared_ptr<ChBody> mbodyA,    ///< body A
        std::shared_ptr<ChBody> mbodyB,    ///< body B
        const ChFrame<>& abs_application,  ///< create the bushing here, in abs. coordinates.
        const ChVector<>& mstiffness,      ///< stiffness, along the x y z axes of the abs_application
        const ChVector<>& mdamping,        ///< damping, along the x y z axes of the abs_application
        const ChVector<>& myield           ///< plastic yield, along the x y z axes of the abs_application
    );

    /// Set plastic yield, forces beyond this limit will be capped.
    /// Expressed along the x y z axes of loc_application_B.
    void SetYeld(const ChVector<> myeld) { yield = myeld; }
    ChVector<> GetYeld() const { return yield; }

    /// Get the current accumulated plastic deformation.
    /// This could become nonzero if forces went beyond the plastic yield.
    ChVector<> GetPlasticDeformation() const { return plastic_def; }

  protected:
    ChVector<> yield;
    ChVector<> plastic_def;

    /// Implement the computation of the bushing force, in local
    /// coordinates of the loc_application_B.
    /// Force is assumed applied to loc_application_B of body B, and its opposite reaction to A.
    virtual void ComputeBodyBodyForceTorque(const ChFrameMoving<>& rel_AB,
                                            ChVector<>& loc_force,
                                            ChVector<>& loc_torque) override;
};

//------------------------------------------------------------------------------------------------

/// Load for a visco-elastic translational/rotational bushing acting between two bodies.
/// It uses three values for stiffness along the X Y Z axes of a coordinate system attached
/// to the second body , and three rotational stiffness values for (small) rotations about X Y Z of the
/// same coordinate system.
/// This is equivalent to having a bushing with 6x6 diagonal local stiffness matrix.
class ChApi ChLoadBodyBodyBushingMate : public ChLoadBodyBodyBushingSpherical {
  public:
    ChLoadBodyBodyBushingMate(
        std::shared_ptr<ChBody> mbodyA,    ///< body A
        std::shared_ptr<ChBody> mbodyB,    ///< body B
        const ChFrame<>& abs_application,  ///< create the bushing here, in abs. coordinates.
        const ChVector<>& mstiffness,      ///< stiffness, along x y z axes of the abs_application
        const ChVector<>& mdamping,        ///< damping, along x y z axes of the abs_application
        const ChVector<>& mrotstiffness,   ///< rotational stiffness, about x y z axes of the abs_application
        const ChVector<>& mrotdamping      ///< rotational damping, about x y z axes of the abs_application
    );

    /// Set radial stiffness, along the x y z axes of loc_application_B, es [N/m]
    void SetRotationalStiffness(const ChVector<> mstiffness) { rot_stiffness = mstiffness; }
    ChVector<> GetRotationalStiffness() const { return rot_stiffness; }

    /// Set radial damping, along the x y z axes of loc_application_B, es [Ns/m]
    void SetRotationalDamping(const ChVector<> mdamping) { rot_damping = mdamping; }
    ChVector<> GetRotationalDamping() const { return rot_damping; }

  protected:
    ChVector<> rot_stiffness;
    ChVector<> rot_damping;

    /// Implement the computation of the bushing force, in local
    /// coordinates of the loc_application_B.
    /// Force is assumed applied to loc_application_B of body B, and its opposite reaction to A.
    virtual void ComputeBodyBodyForceTorque(const ChFrameMoving<>& rel_AB,
                                            ChVector<>& loc_force,
                                            ChVector<>& loc_torque) override;
};

//------------------------------------------------------------------------------------------------

/// Load for a visco-elastic translational/rotational bushing acting between two bodies.
/// It uses a full user-defined 6x6 matrix [K] to express the local stiffness of the
/// bushing, assumed expressed in the bushing coordinate system  attached
/// to the second body. A user-defined 6x6 matrix [D] can be defined for damping, as well.
/// Note that this assumes small rotations.
/// Differently from the simpler ChLoadBodyBodyBushingMate and ChLoadBodyBodyBushingSpherical
/// this can represent coupled effects, by using extra-diagonal terms in [K] and/or [D].
class ChApi ChLoadBodyBodyBushingGeneric : public ChLoadBodyBody {
  public:
    ChLoadBodyBodyBushingGeneric(
        std::shared_ptr<ChBody> mbodyA,    ///< body A
        std::shared_ptr<ChBody> mbodyB,    ///< body B
        const ChFrame<>& abs_application,  ///< create the bushing here, in abs. coordinates.
        ChMatrixConstRef mstiffness,       ///< stiffness as a 6x6 matrix, local in the abs_application frame
        ChMatrixConstRef mdamping          ///< damping as a 6x6 matrix, local in the abs_application frame
    );

    /// "Virtual" copy constructor (covariant return type).
    virtual ChLoadBodyBodyBushingGeneric* Clone() const override { return new ChLoadBodyBodyBushingGeneric(*this); }

    /// Set a generic 6x6 stiffness matrix, expressed in local
    /// coordinate system of loc_application_B.
    void SetStiffnessMatrix(ChMatrixConstRef mstiffness) { stiffness = mstiffness; }
    const ChMatrixNM<double, 6, 6>& GetStiffnessMatrix() const { return stiffness; }

    /// Set a generic 6x6 damping matrix, expressed in local
    /// coordinate system of loc_application_B.
    void SetDampingMatrix(ChMatrixConstRef mdamping) { damping = mdamping; }
    const ChMatrixNM<double, 6, 6>& GetDampingMatrix() const { return damping; }

    /// Set the initial pre-load of the bushing, applied to loc_application_A,
    /// expressed in local coordinate system of loc_application_B.
    /// By default it is zero.
    void SetNeutralForce(const ChVector<> mf) { neutral_force = mf; }
    ChVector<> GetNeutralForce() const { return neutral_force; }

    /// Set the initial pre-load torque of the bushing, applied to loc_application_A,
    /// expressed in local coordinate system of loc_application_B.
    /// By default it is zero.
    void SetNeutralTorque(const ChVector<> mt) { neutral_torque = mt; }
    ChVector<> GetNeutralTorque() const { return neutral_torque; }

    /// Set/get the initial pre-displacement of the bushing, as the pre-displacement
    /// of A, expressed in local coordinate system of loc_application_B.
    /// Default behavior is no initial pre-displacement.
    ChFrame<>& NeutralDisplacement() { return neutral_displacement; }

  protected:
    ChMatrixNM<double, 6, 6> stiffness;
    ChMatrixNM<double, 6, 6> damping;

    ChVector<> neutral_force;
    ChVector<> neutral_torque;
    ChFrame<> neutral_displacement;

    virtual bool IsStiff() override { return true; }

    /// Implement the computation of the bushing force, in local
    /// coordinates of the loc_application_B.
    /// Force is assumed applied to loc_application_B of body B, and its opposite reaction to A.
    virtual void ComputeBodyBodyForceTorque(const ChFrameMoving<>& rel_AB,
                                            ChVector<>& loc_force,
                                            ChVector<>& loc_torque) override;

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}  // end namespace chrono

#endif
