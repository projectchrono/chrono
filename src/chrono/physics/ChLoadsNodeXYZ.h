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
// This file contains a number of ready-to-use loads (ChLoad inherited classes
// and their embedded ChLoader classes) that can be applied to objects of
// ChNodeXYZ (and inherited classes) type. These are 'simplified' tools, as an
// alternative to implementing custom loads.
//
// =============================================================================

#ifndef CH_LOADS_NODE_XYZ_H
#define CH_LOADS_NODE_XYZ_H

#include "chrono/physics/ChLoad.h"
#include "chrono/physics/ChNodeXYZ.h"
#include "chrono/physics/ChBody.h"
#include "chrono/functions/ChFunction.h"

namespace chrono {

/// Loader for a constant force applied at a XYZ node.
/// An alternative is to set directly the node's own force member data as mynode->SetForce(), but this approach is more
/// flexible (e.g. you can  apply multiple forces at a single node). Another option is to use ChLoadNodeXYZForceAbs.
class ChLoaderNodeXYZ : public ChLoaderUVWatomic {
  public:
    ChLoaderNodeXYZ(std::shared_ptr<ChLoadableUVW> loadable) : ChLoaderUVWatomic(loadable, 0, 0, 0), force(VNULL) {}

    /// Compute F=F(u,v,w).
    virtual void ComputeF(
        double U,                    ///< parametric coordinate -not used
        double V,                    ///< parametric coordinate -not used
        double W,                    ///< parametric coordinate -not used
        ChVectorDynamic<>& F,        ///< result vector, size = field dim of loadable
        ChVectorDynamic<>* state_x,  ///< if not null, update state (pos. part) to this, then evaluate F
        ChVectorDynamic<>* state_w   ///< if not null, update state (speed part) to this, then evaluate F
        ) override {
        F.segment(0, 3) = force.eigen();
    }

    /// Set the applied nodal force, assumed to be constant in space and time.
    void SetForce(const ChVector3d& mf) { force = mf; }

    /// Get the applied force.
    const ChVector3d& GetForce() const { return force; }

  private:
    ChVector3d force;
};

/// Force at XYZ node (ready to use load).
class ChLoadNodeXYZ : public ChLoad {
  public:
    ChLoadNodeXYZ(std::shared_ptr<ChNodeXYZ> node) { SetLoader(chrono_types::make_shared<ChLoaderNodeXYZ>(node)); }
    ChLoadNodeXYZ(std::shared_ptr<ChNodeXYZ> node, const ChVector3d& force) {
        auto node_loader = chrono_types::make_shared<ChLoaderNodeXYZ>(node);
        node_loader->SetForce(force);
        SetLoader(node_loader);
    }
    virtual ChLoadNodeXYZ* Clone() const override { return new ChLoadNodeXYZ(*this); }
};

// -----------------------------------------------------------------------------
//
//  LOADS ON:   ChNodeXYZ

/// Base class for loads representing a concentrated force acting on a ChNodeXYZ.
/// Users should inherit from this and implement custom ComputeForce().
/// Note: there are already some predefined examples, ie. children classes
/// with common concrete implementations, such as ChLoadNodeXYZForceAbs ;
/// take inspiration from those example to inherit your own load if they are not enough.
class ChApi ChLoadNodeXYZForce : public ChLoadCustom {
  public:
    ChLoadNodeXYZForce(std::shared_ptr<ChNodeXYZ> node);

    /// Compute the force on the node, in absolute coordsystem, given position of node as abs_pos.
    virtual void ComputeForce(const ChVector3d& abs_pos, const ChVector3d& abs_vel, ChVector3d& abs_force) = 0;

    // Optional: inherited classes could implement this to avoid the
    // default numerical computation of Jacobians:
    //   virtual void ComputeJacobian(...) // see ChLoad

    /// Compute Q, the generalized load.
    /// Called automatically at each Update().
    virtual void ComputeQ(ChState* state_x,      ///< state position to evaluate Q
                          ChStateDelta* state_w  ///< state speed to evaluate Q
                          ) override;

    /// Return the last computed value of the applied force.
    /// Used primarily for diagnostics, this function returns the force on the node expressed in absolute coordinates.
    ChVector3d GetForce() const { return computed_abs_force; }

  protected:
    // Declare this load as non-stiff.
    virtual bool IsStiff() override { return false; }

    virtual void Update(double time, bool update_assets) override;

    ChVector3d computed_abs_force;
};

/// Load representing a concentrated force acting on a ChNodeXYZ.
/// The force can be constant or optionally modulated with time.
class ChApi ChLoadNodeXYZForceAbs : public ChLoadNodeXYZForce {
  public:
    ChLoadNodeXYZForceAbs(std::shared_ptr<ChNodeXYZ> node,  ///< node to apply load to
                          const ChVector3d& force           ///< force to apply, assumed in absolute coordsys,
    );

    /// "Virtual" copy constructor (covariant return type).
    virtual ChLoadNodeXYZForceAbs* Clone() const override { return new ChLoadNodeXYZForceAbs(*this); }

    /// Compute the force on the node, in absolute coordsystem, given position of node as abs_pos.
    virtual void ComputeForce(const ChVector3d& abs_pos, const ChVector3d& abs_vel, ChVector3d& abs_force) override;

    /// Set the applied force vector (expressed in absolute coordinates).
    /// The force is assumed constant, unless a scaling time function is provided.
    void SetForceBase(const ChVector3d& force);

    /// Return the current force vector (scaled by the current modulation value).
    ChVector3d GetForce() const;

    /// Set modulation function.
    /// This is a function of time which (optionally) modulates the specified applied force.
    /// By default the modulation is a constant function, always returning a value of 1.
    void SetModulationFunction(std::shared_ptr<ChFunction> modulation) { m_modulation = modulation; }

  protected:
    ChVector3d m_force_base;  ///< base force value

    std::shared_ptr<ChFunction> m_modulation;  ///< modulation function of time
    double m_scale;                            ///< scaling factor (current modulation value)

    // Declare this load as non-stiff.
    virtual bool IsStiff() override { return false; }

    virtual void Update(double time, bool update_assets) override;
};

// -----------------------------------------------------------------------------
//
//  LOADS ON:   ChNodeXYZ ---- ChNodeXYZ

/// Base class for loads representing a concentrated force acting between two ChNodeXYZ.
/// Users should inherit from this and implement a custom ComputeForce().
/// Note: some predefined examples are provided, such as ChLoadNodeXYZNodeXYZSpring ChLoadNodeXYZNodeXYZBushing.
class ChApi ChLoadNodeXYZNodeXYZ : public ChLoadCustomMultiple {
  public:
    ChLoadNodeXYZNodeXYZ(std::shared_ptr<ChNodeXYZ> nodeA,  ///< node A to apply load to
                         std::shared_ptr<ChNodeXYZ> nodeB   ///< node B to apply load to, as reaction
    );

    /// Compute the force on the nodeA, in absolute coordsystem, given relative position of nodeA with respect to B.
    virtual void ComputeForce(const ChVector3d& rel_pos, const ChVector3d& rel_vel, ChVector3d& abs_force) = 0;

    // Optional: inherited classes could implement this to avoid the
    // default numerical computation of Jacobians:
    //   virtual void ComputeJacobian(...) // see ChLoad

    /// Compute Q, the generalized load.
    /// Called automatically at each Update().
    virtual void ComputeQ(ChState* state_x,      ///< state position to evaluate Q
                          ChStateDelta* state_w  ///< state speed to evaluate Q
                          ) override;

    /// Return the last computed value of the applied force.
    /// Used primarily for diagnostics, this function returns the force on the node expressed in absolute coordinates.
    ChVector3d GetForce() const { return computed_abs_force; }

  protected:
    // Declare this load as non-stiff.
    virtual bool IsStiff() override { return false; }

    virtual void Update(double time, bool update_assets) override;

    ChVector3d computed_abs_force;
};

/// Load representing a spring between two ChNodeXYZ.
/// This load, with given damping and spring constants, acts along the line connecting the two nodes.
class ChApi ChLoadNodeXYZNodeXYZSpring : public ChLoadNodeXYZNodeXYZ {
  public:
    ChLoadNodeXYZNodeXYZSpring(std::shared_ptr<ChNodeXYZ> nodeA,  ///< node to apply load to
                               std::shared_ptr<ChNodeXYZ> nodeB,  ///< node to apply load to as reaction
                               double stiffness,                  ///< stiffness
                               double damping,                    ///< damping
                               double rest_length = 0             ///< initial rest length
    );

    /// "Virtual" copy constructor (covariant return type).
    virtual ChLoadNodeXYZNodeXYZSpring* Clone() const override { return new ChLoadNodeXYZNodeXYZSpring(*this); }

    /// Compute the force on the nodeA, in absolute coordsystem, given relative position of nodeA with respect to B.
    virtual void ComputeForce(const ChVector3d& rel_pos, const ChVector3d& rel_vel, ChVector3d& abs_force) override;

    /// Set stiffness, along direction.
    void SetStiffness(const double stiffness) { K = stiffness; }
    double GetStiffness() const { return K; }

    /// Set damping, along direction.
    void SetDamping(const double damping) { R = damping; }
    double GetDamping() const { return R; }

    /// Set initial spring length.
    void SetRestLength(const double mrest) { d0 = mrest; }
    double GetRestLength() const { return d0; }

    /// Declare this load as stiff or non-stiff.
    /// If set as a stiff load, this enables the automatic computation of the Jacobian through finite differences.
    void SetStiff(bool stiff) { is_stiff = stiff; }

  protected:
    double K;
    double R;
    double d0;
    bool is_stiff;

    virtual bool IsStiff() override { return is_stiff; }
};

/// Load representing an XYZ bushing between two ChNodeXYZ.
/// This load is specified through the stiffnesses along each direction, as functions of displacement.
class ChApi ChLoadNodeXYZNodeXYZBushing : public ChLoadNodeXYZNodeXYZ {
  public:
    ChLoadNodeXYZNodeXYZBushing(std::shared_ptr<ChNodeXYZ> nodeA,  ///< node to apply load to
                                std::shared_ptr<ChNodeXYZ> nodeB   ///< node to apply load to as reaction
    );

    /// "Virtual" copy constructor (covariant return type).
    virtual ChLoadNodeXYZNodeXYZBushing* Clone() const override { return new ChLoadNodeXYZNodeXYZBushing(*this); }

    /// Compute the force on the nodeA, in absolute coordsystem, given relative position of nodeA with respect to B.
    virtual void ComputeForce(const ChVector3d& rel_pos, const ChVector3d& rel_vel, ChVector3d& abs_force) override;

    /// Set force as a function of displacement on X (default: constant 0 function).
    void SetFunctionForceX(std::shared_ptr<ChFunction> fx) { force_dX = fx; }

    /// Set force as a function of displacement on X (default: constant 0 function).
    void SetFunctionForceY(std::shared_ptr<ChFunction> fy) { force_dY = fy; }

    /// Set force as a function of displacement on X (default: constant 0 function).
    void SetFunctionForceZ(std::shared_ptr<ChFunction> fz) { force_dZ = fz; }

    /// Set xyz constant damping coefficients along the three absolute directions xyz.
    void SetDamping(const ChVector3d damping) { R = damping; }

    /// Get the damping coefficients along the three absolute directions xyz.
    ChVector3d GetDamping() const { return R; }

    /// Declare this load as stiff or non-stiff.
    /// If set as a stiff load, this enables the automatic computation of the Jacobian through finite differences.
    void SetStiff(bool ms) { is_stiff = ms; }

  protected:
    std::shared_ptr<ChFunction> force_dX;
    std::shared_ptr<ChFunction> force_dY;
    std::shared_ptr<ChFunction> force_dZ;

    ChVector3d R;   ///< damping coefficients along xyz directions
    bool is_stiff;  ///< flag indicating a stiff/non-stiff load

    virtual bool IsStiff() override { return is_stiff; }
};

// -----------------------------------------------------------------------------
//
//  LOADS ON:   ChNodeXYZ ---- ChBody

/// Base class for loads representing a concentrated force acting between a ChNodeXYZ and a ChBody
/// Users should inherit from this and implement a custom ComputeForce().
/// Note: there are already some predefined examples, ie. children classes
/// with common concrete implementations, such as ChLoadNodeXYZBodySpring ChLoadNodeXYZBodyBushing etc.;
/// take inspiration from those example to inherit your own load if they are not enough.
class ChApi ChLoadNodeXYZBody : public ChLoadCustomMultiple {
  public:
    ChLoadNodeXYZBody(std::shared_ptr<ChNodeXYZ> node,  ///< node
                      std::shared_ptr<ChBody> body      ///< body
    );

    /// Compute the force on the nodeA, in local coord system, given relative position of nodeA respect to B.
    /// The local coordinate system is that specified with SetApplicationFrameB (the auxiliary frame attached to body).
    virtual void ComputeForce(const ChFrameMoving<>& rel_AB, ChVector3d& loc_force) = 0;

    // Optional: inherited classes could implement this to avoid the
    // default numerical computation of Jacobians:
    //   virtual void ComputeJacobian(...) // see ChLoad

    /// Return the last computed value of the applied force.
    /// Used primarily for diagnostics, this function returns the force on the node expressed in absolute coordinates.
    ChVector3d GetForce() const { return computed_loc_force; }

    /// Set the application frame of load on body.
    void SetApplicationFrameB(const ChFrame<>& application_frame) { loc_application_B = application_frame; }

    /// Get the application frame of load on body.
    ChFrame<> GetApplicationFrameB() const { return loc_application_B; }

    /// Get absolute coordinate of body frame (last computed).
    ChFrameMoving<> GetAbsoluteFrameB() const { return frame_Bw; }

    std::shared_ptr<ChNodeXYZ> GetNode() const;
    std::shared_ptr<ChBody> GetBody() const;

  protected:
    ChFrame<> loc_application_B;    ///< application point on body B (local)
    ChVector3d computed_loc_force;  ///< store computed values here
    ChFrameMoving<> frame_Aw;       ///< for results
    ChFrameMoving<> frame_Bw;       ///< for results

    /// Compute Q, the generalized load. It calls ComputeBodyBodyForceTorque, so in
    /// children classes you do not need to implement it.
    /// Called automatically at each Update().
    virtual void ComputeQ(ChState* state_x,      ///< state position to evaluate Q
                          ChStateDelta* state_w  ///< state speed to evaluate Q
                          ) override;
};

/// Load representing a spring between a ChNodeXYZ and a ChBody.
/// The anchoring to body can be set via SetApplicationFrameB().
class ChApi ChLoadNodeXYZBodySpring : public ChLoadNodeXYZBody {
  public:
    ChLoadNodeXYZBodySpring(std::shared_ptr<ChNodeXYZ> mnodeA,  ///< node to apply load to
                            std::shared_ptr<ChBody> mbodyB,     ///< node to apply load to as reaction
                            double stiffness,                   ///< stiffness
                            double damping,                     ///< damping
                            double rest_length = 0              ///< initial rest length
    );

    /// "Virtual" copy constructor (covariant return type).
    virtual ChLoadNodeXYZBodySpring* Clone() const override { return new ChLoadNodeXYZBodySpring(*this); }

    /// Compute the force on the nodeA, in local coord system, given relative position of nodeA respect to B.
    /// The local coordinate system is that specified with SetApplicationFrameB (the auxiliary frame attached to body).
    virtual void ComputeForce(const ChFrameMoving<>& rel_AB, ChVector3d& loc_force) override;

    /// Set stiffness, along direction.
    void SetStiffness(const double stiffness) { K = stiffness; }
    double GetStiffness() const { return K; }

    /// Set damping, along direction.
    void SetDamping(const double damping) { R = damping; }
    double GetDamping() const { return R; }

    /// Set initial spring length.
    void SetRestLength(const double mrest) { d0 = mrest; }
    double GetRestLength() const { return d0; }

    /// Declare this load as stiff or non-stiff.
    /// If set as a stiff load, this enables the automatic computation of the Jacobian through finite differences.
    void SetStiff(bool ms) { is_stiff = ms; }

  protected:
    double K;
    double R;
    double d0;
    bool is_stiff;

    virtual bool IsStiff() override { return is_stiff; }
};

/// Load representing a XYZ bushing between a ChNodeXYZ and a ChBody application point, with given
/// with spring stiffness as a ChFunction of displacement, for each X,Y,Z direction along the
/// auxiliary frame at the attachment point. You can set the attachment point via SetApplicationFrameB().
class ChApi ChLoadNodeXYZBodyBushing : public ChLoadNodeXYZBody {
  public:
    ChLoadNodeXYZBodyBushing(std::shared_ptr<ChNodeXYZ> nodeA,  ///< node to apply load
                             std::shared_ptr<ChBody> bodyB      ///< body to apply load
    );

    /// "Virtual" copy constructor (covariant return type).
    virtual ChLoadNodeXYZBodyBushing* Clone() const override { return new ChLoadNodeXYZBodyBushing(*this); }

    /// Compute the force on the nodeA, in local coord system, given relative position of nodeA respect to B.
    /// The local coordinate system is that specified with SetApplicationFrameB (the auxiliary frame attached to body).
    virtual void ComputeForce(const ChFrameMoving<>& rel_AB, ChVector3d& loc_force) override;

    /// Set force as a function of displacement on X (default: constant 0 function).
    void SetFunctionForceX(std::shared_ptr<ChFunction> fx) { force_dX = fx; }

    /// Set force as a function of displacement on X (default: constant 0 function).
    void SetFunctionForceY(std::shared_ptr<ChFunction> fy) { force_dY = fy; }

    /// Set force as a function of displacement on X (default: constant 0 function).
    void SetFunctionForceZ(std::shared_ptr<ChFunction> fz) { force_dZ = fz; }

    /// Set xyz constant damping coefficients along the three directions xyz.
    /// This assumes local xyz directions of the frame attached to body via SetApplicationFrameB.
    void SetDamping(const ChVector3d damping) { R = damping; }

    /// Return the vector of damping coefficients.
    ChVector3d GetDamping() const { return R; }

    /// Declare this load as stiff or non-stiff (default: false).
    /// If set as a stiff load, this enables the automatic computation of the Jacobian through finite differences.
    void SetStiff(bool ms) { is_stiff = ms; }

  protected:
    std::shared_ptr<ChFunction> force_dX;
    std::shared_ptr<ChFunction> force_dY;
    std::shared_ptr<ChFunction> force_dZ;

    ChVector3d R;   ///< damping coefficients along xyz directions
    bool is_stiff;  ///< flag indicating a stiff/non-stiff load

    virtual bool IsStiff() override { return is_stiff; }
};

}  // end namespace chrono

#endif
