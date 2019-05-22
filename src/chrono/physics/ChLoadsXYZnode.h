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
// Authors: Alessandro Tasora
// =============================================================================

#ifndef CHLOADSXYZNODE_H
#define CHLOADSXYZNODE_H

#include "chrono/physics/ChLoad.h"
#include "chrono/physics/ChNodeXYZ.h"
#include "chrono/physics/ChBody.h"
#include "chrono/motion_functions/ChFunction.h"

namespace chrono {

// This file contains a number of ready-to-use loads (ChLoad inherited classes and
// their embedded ChLoader classes) that can be applied to objects of ChNodeXYZ (and 
// inherited classes) type.
// These are 'simplified' tools, that save you from inheriting your custom 
// loads. 
// Or just look at these classes and learn how to implement some special type of load.


// A first example, mostly an exercise, that inherits from ChLoaderUVWatomic:

/// FORCE AT XYZ NODE
/// Loader for a constant force applied at a XYZ node.
/// Note that an equivalent shortcut is to set directly the node's own force member data
/// as mynode->SetForce(), but this other approach is more object-oriented (es. you can 
/// apply multiple forces at a single node, etc)
/// Another option is to use ChLoadXYZnodeForceAbsolute
class ChLoaderXYZnode : public ChLoaderUVWatomic {
  private:
    ChVector<> force;

  public:
    // Useful: a constructor that also sets ChLoadable
    ChLoaderXYZnode(std::shared_ptr<ChLoadableUVW> mloadable) : ChLoaderUVWatomic(mloadable, 0, 0, 0) {
        this->force = VNULL;
    };

    // Compute F=F(u,v,w)
    // Implement it from base class.
    virtual void ComputeF(const double U,        ///< parametric coordinate -not used
                          const double V,        ///< parametric coordinate -not used
                          const double W,        ///< parametric coordinate -not used
                          ChVectorDynamic<>& F,  ///< Result F vector here, size must be = n.field coords.of loadable
                          ChVectorDynamic<>* state_x,  ///< if != 0, update state (pos. part) to this, then evaluate F
                          ChVectorDynamic<>* state_w   ///< if != 0, update state (speed part) to this, then evaluate F
                          ) {
        F.PasteVector(this->force, 0, 0);  // load, force part
    }

    /// Set force (ex. in [N] units), assumed to be constant in space and time,
    /// assumed applied at the node.
    void SetForce(const ChVector<>& mf) { this->force = mf; }
    ChVector<> GetForce() const { return this->force; }
};

/// Force at XYZ node (ready to use load)
class ChLoadXYZnode : public ChLoad<ChLoaderXYZnode> {
  public:
    ChLoadXYZnode(std::shared_ptr<ChNodeXYZ> mloadable) : ChLoad<ChLoaderXYZnode>(mloadable) {}
};



////////////////////////////////////////////////////////////////////////////
// 
//  LOADS ON:   ChNodeXYZ


/// Base class for loads representing a concentrated force acting on a ChNodeXYZ.
/// Users should inherit from this and implement custom ComputeForce(), this is enough
/// to have the load working.
/// Note: there are already some predefined examples, ie. children classes 
/// with common concrete implementations, such as ChLoadXYZnodeForceAbsolute ;
/// take inspiration from those example to inherit your own load if they are not enough.
class ChApi ChLoadXYZnodeForce : public ChLoadCustom {
  public:
    ChLoadXYZnodeForce(std::shared_ptr<ChNodeXYZ> body  ///< node to apply load to
    );

    /// Compute the force on the node, in absolute coordsystem,
    /// given position of node as abs_pos.
    /// Inherited classes MUST IMPLEMENT THIS.
    virtual void ComputeForce(const ChVector<>& abs_pos,
									 const ChVector<>& abs_vel,
                                           ChVector<>& abs_force) = 0;

	// Optional: inherited classes could implement this to avoid the
    // default numerical computation of jacobians:
    //   virtual void ComputeJacobian(...) // see ChLoad

    /// Compute Q, the generalized load.
    /// Called automatically at each Update().
    virtual void ComputeQ(ChState* state_x,      ///< state position to evaluate Q
                          ChStateDelta* state_w  ///< state speed to evaluate Q
                          ) override;

	/// For diagnosis purposes, this can return the actual last computed value of
    /// the applied force, expressed in absolute coordinate system, assumed applied to node
    ChVector<> GetForce() const { return computed_abs_force; }

  protected:

	/// Inherited classes could override this and return true, if the load benefits from a jacobian
	/// when using implicit integrators. 
    virtual bool IsStiff() override { return false; }

    virtual void Update(double time) override;

	ChVector<> computed_abs_force;
};



 // Example: 
/// Load representing a concentrated force acting on a ChNodeXYZ,
/// as a constant force, or it provides a function to modulate 
/// it with time.
class ChApi ChLoadXYZnodeForceAbsolute : public ChLoadXYZnodeForce {
  public:
    ChLoadXYZnodeForceAbsolute(std::shared_ptr<ChNodeXYZ> node,  ///< node to apply load to
                    const ChVector<>& force        ///< force to apply, assumed in absolute coordsys,
    );

    /// "Virtual" copy constructor (covariant return type).
    virtual ChLoadXYZnodeForceAbsolute* Clone() const override { return new ChLoadXYZnodeForceAbsolute(*this); }

	/// Compute the force on the node, in absolute coordsystem,
    /// given position of node as abs_pos.
    virtual void ComputeForce(const ChVector<>& abs_pos,
									 const ChVector<>& abs_vel,
                                           ChVector<>& abs_force) override;

    /// Set the applied force vector: it is assumed constant, unless a
	/// non-constant scaling time function is provided.
    /// It is expressed in absolute coordinates. 
    void SetForceBase(const ChVector<>& force);

    /// Return the current force vector (scaled by the current modulation value).
    ChVector<> GetForce() const;

    /// Set modulation function.
    /// This is a function of time which (optionally) modulates the specified applied force.
    /// By default the modulation is a constant function, always returning a value of 1.
    void SetModulationFunction(std::shared_ptr<ChFunction> modulation) { m_modulation = modulation; }


  protected:
    ChVector<> m_force_base;  ///< base force value

	std::shared_ptr<ChFunction> m_modulation;  ///< modulation function of time
    double m_scale;                            ///< scaling factor (current modulation value)

    virtual bool IsStiff() override { return false; }

    virtual void Update(double time) override;
};



////////////////////////////////////////////////////////////////////////////
// 
//  LOADS ON:   ChNodeXYZ ---- ChNodeXYZ


/// Base class for loads representing a concentrated force acting between two ChNodeXYZ.
/// Users should inherit from this and implement a custom ComputeXYZnodeForce(), this is enough
/// to have the load working.
/// Note: there are already some predefined examples, ie. children classes 
/// with common concrete implementations, such as ChLoadXYZnodeXYZnodeSpring ChLoadXYZnodeXYZnodeBushing etc.;
/// take inspiration from those example to inherit your own load if they are not enough.
class ChApi ChLoadXYZnodeXYZnode : public ChLoadCustomMultiple {
  public:
    ChLoadXYZnodeXYZnode(std::shared_ptr<ChNodeXYZ> mnodeA,  ///< node A to apply load to
						 std::shared_ptr<ChNodeXYZ> mnodeB   ///< node B to apply load to, as reaction
    );

    /// Compute the force on the node A, in absolute coordsystem,
    /// given position and speed of node respect to the other. Node B gets the opposite.
    /// Inherited classes MUST IMPLEMENT THIS.
    virtual void ComputeForce(const ChVector<>& rel_pos,
							  const ChVector<>& rel_vel,
                              ChVector<>& abs_force) = 0;

	// Optional: inherited classes could implement this to avoid the
    // default numerical computation of jacobians:
    //   virtual void ComputeJacobian(...) // see ChLoad

    /// Compute Q, the generalized load.
    /// Called automatically at each Update().
    virtual void ComputeQ(ChState* state_x,      ///< state position to evaluate Q
                          ChStateDelta* state_w  ///< state speed to evaluate Q
                          ) override;

	/// For diagnosis purposes, this can return the actual last computed value of
    /// the applied force, expressed in absolute coordinate system, assumed applied to node
    ChVector<> GetForce() const { return computed_abs_force; }

  protected:

	/// Inherited classes could override this and return true, if the load benefits from a jacobian
	/// when using implicit integrators. 
    virtual bool IsStiff() override { return false; }

    virtual void Update(double time) override;

	ChVector<> computed_abs_force;
};


 // Example:   
/// Load representing a spring between two ChNodeXYZ, with given damping and spring constants,
/// directed as the distance between the two.
class ChApi ChLoadXYZnodeXYZnodeSpring : public ChLoadXYZnodeXYZnode {
  public:
	  ChLoadXYZnodeXYZnodeSpring(std::shared_ptr<ChNodeXYZ> mnodeA,  ///< node to apply load to
								  std::shared_ptr<ChNodeXYZ> mnodeB,   ///< node to apply load to as reaction
								  double mK,	///< stiffness,
								  double mR,	///< damping,
								  double mD0=0	///< initial rest length
    );

    /// "Virtual" copy constructor (covariant return type).
    virtual ChLoadXYZnodeXYZnodeSpring* Clone() const override { return new ChLoadXYZnodeXYZnodeSpring(*this); }

	/// Compute the force on the nodeA, in absolute coordsystem,
    /// given relative position of nodeA respect to B, in absolute basis
    virtual void ComputeForce(const ChVector<>& rel_pos,
									 const ChVector<>& rel_vel,
                                           ChVector<>& abs_force) override;

    /// Set stiffness, along direction, es [N/m]
    void SetStiffness(const double mstiffness) { K = mstiffness; }
    double GetStiffness() const { return K; }

    /// Set damping, along direction, es [Ns/m]
    void SetDamping(const double mdamping) { R = mdamping; }
    double GetDamping() const { return R; }

	/// Set initial spring length, es [m]
    void SetRestLength(const double mrest) { d_0 = mrest; }
    double GetRestLength() const { return d_0; }

	/// Use this to enable the stiff force computation (i.e. it enables the
	/// automated computation of the jacobian by numerical differentiation to 
	/// elp the convergence of implicit integrators, but adding CPU overhead).
	void SetStiff(bool ms) { this->is_stiff = ms; }

  protected:
	  double K;
	  double R;
	  double d_0;
	  bool is_stiff;

	  virtual bool IsStiff() override { return is_stiff; }
};


 // Example:
/// Load representing a XYZ bushing between two ChNodeXYZ, with given 
/// with spring stiffness as a ChFunction of displacement, for each X,Y,Z direction.
class ChApi ChLoadXYZnodeXYZnodeBushing : public ChLoadXYZnodeXYZnode {
  public:
	  ChLoadXYZnodeXYZnodeBushing(std::shared_ptr<ChNodeXYZ> mnodeA,  ///< node to apply load to
								  std::shared_ptr<ChNodeXYZ> mnodeB);   ///< node to apply load to as reaction

    /// "Virtual" copy constructor (covariant return type).
    virtual ChLoadXYZnodeXYZnodeBushing* Clone() const override { return new ChLoadXYZnodeXYZnodeBushing(*this); }

	/// Compute the force on the nodeA, in absolute coordsystem,
    /// given relative position of nodeA respect to B, in absolute basis
    virtual void ComputeForce(const ChVector<>& rel_pos,
									 const ChVector<>& rel_vel,
                                           ChVector<>& abs_force) override;
	
	/// Set force as a function of displacement on X. Default was constant zero. 
	void SetFunctionForceX(std::shared_ptr<ChFunction> fx) { force_dX = fx; }

	/// Set force as a function of displacement on X. Default was constant zero. 
	void SetFunctionForceY(std::shared_ptr<ChFunction> fy) { force_dY = fy; }
	
	/// Set force as a function of displacement on X. Default was constant zero. 
	void SetFunctionForceZ(std::shared_ptr<ChFunction> fz) { force_dZ = fz; }

	/// Set xyz constant damping coefficients [Ns/m] 
	/// along the three absolute directions xyz
    void SetDamping(const ChVector<> mdamping) { R = mdamping; }
    ChVector<> GetDamping() const { return R; }

	/// Use this to enable the stiff force computation (i.e. it enables the
	/// automated computation of the jacobian by numerical differentiation to 
	/// elp the convergence of implicit integrators, but adding CPU overhead).
	void SetStiff(bool ms) { this->is_stiff = ms; }

  protected:
	  std::shared_ptr<ChFunction> force_dX;
	  std::shared_ptr<ChFunction> force_dY;
	  std::shared_ptr<ChFunction> force_dZ;

	  ChVector<> R;

	  bool is_stiff;

	  virtual bool IsStiff() override { return is_stiff; }
};




////////////////////////////////////////////////////////////////////////////
// 
//  LOADS ON:   ChNodeXYZ ---- ChBody



/// Base class for loads representing a concentrated force acting between a ChNodeXYZ and a ChBody
/// Users should inherit from this and implement a custom ComputeForce(), this is enough
/// to have the load working.
/// Note: there are already some predefined examples, ie. children classes 
/// with common concrete implementations, such as ChLoadXYZnodeBodySpring ChLoadXYZnodeBodyBushing etc.;
/// take inspiration from those example to inherit your own load if they are not enough.
class ChApi ChLoadXYZnodeBody : public ChLoadCustomMultiple {
  public:
    ChLoadXYZnodeBody   (std::shared_ptr<ChNodeXYZ> nodeA,    ///< node
						 std::shared_ptr<ChBody> bodyB        ///< body 
    );

    /// Compute the force on the nodeA, in local coordsystem of SetApplicationFrameB (the 
	/// auxiliary frame attached to body) given relative position of nodeA respect to B
    /// Body will receive the reaction force.
    /// Inherited classes MUST IMPLEMENT THIS.
    virtual void ComputeForce(const ChFrameMoving<>& rel_AB,
                               ChVector<>& loc_force) = 0;

    // Optional: inherited classes could implement this to avoid the
    // default numerical computation of jacobians:
    //   virtual void ComputeJacobian(...) // see ChLoad

    /// For diagnosis purposes, this can return the actual last computed value of
    /// the applied force, expressed in coordinate system of loc_application_B, assumed applied to node.
    ChVector<> GetForce() const { return computed_loc_force; }

    /// Set the application frame of bushing on bodyB
    void SetApplicationFrameB(const ChFrame<>& mpB) { loc_application_B = mpB; }
    ChFrame<> GetApplicationFrameB() const { return loc_application_B; }

    /// Get absolute coordinate of frame B (last computed)
    ChFrameMoving<> GetAbsoluteFrameB() const { return frame_Bw; }

    std::shared_ptr<ChNodeXYZ> GetNodeA() const;
    std::shared_ptr<ChBody> GetBodyB() const;

  protected:
    ChFrame<> loc_application_B;  ///< application point on body B (local)
    ChVector<> computed_loc_force;  ///< store computed values here
    ChFrameMoving<> frame_Aw;     ///< for results
    ChFrameMoving<> frame_Bw;     ///< for results

    /// Compute Q, the generalized load. It calls ComputeBodyBodyForceTorque, so in
    /// children classes you do not need to implement it.
    /// Called automatically at each Update().
    virtual void ComputeQ(ChState* state_x,      ///< state position to evaluate Q
                          ChStateDelta* state_w  ///< state speed to evaluate Q
                          ) override;
};


 // Example:
/// Load representing a spring between a ChNodeXYZ and a ChBody, where the
/// anchoring to body can be set via SetApplicationFrameB().
class ChApi ChLoadXYZnodeBodySpring : public ChLoadXYZnodeBody {
  public:
	  ChLoadXYZnodeBodySpring    (std::shared_ptr<ChNodeXYZ> mnodeA,  ///< node to apply load to
								  std::shared_ptr<ChBody> mbodyB,   ///< node to apply load to as reaction
								  double mK,	///< stiffness,
								  double mR,	///< damping,
								  double mD0=0	///< initial rest length
    );

    /// "Virtual" copy constructor (covariant return type).
    virtual ChLoadXYZnodeBodySpring* Clone() const override { return new ChLoadXYZnodeBodySpring(*this); }

	/// Compute the force on the nodeA, in local coordsystem of SetApplicationFrameB (the 
	/// auxiliary frame attached to body) given relative position of nodeA respect to B
    virtual void ComputeForce(const ChFrameMoving<>& rel_AB,
                               ChVector<>& loc_force) override;

    /// Set stiffness, along direction, es [N/m]
    void SetStiffness(const double mstiffness) { K = mstiffness; }
    double GetStiffness() const { return K; }

    /// Set damping, along direction, es [Ns/m]
    void SetDamping(const double mdamping) { R = mdamping; }
    double GetDamping() const { return R; }

	/// Set initial spring length, es [m]
    void SetRestLength(const double mrest) { d_0 = mrest; }
    double GetRestLength() const { return d_0; }

	/// Use this to enable the stiff force computation (i.e. it enables the
	/// automated computation of the jacobian by numerical differentiation to 
	/// elp the convergence of implicit integrators, but adding CPU overhead).
	void SetStiff(bool ms) { this->is_stiff = ms; }

  protected:
	  double K;
	  double R;
	  double d_0;
	  bool is_stiff;

	  virtual bool IsStiff() override { return is_stiff; }
};


 // Example:
/// Load representing a XYZ bushing between a ChNodeXYZ and a ChBody application point, with given 
/// with spring stiffness as a ChFunction of displacement, for each X,Y,Z direction along the 
/// auxiliary frame at the attachment point. You can set the attachment point via SetApplicationFrameB().
class ChApi ChLoadXYZnodeBodyBushing : public ChLoadXYZnodeBody {
  public:
	  ChLoadXYZnodeBodyBushing   (std::shared_ptr<ChNodeXYZ> mnodeA,  ///< node to apply load 
								  std::shared_ptr<ChBody> mbodyB);   ///< body to apply load 

    /// "Virtual" copy constructor (covariant return type).
    virtual ChLoadXYZnodeBodyBushing* Clone() const override { return new ChLoadXYZnodeBodyBushing(*this); }

	/// Compute the force on the nodeA, in local coordsystem of SetApplicationFrameB (the 
	/// auxiliary frame attached to body) given relative position of nodeA respect to B
    virtual void ComputeForce(const ChFrameMoving<>& rel_AB,
                               ChVector<>& loc_force) override;
	
	/// Set force as a function of displacement on X. Default was constant zero. 
	void SetFunctionForceX(std::shared_ptr<ChFunction> fx) { force_dX = fx; }

	/// Set force as a function of displacement on X. Default was constant zero. 
	void SetFunctionForceY(std::shared_ptr<ChFunction> fy) { force_dY = fy; }
	
	/// Set force as a function of displacement on X. Default was constant zero. 
	void SetFunctionForceZ(std::shared_ptr<ChFunction> fz) { force_dZ = fz; }

	/// Set xyz constant damping coefficients [Ns/m] along the three directions xyz, assuming
	/// local xyz directions of the frame attached to body via SetApplicationFrameB.
    void SetDamping(const ChVector<> mdamping) { R = mdamping; }
    ChVector<> GetDamping() const { return R; }

	/// Use this to enable the stiff force computation (i.e. it enables the
	/// automated computation of the jacobian by numerical differentiation to 
	/// elp the convergence of implicit integrators, but adding CPU overhead).
	void SetStiff(bool ms) { this->is_stiff = ms; }

  protected:
	  std::shared_ptr<ChFunction> force_dX;
	  std::shared_ptr<ChFunction> force_dY;
	  std::shared_ptr<ChFunction> force_dZ;
	  
	  ChVector<> R;

	  bool is_stiff;

	  virtual bool IsStiff() override { return is_stiff; }
};



}  // end namespace chrono

#endif
