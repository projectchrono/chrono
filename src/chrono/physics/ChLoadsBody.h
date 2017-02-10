//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2013 Project Chrono
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//
// File authors: Alessandro Tasora

#ifndef CHLOADSBODY_H
#define CHLOADSBODY_H

#include "chrono/physics/ChLoad.h"
#include "chrono/physics/ChBody.h"


namespace chrono {


// This file contains a number of ready-to-use loads (ChLoad inherited classes and
// their embedded ChLoader classes) that can be applied to objects of ChBody (and 
// inherited classes) type.
// These are 'simplified' tools, that save you from inheriting your custom 
// loads. 
// Or just look at these classes and learn how to implement some special type of load.
//
// Example: 
//    std::shared_ptr<ChBodyEasyBox> body_test(new ChBodyEasyBox(8,4,4,1000));
//    mphysicalSystem.Add(body_test);
//   
//    std::shared_ptr<ChLoadContainer> mforcecontainer (new ChLoadContainer);
//    mphysicalSystem.Add(mforcecontainer);
//
//    std::shared_ptr<ChLoadBodyForce> mforce (new ChLoadBodyForce(body_test, ChVector<>(0,80000,0), false, ChVector<>(8,0,0),true));
//    mforcecontainer->Add(mforce);
//
//    std::shared_ptr<ChLoadBodyTorque> mtorque (new ChLoadBodyTorque(body_test, ChVector<>(0,0,-80000*8), true));
//    mforcecontainer->Add(mtorque);



/// FORCE ON A RIGID BODY
/// Load for a constant force applied at a ChBody.
/// The force can rotate together with the body (if in body local coordinates) or not.
/// The application point can follow the body (if in body local coordinates) or not.

class ChLoadBodyForce : public ChLoadCustom {
private:
    ChVector<> force;
    ChVector<> application;
    bool local_force;
    bool local_application;
public:
    ChLoadBodyForce(std::shared_ptr<ChBody> mbody,      ///< object to apply load to
                    const ChVector<> mforce,        ///< force to apply
                    bool mlocal_force,              ///< force is in body local coords
                    const ChVector<> mapplication,  ///< application point for the force
                    bool mlocal_application = true  ///< application point is in body local coords
                    ) 
        : ChLoadCustom(mbody), 
           force(mforce),
           application(mapplication),
           local_force(mlocal_force),
           local_application(mlocal_application)
        {         
        }

        /// Compute Q, the generalized load. 
        /// Called automatically at each Update().
    virtual void ComputeQ(ChState*      state_x, ///< state position to evaluate Q
                          ChStateDelta* state_w  ///< state speed to evaluate Q
                          )  {
        auto mbody = std::dynamic_pointer_cast<ChBody>(this->loadable);
        double detJ;  // not used btw
        
        ChVector<> abs_force;
        if (this->local_force)
            abs_force = mbody->TransformDirectionLocalToParent(this->force);
        else
            abs_force =this->force;

        ChVector<> abs_application;
        if (this->local_application)
            abs_application = mbody->TransformPointLocalToParent(this->application);
        else
            abs_application =this->application;

        // ChBody assumes F={force_abs, torque_abs}
        ChVectorDynamic<> mF(loadable->Get_field_ncoords());
        mF(0) = abs_force.x();
        mF(1) = abs_force.y();
        mF(2) = abs_force.z();

        // Compute Q = N(u,v,w)'*F
        mbody->ComputeNF(abs_application.x(), abs_application.y(), abs_application.z(), load_Q, detJ, mF, state_x, state_w);
    }

    virtual bool IsStiff() {return false;}


        /// Set force (ex. in [N] units), assumed to be constant.
        /// It can be expressed in absolute coordinates or body local coordinates
    void SetForce(const ChVector<>& mf, const bool is_local) {this->force = mf; this->local_force = is_local;}
    ChVector<> GetForce() const {return this->force;}

        /// Set the application point of force, assumed to be constant.
        /// It can be expressed in absolute coordinates or body local coordinates
    void SetApplicationPoint(const ChVector<>& ma, const bool is_local) {this->application = ma; this->local_application = is_local;}
    ChVector<> GetApplicationPoint() const {return this->application;}
};


//------------------------------------------------------------------------------------------------


/// TORQUE ON A RIGID BODY
/// Loader for a constant torque applied at a ChBody.
/// Torque direction does not rotate with the body. 


class ChLoadBodyTorque : public ChLoadCustom {
private:
    ChVector<> torque;
    bool local_torque;
public:
  ChLoadBodyTorque(std::shared_ptr<ChBody> mbody,  ///< object to apply load to
                   const ChVector<> torque,        ///< torque to apply
                   bool mlocal_torque              ///< torque is in body local coords
                   )
      : ChLoadCustom(mbody), torque(torque), local_torque(mlocal_torque) {}

  /// Compute Q, the generalized load.
  /// Called automatically at each Update().
  virtual void ComputeQ(ChState* state_x,      ///< state position to evaluate Q
                        ChStateDelta* state_w  ///< state speed to evaluate Q
                        ) {
      auto mbody = std::dynamic_pointer_cast<ChBody>(this->loadable);
      double detJ;  // not used btw

      ChVector<> abs_torque;
      if (this->local_torque)
          abs_torque = mbody->TransformDirectionLocalToParent(this->torque);
      else
          abs_torque = this->torque;

      // ChBody assumes F={force_abs, torque_abs}
      ChVectorDynamic<> mF(loadable->Get_field_ncoords());
      mF(3) = this->torque.x();
      mF(4) = this->torque.y();
      mF(5) = this->torque.z();

      // Compute Q = N(u,v,w)'*F
      mbody->ComputeNF(0, 0, 0, load_Q, detJ, mF, state_x, state_w);
    }

    virtual bool IsStiff() {return false;}


        /// Set force (ex. in [N] units), assumed to be constant in space and time
    void SetTorque(const ChVector<>& mf) {this->torque = mf;}
    ChVector<> GetTorque() const {return this->torque;}
};



//------------------------------------------------------------------------------------------------

/*
/// BUSHING BETWEEN RIGID BODIES
/// Load for a spherical flexible bushing connection between two bodies.

class ChLoadBodyBushing : public ChLoadCustomMultiple {
private:
    double radial_stiffness;
    double radial_damping;
    ChVector<> loc_application_A;
    ChVector<> loc_application_B;
public:
    ChLoadBodyBushing(std::shared_ptr<ChBody> mbodyA,       ///< object A
                      std::shared_ptr<ChBody> mbodyB,       ///< object B
                      const ChVector<> abs_application, ///< create the bushing here, in abs. coordinates
                      const double mstiffness,          ///< radial stiffness
                      const double mdamping             ///< radial damping
                    ) 
         : ChLoadCustomMultiple(mbodyA,mbodyB), 
           radial_stiffness(mstiffness),
           radial_damping(mdamping) {         
            
            loc_application_A = mbodyA->TransformPointParentToLocal(abs_application);
            loc_application_B = mbodyB->TransformPointParentToLocal(abs_application);
        }

        /// Compute Q, the generalized load. 
        /// Called automatically at each Update().
    virtual void ComputeQ(ChState*      state_x, ///< state position to evaluate Q
                          ChStateDelta* state_w  ///< state speed to evaluate Q
                          )  {
        auto mbodyA = std::dynamic_pointer_cast<ChBody>(this->loadables[0]);
        auto mbodyB = std::dynamic_pointer_cast<ChBody>(this->loadables[1]);
    
        ChCoordsys<> bodycoordA, bodycoordB;
        if(state_x) {
            bodycoordA = state_x->ClipCoordsys(0,0); // the numerical jacobian algo might change state_x
            bodycoordB = state_x->ClipCoordsys(7,0); // the numerical jacobian algo might change state_x
        }
        else  {
            bodycoordA = mbodyA->coord;
            bodycoordB = mbodyB->coord;
        }
        ChVector<> abs_d = bodycoordB.pos -  bodycoordA.pos;
        ChVector<> abs_force = abs_d * this->radial_stiffness;

        // Compute Q 

        ChVector<> abs_applicationA = bodycoordA.TransformPointLocalToParent(loc_application_A);
        ChVector<> loc_torque = bodycoordA.rot.RotateBack(  ((abs_applicationA-bodycoordA.pos) %  abs_force) );
        this->load_Q.PasteVector(abs_force, 0,0);
        this->load_Q.PasteVector(loc_torque,3,0);

        ChVector<> abs_applicationB = bodycoordB.TransformPointLocalToParent(loc_application_B);
                   loc_torque = bodycoordB.rot.RotateBack(  ((abs_applicationB-bodycoordB.pos) % -abs_force) );
        this->load_Q.PasteVector(-abs_force, 6,0);
        this->load_Q.PasteVector( loc_torque,9,0);
    }

    virtual bool IsStiff() {return true;}


        /// Set radial stiffness, es [Ns/m]
    void SetRadialStiffness(const double mstiffness) {this->radial_stiffness = mstiffness;}
    double GetRadialStiffness() const {return this->radial_stiffness;}

        /// Set radial damping, es [N/m]
    void SetRadialDamping(const double mdamping) {this->radial_damping = mdamping;}
    double GetRadialDamping() const {return this->radial_damping;}

        /// Set the application point of bushing on bodyA
    void SetApplicationPointA(const ChVector<> mpA) {this->loc_application_A = mpA;}
    ChVector<> GetApplicationPointA() const {return this->loc_application_A;}

        /// Set the application point of bushing on bodyB
    void SetApplicationPointB(const ChVector<> mpB) {this->loc_application_B = mpB;}
    ChVector<> GetApplicationPointB() const {return this->loc_application_B;}
};


*/

}  // end namespace chrono

#endif
