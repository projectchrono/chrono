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


/// Bushing base class.
/// Base class for wrench loads (a force + a torque) acting between two bodies.
/// See children classes for concrete implementations.

class ChLoadBodyBody : public ChLoadCustomMultiple {
protected:
    ChFrame<>  loc_application_A;
    ChFrame<>  loc_application_B;
    ChVector<> locB_force;  // store computed values here 
    ChVector<> locB_torque; // store computed values here 
    ChFrameMoving<> frame_Aw; // for results
    ChFrameMoving<> frame_Bw; // for results
public:
    ChLoadBodyBody( std::shared_ptr<ChBody> mbodyA,   ///< object A
                    std::shared_ptr<ChBody> mbodyB,   ///< object B
                    const ChFrame<> abs_application   ///< create the bushing here, in abs. coordinates.
                        ) 
         : ChLoadCustomMultiple(mbodyA,mbodyB)  {         
            
            mbodyA->ChFrame::TransformParentToLocal(abs_application,loc_application_A);
            mbodyB->ChFrame::TransformParentToLocal(abs_application,loc_application_B);
        }

        /// Compute the force between the two bodies, in local reference loc_application_B,
        /// given rel_AB, i.e. the position and speed of loc_application_A respect to loc_application_B.
        /// Force is assumed applied to body B, and its opposite to A.
        /// Inherited classes MUST IMPLEMENT THIS.
    virtual void ComputeBushingForceTorque(const ChFrameMoving<>& rel_AB, 
                                            ChVector<>& loc_force,
                                            ChVector<>& loc_torque) 
                                            = 0;

        // Optional: inherited classes could implement this to avoid the
        // default numerical computation of jacobians:
        //   virtual void ComputeJacobian(...) // see ChLoad

        /// For diagnosis purposes, this can return the actual last computed value of 
        /// the applied force, expressed in coordinate system of loc_application_B, assumed applied to body B
    ChVector<> GetBushingForce() {return locB_force;}
        /// For diagnosis purposes, this can return the actual last computed value of 
        /// the applied torque, expressed in coordinate system of loc_application_B, assumed applied to body B
    ChVector<> GetBushingTorque() {return locB_torque;}

        /// Set the application frame of bushing on bodyA
    void SetApplicationFrameA(const ChFrame<>& mpA) {this->loc_application_A = mpA;}
    ChFrame<> GetApplicationFrameA() const {return this->loc_application_A;}

        /// Set the application frame of bushing on bodyB
    void SetApplicationFrameB(const ChFrame<>& mpB) {this->loc_application_B = mpB;}
    ChFrame<> GetApplicationFrameB() const {return this->loc_application_B;}

        /// Get absolute coordinate of frame A (last computed) 
    ChFrameMoving<> GetAbsoluteFrameA() const {return this->frame_Aw;}

        /// Get absolute coordinate of frame B (last computed)
    ChFrameMoving<> GetAbsoluteFrameB() const {return this->frame_Bw;}

    std::shared_ptr<ChBody> GetBodyA() const {return std::dynamic_pointer_cast<ChBody> (this->loadables[0]); }
    std::shared_ptr<ChBody> GetBodyB() const {return std::dynamic_pointer_cast<ChBody> (this->loadables[1]); }

        /// Compute Q, the generalized load. It calls ComputeBushingForceTorque, so in
        /// children classes you do not need to implement it.
        /// Called automatically at each Update().
    virtual void ComputeQ(ChState*      state_x, ///< state position to evaluate Q
                          ChStateDelta* state_w  ///< state speed to evaluate Q
                          )  {
        auto mbodyA = std::dynamic_pointer_cast<ChBody>(this->loadables[0]);
        auto mbodyB = std::dynamic_pointer_cast<ChBody>(this->loadables[1]);

        ChFrameMoving<> bodycoordA, bodycoordB;
        if(state_x) {
            // the numerical jacobian algo might change state_x
            bodycoordA.SetCoord(state_x->ClipCoordsys(0,0)); 
            bodycoordB.SetCoord(state_x->ClipCoordsys(7,0));
        }
        else  {
            bodycoordA.SetCoord(mbodyA->coord);
            bodycoordB.SetCoord(mbodyB->coord);
        }
        if(state_w) {
             // the numerical jacobian algo might change state_w
            bodycoordA.SetPos_dt  (state_w->ClipVector(0,0));
            bodycoordA.SetWvel_loc(state_w->ClipVector(3,0));
            bodycoordB.SetPos_dt  (state_w->ClipVector(6,0));
            bodycoordB.SetWvel_loc(state_w->ClipVector(9,0));
        }
        else  {
            bodycoordA.SetCoord_dt(mbodyA->GetCoord_dt());
            bodycoordB.SetCoord_dt(mbodyB->GetCoord_dt());
        }
        
        frame_Aw = ChFrameMoving<>(loc_application_A) >> bodycoordA;
        frame_Bw = ChFrameMoving<>(loc_application_B) >> bodycoordB;
        ChFrameMoving<> rel_AB = frame_Aw >> frame_Bw.GetInverse();
        
        // ----COMPUTE THE BUSHING FORCE

        this->ComputeBushingForceTorque(rel_AB, locB_force, locB_torque);

        // ----
        
        ChVector<> abs_force  = frame_Bw.TransformDirectionLocalToParent(locB_force);
        ChVector<> abs_torque = frame_Bw.TransformDirectionLocalToParent(locB_torque);

        // Compute Q 

        ChVector<> loc_ftorque = bodycoordA.GetRot().RotateBack(  ((frame_Aw.GetPos()-bodycoordA.GetPos()) %  -abs_force) );
        ChVector<> loc_torque  = bodycoordA.GetRot().RotateBack(-abs_torque);
        this->load_Q.PasteVector(-abs_force, 0,0);
        this->load_Q.PasteVector(loc_ftorque+loc_torque,3,0);

                   loc_ftorque = bodycoordB.GetRot().RotateBack(  ((frame_Bw.GetPos()-bodycoordB.GetPos()) %  abs_force) );
                   loc_torque  = bodycoordB.GetRot().RotateBack( abs_torque);
        this->load_Q.PasteVector( abs_force, 6,0);
        this->load_Q.PasteVector( loc_ftorque+loc_torque,9,0);
        
    }


};



//------------------------------------------------------------------------------------------------


/// Load for a visco-elastic bushing acting between two bodies.
/// It uses three values for stiffness along the X Y Z axes of a coordinate system attached 
/// to the second body. This is equivalent to having a bushing with 3x3 diagonal local stiffness matrix.

class ChLoadBodyBodyBushingSpherical : public ChLoadBodyBody {
protected:
    ChVector<> stiffness;
    ChVector<> damping;
   
public:
    ChLoadBodyBodyBushingSpherical(
                          std::shared_ptr<ChBody> mbodyA,   ///< object A
                          std::shared_ptr<ChBody> mbodyB,   ///< object B
                          const ChFrame<> abs_application,  ///< create the bushing here, in abs. coordinates. Initial alignment as world xyz.
                          const ChVector<> mstiffness,      ///< stiffness, along x y z axes of the abs_application
                          const ChVector<> mdamping         ///< damping, along x y z axes of the abs_application
                        ) 
         : ChLoadBodyBody(mbodyA,mbodyB,abs_application), 
           stiffness(mstiffness),
           damping(mdamping) {         
        }

        /// Implement the computation of bushing force, in local 
        /// coordinates of the loc_application_B.
        /// Force is assumed applied to body B, and its opposite to A.
    virtual void ComputeBushingForceTorque(const ChFrameMoving<>& rel_AB, 
                                            ChVector<>& loc_force,
                                            ChVector<>& loc_torque)  override {

        loc_force  = rel_AB.GetPos()    * this->stiffness  // element-wise product!
                   + rel_AB.GetPos_dt() * this->damping;   // element-wise product!
        loc_torque = VNULL;
    }

    virtual bool IsStiff() {return true;}


        /// Set stiffness, along the x y z axes of loc_application_B, es [N/m]
    void SetStiffness(const ChVector<> mstiffness) {this->stiffness = mstiffness;}
    ChVector<> GetStiffness() const {return this->stiffness;}

        /// Set damping, along the x y z axes of loc_application_B, es [Ns/m]
    void SetDamping(const ChVector<> mdamping) {this->damping = mdamping;}
    ChVector<> GetDamping() const {return this->damping;}

};


//------------------------------------------------------------------------------------------------


/// Load for a visco-elasto-plastic bushing acting between two bodies.
/// It uses three values for stiffness along the X Y Z axes of a coordinate system attached 
/// to the second body. This is equivalent to having a bushing with 3x3 diagonal local stiffness matrix.
/// Also, it allows a very simple plasticity model, to cap the plastic force on x,y,z given three yelds.

class ChLoadBodyBodyBushingPlastic : public ChLoadBodyBodyBushingSpherical {
protected:
    ChVector<> yeld;
    ChVector<> plastic_def;
   
public:
    ChLoadBodyBodyBushingPlastic(
                          std::shared_ptr<ChBody> mbodyA,   ///< object A
                          std::shared_ptr<ChBody> mbodyB,   ///< object B
                          const ChFrame<> abs_application,  ///< create the bushing here, in abs. coordinates. Initial alignment as world xyz.
                          const ChVector<> mstiffness,      ///< stiffness, along x y z axes of the abs_application
                          const ChVector<> mdamping,        ///< damping, along x y z axes of the abs_application
                          const ChVector<> myeld            ///< plastic yeld, along x y z axes of the abs_application
                        ) 
         : ChLoadBodyBodyBushingSpherical(mbodyA,mbodyB,abs_application, mstiffness, mdamping), 
           yeld(myeld),
           plastic_def(VNULL) {         
        }

        /// Implement the computation of bushing force, in local 
        /// coordinates of the loc_application_B.
        /// Force is assumed applied to body B, and its opposite to A.
    virtual void ComputeBushingForceTorque(const ChFrameMoving<>& rel_AB, 
                                            ChVector<>& loc_force,
                                            ChVector<>& loc_torque)  override {

        loc_force  = (rel_AB.GetPos()-plastic_def)  * this->stiffness  // element-wise product!
                   + rel_AB.GetPos_dt() * this->damping;   // element-wise product!

        // A basic plasticity, assumed with box capping, without hardening:
        
        if (loc_force.x() > yeld.x()) {
            loc_force.x() = yeld.x();
            plastic_def.x() = rel_AB.GetPos().x() - loc_force.x() / this->stiffness.x();
        }
        if (loc_force.x() < -yeld.x()) {
            loc_force.x() = -yeld.x();
            plastic_def.x() = rel_AB.GetPos().x() - loc_force.x() / this->stiffness.x();
        }
        if (loc_force.y() > yeld.y()) {
            loc_force.y() = yeld.y();
            plastic_def.y() = rel_AB.GetPos().y() - loc_force.y() / this->stiffness.y();
        }
        if (loc_force.y() < -yeld.y()) {
            loc_force.y() = -yeld.y();
            plastic_def.y() = rel_AB.GetPos().y() - loc_force.y() / this->stiffness.y();
        }
        if (loc_force.z() > yeld.z()) {
            loc_force.z() = yeld.z();
            plastic_def.z() = rel_AB.GetPos().z() - loc_force.z() / this->stiffness.z();
        }
        if (loc_force.z() < -yeld.z()) {
            loc_force.z() = -yeld.z();
            plastic_def.z() = rel_AB.GetPos().z() - loc_force.z() / this->stiffness.z();
        }
        
        // GetLog() << "loc_force" << loc_force << "\n";
        // GetLog() << "plastic_def" << plastic_def << "\n";
        loc_torque = VNULL;
    }

    virtual bool IsStiff() {return true;}


        /// Set plastic yeld, forces beyond this limit will be capped. 
        /// Expressed along the x y z axes of loc_application_B, es [N/m].
    void SetYeld(const ChVector<> myeld) {this->yeld = myeld;}
    ChVector<> GetYeld() const {return this->yeld;}

        /// Get the current accumulated plastic deformation, in [m], that
        /// could become nonzero if forces went beyond the plastic yeld.
    ChVector<> GetPlasticDeformation() const {return this->plastic_def;}

};




//------------------------------------------------------------------------------------------------


/// Load for a visco-elastic translational/rotational bushing acting between two bodies.
/// It uses three values for stiffness along the X Y Z axes of a coordinate system attached 
/// to the second body , and three rotational stiffness values for (small) rotations about X Y Z of the
/// same coordinate system. 
/// This is equivalent to having a bushing with 6x6 diagonal local stiffness matrix.

class ChLoadBodyBodyBushingMate : public ChLoadBodyBodyBushingSpherical {
protected:
    ChVector<> rot_stiffness;
    ChVector<> rot_damping;
   
public:
    ChLoadBodyBodyBushingMate(
                          std::shared_ptr<ChBody> mbodyA,   ///< object A
                          std::shared_ptr<ChBody> mbodyB,   ///< object B
                          const ChFrame<> abs_application,  ///< create the bushing here, in abs. coordinates. Initial alignment as world xyz.
                          const ChVector<> mstiffness,      ///< stiffness, along x y z axes of the abs_application
                          const ChVector<> mdamping,        ///< damping, along x y z axes of the abs_application
                          const ChVector<> mrotstiffness,   ///< rotational stiffness, about x y z axes of the abs_application
                          const ChVector<> mrotdamping      ///< rotational damping, about x y z axes of the abs_application
                        ) 
         : ChLoadBodyBodyBushingSpherical(mbodyA,mbodyB,abs_application, mstiffness, mdamping), 
           rot_stiffness(mrotstiffness),
           rot_damping(mrotdamping) {         
        }

        /// Implement the computation of bushing force, in local 
        /// coordinates of the loc_application_B.
        /// Force is assumed applied to body B, and its opposite to A.
    virtual void ComputeBushingForceTorque(const ChFrameMoving<>& rel_AB, 
                                            ChVector<>& loc_force,
                                            ChVector<>& loc_torque)  override {
        // inherit parent to compute loc_force = ...
        ChLoadBodyBodyBushingSpherical::ComputeBushingForceTorque(rel_AB, loc_force, loc_torque);

        // compute local torque using small rotations:
        ChQuaternion<> rel_rot = rel_AB.GetRot();
        ChVector<> vect_rot = rel_rot.Q_to_Rotv();
        loc_torque  = vect_rot  * this->rot_stiffness  // element-wise product!
                    + rel_AB.GetWvel_par() * this->damping;   // element-wise product!
    }

    virtual bool IsStiff() {return true;}


        /// Set radial stiffness, along the x y z axes of loc_application_B, es [N/m]
    void SetRotationalStiffness(const ChVector<> mstiffness) {this->rot_stiffness = mstiffness;}
    ChVector<> GetRotationalStiffness() const {return this->rot_stiffness;}

        /// Set radial damping, along the x y z axes of loc_application_B, es [Ns/m]
    void SetRotationalDamping(const ChVector<> mdamping) {this->rot_damping = mdamping;}
    ChVector<> GetRotationalDamping() const {return this->rot_damping;}

};


//------------------------------------------------------------------------------------------------


/// Load for a visco-elastic translational/rotational bushing acting between two bodies.
/// It uses a full user-defined 6x6 matrix [K] to express the local stiffness of the
/// bushing, assumed expressed in the bushing coordinate system  attached 
/// to the second body. A user-defined 6x6 matrix [D] can be defined for damping, as well. 
/// Note that this assumes small rotations.
/// Differently from the simplier ChLoadBodyBodyBushingMate and ChLoadBodyBodyBushingSpherical
/// this can represent coupled effects, by using extra-diagonal terms in [K] and/or [D].

class ChLoadBodyBodyBushingGeneric : public ChLoadBodyBody {
protected:
    ChMatrixNM<double,6,6> stiffness;
    ChMatrixNM<double,6,6> damping;
   
public:
    ChLoadBodyBodyBushingGeneric(
                          std::shared_ptr<ChBody> mbodyA,   ///< object A
                          std::shared_ptr<ChBody> mbodyB,   ///< object B
                          const ChFrame<> abs_application,  ///< create the bushing here, in abs. coordinates. Initial alignment as world xyz.
                          const ChMatrix<>& mstiffness,     ///< stiffness as a 6x6 matrix, local in the abs_application frame
                          const ChMatrix<>& mdamping        ///< damping as a 6x6 matrix, local in the abs_application frame
                        ) 
         : ChLoadBodyBody(mbodyA,mbodyB,abs_application), 
           stiffness(mstiffness),
           damping(mdamping) {         
        }

        /// Implement the computation of bushing force, in local 
        /// coordinates of the loc_application_B.
        /// Force is assumed applied to body B, and its opposite to A.
    virtual void ComputeBushingForceTorque(const ChFrameMoving<>& rel_AB, 
                                            ChVector<>& loc_force,
                                            ChVector<>& loc_torque)  override {
        
        // compute local force & torque (assuming small rotations):
        ChVectorDynamic<> mF(6);
        ChVectorDynamic<> mS(6);
        ChVectorDynamic<> mSdt(6);
        ChQuaternion<> rel_rot = rel_AB.GetRot();
        mS.PasteVector(rel_AB.GetPos(), 0,0);
        mS.PasteVector(rel_rot.Q_to_Rotv(), 3,0);
        mSdt.PasteVector(rel_AB.GetPos_dt(), 0,0);
        mSdt.PasteVector(rel_AB.GetWvel_par(), 3,0);
        
        mF = stiffness * mS + damping * mSdt;
        
        loc_force   = mF.ClipVector(0,0);
        loc_torque  = mF.ClipVector(3,0);
    }

    virtual bool IsStiff() {return true;}


        /// Set a generic 6x6 stiffness matrix, expressed in local 
        /// coordinate system of loc_application_B.
    void SetStiffnessMatrix(const ChMatrix<>& mstiffness) {this->stiffness = mstiffness;}
    const ChMatrix<>& GetStiffnessMatrix() const {return this->stiffness;}

        /// Set a generic 6x6 damping matrix, expressed in local 
        /// coordinate system of loc_application_B.
    void SetDampingMatrix(const ChMatrix<>& mdamping) {this->damping = mdamping;}
    const ChMatrix<>& GetDampingMatrix() const {return this->damping;}

};





}  // end namespace chrono

#endif
