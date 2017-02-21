//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//

#ifndef CHCONTACTDVIROLLING_H
#define CHCONTACTDVIROLLING_H


#include "chrono/physics/ChContactDVI.h"
#include "chrono/solver/ChConstraintTwoTuplesRollingN.h"
#include "chrono/solver/ChConstraintTwoTuplesRollingT.h"

namespace chrono {


/// Class for DVI contact between two generic ChContactable objects.
/// It inherits ChContactDVI, that has three reaction forces (N,U,V), but also adds 
/// three rolling reaction torques. 
/// This means that it requires about 2x the memory required by the ChContactDVI.
/// Ta and Tb are of ChContactable sub classes.

template <class Ta, class Tb>  
class ChContactDVIrolling : 
            public ChContactDVI<Ta, Tb> {

  public: 
    typedef typename ChContactTuple<Ta, Tb>::typecarr_a typecarr_a;
    typedef typename ChContactTuple<Ta, Tb>::typecarr_b typecarr_b;

  protected:
    //
    // DATA
    //
 
    // The three scalar constraints, to be feed into the
    // system solver. They contain jacobians data and special functions.
    ChConstraintTwoTuplesRollingN < typecarr_a,  typecarr_b> Rx;
    ChConstraintTwoTuplesRollingT < typecarr_a,  typecarr_b> Ru;
    ChConstraintTwoTuplesRollingT < typecarr_a,  typecarr_b> Rv;

    ChVector<> react_torque;

    float complianceRoll;
    float complianceSpin;

  public:
    //
    // CONSTRUCTORS
    //

    ChContactDVIrolling() {
        Rx.SetRollingConstraintU(&this->Ru);
        Rx.SetRollingConstraintV(&this->Rv);
        Rx.SetNormalConstraint(&this->Nx);
    }

    ChContactDVIrolling(
              ChContactContainerBase* mcontainer,
              Ta* mobjA,  ///< collidable object A
              Tb* mobjB,  ///< collidable object B
              const collision::ChCollisionInfo& cinfo
              ) 
        : ChContactDVI< Ta, Tb >(mcontainer, mobjA, mobjB, cinfo)
    {   
        Rx.SetRollingConstraintU(&this->Ru);
        Rx.SetRollingConstraintV(&this->Rv);
        Rx.SetNormalConstraint(&this->Nx);

        Reset(mobjA, 
              mobjB,
              cinfo);
    }
    
    virtual ~ChContactDVIrolling() {}

    //
    // FUNCTIONS
    //

    /// Initialize again this constraint.
    virtual void Reset(
            Ta* mobjA,  ///< collidable object A
            Tb* mobjB,  ///< collidable object B
            const collision::ChCollisionInfo& cinfo) {
        
        // Base method call:
        ChContactDVI< Ta, Tb >::Reset( mobjA,  mobjB, cinfo);

        Rx.Get_tuple_a().SetVariables(*this->objA);               Rx.Get_tuple_b().SetVariables(*this->objB);
        Ru.Get_tuple_a().SetVariables(*this->objA);               Ru.Get_tuple_b().SetVariables(*this->objB);
        Rv.Get_tuple_a().SetVariables(*this->objA);               Rv.Get_tuple_b().SetVariables(*this->objB);

        // Compute the 'average' material

          // just low level casting, now, since we are sure that this contact was created only if dynamic casting was fine
        ChMaterialSurface* mmatA = (ChMaterialSurface*)(this->objA->GetMaterialSurfaceBase().get());
        ChMaterialSurface* mmatB = (ChMaterialSurface*)(this->objB->GetMaterialSurfaceBase().get());

        ChMaterialCouple mat;
        mat.rolling_friction  = (float)ChMin(mmatA->rolling_friction,  mmatB->rolling_friction);
        mat.spinning_friction = (float)ChMin(mmatA->spinning_friction, mmatB->spinning_friction);
        mat.complianceRoll = (float)(mmatA->complianceRoll + mmatB->complianceRoll);
        mat.complianceSpin = (float)(mmatA->complianceSpin + mmatB->complianceSpin);
        
        Rx.SetRollingFrictionCoefficient(mat.rolling_friction);
        Rx.SetSpinningFrictionCoefficient(mat.spinning_friction);

        this->complianceRoll = mat.complianceRoll;
        this->complianceSpin = mat.complianceSpin;

        // COMPUTE JACOBIANS

        // delegate objA to compute its half of jacobian
        this->objA->ComputeJacobianForRollingContactPart(this->p1, 
                                        this->contact_plane, 
                                        Rx.Get_tuple_a(),
                                        Ru.Get_tuple_a(),
                                        Rv.Get_tuple_a(),
                                        false);

        // delegate objB to compute its half of jacobian
        this->objB->ComputeJacobianForRollingContactPart(this->p2, 
                                        this->contact_plane, 
                                        Rx.Get_tuple_b(),
                                        Ru.Get_tuple_b(),
                                        Rv.Get_tuple_b(),
                                        true);
        
        this->react_torque = VNULL;
    }

 
    /// Get the contact force, if computed, in contact coordinate system
    virtual ChVector<> GetContactTorque() { return react_torque; };

    /// Get the contact rolling friction coefficient
    virtual float GetRollingFriction() { return Rx.GetRollingFrictionCoefficient(); };
    /// Set the contact rolling friction coefficient
    virtual void SetRollingFriction(float mf) { Rx.SetRollingFrictionCoefficient(mf); };

    /// Get the contact spinning friction coefficient
    virtual float GetSpinningFriction() { return Rx.GetSpinningFrictionCoefficient(); };
    /// Set the contact spinning friction coefficient
    virtual void SetSpinningFriction(float mf) { Rx.SetSpinningFrictionCoefficient(mf); };



    //
    // UPDATING FUNCTIONS
    //

    virtual void ContIntStateGatherReactions(const unsigned int off_L, ChVectorDynamic<>& L) {
        // base behaviour too
        ChContactDVI< Ta, Tb >::ContIntStateGatherReactions(off_L, L);

        L(off_L + 3) = react_torque.x();
        L(off_L + 4) = react_torque.y();
        L(off_L + 5) = react_torque.z();
    }

    virtual void ContIntStateScatterReactions(const unsigned int off_L, const ChVectorDynamic<>& L) {
         // base behaviour too
        ChContactDVI< Ta, Tb >::ContIntStateScatterReactions(off_L, L);

        react_torque.x() = L(off_L + 3);
        react_torque.y() = L(off_L + 4);
        react_torque.z() = L(off_L + 5);
    }

    virtual void ContIntLoadResidual_CqL(const unsigned int off_L,    ///< offset in L multipliers
                                 ChVectorDynamic<>& R,        ///< result: the R residual, R += c*Cq'*L
                                 const ChVectorDynamic<>& L,  ///< the L vector
                                 const double c               ///< a scaling factor
                                 ) {
        // base behaviour too
        ChContactDVI< Ta, Tb >::ContIntLoadResidual_CqL(off_L, R, L, c);

        this->Rx.MultiplyTandAdd(R, L(off_L + 3) * c);
        this->Ru.MultiplyTandAdd(R, L(off_L + 4) * c);
        this->Rv.MultiplyTandAdd(R, L(off_L + 5) * c);
    }

    virtual void ContIntLoadConstraint_C(const unsigned int off_L,  ///< offset in Qc residual
                                 ChVectorDynamic<>& Qc,     ///< result: the Qc residual, Qc += c*C
                                 const double c,            ///< a scaling factor
                                 bool do_clamp,             ///< apply clamping to c*C?
                                 double recovery_clamp      ///< value for min/max clamping of c*C
                                 )  {
        // base behaviour too
        ChContactDVI< Ta, Tb >::ContIntLoadConstraint_C(off_L, Qc, c, do_clamp, recovery_clamp);

        // If rolling and spinning compliance, set the cfm terms
        double h = this->container->GetSystem()->GetStep();

        //***TODO*** move to KRMmatricesLoad() the following, and only for !bounced case
        double alpha = this->dampingf;              // [R]=alpha*[K]
        double inv_hpa = 1.0 / (h + alpha);         // 1/(h+a)
        double inv_hhpa = 1.0 / (h * (h + alpha));  // 1/(h*(h+a))

        this->Ru.Set_cfm_i((inv_hhpa) * this->complianceRoll);
        this->Rv.Set_cfm_i((inv_hhpa) * this->complianceRoll);
        this->Rx.Set_cfm_i((inv_hhpa) * this->complianceSpin);
    }

    //virtual void ContIntLoadResidual_F(ChVectorDynamic<>& R, const double c)  {
        // no force to add - this is DVI, not DEM
    //};

    virtual void ContIntToDescriptor(const unsigned int off_L,  ///< offset in L, Qc
                                     const ChVectorDynamic<>& L,
                                     const ChVectorDynamic<>& Qc) {
        // base behaviour too
        ChContactDVI<Ta, Tb>::ContIntToDescriptor(off_L, L, Qc);

        Rx.Set_l_i(L(off_L + 3));
        Ru.Set_l_i(L(off_L + 4));
        Rv.Set_l_i(L(off_L + 5));

        Rx.Set_b_i(Qc(off_L + 3));
        Ru.Set_b_i(Qc(off_L + 4));
        Rv.Set_b_i(Qc(off_L + 5));
    }

    virtual void ContIntFromDescriptor(const unsigned int off_L,  ///< offset in L
                                       ChVectorDynamic<>& L) {
        // base behaviour too
        ChContactDVI<Ta, Tb>::ContIntFromDescriptor(off_L, L);

        L(off_L + 3) = Rx.Get_l_i();
        L(off_L + 4) = Ru.Get_l_i();
        L(off_L + 5) = Rv.Get_l_i();
    }

    virtual void InjectConstraints(ChSystemDescriptor& mdescriptor)  {
        // base behaviour too
        ChContactDVI< Ta, Tb >::InjectConstraints(mdescriptor);

        mdescriptor.InsertConstraint(&Rx);
        mdescriptor.InsertConstraint(&Ru);
        mdescriptor.InsertConstraint(&Rv);
    }

    virtual void ConstraintsBiReset()  {
        // base behaviour too
        ChContactDVI< Ta, Tb >::ConstraintsBiReset();

        Rx.Set_b_i(0.);
        Ru.Set_b_i(0.);
        Rv.Set_b_i(0.);
    }

    virtual void ConstraintsBiLoad_C(double factor = 1., double recovery_clamp = 0.1, bool do_clamp = false)  {
        // base behaviour too
        ChContactDVI< Ta, Tb >::ConstraintsBiLoad_C(factor, recovery_clamp, do_clamp);

        // If rolling and spinning compliance, set the cfm terms
        double h = this->container->GetSystem()->GetStep();

        //***TODO*** move to KRMmatricesLoad() the following, and only for !bounced case
        double alpha = this->dampingf;              // [R]=alpha*[K]
        double inv_hpa = 1.0 / (h + alpha);         // 1/(h+a)
        double inv_hhpa = 1.0 / (h * (h + alpha));  // 1/(h*(h+a))

        this->Ru.Set_cfm_i((inv_hhpa) * this->complianceRoll);
        this->Rv.Set_cfm_i((inv_hhpa) * this->complianceRoll);
        this->Rx.Set_cfm_i((inv_hhpa) * this->complianceSpin);

        // Assume no residual ever, do not load in C
    }

    virtual void ConstraintsFetch_react(double factor)  {
        // base behaviour too
        ChContactDVI< Ta, Tb >::ConstraintsFetch_react(factor);

        // From constraints to react torque:
        react_torque.x() = Rx.Get_l_i() * factor;
        react_torque.y() = Ru.Get_l_i() * factor;
        react_torque.z() = Rv.Get_l_i() * factor;
    }
};

}  // end namespace chrono

#endif
