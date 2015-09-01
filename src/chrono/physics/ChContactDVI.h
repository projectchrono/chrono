//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2010-2011 Alessandro Tasora
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//

#ifndef CHCONTACTDVI_H
#define CHCONTACTDVI_H

#include "core/ChFrame.h"
#include "core/ChVectorDynamic.h"
#include "lcp/ChLcpConstraintTwoTuplesContactN.h"
#include "lcp/ChLcpSystemDescriptor.h"
#include "collision/ChCCollisionModel.h"
#include "physics/ChContactTuple.h"
#include "physics/ChContactContainerBase.h"
#include "physics/ChSystem.h"

namespace chrono {

/// Class for DVI contact between two generic ChContactable objects.
/// Ta and Tb are of ChContactable sub classes.

template <class Ta, class Tb>
class ChContactDVI : public ChContactTuple<Ta, Tb> {
  public:
    typedef typename ChContactTuple<Ta, Tb>::typecarr_a typecarr_a;
    typedef typename ChContactTuple<Ta, Tb>::typecarr_b typecarr_b;

  protected:
    float* reactions_cache;  ///< N,U,V reactions which might be stored in a persistent contact manifold

    /// The three scalar constraints, to be fed into the system solver.
    /// They contain jacobians data and special functions.
    ChLcpConstraintTwoTuplesContactN<typecarr_a, typecarr_b> Nx;
    ChLcpConstraintTwoTuplesFrictionT<typecarr_a, typecarr_b> Tu;
    ChLcpConstraintTwoTuplesFrictionT<typecarr_a, typecarr_b> Tv;

    ChVector<> react_force;

    double compliance;
    double complianceT;
    double restitution;
    double dampingf;

  public:
    //
    // CONSTRUCTORS
    //

    ChContactDVI() {
        Nx.SetTangentialConstraintU(&Tu);
        Nx.SetTangentialConstraintV(&Tv);
    }

    ChContactDVI(ChContactContainerBase* mcontainer,      ///< contact container
                 Ta* mobjA,                               ///< collidable object A
                 Tb* mobjB,                               ///< collidable object B
                 const collision::ChCollisionInfo& cinfo  ///< data for the contact pair
                 )
        : ChContactTuple<Ta, Tb>(mcontainer, mobjA, mobjB, cinfo) {
        Nx.SetTangentialConstraintU(&Tu);
        Nx.SetTangentialConstraintV(&Tv);

        Reset(mobjA, mobjB, cinfo);
    }

    ~ChContactDVI() {}

    //
    // FUNCTIONS
    //

    /// Initialize again this constraint.
    virtual void Reset(Ta* mobjA,                               ///< collidable object A
                       Tb* mobjB,                               ///< collidable object B
                       const collision::ChCollisionInfo& cinfo  ///< data for the contact pair
                       ) {
        // inherit base class:
        ChContactTuple<Ta, Tb>::Reset(mobjA, mobjB, cinfo);

        Nx.Get_tuple_a().SetVariables(*this->objA);
        Nx.Get_tuple_b().SetVariables(*this->objB);
        Tu.Get_tuple_a().SetVariables(*this->objA);
        Tu.Get_tuple_b().SetVariables(*this->objB);
        Tv.Get_tuple_a().SetVariables(*this->objA);
        Tv.Get_tuple_b().SetVariables(*this->objB);

        // Compute the 'average' material

        // just low level casting, now, since we are sure that this contact was created only if dynamic casting was fine
        ChMaterialSurface* mmatA = (ChMaterialSurface*)(this->objA->GetMaterialSurfaceBase().get_ptr());
        ChMaterialSurface* mmatB = (ChMaterialSurface*)(this->objB->GetMaterialSurfaceBase().get_ptr());

        ChMaterialCouple mat;
        mat.static_friction = (float)ChMin(mmatA->static_friction, mmatB->static_friction);
        mat.restitution = (float)ChMin(mmatA->restitution, mmatB->restitution);
        mat.cohesion = (float)ChMin(mmatA->cohesion, mmatB->cohesion);
        mat.dampingf = (float)ChMin(mmatA->dampingf, mmatB->dampingf);
        mat.compliance = (float)(mmatA->compliance + mmatB->compliance);
        mat.complianceT = (float)(mmatA->complianceT + mmatB->complianceT);
        // mat.complianceRoll = (float)(mmatA->complianceRoll + mmatB->complianceRoll);
        // mat.complianceSpin = (float)(mmatA->complianceSpin + mmatB->complianceSpin);
        // mat.rolling_friction = (float)ChMin(mmatA->rolling_friction, mmatB->rolling_friction);
        // mat.spinning_friction = (float)ChMin(mmatA->spinning_friction, mmatB->spinning_friction);

        // see if the user wants to modify the material via a callback:
        if (this->container->GetAddContactCallback()) {
            this->container->GetAddContactCallback()->ContactCallback(cinfo, mat);
        }

        Nx.SetFrictionCoefficient(mat.static_friction);
        Nx.SetCohesion(mat.cohesion);

        this->restitution = mat.restitution;
        this->dampingf = mat.dampingf;
        this->compliance = mat.compliance;
        this->complianceT = mat.complianceT;

        this->reactions_cache = cinfo.reaction_cache;

        // COMPUTE JACOBIANS

        // delegate objA to compute its half of jacobian
        this->objA->ComputeJacobianForContactPart(this->p1, this->contact_plane, Nx.Get_tuple_a(), Tu.Get_tuple_a(),
                                                  Tv.Get_tuple_a(), false);

        // delegate objB to compute its half of jacobian
        this->objB->ComputeJacobianForContactPart(this->p2, this->contact_plane, Nx.Get_tuple_b(), Tu.Get_tuple_b(),
                                                  Tv.Get_tuple_b(), true);

        react_force = VNULL;
    }

    /// Get the contact force, if computed, in contact coordinate system
    virtual ChVector<> GetContactForce() { return react_force; }

    /// Get the contact friction coefficient
    virtual double GetFriction() { return Nx.GetFrictionCoefficient(); }

    /// Set the contact friction coefficient
    virtual void SetFriction(double mf) { Nx.SetFrictionCoefficient(mf); }

    //
    // UPDATING FUNCTIONS
    //

    virtual void ContIntStateGatherReactions(const unsigned int off_L, ChVectorDynamic<>& L) {
        L(off_L) = react_force.x;
        L(off_L + 1) = react_force.y;
        L(off_L + 2) = react_force.z;
    }

    virtual void ContIntStateScatterReactions(const unsigned int off_L, const ChVectorDynamic<>& L) {
        react_force.x = L(off_L);
        react_force.y = L(off_L + 1);
        react_force.z = L(off_L + 2);
    }

    virtual void ContIntLoadResidual_CqL(const unsigned int off_L,    ///< offset in L multipliers
                                         ChVectorDynamic<>& R,        ///< result: the R residual, R += c*Cq'*L
                                         const ChVectorDynamic<>& L,  ///< the L vector
                                         const double c               ///< a scaling factor
                                         ) {
        this->Nx.MultiplyTandAdd(R, L(off_L) * c);
        this->Tu.MultiplyTandAdd(R, L(off_L + 1) * c);
        this->Tv.MultiplyTandAdd(R, L(off_L + 2) * c);
    }

    virtual void ContIntLoadConstraint_C(const unsigned int off_L,  ///< offset in Qc residual
                                         ChVectorDynamic<>& Qc,     ///< result: the Qc residual, Qc += c*C
                                         const double c,            ///< a scaling factor
                                         bool do_clamp,             ///< apply clamping to c*C?
                                         double recovery_clamp      ///< value for min/max clamping of c*C
                                         ) {
        bool bounced = false;

        // Elastic Restitution model (use simple Newton model with coeffcient e=v(+)/v(-))
        // Note that this works only if the two connected items are two ChBody.

        if (this->objA && this->objB) {
            if (this->restitution) {
                // compute normal rebounce speed
                Vector V1_w = this->objA->GetContactPointSpeed(this->p1);
                Vector V2_w = this->objB->GetContactPointSpeed(this->p2);
                Vector Vrel_w = V2_w - V1_w;
                Vector Vrel_cplane = this->contact_plane.MatrT_x_Vect(Vrel_w);

                double h = this->container->GetSystem()->GetStep();  // = 1.0 / c;  // not all steppers have c = 1/h

                double neg_rebounce_speed = Vrel_cplane.x * this->restitution;
                if (neg_rebounce_speed < -this->container->GetSystem()->GetMinBounceSpeed())
                    if (this->norm_dist + neg_rebounce_speed * h < 0) {
                        // CASE: BOUNCE
                        bounced = true;
                        Qc(off_L) += c * neg_rebounce_speed;
                    }
            }
        }

        if (!bounced) {
            // CASE: SETTLE (most often, and also default if two colliding items are not two ChBody)

            if (this->compliance) {
                double h = this->container->GetSystem()->GetStep();  // = 1.0 / c;  // not all steppers have c = 1/h

                double alpha = this->dampingf;              // [R]=alpha*[K]
                double inv_hpa = 1.0 / (h + alpha);         // 1/(h+a)
                double inv_hhpa = 1.0 / (h * (h + alpha));  // 1/(h*(h+a))

                //***TODO*** move to KRMmatricesLoad() the following, and only for !bounced case
                Nx.Set_cfm_i((inv_hhpa) * this->compliance);
                Tu.Set_cfm_i((inv_hhpa) * this->complianceT);
                Tv.Set_cfm_i((inv_hhpa) * this->complianceT);

                Qc(off_L) += c * inv_hpa * this->norm_dist;
            } else {
                if (do_clamp)
                    if (this->Nx.GetCohesion())
                        Qc(off_L) += ChMin(0.0, ChMax(c * this->norm_dist, -recovery_clamp));
                    else
                        Qc(off_L) += ChMax(c * this->norm_dist, -recovery_clamp);
                else
                    Qc(off_L) += c * this->norm_dist;
            }
        }
    }

    virtual void ContIntToLCP(const unsigned int off_L,    ///< offset in L, Qc
                              const ChVectorDynamic<>& L,  ///<
                              const ChVectorDynamic<>& Qc  ///<
                              ) {
        // only for LCP warm start
        Nx.Set_l_i(L(off_L));
        Tu.Set_l_i(L(off_L + 1));
        Tv.Set_l_i(L(off_L + 2));

        // LCP known terms
        Nx.Set_b_i(Qc(off_L));
        Tu.Set_b_i(Qc(off_L + 1));
        Tv.Set_b_i(Qc(off_L + 2));
    }

    virtual void ContIntFromLCP(const unsigned int off_L,  ///< offset in L
                                ChVectorDynamic<>& L       ///<
                                ) {
        L(off_L) = Nx.Get_l_i();
        L(off_L + 1) = Tu.Get_l_i();
        L(off_L + 2) = Tv.Get_l_i();
    }

    virtual void InjectConstraints(ChLcpSystemDescriptor& mdescriptor) {
        mdescriptor.InsertConstraint(&Nx);
        mdescriptor.InsertConstraint(&Tu);
        mdescriptor.InsertConstraint(&Tv);
    }

    virtual void ConstraintsBiReset() {
        Nx.Set_b_i(0.);
        Tu.Set_b_i(0.);
        Tv.Set_b_i(0.);
    }

    virtual void ConstraintsBiLoad_C(double factor = 1., double recovery_clamp = 0.1, bool do_clamp = false) {
        bool bounced = false;

        // Elastic Restitution model (use simple Newton model with coeffcient e=v(+)/v(-))
        // Note that this works only if the two connected items are two ChBody.

        if (this->objA && this->objB) {
            if (this->restitution) {
                // compute normal rebounce speed
                Vector V1_w = this->objA->GetContactPointSpeed(this->p1);
                Vector V2_w = this->objB->GetContactPointSpeed(this->p2);
                Vector Vrel_w = V2_w - V1_w;
                Vector Vrel_cplane = this->contact_plane.MatrT_x_Vect(Vrel_w);

                double h = 1.0 / factor;  // inverse timestep is factor

                double neg_rebounce_speed = Vrel_cplane.x * this->restitution;
                if (neg_rebounce_speed < -this->container->GetSystem()->GetMinBounceSpeed())
                    if (this->norm_dist + neg_rebounce_speed * h < 0) {
                        // CASE: BOUNCE
                        bounced = true;
                        Nx.Set_b_i(Nx.Get_b_i() + neg_rebounce_speed);
                    }
            }
        }

        if (!bounced) {
            // CASE: SETTLE (most often, and also default if two colliding items are not two ChBody)

            if (this->compliance) {
                //  inverse timestep is factor
                double h = 1.0 / factor;

                double alpha = this->dampingf;              // [R]=alpha*[K]
                double inv_hpa = 1.0 / (h + alpha);         // 1/(h+a)
                double inv_hhpa = 1.0 / (h * (h + alpha));  // 1/(h*(h+a))

                Nx.Set_cfm_i((inv_hhpa) * this->compliance);  // was (inv_hh)* ...   //***TEST DAMPING***//
                Tu.Set_cfm_i((inv_hhpa) * this->complianceT);
                Tv.Set_cfm_i((inv_hhpa) * this->complianceT);

                // GetLog()<< "compliance " << (int)this << "  compl=" << this->compliance << "  damping=" <<
                // this->dampingf
                // << "  h=" << h << "\n";

                // no clamping of residual
                Nx.Set_b_i(Nx.Get_b_i() + inv_hpa * this->norm_dist);  // was (inv_h)* ...   //***TEST DAMPING***//
            } else {
                // GetLog()<< "rigid " << (int)this << "  recov_clamp=" << recovery_clamp << "\n";
                if (do_clamp)
                    if (this->Nx.GetCohesion())
                        Nx.Set_b_i(Nx.Get_b_i() + ChMin(0.0, ChMax(factor * this->norm_dist, -recovery_clamp)));
                    else
                        Nx.Set_b_i(Nx.Get_b_i() + ChMax(factor * this->norm_dist, -recovery_clamp));
                else
                    Nx.Set_b_i(Nx.Get_b_i() + factor * this->norm_dist);
            }
        }
    }

    virtual void ConstraintsFetch_react(double factor) {
        // From constraints to react vector:
        react_force.x = Nx.Get_l_i() * factor;
        react_force.y = Tu.Get_l_i() * factor;
        react_force.z = Tv.Get_l_i() * factor;
    }

    virtual void ConstraintsLiLoadSuggestedSpeedSolution() {
        // Fetch the last computed impulsive reactions from the persistent contact manifold (could
        // be used for warm starting the CCP speed solver):
        if (this->reactions_cache) {
            Nx.Set_l_i(reactions_cache[0]);
            Tu.Set_l_i(reactions_cache[1]);
            Tv.Set_l_i(reactions_cache[2]);
        }
        // GetLog() << "++++      " << (int)this << "  fetching N=" << (double)mn <<"\n";
    }

    virtual void ConstraintsLiLoadSuggestedPositionSolution() {
        // Fetch the last computed 'positional' reactions from the persistent contact manifold (could
        // be used for warm starting the CCP position stabilization solver):
        if (this->reactions_cache) {
            Nx.Set_l_i(reactions_cache[3]);
            Tu.Set_l_i(reactions_cache[4]);
            Tv.Set_l_i(reactions_cache[5]);
        }
    }

    virtual void ConstraintsLiFetchSuggestedSpeedSolution() {
        // Store the last computed reactions into the persistent contact manifold (might
        // be used for warm starting CCP the speed solver):
        if (reactions_cache) {
            reactions_cache[0] = (float)Nx.Get_l_i();
            reactions_cache[1] = (float)Tu.Get_l_i();
            reactions_cache[2] = (float)Tv.Get_l_i();
        }
        // GetLog() << "         " << (int)this << "  storing  N=" << Nx.Get_l_i() <<"\n";
    }

    virtual void ConstraintsLiFetchSuggestedPositionSolution() {
        // Store the last computed 'positional' reactions into the persistent contact manifold (might
        // be used for warm starting the CCP position stabilization solver):
        if (reactions_cache) {
            reactions_cache[3] = (float)Nx.Get_l_i();
            reactions_cache[4] = (float)Tu.Get_l_i();
            reactions_cache[5] = (float)Tv.Get_l_i();
        }
    }
};

}  // END_OF_NAMESPACE____

#endif
