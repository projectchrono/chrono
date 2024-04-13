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

#ifndef CH_CONTACT_NSC_H
#define CH_CONTACT_NSC_H

#include "chrono/core/ChFrame.h"
#include "chrono/solver/ChConstraintTwoTuplesContactN.h"
#include "chrono/solver/ChSystemDescriptor.h"
#include "chrono/collision/ChCollisionModel.h"
#include "chrono/physics/ChContactTuple.h"
#include "chrono/physics/ChContactContainer.h"
#include "chrono/physics/ChSystem.h"

namespace chrono {

/// Class for non-smooth contact between two generic ChContactable objects.
/// Ta and Tb are of ChContactable sub classes.

template <class Ta, class Tb>
class ChContactNSC : public ChContactTuple<Ta, Tb> {
  public:
    typedef typename ChContactTuple<Ta, Tb>::typecarr_a typecarr_a;
    typedef typename ChContactTuple<Ta, Tb>::typecarr_b typecarr_b;

  protected:
    float* reactions_cache;  ///< N,U,V reactions which might be stored in a persistent contact manifold

    /// The three scalar constraints, to be fed into the system solver.
    /// They contain jacobians data and special functions.
    ChConstraintTwoTuplesContactN<typecarr_a, typecarr_b> Nx;
    ChConstraintTwoTuplesFrictionT<typecarr_a, typecarr_b> Tu;
    ChConstraintTwoTuplesFrictionT<typecarr_a, typecarr_b> Tv;

    ChVector3d react_force;

    double compliance;
    double complianceT;
    double restitution;
    double dampingf;

    double min_rebounce_speed;

  public:
    ChContactNSC() {
        Nx.SetTangentialConstraintU(&Tu);
        Nx.SetTangentialConstraintV(&Tv);
    }

    ChContactNSC(ChContactContainer* contact_container,     ///< contact container
                 Ta* obj_A,                                 ///< contactable object A
                 Tb* obj_B,                                 ///< contactable object B
                 const ChCollisionInfo& cinfo,              ///< data for the collision pair
                 const ChContactMaterialCompositeNSC& mat,  ///< composite material
                 double min_speed                           ///< minimum speed for rebounce
                 )
        : ChContactTuple<Ta, Tb>(contact_container, obj_A, obj_B) {
        Nx.SetTangentialConstraintU(&Tu);
        Nx.SetTangentialConstraintV(&Tv);

        Reset(obj_A, obj_B, cinfo, mat, min_speed);
    }

    ~ChContactNSC() {}

    /// Reinitialize this contact for reuse.
    virtual void Reset(Ta* obj_A,                                 ///< contactable object A
                       Tb* obj_B,                                 ///< contactable object B
                       const ChCollisionInfo& cinfo,              ///< data for the collision pair
                       const ChContactMaterialCompositeNSC& mat,  ///< composite material
                       double min_speed                           ///< minimum speed for rebounce
    ) {
        // Reset geometric information
        this->Reset_cinfo(obj_A, obj_B, cinfo);

        Nx.Get_tuple_a().SetVariables(*this->objA);
        Nx.Get_tuple_b().SetVariables(*this->objB);
        Tu.Get_tuple_a().SetVariables(*this->objA);
        Tu.Get_tuple_b().SetVariables(*this->objB);
        Tv.Get_tuple_a().SetVariables(*this->objA);
        Tv.Get_tuple_b().SetVariables(*this->objB);

        // Cache composite material properties
        Nx.SetFrictionCoefficient(mat.static_friction);
        Nx.SetCohesion(mat.cohesion);

        this->restitution = mat.restitution;
        this->dampingf = mat.dampingf;
        this->compliance = mat.compliance;
        this->complianceT = mat.complianceT;

        this->reactions_cache = cinfo.reaction_cache;

        this->min_rebounce_speed = min_speed;

        // COMPUTE JACOBIANS

        // delegate objA to compute its half of jacobian
        this->objA->ComputeJacobianForContactPart(this->p1, this->contact_plane, Nx.Get_tuple_a(), Tu.Get_tuple_a(),
                                                  Tv.Get_tuple_a(), false);

        // delegate objB to compute its half of jacobian
        this->objB->ComputeJacobianForContactPart(this->p2, this->contact_plane, Nx.Get_tuple_b(), Tu.Get_tuple_b(),
                                                  Tv.Get_tuple_b(), true);

        if (reactions_cache) {
            react_force.x() = reactions_cache[0];
            react_force.y() = reactions_cache[1];
            react_force.z() = reactions_cache[2];
            // std::cout << "Reset Fn=" << (double)reactions_cache[0] << "  at cache address:" <<
            // (int)this->reactions_cache << std::endl;
        } else {
            react_force = VNULL;
        }
    }

    /// Get the contact force, if computed, in contact coordinate system
    virtual ChVector3d GetContactForce() const override { return react_force; }

    /// Get the contact friction coefficient
    virtual double GetFriction() { return Nx.GetFrictionCoefficient(); }

    /// Set the contact friction coefficient
    virtual void SetFriction(double mf) { Nx.SetFrictionCoefficient(mf); }

    /// Access the constraints
    ChConstraint* GetConstraintNx() { return &Nx; }
    ChConstraint* GetConstraintTu() { return &Tu; }
    ChConstraint* GetConstraintTv() { return &Tv; }

    // UPDATING FUNCTIONS

    virtual void ContIntStateGatherReactions(const unsigned int off_L, ChVectorDynamic<>& L) override {
        L(off_L) = react_force.x();
        L(off_L + 1) = react_force.y();
        L(off_L + 2) = react_force.z();
    }

    virtual void ContIntStateScatterReactions(const unsigned int off_L, const ChVectorDynamic<>& L) override {
        react_force.x() = L(off_L);
        react_force.y() = L(off_L + 1);
        react_force.z() = L(off_L + 2);

        if (reactions_cache) {
            reactions_cache[0] = (float)L(off_L);      // react_force.x();
            reactions_cache[1] = (float)L(off_L + 1);  // react_force.y();
            reactions_cache[2] = (float)L(off_L + 2);  // react_force.z();
        }
    }

    virtual void ContIntLoadResidual_CqL(const unsigned int off_L,
                                         ChVectorDynamic<>& R,
                                         const ChVectorDynamic<>& L,
                                         const double c) override {
        this->Nx.AddJacobianTransposedTimesScalarInto(R, L(off_L) * c);
        this->Tu.AddJacobianTransposedTimesScalarInto(R, L(off_L + 1) * c);
        this->Tv.AddJacobianTransposedTimesScalarInto(R, L(off_L + 2) * c);
    }

    virtual void ContIntLoadConstraint_C(const unsigned int off_L,
                                         ChVectorDynamic<>& Qc,
                                         const double c,
                                         bool do_clamp,
                                         double recovery_clamp) override {
        bool bounced = false;

        // Elastic Restitution model (use simple Newton model with coefficient e=v(+)/v(-))
        // Note that this works only if the two connected items are two ChBody.

        if (this->objA && this->objB) {
            if (this->restitution) {
                // compute normal rebounce speed
                ChVector3d V1_w = this->objA->GetContactPointSpeed(this->p1);
                ChVector3d V2_w = this->objB->GetContactPointSpeed(this->p2);
                ChVector3d Vrel_w = V2_w - V1_w;
                ChVector3d Vrel_cplane = this->contact_plane.transpose() * Vrel_w;

                double h = this->container->GetSystem()->GetStep();  // = 1.0 / c;  // not all steppers have c = 1/h

                double neg_rebounce_speed = Vrel_cplane.x() * this->restitution;
                if (neg_rebounce_speed < -min_rebounce_speed)
                    if (this->norm_dist + neg_rebounce_speed * h < 0) {
                        // CASE: BOUNCE
                        bounced = true;
                        Qc(off_L) += neg_rebounce_speed;
                    }
            }
        }

        if (!bounced) {
            // CASE: SETTLE (most often, and also default if two colliding items are not two ChBody)

            if (this->compliance) {
                double h = 1.0 / c;  // was: this->container->GetSystem()->GetStep(); note not all steppers have c = 1/h

                double alpha = this->dampingf;              // [R]=alpha*[K]
                double inv_hpa = 1.0 / (h + alpha);         // 1/(h+a)
                double inv_hhpa = 1.0 / (h * (h + alpha));  // 1/(h*(h+a))

                //// TODO  move to LoadKRMMatrices() the following, and only for !bounced case
                Nx.SetComplianceTerm((inv_hhpa) * this->compliance);
                Tu.SetComplianceTerm((inv_hhpa) * this->complianceT);
                Tv.SetComplianceTerm((inv_hhpa) * this->complianceT);

                double qc = inv_hpa * this->norm_dist;  //// TODO  see how to move this in LoadKRMMatrices()

                // Note: clamping of Qc in case of compliance is questionable: it does not limit only the outbound
                // speed, but also the reaction, so it might allow longer 'sinking' not related to the real compliance.
                // I.e. If clamping kicks in (when using large timesteps and low compliance), it acts as a numerical
                // damping.
                if (do_clamp) {
                    qc = std::max(qc, -recovery_clamp);
                }

                Qc(off_L) += qc;

            } else {
                if (do_clamp)
                    if (this->Nx.GetCohesion())
                        Qc(off_L) += std::min(0.0, std::max(c * this->norm_dist, -recovery_clamp));
                    else
                        Qc(off_L) += std::max(c * this->norm_dist, -recovery_clamp);
                else
                    Qc(off_L) += c * this->norm_dist;
            }
        }
    }

    virtual void ContIntToDescriptor(const unsigned int off_L,
                                     const ChVectorDynamic<>& L,
                                     const ChVectorDynamic<>& Qc) override {
        // only for solver warm start
        Nx.SetLagrangeMultiplier(L(off_L));
        Tu.SetLagrangeMultiplier(L(off_L + 1));
        Tv.SetLagrangeMultiplier(L(off_L + 2));

        // solver known terms
        Nx.SetRightHandSide(Qc(off_L));
        Tu.SetRightHandSide(Qc(off_L + 1));
        Tv.SetRightHandSide(Qc(off_L + 2));
    }

    virtual void ContIntFromDescriptor(const unsigned int off_L, ChVectorDynamic<>& L) override {
        L(off_L) = Nx.GetLagrangeMultiplier();
        L(off_L + 1) = Tu.GetLagrangeMultiplier();
        L(off_L + 2) = Tv.GetLagrangeMultiplier();
    }

    virtual void InjectConstraints(ChSystemDescriptor& descriptor) override {
        descriptor.InsertConstraint(&Nx);
        descriptor.InsertConstraint(&Tu);
        descriptor.InsertConstraint(&Tv);
    }

    virtual void ConstraintsBiReset() override {
        Nx.SetRightHandSide(0.);
        Tu.SetRightHandSide(0.);
        Tv.SetRightHandSide(0.);
    }

    virtual void ConstraintsBiLoad_C(double factor = 1., double recovery_clamp = 0.1, bool do_clamp = false) override {
        bool bounced = false;

        // Elastic Restitution model (use simple Newton model with coefficient e=v(+)/v(-))
        // Note that this works only if the two connected items are two ChBody.

        if (this->objA && this->objB) {
            if (this->restitution) {
                // compute normal rebounce speed
                ChVector3d V1_w = this->objA->GetContactPointSpeed(this->p1);
                ChVector3d V2_w = this->objB->GetContactPointSpeed(this->p2);
                ChVector3d Vrel_w = V2_w - V1_w;
                ChVector3d Vrel_cplane = this->contact_plane.transpose() * Vrel_w;

                double h = 1.0 / factor;  // inverse timestep is factor

                double neg_rebounce_speed = Vrel_cplane.x() * this->restitution;
                if (neg_rebounce_speed < -min_rebounce_speed)
                    if (this->norm_dist + neg_rebounce_speed * h < 0) {
                        // CASE: BOUNCE
                        bounced = true;
                        Nx.SetRightHandSide(Nx.GetRightHandSide() + neg_rebounce_speed);
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

                Nx.SetComplianceTerm((inv_hhpa) * this->compliance);  // was (inv_hh)* ...   //// TEST DAMPING
                Tu.SetComplianceTerm((inv_hhpa) * this->complianceT);
                Tv.SetComplianceTerm((inv_hhpa) * this->complianceT);

                double qc = inv_hpa * this->norm_dist;

                // If clamping kicks in(when using large timesteps and low compliance), it acts as a numerical damping.
                if (do_clamp)
                    qc = std::max(qc, -recovery_clamp);

                Nx.SetRightHandSide(Nx.GetRightHandSide() + qc);

            } else {
                // std::cout << "rigid " << (int)this << "  recov_clamp=" << recovery_clamp << std::endl;
                if (do_clamp)
                    if (this->Nx.GetCohesion())
                        Nx.SetRightHandSide(Nx.GetRightHandSide() + std::min(0.0, std::max(factor * this->norm_dist, -recovery_clamp)));
                    else
                        Nx.SetRightHandSide(Nx.GetRightHandSide() + std::max(factor * this->norm_dist, -recovery_clamp));
                else
                    Nx.SetRightHandSide(Nx.GetRightHandSide() + factor * this->norm_dist);
            }
        }
    }

    virtual void ConstraintsFetch_react(double factor) override {
        // From constraints to react vector:
        react_force.x() = Nx.GetLagrangeMultiplier() * factor;
        react_force.y() = Tu.GetLagrangeMultiplier() * factor;
        react_force.z() = Tv.GetLagrangeMultiplier() * factor;
    }
};

}  // end namespace chrono

#endif
