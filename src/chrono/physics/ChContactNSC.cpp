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

#include "chrono/physics/ChContactNSC.h"

namespace chrono {

ChContactNSC::ChContactNSC() {
    Nx.SetTangentialConstraintU(&Tu);
    Nx.SetTangentialConstraintV(&Tv);
}

ChContactNSC::ChContactNSC(ChContactContainer* contact_container,     // contact container
                           ChContactable* obj_A,                      // contactable object A
                           ChContactable* obj_B,                      // contactable object B
                           const ChCollisionInfo& cinfo,              // data for the collision pair
                           const ChContactMaterialCompositeNSC& mat,  // composite material
                           double min_speed                           // minimum speed for rebounce
                           )
    : ChContact(contact_container, obj_A, obj_B) {
    Nx.SetTangentialConstraintU(&Tu);
    Nx.SetTangentialConstraintV(&Tv);

    Reset(obj_A, obj_B, cinfo, mat, min_speed);
}

void ChContactNSC::Reset(ChContactable* obj_A,                      // contactable object A
                         ChContactable* obj_B,                      // contactable object B
                         const ChCollisionInfo& cinfo,              // data for the collision pair
                         const ChContactMaterialCompositeNSC& mat,  // composite material
                         double min_speed                           // minimum speed for rebounce
) {
    // Reset geometric information
    Reset_cinfo(obj_A, obj_B, cinfo);

    // Create constraint tuples and set variables
    Nx.SetTuplesFromContactables(objA, objB);
    Tu.SetTuplesFromContactables(objA, objB);
    Tv.SetTuplesFromContactables(objA, objB);

    // Cache composite material properties
    Nx.SetFrictionCoefficient(mat.static_friction);
    Nx.SetCohesion(mat.cohesion);

    restitution = mat.restitution;
    dampingf = mat.dampingf;
    compliance = mat.compliance;
    complianceT = mat.complianceT;

    reactions_cache = cinfo.reaction_cache;

    min_rebounce_speed = min_speed;

    // COMPUTE JACOBIANS

    // delegate objA to compute its half of jacobian
    objA->ComputeJacobianForContactPart(p1, contact_plane, Nx.TupleA(), Tu.TupleA(), Tv.TupleA(), false);

    // delegate objB to compute its half of jacobian
    objB->ComputeJacobianForContactPart(p2, contact_plane, Nx.TupleB(), Tu.TupleB(), Tv.TupleB(), true);

    if (reactions_cache) {
        react_force.x() = reactions_cache[0];
        react_force.y() = reactions_cache[1];
        react_force.z() = reactions_cache[2];
        // std::cout << "Reset Fn=" << (double)reactions_cache[0] << "  at cache address:" <<
        // (int)reactions_cache << std::endl;
    } else {
        react_force = VNULL;
    }
}

void ChContactNSC::ContIntStateGatherReactions(const unsigned int off_L, ChVectorDynamic<>& L) {
    L(off_L) = react_force.x();
    L(off_L + 1) = react_force.y();
    L(off_L + 2) = react_force.z();
}

void ChContactNSC::ContIntStateScatterReactions(const unsigned int off_L, const ChVectorDynamic<>& L) {
    react_force.x() = L(off_L);
    react_force.y() = L(off_L + 1);
    react_force.z() = L(off_L + 2);

    if (reactions_cache) {
        reactions_cache[0] = (float)L(off_L);      // react_force.x();
        reactions_cache[1] = (float)L(off_L + 1);  // react_force.y();
        reactions_cache[2] = (float)L(off_L + 2);  // react_force.z();
    }
}

void ChContactNSC::ContIntLoadResidual_CqL(const unsigned int off_L,
                                           ChVectorDynamic<>& R,
                                           const ChVectorDynamic<>& L,
                                           const double c) {
    Nx.AddJacobianTransposedTimesScalarInto(R, L(off_L) * c);
    Tu.AddJacobianTransposedTimesScalarInto(R, L(off_L + 1) * c);
    Tv.AddJacobianTransposedTimesScalarInto(R, L(off_L + 2) * c);
}

void ChContactNSC::ContIntLoadConstraint_C(const unsigned int off_L,
                                           ChVectorDynamic<>& Qc,
                                           const double c,
                                           bool do_clamp,
                                           double recovery_clamp) {
    bool bounced = false;

    // Elastic Restitution model (use simple Newton model with coefficient e=v(+)/v(-))
    // Note that this works only if the two connected items are two ChBody.

    if (objA && objB) {
        if (restitution) {
            // compute normal rebounce speed
            ChVector3d V1_w = objA->GetContactPointSpeed(p1);
            ChVector3d V2_w = objB->GetContactPointSpeed(p2);
            ChVector3d Vrel_w = V2_w - V1_w;
            ChVector3d Vrel_cplane = contact_plane.transpose() * Vrel_w;

            double h = container->GetSystem()->GetStep();  // = 1.0 / c;  // not all steppers have c = 1/h

            double neg_rebounce_speed = Vrel_cplane.x() * restitution;
            if (neg_rebounce_speed < -min_rebounce_speed)
                if (norm_dist + neg_rebounce_speed * h < 0) {
                    // CASE: BOUNCE
                    bounced = true;
                    Qc(off_L) += neg_rebounce_speed;
                }
        }
    }

    if (!bounced) {
        // CASE: SETTLE (most often, and also default if two colliding items are not two ChBody)

        if (compliance) {
            double h = 1.0 / c;  // was: container->GetSystem()->GetStep(); note not all steppers have c = 1/h

            double alpha = dampingf;                    // [R]=alpha*[K]
            double inv_hpa = 1.0 / (h + alpha);         // 1/(h+a)
            double inv_hhpa = 1.0 / (h * (h + alpha));  // 1/(h*(h+a))

            //// TODO  move to LoadKRMMatrices() the following, and only for !bounced case
            Nx.SetComplianceTerm((inv_hhpa)*compliance);
            Tu.SetComplianceTerm((inv_hhpa)*complianceT);
            Tv.SetComplianceTerm((inv_hhpa)*complianceT);

            double qc = inv_hpa * norm_dist;  //// TODO  see how to move this in LoadKRMMatrices()

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
                if (Nx.GetCohesion())
                    Qc(off_L) += std::min(0.0, std::max(c * norm_dist, -recovery_clamp));
                else
                    Qc(off_L) += std::max(c * norm_dist, -recovery_clamp);
            else
                Qc(off_L) += c * norm_dist;
        }
    }
}

void ChContactNSC::ContIntToDescriptor(const unsigned int off_L,
                                       const ChVectorDynamic<>& L,
                                       const ChVectorDynamic<>& Qc) {
    // only for solver warm start
    Nx.SetLagrangeMultiplier(L(off_L));
    Tu.SetLagrangeMultiplier(L(off_L + 1));
    Tv.SetLagrangeMultiplier(L(off_L + 2));

    // solver known terms
    Nx.SetRightHandSide(Qc(off_L));
    Tu.SetRightHandSide(Qc(off_L + 1));
    Tv.SetRightHandSide(Qc(off_L + 2));
}

void ChContactNSC::ContIntFromDescriptor(const unsigned int off_L, ChVectorDynamic<>& L) {
    L(off_L) = Nx.GetLagrangeMultiplier();
    L(off_L + 1) = Tu.GetLagrangeMultiplier();
    L(off_L + 2) = Tv.GetLagrangeMultiplier();
}

void ChContactNSC::InjectConstraints(ChSystemDescriptor& descriptor) {
    descriptor.InsertConstraint(&Nx);
    descriptor.InsertConstraint(&Tu);
    descriptor.InsertConstraint(&Tv);
}

void ChContactNSC::ConstraintsBiReset() {
    Nx.SetRightHandSide(0.);
    Tu.SetRightHandSide(0.);
    Tv.SetRightHandSide(0.);
}

void ChContactNSC::ConstraintsBiLoad_C(double factor, double recovery_clamp, bool do_clamp) {
    bool bounced = false;

    // Elastic Restitution model (use simple Newton model with coefficient e=v(+)/v(-))
    // Note that this works only if the two connected items are two ChBody.

    if (objA && objB) {
        if (restitution) {
            // compute normal rebounce speed
            ChVector3d V1_w = objA->GetContactPointSpeed(p1);
            ChVector3d V2_w = objB->GetContactPointSpeed(p2);
            ChVector3d Vrel_w = V2_w - V1_w;
            ChVector3d Vrel_cplane = contact_plane.transpose() * Vrel_w;

            double h = 1.0 / factor;  // inverse timestep is factor

            double neg_rebounce_speed = Vrel_cplane.x() * restitution;
            if (neg_rebounce_speed < -min_rebounce_speed)
                if (norm_dist + neg_rebounce_speed * h < 0) {
                    // CASE: BOUNCE
                    bounced = true;
                    Nx.SetRightHandSide(Nx.GetRightHandSide() + neg_rebounce_speed);
                }
        }
    }

    if (!bounced) {
        // CASE: SETTLE (most often, and also default if two colliding items are not two ChBody)

        if (compliance) {
            //  inverse timestep is factor
            double h = 1.0 / factor;

            double alpha = dampingf;                    // [R]=alpha*[K]
            double inv_hpa = 1.0 / (h + alpha);         // 1/(h+a)
            double inv_hhpa = 1.0 / (h * (h + alpha));  // 1/(h*(h+a))

            Nx.SetComplianceTerm((inv_hhpa)*compliance);  // was (inv_hh)* ...   //// TEST DAMPING
            Tu.SetComplianceTerm((inv_hhpa)*complianceT);
            Tv.SetComplianceTerm((inv_hhpa)*complianceT);

            double qc = inv_hpa * norm_dist;

            // If clamping kicks in(when using large timesteps and low compliance), it acts as a numerical damping.
            if (do_clamp)
                qc = std::max(qc, -recovery_clamp);

            Nx.SetRightHandSide(Nx.GetRightHandSide() + qc);

        } else {
            // std::cout << "rigid " << (int)this << "  recov_clamp=" << recovery_clamp << std::endl;
            if (do_clamp)
                if (Nx.GetCohesion())
                    Nx.SetRightHandSide(Nx.GetRightHandSide() +
                                        std::min(0.0, std::max(factor * norm_dist, -recovery_clamp)));
                else
                    Nx.SetRightHandSide(Nx.GetRightHandSide() + std::max(factor * norm_dist, -recovery_clamp));
            else
                Nx.SetRightHandSide(Nx.GetRightHandSide() + factor * norm_dist);
        }
    }
}

void ChContactNSC::ConstraintsFetch_react(double factor) {
    // From constraints to react vector:
    react_force.x() = Nx.GetLagrangeMultiplier() * factor;
    react_force.y() = Tu.GetLagrangeMultiplier() * factor;
    react_force.z() = Tv.GetLagrangeMultiplier() * factor;
}

}  // end namespace chrono
