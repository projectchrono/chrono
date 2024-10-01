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
// Authors: Radu Serban, Alessandro Tasora
// =============================================================================
//
// Smooth (penalty-based) contact between two generic contactable objects.
//
// =============================================================================

#ifndef CH_CONTACT_SMC_H
#define CH_CONTACT_SMC_H

#include <cmath>
#include <algorithm>
#include <limits>

#include "chrono/collision/ChCollisionModel.h"
#include "chrono/core/ChFrame.h"
#include "chrono/core/ChMatrix.h"
#include "chrono/solver/ChKRMBlock.h"
#include "chrono/solver/ChSystemDescriptor.h"
#include "chrono/physics/ChContactContainer.h"
#include "chrono/physics/ChContactTuple.h"
#include "chrono/physics/ChContactMaterialSMC.h"
#include "chrono/physics/ChSystemSMC.h"
#include "chrono/timestepper/ChState.h"

namespace chrono {

/// Default implementation of the SMC normal and tangential force calculation.
class ChDefaultContactForceTorqueSMC : public ChSystemSMC::ChContactForceTorqueSMC {
  public:
    /// Default SMC force calculation algorithm.
    /// This implementation depends on various settings specified at the ChSystemSMC level (such as normal force model,
    /// tangential force model, use of material physical properties, etc).
    virtual ChWrenchd CalculateForceTorque(
        const ChSystemSMC& sys,                    ///< containing system
        const ChVector3d& normal_dir,              ///< normal contact direction (expressed in global frame)
        const ChVector3d& p1,                      ///< most penetrated point on obj1 (expressed in global frame)
        const ChVector3d& p2,                      ///< most penetrated point on obj2 (expressed in global frame)
        const ChVector3d& vel1,                    ///< velocity of contact point on obj1 (expressed in global frame)
        const ChVector3d& vel2,                    ///< velocity of contact point on obj2 (expressed in global frame)
        const ChContactMaterialCompositeSMC& mat,  ///< composite material for contact pair
        double delta,                              ///< overlap in normal direction
        double eff_radius,                         ///< effective radius of curvature at contact
        double mass1,                              ///< mass of obj1
        double mass2,                              ///< mass of obj2
        ChContactable* objA,                       ///< pointer to contactable obj1
        ChContactable* objB                        ///< pointer to contactable obj2
    ) const override {
        // Set contact force to zero if no penetration.
        if (delta <= 0) {
            return {VNULL, VNULL};
        }

        // Extract parameters from containing system
        double dT = sys.GetStep();
        bool use_mat_props = sys.UsingMaterialProperties();
        ChSystemSMC::ContactForceModel contact_model = sys.GetContactForceModel();
        ChSystemSMC::AdhesionForceModel adhesion_model = sys.GetAdhesionForceModel();
        ChSystemSMC::TangentialDisplacementModel tdispl_model = sys.GetTangentialDisplacementModel();

        // Relative velocity at contact
        ChVector3d relvel = vel2 - vel1;
        double relvel_n_mag = relvel.Dot(normal_dir);
        ChVector3d relvel_n = relvel_n_mag * normal_dir;
        ChVector3d relvel_t = relvel - relvel_n;
        double relvel_t_mag = relvel_t.Length();

        // Calculate effective mass
        double eff_mass = mass1 * mass2 / (mass1 + mass2);

        // Calculate stiffness and viscous damping coefficients.
        // All models use the following formulas for normal and tangential forces:
        //     Fn = kn * delta_n - gn * v_n
        //     Ft = kt * delta_t - gt * v_t
        double kn = 0;
        double kt = 0;
        double gn = 0;
        double gt = 0;

        constexpr double eps = std::numeric_limits<double>::epsilon();

        switch (contact_model) {
            case ChSystemSMC::Flores:
                // Currently not implemented.  Fall through to Hooke.
            case ChSystemSMC::Hooke:
                if (use_mat_props) {
                    double tmp_k = (16.0 / 15) * std::sqrt(eff_radius) * mat.E_eff;
                    double v2 = sys.GetCharacteristicImpactVelocity() * sys.GetCharacteristicImpactVelocity();
                    double loge = (mat.cr_eff < eps) ? std::log(eps) : std::log(mat.cr_eff);
                    loge = (mat.cr_eff > 1 - eps) ? std::log(1 - eps) : loge;
                    double tmp_g = 1 + std::pow(CH_PI / loge, 2);
                    kn = tmp_k * std::pow(eff_mass * v2 / tmp_k, 1.0 / 5);
                    kt = kn;
                    gn = std::sqrt(4 * eff_mass * kn / tmp_g);
                    gt = gn;
                } else {
                    kn = mat.kn;
                    kt = mat.kt;
                    gn = eff_mass * mat.gn;
                    gt = eff_mass * mat.gt;
                }

                break;

            case ChSystemSMC::Hertz:
                if (use_mat_props) {
                    double sqrt_Rd = std::sqrt(eff_radius * delta);
                    double Sn = 2 * mat.E_eff * sqrt_Rd;
                    double St = 8 * mat.G_eff * sqrt_Rd;
                    double loge = (mat.cr_eff < eps) ? std::log(eps) : std::log(mat.cr_eff);
                    double beta = loge / std::sqrt(loge * loge + CH_PI * CH_PI);
                    kn = (2.0 / 3) * Sn;
                    kt = St;
                    gn = -2 * std::sqrt(5.0 / 6) * beta * std::sqrt(Sn * eff_mass);
                    gt = -2 * std::sqrt(5.0 / 6) * beta * std::sqrt(St * eff_mass);
                } else {
                    double tmp = eff_radius * std::sqrt(delta);
                    kn = tmp * mat.kn;
                    kt = tmp * mat.kt;
                    gn = tmp * eff_mass * mat.gn;
                    gt = tmp * eff_mass * mat.gt;
                }

                break;

            case ChSystemSMC::PlainCoulomb:
                if (use_mat_props) {
                    double sqrt_Rd = std::sqrt(delta);
                    double Sn = 2 * mat.E_eff * sqrt_Rd;
                    double loge = (mat.cr_eff < eps) ? std::log(eps) : std::log(mat.cr_eff);
                    double beta = loge / std::sqrt(loge * loge + CH_PI * CH_PI);
                    kn = (2.0 / 3) * Sn;
                    gn = -2 * std::sqrt(5.0 / 6) * beta * std::sqrt(Sn * eff_mass);
                } else {
                    double tmp = std::sqrt(delta);
                    kn = tmp * mat.kn;
                    gn = tmp * mat.gn;
                }

                kt = 0;
                gt = 0;

                {
                    double forceN = kn * delta - gn * relvel_n_mag;
                    if (forceN < 0)
                        forceN = 0;
                    double forceT = mat.mu_eff * std::tanh(5.0 * relvel_t_mag) * forceN;
                    switch (adhesion_model) {
                        case ChSystemSMC::AdhesionForceModel::Perko:
                            // Currently not implemented.  Fall through to Constant.
                        case ChSystemSMC::AdhesionForceModel::Constant:
                            forceN -= mat.adhesion_eff;
                            break;
                        case ChSystemSMC::AdhesionForceModel::DMT:
                            forceN -= mat.adhesionMultDMT_eff * std::sqrt(eff_radius);
                            break;
                    }
                    ChVector3d force = forceN * normal_dir;
                    if (relvel_t_mag >= sys.GetSlipVelocityThreshold())
                        force -= (forceT / relvel_t_mag) * relvel_t;

                    return {force, VNULL};  // zero torque anyway
                }
        }

        // Tangential displacement (magnitude)
        double delta_t = 0;
        switch (tdispl_model) {
            case ChSystemSMC::OneStep:
                delta_t = relvel_t_mag * dT;
                break;
            case ChSystemSMC::MultiStep:
                //// TODO: implement proper MultiStep mode
                delta_t = relvel_t_mag * dT;
                break;
            default:
                break;
        }

        // Calculate the magnitudes of the normal and tangential contact forces
        double forceN = kn * delta - gn * relvel_n_mag;
        double forceT = kt * delta_t + gt * relvel_t_mag;

        // If the resulting normal contact force is negative, the two shapes are moving
        // away from each other so fast that no contact force is generated.
        if (forceN < 0) {
            forceN = 0;
            forceT = 0;
        }

        // Include adhesion force
        switch (adhesion_model) {
            case ChSystemSMC::AdhesionForceModel::Perko:
                // Currently not implemented.  Fall through to Constant.
            case ChSystemSMC::AdhesionForceModel::Constant:
                forceN -= mat.adhesion_eff;
                break;
            case ChSystemSMC::AdhesionForceModel::DMT:
                forceN -= mat.adhesionMultDMT_eff * std::sqrt(eff_radius);
                break;
        }

        // Coulomb law
        forceT = std::min<double>(forceT, mat.mu_eff * std::abs(forceN));

        // Accumulate normal and tangential forces
        ChVector3d force = forceN * normal_dir;
        if (relvel_t_mag >= sys.GetSlipVelocityThreshold())
            force -= (forceT / relvel_t_mag) * relvel_t;

        return {force, VNULL};  // zero torque anyway
    }
};

/// Class for smooth (penalty-based) contact between two generic contactable objects.
/// Ta and Tb are of ChContactable sub classes.
template <class Ta, class Tb>
class ChContactSMC : public ChContactTuple<Ta, Tb> {
  public:
    typedef typename ChContactTuple<Ta, Tb>::typecarr_a typecarr_a;
    typedef typename ChContactTuple<Ta, Tb>::typecarr_b typecarr_b;

  private:
    struct ChContactJacobian {
        ChKRMBlock m_KRM;        ///< sum of scaled K and R, with pointers to sparse variables
        ChMatrixDynamic<double> m_K;  ///< K = dQ/dx
        ChMatrixDynamic<double> m_R;  ///< R = dQ/dv
    };

    ChVector3d m_force;        ///< contact force on objB
    ChVector3d m_torque;       ///< contact torque on objB
    ChContactJacobian* m_Jac;  ///< contact Jacobian data

  public:
    ChContactSMC() : m_Jac(NULL) {}

    ChContactSMC(ChContactContainer* contact_container,    ///< contact container
                 Ta* obj_A,                                ///< contactable object A
                 Tb* obj_B,                                ///< contactable object B
                 const ChCollisionInfo& cinfo,             ///< data for the collision pair
                 const ChContactMaterialCompositeSMC& mat  ///< composite material
                 )
        : ChContactTuple<Ta, Tb>(contact_container, obj_A, obj_B), m_Jac(NULL) {
        Reset(obj_A, obj_B, cinfo, mat);
    }

    ~ChContactSMC() { delete m_Jac; }

    /// Get the contact force, if computed, in contact coordinate system
    virtual ChVector3d GetContactForce() const override { return this->contact_plane.transpose() * m_force; }

    /// Get the contact torque, if computed, in contact coordinate system
    virtual ChVector3d GetContactTorque() const override { return this->contact_plane.transpose() * m_torque; }

    /// Get the contact penetration (positive if there is overlap).
    double GetContactPenetration() const { return -this->norm_dist; }

    /// Get the contact force, expressed in the absolute frame
    ChVector3d GetContactForceAbs() const { return m_force; }

    /// Get the contact torque, expressed in the absolute frame
    ChVector3d GetContactTorqueAbs() const { return m_torque; }

    /// Access the proxy to the Jacobian.
    const ChKRMBlock* GetJacobianKRM() const { return m_Jac ? &(m_Jac->m_KRM) : NULL; }
    const ChMatrixDynamic<double>* GetJacobianK() const { return m_Jac ? &(m_Jac->m_K) : NULL; }
    const ChMatrixDynamic<double>* GetJacobianR() const { return m_Jac ? &(m_Jac->m_R) : NULL; }

    /// Reinitialize this contact for reuse.
    void Reset(Ta* obj_A,                                ///< contactable object A
               Tb* obj_B,                                ///< contactable object B
               const ChCollisionInfo& cinfo,             ///< data for the collision pair
               const ChContactMaterialCompositeSMC& mat  ///< composite material
    ) {
        // Reset geometric information
        this->Reset_cinfo(obj_A, obj_B, cinfo);

        // Note: cinfo.distance is the same as this->norm_dist.
        assert(cinfo.distance < 0);

        // Calculate contact force.
        auto wrench =
            CalculateForceTorque(-this->norm_dist,                            // overlap (here, always positive)
                                 this->normal,                                // normal contact direction
                                 this->objA->GetContactPointSpeed(this->p1),  // velocity of contact point on objA
                                 this->objB->GetContactPointSpeed(this->p2),  // velocity of contact point on objB
                                 mat                                          // composite material for contact pair
            );
        m_force = wrench.force;
        m_torque = wrench.torque;

        // Set up and compute Jacobian matrices.
        if (static_cast<ChSystemSMC*>(this->container->GetSystem())->IsContactStiff()) {
            CreateJacobians();
            CalculateJacobians(mat);
        }
    }

    /// Calculate contact force, and maybe torque too, expressed in absolute coordinates.
    ChWrenchd CalculateForceTorque(
        double delta,                             ///< overlap in normal direction
        const ChVector3d& normal_dir,             ///< normal contact direction (expressed in global frame)
        const ChVector3d& vel1,                   ///< velocity of contact point on objA (expressed in global frame)
        const ChVector3d& vel2,                   ///< velocity of contact point on objB (expressed in global frame)
        const ChContactMaterialCompositeSMC& mat  ///< composite material for contact pair
    ) {
        // Set contact force to zero if no penetration.
        if (delta <= 0) {
            return {VNULL, VNULL};
        }

        // Use current SMC algorithm to calculate the force
        ChSystemSMC* sys = static_cast<ChSystemSMC*>(this->container->GetSystem());
        return sys->GetContactForceTorqueAlgorithm().CalculateForceTorque(
            *sys, normal_dir, this->p1, this->p2, vel1, vel2, mat, delta, this->eff_radius,
            this->objA->GetContactableMass(), this->objB->GetContactableMass(), this->objA, this->objB);
    }

    /// Compute all forces in a contiguous array.
    /// Used in finite-difference Jacobian approximation.
    void CalculateQ(const ChState& stateA_x,                   ///< state positions for objA
                    const ChStateDelta& stateA_w,              ///< state velocities for objA
                    const ChState& stateB_x,                   ///< state positions for objB
                    const ChStateDelta& stateB_w,              ///< state velocities for objB
                    const ChContactMaterialCompositeSMC& mat,  ///< composite material for contact pair
                    ChVectorDynamic<>& Q                       ///< output generalized forces
    ) {
        // Express contact points in local frames.
        // We assume that these points remain fixed to their respective contactable objects.
        ChVector3d p1_loc = this->objA->GetCollisionModelFrame().TransformPointParentToLocal(this->p1);
        ChVector3d p2_loc = this->objB->GetCollisionModelFrame().TransformPointParentToLocal(this->p2);

        // Express the local points in global frame
        ChVector3d p1_abs = this->objA->GetContactPoint(p1_loc, stateA_x);
        ChVector3d p2_abs = this->objB->GetContactPoint(p2_loc, stateB_x);

        /*
            Note: while this can be somewhat justified for a ChBody, it will not work
                  for a mesh vertex for instance...

        // Project the points onto the unperturbed normal line
        p1_abs = this->p1 + Vdot(p1_abs - this->p1, this->normal) * this->normal;
        p2_abs = this->p2 + Vdot(p2_abs - this->p2, this->normal) * this->normal;
        */

        // Calculate normal direction (expressed in global frame)
        ChVector3d normal_dir = (p1_abs - p2_abs).GetNormalized();

        // Calculate penetration depth
        double delta = (p1_abs - p2_abs).Length();

        // If the normal direction flipped sign, change sign of delta
        if (Vdot(normal_dir, this->normal) < 0)
            delta = -delta;

        // Calculate velocity of contact points (expressed in global frame)
        ChVector3d vel1 = this->objA->GetContactPointSpeed(p1_loc, stateA_x, stateA_w);
        ChVector3d vel2 = this->objB->GetContactPointSpeed(p2_loc, stateB_x, stateB_w);

        // Compute the contact force and torque
        auto wrench = CalculateForceTorque(delta, normal_dir, vel1, vel2, mat);
        auto force = wrench.force;
        auto torque = wrench.torque;

        // Compute and load the generalized contact forces.
        this->objA->ContactComputeQ(-force, -torque, p1_abs, stateA_x, Q, 0);
        this->objB->ContactComputeQ(force, torque, p2_abs, stateB_x, Q, this->objA->GetContactableNumCoordsVelLevel());
    }

    /// Create the Jacobian matrices.
    /// These matrices are created/resized as needed.
    void CreateJacobians() {
        delete m_Jac;
        m_Jac = new ChContactJacobian;

        // Set variables and resize Jacobian matrices.
        // NOTE: currently, only contactable objects derived from ChContactable_1vars<6>,
        //       ChContactable_1vars<3>, and ChContactable_3vars<3,3,3> are supported.
        int ndof_w = 0;
        std::vector<ChVariables*> vars;

        vars.push_back(this->objA->GetVariables1());
        if (auto objA_333 = dynamic_cast<ChContactable_3vars<3, 3, 3>*>(this->objA)) {
            vars.push_back(objA_333->GetVariables2());
            vars.push_back(objA_333->GetVariables3());
        }
        ndof_w += this->objA->GetContactableNumCoordsVelLevel();

        vars.push_back(this->objB->GetVariables1());
        if (auto objB_333 = dynamic_cast<ChContactable_3vars<3, 3, 3>*>(this->objB)) {
            vars.push_back(objB_333->GetVariables2());
            vars.push_back(objB_333->GetVariables3());
        }
        ndof_w += this->objB->GetContactableNumCoordsVelLevel();

        m_Jac->m_KRM.SetVariables(vars);
        m_Jac->m_K.setZero(ndof_w, ndof_w);
        m_Jac->m_R.setZero(ndof_w, ndof_w);
        assert(m_Jac->m_KRM.GetMatrix().cols() == ndof_w);
    }

    /// Calculate Jacobian of generalized contact forces.
    void CalculateJacobians(const ChContactMaterialCompositeSMC& mat) {
        // Compute a finite-difference approximations to the Jacobians of the contact forces and
        // load dQ/dx into m_Jac->m_K and dQ/dw into m_Jac->m_R.
        // Note that we only calculate these Jacobians whenever the contact force itself is calculated,
        // that is only once per step.  The Jacobian of generalized contact forces will therefore be
        // constant over the time step.

        // Get states for objA
        int ndofA_x = this->objA->GetContactableNumCoordsPosLevel();
        int ndofA_w = this->objA->GetContactableNumCoordsVelLevel();
        ChState stateA_x(ndofA_x, NULL);
        ChStateDelta stateA_w(ndofA_w, NULL);
        this->objA->ContactableGetStateBlockPosLevel(stateA_x);
        this->objA->ContactableGetStateBlockVelLevel(stateA_w);

        // Get states for objB
        int ndofB_x = this->objB->GetContactableNumCoordsPosLevel();
        int ndofB_w = this->objB->GetContactableNumCoordsVelLevel();
        ChState stateB_x(ndofB_x, NULL);
        ChStateDelta stateB_w(ndofB_w, NULL);
        this->objB->ContactableGetStateBlockPosLevel(stateB_x);
        this->objB->ContactableGetStateBlockVelLevel(stateB_w);

        // Compute Q at current state
        ChVectorDynamic<> Q0(ndofA_w + ndofB_w);
        CalculateQ(stateA_x, stateA_w, stateB_x, stateB_w, mat, Q0);

        // Finite-difference approximation perturbation.
        // Note that ChState and ChStateDelta are set to 0 on construction.
        // To accommodate objects with quaternion states, use the method ContactableIncrementState while
        // calculating Jacobian columns corresponding to position states.
        double perturbation = 1e-5;
        ChState stateA_x1(ndofA_x, NULL);
        ChState stateB_x1(ndofB_x, NULL);
        ChStateDelta prtrbA(ndofA_w, NULL);
        ChStateDelta prtrbB(ndofB_w, NULL);

        ChVectorDynamic<> Q1(ndofA_w + ndofB_w);

        // Jacobian w.r.t. variables of objA
        for (int i = 0; i < ndofA_w; i++) {
            prtrbA(i) += perturbation;
            this->objA->ContactableIncrementState(stateA_x, prtrbA, stateA_x1);
            CalculateQ(stateA_x1, stateA_w, stateB_x, stateB_w, mat, Q1);
            prtrbA(i) -= perturbation;

            m_Jac->m_K.col(i) = (Q1 - Q0) * (-1 / perturbation);  // note sign change

            stateA_w(i) += perturbation;
            CalculateQ(stateA_x, stateA_w, stateB_x, stateB_w, mat, Q1);
            stateA_w(i) -= perturbation;

            m_Jac->m_R.col(i) = (Q1 - Q0) * (-1 / perturbation);  // note sign change
        }

        // Jacobian w.r.t. variables of objB
        for (int i = 0; i < ndofB_w; i++) {
            prtrbB(i) += perturbation;
            this->objB->ContactableIncrementState(stateB_x, prtrbB, stateB_x1);
            CalculateQ(stateA_x, stateA_w, stateB_x1, stateB_w, mat, Q1);
            prtrbB(i) -= perturbation;

            m_Jac->m_K.col(ndofA_w + i) = (Q1 - Q0) * (-1 / perturbation);  // note sign change

            stateB_w(i) += perturbation;
            CalculateQ(stateA_x, stateA_w, stateB_x, stateB_w, mat, Q1);
            stateB_w(i) -= perturbation;

            m_Jac->m_R.col(ndofA_w + i) = (Q1 - Q0) * (-1 / perturbation);  // note sign change
        }
    }

    /// Apply contact forces to the two objects.
    /// (new version, for interfacing to ChTimestepper and ChIntegrable)
    virtual void ContIntLoadResidual_F(ChVectorDynamic<>& R, const double c) override {
        ChVector3d abs_force_scaled(m_force * c);
        ChVector3d abs_torque_scaled(m_torque * c);

        if (this->objA->IsContactActive())
            this->objA->ContactForceLoadResidual_F(-abs_force_scaled, -abs_torque_scaled, this->p1, R);

        if (this->objB->IsContactActive())
            this->objB->ContactForceLoadResidual_F(abs_force_scaled, abs_torque_scaled, this->p2, R);
    }

    /// Inject Jacobian blocks into the system descriptor.
    /// Tell to a system descriptor that there are item(s) of type ChKRMBlock in this object
    /// (for further passing it to a solver)
    virtual void ContInjectKRMmatrices(ChSystemDescriptor& mdescriptor) override {
        if (m_Jac)
            mdescriptor.InsertKRMBlock(&m_Jac->m_KRM);
    }

    /// Compute Jacobian of contact forces.
    virtual void ContKRMmatricesLoad(double Kfactor, double Rfactor) override {
        if (m_Jac) {
            m_Jac->m_KRM.GetMatrix().setZero();

            m_Jac->m_KRM.GetMatrix() += m_Jac->m_K * Kfactor;
            m_Jac->m_KRM.GetMatrix() += m_Jac->m_R * Rfactor;
        }
    }
};

}  // end namespace chrono

#endif
