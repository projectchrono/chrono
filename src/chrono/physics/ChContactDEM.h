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

#ifndef CHCONTACTDEM_H
#define CHCONTACTDEM_H

#include <cmath>

#include "chrono/collision/ChCCollisionModel.h"
#include "chrono/core/ChFrame.h"
#include "chrono/core/ChMatrixDynamic.h"
#include "chrono/core/ChVectorDynamic.h"
#include "chrono/lcp/ChLcpKblockGeneric.h"
#include "chrono/lcp/ChLcpSystemDescriptor.h"
#include "chrono/physics/ChContactContainerBase.h"
#include "chrono/physics/ChContactTuple.h"
#include "chrono/physics/ChMaterialSurfaceDEM.h"
#include "chrono/physics/ChSystemDEM.h"
#include "chrono/timestepper/ChState.h"

namespace chrono {

/// Class for DEM-P contact between two generic ChContactable objects.
/// Ta and Tb are of ChContactable sub classes.
template <class Ta, class Tb>
class ChContactDEM : public ChContactTuple<Ta, Tb> {
  public:
    typedef typename ChContactTuple<Ta, Tb>::typecarr_a typecarr_a;
    typedef typename ChContactTuple<Ta, Tb>::typecarr_b typecarr_b;

  private:
    struct ChContactJacobian {
        ChLcpKblockGeneric m_KRM;     ///< sum of scaled K and R, with pointers to sparse variables
        ChMatrixDynamic<double> m_K;  ///< K = dQ/dx
        ChMatrixDynamic<double> m_R;  ///< R = dQ/dv
    };

    ChVector<> m_force;        ///< contact force on objB
    ChContactJacobian* m_Jac;  ///< contact Jacobian data

  public:
    ChContactDEM() : m_Jac(NULL) {}

    ChContactDEM(ChContactContainerBase* mcontainer,      ///< contact container
                 Ta* mobjA,                               ///< collidable object A
                 Tb* mobjB,                               ///< collidable object B
                 const collision::ChCollisionInfo& cinfo  ///< data for the contact pair
                 )
        : ChContactTuple<Ta, Tb>(mcontainer, mobjA, mobjB, cinfo), m_Jac(NULL) {
        Reset(mobjA, mobjB, cinfo);
    }

    ~ChContactDEM() {
        delete m_Jac;
    }

    /// Get the contact force, if computed, in absolute coordinate system.
    virtual ChVector<> GetContactForce() override { return m_force; }

    /// Get the contact penetration (positive if there is overlap).
    double GetContactPenetration() const { return -this->norm_dist; }

    /// Get the contact force, expressed in the frame of the contact.
    ChVector<> GetContactForceLocal() const { return this->contact_plane.MatrT_x_Vect(m_force); }

    /// Access the proxy to the Jacobian, for sparse LCP solver.
    ChLcpKblockGeneric& GetJacobian() { return m_Jac; }

    /// Reinitialize this contact.
    virtual void Reset(Ta* mobjA,                               ///< collidable object A
                       Tb* mobjB,                               ///< collidable object B
                       const collision::ChCollisionInfo& cinfo  ///< data for the contact pair
                       ) override {
        // Inherit base class.
        ChContactTuple<Ta, Tb>::Reset(mobjA, mobjB, cinfo);

        // Note: cinfo.distance is the same as this->norm_dist.
        assert(cinfo.distance < 0);

        // Calculate contact force.
        CalculateForce(-this->norm_dist,                            // overlap (here, always positive)
                       this->normal,                                // normal contact direction
                       this->objA->GetContactPointSpeed(this->p1),  // velocity of contact point on objA
                       this->objB->GetContactPointSpeed(this->p2),  // velocity of contact point on objB
                       m_force                                      // computed force
                       );

        // Set up and compute Jacobian matrices.
        if (static_cast<ChSystemDEM*>(this->container->GetSystem())->GetStiffContact()) {
            CreateJacobians();
            CalculateJacobians();
        }
    }

    /// Calculate contact force, expressed in absolute coordinates.
    void CalculateForce(double delta,                  ///< overlap in normal direction (positive)
                        const ChVector<>& normal_dir,  ///< normal contact direction (expressed in global frame)
                        const ChVector<>& vel1,        ///< velocity of contact point on objA (expressed in global frame)
                        const ChVector<>& vel2,        ///< velocity of contact point on objB (expressed in global frame)
                        ChVector<>& force              ///< output force vector
                        ) {
        // Set contact force to zero if overlap is non-positive.
        if (delta <= 0) {
            force = ChVector<>(0, 0, 0);
            return;
        }

        // Extract parameters from containing system
        ChSystemDEM* sys = static_cast<ChSystemDEM*>(this->container->GetSystem());
        double dT = sys->GetStep();
        bool use_mat_props = sys->UseMaterialProperties();
        bool use_history = sys->UseContactHistory();
        ChSystemDEM::ContactForceModel contact_model = sys->GetContactForceModel();
        ChSystemDEM::AdhesionForceModel adhesion_model = sys->GetAdhesionForceModel();

        // Relative velocity at contact
        ChVector<> relvel = vel2 - vel1;
        double relvel_n_mag = relvel.Dot(normal_dir);
        ChVector<> relvel_n = relvel_n_mag * normal_dir;
        ChVector<> relvel_t = relvel - relvel_n;
        double relvel_t_mag = relvel_t.Length();

        // Calculate effective mass
        double m_eff = this->objA->GetContactableMass() * this->objB->GetContactableMass() /
                       (this->objA->GetContactableMass() + this->objB->GetContactableMass());

        // Calculate effective contact radius
        //// TODO:  how can I get this with current collision system!?!?!?
        double R_eff = 1;

        // Just casting, now, since we are sure that this contact was created only if dynamic casting was fine
        auto mmatA = std::static_pointer_cast<ChMaterialSurfaceDEM>(this->objA->GetMaterialSurfaceBase());
        auto mmatB = std::static_pointer_cast<ChMaterialSurfaceDEM>(this->objB->GetMaterialSurfaceBase());

        // Calculate composite material properties
        ChCompositeMaterialDEM mat = ChMaterialSurfaceDEM::CompositeMaterial(mmatA, mmatB);

        // Contact forces.
        // All models use the following formulas for normal and tangential forces:
        //     Fn = kn * delta_n - gn * v_n
        //     Ft = kt * delta_t - gt * v_t
        double kn;
        double kt;
        double gn;
        double gt;

        double delta_t = use_history ? relvel_t_mag * dT : 0;

        // Include contact force
        switch (contact_model) {
            case ChSystemDEM::Hooke:
                if (use_mat_props) {
                    double tmp_k = (16.0 / 15) * std::sqrt(R_eff) * mat.E_eff;
                    double v2 = sys->GetCharacteristicImpactVelocity() * sys->GetCharacteristicImpactVelocity();
                    double loge = (mat.cr_eff < CH_MICROTOL) ? std::log(CH_MICROTOL) : std::log(mat.cr_eff);
                    loge = (mat.cr_eff > 1 - CH_MICROTOL) ? std::log(1 - CH_MICROTOL) : loge;
                    double tmp_g = 1 + std::pow(CH_C_PI / loge, 2);
                    kn = tmp_k * std::pow(m_eff * v2 / tmp_k, 1.0 / 5);
                    kt = kn;
                    gn = std::sqrt(4 * m_eff * kn / tmp_g);
                    gt = gn;
                } else {
                    kn = mat.kn;
                    kt = mat.kt;
                    gn = m_eff * mat.gn;
                    gt = m_eff * mat.gt;
                }

                break;

            case ChSystemDEM::Hertz:
                if (use_mat_props) {
                    double sqrt_Rd = std::sqrt(R_eff * delta);
                    double Sn = 2 * mat.E_eff * sqrt_Rd;
                    double St = 8 * mat.G_eff * sqrt_Rd;
                    double loge = (mat.cr_eff < CH_MICROTOL) ? std::log(CH_MICROTOL) : std::log(mat.cr_eff);
                    double beta = loge / std::sqrt(loge * loge + CH_C_PI * CH_C_PI);
                    kn = (2.0 / 3) * Sn;
                    kt = St;
                    gn = -2 * std::sqrt(5.0 / 6) * beta * std::sqrt(Sn * m_eff);
                    gt = -2 * std::sqrt(5.0 / 6) * beta * std::sqrt(St * m_eff);
                } else {
                    double tmp = R_eff * std::sqrt(delta);
                    kn = tmp * mat.kn;
                    kt = tmp * mat.kt;
                    gn = tmp * m_eff * mat.gn;
                    gt = tmp * m_eff * mat.gt;
                }

                break;
        }

        // Calculate the magnitudes of the normal and tangential contact forces
        double forceN = kn * delta - gn * relvel_n_mag;
        double forceT = kt * delta_t + gt * relvel_t_mag;

        // If the resulting force is negative, the two shapes are moving away from
        // each other so fast that no contact force is generated.
        if (forceN < 0) {
            forceN = 0;
            forceT = 0;
        }

        // Include adhesion force
        switch (adhesion_model) {
        case ChSystemDEM::Constant:
            forceN -= mat.adhesion_eff;
            break;
        case ChSystemDEM::DMT:
            forceN -= mat.adhesionMultDMT_eff * sqrt(R_eff);
            break;
        }

        // Coulomb law
        double forceT_mag = std::abs(forceT);
        double forceT_max = mat.mu_eff * std::abs(forceN);
        double ratio = ((forceT_mag > forceT_max) && (forceT_max > CH_MICROTOL)) ? forceT_max / forceT_mag : 0;
        forceT *= ratio;

        // Accumulate normal and tangential forces
        force = forceN * normal_dir;
        if (relvel_t_mag >= sys->GetSlipVelocitythreshold())
            force -= (forceT / std::max(relvel_t_mag, CH_MICROTOL)) * relvel_t;
    }

    /// Compute all forces in a contiguous array.
    /// Used in finite-difference Jacobian approximation.
    void CalculateQ(const ChState& stateA_x,       ///< state positions for objA
                    const ChStateDelta& stateA_w,  ///< state velocities for objA
                    const ChState& stateB_x,       ///< state positions for objB
                    const ChStateDelta& stateB_w,  ///< state velocities for objB
                    ChVectorDynamic<>& Q           ///< output generalized forces
                    ) {
        /*
        double delta;           // overlap in normal direction (positive)
        ChVector<> normal_dir;  // normal contact direction (expressed in global frame)
        ChVector<> pt1;         // contact point on objA (expressed in global frame)
        ChVector<> pt2;         // contact point on objB (expressed in global frame)
        ChVector<> vel1;        // velocity of contact point on objA (expressed in global frame)
        ChVector<> vel2;        // velocity of contact point on objB (expressed in global frame)

        // Calculate contact points, their velocity, penetration, and normal direction
        // at the specified states for the two contactable objects.

        //// TODO
        ////   - assume that the two contact points remain fixed on the respective contactables
        ////   - calculate contact penetration, contact normal, and velocities of the contact points
        ////     given (perturbed) states of the contactables
        ////     (what new methods do we need on a ChContactable?)

        // Compute the contact force.
        ChVector<> force;
        CalculateForce(delta, normal_dir, vel1, vel2, force);

        // Compute and load the generalized contact forces.
        this->objA->ContactForceLoadQ(-force, pt1, Q, 0);
        this->objB->ContactForceLoadQ(force, pt2, Q, this->objA->ContactableGet_ndof_w());
        */
    }

    /// Create the Jacobian matrices.
    /// These matrices are created/resized as needed.
    void CreateJacobians() {
        delete m_Jac;
        m_Jac = new ChContactJacobian;

        // Set variables and resize Jacobian matrices.
        // NOTE: currently, only contactable objects derived from ChContactable_1vars<6>,
        //       ChContactable_1vars<3>, and ChContactable_3vars<3,3,3> are supported.
        int ndof_x = 0;
        int ndof_w = 0;
        std::vector<ChLcpVariables*> vars;

        vars.push_back(this->objA->GetVariables1());
        if (auto objA_333 = dynamic_cast<ChContactable_3vars<3, 3, 3>*>(this->objA)) {
            vars.push_back(objA_333->GetVariables2());
            vars.push_back(objA_333->GetVariables3());
        }
        ndof_x += this->objA->ContactableGet_ndof_x();
        ndof_w += this->objA->ContactableGet_ndof_w();

        vars.push_back(this->objB->GetVariables1());
        if (auto objB_333 = dynamic_cast<ChContactable_3vars<3, 3, 3>*>(this->objB)) {
            vars.push_back(objB_333->GetVariables2());
            vars.push_back(objB_333->GetVariables3());
        }
        ndof_x += this->objB->ContactableGet_ndof_x();
        ndof_w += this->objB->ContactableGet_ndof_w();

        m_Jac->m_KRM.SetVariables(vars);
        m_Jac->m_K.Reset(ndof_w, ndof_x);
        m_Jac->m_R.Reset(ndof_w, ndof_w);
    }

    /// Calculate Jacobian of generalized contact forces.
    void CalculateJacobians() {
        // Compute a finite-difference approximations to the Jacobians of the contact forces and
        // load dQ/dx into m_Jac->m_K and dQ/dw into m_Jac->m_R.
        // Note that we only calculate these Jacobians whenever the contact force itself is calculated,
        // that is only once per step.  The Jacobian of generalized contact forces will therefore be
        // constant over the time step.

        /*

        //// TODO
        ////   - figure out how we get the current states (x and v) for the two contactables
        ////     (what new methods do we need on a ChContactable?)
        ////   - call CalculateForce at perturbed states and calculate F-D approximate Jacobian column

        //// TODO
        ////   - how do we deal with quaternion states?!?

        // Get states for objA
        int ndofA_x = this->objA->ContactableGet_ndof_x();
        int ndofA_w = this->objA->ContactableGet_ndof_w();
        ChState stateA_x(ndofA_x);
        ChStateDelta stateA_w(ndofA_w);
        this->objA->ContactableGetStateBlock_x(stateA_x);
        this->objA->ContactableGetStateBlock_w(stateA_w);

        // Get states for objB
        int ndofB_x = this->objB->ContactableGet_ndof_x();
        int ndofB_w = this->objB->ContactableGet_ndof_w();
        ChState stateB_x(ndofB_x);
        ChStateDelta stateB_w(ndofB_w);
        this->objB->ContactableGetStateBlock_x(stateB_x);
        this->objB->ContactableGetStateBlock_w(stateB_w);

        // Compute Q at current state
        ChVectorDynamic<> Q0(ndofA_w + ndofB_w);
        CalculateQ(stateA_x, stateA_w, stateB_x, stateB_w, Q0);

        double perturbation = 1e-8;
        ChVectorDynamic<> Q1(ndofA_w + ndofB_w);
        ChVectorDynamic<> Jcolumn(ndofA_w + ndofB_w);

        // Jacobian w.r.t. positions of objA
        for (int i = 0; i < ndofA_x; i++) {
            stateA_x(i) += perturbation;
            CalculateQ(stateA_x, stateA_w, stateB_x, stateB_w, Q1);
            stateA_x(i) -= perturbation;

            Jcolumn = (Q1 - Q0) * (-1 / perturbation);  // note sign change
            m_Jac->m_K.PasteMatrix(&Jcolumn, 0, i);
        }

        // Jacobian w.r.t. positions of objB
        for (int i = 0; i < ndofB_x; i++) {
            stateB_x(i) += perturbation;
            CalculateQ(stateA_x, stateA_w, stateB_x, stateB_w, Q1);
            stateB_x(i) -= perturbation;

            Jcolumn = (Q1 - Q0) * (-1 / perturbation);  // note sign change
            m_Jac->m_K.PasteMatrix(&Jcolumn, 0, ndofA_x + i);
        }

        // Jacobian w.r.t. velocities of objA
        for (int i = 0; i < ndofA_w; i++) {
            stateA_w(i) += perturbation;
            CalculateQ(stateA_x, stateA_w, stateB_x, stateB_w, Q1);
            stateA_w(i) -= perturbation;

            Jcolumn = (Q1 - Q0) * (-1 / perturbation);  // note sign change
            m_Jac->m_R.PasteMatrix(&Jcolumn, 0, i);
        }

        // Jacobian w.r.t. velocities of objB
        for (int i = 0; i < ndofB_w; i++) {
            stateB_w(i) += perturbation;
            CalculateQ(stateA_x, stateA_w, stateB_x, stateB_w, Q1);
            stateB_w(i) -= perturbation;

            Jcolumn = (Q1 - Q0) * (-1 / perturbation);  // note sign change
            m_Jac->m_R.PasteMatrix(&Jcolumn, 0, ndofA_w + i);
        }
        */
    }

    /// Apply contact forces to the two objects.
    /// (new version, for interfacing to ChTimestepper and ChIntegrable)
    virtual void ContIntLoadResidual_F(ChVectorDynamic<>& R, const double c) override {
        ChVector<> abs_force_scaled(m_force * c);

        if (this->objA->IsContactActive())
            this->objA->ContactForceLoadResidual_F(-abs_force_scaled, this->p1, R);

        if (this->objB->IsContactActive())
            this->objB->ContactForceLoadResidual_F(abs_force_scaled, this->p2, R);
    }

    /// Inject Jacobian blocks into the system descriptor.
    /// Tell to a system descriptor that there are item(s) of type ChLcpKblock in this object
    /// (for further passing it to a LCP solver)
    virtual void ContInjectKRMmatrices(ChLcpSystemDescriptor& mdescriptor) override {
        if (m_Jac)
            mdescriptor.InsertKblock(&m_Jac->m_KRM);
    }

    /// Compute Jacobian of contact forces.
    virtual void ContKRMmatricesLoad(double Kfactor, double Rfactor) override {
        if (m_Jac) {
            m_Jac->m_KRM.Get_K()->FillElem(0);

            //// TODO
            ////  - this needs to be changed to take into account dimensions of K & R in KRM.
            ////  - account for quaternion transformation here?

            /*
            m_Jac->m_KRM.Get_K()->MatrInc(m_Jac->m_K * Kfactor);
            m_Jac->m_KRM.Get_K()->MatrInc(m_Jac->m_R * Rfactor);
            */
        }
    }
};

}  // END_OF_NAMESPACE____

#endif
