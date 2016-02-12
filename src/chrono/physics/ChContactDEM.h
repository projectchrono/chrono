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

#include "core/ChFrame.h"
#include "core/ChVectorDynamic.h"
#include "lcp/ChLcpSystemDescriptor.h"
#include "collision/ChCCollisionModel.h"
#include "physics/ChContactTuple.h"
#include "physics/ChContactContainerBase.h"
#include "physics/ChMaterialSurfaceDEM.h"
#include "physics/ChSystemDEM.h"
#include <cmath>

namespace chrono {

/// Class for DEM contact between two generic ChContactable objects.
/// Ta and Tb are of ChContactable sub classes.

template <class Ta, class Tb>
class ChContactDEM : public ChContactTuple<Ta, Tb> {
  public:
    typedef typename ChContactTuple<Ta, Tb>::typecarr_a typecarr_a;
    typedef typename ChContactTuple<Ta, Tb>::typecarr_b typecarr_b;

  protected:
    ChVector<> m_force;  ///< contact force on body2

  public:
    //
    // CONSTRUCTORS
    //

    ChContactDEM() {}

    ChContactDEM(ChContactContainerBase* mcontainer,      ///< contact container
                 Ta* mobjA,                               ///< collidable object A
                 Tb* mobjB,                               ///< collidable object B
                 const collision::ChCollisionInfo& cinfo  ///< data for the contact pair
                 )
        : ChContactTuple<Ta, Tb>(mcontainer, mobjA, mobjB, cinfo) {
        Reset(mobjA, mobjB, cinfo);
    }

    ~ChContactDEM() {}

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

        assert(cinfo.distance < 0);

        // m_delta = -cinfo.distance; = -this->norm_dist

        // Calculate contact force
        CalculateForce();
    }

    /// Get the contact force, if computed, in absolute coordinate system
    virtual ChVector<> GetContactForce() { return this->m_force; }

    /// Get the contact penetration (positive if there is overlap).
    double GetContactPenetration() const { return -this->norm_dist; }

    /// Get the contact force, expressed in the frame of the contact.
    ChVector<> GetContactForceLocal() const { return this->contact_plane.MatrT_x_Vect(this->m_force); }

    /// Calculate contact force, expressed in absolute coordinates.
    void CalculateForce() {
        double m_delta = -this->norm_dist;

        ChSystemDEM* sys = static_cast<ChSystemDEM*>(this->container->GetSystem());

        double dT = sys->GetStep();
        bool use_mat_props = sys->UseMaterialProperties();
        bool use_history = sys->UseContactHistory();
        ChSystemDEM::ContactForceModel contact_model = sys->GetContactForceModel();
        ChSystemDEM::AdhesionForceModel adhesion_model = sys->GetAdhesionForceModel();


        // Relative velocity at contact
        ChVector<> vel2 = this->objB->GetContactPointSpeed(this->p2);
        ChVector<> vel1 = this->objA->GetContactPointSpeed(this->p1);
        ChVector<> relvel = vel2 - vel1;
        double relvel_n_mag = relvel.Dot(this->normal);
        ChVector<> relvel_n = relvel_n_mag * this->normal;
        ChVector<> relvel_t = relvel - relvel_n;
        double relvel_t_mag = relvel_t.Length();

        // Calculate effective mass
        double m_eff = this->objA->GetContactableMass() * this->objB->GetContactableMass() /
                       (this->objA->GetContactableMass() + this->objB->GetContactableMass());

        // Calculate effective contact radius
        //// TODO:  how can I get this with current collision system!?!?!?
        double R_eff = 1;

        // just casting, now, since we are sure that this contact was created only if dynamic casting was fine
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
                    double sqrt_Rd = std::sqrt(R_eff * m_delta);
                    double Sn = 2 * mat.E_eff * sqrt_Rd;
                    double St = 8 * mat.G_eff * sqrt_Rd;
                    double loge = (mat.cr_eff < CH_MICROTOL) ? std::log(CH_MICROTOL) : std::log(mat.cr_eff);
                    double beta = loge / std::sqrt(loge * loge + CH_C_PI * CH_C_PI);
                    kn = (2.0 / 3) * Sn;
                    kt = St;
                    gn = -2 * std::sqrt(5.0 / 6) * beta * std::sqrt(Sn * m_eff);
                    gt = -2 * std::sqrt(5.0 / 6) * beta * std::sqrt(St * m_eff);
                } else {
                    double tmp = R_eff * std::sqrt(m_delta);
                    kn = tmp * mat.kn;
                    kt = tmp * mat.kt;
                    gn = tmp * m_eff * mat.gn;
                    gt = tmp * m_eff * mat.gt;
                }

                break;
        }

        // Calculate the magnitudes of the normal and tangential contact forces
        double forceN = kn * m_delta - gn * relvel_n_mag;
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
        m_force = forceN * this->normal;
        if (relvel_t_mag >= sys->GetSlipVelocitythreshold())
            m_force -= (forceT / std::max(relvel_t_mag, CH_MICROTOL)) * relvel_t;
    }

    /// Apply contact forces to bodies (new version, for interfacing to ChTimestepper and ChIntegrable)
    virtual void ContIntLoadResidual_F(ChVectorDynamic<>& R, const double c) {
        ChVector<> abs_force_scaled(m_force * c);

        if (this->objA->IsContactActive())
            this->objA->ContactForceLoadResidual_F(-abs_force_scaled, this->p1, R);

        if (this->objB->IsContactActive())
            this->objB->ContactForceLoadResidual_F(abs_force_scaled, this->p2, R);
    }

    //***OBSOLETE*** moved to ChSystemDEM
    // static void SetSlipVelocitythreshold(double vel) { m_minSlipVelocity = vel; }
    // static void SetCharacteristicImpactVelocity(double vel) { m_characteristicVelocity = vel; }
};

}  // END_OF_NAMESPACE____

#endif
