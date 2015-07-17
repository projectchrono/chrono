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



/// Class for DVI contact between two generic ChContactable objects.
/// Ta and Tb are of ChContactable sub classes.

template <class Ta, class Tb>  
class ChContactDEM : public ChContactTuple<Ta, Tb> {

  public: 
    typedef typename ChContactTuple<Ta, Tb>::typecarr_a typecarr_a;
    typedef typename ChContactTuple<Ta, Tb>::typecarr_b typecarr_b;

  protected:
    //
    // DATA
    //

    ChVector<> m_force;  ///< contact force on body2

  public:

    

    //
    // CONSTRUCTORS
    //

    ChContactDEM() {
    }

    ChContactDEM(
              ChContactContainerBase* mcontainer,
              Ta* mobjA,  ///< collidable object A
              Tb* mobjB,  ///< collidable object B
              const collision::ChCollisionInfo& cinfo
              ) 
        : ChContactTuple< Ta, Tb >(mcontainer, mobjA, mobjB, cinfo)
    {   
        Reset(mobjA, 
              mobjB,
              cinfo);
    }
    
    virtual ~ChContactDEM() {}

    //
    // FUNCTIONS
    //

    /// Initialize again this constraint.
    virtual void Reset(
            Ta* mobjA,  ///< collidable object A
            Tb* mobjB,  ///< collidable object B
            const collision::ChCollisionInfo& cinfo) {
        
        // inherit base class:
        ChContactTuple< Ta, Tb >::Reset(mobjA, mobjB, cinfo);

        assert(cinfo.distance < 0);

        //m_delta = -cinfo.distance; = -this->norm_dist

        // Calculate contact force
        CalculateForce();
    }

 
    /// Get the contact force, if computed, in absolute coordinate system
    virtual ChVector<> GetContactForce() { return this->m_force; };
 
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
        bool use_history   = sys->UseContactHistory();
        ContactForceModel force_model = sys->GetContactForceModel();

        // Relative velocity at contact
        ChVector<> vel2 = this->objB->GetContactPointSpeed(this->p2);
        ChVector<> vel1 = this->objA->GetContactPointSpeed(this->p1);
        ChVector<> relvel = vel2 - vel1;
        double relvel_n_mag = relvel.Dot(this->normal);
        ChVector<> relvel_n = relvel_n_mag * this->normal;
        ChVector<> relvel_t = relvel - relvel_n;
        double relvel_t_mag = relvel_t.Length();

        // Calculate effective mass
        double m_eff = this->objA->GetContactableMass() * this->objB->GetContactableMass() / (this->objA->GetContactableMass() + this->objB->GetContactableMass());

        // Calculate effective contact radius
        //// TODO:  how can I get this with current collision system!?!?!?
        double R_eff = 1;

        // just casting, now, since we are sure that this contact was created only if dynamic casting was fine
        ChSharedPtr<ChMaterialSurfaceDEM> mmatA = this->objA->GetMaterialSurfaceBase().template DynamicCastTo<ChMaterialSurfaceDEM>();
        ChSharedPtr<ChMaterialSurfaceDEM> mmatB = this->objB->GetMaterialSurfaceBase().template DynamicCastTo<ChMaterialSurfaceDEM>();

        // Calculate composite material properties
        ChCompositeMaterialDEM mat =
            ChMaterialSurfaceDEM::CompositeMaterial(mmatA, mmatB);

        // Contact forces.
        // All models use the following formulas for normal and tangential forces:
        //     Fn = kn * delta_n - gn * v_n
        //     Ft = kt * delta_t - gt * v_t
        double kn;
        double kt;
        double gn;
        double gt;

        double delta_t = use_history ? relvel_t_mag * dT : 0;

        switch (force_model) {
            case Hooke:
                if (use_mat_props) {
                    double tmp_k = (16.0 / 15) * std::sqrt(R_eff) * mat.E_eff;
                    double v2 = sys->GetCharacteristicImpactVelocity() * sys->GetCharacteristicImpactVelocity();
                    double tmp_g = 1 + std::pow(CH_C_PI / std::log(mat.cr_eff), 2);
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

            case Hertz:
                if (use_mat_props) {
                    double sqrt_Rd = std::sqrt(R_eff * m_delta);
                    double Sn = 2 * mat.E_eff * sqrt_Rd;
                    double St = 8 * mat.G_eff * sqrt_Rd;
                    double loge = std::log(mat.cr_eff);
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

        // Include cohesion force
        forceN -= mat.cohesion_eff;

        // Coulomb law
        forceT = std::min<double>(forceT, mat.mu_eff * std::abs(forceN));

        // Accumulate normal and tangential forces
        m_force = forceN * this->normal;
        if (relvel_t_mag >= sys->GetSlipVelocitythreshold())
            m_force -= (forceT / relvel_t_mag) * relvel_t;
    }




    /// Apply contact forces to bodies (new version, for interfacing to ChTimestepper and ChIntegrable)
    virtual void ContIntLoadResidual_F(ChVectorDynamic<>& R, const double c) {

        ChVector<> abs_force_scaled( m_force*c );

        if (this->objA->IsContactActive())
            this->objA->ContactForceLoadResidual_F(-abs_force_scaled, this->p1, R);

        if (this->objB->IsContactActive())
            this->objB->ContactForceLoadResidual_F( abs_force_scaled, this->p2, R);
    }

    // other timestepper interfaces
    virtual void ContIntStateGatherReactions(const unsigned int off_L, ChVectorDynamic<>& L) {};
    virtual void ContIntStateScatterReactions(const unsigned int off_L, const ChVectorDynamic<>& L) {};
    virtual void ContIntLoadResidual_CqL(const unsigned int off_L,    ///< offset in L multipliers
                                 ChVectorDynamic<>& R,        ///< result: the R residual, R += c*Cq'*L
                                 const ChVectorDynamic<>& L,  ///< the L vector
                                 const double c               ///< a scaling factor
                                 ) {};
    virtual void ContIntLoadConstraint_C(const unsigned int off_L,  ///< offset in Qc residual
                                 ChVectorDynamic<>& Qc,     ///< result: the Qc residual, Qc += c*C
                                 const double c,            ///< a scaling factor
                                 bool do_clamp,             ///< apply clamping to c*C?
                                 double recovery_clamp      ///< value for min/max clamping of c*C
                                 ) {};
    virtual void ContIntToLCP(const unsigned int off_L,  ///< offset in L, Qc
                      const ChVectorDynamic<>& L,
                      const ChVectorDynamic<>& Qc) {};
    virtual void ContIntFromLCP(const unsigned int off_L,  ///< offset in L
                      ChVectorDynamic<>& L) {};

    virtual void InjectConstraints(ChLcpSystemDescriptor& mdescriptor)  {};

    virtual void ConstraintsBiReset() {};
    virtual void ConstraintsBiLoad_C(double factor = 1., double recovery_clamp = 0.1, bool do_clamp = false) {};
    virtual void ConstraintsFetch_react(double factor) {};
    virtual void ConstraintsLiLoadSuggestedSpeedSolution() {};
    virtual void ConstraintsLiLoadSuggestedPositionSolution() {};
    virtual void ConstraintsLiFetchSuggestedSpeedSolution() {};
    virtual void ConstraintsLiFetchSuggestedPositionSolution() {};
    virtual void ConstraintsFbLoadForces(double factor) {
        GetLog() << "ConstraintsFbLoadForces NOT SUPPORTED - OBSOLETE - use new bookkeeping \n";
    }
    //***OBSOLETE*** moved to ChSystemDEM
    //static void SetSlipVelocitythreshold(double vel) { m_minSlipVelocity = vel; }
    //static void SetCharacteristicImpactVelocity(double vel) { m_characteristicVelocity = vel; }


};



}  // END_OF_NAMESPACE____

#endif
