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
// Physical system in which contact is modeled using a smooth (penalty-based)
// method.
//
// =============================================================================

#ifndef CH_SYSTEM_SMC_H
#define CH_SYSTEM_SMC_H

#include <algorithm>

#include "chrono/physics/ChSystem.h"

namespace chrono {

/// Class for a physical system in which contact is modeled using a smooth (penalty-based) method.
class ChApi ChSystemSMC : public ChSystem {
  public:
    /// Enum for SMC contact type.
    enum ContactForceModel {
        Hooke,         ///< linear Hookean model
        Hertz,         ///< nonlinear Hertzian model
        PlainCoulomb,  ///< basic tangential force definition for non-granular bodies
        Flores         ///< nonlinear Hertzian model
    };

    /// Enum for adhesion force model.
    enum AdhesionForceModel {
        Constant,  ///< constant adhesion force
        DMT,       ///< Derjagin-Muller-Toropov model
        Perko      ///< Perko et al. (2001) model
    };

    /// Enum for tangential displacement model.
    enum TangentialDisplacementModel {
        None,      ///< no tangential force
        OneStep,   ///< use only current relative tangential velocity
        MultiStep  ///< use contact history (from contact initiation)
    };

    /// Constructor for ChSystemSMC.
    ChSystemSMC(bool use_material_properties = true);

    /// Copy constructor
    ChSystemSMC(const ChSystemSMC& other);

    virtual ~ChSystemSMC() {}

    /// "Virtual" copy constructor (covariant return type).
    virtual ChSystemSMC* Clone() const override { return new ChSystemSMC(*this); }

    /// Return the contact method supported by this system.
    virtual ChContactMethod GetContactMethod() const override { return ChContactMethod::SMC; }

    /// Replace the contact container.
    /// The provided container object must be inherited from ChContactContainerSMC.
    virtual void SetContactContainer(std::shared_ptr<ChContactContainer> container) override;

    /// Enable/disable using physical contact material properties.
    /// If true, contact coefficients are estimated from physical material properties.
    /// Otherwise, explicit values of stiffness and damping coefficients are used.
    void UseMaterialProperties(bool val) { m_use_mat_props = val; }
    /// Return true if contact coefficients are estimated from physical material properties.
    bool UsingMaterialProperties() const { return m_use_mat_props; }

    /// Set the normal contact force model.
    void SetContactForceModel(ContactForceModel model) { m_contact_model = model; }
    /// Get the current normal contact force model.
    ContactForceModel GetContactForceModel() const { return m_contact_model; }

    /// Set the adhesion force model.
    void SetAdhesionForceModel(AdhesionForceModel model) { m_adhesion_model = model; }
    /// Get the current adhesion force model.
    AdhesionForceModel GetAdhesionForceModel() const { return m_adhesion_model; }

    /// Set the tangential displacement model.
    /// Note that currently MultiStep falls back to OneStep.
    void SetTangentialDisplacementModel(TangentialDisplacementModel model) { m_tdispl_model = model; }
    /// Get the current tangential displacement model.
    TangentialDisplacementModel GetTangentialDisplacementModel() const { return m_tdispl_model; }

    /// Declare the contact forces as stiff.
    /// If true, this enables calculation of contact force Jacobians.
    void SetStiffContact(bool val) { m_stiff_contact = val; }
    bool GetStiffContact() const { return m_stiff_contact; }

    /// Slip velocity threshold.
    /// No tangential contact forces are generated if the magnitude of the tangential
    /// relative velocity is below this value.
    void SetSlipVelocityThreshold(double vel);
    double GetSlipVelocityThreshold() const { return m_minSlipVelocity; }

    /// Characteristic impact velocity (Hooke contact force model).
    void SetCharacteristicImpactVelocity(double vel) { m_characteristicVelocity = vel; }
    double GetCharacteristicImpactVelocity() const { return m_characteristicVelocity; }

    /// Base class for contact force calculation.
    /// A user can override thie default implementation by attaching a custom derived class; see
    /// SetContactForceAlgorithm.
    class ChApi ChContactForceSMC {
      public:
        virtual ~ChContactForceSMC() {}

        /// Calculate contact force (resultant of both normal and tangential components) for a contact between two
        /// objects, obj1 and obj2. Note that this function is always called with delta > 0.
        virtual ChVector<> CalculateForce(
            const ChSystemSMC& sys,             ///< containing system
            const ChVector<>& normal_dir,       ///< normal contact direction (expressed in global frame)
            const ChVector<>& p1,               ///< most penetrated point on obj1 (expressed in global frame)
            const ChVector<>& p2,               ///< most penetrated point on obj2 (expressed in global frame)
            const ChVector<>& vel1,             ///< velocity of contact point on obj1 (expressed in global frame)
            const ChVector<>& vel2,             ///< velocity of contact point on obj2 (expressed in global frame)
            const ChMaterialCompositeSMC& mat,  ///< composite material for contact pair
            double delta,                       ///< overlap in normal direction
            double eff_radius,                  ///< effective radius of curvature at contact
            double mass1,                       ///< mass of obj1
            double mass2                        ///< mass of obj2
            ) const = 0;
    };

    /// Change the default SMC contact force calculation.
    virtual void SetContactForceAlgorithm(std::unique_ptr<ChContactForceSMC>&& algorithm);

    /// Accessor for the current SMC contact force calculation.
    const ChContactForceSMC& GetContactForceAlgorithm() const { return *m_force_algo; }

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOUT(ChArchiveOut& marchive) override;

    /// Method to allow deserialization of transient data from archives.
    virtual void ArchiveIN(ChArchiveIn& marchive) override;

  private:
    bool m_use_mat_props;                        ///< if true, derive contact parameters from mat. props.
    ContactForceModel m_contact_model;           ///< type of the contact force model
    AdhesionForceModel m_adhesion_model;         ///< type of the adhesion force model
    TangentialDisplacementModel m_tdispl_model;  ///< type of tangential displacement model
    bool m_stiff_contact;                        ///< flag indicating stiff contacts (triggers Jacobian calculation)
    double m_minSlipVelocity;                    ///< slip velocity below which no tangential forces are generated
    double m_characteristicVelocity;             ///< characteristic impact velocity (Hooke model)
    std::unique_ptr<ChContactForceSMC> m_force_algo;  /// contact force calculation
};

CH_CLASS_VERSION(ChSystemSMC, 0)

}  // end namespace chrono

#endif
