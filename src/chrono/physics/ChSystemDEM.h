// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All right reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban, Alessandro Tasora
// =============================================================================
//
// Physical system in which contact is modeled using a Penalty Method.
//
// =============================================================================

#ifndef CHSYSTEMDEM_H
#define CHSYSTEMDEM_H

#include <algorithm>

#include "chrono/physics/ChSystem.h"


namespace chrono {

/// Class for a physical system in which contact is modeled using a Penalty Method.
class ChApi ChSystemDEM : public ChSystem {

    // Tag needed for class factory in archive (de)serialization:
    CH_FACTORY_TAG(ChSystemDEM)

  public:
    /// Enum for DEM contact type.
    enum ContactForceModel {
        Hooke,        ///< linear Hookean model
        Hertz,        ///< nonlinear Hertzian model
        PlainCoulomb  ///< basic tangential force definition for non-granular bodies
    };

    /// Enum for adhesion force model.
    enum AdhesionForceModel {
        Constant,  ///< constant adhesion force
        DMT        ///< Derjagin-Muller-Toropov model
    };

    /// Enum for tangential displacement model.
    enum TangentialDisplacementModel {
        None,      ///< no tangential force
        OneStep,   ///< use only current relative tangential velocity
        MultiStep  ///< use contact history (from contact initiation)
    };

    /// Constructor for ChSystemDEM.
    /// Note that, in case you will use collision detection, the values of
    /// 'max_objects' and 'scene_size' can be used to initialize the broadphase
    /// collision algorithm in an optimal way. Scene size should be approximately
    /// the radius of the expected area where colliding objects will move.
    ChSystemDEM(bool use_material_properties = true,  ///< use physical contact material properties
                unsigned int max_objects = 16000,     ///< maximum number of contactable objects
                double scene_size = 500               ///< approximate bounding radius of the scene
                );

    virtual ~ChSystemDEM() {}

    /// "Virtual" copy constructor (covariant return type).
    virtual ChSystemDEM* Clone() const override { return new ChSystemDEM(*this); }

    virtual ChMaterialSurfaceBase::ContactMethod GetContactMethod() const override { return ChMaterialSurfaceBase::DEM; }

    /// Create a new body, consistent with the contact method and collision model used by this system.
    /// The returned body is not added to the system.
    virtual ChBody* NewBody() override { return new ChBody(ChMaterialSurfaceBase::DEM); }

    /// Create a new body with non-centroidal reference frame, consistent with the contact method and
    /// collision model used by this system.  The returned body is not added to the system.
    virtual ChBodyAuxRef* NewBodyAuxRef() override { return new ChBodyAuxRef(ChMaterialSurfaceBase::DEM); }

    virtual void SetSolverType(ChSolver::Type type) override;

    /// Replace the contact container. 
    /// The provided container object must be inherited from ChContactContainerDEM.
    virtual void SetContactContainer(std::shared_ptr<ChContactContainerBase>  container) override;

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
    void SetSlipVelocitythreshold(double vel) { m_minSlipVelocity = std::max(vel, CH_MICROTOL); }
    double GetSlipVelocitythreshold() const {return m_minSlipVelocity;}

    /// Characteristic impact velocity (Hooke contact force model).
    void SetCharacteristicImpactVelocity(double vel) { m_characteristicVelocity = vel; }
    double GetCharacteristicImpactVelocity() const {return m_characteristicVelocity;}

    //
    // SERIALIZATION
    //

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
};

CH_CLASS_VERSION(ChSystemDEM,0)

}  // end namespace chrono

#endif
