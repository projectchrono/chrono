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

#include "chrono/physics/ChSystem.h"


namespace chrono {

/// Class for a physical system in which contact is modeled using a Penalty Method.
class ChApi ChSystemDEM : public ChSystem {
    CH_RTTI(ChSystemDEM, ChSystem);

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
    /// Note, in case you will use collision detection, the values of
    /// 'max_objects' and 'scene_size' can be used to initialize the broadphase
    /// collision algorithm in an optimal way. Scene size should be approximately
    /// the radius of the expected area where colliding objects will move.
    ChSystemDEM(bool use_material_properties = true,  ///< use physical contact material properties
                bool use_history = true,              ///< use tangential stiffness in contact calculation
                unsigned int max_objects = 16000,     ///< maximum number of contactable objects
                double scene_size = 500               ///< approximate bounding radius of the scene
                );

    virtual ~ChSystemDEM() {}

    virtual ChMaterialSurfaceBase::ContactMethod GetContactMethod() const { return ChMaterialSurfaceBase::DEM; }
    virtual ChBody* NewBody() override { return new ChBody(ChMaterialSurfaceBase::DEM); }
    virtual ChBodyAuxRef* NewBodyAuxRef() override { return new ChBodyAuxRef(ChMaterialSurfaceBase::DEM); }

    virtual void SetLcpSolverType(eCh_lcpSolver mval);
    // virtual void ChangeLcpSolverSpeed(ChLcpSolver* newsolver);
    virtual void ChangeContactContainer(std::shared_ptr<ChContactContainerBase>  newcontainer);

    /// Enable/disable using physical contact material properties.
    /// If true, contact coefficients are estimated from physical material properties. Otherwise,
    /// explicit values of stiffness and damping coefficients are used.
    void UseMaterialProperties(bool val) { m_use_mat_props = val; }
    bool UsingMaterialProperties() const { return m_use_mat_props; }

    /// Enable use of contact tangential stiffness.
    void UseContactHistory(bool val) { m_use_history = val; }
    bool UsingContactHistory() const { return m_use_history; }

    /// Set the normal contact force model.
    void SetContactForceModel(ContactForceModel model) { m_contact_model = model; }
    ContactForceModel GetContactForceModel() const { return m_contact_model; }

    /// Set the adhesion force model.
    void SetAdhesionForceModel(AdhesionForceModel model) { m_adhesion_model = model; }
    AdhesionForceModel GetAdhesionForceModel() const { return m_adhesion_model; }

    /// Declare the contact forces as stiff.
    /// If true, this enables calculation of contact force Jacobians.
    void SetStiffContact(bool val) { m_stiff_contact = val; }
    bool GetStiffContact() const { return m_stiff_contact; }

    /// Slip velocity threshold. 
    /// No tangential contact forces are generated if the magnitude of the tangential
    /// relative velocity is below this value.
    void SetSlipVelocitythreshold(double vel) { m_minSlipVelocity = vel; }
    double GetSlipVelocitythreshold() const {return m_minSlipVelocity;}

    /// Characteristic impact velocity (Hooke contact force model).
    void SetCharacteristicImpactVelocity(double vel) { m_characteristicVelocity = vel; }
    double GetCharacteristicImpactVelocity() const {return m_characteristicVelocity;}

    //
    // SERIALIZATION
    //

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOUT(ChArchiveOut& marchive);

    /// Method to allow deserialization of transient data from archives.
    virtual void ArchiveIN(ChArchiveIn& marchive);

  private:
    bool m_use_mat_props;                 ///< flag indicating if contact parameters are based on physical mat. props.
    bool m_use_history;                   ///< flag indicating whether or not tangential displacement is considered
    ContactForceModel m_contact_model;    ///< type of the contact force model
    AdhesionForceModel m_adhesion_model;  ///< type of the adhesion force model
    bool m_stiff_contact;                 ///< flag indicating stiff contacts (triggers Jacobian calculation)
    double m_minSlipVelocity;             ///< slip velocity below which no tangential forces are generated
    double m_characteristicVelocity;      ///< characteristic impact velocity (Hooke model)
};

}  // end namespace chrono

#endif
