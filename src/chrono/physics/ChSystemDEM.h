//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2010-2012 Alessandro Tasora
// Copyright (c) 2013 Project Chrono
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//

#ifndef CHSYSTEMDEM_H
#define CHSYSTEMDEM_H

#include "physics/ChSystem.h"


namespace chrono {

/// Class for a physical system in which contact is modeled using a
/// Penalty Method (aka DEM)
class ChApi ChSystemDEM : public ChSystem {
    CH_RTTI(ChSystemDEM, ChSystem);

  public:
    /// Enum for DEM contact type
    enum ContactForceModel {
        Hooke,  ///< linear Hookean model
        Hertz   ///< nonlinear Hertzian model
    };

    /// Enum for adhesion force model
    enum AdhesionForceModel {
        Constant,  ///< constant adhesion force
        DMT        ///< Derjagin-Muller-Toropov model
    };

    /// Enum for tangential displacement model
    enum TangentialDisplacementModel {
        None,      ///< no tangential force
        OneStep,   ///< use only current relative tangential velocity
        MultiStep  ///< use contact history (from contact initiation)
    };

    /// Constructor
    /// Note, in case you will use collision detection, the values of
    /// 'max_objects' and 'scene_size' can be used to initialize the broadphase
    /// collision algorithm in an optimal way. Scene size should be approximately
    /// the radius of the expected area where colliding objects will move.
    ChSystemDEM(bool use_material_properties = true,
                bool use_history = true,
                unsigned int max_objects = 16000,
                double scene_size = 500);

    virtual ~ChSystemDEM() {}

    virtual ChMaterialSurfaceBase::ContactMethod GetContactMethod() const { return ChMaterialSurfaceBase::DEM; }
    virtual ChBody* NewBody() override { return new ChBody(ChMaterialSurfaceBase::DEM); }
    virtual ChBodyAuxRef* NewBodyAuxRef() override { return new ChBodyAuxRef(ChMaterialSurfaceBase::DEM); }

    virtual void SetLcpSolverType(eCh_lcpSolver mval);
    // virtual void ChangeLcpSolverSpeed(ChLcpSolver* newsolver);
    virtual void ChangeContactContainer(std::shared_ptr<ChContactContainerBase>  newcontainer);

    bool UseMaterialProperties() const { return m_use_mat_props; }
    bool UseContactHistory() const { return m_use_history; }
    void SetContactForceModel(ContactForceModel model) { m_contact_model = model; }
    ContactForceModel GetContactForceModel() const { return m_contact_model; }

    void SetAdhesionForceModel(AdhesionForceModel model) { m_adhesion_model = model; }
    AdhesionForceModel GetAdhesionForceModel() const { return m_adhesion_model; }


    /// Slip velocity threshold. No tangential contact forces are generated
    /// if the magnitude of the tangential relative velocity is below this.
    void SetSlipVelocitythreshold(double vel) { m_minSlipVelocity = vel; }
    double GetSlipVelocitythreshold()  {return m_minSlipVelocity;}

    /// Characteristic impact velocity (Hooke)
    void SetCharacteristicImpactVelocity(double vel) { m_characteristicVelocity = vel; }
    double GetCharacteristicImpactVelocity() {return m_characteristicVelocity;}

    //
    // SERIALIZATION
    //

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOUT(ChArchiveOut& marchive);

    /// Method to allow deserialization of transient data from archives.
    virtual void ArchiveIN(ChArchiveIn& marchive);

  private:
    bool m_use_mat_props;
    bool m_use_history;
    ContactForceModel m_contact_model;
    AdhesionForceModel m_adhesion_model;
    double m_minSlipVelocity;
    double m_characteristicVelocity;
};

}  // end namespace chrono

#endif
