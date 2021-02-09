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
// Authors: Radu Serban
// =============================================================================
//
// Vehicle rigid chassis model constructed with data from file (JSON format).
//
// =============================================================================

#ifndef RIGID_CHASSIS_H
#define RIGID_CHASSIS_H

#include "chrono_vehicle/ChApiVehicle.h"
#include "chrono_vehicle/chassis/ChRigidChassis.h"

#include "chrono_thirdparty/rapidjson/document.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle
/// @{

/// Vehicle rigid chassis model constructed with data from file (JSON format).
class CH_VEHICLE_API RigidChassis : public ChRigidChassis {
  public:
    RigidChassis(const std::string& filename);
    RigidChassis(const rapidjson::Document& d);
    ~RigidChassis() {}

    /// Return the mass of the chassis body.
    virtual double GetMass() const override { return m_mass; }

    /// Return the inertia tensor of the chassis body.
    virtual const ChMatrix33<>& GetInertia() const override { return m_inertia; }

    /// Get the location of the center of mass in the chassis frame.
    virtual const ChVector<>& GetLocalPosCOM() const override { return m_COM_loc; }

    /// Get the location (in the local frame of this chassis) of the connection to a rear chassis.
    virtual const ChVector<> GetLocalPosRearConnector() const override { return m_connector_rear_loc; }

    /// Get the local driver position and orientation.
    /// This is a coordinate system relative to the chassis reference frame.
    virtual ChCoordsys<> GetLocalDriverCoordsys() const override { return m_driverCsys; }

  private:
    virtual void Create(const rapidjson::Document& d) override;
    virtual void CreateContactMaterials(ChContactMethod contact_method) override;

    double m_mass;                    ///< chassis mass
    ChMatrix33<> m_inertia;           ///< chassis inertia tensor, w.r.t. centroidal frame
    ChVector<> m_COM_loc;             ///< location of the chassis COM in the chassis reference frame
    ChVector<> m_connector_rear_loc;  ///< location of connector to a potential rear chassis
    ChCoordsys<> m_driverCsys;        ///< driver position and orientation relative to chassis

    std::vector<MaterialInfo> m_mat_info;
};

/// Vehicle rigid rear chassis model constructed with data from file (JSON format).
class CH_VEHICLE_API RigidChassisRear : public ChRigidChassisRear {
  public:
    RigidChassisRear(const std::string& filename);
    RigidChassisRear(const rapidjson::Document& d);
    ~RigidChassisRear() {}

    /// Return the mass of the chassis body.
    virtual double GetMass() const override { return m_mass; }

    /// Return the inertia tensor of the chassis body.
    virtual const ChMatrix33<>& GetInertia() const override { return m_inertia; }

    /// Get the location of the center of mass in the chassis frame.
    virtual const ChVector<>& GetLocalPosCOM() const override { return m_COM_loc; }

    /// Get the location (in the local frame of this chassis) of the connection to the front chassis.
    virtual const ChVector<>& GetLocalPosFrontConnector() const override { return m_connector_front_loc; }

    /// Get the location (in the local frame of this chassis) of the connection to a rear chassis.
    virtual const ChVector<> GetLocalPosRearConnector() const override { return m_connector_rear_loc; }

  private:
    virtual void Create(const rapidjson::Document& d) override;
    virtual void CreateContactMaterials(ChContactMethod contact_method) override;

    double m_mass;                     ///< chassis mass
    ChMatrix33<> m_inertia;            ///< chassis inertia tensor, w.r.t. centroidal frame
    ChVector<> m_COM_loc;              ///< location of the chassis COM in the chassis reference frame
    ChVector<> m_connector_front_loc;  ///< location of connector to the front chassis
    ChVector<> m_connector_rear_loc;   ///< location of connector to a potential rear chassis

    std::vector<MaterialInfo> m_mat_info;
};

/// @} vehicle

}  // end namespace vehicle
}  // end namespace chrono

#endif
