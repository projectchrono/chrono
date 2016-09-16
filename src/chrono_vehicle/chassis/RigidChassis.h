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
// Authors: Radu Serban
// =============================================================================
//
// Vehicle rigid chassis model constructed with data from file (JSON format).
//
// =============================================================================

#ifndef SPROCKET_SINGLE_PIN_H
#define SPROCKET_SINGLE_PIN_H

#include "chrono_vehicle/ChApiVehicle.h"
#include "chrono_vehicle/ChChassis.h"

#include "chrono_thirdparty/rapidjson/document.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle
/// @{

/// Vehicle rigid chassis model constructed with data from file(JSON format).
class CH_VEHICLE_API RigidChassis : public ChChassis {
  public:
    RigidChassis(const std::string& filename);
    RigidChassis(const rapidjson::Document& d);
    ~RigidChassis() {}

    /// Return the mass of the chassis body.
    virtual double GetMass() const override { return m_mass; }
    
    /// Return the moments of inertia of the chassis body.
    virtual const ChVector<>& GetInertia() const override { return m_inertia; }
    
    /// Get the location of the center of mass in the chassis frame.
    virtual const ChVector<>& GetLocalPosCOM() const override { return m_COM_loc; }
    
    /// Get the local driver position and orientation.
    /// This is a coordinate system relative to the chassis reference frame.
    virtual ChCoordsys<> GetLocalDriverCoordsys() const override { return m_driverCsys; }

    /// Specifies whether or not a visualization mesh was specified.
    bool HasMesh() const { return m_has_mesh; }

    /// Get the name of the Wavefront file with chassis visualization mesh.
    /// An empty string is returned if no mesh was specified.
    const std::string& GetMeshFilename() const { return m_meshFile; }

    /// Get the name of the chassis visualization mesh asset.
    /// An empty string is returned if no mesh was specified.
    const std::string& GetMeshName() const { return m_meshName; }

    /// Add visualization of the road wheel.
    virtual void AddVisualizationAssets(VisualizationType vis) override;

  private:
    void Create(const rapidjson::Document& d);

    double m_mass;         // chassis mass
    ChVector<> m_inertia;  // moments of inertia of the chassis
    ChVector<> m_COM_loc;  // location of the chassis COM in the chassis reference frame

    ChCoordsys<> m_driverCsys;  // driver position and orientation relative to chassis

    bool m_has_mesh;
    std::string m_meshName;
    std::string m_meshFile;
};

/// @} vehicle

}  // end namespace vehicle
}  // end namespace chrono

#endif
