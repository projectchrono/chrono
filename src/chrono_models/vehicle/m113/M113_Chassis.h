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
// M113 chassis subsystem.
//
// =============================================================================

#ifndef M113_CHASSIS_H
#define M113_CHASSIS_H

#include <string>

#include "chrono_vehicle/ChChassis.h"

#include "chrono_models/ChApiModels.h"

namespace chrono {
namespace vehicle {
namespace m113 {

/// M113 chassis subsystem.
class CH_MODELS_API M113_Chassis : public ChChassis {
  public:
    M113_Chassis(const std::string& name, bool fixed = false);
    ~M113_Chassis() {}

    /// Return the mass of the chassis body.
    virtual double GetMass() const override { return m_mass; }

    /// Return the inertia tensor of the chassis body.
    virtual const ChMatrix33<>& GetInertia() const override { return m_inertia; }

    /// Get the location of the center of mass in the chassis frame.
    virtual const ChVector<>& GetLocalPosCOM() const override { return m_COM_loc; }

    /// Get the local driver position and orientation.
    /// This is a coordinate system relative to the chassis reference frame.
    virtual ChCoordsys<> GetLocalDriverCoordsys() const override { return m_driverCsys; }

    /// Add visualization of the road wheel.
    virtual void AddVisualizationAssets(VisualizationType vis) override;

  protected:
    ChMatrix33<> m_inertia;

    static const double m_mass;
    static const ChVector<> m_inertiaXX;
    static const ChVector<> m_inertiaXY;
    static const ChVector<> m_COM_loc;
    static const ChCoordsys<> m_driverCsys;

    static const std::string m_meshName;
    static const std::string m_meshFile;
};

}  // end namespace m113
}  // end namespace vehicle
}  // end namespace chrono

#endif
