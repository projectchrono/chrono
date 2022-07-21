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
// Double roller model constructed with data from file (JSON format).
//
// =============================================================================

#ifndef DOUBLE_ROLLER_H
#define DOUBLE_ROLLER_H

#include "chrono_vehicle/ChApiVehicle.h"
#include "chrono_vehicle/tracked_vehicle/roller/ChDoubleRoller.h"

#include "chrono_thirdparty/rapidjson/document.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle_tracked_roller
/// @{

/// Double roller model constructed with data from file (JSON format).
class CH_VEHICLE_API DoubleRoller : public ChDoubleRoller {
  public:
    DoubleRoller(const std::string& filename);
    DoubleRoller(const rapidjson::Document& d);
    ~DoubleRoller() {}

    virtual double GetRadius() const override { return m_roller_radius; }
    virtual double GetWidth() const override { return m_roller_width; }
    virtual double GetGap() const override { return m_roller_gap; }

    virtual double GetRollerMass() const override { return m_roller_mass; }
    virtual const ChVector<>& GetRollerInertia() const override { return m_roller_inertia; }

  private:
    virtual void Create(const rapidjson::Document& d) override;
    virtual void CreateContactMaterial(ChContactMethod contact_method) override;
    virtual void AddVisualizationAssets(VisualizationType vis) override;

    double m_roller_radius;
    double m_roller_width;
    double m_roller_gap;

    double m_roller_mass;
    ChVector<> m_roller_inertia;

    bool m_has_mesh;
    std::string m_meshFile;

    MaterialInfo m_mat_info;
};

/// @} vehicle_tracked_roller

}  // end namespace vehicle
}  // end namespace chrono

#endif
