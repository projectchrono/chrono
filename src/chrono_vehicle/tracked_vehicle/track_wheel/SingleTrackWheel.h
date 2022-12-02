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
// Single track-wheel model constructed with data from file (JSON format).
//
// =============================================================================

#ifndef SINGLE_TRACK_WHEEL_H
#define SINGLE_TRACK_WHEEL_H

#include "chrono_vehicle/ChApiVehicle.h"
#include "chrono_vehicle/tracked_vehicle/track_wheel/ChSingleTrackWheel.h"

#include "chrono_thirdparty/rapidjson/document.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle_tracked_suspension
/// @{

/// Single track-wheel model constructed with data from file (JSON format).
class CH_VEHICLE_API SingleTrackWheel : public ChSingleTrackWheel {
  public:
    SingleTrackWheel(const std::string& filename);
    SingleTrackWheel(const rapidjson::Document& d);
    ~SingleTrackWheel() {}

    virtual double GetRadius() const override { return m_wheel_radius; }
    virtual double GetWidth() const override { return m_wheel_width; }

    virtual double GetMass() const override { return m_wheel_mass; }
    virtual const ChVector<>& GetInertia() override { return m_wheel_inertia; }

  private:
    virtual void Create(const rapidjson::Document& d) override;
    virtual void CreateContactMaterial(ChContactMethod contact_method) override;
    virtual void AddVisualizationAssets(VisualizationType vis) override;

    double m_wheel_radius;
    double m_wheel_width;

    double m_wheel_mass;
    ChVector<> m_wheel_inertia;

    bool m_has_mesh;
    std::string m_meshFile;

    ChContactMaterialData m_mat_info;
};

/// @} vehicle_tracked_suspension

}  // end namespace vehicle
}  // end namespace chrono

#endif
