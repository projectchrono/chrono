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
// Track driveline model template based on ChShaft objects using data from file
// (JSON format). 
//
// =============================================================================

#ifndef TRACK_DRIVELINE_BDS_H
#define TRACK_DRIVELINE_BDS_H

#include "chrono_vehicle/ChApiVehicle.h"
#include "chrono_vehicle/tracked_vehicle/driveline/ChTrackDrivelineBDS.h"

#include "chrono_thirdparty/rapidjson/document.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle_tracked_driveline
/// @{

/// BDS tracked vehicle driveline model template using data from file (JSON format).
class CH_VEHICLE_API TrackDrivelineBDS : public ChTrackDrivelineBDS {
  public:
    TrackDrivelineBDS(const std::string& filename);
    TrackDrivelineBDS(const rapidjson::Document& d);
    ~TrackDrivelineBDS() {}

    virtual double GetDriveshaftInertia() const override { return m_driveshaft_inertia; }
    virtual double GetDifferentialBoxInertia() const override { return m_differentialbox_inertia; }

    virtual double GetConicalGearRatio() const override { return m_conicalgear_ratio; }
    virtual double GetDifferentialRatio() const override { return m_differential_ratio; }

  private:
    virtual void Create(const rapidjson::Document& d) override;

    // Shaft inertias.
    double m_driveshaft_inertia;
    double m_differentialbox_inertia;

    // Gear ratios.
    double m_conicalgear_ratio;
    double m_differential_ratio;
};

/// @} vehicle_tracked_driveline

}  // end namespace vehicle
}  // end namespace chrono

#endif
