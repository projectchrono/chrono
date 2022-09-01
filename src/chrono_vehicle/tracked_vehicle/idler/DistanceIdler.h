// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2022 projectchrono.org
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
// Distance idler model constructed with data from file (JSON format).
//
// =============================================================================

#ifndef DISTANCE_IDLER_H
#define DISTANCE_IDLER_H

#include "chrono_vehicle/ChApiVehicle.h"
#include "chrono_vehicle/tracked_vehicle/idler/ChDistanceIdler.h"

#include "chrono_thirdparty/rapidjson/document.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle_tracked_idler
/// @{

/// Distance idler model constructed with data from file (JSON format).
class CH_VEHICLE_API DistanceIdler : public ChDistanceIdler {
  public:
    DistanceIdler(const std::string& filename);
    DistanceIdler(const rapidjson::Document& d);
    ~DistanceIdler() {}

    virtual double GetCarrierMass() const override { return m_carrier_mass; }
    virtual const ChVector<>& GetCarrierInertia() override { return m_carrier_inertia; }
    virtual double GetCarrierVisRadius() const override { return m_carrier_vis_radius; }

    virtual double GetTensionerExtensionTime() const override { return m_tensioner_time; }
    virtual double GetTensionerDistance() const { return m_tensioner_dist; }

  private:
    virtual const ChVector<> GetLocation(PointId which) override { return m_points[which]; }

    virtual void Create(const rapidjson::Document& d) override;

    ChVector<> m_points[NUM_POINTS];

    double m_carrier_mass;
    ChVector<> m_carrier_inertia;

    double m_carrier_vis_radius;

    double m_pitch_angle;

    double m_tensioner_time;
    double m_tensioner_dist;
};

/// @} vehicle_tracked_idler

}  // end namespace vehicle
}  // end namespace chrono

#endif
