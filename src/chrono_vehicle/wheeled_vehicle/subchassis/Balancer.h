// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2021 projectchrono.org
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
// Balancer subchassis system constructed with data from file.
//
// =============================================================================

#ifndef BALANCER_H
#define BALANCER_H

#include "chrono_vehicle/ChApiVehicle.h"
#include "chrono_vehicle/wheeled_vehicle/subchassis/ChBalancer.h"

#include "chrono_thirdparty/rapidjson/document.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle_wheeled_subchassis
/// @{

/// Balancer subchassis system constructed with data from file.
class CH_VEHICLE_API Balancer : public ChBalancer {
  public:
    Balancer(const std::string& filename);
    Balancer(const rapidjson::Document& d);
    ~Balancer();

    virtual double GetBalancerBeamMass() const override { return m_beam_mass; }
    virtual const ChVector<>& GetBalancerBeamInertia() const override { return m_beam_inertia; }
    virtual const double GetBalancerMaxPitch() const override { return m_beam_max_pitch; }
    virtual const ChVector<>& GetBalancerBeamDimensions() const override { return m_beam_dimensions; }

    virtual std::shared_ptr<ChVehicleBushingData> GetBushingData() const override { return m_bushingData; }

  private:
    virtual const ChVector<> GetLocation(PointId which) override { return m_points[which]; }
    virtual const ChVector<> GetDirection() override { return m_dir; }

    virtual void Create(const rapidjson::Document& d) override;

    ChVector<> m_points[NUM_POINTS];
    ChVector<> m_dir;
    double m_beam_max_pitch;
    double m_beam_mass;
    ChVector<> m_beam_inertia;
    ChVector<> m_beam_dimensions;

    std::shared_ptr<ChVehicleBushingData> m_bushingData;
};

/// @} vehicle_wheeled_subchassis

}  // end namespace vehicle
}  // end namespace chrono

#endif
