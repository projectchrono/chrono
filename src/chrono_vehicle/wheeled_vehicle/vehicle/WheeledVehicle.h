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
// Wheeled vehicle model constructed from a JSON specification file
//
// =============================================================================

#ifndef WHEELED_VEHICLE_H
#define WHEELED_VEHICLE_H

#include "chrono/core/ChCoordsys.h"
#include "chrono/physics/ChSystem.h"
#include "chrono/physics/ChMaterialSurface.h"

#include "chrono_vehicle/wheeled_vehicle/ChWheeledVehicle.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle_wheeled
/// @{

/// Wheeled vehicle model constructed from a JSON specification file.
class CH_VEHICLE_API WheeledVehicle : public ChWheeledVehicle {
  public:
    WheeledVehicle(const std::string& filename,
                   ChMaterialSurface::ContactMethod contact_method = ChMaterialSurface::NSC);

    WheeledVehicle(ChSystem* system, const std::string& filename);

    ~WheeledVehicle() {}

    virtual int GetNumberAxles() const override { return m_num_axles; }

    virtual double GetWheelbase() const override { return m_wheelbase; }
    virtual double GetMinTurningRadius() const override { return m_turn_radius; }
    virtual double GetMaxSteeringAngle() const override { return m_steer_angle; }

    virtual void Initialize(const ChCoordsys<>& chassisPos, double chassisFwdVel = 0) override;

  private:
    void Create(const std::string& filename);

  private:
    int m_num_axles;                           // number of axles for this vehicle
    std::vector<ChVector<> > m_suspLocations;  // locations of the suspensions relative to chassis
    std::vector<int> m_suspSteering;           // indexes of steering subsystems (-1 indicates a non-steered suspension)
    double m_wheelbase;                        // vehicle wheel base

    std::vector<ChVector<> > m_arbLocations;  // locations of the antirollbar subsystems relative to chassis
    std::vector<int> m_arbSuspension;         // indexes of steering subsystems

    int m_num_strs;                               // number of steering subsystems
    std::vector<ChVector<> > m_strLocations;      // locations of the steering subsystems relative to chassis
    std::vector<ChQuaternion<> > m_strRotations;  // orientations of the steering subsystems relative to chassis

    std::vector<int> m_driven_susp;  // indexes of the driven suspensions

    double m_turn_radius;  // minimum turning radius
    double m_steer_angle;  // maximum steering angle
};

/// @} vehicle_wheeled

}  // end namespace vehicle
}  // end namespace chrono

#endif
