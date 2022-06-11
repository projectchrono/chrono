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

#include "chrono_vehicle/wheeled_vehicle/ChWheeledVehicle.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle_wheeled
/// @{

/// Wheeled vehicle model constructed from a JSON specification file.
class CH_VEHICLE_API WheeledVehicle : public ChWheeledVehicle {
  public:
    /// Create a wheeled vehicle from the provided JSON specification file.
    /// The vehicle is added to a newly created Chrono system which uses the specified contact formulation. If
    /// indicated, an associated powertrain and tires are created (if specified in the JSON file).
    WheeledVehicle(const std::string& filename,
                   ChContactMethod contact_method = ChContactMethod::NSC,
                   bool create_powertrain = true,
                   bool create_tires = true);

    /// Create a wheeled vehicle from the provided JSON specification file.
    /// The vehicle is added to the given Chrono system. If indicated, an associated powertrain and tires are created
    /// (if specified in the JSON file).
    WheeledVehicle(ChSystem* system,
                   const std::string& filename,
                   bool create_powertrain = true,
                   bool create_tires = true);

    ~WheeledVehicle() {}

    virtual int GetNumberAxles() const override { return m_num_axles; }

    virtual double GetWheelbase() const override { return m_wheelbase; }
    virtual double GetMinTurningRadius() const override { return m_turn_radius; }
    virtual double GetMaxSteeringAngle() const override { return m_steer_angle; }

    virtual void Initialize(const ChCoordsys<>& chassisPos, double chassisFwdVel = 0) override;

  private:
    void Create(const std::string& filename, bool create_powertrain, bool create_tires);

  private:
    int m_num_rear_chassis;                   // number of rear chassis subsystems for this vehicle
    std::vector<int> m_rearch_chassis_index;  // indexes of connected chassis (-1: main chassis, >=0: rear chassis)

    double m_wheelbase;  // vehicle wheel base

    int m_num_axles;                           // number of axles for this vehicle
    std::vector<ChVector<>> m_susp_locations;  // locations of the suspensions relative to chassis
    std::vector<ChVector<>> m_arb_locations;   // locations of the antirollbar subsystems relative to chassis
    std::vector<int> m_susp_steering_index;    // indexes of associated steering (-1: none, non-steered suspension)
    std::vector<int> m_susp_chassis_index;     // indexes of associated chassis (-1: main chassis, >=0: rear chassis)
    std::vector<int> m_susp_subchassis_index;  // indexes of associated subchassis (-1: none, connect to chassis only)

    int m_num_strs;                               // number of steering subsystems
    std::vector<ChVector<>> m_str_locations;      // locations of the steering subsystems relative to chassis
    std::vector<ChQuaternion<>> m_str_rotations;  // orientations of the steering subsystems relative to chassis
    std::vector<int> m_str_chassis_index;         // indexes of associated chassis (-1: main chassis, >=0: rear chassis)

    int m_num_subch;                            // number of subchassis subsystems
    std::vector<ChVector<>> m_subch_locations;  // locations of the subchassis subsystems relative to chassis
    std::vector<int> m_subch_chassis_index;     // indexes of associated chassis (-1: main chassis, >=0: rear chassis)

    std::vector<int> m_driven_axles;  // indexes of the driven axles

    std::vector<double> m_wheel_separations;  // wheel separations for each axle

    double m_turn_radius;  // minimum turning radius
    double m_steer_angle;  // maximum steering angle
};

/// @} vehicle_wheeled

}  // end namespace vehicle
}  // end namespace chrono

#endif
