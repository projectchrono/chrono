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
// Rear of the articulated vehicle model. This base class has a structure similar
// to that of a ChWheeledVehicle.
//
// =============================================================================

#ifndef ARTICULATED_REAR_H
#define ARTICULATED_REAR_H

#include "chrono/physics/ChLinkMotorRotationAngle.h"
#include "chrono_vehicle/wheeled_vehicle/ChWheeledVehicle.h"
#include "subsystems/Articulated_Front.h"

class Articulated_Rear {
  public:
    Articulated_Rear(std::shared_ptr<Articulated_Chassis> front);

    int GetNumberAxles() const { return 1; }

    void Initialize();

    void Synchronize(double time, double steering, double braking, const chrono::vehicle::TerrainForces& tire_forces);

    void SetSuspensionVisualizationType(chrono::vehicle::VisualizationType vis);
    void SetWheelVisualizationType(chrono::vehicle::VisualizationType vis);

    /// Get a handle to the specified wheel body.
    std::shared_ptr<chrono::ChBody> GetWheelBody(const chrono::vehicle::WheelID& wheel_id) const;

    /// Get the global location of the specified wheel.
    const chrono::ChVector<>& GetWheelPos(const chrono::vehicle::WheelID& wheel_id) const;

    /// Get the orientation of the specified wheel.
    const chrono::ChQuaternion<>& GetWheelRot(const chrono::vehicle::WheelID& wheel_id) const;

    /// Get the linear velocity of the specified wheel.
    const chrono::ChVector<>& GetWheelLinVel(const chrono::vehicle::WheelID& wheel_id) const;

    /// Get the angular velocity of the specified wheel.
    chrono::ChVector<> GetWheelAngVel(const chrono::vehicle::WheelID& wheel_id) const;

    /// Get the complete state for the specified wheel.
    chrono::vehicle::WheelState GetWheelState(const chrono::vehicle::WheelID& wheel_id) const;

  private:
    std::shared_ptr<Articulated_Chassis> m_front;  ///< handle to front side

    std::shared_ptr<chrono::ChBodyAuxRef> m_chassis;  ///< handle to the chassis body

    chrono::vehicle::ChSuspensionList m_suspensions;  ///< list of handles to suspension subsystems
    chrono::vehicle::ChWheelList m_wheels;            ///< list of handles to wheel subsystems
    chrono::vehicle::ChBrakeList m_brakes;            ///< list of handles to brake subsystems

    std::shared_ptr<chrono::ChLinkMotorRotationAngle> m_motor;

    // Chassis mass properties
    static const double m_chassisMass;
    static const chrono::ChVector<> m_chassisCOM;
    static const chrono::ChVector<> m_chassisInertia;

    static const chrono::ChVector<> m_offset;
};

#endif
