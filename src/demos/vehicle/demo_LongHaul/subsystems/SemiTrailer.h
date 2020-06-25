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
// Authors: Rainer Gericke, Radu Serban
// =============================================================================
//
// Trailer for the tractor-trailer vehicle model.
//
// =============================================================================

#ifndef SEMITRAILER_H
#define SEMITRAILER_H

#include "subsystems/SemiTrailer_chassis.h"

#include "chrono_vehicle/wheeled_vehicle/ChWheeledVehicle.h"
#include "chrono_vehicle/ChTerrain.h"

class SemiTrailer {
  public:
    SemiTrailer(chrono::ChSystem* mysystem, const bool fixed);

    ~SemiTrailer() {}

    int GetNumberAxles() const { return 2; }

    double GetSpringForce(int axle, chrono::vehicle::VehicleSide side) const;
    double GetSpringLength(int axle, chrono::vehicle::VehicleSide side) const;
    double GetSpringDeformation(int axle, chrono::vehicle::VehicleSide side) const;

    double GetShockForce(int axle, chrono::vehicle::VehicleSide side) const;
    double GetShockLength(int axle, chrono::vehicle::VehicleSide side) const;
    double GetShockVelocity(int axle, chrono::vehicle::VehicleSide side) const;

    void Initialize(std::shared_ptr<chrono::vehicle::ChChassis> frontChassis, const chrono::ChVector<>& location);

    void InitializeTire(
        std::shared_ptr<chrono::vehicle::ChTire> tire,
        std::shared_ptr<chrono::vehicle::ChWheel> wheel,
        chrono::vehicle::VisualizationType tire_vis = chrono::vehicle::VisualizationType::PRIMITIVES,
        chrono::vehicle::ChTire::CollisionType tire_coll = chrono::vehicle::ChTire::CollisionType::SINGLE_POINT);

    void Synchronize(double time, double braking, const chrono::vehicle::ChTerrain& terrain);

    void Advance(double step);

    void SetChassisVisualizationType(chrono::vehicle::VisualizationType vis);
    void SetSuspensionVisualizationType(chrono::vehicle::VisualizationType vis);
    void SetWheelVisualizationType(chrono::vehicle::VisualizationType vis);

    /// Get all trailer axle subsystems.
    const chrono::vehicle::ChAxleList& GetAxles() const { return m_axles; }

    /// Get the specified trailer axle subsystem.
    std::shared_ptr<chrono::vehicle::ChAxle> GetAxle(int id) const { return m_axles[id]; }

  private:
    std::shared_ptr<chrono::vehicle::ChChassisRear> m_chassis;              ///< trailer chassis
    std::shared_ptr<chrono::vehicle::ChChassisConnectorHitch> m_connector;  ///< connector to pulling vehicle
    chrono::vehicle::ChAxleList m_axles;                                    ///< list of axle subsystems
};

#endif
