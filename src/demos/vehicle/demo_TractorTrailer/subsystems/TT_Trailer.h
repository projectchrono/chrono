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
// Authors: Alessandro Tasora
// =============================================================================
//
// Trailer for the tractor-trailer vehicle model.
//
// =============================================================================

#ifndef TT_TRAILER_H
#define TT_TRAILER_H

#include "chrono_vehicle/wheeled_vehicle/ChWheeledVehicle.h"

class TT_Trailer {
  public:
    TT_Trailer(chrono::ChSystem* mysystem, const bool fixed, chrono::vehicle::SuspensionType suspType);

    ~TT_Trailer() {}

    virtual int GetNumberAxles() const { return 2; }

    double GetSpringForce(const chrono::vehicle::WheelID& wheel_id) const;
    double GetSpringLength(const chrono::vehicle::WheelID& wheel_id) const;
    double GetSpringDeformation(const chrono::vehicle::WheelID& wheel_id) const;

    double GetShockForce(const chrono::vehicle::WheelID& wheel_id) const;
    double GetShockLength(const chrono::vehicle::WheelID& wheel_id) const;
    double GetShockVelocity(const chrono::vehicle::WheelID& wheel_id) const;

    virtual void Initialize(const chrono::ChCoordsys<>& chassisPos,
                            const bool connect_to_puller,
                            std::shared_ptr<chrono::ChBodyAuxRef> pulling_vehicle);

    virtual void Synchronize(double time, double braking, const chrono::vehicle::TireForces& tire_forces);

    void SetSuspensionVisualizationType(chrono::vehicle::VisualizationType vis);
    void SetWheelVisualizationType(chrono::vehicle::VisualizationType vis);

    // Log debugging information
    void LogHardpointLocations();  /// suspension hardpoints at design
    void DebugLog(int what);       /// shock forces and lengths, constraints, etc.

    /// Get a handle to the specified wheel body.
    std::shared_ptr<chrono::ChBody> GetWheelBody(const chrono::vehicle::WheelID& wheelID) const;

  private:
    chrono::vehicle::SuspensionType m_suspType;

    std::shared_ptr<chrono::ChBodyAuxRef> m_chassis;    ///< handle to the chassis body
    std::shared_ptr<chrono::ChBodyAuxRef> m_frontaxle;  ///< handle to the steering axle
    chrono::vehicle::ChSuspensionList m_suspensions;    ///< list of handles to suspension subsystems
    chrono::vehicle::ChWheelList m_wheels;              ///< list of handles to wheel subsystems
    chrono::vehicle::ChBrakeList m_brakes;              ///< list of handles to brake subsystems

    std::shared_ptr<chrono::ChLinkLockSpherical> m_joint;   ///< handle to the joint between chassis and front axle
    std::shared_ptr<chrono::ChLinkLockSpherical> m_puller;  ///< handle to the joint between trailer and pulling vehicle

    // Chassis mass properties
    static const double m_chassisMass;
    static const chrono::ChVector<> m_chassisCOM;
    static const chrono::ChVector<> m_chassisInertia;
    static const double m_frontaxleMass;
    static const chrono::ChVector<> m_frontaxleCOM;
    static const chrono::ChVector<> m_frontaxleREF;
    static const chrono::ChVector<> m_frontaxleInertia;

    static const chrono::ChVector<> m_frontaxleSphericalJoint;
    static const chrono::ChVector<> m_frontaxlePullerJoint;
};

#endif
