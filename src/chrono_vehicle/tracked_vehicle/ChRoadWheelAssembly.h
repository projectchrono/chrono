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
// Base class for a road wheel assembly (suspension).  A road wheel assembly
// contains a road wheel body (connected through a revolute joint to the chassis)
// with different suspension topologies.
//
// The reference frame for a vehicle follows the ISO standard: Z-axis up, X-axis
// pointing forward, and Y-axis towards the left of the vehicle.
//
// =============================================================================

#ifndef CH_ROAD_WHEEL_ASSEMBLY_H
#define CH_ROAD_WHEEL_ASSEMBLY_H

#include "chrono/physics/ChBody.h"
#include "chrono/physics/ChBodyAuxRef.h"
#include "chrono/physics/ChLinkLock.h"

#include "chrono_vehicle/ChApiVehicle.h"
#include "chrono_vehicle/ChChassis.h"

#include "chrono_vehicle/tracked_vehicle/ChRoadWheel.h"

namespace chrono {
namespace vehicle {

class ChTrackAssembly;

/// @addtogroup vehicle_tracked_suspension
/// @{

/// Base class for tracked vehicle suspension (road-wheel assembly) subsystem.
class CH_VEHICLE_API ChRoadWheelAssembly : public ChPart {
  public:
    /// Output structure for spring-damper forces or torques.
    struct ForceTorque {
        double spring_ft;        ///< force (torque) in translational (rotational) spring
        double shock_ft;         ///< force (torque) in translational (rotational) damper
        double spring_displ;     ///< translational (rotational) spring displacement
        double spring_velocity;  ///< translational (rotational) spring displacement velocity
        double shock_displ;      ///< translational (rotational) damper displacement
        double shock_velocity;   ///< translational (rotational) damper displacement velocity
    };

    virtual ~ChRoadWheelAssembly() {}

    /// Return the type of track shoe consistent with this road wheel.
    GuidePinType GetType() const { return m_type; }

    /// Return a handle to the road wheel subsystem.
    std::shared_ptr<ChRoadWheel> GetRoadWheel() const { return m_road_wheel; }

    /// Return a handle to the carrier body.
    virtual std::shared_ptr<ChBody> GetCarrierBody() const = 0;

    /// Return the current pitch angle of the carrier body.
    /// This angle is measured in the x-z transversal plane, from the initial configuration,
    /// and follows the right-hand rule.
    virtual double GetCarrierAngle() const = 0;

    /// Get a handle to the road wheel body.
    std::shared_ptr<ChBody> GetWheelBody() const { return m_road_wheel->GetWheelBody(); }

    /// Get a handle to the revolute joint of the road-wheel.
    std::shared_ptr<ChLinkLockRevolute> GetWheelRevolute() const { return m_road_wheel->GetRevolute(); }

    /// Get the radius of the road wheel.
    double GetWheelRadius() const { return m_road_wheel->GetWheelRadius(); }

    /// Initialize this suspension subsystem.
    /// The suspension subsystem is initialized by attaching it to the specified
    /// chassis body at the specified location (with respect to and expressed in
    /// the reference frame of the chassis). It is assumed that the suspension
    /// reference frame is always centered at the location of the road wheel and
    /// aligned with the chassis reference frame.
    /// Derived classes must call this base class implementation (which only
    /// initializes the road wheel).
    virtual void Initialize(std::shared_ptr<ChChassis> chassis,  ///< [in] associated chassis subsystem
                            const ChVector<>& location,          ///< [in] location relative to the chassis frame
                            ChTrackAssembly* track               ///< [in] containing track assembly
    );

    /// Return current suspension forces or torques, as appropriate (spring and shock).
    /// Different derived types (suspension templates) may load different quantities in the output struct.
    virtual ForceTorque ReportSuspensionForce() const = 0;

    /// Enable/disable output for this subsystem.
    /// This function overrides the output setting for all components of this suspension assembly.
    virtual void SetOutput(bool state) override;

    /// Log current constraint violations.
    virtual void LogConstraintViolations() = 0;

  protected:
    ChRoadWheelAssembly(const std::string& name,  ///< [in] name of the subsystem
                        bool has_shock            ///< [in] specify whether or not the suspension has a damper
    );

    virtual void ExportComponentList(rapidjson::Document& jsonDocument) const override;

    GuidePinType m_type;  ///< type of the track shoe matching this road wheel
    bool m_has_shock;     ///< specifies whether or not the suspension has a damper

    ChVector<> m_rel_loc;                       ///< idler subsystem location relative to chassis
    std::shared_ptr<ChRoadWheel> m_road_wheel;  ///< road-wheel subsystem
    ChTrackAssembly* m_track;                   ///< containing track assembly

    friend class ChTrackAssembly;
};

/// Vector of handles to road wheel assembly subsystems.
typedef std::vector<std::shared_ptr<ChRoadWheelAssembly> > ChRoadWheelAssemblyList;

/// @} vehicle_tracked_suspension

}  // end namespace vehicle
}  // end namespace chrono

#endif
