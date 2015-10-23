// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All right reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban
// =============================================================================
//
// Base class for a tracked vehicle sprocket. A sprocket is responsible for
// contact processing with the track shoes of the containing track assembly.
//
// The reference frame for a vehicle follows the ISO standard: Z-axis up, X-axis
// pointing forward, and Y-axis towards the left of the vehicle.
//
// =============================================================================

#ifndef CH_SPROCKET_H
#define CH_SPROCKET_H

#include <vector>

#include "chrono/core/ChShared.h"
#include "chrono/physics/ChSystem.h"
#include "chrono/physics/ChBody.h"
#include "chrono/physics/ChBodyAuxRef.h"
#include "chrono/physics/ChLinkLock.h"
#include "chrono/physics/ChShaft.h"
#include "chrono/physics/ChShaftsBody.h"
#include "chrono/geometry/ChCLinePath.h"
#include "chrono/assets/ChColor.h"

#include "chrono_vehicle/ChApiVehicle.h"
#include "chrono_vehicle/ChSubsysDefs.h"

namespace chrono {
namespace vehicle {

///
///
///
class CH_VEHICLE_API ChSprocket : public ChShared {
  public:
    ChSprocket(const std::string& name  ///< [in] name of the subsystem
               )
        : m_name(name) {}

    virtual ~ChSprocket() {}

    /// Get the name identifier for this sprocket subsystem.
    const std::string& GetName() const { return m_name; }

    /// Set the name identifier for this sprocket subsystem.
    void SetName(const std::string& name) { m_name = name; }

    /// Get the number of teeth of the gear.
    virtual int GetNumTeeth() const = 0;

    /// Get a handle to the road wheel body.
    ChSharedPtr<ChBody> GetWheel() const { return m_gear; }

    /// Get a handle to the axle shaft.
    ChSharedPtr<ChShaft> GetAxle() const { return m_axle; }

    /// Get a handle to the revolute joint.
    ChSharedPtr<ChLinkLockRevolute> GetRevolute() const { return m_revolute; }

    /// Get the angular speed of the axle.
    double GetAxleSpeed() const { return m_axle->GetPos_dt(); }

    /// Initialize this sprocket subsystem.
    /// The sprocket subsystem is initialized by attaching it to the specified
    /// chassis body at the specified location (with respect to and expressed in
    /// the reference frame of the chassis).
    virtual void Initialize(ChSharedPtr<ChBodyAuxRef> chassis,  ///< [in] handle to the chassis body
                            const ChVector<>& location          ///< [in] location relative to the chassis frame
                            );

    /// Apply the provided motor torque.
    /// The given torque is applied to the axle. This function provides the interface
    /// to the drivetrain subsystem (intermediated by the vehicle system).
    void ApplyAxleTorque(double torque  ///< [in] value of applied torque
                         );

    /// Add visualization of the tooth profiles.
    /// This (optional) function should be called only after a call to Initialize().
    /// It renders the gear profile as a colored path.
    void AddGearVisualization(const ChColor& color);

  protected:
    /// Return the mass of the gear body.
    virtual double getGearMass() const = 0;

    /// Return the moments of inertia of the gear body.
    virtual const ChVector<>& getGearInertia() = 0;

    /// Return the inertia of the axle shaft.
    virtual double getAxleInertia() const = 0;

    /// Return the distance between the two gear profiles.
    virtual double getSeparation() const = 0;

    /// Return the 2D gear profile.
    /// The gear profile, a ChLinePath geometric object, is made up of an arbitrary number
    /// of sub-paths of type ChLineArc or ChLineSegment sub-lines. These must be added in
    /// clockwise order, and the end of sub-path i must be coincident with beginning of
    /// sub-path i+1.
    virtual ChSharedPtr<geometry::ChLinePath> getProfile() = 0;

    std::string m_name;                           ///< name of the subsystem
    ChSharedPtr<ChBody> m_gear;                   ///< handle to the sprocket gear body
    ChSharedPtr<ChShaft> m_axle;                  ///< handle to gear shafts
    ChSharedPtr<ChShaftsBody> m_axle_to_spindle;  ///< handle to gear-shaft connector
    ChSharedPtr<ChLinkLockRevolute> m_revolute;   ///< handle to sprocket revolute joint
};

/// Vector of handles to sprocket subsystems.
typedef std::vector<ChSharedPtr<ChSprocket> > ChSprocketList;

}  // end namespace vehicle
}  // end namespace chrono

#endif
