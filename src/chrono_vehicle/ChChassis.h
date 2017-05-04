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
// Base class for the chassis vehicle subsystem.
//
// =============================================================================

#ifndef CH_CHASSIS_H
#define CH_CHASSIS_H

#include "chrono/physics/ChSystem.h"
#include "chrono/physics/ChBodyAuxRef.h"

#include "chrono_vehicle/ChApiVehicle.h"
#include "chrono_vehicle/ChPart.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle
/// @{

/// Base class for the chassis vehicle subsystem.
class CH_VEHICLE_API ChChassis : public ChPart {
  public:
    /// Construct a vehicle subsystem with the specified name.
    ChChassis(const std::string& name,  ///< [in] name of the subsystem
              bool fixed = false        ///< [in] is the chassis body fixed to ground?
              );

    virtual ~ChChassis() {}

    /// Get the chassis mass.
    virtual double GetMass() const = 0;

    /// Get the inertia tensor of the chassis body.
    /// The return 3x3 symmetric matrix contains the following values:
    /// <pre>
    ///  [ int{x^2+z^2}dm    -int{xy}dm    -int{xz}dm    ]
    ///  [                  int{x^2+z^2}   -int{yz}dm    ]
    ///  [                                int{x^2+y^2}dm ]
    /// </pre>
    /// and represents the inertia tensor with respect to a centroidal frame
    /// aligned with the chassis reference frame.
    virtual const ChMatrix33<>& GetInertia() const = 0;

    /// Get the location of the center of mass in the chassis frame.
    virtual const ChVector<>& GetLocalPosCOM() const = 0;

    /// Get the local driver position and orientation.
    /// This is a coordinate system relative to the chassis reference frame.
    virtual ChCoordsys<> GetLocalDriverCoordsys() const = 0;

    /// Get a handle to the vehicle's chassis body.
    std::shared_ptr<ChBodyAuxRef> GetBody() const { return m_body; }

    /// Get the global location of the chassis reference frame origin.
    const ChVector<>& GetPos() const { return m_body->GetFrame_REF_to_abs().GetPos(); }

    /// Get the orientation of the chassis reference frame.
    /// The chassis orientation is returned as a quaternion representing a
    /// rotation with respect to the global reference frame.
    const ChQuaternion<>& GetRot() const { return m_body->GetFrame_REF_to_abs().GetRot(); }

    /// Get the global location of the chassis center of mass.
    const ChVector<>& GetCOMPos() const { return m_body->GetPos(); }

    /// Get the orientation of the chassis centroidal frame.
    /// The chassis orientation is returned as a quaternion representing a
    /// rotation with respect to the global reference frame.
    const ChQuaternion<>& GetCOMRot() const { return m_body->GetRot(); }

    /// Get the global location of the driver.
    ChVector<> GetDriverPos() const;

    /// Get the vehicle speed.
    /// Return the speed measured at the origin of the chassis reference frame.
    double GetSpeed() const { return m_body->GetFrame_REF_to_abs().GetPos_dt().Length(); }

    /// Get the speed of the chassis COM.
    /// Return the speed measured at the chassis center of mass.
    double GetCOMSpeed() const { return m_body->GetPos_dt().Length(); }

    /// Get the acceleration at the specified point.
    /// The point is assumed to be given relative to the chassis reference frame.
    /// The returned acceleration is expressed in the chassis reference frame.
    ChVector<> GetPointAcceleration(const ChVector<>& locpos) const;

    /// Initialize the chassis at the specified global position and orientation.
    virtual void Initialize(ChSystem* system,                ///< [in] containing system
                            const ChCoordsys<>& chassisPos,  ///< [in] absolute chassis position
                            double chassisFwdVel,            ///< [in] initial chassis forward velocity
                            int collision_family = 0         ///< [in] chassis collision family
                            );

    /// Enable/disable contact for the chassis. This function controls contact of
    /// the chassis with all other collision shapes in the simulation.
    virtual void SetCollide(bool state) = 0;

    /// Set the "fixed to ground" status of the chassis body.
    void SetFixed(bool val) { m_body->SetBodyFixed(val); }

    /// Return true if the chassis body is fixed to ground.
    bool IsFixed() const { return m_body->GetBodyFixed(); }

  protected:
    std::shared_ptr<ChBodyAuxRef> m_body;  ///< handle to the chassis body
    bool m_fixed;                          ///< is the chassis body fixed to ground?
};

/// @} vehicle

}  // end namespace vehicle
}  // end namespace chrono

#endif
