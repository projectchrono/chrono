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
// Base class for a steering subsystem.
//
// =============================================================================

#ifndef CH_STEERING_H
#define CH_STEERING_H

#include "chrono_vehicle/ChApiVehicle.h"
#include "chrono_vehicle/ChChassis.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle_wheeled_steering
/// @{

/// Base class for a steering subsystem.
class CH_VEHICLE_API ChSteering : public ChPart {
  public:
    virtual ~ChSteering();

    /// Get the frame of the steering subsystem relative to the associated chassis reference frame.
    const ChFrame<>& GetRelTransform() const { return m_rel_xform; }

    /// Get a handle to the main link of the steering subsystems.
    /// Return a handle to the body to which the tierods of a steerable
    /// suspension subsystem are attached.
    std::shared_ptr<ChBody> GetSteeringLink() const { return m_link; }

    /// Initialize this steering subsystem.
    /// The steering subsystem is initialized by attaching it to the specified chassis at the specified location (with
    /// respect to and expressed in the reference frame of the chassis) and with specified orientation (with respect to
    /// the chassis reference frame).
    virtual void Initialize(std::shared_ptr<ChChassis> chassis,  ///< [in] associated chassis subsystem
                            const ChVector<>& location,          ///< [in] location relative to the chassis frame
                            const ChQuaternion<>& rotation       ///< [in] orientation relative to the chassis frame
                            ) = 0;

    /// Update the state of this steering subsystem at the current time.
    /// The steering subsystem is provided the current steering driver input (a value between -1 and +1).  Positive
    /// steering input indicates steering to the left. This function is called during the vehicle update.
    virtual void Synchronize(double time,                           ///< [in] current time
                             const DriverInputs& driver_inputs  ///< [in] current driver inputs
                             ) = 0;

    /// Log current constraint violations.
    virtual void LogConstraintViolations() {}

  protected:
    /// Construct a steering subsystem with given name.
    ChSteering(const std::string& name);

    ChFrame<> m_rel_xform;           ///< location and orientation relative to associated chassis
    std::shared_ptr<ChBody> m_link;  ///< handle to the main steering link
};

/// Vector of handles to steering subsystems
typedef std::vector<std::shared_ptr<ChSteering> > ChSteeringList;

/// @} vehicle_wheeled_steering

}  // end namespace vehicle
}  // end namespace chrono

#endif
