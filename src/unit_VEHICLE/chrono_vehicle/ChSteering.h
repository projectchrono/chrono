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
// Base class for a steering subsystem.
//
// =============================================================================

#ifndef CH_STEERING_H
#define CH_STEERING_H

#include <string>

#include "core/ChShared.h"
#include "physics/ChSystem.h"
#include "physics/ChBodyAuxRef.h"

#include "chrono_vehicle/ChApiVehicle.h"

namespace chrono {

///
/// Base class for a steering subsystem.
///
class CH_VEHICLE_API ChSteering : public ChShared
{
public:

  ChSteering(
    const std::string& name    ///< [in] name of the subsystem
    );

  virtual ~ChSteering() {}

  /// Get the name identifier for this steering subsystem.
  const std::string& GetName() const { return m_name; }

  /// Set the name identifier for this steering subsystem.
  void SetName(const std::string& name) { m_name = name; }

  /// Get a handle to the main link of the steering subsystems.
  /// Return a handle to the body to which the tierods of a steerbale
  /// suspension subsystem are attached.
  ChSharedPtr<ChBody>  GetSteeringLink() const { return m_link; }

  /// Initialize this steering subsystem.
  /// The steering subsystem is initialized by attaching it to the specified 
  /// chassis body at the specified location (with respect to and expressed in
  /// the reference frame of the chassis) and with specified orientation (with
  /// respect to the chassis reference frame).
  virtual void Initialize(
    ChSharedPtr<ChBodyAuxRef> chassis,   ///< [in] handle to the chassis body
    const ChVector<>&         location,  ///< [in] location relative to the chassis frame
    const ChQuaternion<>&     rotation   ///< [in] orientation relative to the chassis frame
    ) = 0;

  /// Update the state of this steering subsystem at the current time.
  /// The steering subsystem is provided the current steering driver input (a
  /// value between -1 and +1).  Positive steering input indicates steering
  /// to the left. This function is called during the vehicle update.
  virtual void Update(
    double time,       ///< [in] current time
    double steering    ///< [in] current steering input [-1,+1]
    ) = 0;

  /// Log current constraint violations.
  virtual void LogConstraintViolations() {}

protected:

  std::string  m_name;          ///< name of the subsystem

  ChSharedPtr<ChBody>  m_link;  ///< handle to the main steering link
};


/// Vector of handles to steering subsystems
typedef std::vector<ChSharedPtr<ChSteering> > ChSteeringList;


} // end namespace chrono


#endif
