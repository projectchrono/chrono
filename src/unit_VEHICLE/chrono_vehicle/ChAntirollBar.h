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
// Base class for an anti-roll bar subsystem
//
// =============================================================================

#ifndef CH_ANTIROLLBAR_H
#define CH_ANTIROLLBAR_H

#include <string>
#include <vector>

#include "core/ChShared.h"
#include "physics/ChSystem.h"
#include "physics/ChBodyAuxRef.h"

#include "chrono_vehicle/ChApiVehicle.h"
#include "chrono_vehicle/ChSubsysDefs.h"

namespace chrono {

///
/// Base class for an anti-roll bar subsystem.
///
class CH_VEHICLE_API ChAntirollBar : public ChShared
{
public:

  ChAntirollBar(
    const std::string& name          ///< [in] name of the subsystem
    );

  virtual ~ChAntirollBar() {}

  /// Get the name identifier for this suspension subsystem.
  const std::string& GetName() const { return m_name; }

  /// Set the name identifier for this suspension subsystem.
  void SetName(const std::string& name) { m_name = name; }

  /// Initialize this anti-roll bar subsystem.
  /// The anti-roll bar subsystem is initialized by attaching it to the specified
  /// chassis body at the specified location (with respect to and expressed in
  /// the reference frame of the chassis). It is assumed that the suspension
  /// reference frame is always aligned with the chassis reference frame.
  /// Finally, susp_body_left and susp_body_right are handles to the suspension
  /// bodies to which the anti-roll bar's droplinks are to be attached.
  virtual void Initialize(
    ChSharedPtr<ChBodyAuxRef>  chassis,         ///< [in] handle to the chassis body
    const ChVector<>&          location,        ///< [in] location relative to the chassis frame
    ChSharedPtr<ChBody>        susp_body_left,  ///< [in] susp body to which left droplink is connected
    ChSharedPtr<ChBody>        susp_body_right  ///< [in] susp body to which right droplink is connected
    ) = 0;

  /// Log current constraint violations.
  virtual void LogConstraintViolations() {}

protected:

  std::string  m_name;    ///< name of the subsystem
};

/// Vector of handles to antirollbar subsystems.
typedef std::vector<ChSharedPtr<ChAntirollBar> >  ChAntirollbarList;


} // end namespace chrono


#endif
