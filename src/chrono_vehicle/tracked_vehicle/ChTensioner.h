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
// Base class for a track system tensioner. A tensioner is connected either to
// an idler (if present) or to one of the sprockets of the containing track
// assembly.
//
// The reference frame for a vehicle follows the ISO standard: Z-axis up, X-axis
// pointing forward, and Y-axis towards the left of the vehicle.
//
// =============================================================================

#ifndef CH_TENSIONER_H
#define CH_TENSIONER_H

#include "chrono/core/ChShared.h"
#include "chrono/physics/ChBodyAuxRef.h"

#include "chrono_vehicle/ChApiVehicle.h"
#include "chrono_vehicle/ChSubsysDefs.h"

namespace chrono {
namespace vehicle {

///
///
///
class CH_VEHICLE_API ChTensioner : public ChShared {
  public:
    ChTensioner(const std::string& name  ///< [in] name of the subsystem
                )
        : m_name(name) {}

    virtual ~ChTensioner() {}

    /// Get the name identifier for this tensioner subsystem.
    const std::string& GetName() const { return m_name; }

    /// Set the name identifier for this tensioner subsystem.
    void SetName(const std::string& name) { m_name = name; }

    /// Initialize this tensioner subsystem.
    /// The tensioner subsystem is initialized by attaching it to the specified
    /// chassis body at the specified location (with respect to and expressed in
    /// the reference frame of the chassis) and with specified pitch angle (with
    /// respect to the chassis reference frame).
    virtual void Initialize(ChSharedPtr<ChBodyAuxRef> chassis,  ///< [in] handle to the chassis body
                            const ChVector<>& location,         ///< [in] location relative to the chassis frame
                            double pitch                        ///< [in] pitch angle relative to the chassis frame
                            ) = 0;

  protected:
    std::string m_name;  ///< name of the subsystem
};

}  // end namespace vehicle
}  // end namespace chrono

#endif
