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
// Base class for a single-pin track shoe (template definition).
// A single-pin track shoe can be either of CENTRAL_PIN or LATERAL_PIN type.
//
// =============================================================================

#ifndef CH_SINGLE_PIN_SHOE_H
#define CH_SINGLE_PIN_SHOE_H

#include "chrono_vehicle/ChApiVehicle.h"
#include "chrono_vehicle/ChSubsysDefs.h"

#include "chrono_vehicle/tracked_vehicle/ChTrackShoe.h"

namespace chrono {
namespace vehicle {

///
///
///
class CH_VEHICLE_API ChSinglePinShoe : public ChTrackShoe {
  public:
    ChSinglePinShoe(const std::string& name  ///< [in] name of the subsystem
                    );

    /// Initialize this track shoe subsystem.
    /// The track shoe is created within the specified system and initialized
    /// at the specified location and orientation (expressed in the global frame).
    /// A derived class must extend this default implementation and specify the contact
    /// geometry for the track shoe body.
    virtual void Initialize(ChSystem* system,               ///< [in] containing system
                            const ChVector<>& location,     ///< [in] location relative to the chassis frame
                            const ChQuaternion<>& rotation  ///< [in] orientation relative to the chassis frame
                            ) override;

    /// Connect this track shoe to the specified neighbor.
    /// This function must be called only after both track shoes have been initialized.
    virtual void Connect(ChSharedPtr<ChTrackShoe> next  ///< [in] handle to the neighbor track shoe
                         ) override;

  protected:
    /// Return the mass of the shoe body.
    virtual double GetShoeMass() const = 0;

    /// Return the moments of inertia of the shoe body.
    virtual ChVector<> GetShoeInertia() const = 0;

    /// Return the radius of the track shoe pin.
    //// TODO: is this really needed?
    virtual double GetPinRadius() const = 0;

    /// Return the length of the track shoe pin.
    /// This is the total pin length.
    //// TODO: is this really needed?
    virtual double GetPinLength() const = 0;

    ChSharedPtr<ChLinkLockRevolute> m_revolute;  ///< handle to revolute joint connection to next shoe
};

}  // end namespace vehicle
}  // end namespace chrono

#endif
