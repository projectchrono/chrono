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
// Base class for a track assembly using single-pin track shoes
// (template definitions).
//
// The reference frame for a vehicle follows the ISO standard: Z-axis up, X-axis
// pointing forward, and Y-axis towards the left of the vehicle.
//
// =============================================================================

#ifndef CH_TRACK_ASSEMBLY_SINGLE_PIN_H
#define CH_TRACK_ASSEMBLY_SINGLE_PIN_H

#include <vector>

#include "chrono_vehicle/ChApiVehicle.h"
#include "chrono_vehicle/tracked_vehicle/ChTrackAssembly.h"
#include "chrono_vehicle/tracked_vehicle/sprocket/ChSprocketSinglePin.h"
#include "chrono_vehicle/tracked_vehicle/track_shoe/ChTrackShoeSinglePin.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle_tracked
/// @{

/// Definition of a single-pin track assembly.
/// A track assembly consists of a sprocket, an idler (with tensioner mechanism),
/// a set of suspensions (road-wheel assemblies), and a collection of track shoes.
/// This class defines the template for a track assembly using single-pin track shoes.
class CH_VEHICLE_API ChTrackAssemblySinglePin : public ChTrackAssembly {
  public:
    ChTrackAssemblySinglePin(const std::string& name,  ///< [in] name of the subsystem
                             VehicleSide side          ///< [in] assembly on left/right vehicle side
                             )
        : ChTrackAssembly(name, side) {}

    virtual ~ChTrackAssemblySinglePin() {}

    /// Get the name of the vehicle subsystem template.
    virtual std::string GetTemplateName() const override { return "TrackAssemblySinglePin"; }

    /// Get the number of track shoes.
    virtual size_t GetNumTrackShoes() const override { return m_shoes.size(); }

    /// Get a handle to the sprocket.
    virtual std::shared_ptr<ChSprocket> GetSprocket() const override { return m_sprocket; }

    /// Get a handle to the specified track shoe subsystem.
    virtual std::shared_ptr<ChTrackShoe> GetTrackShoe(size_t id) const override { return m_shoes[id]; }

  protected:
    std::shared_ptr<ChSprocketSinglePin> m_sprocket;  ///< sprocket subsystem
    ChTrackShoeSinglePinList m_shoes;                 ///< track shoes

  private:
    /// Assemble track shoes over wheels.
    /// Return true if the track shoes were initialized in a counter clockwise
    /// direction and false otherwise.
    virtual bool Assemble(std::shared_ptr<ChBodyAuxRef> chassis) override final;

    /// Remove all track shoes from assembly.
    virtual void RemoveTrackShoes() override final;
};

/// @} vehicle_tracked

}  // end namespace vehicle
}  // end namespace chrono

#endif
