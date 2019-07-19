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
// Base class for a track assembly using double-pin track shoes
// (template definitions).
//
// The reference frame for a vehicle follows the ISO standard: Z-axis up, X-axis
// pointing forward, and Y-axis towards the left of the vehicle.
//
// =============================================================================

#ifndef CH_TRACK_ASSEMBLY_DOUBLE_PIN_H
#define CH_TRACK_ASSEMBLY_DOUBLE_PIN_H

#include <vector>

#include "chrono/core/ChVector2.h"

#include "chrono_vehicle/ChApiVehicle.h"
#include "chrono_vehicle/tracked_vehicle/ChTrackAssembly.h"
#include "chrono_vehicle/tracked_vehicle/sprocket/ChSprocketDoublePin.h"
#include "chrono_vehicle/tracked_vehicle/track_shoe/ChTrackShoeDoublePin.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle_tracked
/// @{

/// Definition of a double-pin track assembly.
/// A track assembly consists of a sprocket, an idler (with tensioner mechanism),
/// a set of suspensions (road-wheel assemblies), and a collection of track shoes.
/// This class defines the template for a track assembly using double-pin track shoes.
class CH_VEHICLE_API ChTrackAssemblyDoublePin : public ChTrackAssembly {
  public:
    ChTrackAssemblyDoublePin(const std::string& name,  ///< [in] name of the subsystem
                             VehicleSide side          ///< [in] assembly on left/right vehicle side
                             )
        : ChTrackAssembly(name, side) {}

    virtual ~ChTrackAssemblyDoublePin() {}

    /// Get the name of the vehicle subsystem template.
    virtual std::string GetTemplateName() const override { return "TrackAssemblyDoublePin"; }

    /// Get the number of track shoes.
    virtual size_t GetNumTrackShoes() const override { return m_shoes.size(); }

    /// Get a handle to the sprocket.
    virtual std::shared_ptr<ChSprocket> GetSprocket() const override { return m_sprocket; }

    /// Get a handle to the specified track shoe subsystem.
    virtual std::shared_ptr<ChTrackShoe> GetTrackShoe(size_t id) const override { return m_shoes[id]; }

  protected:
    std::shared_ptr<ChSprocketDoublePin> m_sprocket;  ///< sprocket subsystem
    ChTrackShoeDoublePinList m_shoes;                 ///< track shoes

  private:
    /// Assemble track shoes over wheels.
    /// Return true if the track shoes were initialized in a counter clockwise
    /// direction and false otherwise.
    virtual bool Assemble(std::shared_ptr<ChBodyAuxRef> chassis) override final;

    /// Remove all track shoes from assembly.
    virtual void RemoveTrackShoes() override final;

    /// Utility function to create the bodies of the specified track shoe with
    /// the given configuration. This version specifies locations and orientations
    /// for the shoe and connector bodies separately (in 2D, in the (x-z) plane).
    void CreateTrackShoe(size_t index,    ///< index of track shoe within assembly
                         ChVector2<> ps,  ///< (x-z) location of shoe body
                         ChVector2<> pc,  ///< (x-z) location of connector body
                         double as,       ///< shoe body angle
                         double ac        ///< connector body angle
                         );

    /// Utility function to create the bodies of the specified track shoe with
    /// the given configuration. This version specifies the location of the center
    /// the track shoe system (in 2D, in the (x-z) plane) and a common orientation
    /// angle for both the shoe and connector bodies.
    void CreateTrackShoe(size_t index,   ///< index of track shoe within assembly
                         ChVector2<> p,  ///< (x-z) location of track shoe center
                         double angle    ///< angle of the shoe and connector bodies
                         );

    std::shared_ptr<ChBodyAuxRef> m_chassis;
    double m_sprocket_offset;
    double m_connector_offset;
};

/// @} vehicle_tracked

}  // end namespace vehicle
}  // end namespace chrono

#endif
