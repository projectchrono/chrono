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
// Authors: Radu Serban, Michael Taylor
// =============================================================================
//
// Base class for a continuous band track assembly using a bushing-based web
// (template definition).
//
// The reference frame for a vehicle follows the ISO standard: Z-axis up, X-axis
// pointing forward, and Y-axis towards the left of the vehicle.
//
// =============================================================================

#ifndef CH_TRACK_ASSEMBLY_BAND_BUSHING_H
#define CH_TRACK_ASSEMBLY_BAND_BUSHING_H

#include "chrono_vehicle/ChApiVehicle.h"
#include "chrono_vehicle/tracked_vehicle/track_assembly/ChTrackAssemblyBand.h"
#include "chrono_vehicle/tracked_vehicle/track_shoe/ChTrackShoeBandBushing.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle_tracked
/// @{

/// Definition of a continuous band track assembly using a bushing-based web
/// A track assembly consists of a sprocket, an idler (with tensioner mechanism),
/// a set of suspensions (road-wheel assemblies), and a collection of track shoes.
/// This class defines the template for a track assembly using a web modeled as
/// multiple rigid segments connected with bushings.
class CH_VEHICLE_API ChTrackAssemblyBandBushing : public ChTrackAssemblyBand {
  public:
    /// Construct a bushing-based track assembly on the specified vehicle side.
    ChTrackAssemblyBandBushing(const std::string& name,  ///< [in] name of the subsystem
                               VehicleSide side          ///< [in] assembly on left/right vehicle side
    );

    virtual ~ChTrackAssemblyBandBushing() {}

    /// Get the name of the vehicle subsystem template.
    virtual std::string GetTemplateName() const override { return "TrackAssemblyBandBushing"; }

    /// Get the number of track shoes.
    virtual size_t GetNumTrackShoes() const override { return m_shoes.size(); }

    /// Get a handle to the specified track shoe subsystem.
    virtual std::shared_ptr<ChTrackShoe> GetTrackShoe(size_t id) const override { return m_shoes[id]; }

  protected:
    ChTrackShoeBandBushingList m_shoes;  ///< track shoes

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
