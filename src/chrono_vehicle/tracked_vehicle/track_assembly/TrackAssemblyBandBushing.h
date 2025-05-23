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
// Track assembly (band-bushing) model constructed from a JSON specification file
//
// =============================================================================

#ifndef TRACK_ASSEMBLY_BAND_BUSHING_H
#define TRACK_ASSEMBLY_BAND_BUSHING_H

#include <vector>

#include "chrono_vehicle/ChApiVehicle.h"
#include "chrono_vehicle/tracked_vehicle/track_assembly/ChTrackAssemblyBandBushing.h"

#include "chrono_thirdparty/rapidjson/document.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle_tracked
/// @{

/// Band-bushing track assembly model constructed from a JSON specification file
class CH_VEHICLE_API TrackAssemblyBandBushing : public ChTrackAssemblyBandBushing {
  public:
    TrackAssemblyBandBushing(const std::string& filename);
    TrackAssemblyBandBushing(const rapidjson::Document& d);
    ~TrackAssemblyBandBushing() {}

    virtual const ChVector3d GetSprocketLocation() const override { return m_sprocket_loc; }
    virtual const ChVector3d GetIdlerLocation() const override { return m_idler_loc; }
    virtual const ChVector3d GetRoadWhelAssemblyLocation(int which) const override { return m_susp_locs[which]; }
    virtual const ChVector3d GetRollerLocation(int which) const override { return m_roller_locs[which]; }

  private:
    virtual void Create(const rapidjson::Document& d) override;

    void ReadSprocket(const std::string& filename, int output);
    void ReadTrackShoes(const std::string& filename, int num_shoes, int output);

    int m_num_susp;
    int m_num_rollers;
    int m_num_track_shoes;

    ChVector3d m_sprocket_loc;
    ChVector3d m_idler_loc;
    std::vector<ChVector3d> m_susp_locs;
    std::vector<ChVector3d> m_roller_locs;
};

/// @} vehicle_tracked

}  // end namespace vehicle
}  // end namespace chrono

#endif
