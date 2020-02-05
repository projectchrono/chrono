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
// Track assembly (double-pin) model constructed from a JSON specification file
//
// =============================================================================

#ifndef TRACK_ASSEMBLY_DOUBLE_PIN_H
#define TRACK_ASSEMBLY_DOUBLE_PIN_H

#include <vector>

#include "chrono_vehicle/ChApiVehicle.h"
#include "chrono_vehicle/tracked_vehicle/track_assembly/ChTrackAssemblyDoublePin.h"

#include "chrono_thirdparty/rapidjson/document.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle_tracked
/// @{

/// Double-pin track assembly model constructed from a JSON specification file
class CH_VEHICLE_API TrackAssemblyDoublePin : public ChTrackAssemblyDoublePin {
  public:
    TrackAssemblyDoublePin(const std::string& filename);
    TrackAssemblyDoublePin(const rapidjson::Document& d);
    ~TrackAssemblyDoublePin() {}

    virtual const ChVector<> GetSprocketLocation() const override { return m_sprocket_loc; }
    virtual const ChVector<> GetIdlerLocation() const override { return m_idler_loc; }
    virtual const ChVector<> GetRoadWhelAssemblyLocation(int which) const override { return m_susp_locs[which]; }
    virtual const ChVector<> GetRollerLocation(int which) const override { return m_roller_locs[which]; }

  private:
    virtual void Create(const rapidjson::Document& d) override;

    void ReadSprocket(const std::string& filename, int output);
    void ReadTrackShoes(const std::string& filename, int num_shoes, int output);

    int m_num_susp;
    int m_num_rollers;
    int m_num_track_shoes;

    ChVector<> m_sprocket_loc;
    ChVector<> m_idler_loc;
    std::vector<ChVector<>> m_susp_locs;
    std::vector<ChVector<>> m_roller_locs;
};

/// @} vehicle_tracked

}  // end namespace vehicle
}  // end namespace chrono

#endif
