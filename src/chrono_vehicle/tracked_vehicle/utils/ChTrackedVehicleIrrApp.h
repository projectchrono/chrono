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
// Irrlicht-based visualization wrapper for tracked vehicles.
// This class extends ChVehicleIrrApp.
//
// =============================================================================

#ifndef CH_TRACKED_VEHICLE_IRRAPP_H
#define CH_TRACKED_VEHICLE_IRRAPP_H

#include "chrono_vehicle/utils/ChVehicleIrrApp.h"
#include "chrono_vehicle/tracked_vehicle/ChTrackedVehicle.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle_tracked_utils
/// @{

/// Customized Chrono Irrlicht application for tracked vehicle visualization.
class CH_VEHICLE_API ChTrackedVehicleIrrApp : public ChVehicleIrrApp {
  public:
    /// Construct a tracked vehicle Irrlicht application.
    ChTrackedVehicleIrrApp(ChVehicle* vehicle,                              ///< associated vehicle system
                           const std::wstring& title = L"Chrono::Vehicle",  ///< window title
                           const irr::core::dimension2d<irr::u32>& dims =
                               irr::core::dimension2d<irr::u32>(1000, 800),        ///< window dimensions
                           irrlicht::VerticalDir vert = irrlicht::VerticalDir::Z,  ///< vertical camera direction
                           irr::ELOG_LEVEL log_level = irr::ELL_INFORMATION        ///< Irrlicht logging level
    );

    ~ChTrackedVehicleIrrApp() {}

    /// Enable/disable rendering of track shoe body frames.
    void RenderTrackShoeFrames(VehicleSide side, bool state, double axis_length = 1);

    /// Enable/disable rendering of sprocket body frame.
    void RenderSprocketFrame(VehicleSide side, bool state, double axis_length = 1);

    /// Enable/disable rendering of idler body frame.
    void RenderIdlerFrame(VehicleSide side, bool state, double axis_length = 1);

  private:
    virtual void renderOtherGraphics() override;
    virtual void renderOtherStats(int left, int top) override;
    void renderContacts(const std::list<ChTrackContactManager::ContactInfo>& lst,
                        const irr::video::SColor& col,
                        bool normals,
                        bool forces,
                        double scale_normals,
                        double scale_forces);

    ChTrackedVehicle* m_tvehicle;
    bool m_render_frame_shoes[2];
    bool m_render_frame_sprockets[2];
    bool m_render_frame_idlers[2];
    double m_axis_shoes[2];
    double m_axis_sprockets[2];
    double m_axis_idlers[2];
};

/// @} vehicle_tracked_utils

}  // end namespace vehicle
}  // end namespace chrono

#endif
