// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2021 projectchrono.org
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
// Irrlicht-based visualization wrapper for track test rig.
// This class extends ChVehicleVisualSystemIrrlicht.
//
// =============================================================================

#ifndef CH_TRACK_TESTRIG_VISUAL_SYSTEM_IRRLICHT_H
#define CH_TRACK_TESTRIG_VISUAL_SYSTEM_IRRLICHT_H

#include "chrono_vehicle/utils/ChVehicleVisualSystemIrrlicht.h"
#include "chrono_vehicle/tracked_vehicle/test_rig/ChTrackTestRig.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle_tracked_utils
/// @{

/// Customized Chrono Irrlicht application for track test rig visualization.
class CH_VEHICLE_API ChTrackTestRigVisualSystemIrrlicht : public ChVehicleVisualSystemIrrlicht {
  public:
    /// Construct a track test rig Irrlicht application.
    ChTrackTestRigVisualSystemIrrlicht();

    ~ChTrackTestRigVisualSystemIrrlicht() {}

    /// Enable/disable rendering of track shoe body frames.
    void RenderTrackShoeFrames(bool state, double axis_length = 1);

    /// Enable/disable rendering of sprocket body frame.
    void RenderSprocketFrame(bool state, double axis_length = 1);

    /// Enable/disable rendering of idler body frame.
    void RenderIdlerFrame(bool state, double axis_length = 1);

  private:
    virtual void OnAttachToVehicle() override;
    virtual void renderOtherGraphics() override;
    void renderContacts(const std::list<ChTrackContactManager::ContactInfo>& lst,
                        const irr::video::SColor& col,
                        bool normals,
                        bool forces,
                        double scale_normals,
                        double scale_forces);

    ChTrackTestRig* m_rig;
    bool m_render_frame_shoes;
    bool m_render_frame_sprocket;
    bool m_render_frame_idler;
    double m_axis_shoes;
    double m_axis_sprocket;
    double m_axis_idler;
};

/// @} vehicle_tracked_utils

}  // end namespace vehicle
}  // end namespace chrono

#endif
