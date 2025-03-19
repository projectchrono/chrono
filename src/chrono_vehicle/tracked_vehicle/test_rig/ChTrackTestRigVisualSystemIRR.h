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
//
// =============================================================================

#ifndef CH_TRACK_TESTRIG_VISUAL_SYSTEM_IRRLICHT_H
#define CH_TRACK_TESTRIG_VISUAL_SYSTEM_IRRLICHT_H

#include "chrono_irrlicht/ChVisualSystemIrrlicht.h"
#include "chrono_vehicle/tracked_vehicle/test_rig/ChTrackTestRig.h"

namespace chrono {
namespace vehicle {

class ChTTRKeyboardHandlerIRR;

/// @addtogroup vehicle_tracked
/// @{

/// Customized Chrono Irrlicht application for track test rig visualization.
class CH_VEHICLE_API ChTrackTestRigVisualSystemIRR : public irrlicht::ChVisualSystemIrrlicht {
  public:
    /// Construct a track test rig Irrlicht application.
    ChTrackTestRigVisualSystemIRR();

    ~ChTrackTestRigVisualSystemIRR() {}

    /// Attach a vehicle to this Irrlicht track rig visualization system.
    void AttachTTR(ChTrackTestRig* rig);

    /// Enable/disable rendering of track shoe body frames.
    void RenderTrackShoeFrames(bool state, double axis_length = 1);

    /// Enable/disable rendering of sprocket body frame.
    void RenderSprocketFrame(bool state, double axis_length = 1);

    /// Enable/disable rendering of idler body frame.
    void RenderIdlerFrame(bool state, double axis_length = 1);

    /// Initialize the visualizatin system.
    virtual void Initialize() override;

    /// Render the Irrlicht scene and additional visual elements.
    virtual void Render() override;

  private:
    void renderOtherGraphics();
    void renderContacts(const std::list<ChTrackContactManager::ContactInfo>& lst,
                        const ChColor& col,
                        bool normals,
                        bool forces,
                        double scale_normals,
                        double scale_forces);

    std::shared_ptr<ChTTRKeyboardHandlerIRR> m_keyboard_handler;

    ChTrackTestRig* m_rig;
    bool m_render_frame_shoes;
    bool m_render_frame_sprocket;
    bool m_render_frame_idler;
    double m_axis_shoes;
    double m_axis_sprocket;
    double m_axis_idler;

    friend class ChTTRKeyboardHandlerIRR;
};

/// @} vehicle_tracked

class CH_VEHICLE_API ChTTRKeyboardHandlerIRR : public irr::IEventReceiver {
  public:
    ChTTRKeyboardHandlerIRR(ChTrackTestRigVisualSystemIRR* app);
    bool OnEvent(const irr::SEvent& event) override;

  private:
    ChTrackTestRigVisualSystemIRR* m_app;
};

}  // end namespace vehicle
}  // end namespace chrono

#endif
