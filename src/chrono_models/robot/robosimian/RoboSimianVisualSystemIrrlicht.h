// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2018 projectchrono.org
// All right reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Author: Radu Serban
// =============================================================================
//
// Customized Chrono Irrlicht visual system for RoboSimian.
//
// =============================================================================

#ifndef ROBOSIMIAN_VISUAL_SYSTEM_IRRLICHT_H
#define ROBOSIMIAN_VISUAL_SYSTEM_IRRLICHT_H

#include "chrono_irrlicht/ChVisualSystemIrrlicht.h"
#include "chrono_irrlicht/ChIrrTools.h"

#include "chrono_models/robot/robosimian/RoboSimian.h"

namespace chrono {
namespace robosimian {

/// @addtogroup robot_models_robosimian
/// @{

/// Customized Chrono Irrlicht visualization system for RoboSimian.
/// Provides a simple GUI with various stats and encapsulates an event receiver to allow visualizing the collision
/// shapes (toggle with the 'C' key).
class CH_MODELS_API RoboSimianVisualSystemIrrlicht : public irrlicht::ChVisualSystemIrrlicht {
  public:
    /// Construct a RoboSimian Irrlicht application.
    RoboSimianVisualSystemIrrlicht(RoboSimian* robot,  ///< associated RoboSimian robot
                                   RS_Driver* driver   ///< associated robot driver
    );

    ~RoboSimianVisualSystemIrrlicht();

    /// Render the Irrlicht scene and additional visual elements.
    virtual void Render() override;

  private:
    void renderTextBox(const std::string& msg,
                       int xpos,
                       int ypos,
                       int length = 120,
                       int height = 15,
                       irr::video::SColor color = irr::video::SColor(255, 20, 20, 20));

    RoboSimian* m_robot;  ///< associated RoboSimian robot
    RS_Driver* m_driver;  ///< associated robot driver

    irr::IEventReceiver* m_erecv;  ///< Irrlichr event receiver

    int m_HUD_x;  ///< x-coordinate of upper-left corner of HUD elements
    int m_HUD_y;  ///< y-coordinate of upper-left corner of HUD elements

    friend class RS_IEventReceiver;
};

/// @} robot_models_robosimian

}  // namespace robosimian
}  // namespace chrono

#endif
