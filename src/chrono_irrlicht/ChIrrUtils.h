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

#ifndef CHIRRUTILS_H
#define CHIRRUTILS_H

#include <string>
#include <irrlicht.h>

#include "chrono/core/ChGlobal.h"

#include "chrono_irrlicht/ChApiIrr.h"
#include "chrono_irrlicht/ChIrrCamera.h"

namespace chrono {
namespace irrlicht {
namespace tools {

/// @addtogroup irrlicht_module
/// @{

/// A very basic and simple function which is just a shortcut to avoid lot of
/// typing when someone wants to add a logo in a 3D scene, using Irrlicht.
ChApiIrr void add_typical_Logo(irr::IrrlichtDevice* device,
                               const std::string& mlogofilename = GetChronoDataFile("logo_chronoengine_alpha.png"));

/// A very basic and simple function which is just a shortcut to avoid lot of
/// typing when someone wants to add two lights in a 3D scene, using Irrlicht.
/// Note: if you want more precise control on lights, just use plain commands
/// of Irrlicht.
ChApiIrr void add_typical_Lights(irr::IrrlichtDevice* device,
                                 irr::core::vector3df pos1 = irr::core::vector3df(30.f, 100.f, 30.f),
                                 irr::core::vector3df pos2 = irr::core::vector3df(30.f, 80.f, -30.f),
                                 double rad1 = 290,
                                 double rad2 = 190,
                                 irr::video::SColorf col1 = irr::video::SColorf(0.7f, 0.7f, 0.7f, 1.0f),
                                 irr::video::SColorf col2 = irr::video::SColorf(0.7f, 0.8f, 0.8f, 1.0f));

/// A very basic and simple function which is just a  shortcut to avoid lot of
/// typing when someone wants to add a sky dome (sky box) in a 3D scene, using
/// Irrlicht.
/// Note: it is assumed that the specified "mtexturedir" directory contains
/// the following three texture images: sky_lf.jpg, sky_up.jpg, sky_dn.jpg
ChApiIrr void add_typical_Sky(irr::IrrlichtDevice* device,
                              bool y_up = true,
                              const std::string& mtexturedir = GetChronoDataFile("skybox/"));

/// A very basic and simple function which is just a shortcut to avoid lot of
/// typing when someone wants to add a Maya-like camera in a Irrlicht 3D scene.
/// The camera rotation/pan is controlled by mouse left and right buttons,
/// the zoom is controlled by mouse wheel or rmb+lmb+mouse, the position can
/// be changed also with keyboard up/down/left/right arrows, the height can be
/// changed with keyboard 'PgUp' and 'PgDn' keys. Optional parameters are
/// position and target. Note: if you want more precise control on camera
/// specs, just use plain commands of Irrlicht.
ChApiIrr void add_typical_Camera(irr::IrrlichtDevice* device,
                                 irr::core::vector3df pos = irr::core::vector3df(0, 0, -8),
                                 irr::core::vector3df targ = irr::core::vector3df(0, 0, 0),
                                 bool y_up = true);

/// @} irrlicht_module

}  // end namespace tools
}  // end namespace irrlicht
}  // end namespace chrono

#endif
