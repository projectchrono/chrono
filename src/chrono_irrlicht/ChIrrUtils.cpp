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

#include "chrono_irrlicht/ChIrrUtils.h"

namespace chrono {
namespace irrlicht {
namespace tools {

using namespace irr;

void add_typical_Logo(IrrlichtDevice* device, const std::string& mlogofilename) {
    device->getGUIEnvironment()->addImage(device->getVideoDriver()->getTexture(mlogofilename.c_str()),
                                          irr::core::position2d<s32>(10, 10));
}

void add_typical_Lights(IrrlichtDevice* device,
                        irr::core::vector3df pos1,
                        irr::core::vector3df pos2,
                        double rad1,
                        double rad2,
                        irr::video::SColorf col1,
                        irr::video::SColorf col2) {
    // create lights
    /*scene::ILightSceneNode* mlight1 = */
    device->getSceneManager()->addLightSceneNode(0, pos1, col1, (f32)rad1);
    scene::ILightSceneNode* mlight2 = device->getSceneManager()->addLightSceneNode(0, pos2, col2, (f32)rad2);

    mlight2->enableCastShadow(false);
}

void add_typical_Sky(IrrlichtDevice* device, bool y_up, const std::string& mtexturedir) {
    // create sky box
    std::string str_lf = mtexturedir + "sky_lf.jpg";
    std::string str_up = mtexturedir + "sky_up.jpg";
    std::string str_dn = mtexturedir + "sky_dn.jpg";

    irr::video::ITexture* map_skybox_side = device->getVideoDriver()->getTexture(str_lf.c_str());

    auto mbox = device->getSceneManager()->addSkyBoxSceneNode(
        device->getVideoDriver()->getTexture(str_up.c_str()), device->getVideoDriver()->getTexture(str_dn.c_str()),
        map_skybox_side, map_skybox_side, map_skybox_side, map_skybox_side);

    if (!y_up)
        mbox->setRotation(irr::core::vector3df(90, 0, 0));
}

void add_typical_Camera(IrrlichtDevice* device, irr::core::vector3df pos, irr::core::vector3df targ, bool y_up) {
    // create and init camera
    RTSCamera* camera = new RTSCamera(device, device->getSceneManager()->getRootSceneNode(), device->getSceneManager(),
                                      -1, -160.0f, 1.0f, 0.003f);

    // camera->bindTargetAndRotation(true);
    if (!y_up)
        camera->setZUp();
    camera->setPosition(pos);
    camera->setTarget(targ);

    camera->setNearValue(0.1f);
    camera->setMinZoom(0.6f);
}

}  // end namespace tools
}  // end namespace irrlicht
}  // end namespace chrono
