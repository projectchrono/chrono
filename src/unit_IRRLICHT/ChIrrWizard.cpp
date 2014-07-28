//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2010, 2012 Alessandro Tasora
// Copyright (c) 2013 Project Chrono
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be 
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//


#include "unit_IRRLICHT/ChIrrWizard.h"


namespace irr {


void ChIrrWizard::add_typical_Logo(IrrlichtDevice*  device,
                                   const char*      mtexturedir,
                                   const char*      mlogofilename)
{
  core::stringc str_logo = mtexturedir;
  str_logo += mlogofilename;

  device->getGUIEnvironment()->addImage(
    device->getVideoDriver()->getTexture( str_logo.c_str() ),
    core::position2d<s32>(10,10));
}


void ChIrrWizard::add_typical_Lights(IrrlichtDevice* device,
                                     core::vector3df pos1,
                                     core::vector3df pos2,
                                     double          rad1,
                                     double          rad2,
                                     video::SColorf  col1,
                                     video::SColorf  col2)
{
  // create lights
  irr::scene::ILightSceneNode* mlight1 = 
    device->getSceneManager()->addLightSceneNode(0, pos1, col1, (f32)rad1);

  irr::scene::ILightSceneNode* mlight2 = 
    device->getSceneManager()->addLightSceneNode(0, pos2, col2, (f32)rad2);

  mlight2->enableCastShadow(false);
}


void ChIrrWizard::add_typical_Sky(IrrlichtDevice* device,
                                  const char*     mtexturedir)
{
  // create sky box
  core::stringc str_lf = mtexturedir;
  str_lf +="sky_lf.jpg";
  core::stringc str_up = mtexturedir;
  str_up +="sky_up.jpg";
  core::stringc str_dn = mtexturedir;
  str_dn +="sky_dn.jpg";

  video::ITexture* map_skybox_side = 
    device->getVideoDriver()->getTexture(str_lf.c_str() );

  device->getSceneManager()->addSkyBoxSceneNode(
    device->getVideoDriver()->getTexture(str_up.c_str()),
    device->getVideoDriver()->getTexture(str_dn.c_str()),
    map_skybox_side,
    map_skybox_side,
    map_skybox_side,
    map_skybox_side);
}


void ChIrrWizard::add_typical_Camera(IrrlichtDevice* device,
                                     core::vector3df mpos,
                                     core::vector3df mtarg)
{
  // create and init camera
  scene::RTSCamera* camera = 
    new scene::RTSCamera(device,
                         device->getSceneManager()->getRootSceneNode(),
                         device->getSceneManager(),
                         -1, -160.0f, 1.0f, 0.003f);

  //camera->bindTargetAndRotation(true);
  camera->setPosition(mpos);
  camera->setTarget(mtarg);

  camera->setNearValue(0.1f);
  camera->setMinZoom(0.6f);
}


} // END_OF_NAMESPACE____
