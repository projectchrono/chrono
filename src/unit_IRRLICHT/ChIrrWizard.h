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


#ifndef CHIRRWIZARD_H
#define CHIRRWIZARD_H

#include <string>
#include <irrlicht.h>

#include "physics/ChGlobal.h"

#include "unit_IRRLICHT/ChApiIrr.h"
#include "unit_IRRLICHT/ChIrrCamera.h"


namespace irr {


/// Class with static functions which allow creation of Irrlicht frequent
/// 'scene nodes' like lights, camera, sky box etc. with very simple statements.

class ChApiIrr ChIrrWizard
{
public:

  /// A very basic and simple function which is just a shortcut to avoid lot of
  /// typing when someone wants to add a logo in a 3D scene, using Irrlicht.

  static void add_typical_Logo(
                  IrrlichtDevice*    device,
                  const std::string& mlogofilename = chrono::GetChronoDataFile("logo_chronoengine_alpha.png"));


  /// A very basic and simple function which is just a shortcut to avoid lot of
  /// typing when someone wants to add two lights in a 3D scene, using Irrlicht.
  /// Note: if you want more precise control on lights, just use plain commands
  /// of Irrlicht.

  static void add_typical_Lights(
                  IrrlichtDevice* device,
                  core::vector3df pos1 = core::vector3df(30.f, 100.f,  30.f),
                  core::vector3df pos2 = core::vector3df(30.f, 80.f, -30.f),
                  double          rad1 = 290,
                  double          rad2 = 190,
                  video::SColorf  col1 = video::SColorf(0.7f,0.7f,0.7f,1.0f),
                  video::SColorf  col2 = video::SColorf(0.7f,0.8f,0.8f,1.0f));

  /// A very basic and simple function which is just a  shortcut to avoid lot of
  /// typing when someone wants to add a sky dome (sky box) in a 3D scene, using
  /// Irrlicht. 
  /// Note: it is assumed that the specified "mtexturedir" directory contains
  /// the following three texture images: sky_lf.jpg, sky_up.jpg, sky_dn.jpg

  static void add_typical_Sky(
                  IrrlichtDevice*    device,
                  const std::string& mtexturedir = chrono::GetChronoDataFile("skybox/"));

  /// A very basic and simple function which is just a shortcut to avoid lot of
  /// typing when someone wants to add a Maya-like camera in a Irrlicht 3D scene.
  /// The camera rotation/pan is controlled by mouse left and right buttons,
  /// the zoom is controlled by mouse wheel or rmb+lmb+mouse, the position can 
  /// be changed also with keyboard up/down/left/right arrows, the height can be 
  /// changed with keyboard 'PgUp' and 'PgDn' keys. Optional parameters are
  /// position and target. Note: if you want more precise control on camera
  /// specs, just use plain commands of Irrlicht.

  static void add_typical_Camera(
                  IrrlichtDevice* device,
                  core::vector3df mpos = core::vector3df(0,0,-8),
                  core::vector3df mtarg = core::vector3df(0,0,0));

};


} // END_OF_NAMESPACE____


#endif

