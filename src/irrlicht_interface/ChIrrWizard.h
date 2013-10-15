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

#ifndef CHIRRWIZARD_H
#define CHIRRWIZARD_H

//////////////////////////////////////////////////
//
//   ChIrrWizard.h
//
//   FOR IRRLICHT USERS ONLY!
//
//   Class with static functions which allow creation
//   of Irrlicht frequent 'scene nodes' like lights,
//   camera, sky box etc. with very simple statements.
//
//   HEADER file for CHRONO,
//	 Multibody dynamics engine
//
// ------------------------------------------------
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////

#include <irrlicht.h>
#include "physics/ChSystem.h"

#include "ChIrrCamera.h"



namespace irr
{



/// Class with static functions which allow creation
/// of Irrlicht frequent 'scene nodes' like lights,
/// camera, sky box etc. with very simple statements.

class ChIrrWizard
{
public: 

			/// A very basic and simple function which is just a 
			/// shortcut to avoid lot of typing when someone wants to
			/// add the Chrono logo in a 3D scene, using Irrlicht. 
			/// Note: in "mtexturedir" directory, relative to exe dir,
			/// you must provide logo_chronoengine_alpha.png (or pass
			/// the file name as 3rd optional parameter 'mlogofilename')

	static void add_typical_Logo(IrrlichtDevice* device, 
								const char* mtexturedir = "../data/",
								const char* mlogofilename = "logo_chronoengine_alpha.png")
	{
			core::stringc str_logo = mtexturedir;
			str_logo += mlogofilename;

			device->getGUIEnvironment()->addImage(
				device->getVideoDriver()->getTexture( str_logo.c_str() ),
				core::position2d<s32>(10,10));
	}


			/// A very basic and simple function which is just a 
			/// shortcut to avoid lot of typing when someone wants to
			/// add two lights and the Chrono logo in a 3D 
			/// scene, using Irrlicht. 
			/// Note: if you want more precise control on lights,
			/// just use plain commands of Irrlicht
			/// (see the code below).

	static void add_typical_Lights(IrrlichtDevice* device, 
								core::vector3df pos1 = core::vector3df(30.f, 100.f,  30.f),
								core::vector3df pos2 = core::vector3df(30.f, 80.f, -30.f),
								double rad1 = 290, double rad2 = 190,
								video::SColorf col1  = video::SColorf(0.7f,0.7f,0.7f,1.0f),
								video::SColorf col2  = video::SColorf(0.7f,0.8f,0.8f,1.0f))
	{
			// create lights
			irr::scene::ILightSceneNode* mlight1 = device->getSceneManager()->addLightSceneNode(
				0,pos1, 
				col1, (f32)rad1);
			irr::scene::ILightSceneNode* mlight2 = device->getSceneManager()->addLightSceneNode(
				0,pos2,  
				col2, (f32)rad2);
			mlight2->enableCastShadow(false);
	}

			/// A very basic and simple function which is just a 
			/// shortcut to avoid lot of typing when someone wants to
			/// add a sky dome (sky box) in a 3D 
			/// scene, using Irrlicht. 
			/// Note: in "mtexturedir" directory, relative to exe dir,
			/// you must provide sky_lf.jpg, sky_up.jpg, sky_dn.jpg pictures.

	static void add_typical_Sky(IrrlichtDevice* device, 
							  const char* mtexturedir = "../data/skybox/" )
	{
			// create sky box
			core::stringc str_lf = mtexturedir;
			str_lf +="sky_lf.jpg";
			core::stringc str_up = mtexturedir;
			str_up +="sky_up.jpg";
			core::stringc str_dn = mtexturedir;
			str_dn +="sky_dn.jpg";

			video::ITexture* map_skybox_side = device->getVideoDriver()->getTexture(
				str_lf.c_str() );
			device->getSceneManager()->addSkyBoxSceneNode(
				device->getVideoDriver()->getTexture(str_up.c_str()),
				device->getVideoDriver()->getTexture(str_dn.c_str()),
				map_skybox_side,
				map_skybox_side,
				map_skybox_side,
				map_skybox_side);
	}

			/// A very basic and simple function which is just a 
			/// shortcut to avoid lot of typing when someone wants to
			/// add a Maya-like camera in a Irrlicht 3D scene. 
			/// The camera rotation/pan is controlled by mouse left and right buttons,
			/// the zoom is controlled by mouse wheel or rmb+lmb+mouse, the position can 
			/// be changed also with keyboard up/down/left/right arrows, the height can be 
			/// changed with keyboard 'PgUp' and 'PgDn' keys.
			/// Optional parameters are position and target.
			/// Note: if you want more precise control on camera specs,
			/// just use plain commands of Irrlicht.

	static void add_typical_Camera(IrrlichtDevice* device,
									core::vector3df mpos = core::vector3df(0,0,-8),
									core::vector3df mtarg = core::vector3df(0,0,0))
	{
			// create and init camera
			//scene::ICameraSceneNode* camera = device->getSceneManager()->addCameraSceneNodeMaya();
			//scene::ICameraSceneNode* camera = device->getSceneManager()->addCameraSceneNodeFPS();
			scene::RTSCamera* camera = new scene::RTSCamera(device, device->getSceneManager()->getRootSceneNode(), device->getSceneManager(),-1, -160.0f, 3.0f, 3.0f); 

			//camera->bindTargetAndRotation(true);
			camera->setPosition(mpos);
			camera->setTarget(mtarg);

			camera->setNearValue(0.1f);
			camera->setMinZoom(0.6f);
	}


}; // end of wizard class 



} // END_OF_NAMESPACE____

#endif // END of ChIrrWizard.h

