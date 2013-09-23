//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2011 Alessandro Tasora
// Copyright (c) 2013 Project Chrono
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be 
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//

#ifndef CHPOVTOOLS_H
#define CHPOVTOOLS_H

//////////////////////////////////////////////////
//
//   ChPovTools.h
//
//
//   Some functions to dump collision shapes and
//   other 'debugging' info to POV-ray scripts, for
//   batch ray-tracing rendering of the Chrono::Engine
//   animations
//
//   HEADER file for CHRONO,
//	 Multibody dynamics engine
//
// ------------------------------------------------
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////

//
// ***OBSOLETE***   
//  use the unit_POSTPROCESS

#include "core/ChMath.h"
#include "core/ChStream.h"
#include "physics/ChSystem.h"

 #include "BulletCollision/CollisionDispatch/btCollisionObject.h"
 #include "BulletCollision/CollisionShapes/btBoxShape.h"
 #include "BulletCollision/CollisionShapes/btSphereShape.h"

namespace chrono
{


static void savePovSphere(ChStreamOutAscii& povscript, const ChCoordsys<> csys, double rad, const char* obj_modifiers=0)
{
	povscript << "sphere \n{\n";
	povscript << "\t <0,0,0>, " << rad << "\n";
	if (obj_modifiers) povscript << "\t" << obj_modifiers;
	povscript << "\t quatRotation(<" << csys.rot.e0 << "," << csys.rot.e1 << "," << csys.rot.e2 << "," << csys.rot.e3 << ">)\n";
    povscript << "\t translate <" << csys.pos.x << "," << csys.pos.y << "," <<  csys.pos.z << ">\n";
	povscript << "}\n";
}

static void savePovBox(ChStreamOutAscii& povscript, const ChCoordsys<> csys, const ChVector<> size, const char* obj_modifiers=0)
{
	povscript << "box \n{\n";
	povscript << "\t <" << -0.5*size.x << "," << -0.5*size.y << "," << -0.5*size.z << ">, ";
	povscript << "<"    <<  0.5*size.x << "," <<  0.5*size.y << "," <<  0.5*size.z << ">, ";
	if (obj_modifiers) povscript << "\t" << obj_modifiers;
	povscript << "\t quatRotation(<" << csys.rot.e0 << "," << csys.rot.e1 << "," << csys.rot.e2 << "," << csys.rot.e3 << ">)\n";
    povscript << "\t translate <" << csys.pos.x << "," << csys.pos.y << "," << csys.pos.z << ">\n";
	povscript << "}\n";
}

static void POV_DumpCollisionShapes(ChStreamOutAscii& povscript, ChSystem* asystem)
{
	chrono::ChSystem::IteratorBodies myiter = asystem->IterBeginBodies();
	while (myiter != asystem->IterEndBodies())
	{
		collision::ChCollisionModel* mmod = (*myiter)->GetCollisionModel();
		if (collision::ChModelBulletBody* mbodymod = dynamic_cast<collision::ChModelBulletBody*>(mmod))
		{
			btCollisionObject* bumodel;
			bumodel = mbodymod->GetBulletModel();
			btCollisionShape* bushape;
			bushape = bumodel->getCollisionShape();
			if (bushape->getShapeType() == BOX_SHAPE_PROXYTYPE)
			{
				btBoxShape* ashape = (btBoxShape*) bushape;
				btVector3 buv = ashape->getHalfExtentsWithoutMargin();
				ChVector<> halfsize(buv.x(), buv.y(), buv.z());
				halfsize.x += mmod->GetSafeMargin();
				halfsize.y += mmod->GetSafeMargin();
				halfsize.z += mmod->GetSafeMargin();
				savePovBox(povscript, (*myiter)->GetCoord(), halfsize*2);
			}
			if (bushape->getShapeType() == SPHERE_SHAPE_PROXYTYPE)
			{
				btSphereShape* ashape = (btSphereShape*) bushape;
				savePovSphere(povscript, (*myiter)->GetCoord(), ashape->getRadius());
			}
		}
				
		++myiter;
	}
}




} // END OF NAMESPACE


#endif // END of ChPovTools.h

