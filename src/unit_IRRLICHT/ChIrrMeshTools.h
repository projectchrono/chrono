//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2010 Alessandro Tasora
// Copyright (c) 2013 Project Chrono
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be 
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//

#ifndef CHIRRMESHTOOLS_H
#define CHIRRMESHTOOLS_H


#include <irrlicht.h>
#include "geometry/ChCTriangleMesh.h"
#include "unit_IRRLICHT/ChApiIrr.h"


namespace irr {
namespace scene {


/// Some functions to allow easy creation of meshes for Irrlicht visualization


ChApiIrr IAnimatedMesh* createEllipticalMesh(f32 radiusH,
                                             f32 radiusV,
                                             f32 Ylow,
                                             f32 Yhigh,
                                             f32 offset,
                                             u32 polyCountX,
                                             u32 polyCountY);

/// Same as irr::CGeomentryCreator::createCubeMesh(), but with no shared normals
/// between faces.
ChApiIrr IMesh* createCubeMesh(const core::vector3df& size);

/// Same as irr::CGeomentryCreator::createCylinderMesh(), but with no shared
/// normals between caps and hull
ChApiIrr IMesh* createCylinderMesh(f32 radius,
                                   f32 length,
                                   u32 tesselation);

/// This function is based on a modified version of the irrlicht_bullet demo,
/// see  http://www.continuousphysics.com
/// It is used to convert an Irrlicht mesh into a ChTriangleMesh, which is used
/// for collision detection in Chrono::Engine.
ChApiIrr void fillChTrimeshFromIrlichtMesh(chrono::geometry::ChTriangleMesh* chTrimesh,
                                           IMesh*                            pMesh);

/// Given a ChTriangleMesh object, computes an Irrlicht mesh. 
/// Note: the ChTriangleMesh is a 'triangle soup', so no connectivity is used.
/// As a consequence, no Gourad/Phong shading is possible and all triangles will
/// look flat.
///
/// ***OBSOLETE***
///
ChApiIrr void fillIrlichtMeshFromChTrimesh(IMesh* pMesh,
                                           chrono::geometry::ChTriangleMesh* chTrimesh,
                                           video::SColor clr = video::SColor(255, 255,255,255));


} // END_OF_NAMESPACE____
} // END_OF_NAMESPACE____


#endif

