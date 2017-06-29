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

#ifndef CHIRRMESHTOOLS_H
#define CHIRRMESHTOOLS_H

#include <irrlicht.h>

#include "chrono/geometry/ChTriangleMesh.h"
#include "chrono_irrlicht/ChApiIrr.h"

namespace chrono {
namespace irrlicht {

/// @addtogroup irrlicht_module
/// @{

/// Some functions to allow easy creation of meshes for Irrlicht visualization
ChApiIrr irr::scene::IAnimatedMesh* createEllipticalMesh(irr::f32 radiusH,
                                                         irr::f32 radiusV,
                                                         irr::f32 Ylow,
                                                         irr::f32 Yhigh,
                                                         irr::f32 offset,
                                                         irr::u32 polyCountX,
                                                         irr::u32 polyCountY);

/// Same as irr::CGeomentryCreator::createCubeMesh(), but with no shared normals
/// between faces.
ChApiIrr irr::scene::IMesh* createCubeMesh(const irr::core::vector3df& size);

/// Same as irr::CGeomentryCreator::createCylinderMesh(), but with no shared
/// normals between caps and hull
ChApiIrr irr::scene::IMesh* createCylinderMesh(irr::f32 radius, irr::f32 length, irr::u32 tesselation);

/// This function is based on a modified version of the irrlicht_bullet demo,
/// see  http://www.continuousphysics.com
/// It is used to convert an Irrlicht mesh into a ChTriangleMesh, which is used
/// for collision detection in Chrono.
ChApiIrr void fillChTrimeshFromIrlichtMesh(geometry::ChTriangleMesh* chTrimesh, irr::scene::IMesh* pMesh);

/// Given a ChTriangleMesh object, computes an Irrlicht mesh.
/// Note: the ChTriangleMesh is a 'triangle soup', so no connectivity is used.
/// As a consequence, no Gourad/Phong shading is possible and all triangles will
/// look flat.
///
/// ***OBSOLETE***
///
ChApiIrr void fillIrlichtMeshFromChTrimesh(irr::scene::IMesh* pMesh,
                                           geometry::ChTriangleMesh* chTrimesh,
                                           irr::video::SColor clr = irr::video::SColor(255, 255, 255, 255));

/// @} irrlicht_module

}  // end namespace irrlicht
}  // end namespace chrono

#endif
