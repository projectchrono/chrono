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

/// Create an Irrlicht mesh representing an ellipsoid. 
/// Ellispoid is centered in the origin.
ChApiIrr irr::scene::IAnimatedMesh* createEllipticalMesh(irr::f32 radiusH,
                                                         irr::f32 radiusV,
                                                         irr::f32 Ylow,
                                                         irr::f32 Yhigh,
                                                         irr::f32 offset,
                                                         irr::u32 polyCountX,
                                                         irr::u32 polyCountY);

/// Create an Irrlicht mesh representing a box. 
/// Box is centered in origin, extending +/- size in each direction.
ChApiIrr irr::scene::IMesh* createCubeMesh(const irr::core::vector3df& size);

/// Create an Irrlicht mesh representing a cylinder.
/// Cylinder axis is in Y direction, centered in origin. 
/// Cylinder tot length is 2*height, ranging from y=-height to y=+height.
ChApiIrr irr::scene::IMesh* createCylinderMesh(irr::f32 radius, irr::f32 height, irr::u32 tesselation);

/// Create an Irrlicht mesh representing a truncated cone.
/// Truncated cone axis is in Y direction, centered in origin. 
/// Truncated cone tot length is 2*height, ranging from y=-height to y=+height.
ChApiIrr irr::scene::IMesh* createTruncatedConeMesh(irr::f32 radius_top, irr::f32 radius_low, irr::f32 height, irr::u32 tesselation);

/// Create an Irrlicht mesh representing a cone.
/// Truncated cone axis is in Y direction, centered in origin. 
/// Truncated cone tot length is 2*height, ranging from y=-height to y=+height.
ChApiIrr irr::scene::IMesh* createConeMesh(irr::f32 radius_low, irr::f32 height, irr::u32 tesselation);

/// This function is based on a modified version of the irrlicht_bullet demo,
/// see  http://www.continuousphysics.com
/// It is used to convert an Irrlicht mesh into a ChTriangleMesh, which is used
/// for collision detection in Chrono.
///
/// ***OBSOLETE***
///
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
