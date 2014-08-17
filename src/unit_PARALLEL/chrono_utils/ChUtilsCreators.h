// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All right reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban
// =============================================================================
//
// Utility functions to facilitate adding contact and visualization geometry to
// a body.
//
// =============================================================================

#ifndef CH_UTILS_CREATORS_H
#define CH_UTILS_CREATORS_H

#include <cmath>
#include <vector>
#include <string>

#include "core/ChSmartpointers.h"
#include "core/ChVector.h"
#include "core/ChQuaternion.h"

#include "physics/ChSystem.h"
#include "physics/ChSystemDEM.h"
#include "physics/ChBody.h"
#include "physics/ChBodyDEM.h"
#include "physics/ChMaterialSurface.h"
#include "physics/ChMaterialSurfaceDEM.h"

#include "assets/ChSphereShape.h"
#include "assets/ChEllipsoidShape.h"
#include "assets/ChBoxShape.h"
#include "assets/ChCapsuleShape.h"
#include "assets/ChCylinderShape.h"
#include "assets/ChConeShape.h"
#include "assets/ChTriangleMeshShape.h"
#include "assets/ChRoundedBoxShape.h"
#include "assets/ChRoundedConeShape.h"
#include "assets/ChRoundedCylinderShape.h"

#include "chrono_utils/ChApiUtils.h"
#include "chrono_utils/ChUtilsCommon.h"

#include "collision/ChCModelBulletBody.h"

namespace chrono {
namespace utils {


// -----------------------------------------------------------------------------
// AddSphereGeometry
// AddEllipsoidGeometry
// AddBoxGeometry
// AddCapsuleGeometry
// AddCylinderGeometry
// AddConeGeometry
// AddTriangleMeshGeometry
// AddRoundedBoxGeometry
// AddRoundedCylinderGeometry
// AddTorusGeometry
// Utility functions for adding contact and asset geometry shapes to a body
// -----------------------------------------------------------------------------
inline
void AddSphereGeometry(ChBody*               body,
                       double                radius,
                       const ChVector<>&     pos = ChVector<>(0,0,0),
                       const ChQuaternion<>& rot = ChQuaternion<>(1,0,0,0))
{
  body->GetCollisionModel()->AddSphere(radius, pos);

  ChSharedPtr<ChSphereShape> sphere(new ChSphereShape);
  sphere->GetSphereGeometry().rad = radius;
  sphere->Pos = pos;
  sphere->Rot = rot;

  body->GetAssets().push_back(sphere);
}

inline
void AddEllipsoidGeometry(ChBody*               body,
                          const ChVector<>&     size,
                          const ChVector<>&     pos = ChVector<>(0,0,0),
                          const ChQuaternion<>& rot = ChQuaternion<>(1,0,0,0))
{
  body->GetCollisionModel()->AddEllipsoid(size.x, size.y, size.z, pos, rot);

  ChSharedPtr<ChEllipsoidShape> ellipsoid(new ChEllipsoidShape);
  ellipsoid->GetEllipsoidGeometry().rad = size;
  ellipsoid->Pos = pos;
  ellipsoid->Rot = rot;

  body->GetAssets().push_back(ellipsoid);
}

inline
void AddBoxGeometry(ChBody*               body,
                    const ChVector<>&     size,
                    const ChVector<>&     pos = ChVector<>(0,0,0),
                    const ChQuaternion<>& rot = ChQuaternion<>(1,0,0,0))
{
  body->GetCollisionModel()->AddBox(size.x, size.y, size.z, pos, rot);

  ChSharedPtr<ChBoxShape> box(new ChBoxShape);
  box->GetBoxGeometry().Size = size;
  box->Pos = pos;
  box->Rot = rot;

  body->GetAssets().push_back(box);
}

inline
void AddCapsuleGeometry(ChBody*               body,
                        double                radius,
                        double                hlen,
                        const ChVector<>&     pos = ChVector<>(0,0,0),
                        const ChQuaternion<>& rot = ChQuaternion<>(1,0,0,0))
{
  body->GetCollisionModel()->AddCapsule(radius, hlen, pos, rot);

  ChSharedPtr<ChCapsuleShape> capsule(new ChCapsuleShape);
  capsule->GetCapsuleGeometry().rad = radius;
  capsule->GetCapsuleGeometry().hlen = hlen;
  capsule->Pos = pos;
  capsule->Rot = rot;

  body->GetAssets().push_back(capsule);
}

inline
void AddCylinderGeometry(ChBody*               body,
                         double                radius,
                         double                hlen,
                         const ChVector<>&     pos = ChVector<>(0,0,0),
                         const ChQuaternion<>& rot = ChQuaternion<>(1,0,0,0))
{
  body->GetCollisionModel()->AddCylinder(radius, radius, hlen, pos, rot);

  ChSharedPtr<ChCylinderShape> cylinder(new ChCylinderShape);
  cylinder->GetCylinderGeometry().rad = radius;
  cylinder->GetCylinderGeometry().p1 = ChVector<>(0,  hlen, 0);
  cylinder->GetCylinderGeometry().p2 = ChVector<>(0, -hlen, 0);
  cylinder->Pos = pos;
  cylinder->Rot = rot;

  body->GetAssets().push_back(cylinder);
}

inline
void AddConeGeometry(ChBody*               body,
                     double                radius,
                     double                height,
                     const ChVector<>&     pos = ChVector<>(0,0,0),
                     const ChQuaternion<>& rot = ChQuaternion<>(1,0,0,0))
{
  body->GetCollisionModel()->AddCone(radius, radius, height, pos, rot);

  ChSharedPtr<ChConeShape> cone(new ChConeShape);
  cone->GetConeGeometry().rad = ChVector<>(radius, height, radius);
  cone->Pos = pos;
  cone->Rot = rot;

  body->GetAssets().push_back(cone);
}

inline
void AddTriangleMeshGeometry(ChBody*               body,
                             const std::string&    obj_filename,
                             const std::string&    name,
                             const ChVector<>&     pos = ChVector<>(0,0,0),
                             const ChQuaternion<>& rot = ChQuaternion<>(1,0,0,0))
{
  geometry::ChTriangleMeshConnected trimesh;
  trimesh.LoadWavefrontMesh(obj_filename, false, false);

  for (int i = 0; i < trimesh.m_vertices.size(); i++)
    trimesh.m_vertices[i] = pos + rot.Rotate(trimesh.m_vertices[i]);

  body->GetCollisionModel()->AddTriangleMesh(trimesh, false, false);

  ChSharedPtr<ChTriangleMeshShape> trimesh_shape(new ChTriangleMeshShape);
  trimesh_shape->SetMesh(trimesh);
  trimesh_shape->SetName(name);
  trimesh_shape->Pos = ChVector<>(0,0,0);
  trimesh_shape->Rot = ChQuaternion<>(1,0,0,0);

  body->GetAssets().push_back(trimesh_shape);
}

inline
void AddRoundedBoxGeometry(
          ChBody*               body,
          const ChVector<>&     size,
          double                srad,
          const ChVector<>&     pos = ChVector<>(0,0,0),
          const ChQuaternion<>& rot = ChQuaternion<>(1,0,0,0))
{
   body->GetCollisionModel()->AddRoundedBox(size.x, size.y, size.z, srad, pos, rot);

   ChSharedPtr<ChRoundedBoxShape> box(new ChRoundedBoxShape);
   box->GetRoundedBoxGeometry().Size = size;
   box->GetRoundedBoxGeometry().radsphere = srad;
   box->Pos = pos;
   box->Rot = rot;
   body->GetAssets().push_back(box);
}

inline
void AddRoundedCylinderGeometry(
          ChBody*               body,
          double                radius,
          double                hlen,
          double                srad,
          const ChVector<>&     pos = ChVector<>(0,0,0),
          const ChQuaternion<>& rot = ChQuaternion<>(1,0,0,0))
{
  body->GetCollisionModel()->AddRoundedCylinder(radius, radius, hlen, srad, pos, rot);

  ChSharedPtr<ChRoundedCylinderShape> rcyl(new ChRoundedCylinderShape);
  rcyl->GetRoundedCylinderGeometry().rad = radius;
  rcyl->GetRoundedCylinderGeometry().hlen = hlen;
  rcyl->GetRoundedCylinderGeometry().radsphere = srad;
  rcyl->Pos = pos;
  rcyl->Rot = rot;
  body->GetAssets().push_back(rcyl);
}

//Creates a compound torus shape using cylinders
inline
void AddTorusGeometry(
          ChBody*               body,
          double                radius,
          double                thickness,
          int                   segments = 20,
          int                   angle = 360,
          const ChVector<>&     pos = ChVector<>(0,0,0),
          const ChQuaternion<>& rot = ChQuaternion<>(1,0,0,0))
{
   for (int i = 0; i < angle; i += angle/segments) {
         double angle = i * CH_C_PI / 180.0;
         double x = cos(angle) * radius;
         double z = sin(angle) * radius;
         Quaternion q = chrono::Q_from_AngAxis(-angle, VECT_Y) % chrono::Q_from_AngAxis(CH_C_PI/2.0, VECT_X);

         AddCylinderGeometry(body,thickness,thickness*.75 ,ChVector<>(x, 0, z) + pos,q);
   }
}


// -----------------------------------------------------------------------------
// CreateBoxContainerDEM
// CreateBoxContainerDVI
// InitializeObjectDVI
// FinalizeObjectDVI
// Utility functions for creating objects
// -----------------------------------------------------------------------------

// Create a fixed body with contact and asset geometry representing a box with 5
// walls (no top).
CH_UTILS_API
void CreateBoxContainerDEM(ChSystem*                           system,
                           int                                 id,
                           ChSharedPtr<ChMaterialSurfaceDEM>&  mat,
                           const ChVector<>&                   hdim,
                           double                              hthick,
                           const ChVector<>&                   pos = ChVector<>(0,0,0),
                           const ChQuaternion<>&               rot = ChQuaternion<>(1,0,0,0),
                           bool                                collide = true,
                           bool                                y_up = false);

CH_UTILS_API
void CreateBoxContainerDVI(ChSystem*                           system,
                           int                                 id,
                           ChSharedPtr<ChMaterialSurface>&     mat,
                           const ChVector<>&                   hdim,
                           double                              hthick,
                           const ChVector<>&                   pos = ChVector<>(0,0,0),
                           const ChQuaternion<>&               rot = ChQuaternion<>(1,0,0,0),
                           bool                                collide = true,
                           bool                                y_up = false);

CH_UTILS_API
void InitializeObject(ChSharedBodyPtr                     body,
                      double                              mass,
                      ChSharedPtr<ChMaterialSurface>&     mat,
                      const ChVector<>&                   pos = ChVector<>(0,0,0),
                      const ChQuaternion<>&               rot = ChQuaternion<>(1,0,0,0),
                      bool                                collide = true,
                      bool                                fixed = false,
                      int                                 collision_family = 2,
                      int                                 do_not_collide_with = 4);
CH_UTILS_API
void FinalizeObject(ChSharedBodyPtr                     body,
                    ChSystem*                           system);



} // end namespace utils
} // end namespace chrono


#endif
