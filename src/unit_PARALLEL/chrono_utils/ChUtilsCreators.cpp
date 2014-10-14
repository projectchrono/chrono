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
// =============================================================================

#include "chrono_utils/ChUtilsCreators.h"

namespace chrono {
namespace utils {


// -----------------------------------------------------------------------------

void AddSphereGeometry(ChBody*               body,
                       double                radius,
                       const ChVector<>&     pos,
                       const ChQuaternion<>& rot)
{
  body->GetCollisionModel()->AddSphere(radius, pos);

  ChSharedPtr<ChSphereShape> sphere(new ChSphereShape);
  sphere->GetSphereGeometry().rad = radius;
  sphere->Pos = pos;
  sphere->Rot = rot;

  body->GetAssets().push_back(sphere);
}

// -----------------------------------------------------------------------------

void AddEllipsoidGeometry(ChBody*               body,
                          const ChVector<>&     size,
                          const ChVector<>&     pos,
                          const ChQuaternion<>& rot)
{
  body->GetCollisionModel()->AddEllipsoid(size.x, size.y, size.z, pos, rot);

  ChSharedPtr<ChEllipsoidShape> ellipsoid(new ChEllipsoidShape);
  ellipsoid->GetEllipsoidGeometry().rad = size;
  ellipsoid->Pos = pos;
  ellipsoid->Rot = rot;

  body->GetAssets().push_back(ellipsoid);
}

// -----------------------------------------------------------------------------

void AddBoxGeometry(ChBody*               body,
                    const ChVector<>&     size,
                    const ChVector<>&     pos,
                    const ChQuaternion<>& rot)
{
  body->GetCollisionModel()->AddBox(size.x, size.y, size.z, pos, rot);

  ChSharedPtr<ChBoxShape> box(new ChBoxShape);
  box->GetBoxGeometry().Size = size;
  box->Pos = pos;
  box->Rot = rot;

  body->GetAssets().push_back(box);
}

// -----------------------------------------------------------------------------

void AddCapsuleGeometry(ChBody*               body,
                        double                radius,
                        double                hlen,
                        const ChVector<>&     pos,
                        const ChQuaternion<>& rot)
{
  body->GetCollisionModel()->AddCapsule(radius, hlen, pos, rot);

  ChSharedPtr<ChCapsuleShape> capsule(new ChCapsuleShape);
  capsule->GetCapsuleGeometry().rad = radius;
  capsule->GetCapsuleGeometry().hlen = hlen;
  capsule->Pos = pos;
  capsule->Rot = rot;

  body->GetAssets().push_back(capsule);
}

// -----------------------------------------------------------------------------

void AddCylinderGeometry(ChBody*               body,
                         double                radius,
                         double                hlen,
                         const ChVector<>&     pos,
                         const ChQuaternion<>& rot)
{
  body->GetCollisionModel()->AddCylinder(radius, radius, hlen, pos, rot);

  ChSharedPtr<ChCylinderShape> cylinder(new ChCylinderShape);
  cylinder->GetCylinderGeometry().rad = radius;
  cylinder->GetCylinderGeometry().p1 = ChVector<>(0, hlen, 0);
  cylinder->GetCylinderGeometry().p2 = ChVector<>(0, -hlen, 0);
  cylinder->Pos = pos;
  cylinder->Rot = rot;

  body->GetAssets().push_back(cylinder);
}

// -----------------------------------------------------------------------------

void AddConeGeometry(ChBody*               body,
                     double                radius,
                     double                height,
                     const ChVector<>&     pos,
                     const ChQuaternion<>& rot)
{
  body->GetCollisionModel()->AddCone(radius, radius, height, pos, rot);

  ChSharedPtr<ChConeShape> cone(new ChConeShape);
  cone->GetConeGeometry().rad = ChVector<>(radius, height, radius);
  cone->Pos = pos;
  cone->Rot = rot;

  body->GetAssets().push_back(cone);
}

// -----------------------------------------------------------------------------

void AddTriangleMeshGeometry(ChBody*               body,
                             const std::string&    obj_filename,
                             const std::string&    name,
                             const ChVector<>&     pos,
                             const ChQuaternion<>& rot)
{
  geometry::ChTriangleMeshConnected trimesh;
  trimesh.LoadWavefrontMesh(obj_filename, false, false);

  for (int i = 0; i < trimesh.m_vertices.size(); i++)
    trimesh.m_vertices[i] = pos + rot.Rotate(trimesh.m_vertices[i]);

  body->GetCollisionModel()->AddTriangleMesh(trimesh, false, false);

  ChSharedPtr<ChTriangleMeshShape> trimesh_shape(new ChTriangleMeshShape);
  trimesh_shape->SetMesh(trimesh);
  trimesh_shape->SetName(name);
  trimesh_shape->Pos = ChVector<>(0, 0, 0);
  trimesh_shape->Rot = ChQuaternion<>(1, 0, 0, 0);

  body->GetAssets().push_back(trimesh_shape);
}

// -----------------------------------------------------------------------------

void AddRoundedBoxGeometry(ChBody*               body,
                           const ChVector<>&     size,
                           double                srad,
                           const ChVector<>&     pos,
                           const ChQuaternion<>& rot)
{
  body->GetCollisionModel()->AddRoundedBox(size.x, size.y, size.z, srad, pos, rot);

  ChSharedPtr<ChRoundedBoxShape> box(new ChRoundedBoxShape);
  box->GetRoundedBoxGeometry().Size = size;
  box->GetRoundedBoxGeometry().radsphere = srad;
  box->Pos = pos;
  box->Rot = rot;
  body->GetAssets().push_back(box);
}

// -----------------------------------------------------------------------------

void AddRoundedCylinderGeometry(ChBody*               body,
                                double                radius,
                                double                hlen,
                                double                srad,
                                const ChVector<>&     pos,
                                const ChQuaternion<>& rot)
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

// -----------------------------------------------------------------------------

void AddTorusGeometry(ChBody*               body,
                      double                radius,
                      double                thickness,
                      int                   segments,
                      int                   angle,
                      const ChVector<>&     pos,
                      const ChQuaternion<>& rot)
{
  for (int i = 0; i < angle; i += angle / segments) {
    double angle = i * CH_C_PI / 180.0;
    double x = cos(angle) * radius;
    double z = sin(angle) * radius;
    Quaternion q = chrono::Q_from_AngAxis(-angle, VECT_Y) % chrono::Q_from_AngAxis(CH_C_PI / 2.0, VECT_X);
    double outer_circ = 2 * CH_C_PI*(radius + thickness);

    AddCylinderGeometry(body, thickness, outer_circ / segments*.5, ChVector<>(x, 0, z) + pos, q);
  }
}

// -----------------------------------------------------------------------------
// CreateBoxContainerDEM
// CreateBoxContainerDVI
//
// Create a fixed body with contact and asset geometry representing a box with 5
// walls (no top).
// -----------------------------------------------------------------------------
void AddWall(ChBody*              body,
             const ChVector<>&    loc,
             const ChVector<>&    hdim)
{
  // Append to collision geometry
  body->GetCollisionModel()->AddBox(hdim.x, hdim.y, hdim.z, loc);

  // Append to assets
  ChSharedPtr<ChBoxShape> box_shape(new ChBoxShape);
  box_shape->Pos = loc;
  box_shape->Rot = ChQuaternion<>(1,0,0,0);
  box_shape->GetBoxGeometry().Size = hdim;

  body->GetAssets().push_back(box_shape);
}

void CreateBoxContainerDEM(ChSystem*                           system,
                           int                                 id,
                           ChSharedPtr<ChMaterialSurfaceDEM>&  mat,
                           const ChVector<>&                   hdim,
                           double                              hthick,
                           const ChVector<>&                   pos,
                           const ChQuaternion<>&               rot,
                           bool                                collide,
                           bool                                y_up)
{
  // Infer system type and collision type.
  SystemType sysType = GetSystemType(system);
  CollisionType collType = GetCollisionType(system);
  assert(sysType == SEQUENTIAL_DEM || sysType == PARALLEL_DEM);

  // Create the body and set material
  ChBodyDEM* body;

  if (sysType == SEQUENTIAL_DEM || collType == BULLET_CD)
    body = new ChBodyDEM();
  else
    body = new ChBodyDEM(new collision::ChCollisionModelParallel);

  body->SetMaterialSurfaceDEM(mat);

  // Set body properties and geometry.
  body->SetIdentifier(id);
  body->SetMass(1);
  body->SetPos(pos);
  body->SetRot(rot);
  body->SetCollide(collide);
  body->SetBodyFixed(true);

  body->GetCollisionModel()->ClearModel();
  if(y_up){
     AddBoxGeometry(body,  ChVector<>(hdim.x,  hthick,hdim.y),ChVector<>(0             , -hthick,0             ));
     AddBoxGeometry(body,  ChVector<>(hthick,  hdim.z,hdim.y),ChVector<>(-hdim.x-hthick, hdim.z ,0             ));
     AddBoxGeometry(body,  ChVector<>(hthick,  hdim.z,hdim.y),ChVector<>( hdim.x+hthick, hdim.z ,0             ));
     AddBoxGeometry(body,  ChVector<>(hdim.x,  hdim.z,hthick),ChVector<>(0             , hdim.z ,-hdim.y-hthick));
     AddBoxGeometry(body,  ChVector<>(hdim.x,  hdim.z,hthick),ChVector<>(0             , hdim.z , hdim.y+hthick));
  }else{
     AddBoxGeometry(body,  ChVector<>(hdim.x, hdim.y, hthick),ChVector<>(0             , 0             , -hthick));
     AddBoxGeometry(body,  ChVector<>(hthick, hdim.y, hdim.z),ChVector<>(-hdim.x-hthick, 0             , hdim.z));
     AddBoxGeometry(body,  ChVector<>(hthick, hdim.y, hdim.z),ChVector<>( hdim.x+hthick, 0             , hdim.z));
     AddBoxGeometry(body,  ChVector<>(hdim.x, hthick, hdim.z),ChVector<>(0             , -hdim.y-hthick, hdim.z));
     AddBoxGeometry(body,  ChVector<>(hdim.x, hthick, hdim.z),ChVector<>(0             ,  hdim.y+hthick, hdim.z));
  }
  body->GetCollisionModel()->BuildModel();

  // Attach the body to the system.
  system->AddBody(ChSharedPtr<ChBodyDEM>(body));
}

void CreateBoxContainerDVI(ChSystem*                        system,
                           int                              id,
                           ChSharedPtr<ChMaterialSurface>&  mat,
                           const ChVector<>&                hdim,
                           double                           hthick,
                           const ChVector<>&                pos,
                           const ChQuaternion<>&            rot,
                           bool                             collide,
                           bool                             y_up)
{
  // Infer system type and collision type.
  SystemType sysType = GetSystemType(system);
  CollisionType cdType = GetCollisionType(system);
  assert(sysType == SEQUENTIAL_DVI || sysType == PARALLEL_DVI);

  // Create the body and set material
  ChBody* body;

  if (sysType == SEQUENTIAL_DVI|| cdType==BULLET_CD)
    body = new ChBody();
  else
    body = new ChBody(new collision::ChCollisionModelParallel);

  body->SetMaterialSurface(mat);

  // Set body properties and geometry.
  body->SetIdentifier(id);
  body->SetMass(1);
  body->SetPos(pos);
  body->SetRot(rot);
  body->SetCollide(collide);
  body->SetBodyFixed(true);

  body->GetCollisionModel()->ClearModel();
if(y_up){
   AddBoxGeometry(body,  ChVector<>(hdim.x,  hthick,hdim.y),ChVector<>(0             , -hthick,0             ));
   AddBoxGeometry(body,  ChVector<>(hthick,  hdim.z,hdim.y),ChVector<>(-hdim.x-hthick, hdim.z ,0             ));
   AddBoxGeometry(body,  ChVector<>(hthick,  hdim.z,hdim.y),ChVector<>( hdim.x+hthick, hdim.z ,0             ));
   AddBoxGeometry(body,  ChVector<>(hdim.x,  hdim.z,hthick),ChVector<>(0             , hdim.z ,-hdim.y-hthick));
   AddBoxGeometry(body,  ChVector<>(hdim.x,  hdim.z,hthick),ChVector<>(0             , hdim.z , hdim.y+hthick));
}else{
   AddBoxGeometry(body,  ChVector<>(hdim.x, hdim.y, hthick),ChVector<>(0             , 0             , -hthick));
   AddBoxGeometry(body,  ChVector<>(hthick, hdim.y, hdim.z),ChVector<>(-hdim.x-hthick, 0             , hdim.z));
   AddBoxGeometry(body,  ChVector<>(hthick, hdim.y, hdim.z),ChVector<>( hdim.x+hthick, 0             , hdim.z));
   AddBoxGeometry(body,  ChVector<>(hdim.x, hthick, hdim.z),ChVector<>(0             , -hdim.y-hthick, hdim.z));
   AddBoxGeometry(body,  ChVector<>(hdim.x, hthick, hdim.z),ChVector<>(0             ,  hdim.y+hthick, hdim.z));
}
  body->GetCollisionModel()->BuildModel();

  // Attach the body to the system.
  system->AddBody(ChSharedPtr<ChBody>(body));
}


// -----------------------------------------------------------------------------
void InitializeObject(ChSharedBodyPtr                  body,
                      double                           mass,
                      ChSharedPtr<ChMaterialSurface>&  mat,
                      const ChVector<>&                pos,
                      const ChQuaternion<>&            rot,
                      bool                             collide,
                      bool                             fixed,
                      int                              collision_family,
                      int                              do_not_collide_with)
{
  body->SetMass(mass);
  body->SetPos(pos);
  body->SetRot(rot);
  body->SetCollide(collide);
  body->SetBodyFixed(fixed);
  body->SetMaterialSurface(mat);
  body->GetCollisionModel()->ClearModel();
  body->GetCollisionModel()->SetFamily(collision_family);
  body->GetCollisionModel()->SetFamilyMaskNoCollisionWithFamily(do_not_collide_with);
}

void FinalizeObject(ChSharedBodyPtr  body,
                    ChSystem*        system)
{
  body->GetCollisionModel()->BuildModel();
  system->AddBody(body);
}


}  // namespace utils
}  // namespace chrono
