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
  // Infer system type.
  SystemType sysType = GetSystemType(system);
  assert(sysType == SEQUENTIAL_DEM || sysType == PARALLEL_DEM);

  // Create the body and set material
  ChBodyDEM* body;

  if (sysType == SEQUENTIAL_DEM)
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

void CreateBoxContainerDVI(ChSystem*                           system,
                           int                                 id,
                           ChSharedPtr<ChMaterialSurface>&     mat,
                           const ChVector<>&                   hdim,
                           double                              hthick,
                           const ChVector<>&                   pos,
                           const ChQuaternion<>&               rot,
                           bool                                collide,
                           bool                                y_up)
{
  // Infer system type.
  SystemType sysType = GetSystemType(system);
  CollisionType cdType = GetCollisionType(system);
  assert(sysType == SEQUENTIAL_DVI || sysType == PARALLEL_DVI);

  // Create the body and set material
  ChBody* body;

  if (sysType == SEQUENTIAL_DVI|| cdType==BULLET_CD){
    body = new ChBody();
  }else{
    body = new ChBody(new collision::ChCollisionModelParallel);
  }
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


void InitializeObject(     ChSharedBodyPtr                     body,
                           double                              mass,
                           ChSharedPtr<ChMaterialSurface>&     mat,
                           const ChVector<>&                   pos,
                           const ChQuaternion<>&               rot,
                           bool                                collide,
                           bool                                fixed,
                           int                                 collision_family,
                           int                                 do_not_collide_with){
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

void FinalizeObject(ChSharedBodyPtr                         body,
                    ChSystem*                               system){
  body->GetCollisionModel()->BuildModel();
  system->AddBody(body);
}


}  // namespace utils
}  // namespace chrono
