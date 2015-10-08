/*
 * VehicleExtraProperties.cpp
 *
 *  Created on: Apr 25, 2015
 *      Author: arman
 */

#include "chrono_fsi/VehicleExtraProperties.h"

// Arman : maybe too many includes
#include "core/ChStream.h"

//#include "chrono_parallel/lcp/ChLcpSystemDescriptorParallel.h"
//#include "chrono_parallel/collision/ChCNarrowphaseRUtils.h"
#include "chrono_vehicle/ChVehicleModelData.h"

int chassisFam = 2;
int tireFam = 3;

//#include "hmmwvParams.h"

using namespace chrono;
using namespace collision;

// Tire Coefficient of friction
float mu_t = 0.8;

// Callback class for providing driver inputs.
MyDriverInputs::MyDriverInputs(double delay) : m_delay(delay) {
}
void MyDriverInputs::onCallback(double time, double& throttle, double& steering, double& braking) {
  throttle = 0;
  steering = 0;
  braking = 0;

  double eff_time = time - m_delay;

  // Do not generate any driver inputs for a duration equal to m_delay.
  if (eff_time < 0)
    return;

  if (eff_time > 0.2)
    throttle = 1.0;
  else if (eff_time > 0.1)
    throttle = 10 * (eff_time - 0.1);
}

// Callback class for specifying rigid tire contact model.
// This version uses cylindrical contact shapes.
void MyCylindricalTire::onCallback(ChSharedPtr<ChBody> wheelBody, double radius, double width) {
  wheelBody->ChangeCollisionModel(new collision::ChCollisionModelParallel);

  wheelBody->GetCollisionModel()->ClearModel();
  wheelBody->GetCollisionModel()->AddCylinder(0.46, 0.46, width / 2);
  wheelBody->GetCollisionModel()->BuildModel();

  wheelBody->GetMaterialSurface()->SetFriction(mu_t);
}

// Callback class for specifying rigid tire contact model.
// This version uses a collection of convex contact shapes (meshes).
MyLuggedTire::MyLuggedTire() {
  std::string lugged_file("hmmwv/lugged_wheel_section.obj");
  geometry::ChTriangleMeshConnected lugged_mesh;
  utils::LoadConvexMesh(vehicle::GetDataFile(lugged_file), lugged_mesh, lugged_convex);
  num_hulls = lugged_convex.GetHullCount();
}
void MyLuggedTire::onCallback(ChSharedPtr<ChBody> wheelBody, double radius, double width) {
  wheelBody->ChangeCollisionModel(new collision::ChCollisionModelParallel);

  ChCollisionModelParallel* coll_model = (ChCollisionModelParallel*)wheelBody->GetCollisionModel();
  coll_model->ClearModel();

  // Assemble the tire contact from 15 segments, properly offset.
  // Each segment is further decomposed in convex hulls.
  for (int iseg = 0; iseg < 15; iseg++) {
    ChQuaternion<> rot = Q_from_AngAxis(iseg * 24 * CH_C_DEG_TO_RAD, VECT_Y);
    for (int ihull = 0; ihull < num_hulls; ihull++) {
      std::vector<ChVector<> > convexhull;
      lugged_convex.GetConvexHullResult(ihull, convexhull);
      coll_model->AddConvexHull(convexhull, VNULL, rot);
    }
  }

  // Add a cylinder to represent the wheel hub.
  coll_model->AddCylinder(0.223, 0.223, 0.126);

  coll_model->BuildModel();

  wheelBody->GetMaterialSurface()->SetFriction(mu_t);
}

// Callback class for specifying chassis contact model.
// This version uses a box representing the chassis.
// In addition, this version overrides the visualization assets of the provided
// chassis body with the collision meshes.
void MyChassisBoxModel_vis::onCallback(ChSharedPtr<ChBodyAuxRef> chassisBody) {
  // Clear any existing assets (will be overriden)

  chassisBody->ChangeCollisionModel(new collision::ChCollisionModelParallel);

  chassisBody->GetCollisionModel()->ClearModel();
  ChVector<> chLoc = ChVector<>(0, 0, 0);
  //    ChVector<> chLoc = chassisBody->GetFrame_REF_to_COG().GetPos();
  chassisBody->GetCollisionModel()->AddBox(size.x, size.y, size.z, chLoc, rot);
  //    utils::AddBoxGeometry(
  //        chassisBody.get_ptr(), ChVector<>(1, .5, .05), ChVector<>(0, 0, 0), ChQuaternion<>(1, 0, 0, 0), true);
  chassisBody->GetCollisionModel()->SetFamily(chassisFam);
  chassisBody->GetCollisionModel()->SetFamilyMaskNoCollisionWithFamily(tireFam);
  chassisBody->GetCollisionModel()->BuildModel();
  chassisBody->GetMaterialSurface()->SetFriction(mu_t);

  chassisBody->GetAssets().clear();
  ChSharedPtr<ChBoxShape> box(new ChBoxShape);
  box->GetBoxGeometry().Size = size;
  box->Rot = rot;
  chassisBody->GetAssets().push_back(box);
}

void MyChassisBoxModel_vis::SetAttributes(const ChVector<>& otherSize,
                                          const ChQuaternion<>& otherRot,
                                          const ChVector<>& otherLoc) {
  size = otherSize;
  rot = otherRot;
  loc = otherLoc;
}

// Callback class for specifying chassis contact model.
// This version uses a sphere representing the chassis.
// In addition, this version overrides the visualization assets of the provided
// chassis body with the collision meshes.
void MyChassisSphereModel_vis::onCallback(ChSharedPtr<ChBodyAuxRef> chassisBody) {
  // Clear any existing assets (will be overriden)

  chassisBody->ChangeCollisionModel(new collision::ChCollisionModelParallel);

  chassisBody->GetCollisionModel()->ClearModel();
  ChVector<> chLoc = ChVector<>(0, 0, 0);
  //        ChVector<> chLoc = chassisBody->GetFrame_REF_to_COG().GetPos();
  chassisBody->GetCollisionModel()->AddSphere(rad, chLoc);
  //    utils::AddBoxGeometry(
  //        chassisBody.get_ptr(), ChVector<>(1, .5, .05), ChVector<>(0, 0, 0), ChQuaternion<>(1, 0, 0, 0), true);
  chassisBody->GetCollisionModel()->SetFamily(chassisFam);
  chassisBody->GetCollisionModel()->SetFamilyMaskNoCollisionWithFamily(tireFam);
  chassisBody->GetCollisionModel()->BuildModel();
  chassisBody->GetMaterialSurface()->SetFriction(mu_t);

  chassisBody->GetAssets().clear();
  ChSharedPtr<ChSphereShape> sphere(new ChSphereShape);
  sphere->GetSphereGeometry().rad = rad;
  chassisBody->GetAssets().push_back(sphere);
}

void MyChassisSphereModel_vis::SetAttributes(double otherRad,
                                             const ChQuaternion<>& otherRot,
                                             const ChVector<>& otherLoc) {
  rad = otherRad;
  rot = otherRot;
  loc = otherLoc;
}

// Callback class for specifying chassis contact model.
// This version uses a convex decomposition of an obj representing the chassis.
MyChassisSimpleConvexMesh::MyChassisSimpleConvexMesh()
    : pos(chrono::ChVector<>(0, 0, 0)), rot(chrono::ChQuaternion<>(1, 0, 0, 0)) {
  //    std::string chassis_obj_file("hmmwv/lugged_wheel_section.obj");
  //    std::string chassis_obj_file("hmmwv/lugged_wheel.obj");
  //    std::string chassis_obj_file("hmmwv/myHumvee.obj");
  //  chassis_obj_file = std::string("hmmwv/myHumvee1.obj");
  chassis_obj_file = std::string("hmmwv/hmmwv_chassis_simple.obj");

  utils::LoadConvexMesh(vehicle::GetDataFile(chassis_obj_file), chassis_mesh, chassis_convex);
}

void MyChassisSimpleConvexMesh::onCallback(ChSharedPtr<ChBodyAuxRef> chassisBody) {
  chassisBody->ChangeCollisionModel(new collision::ChCollisionModelParallel);

  ChVector<> chLoc = ChVector<>(0, 0, 0);  // chassisBody->GetFrame_REF_to_COG().GetPos();
  chassisBody->GetCollisionModel()->ClearModel();
  //    utils::AddConvexCollisionModel(chassisBody, chassis_mesh, chassis_convex, chLoc, ChQuaternion<>(1, 0, 0, 0),
  //    false);

  // **************************
  ChSharedPtr<ChBody> body = chassisBody;
  geometry::ChTriangleMeshConnected& convex_mesh = chassis_mesh;
  ChConvexDecompositionHACDv2& convex_shape = chassis_convex;
  ChConvexDecomposition* used_decomposition = &convex_shape;

  //*****
  int hull_count = used_decomposition->GetHullCount();

  for (int c = 0; c < hull_count; c++) {
    std::vector<ChVector<double> > convexhull;
    used_decomposition->GetConvexHullResult(c, convexhull);

    ((collision::ChCollisionModelParallel*)body->GetCollisionModel())->AddConvexHull(convexhull, chLoc, rot);
  }
  chassisBody->GetCollisionModel()->SetFamily(chassisFam);
  //    printf("chassis family %d \n", chassisBody->GetCollisionModel()->GetFamily());
  chassisBody->GetCollisionModel()->SetFamilyMaskNoCollisionWithFamily(tireFam);
  chassisBody->GetCollisionModel()->BuildModel();

  // **************************

  //        chassisBody->RecomputeCollisionModel();
  //        chassisBody->SetCollide(false);

  chassisBody->GetMaterialSurface()->SetFriction(mu_t);
}

// Callback class for specifying chassis contact model.
// This version uses a triangular given in an obj representing the chassis.
// In addition, this version overrides the visualization assets of the provided
// chassis body with the collision meshes.
MyChassisSimpleTriMesh_vis::MyChassisSimpleTriMesh_vis()
    : pos(chrono::ChVector<>(0, 0, 0)), rot(chrono::ChQuaternion<>(1, 0, 0, 0)) {
  //	chassis_obj_file = std::string("hmmwv/myHumvee1.obj");
  chassis_obj_file = std::string("hmmwv/hmmwv_chassis_simple.obj");
}

void MyChassisSimpleTriMesh_vis::onCallback(ChSharedPtr<ChBodyAuxRef> chassisBody) {
  chassisBody->ChangeCollisionModel(new collision::ChCollisionModelParallel);

  // Clear any existing assets (will be overriden)
  const std::string mesh_name("chassis");

  chassisBody->GetAssets().clear();
  //    ChVector<> chLoc = ChVector<>(0,0,0);//chassisBody->GetFrame_REF_to_COG().GetPos();

  chassisBody->GetCollisionModel()->ClearModel();
  //    utils::AddTriangleMeshGeometry(chassisBody.get_ptr(), vehicle::GetDataFile(chassis_obj_file), mesh_name, pos,
  //    rot, true);

  ChVector<> chLoc = ChVector<>(0, 0, 0);  // chassisBody->GetFrame_REF_to_COG().GetPos();
                                           //    ChVector<> chLoc = chassisBody->GetFrame_REF_to_COG().GetPos();

  // *** here
  std::string obj_filename = vehicle::GetDataFile(chassis_obj_file);
  const std::string& name = mesh_name;
  ChBody* body = chassisBody.get_ptr();
  geometry::ChTriangleMeshConnected trimesh;
  trimesh.LoadWavefrontMesh(obj_filename, false, false);

  for (int i = 0; i < trimesh.m_vertices.size(); i++)
    trimesh.m_vertices[i] = pos + rot.Rotate(trimesh.m_vertices[i]);

  body->GetCollisionModel()->AddTriangleMesh(trimesh, false, false, chLoc);

  if (true) {
    ChSharedPtr<ChTriangleMeshShape> trimesh_shape(new ChTriangleMeshShape);
    trimesh_shape->SetMesh(trimesh);
    trimesh_shape->SetName(name);
    trimesh_shape->Pos = ChVector<>(0, 0, 0);
    trimesh_shape->Rot = ChQuaternion<>(1, 0, 0, 0);
    body->GetAssets().push_back(trimesh_shape);
  }
  // *** to here
  chassisBody->GetCollisionModel()->SetFamily(chassisFam);
  chassisBody->GetCollisionModel()->SetFamilyMaskNoCollisionWithFamily(tireFam);
  chassisBody->GetCollisionModel()->BuildModel();

  chassisBody->GetMaterialSurface()->SetFriction(mu_t);
}
