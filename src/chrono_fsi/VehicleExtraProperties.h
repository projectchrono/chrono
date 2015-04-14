/*
 * VehicleProperties.h
 *
 *  Created on: Apr 1, 2015
 *      Author: Arman Pazouki
 */

#ifndef VEHICLEEXTRAPROPERTIES_H_
#define VEHICLEEXTRAPROPERTIES_H_

#include "chrono_parallel/physics/ChSystemParallel.h"

// Arman : maybe too many includes
#include "core/ChFileutils.h"
#include "core/ChStream.h"

#include "chrono_parallel/physics/ChSystemParallel.h"
#include "chrono_parallel/lcp/ChLcpSystemDescriptorParallel.h"
#include "chrono_parallel/collision/ChCNarrowphaseRUtils.h"

#include "chrono_utils/ChUtilsVehicle.h"
#include "chrono_utils/ChUtilsGeometry.h"
#include "chrono_utils/ChUtilsCreators.h"
#include "chrono_utils/ChUtilsGenerators.h"
#include "chrono_utils/ChUtilsInputOutput.h"

//#include "hmmwvParams.h"

using namespace chrono;
using namespace chrono::collision;

// Tire Coefficient of friction
float mu_t = 0.8;

// Callback class for providing driver inputs.
class MyDriverInputs : public utils::DriverInputsCallback {
 public:
  MyDriverInputs(double delay) : m_delay(delay) {}

  virtual void onCallback(double time, double& throttle, double& steering, double& braking) {
    throttle = 0;
    steering = 0;
    braking = 0;

    double eff_time = time - m_delay;

    // Do not generate any driver inputs for a duration equal to m_delay.
    if (eff_time < 0)
      return;

    if (eff_time > 0.5)
      throttle = 1.0;
    else if (eff_time > 0.25)
      throttle = 4 * (eff_time - 0.25);
  }

 private:
  double m_delay;
};

// Callback class for specifying rigid tire contact model.
// This version uses cylindrical contact shapes.
class MyCylindricalTire : public utils::TireContactCallback {
 public:
  virtual void onCallback(ChSharedPtr<ChBody> wheelBody, double radius, double width) {
    wheelBody->GetCollisionModel()->ClearModel();
    wheelBody->GetCollisionModel()->AddCylinder(0.46, 0.46, width / 2);
    wheelBody->GetCollisionModel()->BuildModel();

    wheelBody->GetMaterialSurface()->SetFriction(mu_t);
  }
};

// Callback class for specifying rigid tire contact model.
// This version uses a collection of convex contact shapes (meshes).
class MyLuggedTire : public utils::TireContactCallback {
 public:
  MyLuggedTire() {
    std::string lugged_file("hmmwv/lugged_wheel_section.obj");
    geometry::ChTriangleMeshConnected lugged_mesh;
    utils::LoadConvexMesh(vehicle::GetDataFile(lugged_file), lugged_mesh, lugged_convex);
    num_hulls = lugged_convex.GetHullCount();
  }

  virtual void onCallback(ChSharedPtr<ChBody> wheelBody, double radius, double width) {
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

 private:
  ChConvexDecompositionHACDv2 lugged_convex;
  int num_hulls;
};

// Callback class for specifying rigid tire contact model.
// This version uses a collection of convex contact shapes (meshes).
// In addition, this version overrides the visualization assets of the provided
// wheel body with the collision meshes.
class MyLuggedTire_vis : public utils::TireContactCallback {
 public:
  MyLuggedTire_vis() {
    std::string lugged_file("hmmwv/lugged_wheel_section.obj");
    utils::LoadConvexMesh(vehicle::GetDataFile(lugged_file), lugged_mesh, lugged_convex);
  }

  virtual void onCallback(ChSharedPtr<ChBody> wheelBody, double radius, double width) {
    // Clear any existing assets (will be overriden)
    wheelBody->GetAssets().clear();

    wheelBody->GetCollisionModel()->ClearModel();
    for (int j = 0; j < 15; j++) {
      utils::AddConvexCollisionModel(
          wheelBody, lugged_mesh, lugged_convex, VNULL, Q_from_AngAxis(j * 24 * CH_C_DEG_TO_RAD, VECT_Y), false);
    }
    // This cylinder acts like the rims
    utils::AddCylinderGeometry(wheelBody.get_ptr(), 0.223, 0.126);
    wheelBody->GetCollisionModel()->BuildModel();

    wheelBody->GetMaterialSurface()->SetFriction(mu_t);
  }

 private:
  ChConvexDecompositionHACDv2 lugged_convex;
  geometry::ChTriangleMeshConnected lugged_mesh;
};

// Callback class for specifying chassis contact model.
// This version uses a box representing the chassis.
// In addition, this version overrides the visualization assets of the provided
// chassis body with the collision meshes.
class MyChassisBoxModel_vis : public utils::ChassisContactCallback {
 public:
  virtual void onCallback(ChSharedPtr<ChBodyAuxRef> chassisBody) {
    // Clear any existing assets (will be overriden)

    chassisBody->GetCollisionModel()->ClearModel();
    ChVector<> chLoc = ChVector<>(0,0,0);
//    ChVector<> chLoc = chassisBody->GetFrame_REF_to_COG().GetPos();
    chassisBody->GetCollisionModel()->AddBox(size.x, size.y, size.z, chLoc);
    //    utils::AddBoxGeometry(
    //        chassisBody.get_ptr(), ChVector<>(1, .5, .05), ChVector<>(0, 0, 0), ChQuaternion<>(1, 0, 0, 0), true);
    chassisBody->GetCollisionModel()->BuildModel();
    chassisBody->GetMaterialSurface()->SetFriction(mu_t);

    chassisBody->GetAssets().clear();
    ChSharedPtr<ChBoxShape> box(new ChBoxShape);
    box->GetBoxGeometry().Size = size;
    chassisBody->GetAssets().push_back(box);
  }

  virtual void SetAttributes(const ChVector<>& otherSize,
                             const ChQuaternion<>& otherRot = ChQuaternion<>(1, 0, 0, 0),
                             const ChVector<>& otherLoc = ChVector<>(0, 0, 0)) {
    size = otherSize;
    rot = otherRot;
    loc = otherLoc;
  }

 private:
  ChConvexDecompositionHACDv2 chassis_convex;
  geometry::ChTriangleMeshConnected chassis_mesh;

  ChVector<> size;
  ChQuaternion<> rot;
  ChVector<> loc;
};

// Callback class for specifying chassis contact model.
// This version uses a sphere representing the chassis.
// In addition, this version overrides the visualization assets of the provided
// chassis body with the collision meshes.
class MyChassisSphereModel_vis : public utils::ChassisContactCallback {
 public:
  virtual void onCallback(ChSharedPtr<ChBodyAuxRef> chassisBody) {
    // Clear any existing assets (will be overriden)

    chassisBody->GetCollisionModel()->ClearModel();
    ChVector<> chLoc = ChVector<>(0,0,0);
//    ChVector<> chLoc = chassisBody->GetFrame_REF_to_COG().GetPos();
    chassisBody->GetCollisionModel()->AddSphere(rad, chLoc);
    //    utils::AddBoxGeometry(
    //        chassisBody.get_ptr(), ChVector<>(1, .5, .05), ChVector<>(0, 0, 0), ChQuaternion<>(1, 0, 0, 0), true);
    chassisBody->GetCollisionModel()->BuildModel();
    chassisBody->GetMaterialSurface()->SetFriction(mu_t);

    chassisBody->GetAssets().clear();
    ChSharedPtr<ChSphereShape> sphere(new ChSphereShape);
    sphere->GetSphereGeometry().rad = rad;
    chassisBody->GetAssets().push_back(sphere);
  }

  virtual void SetAttributes(double otherRad,
                             const ChQuaternion<>& otherRot = ChQuaternion<>(1, 0, 0, 0),
                             const ChVector<>& otherLoc = ChVector<>(0, 0, 0)) {
    rad = otherRad;
    rot = otherRot;
    loc = otherLoc;
  }

 private:
  ChConvexDecompositionHACDv2 chassis_convex;
  geometry::ChTriangleMeshConnected chassis_mesh;

  double rad;
  ChQuaternion<> rot;
  ChVector<> loc;
};

// Callback class for specifying chassis contact model.
// This version uses a box representing the chassis.
// In addition, this version overrides the visualization assets of the provided
// chassis body with the collision meshes.
class MyChassisSimpleMesh_vis : public utils::ChassisContactCallback {
 public:
  MyChassisSimpleMesh_vis() {
//    std::string chassis_file("hmmwv/lugged_wheel_section.obj");
//    std::string chassis_file("hmmwv/lugged_wheel.obj");
    std::string chassis_file("hmmwv/myHumvee.obj");

    utils::LoadConvexMesh(vehicle::GetDataFile(chassis_file), chassis_mesh, chassis_convex);
  }

  virtual void onCallback(ChSharedPtr<ChBodyAuxRef> chassisBody) {
    // Clear any existing assets (will be overriden)
    chassisBody->GetAssets().clear();
    ChVector<> chLoc = ChVector<>(0,0,0);//chassisBody->GetFrame_REF_to_COG().GetPos();
    chassisBody->GetCollisionModel()->ClearModel();
    //    utils::AddConvexCollisionModel(chassisBody, chassis_mesh, chassis_convex, chLoc, ChQuaternion<>(1, 0, 0, 0),
    //    false);

    // **************************
    ChSharedPtr<ChBody> body = chassisBody;
    geometry::ChTriangleMeshConnected& convex_mesh = chassis_mesh;
    ChConvexDecompositionHACDv2& convex_shape = chassis_convex;
    ChConvexDecomposition* used_decomposition = &convex_shape;
    bool use_original_asset = false;

    //*****
    int hull_count = used_decomposition->GetHullCount();

    for (int c = 0; c < hull_count; c++) {
      std::vector<ChVector<double> > convexhull;
      used_decomposition->GetConvexHullResult(c, convexhull);

      ((collision::ChCollisionModelParallel*)body->GetCollisionModel())->AddConvexHull(convexhull, chLoc, rot);
      // Add each convex chunk as a new asset
      if (!use_original_asset) {
        std::stringstream ss;
        ss << convex_mesh.GetFileName() << "_" << c;
        geometry::ChTriangleMeshConnected trimesh_convex;
        used_decomposition->GetConvexHullResult(c, trimesh_convex);

        ChSharedPtr<ChTriangleMeshShape> trimesh_shape(new ChTriangleMeshShape);
        trimesh_shape->SetMesh(trimesh_convex);
        trimesh_shape->SetName(ss.str());
        trimesh_shape->Pos = pos;
        trimesh_shape->Rot = rot;
        body->GetAssets().push_back(trimesh_shape);
      }
    }
    // Add the original triangle mesh as asset
    if (use_original_asset) {
      ChSharedPtr<ChTriangleMeshShape> trimesh_shape(new ChTriangleMeshShape);
      trimesh_shape->SetMesh(convex_mesh);
      trimesh_shape->SetName(convex_mesh.GetFileName());
      trimesh_shape->Pos = VNULL;
      trimesh_shape->Rot = QUNIT;
      body->GetAssets().push_back(trimesh_shape);
    }

    // **************************

    //        chassisBody->RecomputeCollisionModel();
    //        chassisBody->SetCollide(false);

    chassisBody->GetMaterialSurface()->SetFriction(mu_t);
  }

  virtual void SetAttributes(const ChVector<>& otherPos = ChVector<>(0, 0, 0),
                             const ChQuaternion<>& otherRot = ChQuaternion<>(1, 0, 0, 0)) {
    rot = otherRot;
    pos = otherPos;
  }

 private:
  ChConvexDecompositionHACDv2 chassis_convex;
  geometry::ChTriangleMeshConnected chassis_mesh;

  ChQuaternion<> rot;
  ChVector<> pos;
};

#endif /* VEHICLEEXTRAPROPERTIES_H_ */
