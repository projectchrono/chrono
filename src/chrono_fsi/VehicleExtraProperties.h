/*
 * VehicleExtraProperties.h
 *
 *  Created on: Apr 1, 2015
 *      Author: Arman Pazouki
 */

#ifndef VEHICLEEXTRAPROPERTIES_H_
#define VEHICLEEXTRAPROPERTIES_H_



#include "chrono_parallel/physics/ChSystemParallel.h"

#include "chrono_vehicle/utils/ChWheeledVehicleAssembly.h"
//#include "chrono_utils/ChUtilsGeometry.h"
#include "utils/ChUtilsCreators.h"
//#include "chrono_utils/ChUtilsGenerators.h"
//#include "chrono_utils/ChUtilsInputOutput.h"

// Callback class for providing driver inputs.
class MyDriverInputs : public chrono::ChDriverInputsCallback {
 public:
  MyDriverInputs(double delay);
  virtual void onCallback(double time, double& throttle, double& steering, double& braking);

 private:
  double m_delay;
};

// Callback class for specifying rigid tire contact model.
// This version uses cylindrical contact shapes.
class MyCylindricalTire : public chrono::ChTireContactCallback {
 public:
  virtual void onCallback(chrono::ChSharedPtr<chrono::ChBody> wheelBody, double radius, double width);
};

// Callback class for specifying rigid tire contact model.
// This version uses a collection of convex contact shapes (meshes).
class MyLuggedTire : public chrono::ChTireContactCallback {
 public:
  MyLuggedTire();
  virtual void onCallback(chrono::ChSharedPtr<chrono::ChBody> wheelBody, double radius, double width);

 private:
  chrono::collision::ChConvexDecompositionHACDv2 lugged_convex;
  int num_hulls;
};

// Callback class for specifying chassis contact model.
// This version uses a box representing the chassis.
// In addition, this version overrides the visualization assets of the provided
// chassis body with the collision meshes.
class MyChassisBoxModel_vis : public chrono::ChChassisContactCallback {
 public:
  virtual void onCallback(chrono::ChSharedPtr<chrono::ChBodyAuxRef> chassisBody);
  virtual void SetAttributes(const chrono::ChVector<>& otherSize,
                             const chrono::ChQuaternion<>& otherRot = chrono::ChQuaternion<>(1, 0, 0, 0),
                             const chrono::ChVector<>& otherLoc = chrono::ChVector<>(0, 0, 0));

 private:
  chrono::collision::ChConvexDecompositionHACDv2 chassis_convex;
  chrono::geometry::ChTriangleMeshConnected chassis_mesh;

  chrono::ChVector<> size;
  chrono::ChQuaternion<> rot;
  chrono::ChVector<> loc;
};

// Callback class for specifying chassis contact model.
// This version uses a sphere representing the chassis.
// In addition, this version overrides the visualization assets of the provided
// chassis body with the collision meshes.
class MyChassisSphereModel_vis : public chrono::ChChassisContactCallback {
 public:
  virtual void onCallback(chrono::ChSharedPtr<chrono::ChBodyAuxRef> chassisBody) ;
  virtual void SetAttributes(double otherRad,
                             const chrono::ChQuaternion<>& otherRot = chrono::ChQuaternion<>(1, 0, 0, 0),
                             const chrono::ChVector<>& otherLoc = chrono::ChVector<>(0, 0, 0)) ;

 private:
  chrono::collision::ChConvexDecompositionHACDv2 chassis_convex;
  chrono::geometry::ChTriangleMeshConnected chassis_mesh;

  double rad;
  chrono::ChQuaternion<> rot;
  chrono::ChVector<> loc;
};

// Callback class for specifying chassis contact model.
// This version uses a convex decomposition of an obj representing the chassis.
// In addition, this version overrides the visualization assets of the provided
// chassis body with the collision meshes.
class MyChassisSimpleConvexMesh : public chrono::ChChassisContactCallback {
 public:
  MyChassisSimpleConvexMesh();
  virtual void onCallback(chrono::ChSharedPtr<chrono::ChBodyAuxRef> chassisBody);

 private:
  chrono::collision::ChConvexDecompositionHACDv2 chassis_convex;
  chrono::geometry::ChTriangleMeshConnected chassis_mesh;
  std::string chassis_obj_file;

  chrono::ChQuaternion<> rot;
  chrono::ChVector<> pos;
};

// Callback class for specifying chassis contact model.
// This version uses a triangular given in an obj representing the chassis.
// In addition, this version overrides the visualization assets of the provided
// chassis body with the collision meshes.
class MyChassisSimpleTriMesh_vis : public chrono::ChChassisContactCallback {
 public:
  MyChassisSimpleTriMesh_vis();
  virtual void onCallback(chrono::ChSharedPtr<chrono::ChBodyAuxRef> chassisBody);

 private:
  std::string chassis_obj_file;

  chrono::ChQuaternion<> rot;
  chrono::ChVector<> pos;
};

#endif /* VEHICLEEXTRAPROPERTIES_H_ */
