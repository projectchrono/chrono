/*
 * VehicleProperties.h
 *
 *  Created on: Apr 1, 2015
 *      Author: Arman Pazouki
 */

#ifndef VEHICLEPROPERTIES_H_
#define VEHICLEPROPERTIES_H_

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
//


using namespace chrono;
using namespace chrono::collision;
// -----------------------------------------------------------------------------
// Specification of the vehicle model
// -----------------------------------------------------------------------------

enum WheelType { CYLINDRICAL, LUGGED };

// Type of wheel/tire (controls both contact and visualization)
WheelType wheel_type = CYLINDRICAL;

// JSON files for vehicle model (using different wheel visualization meshes)
std::string vehicle_file_cyl("hmmwv/vehicle/HMMWV_Vehicle_simple.json");
std::string vehicle_file_lug("hmmwv/vehicle/HMMWV_Vehicle_simple_lugged.json");

// JSON files for powertrain (simple)
std::string simplepowertrain_file("hmmwv/powertrain/HMMWV_SimplePowertrain.json");

// Initial vehicle position and orientation
ChVector<> initLoc(-3.0, 0, 0.75);
ChQuaternion<> initRot(1, 0, 0, 0);

// Coefficient of friction
float mu_t = 0.8;

// -----------------------------------------------------------------------------
// Specification of the terrain
// -----------------------------------------------------------------------------

enum TerrainType { RIGID, GRANULAR };

// Type of terrain
TerrainType terrain_type = RIGID;

// Control visibility of containing bin walls
bool visible_walls = false;


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
		ChCollisionModelParallel* coll_model = (ChCollisionModelParallel*) wheelBody->GetCollisionModel();
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
		  utils::AddConvexCollisionModel(wheelBody, lugged_mesh, lugged_convex, VNULL,
		    Q_from_AngAxis(j * 24 * CH_C_DEG_TO_RAD, VECT_Y), false);
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


#endif /* VEHICLEPROPERTIES_H_ */
