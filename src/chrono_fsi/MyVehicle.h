/*
 * MyVehicle.h
 *
 *  Created on: Mar 31, 2015
 *      Author: Arman Pazouki
 */

#ifndef MYVEHICLE_H_
#define MYVEHICLE_H_

#include "chrono_parallel/physics/ChSystemParallel.h"

#include "config.h"
#include "subsys/ChVehicleModelData.h"
#include "subsys/vehicle/Vehicle.h"
#include "subsys/powertrain/SimplePowertrain.h"

using namespace chrono;

class MyVehicle {
 public:
  MyVehicle(ChSystem* system);

  void InitializeVehicle();
  void Update(double time);

  ChSharedPtr<Vehicle> m_vehicle;
  ChSharedPtr<SimplePowertrain> m_powertrain;
  ChTireForces m_tire_forces;

 private:
  std::string vehicle_file;				// JSON file for vehicle model
  std::string simplepowertrain_file;	// JSON files for powertrain (simple)
  ChVector<> initLoc;		// Initial vehicle position
  ChQuaternion<> initRot;	// Initial vehicle orientation
};


#endif /* MYVEHICLE_H_ */
