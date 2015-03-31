/*
 * MyVehicle.cpp
 *
 *  Created on: Mar 31, 2015
 *      Author: Arman Pazouki
 */

#include "MyVehicle.h"

MyVehicle::MyVehicle(ChSystem* system) {

	InitializeVehicle();

  // Create and initialize the vehicle system

  m_vehicle = ChSharedPtr<Vehicle>(new Vehicle(system, vehicle::GetDataFile(vehicle_file)));
  m_vehicle->Initialize(ChCoordsys<>(initLoc, initRot));

  // Create and initialize the powertrain system
  m_powertrain = ChSharedPtr<SimplePowertrain>(new SimplePowertrain(vehicle::GetDataFile(simplepowertrain_file)));
  m_powertrain->Initialize();

  // Add contact geometry to the vehicle wheel bodies
  double radius = 0.47;
  double width = 0.254;

  int numAxles = m_vehicle->GetNumberAxles();
  int numWheels = 2 * numAxles;

  for (int i = 0; i < numWheels; i++) {
    double radius = m_vehicle->GetWheel(i)->GetRadius();
    double width = m_vehicle->GetWheel(i)->GetWidth();

    ChSharedPtr<ChBody> wheelBody = m_vehicle->GetWheelBody(i);

    wheelBody->ChangeCollisionModel(new collision::ChCollisionModelParallel);

    wheelBody->GetCollisionModel()->ClearModel();
    wheelBody->GetCollisionModel()->AddCylinder(radius, radius, width / 2);
    wheelBody->GetCollisionModel()->BuildModel();

    wheelBody->SetCollide(true);
    wheelBody->GetMaterialSurface()->SetFriction(0.8f);
  }

  // The vector of tire forces is required by the ChronoVehicle API. Since we
  // use rigid contact for tire-terrain interaction, these are always zero.
  m_tire_forces.resize(numWheels);
}

void MyVehicle::InitializeVehicle() {
	vehicle_file = std::string("hmmwv/vehicle/HMMWV_Vehicle_simple.json");
	simplepowertrain_file = std::string("hmmwv/powertrain/HMMWV_SimplePowertrain.json");
	initLoc = ChVector<>(0, 0, 1.0);
	initRot = ChQuaternion<>(1, 0, 0, 0);
}

void MyVehicle::Update(double time) {
  // Calculate driver inputs at current time
  double throttle = 0;
  double steering = 0;
  double braking = 0;

  if (time > 0.5)
    throttle = 1.0;
  else if (time > 0.25)
    throttle = 4 * (time - 0.25);

  // Update the powertrain system
  m_powertrain->Update(time, throttle, m_vehicle->GetDriveshaftSpeed());

  // Update the vehicle system.
  m_vehicle->Update(time, steering, braking, m_powertrain->GetOutputTorque(), m_tire_forces);
}
