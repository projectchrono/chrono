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
// Vehicle model constructed from a JSON specification file
//
// =============================================================================

#include <cstdio>

#include "assets/ChCylinderShape.h"
#include "assets/ChTriangleMeshShape.h"
#include "assets/ChTexture.h"
#include "assets/ChColorAsset.h"
#include "physics/ChGlobal.h"

#include "subsys/vehicle/Vehicle.h"

#include "subsys/suspension/DoubleWishbone.h"
#include "subsys/suspension/DoubleWishboneReduced.h"
#include "subsys/suspension/SolidAxle.h"
#include "subsys/suspension/MultiLink.h"

#include "subsys/steering/PitmanArm.h"
#include "subsys/steering/RackPinion.h"

#include "subsys/driveline/ShaftsDriveline2WD.h"
#include "subsys/driveline/ShaftsDriveline4WD.h"
#include "subsys/wheel/Wheel.h"
#include "subsys/brake/BrakeSimple.h"

#include "utils/ChUtilsInputOutput.h"
#include "utils/ChUtilsData.h"

#include "rapidjson/document.h"
#include "rapidjson/filereadstream.h"

using namespace rapidjson;

namespace chrono {


// -----------------------------------------------------------------------------
// These utility functions return a ChVector and a ChQuaternion<>, respectively,
// from the specified JSON array.
// -----------------------------------------------------------------------------
static ChVector<> loadVector(const Value& a)
{
  assert(a.IsArray());
  assert(a.Size() == 3);
  return ChVector<>(a[0u].GetDouble(), a[1u].GetDouble(), a[2u].GetDouble());
}

static ChQuaternion<> loadQuaternion(const Value& a)
{
  assert(a.IsArray());
  assert(a.Size() == 4);
  return ChQuaternion<>(a[0u].GetDouble(), a[1u].GetDouble(), a[2u].GetDouble(), a[3u].GetDouble());
}


// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void Vehicle::LoadSteering(const std::string& filename)
{
  FILE* fp = fopen(filename.c_str(), "r");

  char readBuffer[65536];
  FileReadStream is(fp, readBuffer, sizeof(readBuffer));

  fclose(fp);

  Document d;
  d.ParseStream(is);

  // Check that the given file is a steering specification file.
  assert(d.HasMember("Type"));
  std::string type = d["Type"].GetString();
  assert(type.compare("Steering") == 0);

  // Extract the driveline type.
  assert(d.HasMember("Template"));
  std::string subtype = d["Template"].GetString();

  // Create the steering using the appropriate template.
  // Create the driveline using the appropriate template.
  if (subtype.compare("PitmanArm") == 0)
  {
    m_steering = ChSharedPtr<ChSteering>(new PitmanArm(d));
  }
  else if (subtype.compare("RackPinion") == 0)
  {
    m_steering = ChSharedPtr<ChSteering>(new RackPinion(d));
  }
}


// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void Vehicle::LoadDriveline(const std::string& filename)
{
  FILE* fp = fopen(filename.c_str(), "r");

  char readBuffer[65536];
  FileReadStream is(fp, readBuffer, sizeof(readBuffer));

  fclose(fp);

  Document d;
  d.ParseStream(is);

  // Check that the given file is a driveline specification file.
  assert(d.HasMember("Type"));
  std::string type = d["Type"].GetString();
  assert(type.compare("Driveline") == 0);

  // Extract the driveline type.
  assert(d.HasMember("Template"));
  std::string subtype = d["Template"].GetString();

  // Create the driveline using the appropriate template.
  if (subtype.compare("ShaftsDriveline2WD") == 0)
  {
    m_driveline = ChSharedPtr<ChDriveline>(new ShaftsDriveline2WD(d));
  }
  else if (subtype.compare("ShaftsDriveline4WD") == 0)
  {
    m_driveline = ChSharedPtr<ChDriveline>(new ShaftsDriveline4WD(d));
  }
}


// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void Vehicle::LoadSuspension(const std::string& filename,
                             int                axle)
{
  FILE* fp = fopen(filename.c_str(), "r");

  char readBuffer[65536];
  FileReadStream is(fp, readBuffer, sizeof(readBuffer));

  fclose(fp);

  Document d;
  d.ParseStream(is);

  // Check that the given file is a suspension specification file.
  assert(d.HasMember("Type"));
  std::string type = d["Type"].GetString();
  assert(type.compare("Suspension") == 0);

  // Extract the suspension type.
  assert(d.HasMember("Template"));
  std::string subtype = d["Template"].GetString();

  // Create the suspension using the appropriate template.
  if (subtype.compare("DoubleWishbone") == 0)
  {
    m_suspensions[axle] = ChSharedPtr<ChSuspension>(new DoubleWishbone(d));
  }
  else if (subtype.compare("DoubleWishboneReduced") == 0)
  {
    m_suspensions[axle] = ChSharedPtr<ChSuspension>(new DoubleWishboneReduced(d));
  }
  else if (subtype.compare("SolidAxle") == 0)
  {
    m_suspensions[axle] = ChSharedPtr<ChSuspension>(new SolidAxle(d));
  }
  else if (subtype.compare("MultiLink") == 0)
  {
    m_suspensions[axle] = ChSharedPtr<ChSuspension>(new MultiLink(d));
  }
}


// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void Vehicle::LoadWheel(const std::string& filename, int axle, int side)
{
  FILE* fp = fopen(filename.c_str(), "r");

  char readBuffer[65536];
  FileReadStream is(fp, readBuffer, sizeof(readBuffer));

  fclose(fp);

  Document d;
  d.ParseStream(is);

  // Check that the given file is a wheel specification file.
  assert(d.HasMember("Type"));
  std::string type = d["Type"].GetString();
  assert(type.compare("Wheel") == 0);

  // Extract the wheel type.
  assert(d.HasMember("Template"));
  std::string subtype = d["Template"].GetString();

  // Create the wheel using the appropriate template.
  if (subtype.compare("Wheel") == 0)
  {
    m_wheels[2 * axle + side] = ChSharedPtr<ChWheel>(new Wheel(filename));
  }
}


// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void Vehicle::LoadBrake(const std::string& filename, int axle, int side)
{
  FILE* fp = fopen(filename.c_str(), "r");

  char readBuffer[65536];
  FileReadStream is(fp, readBuffer, sizeof(readBuffer));

  fclose(fp);

  Document d;
  d.ParseStream(is);

  // Check that the given file is a wheel specification file.
  assert(d.HasMember("Type"));
  std::string type = d["Type"].GetString();
  assert(type.compare("Brake") == 0);

  // Extract the wheel type.
  assert(d.HasMember("Template"));
  std::string subtype = d["Template"].GetString();

  // Create the brake using the appropriate template.
  if (subtype.compare("BrakeSimple") == 0)
  {
    m_brakes[2 * axle + side] = ChSharedPtr<ChBrake>(new BrakeSimple(filename));
  }
}


// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
Vehicle::Vehicle(const std::string& filename,
                 bool               fixed)
: m_chassisUseMesh(false)
{
  // -------------------------------------------
  // Open and parse the input file
  // -------------------------------------------
  FILE* fp = fopen(filename.c_str(), "r");

  char readBuffer[65536];
  FileReadStream is(fp, readBuffer, sizeof(readBuffer));

  fclose(fp);

  Document d;
  d.ParseStream(is);

  // Read top-level data
  assert(d.HasMember("Type"));
  assert(d.HasMember("Template"));
  assert(d.HasMember("Name"));

  // -------------------------------------------
  // Create the chassis body
  // -------------------------------------------

  m_chassis = ChSharedPtr<ChBodyAuxRef>(new ChBodyAuxRef);

  m_chassisMass = d["Chassis"]["Mass"].GetDouble();
  m_chassisCOM = loadVector(d["Chassis"]["COM"]);
  m_chassisInertia = loadVector(d["Chassis"]["Inertia"]);

  m_chassis->SetIdentifier(0);
  m_chassis->SetName("chassis");
  m_chassis->SetMass(m_chassisMass);
  m_chassis->SetFrame_COG_to_REF(ChFrame<>(m_chassisCOM, ChQuaternion<>(1, 0, 0, 0)));
  m_chassis->SetInertiaXX(m_chassisInertia);
  m_chassis->SetBodyFixed(fixed);

  if (d.HasMember("Visualization"))
  {
    assert(d["Visualization"].HasMember("Mesh Filename"));
    assert(d["Visualization"].HasMember("Mesh Name"));

    m_chassisMeshFile = d["Visualization"]["Mesh Filename"].GetString();
    m_chassisMeshName = d["Visualization"]["Mesh Name"].GetString();

    geometry::ChTriangleMeshConnected trimesh;
    trimesh.LoadWavefrontMesh(utils::GetModelDataFile(m_chassisMeshFile), false, false);

    ChSharedPtr<ChTriangleMeshShape> trimesh_shape(new ChTriangleMeshShape);
    trimesh_shape->SetMesh(trimesh);
    trimesh_shape->SetName(m_chassisMeshName);
    m_chassis->AddAsset(trimesh_shape);

    m_chassisUseMesh = true;
  }
  else
  {
    ChSharedPtr<ChSphereShape> sphere(new ChSphereShape);
    sphere->GetSphereGeometry().rad = 0.1;
    sphere->Pos = m_chassisCOM;
    m_chassis->AddAsset(sphere);
  }

  Add(m_chassis);

  // ---------------------------------
  // More validations of the JSON file
  // ---------------------------------

  assert(d.HasMember("Steering"));
  assert(d.HasMember("Driveline"));
  assert(d.HasMember("Axles"));
  assert(d["Axles"].IsArray());

  // Extract the number of axles.
  m_num_axles = d["Axles"].Size();

  // Resize arrays
  m_suspensions.resize(m_num_axles);
  m_suspLocations.resize(m_num_axles);
  m_wheels.resize(2 * m_num_axles);
  m_brakes.resize(2 * m_num_axles);

  // -----------------------------
  // Create the steering subsystem
  // -----------------------------

  {
    std::string file_name = d["Steering"]["Input File"].GetString();
    LoadSteering(utils::GetModelDataFile(file_name));
    m_steeringLoc = loadVector(d["Steering"]["Location"]);
    m_steeringRot = loadQuaternion(d["Steering"]["Orientation"]);
    m_steer_susp = d["Steering"]["Suspension Index"].GetInt();
  }

  // --------------------
  // Create the driveline
  // --------------------

  {
    std::string file_name = d["Driveline"]["Input File"].GetString();
    LoadDriveline(utils::GetModelDataFile(file_name));
    SizeType num_driven_susp = d["Driveline"]["Suspension Indexes"].Size();
    m_driven_susp.resize(num_driven_susp);
    for (SizeType i = 0; i < num_driven_susp; i++) {
      m_driven_susp[i] = d["Driveline"]["Suspension Indexes"][i].GetInt();
    }

    assert(num_driven_susp == GetDriveline()->GetNumDrivenAxles());
  }

  // ---------------------------------------------------
  // Create the suspension, wheel, and brake subsystems.
  // ---------------------------------------------------

  for (int i = 0; i < m_num_axles; i++) {
    // Suspension
    std::string file_name = d["Axles"][i]["Suspension Input File"].GetString();
    LoadSuspension(utils::GetModelDataFile(file_name), i);
    m_suspLocations[i] = loadVector(d["Axles"][i]["Suspension Location"]);

    // Left and right wheels
    file_name = d["Axles"][i]["Left Wheel Input File"].GetString();
    LoadWheel(utils::GetModelDataFile(file_name), i, 0);
    file_name = d["Axles"][i]["Right Wheel Input File"].GetString();
    LoadWheel(utils::GetModelDataFile(file_name), i, 1);

    // Left and right brakes
    file_name = d["Axles"][i]["Left Brake Input File"].GetString();
    LoadBrake(utils::GetModelDataFile(file_name), i, 0);

    file_name = d["Axles"][i]["Right Brake Input File"].GetString();
    LoadBrake(utils::GetModelDataFile(file_name), i, 1);
  }

  // -----------------------
  // Extract driver position
  // -----------------------

  m_driverCsys.pos = loadVector(d["Driver Position"]["Location"]);
  m_driverCsys.rot = loadQuaternion(d["Driver Position"]["Orientation"]);
}


Vehicle::~Vehicle()
{
}


// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void Vehicle::Initialize(const ChCoordsys<>& chassisPos)
{
  m_chassis->SetFrame_REF_to_abs(ChFrame<>(chassisPos));

  // Initialize the steering subsystem.
  m_steering->Initialize(m_chassis, m_steeringLoc, m_steeringRot);

  // Initialize the suspension, wheel, and brake subsystems.
  for (int i = 0; i < m_num_axles; i++)
  {
    if (m_steer_susp == i)
      m_suspensions[i]->Initialize(m_chassis, m_suspLocations[i], m_steering->GetSteeringLink());
    else
      m_suspensions[i]->Initialize(m_chassis, m_suspLocations[i], m_chassis);

    m_wheels[2 * i]->Initialize(m_suspensions[i]->GetSpindle(LEFT));
    m_wheels[2 * i + 1]->Initialize(m_suspensions[i]->GetSpindle(RIGHT));

    m_brakes[2 * i]->Initialize(m_suspensions[i]->GetRevolute(LEFT));
    m_brakes[2 * i + 1]->Initialize(m_suspensions[i]->GetRevolute(RIGHT));
  }

  // Initialize the driveline
  m_driveline->Initialize(m_chassis, m_suspensions, m_driven_susp);
}


// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void Vehicle::Update(double              time,
                     double              steering,
                     double              braking,
                     double              powertrain_torque,
                     const ChTireForces& tire_forces)
{
  // Apply powertrain torque to the driveline's input shaft.
  m_driveline->ApplyDriveshaftTorque(powertrain_torque);

  // Let the steering subsystem process the steering input.
  m_steering->Update(time, steering);

  // Apply tire forces to spindle bodies and apply braking.
  for (int i = 0; i < m_num_axles; i++) {
    m_suspensions[i]->ApplyTireForce(LEFT, tire_forces[2 * i]);
    m_suspensions[i]->ApplyTireForce(RIGHT, tire_forces[2 * i + 1]);

    m_brakes[2 * i]->ApplyBrakeModulation(braking);
    m_brakes[2 * i + 1]->ApplyBrakeModulation(braking);
  }
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void Vehicle::ExportMeshPovray(const std::string& out_dir)
{
  if (m_chassisUseMesh) {
    
    utils::WriteMeshPovray(utils::GetModelDataFile(m_chassisMeshFile),
                           m_chassisMeshName,
                           out_dir,
                           ChColor(0.82f, 0.7f, 0.5f));
  }

  for (int i = 0; i < m_num_axles; i++) {
    Wheel* wL = static_cast<Wheel*>(m_wheels[2 * i].get_ptr());
    Wheel* wR = static_cast<Wheel*>(m_wheels[2 * i + 1].get_ptr());

    wL->ExportMeshPovray(out_dir);
    wR->ExportMeshPovray(out_dir);
  }
}



// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
double Vehicle::GetSpringForce(const ChWheelID& wheel_id) const
{
  return m_suspensions[wheel_id.axle()].StaticCastTo<ChDoubleWishbone>()->GetSpringForce(wheel_id.side());
}

double Vehicle::GetSpringLength(const ChWheelID& wheel_id) const
{
  return m_suspensions[wheel_id.axle()].StaticCastTo<ChDoubleWishbone>()->GetSpringLength(wheel_id.side());
}

double Vehicle::GetSpringDeformation(const ChWheelID& wheel_id) const
{
  return m_suspensions[wheel_id.axle()].StaticCastTo<ChDoubleWishbone>()->GetSpringDeformation(wheel_id.side());
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
double Vehicle::GetShockForce(const ChWheelID& wheel_id) const
{
  return m_suspensions[wheel_id.axle()].StaticCastTo<ChDoubleWishbone>()->GetShockForce(wheel_id.side());
}

double Vehicle::GetShockLength(const ChWheelID& wheel_id) const
{
  return m_suspensions[wheel_id.axle()].StaticCastTo<ChDoubleWishbone>()->GetShockLength(wheel_id.side());
}

double Vehicle::GetShockVelocity(const ChWheelID& wheel_id) const
{
  return m_suspensions[wheel_id.axle()].StaticCastTo<ChDoubleWishbone>()->GetShockVelocity(wheel_id.side());
}



// -----------------------------------------------------------------------------
// Log the hardpoint locations for the front-right and rear-right suspension
// subsystems (display in inches)
// -----------------------------------------------------------------------------
void Vehicle::LogHardpointLocations()
{
  GetLog().SetNumFormat("%7.3f");

  GetLog() << "\n---- FRONT suspension hardpoint locations (RIGHT side)\n";
  m_suspensions[0].StaticCastTo<ChDoubleWishbone>()->LogHardpointLocations(ChVector<>(-37.78, 0, 30.77), true);

  GetLog() << "\n---- REAR suspension hardpoint locations (RIGHT side)\n";
  m_suspensions[1].StaticCastTo<ChDoubleWishbone>()->LogHardpointLocations(ChVector<>(-170.77, 0, 30.77), true);

  GetLog() << "\n\n";

  GetLog().SetNumFormat("%g");
}


// -----------------------------------------------------------------------------
// Log the spring length, deformation, and force.
// Log the shock length, velocity, and force.
// Log constraint violations of suspension joints.
//
// Lengths are reported in inches, velocities in inches/s, and forces in lbf
// -----------------------------------------------------------------------------
void Vehicle::DebugLog(int what)
{
  GetLog().SetNumFormat("%10.2f");

  if (what & DBG_SPRINGS)
  {
    GetLog() << "\n---- Spring (front-left, front-right, rear-left, rear-right)\n";
    GetLog() << "Length [m]       "
      << GetSpringLength(FRONT_LEFT) << "  "
      << GetSpringLength(FRONT_RIGHT) << "  "
      << GetSpringLength(REAR_LEFT) << "  "
      << GetSpringLength(REAR_RIGHT) << "\n";
    GetLog() << "Deformation [m]  "
      << GetSpringDeformation(FRONT_LEFT) << "  "
      << GetSpringDeformation(FRONT_RIGHT) << "  "
      << GetSpringDeformation(REAR_LEFT) << "  "
      << GetSpringDeformation(REAR_RIGHT) << "\n";
    GetLog() << "Force [N]         "
      << GetSpringForce(FRONT_LEFT) << "  "
      << GetSpringForce(FRONT_RIGHT) << "  "
      << GetSpringForce(REAR_LEFT) << "  "
      << GetSpringForce(REAR_RIGHT) << "\n";
  }

  if (what & DBG_SHOCKS)
  {
    GetLog() << "\n---- Shock (front-left, front-right, rear-left, rear-right)\n";
    GetLog() << "Length [m]       "
      << GetShockLength(FRONT_LEFT) << "  "
      << GetShockLength(FRONT_RIGHT) << "  "
      << GetShockLength(REAR_LEFT) << "  "
      << GetShockLength(REAR_RIGHT) << "\n";
    GetLog() << "Velocity [m/s]   "
      << GetShockVelocity(FRONT_LEFT) << "  "
      << GetShockVelocity(FRONT_RIGHT) << "  "
      << GetShockVelocity(REAR_LEFT) << "  "
      << GetShockVelocity(REAR_RIGHT) << "\n";
    GetLog() << "Force [N]         "
      << GetShockForce(FRONT_LEFT) << "  "
      << GetShockForce(FRONT_RIGHT) << "  "
      << GetShockForce(REAR_LEFT) << "  "
      << GetShockForce(REAR_RIGHT) << "\n";
  }

  if (what & DBG_CONSTRAINTS)
  {
    // Report constraint violations for all joints
    LogConstraintViolations();
  }

  GetLog().SetNumFormat("%g");
}




} // end namespace chrono
