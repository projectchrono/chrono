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

#include "assets/ChSphereShape.h"
#include "assets/ChCylinderShape.h"
#include "assets/ChTriangleMeshShape.h"
#include "assets/ChTexture.h"
#include "assets/ChColorAsset.h"
#include "physics/ChGlobal.h"

#include "chrono_vehicle/vehicle/Vehicle.h"

#include "chrono_vehicle/suspension/DoubleWishbone.h"
#include "chrono_vehicle/suspension/DoubleWishboneReduced.h"
#include "chrono_vehicle/suspension/SolidAxle.h"
#include "chrono_vehicle/suspension/MultiLink.h"

#include "chrono_vehicle/antirollbar/AntirollBarRSD.h"

#include "chrono_vehicle/steering/PitmanArm.h"
#include "chrono_vehicle/steering/RackPinion.h"

#include "chrono_vehicle/driveline/ShaftsDriveline2WD.h"
#include "chrono_vehicle/driveline/ShaftsDriveline4WD.h"
#include "chrono_vehicle/driveline/SimpleDriveline.h"
#include "chrono_vehicle/wheel/Wheel.h"
#include "chrono_vehicle/brake/BrakeSimple.h"

#include "chrono_vehicle/ChVehicleModelData.h"

#include "thirdparty/rapidjson/document.h"
#include "thirdparty/rapidjson/filereadstream.h"

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
void Vehicle::LoadSteering(const std::string& filename,
                           int                which)
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
    m_steerings[which] = ChSharedPtr<ChSteering>(new PitmanArm(d));
  }
  else if (subtype.compare("RackPinion") == 0)
  {
    m_steerings[which] = ChSharedPtr<ChSteering>(new RackPinion(d));
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
  else if (subtype.compare("SimpleDriveline") == 0) {
    m_driveline = ChSharedPtr<ChDriveline>(new SimpleDriveline(d));
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
void Vehicle::LoadAntirollbar(const std::string& filename)
{
  FILE* fp = fopen(filename.c_str(), "r");

  char readBuffer[65536];
  FileReadStream is(fp, readBuffer, sizeof(readBuffer));

  fclose(fp);

  Document d;
  d.ParseStream(is);

  // Check that the given file is an antirollbar specification file.
  assert(d.HasMember("Type"));
  std::string type = d["Type"].GetString();
  assert(type.compare("Antirollbar") == 0);

  // Extract the antirollbar type.
  assert(d.HasMember("Template"));
  std::string subtype = d["Template"].GetString();

  if (subtype.compare("AntirollBarRSD") == 0) {
    m_antirollbars.push_back(ChSharedPtr<ChAntirollBar>(new AntirollBarRSD(d)));
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
Vehicle::Vehicle(const std::string& filename)
: m_chassisUseMesh(false)
{
  Create(filename);
}

Vehicle::Vehicle(ChSystem*          system,
                 const std::string& filename)
: ChVehicle(system),
  m_chassisUseMesh(false)
{
  Create(filename);
}


// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void Vehicle::Create(const std::string& filename)
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

  m_chassis = ChSharedPtr<ChBodyAuxRef>(new ChBodyAuxRef(m_system->GetContactMethod()));

  m_chassisMass = d["Chassis"]["Mass"].GetDouble();
  m_chassisCOM = loadVector(d["Chassis"]["COM"]);
  m_chassisInertia = loadVector(d["Chassis"]["Inertia"]);

  m_chassis->SetIdentifier(0);
  m_chassis->SetName("chassis");
  m_chassis->SetMass(m_chassisMass);
  m_chassis->SetFrame_COG_to_REF(ChFrame<>(m_chassisCOM, ChQuaternion<>(1, 0, 0, 0)));
  m_chassis->SetInertiaXX(m_chassisInertia);

  if (d.HasMember("Visualization"))
  {
    assert(d["Visualization"].HasMember("Mesh Filename"));
    assert(d["Visualization"].HasMember("Mesh Name"));

    m_chassisMeshFile = d["Visualization"]["Mesh Filename"].GetString();
    m_chassisMeshName = d["Visualization"]["Mesh Name"].GetString();

    geometry::ChTriangleMeshConnected trimesh;
    trimesh.LoadWavefrontMesh(vehicle::GetDataFile(m_chassisMeshFile), false, false);

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

  m_system->Add(m_chassis);

  // ---------------------------------
  // More validations of the JSON file
  // ---------------------------------

  assert(d.HasMember("Steering Subsystems"));
  assert(d.HasMember("Driveline"));
  assert(d.HasMember("Axles"));
  assert(d["Axles"].IsArray());
  assert(d["Steering Subsystems"].IsArray());

  // Extract the number of axles.
  m_num_axles = d["Axles"].Size();

  // Extract the number of steering subsystems
  m_num_strs = d["Steering Subsystems"].Size();

  // Resize arrays
  m_suspensions.resize(m_num_axles);
  m_suspLocations.resize(m_num_axles);
  m_suspSteering.resize(m_num_axles, -1);
  m_wheels.resize(2 * m_num_axles);
  m_brakes.resize(2 * m_num_axles);

  m_steerings.resize(m_num_strs);
  m_strLocations.resize(m_num_strs);
  m_strRotations.resize(m_num_strs);

  // ------------------------------
  // Create the steering subsystems
  // ------------------------------

  for (int i = 0; i < m_num_strs; i++) {
    std::string file_name = d["Steering Subsystems"][i]["Input File"].GetString();
    LoadSteering(vehicle::GetDataFile(file_name), i);
    m_strLocations[i] = loadVector(d["Steering Subsystems"][i]["Location"]);
    m_strRotations[i] = loadQuaternion(d["Steering Subsystems"][i]["Orientation"]);
  }

  // --------------------
  // Create the driveline
  // --------------------

  {
    std::string file_name = d["Driveline"]["Input File"].GetString();
    LoadDriveline(vehicle::GetDataFile(file_name));
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
    LoadSuspension(vehicle::GetDataFile(file_name), i);
    m_suspLocations[i] = loadVector(d["Axles"][i]["Suspension Location"]);

    // Index of steering subsystem (if applicable)
    if (d["Axles"][i].HasMember("Steering Index")) {
      m_suspSteering[i] = d["Axles"][i]["Steering Index"].GetInt();
    }

    // Antirollbar (if applicable)
    if (d["Axles"][i].HasMember("Antirollbar Input File")) {
      assert(m_suspensions[i]->IsIndependent());
      assert(d["Axles"][i].HasMember("Antirollbar Location"));
      file_name = d["Axles"][i]["Antirollbar Input File"].GetString();
      LoadAntirollbar(vehicle::GetDataFile(file_name));
      m_arbLocations.push_back(loadVector(d["Axles"][i]["Antirollbar Location"]));
      m_arbSuspension.push_back(i);
    }

    // Left and right wheels
    file_name = d["Axles"][i]["Left Wheel Input File"].GetString();
    LoadWheel(vehicle::GetDataFile(file_name), i, 0);
    file_name = d["Axles"][i]["Right Wheel Input File"].GetString();
    LoadWheel(vehicle::GetDataFile(file_name), i, 1);

    // Left and right brakes
    file_name = d["Axles"][i]["Left Brake Input File"].GetString();
    LoadBrake(vehicle::GetDataFile(file_name), i, 0);

    file_name = d["Axles"][i]["Right Brake Input File"].GetString();
    LoadBrake(vehicle::GetDataFile(file_name), i, 1);
  }

  // -----------------------
  // Extract driver position
  // -----------------------

  m_driverCsys.pos = loadVector(d["Driver Position"]["Location"]);
  m_driverCsys.rot = loadQuaternion(d["Driver Position"]["Orientation"]);
}


// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void Vehicle::Initialize(const ChCoordsys<>& chassisPos)
{
  m_chassis->SetFrame_REF_to_abs(ChFrame<>(chassisPos));

  // Initialize the steering subsystems.
  for (int i = 0; i < m_num_strs; i++) {
    m_steerings[i]->Initialize(m_chassis, m_strLocations[i], m_strRotations[i]);
  }

  // Initialize the suspension, wheel, and brake subsystems.
  for (int i = 0; i < m_num_axles; i++) {
    if (m_suspSteering[i] >= 0)
      m_suspensions[i]->Initialize(m_chassis, m_suspLocations[i], m_steerings[m_suspSteering[i]]->GetSteeringLink());
    else
      m_suspensions[i]->Initialize(m_chassis, m_suspLocations[i], m_chassis);

    m_wheels[2 * i]->Initialize(m_suspensions[i]->GetSpindle(LEFT));
    m_wheels[2 * i + 1]->Initialize(m_suspensions[i]->GetSpindle(RIGHT));

    m_brakes[2 * i]->Initialize(m_suspensions[i]->GetRevolute(LEFT));
    m_brakes[2 * i + 1]->Initialize(m_suspensions[i]->GetRevolute(RIGHT));
  }

  // Initialize the antirollbar subsystems.
  for (unsigned int i = 0; i < m_antirollbars.size(); i++) {
    int j = m_arbSuspension[i];
    m_antirollbars[i]->Initialize(m_chassis, m_arbLocations[i],
                                  m_suspensions[j]->GetLeftBody(),
                                  m_suspensions[j]->GetRightBody());
  }

  // Initialize the driveline
  m_driveline->Initialize(m_chassis, m_suspensions, m_driven_susp);
}


} // end namespace chrono
