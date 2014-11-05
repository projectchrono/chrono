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
// Authors: Justin Madsen, Radu Serban
// =============================================================================
//
// Suspension tester from a JSON specification file for the steering and suspension
//
// =============================================================================

#include <cstdio>

#include "assets/ChCylinderShape.h"
#include "assets/ChTexture.h"
#include "assets/ChColorAsset.h"
#include "physics/ChGlobal.h"

#include "subsys/suspensionTest/SuspensionTest.h"

#include "subsys/suspension/DoubleWishbone.h"
#include "subsys/suspension/DoubleWishboneReduced.h"
#include "subsys/suspension/SolidAxle.h"
#include "subsys/suspension/MultiLink.h"

#include "subsys/steering/PitmanArm.h"
#include "subsys/steering/RackPinion.h"
#include "subsys/wheel/Wheel.h"

#include "utils/ChUtilsInputOutput.h"
#include "utils/ChUtilsData.h"

#include "rapidjson/document.h"
#include "rapidjson/filereadstream.h"

using namespace rapidjson;

namespace chrono {


// -----------------------------------------------------------------------------
// These utility functions return a ChVector and a ChQuaternion<>, respectively,
// from the specified JSON array.
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
void SuspensionTest::LoadSteering(const std::string& filename)
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
void SuspensionTest::LoadSuspension(const std::string& filename,
                             int                axle,
                             bool               driven)
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
void SuspensionTest::LoadWheel(const std::string& filename, int axle, int side)
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


// ----------------------------------------------------------
SuspensionTest::SuspensionTest(const std::string& filename): m_num_axles(1)
{
  // Open and parse the input file
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

  // Create the chassis body, no visualizastion
  m_chassis = ChSharedPtr<ChBodyAuxRef>(new ChBodyAuxRef);

  m_chassisMass = d["Chassis"]["Mass"].GetDouble();
  m_chassisCOM = loadVector(d["Chassis"]["COM"]);
  m_chassisInertia = loadVector(d["Chassis"]["Inertia"]);

  m_chassis->SetIdentifier(0);
  m_chassis->SetName("chassis");
  m_chassis->SetMass(m_chassisMass);
  m_chassis->SetFrame_COG_to_REF(ChFrame<>(m_chassisCOM, ChQuaternion<>(1, 0, 0, 0)));
  m_chassis->SetInertiaXX(m_chassisInertia);
  // suspension test mechanism isn't going anywhere
  m_chassis->SetBodyFixed(true);

  ChSharedPtr<ChSphereShape> sphere(new ChSphereShape);
  sphere->GetSphereGeometry().rad = 0.1;
  sphere->Pos = m_chassisCOM;
  m_chassis->AddAsset(sphere);

  Add(m_chassis);

  // ---------------------------------
  // More validations of the JSON file
  assert(d.HasMember("Steering"));
  assert(d.HasMember("Driveline"));
  assert(d.HasMember("Axles"));
  assert(d["Axles"].IsArray());

  // Resize arrays, 1 suspension unit only
  m_suspensions.resize(1);
  m_suspLocations.resize(1);
  m_wheels.resize(2 * m_num_axles);

  // Array of flags for driven suspensions.
  std::vector<bool> driven(m_num_axles, false);

  // -----------------------------
  // Create the steering subsystem
  {
    std::string file_name = d["Steering"]["Input File"].GetString();
    LoadSteering(utils::GetModelDataFile(file_name));
    m_steeringLoc = loadVector(d["Steering"]["Location"]);
    m_steeringRot = loadQuaternion(d["Steering"]["Orientation"]);
    m_steer_susp = d["Steering"]["Suspension Index"].GetInt();
  }

  // ---------------------------------------------------
  // Create the suspension, wheel, and brake subsystems.
  // Suspension
  std::string file_name = d["Axles"][0u]["Suspension Input File"].GetString();
  LoadSuspension(utils::GetModelDataFile(file_name), 0, driven[0]);
  m_suspLocations[0] = loadVector(d["Axles"][0u]["Suspension Location"]);

// Left and right wheels
  file_name = d["Axles"][0u]["Left Wheel Input File"].GetString();
  LoadWheel(utils::GetModelDataFile(file_name), 0, 0);
  file_name = d["Axles"][0u]["Right Wheel Input File"].GetString();
  LoadWheel(utils::GetModelDataFile(file_name), 0, 1);

  // -----------------------
  // Extract driver position
  m_driverCsys.pos = loadVector(d["Driver Position"]["Location"]);
  m_driverCsys.rot = loadQuaternion(d["Driver Position"]["Orientation"]);

  // -----------------------
  // add the shaker posts, Left then right. 
  m_post_height = 0.1;
  m_post_rad = 0.4;
  double wheel_rad = m_wheels[LEFT]->GetRadius();
  m_post_L = ChSharedPtr<ChBody>(new ChBody());

  // DEBUGGING, check the radius we get
  GetLog () << "\n\n &^#%$&^#$%^&#$% \n\n wheel radius, according to Suspension test : " 
    << wheel_rad << "\n\n";
  // Cylinders for left post visualization
  AddVisualize_post(m_post_L, m_post_height, m_post_rad);
  Add(m_post_L);  // add left post body to System

  // TODO:
  // constrain L post to only translate vetically


  m_post_R = ChSharedPtr<ChBody>(new ChBody());
  

  // Cylinder for visualization of right post
  AddVisualize_post(m_post_R, m_post_height, m_post_rad);
  Add(m_post_R);  // add right post body to System

  // TODO:
  // constrain R post to only translate vetically

}


SuspensionTest::~SuspensionTest()
{
}

// -----------------------------------------------------------------------------
void SuspensionTest::Initialize(const ChCoordsys<>& chassisPos)
{
  m_chassis->SetFrame_REF_to_abs(ChFrame<>(chassisPos));

  // Initialize the steering subsystem. ChPhysicsItem has a virtual destructor, should be able to cast to ChBody
  m_steering->Initialize(m_chassis, m_steeringLoc, m_steeringRot);

  // Initialize the suspension subsys
  m_suspensions[0]->Initialize(m_chassis, m_suspLocations[0], m_steering->GetSteeringLink());

  // initialize the two wheels
  m_wheels[0]->Initialize(m_suspensions[0]->GetSpindle(LEFT));
  m_wheels[1]->Initialize(m_suspensions[0]->GetSpindle(RIGHT));

  // initialize the posts relative to the wheels
  // left side post
  ChVector<> post_L_pos = m_suspensions[0]->GetSpindlePos(LEFT);
  post_L_pos.z -= m_wheels[LEFT]->GetRadius() + m_post_height/2.0;  // shift down
  m_post_L->SetPos(post_L_pos);

  // test a spherical joint, above and to the side of the CM of the left post. should act like
  // a massless link, since it's attached to the ground
  ChSharedPtr<ChLinkLockSpherical> m_test_sph_j(new ChLinkLockSpherical);
  m_test_sph_j->Initialize(m_chassis, m_post_L, ChCoordsys<>(ChVector<>(post_L_pos.x+.2, post_L_pos.y, post_L_pos.z+.7),QUNIT) );
  AddLink(m_test_sph_j);
  
  // right side post
  ChVector<> post_R_pos = m_suspensions[0]->GetSpindlePos(RIGHT);
  post_R_pos.z -= m_wheels[RIGHT]->GetRadius() + m_post_height/2.0; // shift down
  m_post_R->SetPos(post_R_pos);

}


// -----------------------------------------------------------------------------
void SuspensionTest::Update(double       time,
                     double              steering,
                     double              disp_L,
                     double              disp_R,
                     const ChTireForces& tire_forces)
{
 
  // Let the steering subsystem process the steering input.
  m_steering->Update(time, steering);

  // Apply the displacements to the left/right posts
  

  // Apply tire forces to spindle bodies.
  m_suspensions[0]->ApplyTireForce(LEFT, tire_forces[0]);
  m_suspensions[0]->ApplyTireForce(RIGHT, tire_forces[1]);

}


void SuspensionTest::AddVisualize_post(ChSharedBodyPtr post_body,
                                double height,
                                double rad)
{
  // post platform visualized as a cylinder
  ChSharedPtr<ChCylinderShape> left_cyl(new ChCylinderShape);
  left_cyl->GetCylinderGeometry().rad = rad;
  left_cyl->GetCylinderGeometry().p1 = ChVector<>(0,0,height/2.0);
  left_cyl->GetCylinderGeometry().p2 = ChVector<>(0,0,-height/2.0);

  post_body->AddAsset(left_cyl); // add geometry asset to left post body

  ChSharedPtr<ChColorAsset> col(new ChColorAsset);
  col->SetColor(ChColor(0.4f, 0.2f, 0.6f));
  post_body->AddAsset(col);
}

} // end namespace chrono
