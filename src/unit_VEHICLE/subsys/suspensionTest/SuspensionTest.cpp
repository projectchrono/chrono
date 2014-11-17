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

#include "subsys/suspensionTest/SuspensionTest.h"

#include "assets/ChCylinderShape.h"
#include "assets/ChTexture.h"
#include "assets/ChColorAsset.h"
#include "physics/ChGlobal.h"

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

static const double in2m = 0.0254;
static const double lb2kg = 0.453592;
static const double lbf2N = 4.44822162;
static const double rad2deg = 180.0/CH_C_PI;

// Utility functions
// -----------------------------------------------------------------------------
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


// ----------------------------------------------------------
// Constructor guaranteers that <ChBody> objects are Added to the system here
// Links are added to the system during Initialize()
SuspensionTest::SuspensionTest(const std::string& filename): 
  m_num_axles(1), m_save_log_to_file(false), m_log_file_exists(false), m_log_what(0)
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

  // Create the ground body, no visualizastion
  m_ground = ChSharedPtr<ChBodyAuxRef>(new ChBodyAuxRef);

  m_groundCOM = ChVector<>(0,0,0);

  m_ground->SetIdentifier(0);
  m_ground->SetName("ground");
  m_ground->SetMass(1000.0);
  m_ground->SetFrame_COG_to_REF(ChFrame<>(m_groundCOM, ChQuaternion<>(1, 0, 0, 0)));
  m_ground->SetInertiaXX(ChVector<>(10,10,10));
  // suspension test mechanism isn't going anywhere
  m_ground->SetBodyFixed(true);

  ChSharedPtr<ChSphereShape> sphere(new ChSphereShape);
  sphere->GetSphereGeometry().rad = 0.1;
  sphere->Pos = m_groundCOM;
  m_ground->AddAsset(sphere);  // add asset sphere to chassis body

  ChSharedPtr<ChColorAsset> blue(new ChColorAsset);
  blue->SetColor(ChColor(0.2f, 0.2f, 0.8f));
  m_ground->AddAsset(blue);  // add asset color to chassis body

  Add(m_ground); // add chassis body to the system

  // ---------------------------------
  // More validations of the JSON file
  // assert(d.HasMember("Steering"));
  assert(d.HasMember("Suspension"));


  // Resize arrays, 1 suspension unit only
  m_wheels.resize(2 * m_num_axles);

  // -----------------------------
  // Create the steering subsystem, if specified
  if(d.HasMember("Steering") )
  {
    std::string file_name = d["Steering"]["Input File"].GetString();
    LoadSteering(utils::GetModelDataFile(file_name));
    m_steeringLoc = loadVector(d["Steering"]["Location"]);
    m_steeringRot = loadQuaternion(d["Steering"]["Orientation"]);
    m_steer_susp = d["Steering"]["Suspension Index"].GetInt();
    m_has_steering = true;
  }

  // ---------------------------------------------------
  // Create the suspension, wheel, and brake subsystems.
  // Suspension
  std::string file_name = d["Suspension"]["Input File"].GetString();
  LoadSuspension(utils::GetModelDataFile(file_name), 0, false);
  m_suspLocation = loadVector(d["Suspension"]["Location"]);

// Left and right wheels
  file_name = d["Suspension"]["Left Wheel Input File"].GetString();
  LoadWheel(utils::GetModelDataFile(file_name), 0, 0);
  file_name = d["Suspension"]["Right Wheel Input File"].GetString();
  LoadWheel(utils::GetModelDataFile(file_name), 0, 1);

  // -----------------------
  // Extract driver position
  if( m_has_steering)
  {
    m_driverCsys.pos = loadVector(d["Driver Position"]["Location"]);
    m_driverCsys.rot = loadQuaternion(d["Driver Position"]["Orientation"]);
  }

  // -----------------------
  // add the shaker posts, Left then right. 
  m_post_height = 0.1;
  m_post_rad = 0.4;

  // left post body
  m_post_L = ChSharedPtr<ChBody>(new ChBody());

  // Cylinders for left post visualization
  // AddVisualize_post(m_post_L, m_chassis, m_post_height, m_post_rad);
  Add(m_post_L);  // add left post body to System

  // constrain L post to only translate vetically
  m_post_L_prismatic = ChSharedPtr<ChLinkLockPrismatic>(new ChLinkLockPrismatic);
  m_post_L_prismatic->SetNameString("L_post_prismatic");
  // actutate L post
  m_post_L_linact = ChSharedPtr<ChLinkLinActuator>(new ChLinkLinActuator);
  m_post_L_linact->SetNameString("L_post_linActuator");

  // right post body
  m_post_R = ChSharedPtr<ChBody>(new ChBody());
  // Cylinder for visualization of right post
  // AddVisualize_post(m_post_R, m_chassis, m_post_height, m_post_rad);
  Add(m_post_R);  // add right post body to System

  // constrain R post to only translate vetically
  m_post_R_prismatic = ChSharedPtr<ChLinkLockPrismatic>(new ChLinkLockPrismatic);
  m_post_R_prismatic->SetNameString("R_post_prismatic");
  // actuate R post
  m_post_R_linact = ChSharedPtr<ChLinkLinActuator>(new ChLinkLinActuator);
  m_post_R_linact->SetNameString("R_post_linActuator");

  // left post point on plane
  m_post_L_ptPlane = ChSharedPtr<ChLinkLockPointPlane>(new ChLinkLockPointPlane());
  m_post_L_ptPlane->SetNameString("L_post_pointPlane");
  
  // right post point on plane
  m_post_R_ptPlane = ChSharedPtr<ChLinkLockPointPlane>(new ChLinkLockPointPlane());
  m_post_R_ptPlane->SetNameString("R_post_pointPlane");
}


SuspensionTest::~SuspensionTest()
{
}

// -----------------------------------------------------------------------------
// <ChBody> objects should have already been added to the system.
// Links need to be added to the system here.
void SuspensionTest::Initialize(const ChCoordsys<>& chassisPos)
{
  // Initialize the steering subsystem. ChPhysicsItem has a virtual destructor, should be able to cast to ChBody
  m_steering->Initialize(m_ground, m_steeringLoc, m_steeringRot);

  // Initialize the suspension subsys
  m_suspension->Initialize(m_ground, m_suspLocation, m_steering->GetSteeringLink());

  // initialize the two wheels
  m_wheels[0]->Initialize(m_suspension->GetSpindle(LEFT));
  m_wheels[1]->Initialize(m_suspension->GetSpindle(RIGHT));

  // initialize the posts relative to the wheels
  // left side post
  ChVector<> spindle_L_pos = m_suspension->GetSpindlePos(LEFT);
  ChVector<> post_L_pos = spindle_L_pos;
  post_L_pos.z -= (m_wheels[LEFT]->GetRadius() + m_post_height/2.0);  // shift down
  m_post_L->SetPos(post_L_pos);

  // constrain left post to vertical. Prismatic default aligned to z-axis
  m_post_L_prismatic->Initialize(m_ground, m_post_L, ChCoordsys<>(ChVector<>(post_L_pos), QUNIT) );
  AddLink(m_post_L_prismatic);
  
  // actuate the L post DOF with a linear actuator in the vertical direction.
  // body 2 is the post, so this link will be w.r.t. marker on that body
  ChVector<> m1_L = post_L_pos;
  m1_L.z -= 1.0;    // offset marker 1 location 1 meter below marker 2
  m_post_L_linact->Initialize(m_ground, m_post_L, false, ChCoordsys<>(m1_L,QUNIT), ChCoordsys<>(post_L_pos,QUNIT) );
  m_post_L_linact->Set_lin_offset( (post_L_pos - m1_L).z );
  // displacement motion set as a constant function
  ChSharedPtr<ChFunction_Const> func_L(new ChFunction_Const(0));
  m_post_L_linact->Set_dist_funct( func_L );
  AddLink(m_post_L_linact);

  // right side post
  ChVector<> spindle_R_pos = m_suspension->GetSpindlePos(RIGHT);
  ChVector<> post_R_pos = spindle_R_pos;
  post_R_pos.z -= (m_wheels[RIGHT]->GetRadius() + m_post_height/2.0); // shift down
  m_post_R->SetPos(post_R_pos);

  // constrain right post to vertical.
  m_post_R_prismatic->Initialize(m_ground, m_post_R, ChCoordsys<>(ChVector<>(post_R_pos), QUNIT) );
  AddLink(m_post_R_prismatic);

  // actuate the R post DOF with a linear actuator in the vertical direction
  ChVector<> m1_R = post_R_pos;
  m1_R.z -= 1.0;    // offset marker 1 location 1 meter below marker 2
  m_post_R_linact->Initialize(m_ground, m_post_R, false, ChCoordsys<>(m1_R,QUNIT), ChCoordsys<>(post_R_pos,QUNIT) );
  m_post_R_linact->Set_lin_offset( (post_R_pos - m1_R).z );
  // displacement motion set as a constant function
  ChSharedPtr<ChFunction_Const> func_R(new ChFunction_Const(0));
  m_post_R_linact->Set_dist_funct( func_R );
  AddLink(m_post_R_linact);

  // keep the suspension at the specified height by keeping a point on the spindle
  // body on a plane whose height is based on the shaker post.
  m_post_L_ptPlane->Initialize(m_suspension->GetSpindle(LEFT), m_post_L, ChCoordsys<>(spindle_L_pos, QUNIT));
  AddLink(m_post_L_ptPlane);

  // right post point on plane
  m_post_R_ptPlane->Initialize(m_suspension->GetSpindle(RIGHT), m_post_R, ChCoordsys<>(spindle_R_pos, QUNIT));
  AddLink(m_post_R_ptPlane);

  // some visualizations.
  // left post: Green
  AddVisualize_post(m_post_L, m_ground, m_post_height, m_post_rad);
  // right post: Red
  AddVisualize_post(m_post_R, m_ground, m_post_height, m_post_rad, ChColor(0.8f,0.1f,0.1f));

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

  // Apply the displacements to the left/right post actuators
  if( ChSharedPtr<ChFunction_Const> func_L = m_post_L_linact->Get_dist_funct().DynamicCastTo<ChFunction_Const>() )
    func_L->Set_yconst(disp_L);
  if( ChSharedPtr<ChFunction_Const> func_R = m_post_R_linact->Get_dist_funct().DynamicCastTo<ChFunction_Const>() )
    func_R->Set_yconst(disp_R);

  // Apply tire forces to spindle bodies.
  m_suspension->ApplyTireForce(LEFT, tire_forces[0]);
  m_suspension->ApplyTireForce(RIGHT, tire_forces[1]);

  m_steer = steering;
  m_postDisp[LEFT] = disp_L;
  m_postDisp[RIGHT] = disp_R;
}


// -----------------------------------------------------------------------------
void SuspensionTest::Update(double       time,
                     double              disp_L,
                     double              disp_R,
                     const ChTireForces& tire_forces)
{

  // Apply the displacements to the left/right post actuators
  if( ChSharedPtr<ChFunction_Const> func_L = m_post_L_linact->Get_dist_funct().DynamicCastTo<ChFunction_Const>() )
    func_L->Set_yconst(disp_L);
  if( ChSharedPtr<ChFunction_Const> func_R = m_post_R_linact->Get_dist_funct().DynamicCastTo<ChFunction_Const>() )
    func_R->Set_yconst(disp_R);

  // Apply tire forces to spindle bodies.
  m_suspension->ApplyTireForce(LEFT, tire_forces[0]);
  m_suspension->ApplyTireForce(RIGHT, tire_forces[1]);

  m_postDisp[LEFT] = disp_L;
  m_postDisp[RIGHT] = disp_R;
}

// set what to save to file each time .DebugLog() is called during the simulation loop
// creates a new file (or overwrites old existing one), and sets the first row w/ headers
// for easy postprocessing with python pandas scripts
void SuspensionTest::Save_DebugLog(int what,
                                   const std::string& filename)
{
  m_log_file_name = filename;
  m_save_log_to_file = true;
  m_log_what = what;
  
  create_fileHeader(what);
  m_log_file_exists = true;

  // initialize the rig input values to zero
  m_steer = 0;
  m_postDisp[0] = m_postDisp[1] = 0;
}


void SuspensionTest::DebugLog(int console_what)
{
  GetLog().SetNumFormat("%10.2f");

  if (console_what & DBG_SPRINGS)
  {
    GetLog() << "\n---- Spring (left, right)\n";
    GetLog() << "Length [inch]       "
      << GetSpringLength(FRONT_LEFT) / in2m << "  "
      << GetSpringLength(FRONT_RIGHT) / in2m << "\n";
    GetLog() << "Deformation [inch]  "
      << GetSpringDeformation(FRONT_LEFT) / in2m << "  "
      << GetSpringDeformation(FRONT_RIGHT) / in2m << "\n";
    GetLog() << "Force [lbf]         "
      << GetSpringForce(FRONT_LEFT) / lbf2N << "  "
      << GetSpringForce(FRONT_RIGHT) / lbf2N << "\n";
  }

  if (console_what & DBG_SHOCKS)
  {
    GetLog() << "\n---- Shock (left, right,)\n";
    GetLog() << "Length [inch]       "
      << GetShockLength(FRONT_LEFT) / in2m << "  "
      << GetShockLength(FRONT_RIGHT) / in2m << "\n";
    GetLog() << "Velocity [inch/s]   "
      << GetShockVelocity(FRONT_LEFT) / in2m << "  "
      << GetShockVelocity(FRONT_RIGHT) / in2m << "\n";
    GetLog() << "Force [lbf]         "
      << GetShockForce(FRONT_LEFT) / lbf2N << "  "
      << GetShockForce(FRONT_RIGHT) / lbf2N << "\n";
  }

  if (console_what & DBG_CONSTRAINTS)
  {
    // Report constraint violations for all joints
    LogConstraintViolations();
  }

  if (console_what & DBG_SUSPENSIONTEST)
  {
    GetLog() << "\n---- suspension test (left, right)\n";
    /*
    GetLog() << "Actuator Displacement [in] "
      << GetActuatorDisp(FRONT_LEFT) / in2m << "  "
      << GetActuatorDisp(FRONT_RIGHT) / in2m << "\n";
    GetLog() << "Actuator Force [N] "
      << GetActuatorForce(FRONT_LEFT) / in2m << "  "
      << GetActuatorForce(FRONT_RIGHT) / in2m << "\n";
    GetLog() << "Actuator marker dist [in] "
      << GetActuatorMarkerDist(FRONT_LEFT) / in2m << "  "
      << GetActuatorMarkerDist(FRONT_RIGHT) / in2m << "\n";
    */



//    GetLog() << "steer input: " << 
  
    
    GetLog() << "Kingpin angle [deg] "
      << Get_KingpinAng(LEFT) * rad2deg << "  "
      << Get_KingpinAng(RIGHT) * rad2deg << "\n";
    GetLog() << "Kingpin offset [in] "
      << Get_KingpinOffset(LEFT) / in2m << "  "
      << Get_KingpinOffset(RIGHT) / in2m << "\n";
    GetLog() << "Caster angle [deg] "
      << Get_CasterAng(LEFT) * rad2deg << "  "
      << Get_CasterAng(RIGHT) * rad2deg << "\n";
    GetLog() << "Caster offset [in] "
      << Get_CasterOffset(LEFT) / in2m << "  "
      << Get_CasterOffset(RIGHT) / in2m << "\n";
    GetLog() << "Toe angle [deg] "
      << Get_ToeAng(LEFT) * rad2deg << "  "
      << Get_ToeAng(RIGHT) * rad2deg << "\n";
    GetLog() << "suspension roll angle [deg] "
      << Get_LCArollAng() << "\n";
  }

  GetLog().SetNumFormat("%g");
}




void SuspensionTest::SaveLog()
{
  // told to save the data?
  if( m_save_log_to_file )
  {
    if( !m_log_file_exists ) 
    {
      std::cerr << "Must call Save_DebugLog() before trying to save the log data to file!!! \n\n\n";
    }
    // open the file to append
    // open the data file for writing the header
  ChStreamOutAsciiFile ofile(m_log_file_name.c_str(), std::ios::app);

    // write the simulation time first, and rig inputs
    std::stringstream ss;
    ss << GetChTime() <<","<< m_steer <<","<< m_postDisp[LEFT] /in2m <<","<< m_postDisp[RIGHT] / in2m;

    // python pandas expects csv w/ no whitespace
    if( m_log_what & DBG_SPRINGS )
    {
      ss << "," << GetSpringLength(FRONT_LEFT) / in2m << ","
        << GetSpringLength(FRONT_RIGHT) / in2m << ","
        << GetSpringDeformation(FRONT_LEFT) / in2m << ","
        << GetSpringDeformation(FRONT_RIGHT) / in2m << ","
        << GetSpringForce(FRONT_LEFT) / lbf2N << ","
        << GetSpringForce(FRONT_RIGHT) / lbf2N;
  
    }
    if (m_log_what & DBG_SHOCKS)
    {
      ss << "," << GetShockLength(FRONT_LEFT) / in2m << ","
        << GetShockLength(FRONT_RIGHT) / in2m << ","
        << GetShockVelocity(FRONT_LEFT) / in2m << ","
        << GetShockVelocity(FRONT_RIGHT) / in2m << ","
        << GetShockForce(FRONT_LEFT) / lbf2N << ","
        << GetShockForce(FRONT_RIGHT) / lbf2N;
    }

    if (m_log_what & DBG_CONSTRAINTS)
    {
      // Report constraint violations for all joints
      LogConstraintViolations();
    }
    
    // ",KA_L,KA_R,Koff_L,Koff_R,CA_L,CA_R,Coff_L,Coff_R,TA_L,TA_R,LCA_roll";
    if (m_log_what & DBG_SUSPENSIONTEST)
    {
      ss <<","<< Get_KingpinAng(LEFT)*rad2deg <<","<< Get_KingpinAng(RIGHT)*rad2deg 
        <<","<< Get_KingpinOffset(LEFT)/in2m <<","<< Get_KingpinOffset(RIGHT)/in2m
        <<","<< Get_CasterAng(LEFT)*rad2deg <<","<< Get_CasterAng(RIGHT)*rad2deg 
        <<","<< Get_CasterOffset(LEFT)/in2m <<","<< Get_CasterOffset(RIGHT)/in2m
        <<","<< Get_ToeAng(LEFT)*rad2deg <<","<< Get_ToeAng(RIGHT)*rad2deg
        <<","<< Get_LCArollAng();
      /*
      ss << "," << GetActuatorDisp(FRONT_LEFT) / in2m << ","
        << GetActuatorDisp(FRONT_RIGHT) / in2m << ","
        << GetActuatorForce(FRONT_LEFT) / in2m << ","
        << GetActuatorForce(FRONT_RIGHT) / in2m << ","
        << GetActuatorMarkerDist(FRONT_LEFT) / in2m << ","
        << GetActuatorMarkerDist(FRONT_RIGHT) / in2m;
       */
    }
    // next line last, then write to file
    ss << "\n";
    ofile << ss.str().c_str();
  }
}


// Public Accessors
// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
double SuspensionTest::GetSpringForce(const ChWheelID& wheel_id) const
{
  return m_suspension.StaticCastTo<ChDoubleWishbone>()->GetSpringForce(wheel_id.side());
}

double SuspensionTest::GetSpringLength(const ChWheelID& wheel_id) const
{
  return m_suspension.StaticCastTo<ChDoubleWishbone>()->GetSpringLength(wheel_id.side());
}

double SuspensionTest::GetSpringDeformation(const ChWheelID& wheel_id) const
{
  return m_suspension.StaticCastTo<ChDoubleWishbone>()->GetSpringDeformation(wheel_id.side());
}


double SuspensionTest::GetShockForce(const ChWheelID& wheel_id) const
{
  return m_suspension.StaticCastTo<ChDoubleWishbone>()->GetShockForce(wheel_id.side());
}

double SuspensionTest::GetShockLength(const ChWheelID& wheel_id) const
{
  return m_suspension.StaticCastTo<ChDoubleWishbone>()->GetShockLength(wheel_id.side());
}

double SuspensionTest::GetShockVelocity(const ChWheelID& wheel_id) const
{
  return m_suspension.StaticCastTo<ChDoubleWishbone>()->GetShockVelocity(wheel_id.side());
}


double SuspensionTest::GetActuatorDisp(const ChWheelID& wheel_id) const
{
  if(wheel_id.side() == LEFT)
    return m_post_L_linact.StaticCastTo<ChLinkLinActuator>()->Get_dist_funct()->Get_y(ChTime);
  else
    return m_post_R_linact.StaticCastTo<ChLinkLinActuator>()->Get_dist_funct()->Get_y(ChTime);
}

double SuspensionTest::GetActuatorForce(const chrono::ChWheelID& wheel_id)const
{
  if(wheel_id.side() == LEFT)
    return m_post_L_linact.StaticCastTo<ChLinkLinActuator>()->Get_react_force().x;
  else
    return m_post_R_linact.StaticCastTo<ChLinkLinActuator>()->Get_react_force().x;
}

double SuspensionTest::GetActuatorMarkerDist(const chrono::ChWheelID& wheel_id)const
{
  if(wheel_id.side() == LEFT)
    return m_post_L_linact.StaticCastTo<ChLinkLinActuator>()->GetDist();
  else
    return m_post_R_linact.StaticCastTo<ChLinkLinActuator>()->GetDist();
}

// -----------------------------------------------------------------------------
// Measurements to log for suspension test rig

// NOTE: should probably check to make sure the suspension type is compatable
/// suspension kingpin angle, in radians. Assume x-forward, z-up.
/// Sets m_KA[side]
double SuspensionTest::Get_KingpinAng(const chrono::ChVehicleSide side)
{
  // global coordinates
  ChVector<> UCA_bj_pos = m_suspension.DynamicCastTo<ChDoubleWishbone>()->Get_UCA_sph_pos(side);
  ChVector<> LCA_bj_pos = m_suspension.DynamicCastTo<ChDoubleWishbone>()->Get_LCA_sph_pos(side);
  // kingpin direction vector, global coords
  ChVector<> k_hat = (UCA_bj_pos - LCA_bj_pos).GetNormalized();

  // front view of things, (no x-component)
  k_hat.x = 0;
  k_hat.Normalize();

  // cos(KA) = (k_hat) dot (0,0,1)T
  double KA = std::acos(k_hat.z);

  // KA is positive when the top of wheel points towards vehicle center.
  int sign_y = k_hat >= 0 ? 1: -1;  // is k_hat.y pos. or neg.?
  // on the left, +y indicates a 
  if(side == LEFT) {
    KA *= -sign_y;
  } else {
    KA *= sign_y;
  }

    

  m_KA[side] = KA;
  return KA;
}

/// suspension kingpin offset, in meters. Assume x-forward, z-up.
/// Sets m_Koffset[side]
/// Assumes you already called Get_KingpinAng(side);
double SuspensionTest::Get_KingpinOffset(const chrono::ChVehicleSide side)
{
  // global coordinates
  ChVector<> LCA_bj_pos = m_suspension.DynamicCastTo<ChDoubleWishbone>()->Get_LCA_sph_pos(side);
 
  /*
  ChVector<> UCA_bj_pos = m_suspension.DynamicCastTo<ChDoubleWishbone>()->Get_UCA_sph_pos(side);
  // kingpin direction vector, global coords
  ChVector<> k_hat = (UCA_bj_pos - LCA_bj_pos).GetNormalized();

  // front view of things, (no x-component)
  k_hat.x = 0;
  k_hat.Normalize();
  double KA = std::acos(k_hat.z);
  double k_g = LCA_bj_pos.y + std::tan(KA) * LCA_bj_pos.z;
  */
  int sign_KA = 1;
  if(side == RIGHT )
    sign_KA = -1;

  double k_g = LCA_bj_pos.y + std::tan(sign_KA * m_KA[side]) * (LCA_bj_pos.z - GetPostSurfacePos(side).z );

  // wheel y-displacement from vehicle centerline
  double y_wheel = m_suspension->GetSpindlePos(side).y;

  double KP_offset = (y_wheel - k_g) * sign_KA;

  m_Koffset[side] = KP_offset;
  return KP_offset;
}

/// suspension wheel caster angle. Similar to kingpin angle, but in X-Z plane.
/// Sets m_CA[side]
double SuspensionTest::Get_CasterAng(const chrono::ChVehicleSide side)
{
  // global coordinates
  ChVector<> UCA_bj_pos = m_suspension.DynamicCastTo<ChDoubleWishbone>()->Get_UCA_sph_pos(side);
  ChVector<> LCA_bj_pos = m_suspension.DynamicCastTo<ChDoubleWishbone>()->Get_LCA_sph_pos(side);
  // kingpin direction vector, global coords
  ChVector<> c_hat = (UCA_bj_pos - LCA_bj_pos).GetNormalized();

  // side view, (no y-component)
  c_hat.y = 0;
  c_hat.Normalize();

  // cos(CA) = (c_hat) dot (0,0,1)T
  double c_ang = std::acos(c_hat.z);
  // positive caster angle has the UCA rear-ward of the LCA
  if( c_hat.x > 0 )
    c_ang = -c_ang;

  m_CA[side] = c_ang;
  return c_ang;
}


/// suspension wheel caster offset.
/// Sets m_Coffset[side]
/// assumes you already called Get_CasterAng[side]
double SuspensionTest::Get_CasterOffset(const chrono::ChVehicleSide side)
{
  ChVector<> LCA_bj_pos = m_suspension.DynamicCastTo<ChDoubleWishbone>()->Get_LCA_sph_pos(side);
  
  // wheel x-displacement from vehicle centerline
  double x_wheel = m_suspension->GetSpindlePos(side).x;

  double c_g = LCA_bj_pos.x + std::tan(m_CA[side]) * (LCA_bj_pos.z - GetPostSurfacePos(side).z );

  double offset = c_g - x_wheel;
  return offset;

}

/// Wheel spindle toe angle
double SuspensionTest::Get_ToeAng(const chrono::ChVehicleSide side)
{
  // y_hat should be parallel to the ground
  // get it by crossing x_hat by z_hat, parallel and perpendicular to road surface, respectively
  ChVector<> x_hat = m_suspension->GetSpindleRot(side).GetXaxis();
  x_hat.z = 0;
  x_hat.Normalize();
  ChVector<> y_hat = ChVector<>(0,0,1) % x_hat;
 

  int sign_TA = 1;
  if(side == LEFT)
  {
    // on the left side, pos. toe has a neg. y component for x_hat
    if(x_hat.y > 0)
      sign_TA = -1;
  } else {

    // on the right, pos. toe angle has pos. y component for x_hat
    if(x_hat.y < 0)
      sign_TA = -1;
  }

  // cos(TA) = x_hat_wheel dot (1,0,0)T, TA always calculated as positive here
  double TA = std::acos(x_hat.x) * sign_TA;

  ChVector<> xx = m_suspension->GetSpindle(side)->GetRot().GetXaxis();
  ChVector<> yy = m_suspension->GetSpindle(side)->GetRot().GetYaxis();
  ChVector<> zz = m_suspension->GetSpindle(side)->GetRot().GetZaxis();

  m_TA[side] = TA;
  return TA;
}

// get the roll angle, defined as the angle between CM of two LCAs w/ the horizontal plane
double SuspensionTest::Get_LCArollAng()
{

  double ang = 0;
  return 0;
}

// global center of the post surface
ChVector<> SuspensionTest::GetPostSurfacePos(const chrono::ChVehicleSide& side)
{
  ChVector<> pos;
  if(side == LEFT)
    pos = m_post_L->GetPos();
  else
    pos = m_post_R->GetPos();

  pos.z += m_post_height/2.0;

  return pos;
}

// Private class functions
// -----------------------------------------------------------------------------
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
    m_suspension = ChSharedPtr<ChSuspension>(new DoubleWishbone(d));
  }
  else if (subtype.compare("DoubleWishboneReduced") == 0)
  {
    m_suspension = ChSharedPtr<ChSuspension>(new DoubleWishboneReduced(d));
  }
  else if (subtype.compare("SolidAxle") == 0)
  {
    m_suspension = ChSharedPtr<ChSuspension>(new SolidAxle(d));
  }
  else if (subtype.compare("MultiLink") == 0)
  {
    m_suspension = ChSharedPtr<ChSuspension>(new MultiLink(d));
  }
}


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


void SuspensionTest::AddVisualize_post(ChSharedBodyPtr post_body,
                                       ChSharedBodyPtr ground_body,
                                       double height,
                                       double rad,
                                       const ChColor& color)
{
  // post platform visualized as a cylinder
  ChSharedPtr<ChCylinderShape> base_cyl(new ChCylinderShape);
  base_cyl->GetCylinderGeometry().rad = rad;
  base_cyl->GetCylinderGeometry().p1 = ChVector<>(0,0,height/2.0);
  base_cyl->GetCylinderGeometry().p2 = ChVector<>(0,0,-height/2.0);

  post_body->AddAsset(base_cyl); // add geometry asset to post body

  // actuator would be a vertical piston-cylinder
  ChSharedPtr<ChCylinderShape> piston(new ChCylinderShape);
  piston->GetCylinderGeometry().rad = rad/6.0;
  piston->GetCylinderGeometry().p1 = ChVector<>(0,0,-height/2.0);
  piston->GetCylinderGeometry().p2 = ChVector<>(0,0,-height*8.0);
  post_body->AddAsset(piston);  // add asset to post body

  // ground body
  ChSharedPtr<ChCylinderShape> cyl(new ChCylinderShape);
  cyl->GetCylinderGeometry().rad = rad/4.0;

  ChVector<> p1 = post_body->GetPos();
  p1.z -=height*8.0;
  ChVector<> p2 = p1;
  p2.z -= height*8.0;
  cyl->GetCylinderGeometry().p1 = p1;
  cyl->GetCylinderGeometry().p2 = p2;
  ground_body->AddAsset(cyl);  // add asset to ground body

  // give the post a color
  ChSharedPtr<ChColorAsset> col(new ChColorAsset);
  col->SetColor(color);
  post_body->AddAsset(col); // add asset color to post body
}


void SuspensionTest::create_fileHeader(int what)
{
  // open the data file for writing the header
  ChStreamOutAsciiFile ofile(m_log_file_name.c_str());
  // write the headers, output types specified by "what"
  std::stringstream ss;
  ss << "time,steer,postDisp_L,postDisp_R";
  if(what & DBG_SPRINGS)
  {
    // L/R spring length, delta x, force
    ss << ",k_len_L,k_len_R,k_dx_L,k_dx_R,k_F_L,k_F_R";
  }
  if(what & DBG_SHOCKS)
  {
    ss << ",d_len_L,d_len_R,d_vel_L,d_vel_R,d_F_L,d_F_R";
  }
  if(what & DBG_CONSTRAINTS)
  {
    // TODO:
  }
  if(what & DBG_SUSPENSIONTEST)
  {
    ss << ",KA_L,KA_R,Koff_L,Koff_R,CA_L,CA_R,Coff_L,Coff_R,TA_L,TA_R,LCA_roll";
  }

  // write to file, go to next line in file in prep. for next step.
  ofile << ss.str().c_str();
  ofile << "\n";
}


} // end namespace chrono
     