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
// Authors: Justin Madsen
// =============================================================================
//
// Main mechanism is a single body slider with a ChLinkLockRevolutePrismatic.
//  Body rotates about z-axis, lateral and orthogonal to direction of translation.
// Second mechanism is broken into a body two constraint mechanism, to mimic the other joint.
// In both cases, the rotating body is attached to a weight and spring, offset from the COM in the x- and z- dirs.
// Resulting motion should see the body translate and rotate in a oscillatory manner about the expected x- and z-axes, respectively.
//
// The global reference frame is Y up.
// =============================================================================

#include <stdio.h>
#include <cmath>

#include "core/ChFileutils.h"
#include "core/ChStream.h"
#include "physics/ChBodyEasy.h"
#include "physics/ChSystem.h"

#include "utils/ChUtilsCreators.h"
#include "utils/ChUtilsInputOutput.h"

#include <irrlicht.h>
#include "unit_IRRLICHT/ChIrrApp.h"

using namespace chrono;
using namespace irr;

// -----------------------------------------------------------------------------
// Model Parameters
const double wheel_rad = 0.3;
const double wheel_wid = 0.2;
const double wheel_density = 1000;  // [kg/m3]

const double weight_side_len = 0.1;
const double weight_density = 2500;
const ChVector<> weight_offset = ChVector<>(1.0, 0, 0.3);  // relative to the other body

const double spring_K = 1500;  // [N/m]
const double spring_C = 15;   // [N-s/m]

const double test_system_spacing = 2.0; // lateral offset distance to place the test bodies from the origin

// -----------------------------------------------------------------------------
// Simulation parameters

// test the two bodies on the following state levels:
const bool compare_pos = false; 
const bool compare_vel = true;
const bool compare_acc = false;

// Magnitude of the gravitational acceleration
const double gravity = 9.81;

// Simulation time, integration time-step, and output FPS
// const double time_end = 4.0;
const double time_step = 0.001;
const double out_fps = 10;
const double render_fps = 50;

// Name of the output folder
const std::string out_dir = "../SLIDER_CRANK";

// Name of the subfolder for PovRay output files (one file per output frame)
const std::string pov_dir = out_dir + "/POVRAY";

// Name of the output file with body positions (one line per output frame)
const std::string out_file = out_dir + "/output.txt";

// -----------------------------------------------------------------------------
// Append a new line to the output file, containing body positions and
// orientations at the current time.
// -----------------------------------------------------------------------------
void OutputData(ChSystem*             system,
                ChStreamOutAsciiFile& ofile,
                double                time)
{
  ofile << time << ", ";

  for (int i = 0; i < system->Get_bodylist()->size(); i++) {
    ChBody* body = (ChBody*) system->Get_bodylist()->at(i);

    if (!body->IsActive())
      continue;

    const ChVector<>& pos = body->GetPos();
    const ChQuaternion<>& rot = body->GetRot();

    ofile << pos.x  << ", " << pos.y  << ", " << pos.z  << ", ";
    ofile << rot.e0 << ", " << rot.e1 << ", " << rot.e2 << ", " << rot.e3 << ", ";
  }

  ofile << "\n";
}

// test the difference between the two bodies on the specified levels.
// return true if all the tolerances are met
// return false if they are exceed
bool run_test(double time, ChSharedPtr<ChBody> b1, ChSharedPtr<ChBody> b2, bool to_console = false)
{
  // fail test if the infinity norm of any of the difference vectors is above allowed tolerance.
  // what are the allowable tolerances?
  double tol_pos = 1e-6;
  double tol_vel = 1e-4;
  double tol_acc = 1e-3;

  // compare the states of the two bodies
  ChVector<> dx = b2->GetPos() - b1->GetPos();
  ChVector<> d_ang = Q_to_NasaAngles(b2->GetRot()) - Q_to_NasaAngles(b1->GetRot()); 
  ChVector<> dv = b2->GetPos_dt() - b1->GetPos_dt();
  ChVector<> dw = b2->GetWvel_loc() - b1->GetWvel_loc();
  ChVector<> da = b2->GetPos_dtdt() - b1->GetPos_dtdt();
  ChVector<> d_alpha = b2->GetWacc_loc() - b1->GetWacc_loc();
  
  // print difference to console, if enabled
  if(to_console)
  {
    if(compare_pos)
    {
      GetLog() << "position difference: " << " \n";
      GetLog() << " x:  " << dx.x << "\n y: " << dx.y << "\n z: " << dx.z << "\n";
      GetLog() << "AngAxis difference (attitude, bank, heading) : " << " \n";
      GetLog() << " x-axis:  " << d_ang.x << "\n y-axis: " << d_ang.y << "\n z-axis: " << d_ang.z << "\n";
    }

    if(compare_vel)
    {
     
      GetLog() << "velocity difference: " << " \n";
      GetLog() << " v_x:  " << dv.x << "\n v_y: " << dv.y << "\n v_z: " << dv.z << "\n";
      GetLog() << "rotationaly velocity difference: " << " \n";
      GetLog() << " w_x:  " << dw.x << "\n w_y: " << dw.y << "\n w_z: " << dw.z << "\n";
  
    }

    if(compare_acc)
    {
     
      GetLog() << "accel difference: " << " \n";
      GetLog() << " a_x:  " << da.x << "\n a_y: " << da.y << "\n a_z: " << da.z << "\n";
      GetLog() << "angular accel difference: " << " \n";
      GetLog() << " alpha_x:  " << d_alpha.x << "\n alpha_y: " << d_alpha.y << "\n alpha_z: " << d_alpha.z << "\n";
  
  
    }
  } // end to_console

  bool test_result = true;
  // test position
  if( dx.LengthInf() > tol_pos )
  {
    test_result = false;
    GetLog() << " position test failed, with inf. norm: " << dv.LengthInf() << "\n";
  }

  // test translational velocity
  if( dv.LengthInf() > tol_vel )
  {
    test_result = false;
    GetLog() << " velocity test failed, with inf. norm: " << dv.LengthInf() << "\n";
  }

  // test angular veloity
  if( dw.LengthInf() > tol_vel )
  {
    test_result = false;
    GetLog() << " angular velocity test failed, with inf. norm: " << dv.LengthInf() << "\n";
  }

  // test accel
  if( da.LengthInf() > tol_acc )
  {
    test_result = false;
    GetLog() << " accel test failed, with inf. norm: " << dv.LengthInf() << "\n";
  }
  
  return test_result;
}

// -----------------------------------------------------------------------------
// Create the two mechanical systems, define bodies, joints and springs.
// simulation.
int main(int   argc,
         char* argv[])
{
  // 0. Create output directories.
  if(ChFileutils::MakeDirectory(out_dir.c_str()) < 0) {
    std::cout << "Error creating directory " << out_dir << std::endl;
    return 1;
  }
  if(ChFileutils::MakeDirectory(pov_dir.c_str()) < 0) {
    std::cout << "Error creating directory " << pov_dir << std::endl;
    return 1;
  }

  ChStreamOutAsciiFile ofile(out_file.c_str());

  // 1. Create the physical system 
  //  gravity acts Y-down
  ChSystem system;
  system.Set_G_acc(ChVector<>(0, -gravity, 0));

  // 2. Create the two mechanisms, attach to a common ground body
  // 2a.  Ground
  ChSharedBodyPtr ground(new ChBody);
  system.AddBody(ground);
  ground->SetIdentifier(-1);
  ground->SetBodyFixed(true);
  ground->SetCollide(false);
  ground->GetCollisionModel()->ClearModel();
  utils::AddCylinderGeometry(ground.get_ptr(), 0.15, 0.1);
  ground->GetCollisionModel()->BuildModel();

  // 2b. System one: ChLinkLockRevolutePrismatic - 1 body, supposed to act like an idler
  ChSharedPtr<ChBodyEasyCylinder> idler(new ChBodyEasyCylinder(wheel_rad, wheel_wid, wheel_density));
  idler->SetName("idler body");
  idler->SetPos(ChVector<>(0, 0, -test_system_spacing));
  system.Add(idler);

  // make the idler body red
  ChSharedPtr<ChColorAsset> red_col(new ChColorAsset);
	red_col->SetColor(ChColor(0.9f,0.4f,0.2f));
  idler->AddAsset(red_col);

  // idler constraint: x-translate, z-rotate DOFs:
  ChSharedPtr<ChLinkLockRevolutePrismatic> revolutePrismatic(new ChLinkLockRevolutePrismatic);
  revolutePrismatic->SetName("idler constraint");
  revolutePrismatic->Initialize(idler, ground,  idler->GetCoord() );
  system.AddLink(revolutePrismatic);

  // attach the weight to the idler with a spring
  ChSharedPtr<ChBodyEasyBox> weight_idler(new ChBodyEasyBox(0.1, 0.1, 0.2, weight_density));
  weight_idler->SetPos( idler->GetPos() + weight_offset);
  weight_idler->SetName("weight_idler box body");
  system.Add(weight_idler);

  ChSharedPtr<ChLinkSpring> idler_spring(new ChLinkSpring);
  idler_spring->Initialize(weight_idler, idler, false, weight_idler->GetPos(), idler->GetPos() );
  idler_spring->Set_SpringK(spring_K);
  idler_spring->Set_SpringF(spring_C);
  idler_spring->SetName("idler spring");
  system.AddLink(idler_spring);

  // 2c. System two: use two bodies, and wheel and spindle, and two constraints, a revolute and prismatic.
  // wheel rotates and translates
  ChSharedPtr<ChBodyEasyCylinder> wheel(new ChBodyEasyCylinder(wheel_rad, wheel_wid, wheel_density));
  wheel->SetPos(ChVector<>(0, 0, test_system_spacing));
  wheel->SetName("wheel body");
  system.Add(wheel);
  // make the reference system blue
  ChSharedPtr<ChColorAsset> ref_col(new ChColorAsset);
	ref_col->SetColor(ChColor(0.2f,0.4f,0.9f));
  wheel->AddAsset(ref_col);

  // spindle transltes only
  ChSharedPtr<ChBodyEasyBox> spindle(new ChBodyEasyBox(0.1, 0.1, 0.2, wheel_density/20.0));
  spindle->SetPos(ChVector<>(0, 0, test_system_spacing));
  spindle->SetName("spindle box body");
  system.Add(spindle);

  // constraint the spindle to the ground
  ChSharedPtr<ChLinkLockPrismatic> prismatic(new ChLinkLockPrismatic);
  prismatic->SetName("prismatic joint");
  // default is along z-axis, rotate so to x-axis
  prismatic->Initialize(spindle, ground, ChCoordsys<>(spindle->GetPos(), Q_from_AngY(CH_C_PI_2)) );
  system.AddLink(prismatic);

  // constrain the spindle to the wheel
  ChSharedPtr<ChLinkLockRevolute> revolute(new ChLinkLockRevolute);
  revolute->SetName("Revolute joint");
  // default is along z-axis, so leave as is
  revolute->Initialize(wheel, spindle, ChCoordsys<>(wheel->GetPos(),QUNIT) );
  system.AddLink(revolute);
  
  // same as the first system, constraint the weight with a spring to the wheel body with a spring
  ChSharedPtr<ChBodyEasyBox> weight_wheel(new ChBodyEasyBox(0.1, 0.1, 0.2, weight_density));
  weight_wheel->SetPos( wheel->GetPos() + weight_offset);
  weight_wheel->SetName("weight_wheel box body");
  system.Add(weight_wheel);

  // spring between wheel weight and wheel
  ChSharedPtr<ChLinkSpring> wheel_spring(new ChLinkSpring);
  wheel_spring->Initialize(weight_wheel, wheel, false, weight_wheel->GetPos(), wheel->GetPos() );
  wheel_spring->Set_SpringK(spring_K);
  wheel_spring->Set_SpringF(spring_C);
  wheel_spring->SetName("wheel_spring");
  system.AddLink(wheel_spring);

  // 4. Create and setup the Irrlicht App
  ChIrrApp irrapp(&system, L"testing idler joint", core::dimension2d<u32>(1000,800),false,true);
  
  irrapp.AddTypicalLogo();
  irrapp.AddTypicalSky();
  irrapp.AddTypicalLights();
  irrapp.AddTypicalCamera(core::vector3df(0,4,-3));
  
  bool do_shadows = true; // shadow map is experimental
  irr::scene::ILightSceneNode* mlight = 0;

  if (do_shadows)
  {
    mlight = irrapp.AddLightWithShadow(
      irr::core::vector3df(10.f, 60.f, 30.f),
      irr::core::vector3df(0.f, 0.f, 0.f),
      150, 60, 80, 15, 512, irr::video::SColorf(1, 1, 1), false, false);
  }
  else
  {
    irrapp.AddTypicalLights(
      irr::core::vector3df(30.f, 100.f, -30.f),
      irr::core::vector3df(30.f, 100.f, 500.f),
      250, 130);
  }
  
  // pass the desired timestep to the irrlicht app
  irrapp.SetTimestep(time_step);

  // Complete asset specification: convert all assets to Irrlicht
  irrapp.AssetBindAll();
  irrapp.AssetUpdateAll();

  if (do_shadows)
  {
    irrapp.AddShadowAll();
  }

  // 5. Perform the simulation.

  // Calculate the required number of simulation steps and the number of steps
  // between output frames (based on the requested FPS).
  // int num_steps = (int) std::ceil(time_end / time_step);
  int out_steps = (int) std::ceil((1 / time_step) / out_fps);
   // Number of simulation steps between two 3D view render frames
  // Time interval between two render frames
  double render_step_size = 1.0 / render_fps;
  int render_steps = (int)std::ceil(render_step_size / time_step);

  // Initialize simulation frame counter and simulation time
  int step_number = 0;
  double time = 0;
  ChRealtimeStepTimer realtime_timer;
  int out_frame = 0;
  char filename[100];

  //for (int i = 0; i < num_steps; i++) {
  while(irrapp.GetDevice()->run()) 
  {
    // current simulation time
    time = system.GetChTime();
	
    // Render scene
    if (step_number % render_steps == 0) 
    {
      irrapp.GetVideoDriver()->beginScene(true, true, irr::video::SColor(255, 140, 161, 192));
	    irrapp.DrawAll();
      irrapp.GetVideoDriver()->endScene();
    }
	  // regardless of rendering the scene, take atime step
	  irrapp.DoStep();
		
    // test the state of the two bodies, and report on the difference
    // run_test(time, idler, wheel);

    // If this is an output frame, append body positions and orientations to the
    // output file and generate a rendering data file for this frame.
    // if (i % out_steps == 0) {
	  if (step_number % out_steps == 0) 
    {
      OutputData(&system, ofile, time);
      sprintf(filename, "%s/data_%03d.dat", pov_dir.c_str(), out_frame+1);
      utils::WriteShapesPovray(&system, filename);
      out_frame++;
    }

    // when not using irrlicht:
    // system.DoStepDynamics(time_step);

	  // increment the step counter
    step_number++;
  }

  return 0;
}


