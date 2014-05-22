#include <stdio.h>
#include <vector>
#include <cmath>

#include "ChSystemParallel.h"
#include "ChLcpSystemDescriptorParallel.h"

#include "utils/input_output.h"

using namespace chrono;
using namespace geometry;

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void AddSphereGeometry(ChBody*               body,
                       double                radius,
                       const ChVector<>&     pos = ChVector<>(0,0,0),
                       const ChQuaternion<>& rot = ChQuaternion<>(1,0,0,0))
{
  body->GetCollisionModel()->AddSphere(radius, pos);

  ChSharedPtr<ChSphereShape> sphere = ChSharedPtr<ChAsset>(new ChSphereShape);
  sphere->GetSphereGeometry().rad = radius;
  sphere->Pos = pos;
  sphere->Rot = rot;

  body->GetAssets().push_back(sphere);
}

void AddBoxGeometry(ChBody*                body,
                    const ChVector<>&      hdim,
                    const ChVector<>&      pos = ChVector<>(0,0,0),
                    const ChQuaternion<>&  rot = ChQuaternion<>(1,0,0,0))
{
  // Append to collision geometry
  body->GetCollisionModel()->AddBox(hdim.x, hdim.y, hdim.z, pos, rot);

  // Append to assets
  ChSharedPtr<ChBoxShape> box_shape = ChSharedPtr<ChAsset>(new ChBoxShape);
  box_shape->GetBoxGeometry().Size = hdim;
  box_shape->Pos = pos;
  box_shape->Rot = rot;

  body->GetAssets().push_back(box_shape);
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void AddContainer(ChSystemParallelDVI* sys)
{
  // IDs for the two bodies
  int  binId = -200;
  int  mixerId = -201;

  // Create a common material
  ChSharedPtr<ChMaterialSurface> mat(new ChMaterialSurface);
  mat->SetFriction(0.4f);

  // Create the containing bin (2 x 2 x 1)
  ChSharedBodyPtr bin(new ChBody(new ChCollisionModelParallel));
  bin->SetMaterialSurface(mat);
  bin->SetIdentifier(binId);
  bin->SetMass(1);
  bin->SetPos(ChVector<>(0,0,0));
  bin->SetRot(ChQuaternion<>(1,0,0,0));
  bin->SetCollide(true);
  bin->SetBodyFixed(true);

  ChVector<> hdim(1, 1, 0.5);
  double     hthick = 0.1;

  bin->GetCollisionModel()->ClearModel();
  AddBoxGeometry(bin.get_ptr(), ChVector<>(hdim.x, hdim.y, hthick), ChVector<>(0, 0, -hthick));
  AddBoxGeometry(bin.get_ptr(), ChVector<>(hthick, hdim.y, hdim.z), ChVector<>(-hdim.x-hthick, 0, hdim.z));
  AddBoxGeometry(bin.get_ptr(), ChVector<>(hthick, hdim.y, hdim.z), ChVector<>( hdim.x+hthick, 0, hdim.z));
  AddBoxGeometry(bin.get_ptr(), ChVector<>(hdim.x, hthick, hdim.z), ChVector<>(0, -hdim.y-hthick, hdim.z));
  AddBoxGeometry(bin.get_ptr(), ChVector<>(hdim.x, hthick, hdim.z), ChVector<>(0,  hdim.y+hthick, hdim.z));
  bin->GetCollisionModel()->BuildModel();

  sys->AddBody(bin);

  // The rotating mixer body (1.6 x 0.2 x 0.4)
  ChSharedBodyPtr mixer(new ChBody(new ChCollisionModelParallel));
  mixer->SetMaterialSurface(mat);
  mixer->SetIdentifier(mixerId);
  mixer->SetMass(10.0);
  mixer->SetInertiaXX(ChVector<>(50,50,50));
  mixer->SetPos(ChVector<>(0,0,0.205));
  mixer->SetBodyFixed(false);
  mixer->SetCollide(true);

  ChVector<> hsize(0.8, 0.1, 0.2);

  mixer->GetCollisionModel()->ClearModel();
  AddBoxGeometry(mixer.get_ptr(), hsize);
  mixer->GetCollisionModel()->BuildModel();

  sys->AddBody(mixer);

  // Create an engine between the two bodies
  ChSharedPtr<ChLinkEngine> motor(new ChLinkEngine);

  motor->Initialize(mixer, bin,
                    ChCoordsys<>(ChVector<>(0,0,0), ChQuaternion<>(1,0,0,0)));
  motor->Set_eng_mode(ChLinkEngine::ENG_MODE_SPEED);
  if (ChFunction_Const* mfun = dynamic_cast<ChFunction_Const*>(motor->Get_spe_funct()))
    mfun->Set_yconst(CH_C_PI/2); // speed w=90°/s

  sys->AddLink(motor);
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void AddFallingBalls(ChSystemParallel* sys)
{
  // Common material
  ChSharedPtr<ChMaterialSurface> ballMat(new ChMaterialSurface);
  ballMat->SetFriction(0.4f);

  // Create the falling balls
  int        ballId = 0;
  double     mass = 1;
  double     radius = 0.15;
  ChVector<> inertia = (2.0/5.0)*mass*radius*radius*ChVector<>(1,1,1);

  for (int ix = -2; ix < 3; ix++) {
    for (int iy = -2; iy < 3; iy++) {
      ChVector<> pos(0.4 * ix, 0.4 * iy, 1);

      ChSharedBodyPtr ball(new ChBody(new ChCollisionModelParallel));
      ball->SetMaterialSurface(ballMat);

      ball->SetIdentifier(ballId++);
      ball->SetMass(mass);
      ball->SetInertiaXX(inertia);
      ball->SetPos(pos);
      ball->SetRot(ChQuaternion<>(1, 0, 0, 0));
      ball->SetBodyFixed(false);
      ball->SetCollide(true);

      ball->GetCollisionModel()->ClearModel();
      AddSphereGeometry(ball.get_ptr(), radius);
      ball->GetCollisionModel()->BuildModel();

      sys->AddBody(ball);
    }
  }
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
int main(int argc, char* argv[])
{
  int threads = 8;

  // Simulation parameters
  // ---------------------

  double gravity = 9.81;
  double time_step = 1e-3;
  double time_end = 1;
  int    num_steps = std::ceil(time_end / time_step);

  uint max_iteration = 50;
  real tolerance = 1e-8;

  const char* out_folder = "../MIXER_DVI/POVRAY";
  double out_fps = 50;
  int out_steps = std::ceil((1 / time_step) / out_fps);


  // Create system
  // -------------

  ChSystemParallelDVI msystem;

  // Set number of threads.
  int max_threads = msystem.GetParallelThreadNumber();
  if (threads > max_threads)
    threads = max_threads;
  msystem.SetParallelThreadNumber(threads);
  omp_set_num_threads(threads);

  // Set gravitational acceleration
  msystem.Set_G_acc(ChVector<>(0, 0, -gravity));

  // Set solver parameters
  msystem.SetMaxiter(max_iteration);
  msystem.SetIterLCPmaxItersSpeed(max_iteration);
  msystem.SetTol(1e-3);
  msystem.SetTolSpeeds(1e-3);
  msystem.SetStep(time_step);

  ((ChLcpSolverParallelDVI*) msystem.GetLcpSolverSpeed())->SetMaxIteration(max_iteration);
  ((ChLcpSolverParallelDVI*) msystem.GetLcpSolverSpeed())->SetTolerance(0);
  ((ChLcpSolverParallelDVI*) msystem.GetLcpSolverSpeed())->SetCompliance(0);
  ((ChLcpSolverParallelDVI*) msystem.GetLcpSolverSpeed())->SetContactRecoverySpeed(1);
  ((ChLcpSolverParallelDVI*) msystem.GetLcpSolverSpeed())->SetSolverType(ACCELERATED_PROJECTED_GRADIENT_DESCENT);

  ((ChCollisionSystemParallel*) msystem.GetCollisionSystem())->SetCollisionEnvelope(0.01);
  ((ChCollisionSystemParallel*) msystem.GetCollisionSystem())->setBinsPerAxis(I3(10, 10, 10));
  ((ChCollisionSystemParallel*) msystem.GetCollisionSystem())->setBodyPerBin(100, 50);


  // Create the fixed and moving bodies
  // ----------------------------------

  AddContainer(&msystem);
  AddFallingBalls(&msystem);

  // Perform the simulation
  // ----------------------

  double time = 0;
  int out_frame = 0;
  char filename[100];

  for (int i = 0; i < num_steps; i++) {

    if (i % out_steps == 0) {
      sprintf(filename, "%s/data_%03d.dat", out_folder, out_frame);
      utils::WriteShapesPovray(&msystem, filename);
      out_frame++;
    }

    msystem.DoStepDynamics(time_step);
    time += time_step;
  }

  return 0;
}

