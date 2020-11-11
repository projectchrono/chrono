// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban
// =============================================================================
//
// ChronoParallel test program using SMC method for frictional contact.
//
// The model simulated here consists of a number of spherical objects falling
// in a fixed container.
//
// The global reference frame has Z up.
//
// If available, OpenGL is used for run-time rendering. Otherwise, the
// simulation is carried out for a pre-defined duration and output files are
// generated for post-processing with POV-Ray.
// =============================================================================

#include <cstdio>
#include <vector>
#include <cmath>

#include "chrono_parallel/physics/ChSystemParallel.h"

#include "chrono/ChConfig.h"
#include "chrono/utils/ChUtilsCreators.h"
#include "chrono/utils/ChUtilsInputOutput.h"


#ifdef CHRONO_OPENGL
#include "chrono_opengl/ChOpenGLWindow.h"
#endif

using namespace chrono;
using namespace chrono::collision;

// Tilt angle (about global Y axis) of the container.
double tilt_angle = 0;

// Number of balls: (2 * count_X + 1) * (2 * count_Y + 1)
float b_radius =0.1;
int count_X = 60;
int count_Y = 60;
int count_Z = 15;

// Material properties (same on bin and balls)
float Y = 2e6f;
float mu = 0.5f;
float cr = 0.4f;
float damp = 1e4f;


auto top_middle = chrono_types::make_shared<ChBody>(chrono_types::make_shared<ChCollisionModelParallel>());

// -----------------------------------------------------------------------------
// Create a bin consisting of five boxes attached to the ground.
// -----------------------------------------------------------------------------
void AddContainer(ChSystemParallelSMC* sys) {
    // IDs for the two bodies
    int binId = -200;

    // Create a common material
    auto mat = chrono_types::make_shared<ChMaterialSurfaceSMC>();
    mat->SetYoungModulus(Y);
    mat->SetFriction(1);
    mat->SetRestitution(cr);
    mat->SetRollingFriction(1);
    mat->SetKfriction(1);
    mat->SetGn(damp);
    mat->SetGt(damp);
    mat->SetKn(3000*9.81*1/0.1);

    // Create the containing bin (4 x 4 x 1)
    auto bin = chrono_types::make_shared<ChBody>(chrono_types::make_shared<ChCollisionModelParallel>());
    bin->SetIdentifier(binId);
    bin->SetMass(1);
    bin->SetPos(ChVector<>(0, 0, 0));
    bin->SetRot(Q_from_AngY(tilt_angle));
    bin->SetCollide(true);
    bin->SetBodyFixed(true);

    ChVector<> hdim(6.05, 6.05, 4);
    double hthick = 0.0;


    bin->GetCollisionModel()->ClearModel();
    utils::AddBoxGeometry(bin.get(), mat, ChVector<>(hdim.x(), hdim.y(), hthick), ChVector<>(0, 0, -hthick));
    utils::AddBoxGeometry(bin.get(), mat, ChVector<>(hthick, hdim.y(), hdim.z()),
                          ChVector<>(-hdim.x() - hthick, 0, hdim.z()));
    utils::AddBoxGeometry(bin.get(), mat, ChVector<>(hthick, hdim.y(), hdim.z()),
                          ChVector<>(hdim.x() + hthick, 0, hdim.z()));
    utils::AddBoxGeometry(bin.get(), mat, ChVector<>(hdim.x(), hthick, hdim.z()),
                          ChVector<>(0, -hdim.y() - hthick, hdim.z()));
    utils::AddBoxGeometry(bin.get(), mat, ChVector<>(hdim.x(), hthick, hdim.z()),
                          ChVector<>(0, hdim.y() + hthick, hdim.z()));
    bin->GetCollisionModel()->BuildModel();

    sys->AddBody(bin);
}

// -----------------------------------------------------------------------------
// Create the falling spherical objects in a uniform rectangular grid.
// -----------------------------------------------------------------------------
void AddFallingBalls(ChSystemParallel* sys) {
    // Common material
    auto ballMat = chrono_types::make_shared<ChMaterialSurfaceSMC>();
    ballMat->SetYoungModulus(Y);
    ballMat->SetFriction(1);
    ballMat->SetRestitution(cr);
    ballMat->SetAdhesion(1);  // Magnitude of the adhesion in Constant adhesion model
    ballMat->SetRollingFriction(1);
    ballMat->SetKn(3000*9.81*1/0.1);

    // Create the falling balls
    int ballId = 0;
    double mass = 2;
    double radius = b_radius;

    

    ChVector<> inertia = (2.0 / 5.0) * mass * radius * radius * ChVector<>(1, 1, 1);
    for (int iz = 0; iz < count_Z; iz++){
        if(iz%2 == 0){
                for (int ix = -count_X/2; ix <= count_X/2; ix++) {
                        ChVector<> pos((2*radius) * ix, 0,  radius+2*radius*iz+0.0001);

                        auto ball = chrono_types::make_shared<ChBody>(chrono_types::make_shared<ChCollisionModelParallel>());
                        ball->SetIdentifier(ballId++);
                        ball->SetMass(mass);
                        ball->SetInertiaXX(inertia);
                        ball->SetPos(pos);
                        ball->SetRot(ChQuaternion<>(1, 0, 0, 0));
                        ball->SetBodyFixed(false);
                        ball->SetCollide(true);

                        ball->GetCollisionModel()->ClearModel();
                        utils::AddSphereGeometry(ball.get(), ballMat, radius);
                        ball->GetCollisionModel()->BuildModel();

                        sys->AddBody(ball);

                        if(ix==0&&iz==14){
                            top_middle = ball;
                        }
                        
                }
        }else{
                for (int ix = -count_X/2; ix <= count_X/2-1; ix++) {
                        ChVector<> pos((2*radius) * ix+radius, 0, radius+2*radius*iz+0.0001);

                        auto ball = chrono_types::make_shared<ChBody>(chrono_types::make_shared<ChCollisionModelParallel>());
                        ball->SetIdentifier(ballId++);
                        ball->SetMass(mass);
                        ball->SetInertiaXX(inertia);
                        ball->SetPos(pos);
                        ball->SetRot(ChQuaternion<>(1, 0, 0, 0));
                        ball->SetBodyFixed(false);
                        ball->SetCollide(true);

                        ball->GetCollisionModel()->ClearModel();
                        utils::AddSphereGeometry(ball.get(), ballMat, radius);
                        ball->GetCollisionModel()->BuildModel();

                        sys->AddBody(ball);


                    
                }
        }

    }
}

// -----------------------------------------------------------------------------
// Create the system, specify simulation parameters, and run simulation loop.
// -----------------------------------------------------------------------------
int main(int argc, char* argv[]) {
    GetLog() << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

    // Simulation parameters
    // ---------------------

    double gravity = 9.81;
    double time_step = 1e-3;
    double time_end = 2;

    double out_fps = 50;

    uint max_iteration = 100;
    real tolerance = 1e-3;

    // Create system
    // -------------

    ChSystemParallelSMC msystem;

    // Set number of threads
    msystem.SetNumThreads(8);

    // Set gravitational acceleration
    msystem.Set_G_acc(ChVector<>(0, 0, -gravity));

    // Set solver parameters
    msystem.GetSettings()->solver.max_iteration_bilateral = max_iteration;
    msystem.GetSettings()->solver.tolerance = tolerance;

    msystem.GetSettings()->collision.narrowphase_algorithm = NarrowPhaseType::NARROWPHASE_HYBRID_MPR;
    msystem.GetSettings()->collision.bins_per_axis = vec3(10, 10, 10);

    // The following two lines are optional, since they are the default options. They are added for future reference,
    // i.e. when needed to change those models.
    msystem.GetSettings()->solver.contact_force_model = ChSystemSMC::ContactForceModel::Hertz;
    msystem.GetSettings()->solver.adhesion_force_model = ChSystemSMC::AdhesionForceModel::Constant;

    // Create the fixed and moving bodies
    // ----------------------------------
    AddContainer(&msystem);
    AddFallingBalls(&msystem);
    
// Perform the simulation
// ----------------------

#ifdef CHRONO_OPENGL
    opengl::ChOpenGLWindow& gl_window = opengl::ChOpenGLWindow::getInstance();
    //std::cout<<"test1"<<std::endl;
    gl_window.Initialize(1280, 720, "ballsSMC", &msystem);
    gl_window.SetCamera(ChVector<>(0, -8, 5), ChVector<>(0, 10, 0), ChVector<>(0, 0, 20));
    gl_window.SetRenderMode(opengl::WIREFRAME);

    int currframe = 0;
    int outputframe = 0;
    while (true) {
        if (gl_window.Active()) {
            //std::cout<<"test3"<<std::endl;
            gl_window.DoStepDynamics(time_step);
            //std::cout<<"test4"<<std::endl;
            gl_window.Render();
            //std::cout<<"test5"<<std::endl;

            if(currframe%10==0){
                char filename[100];
                sprintf(filename, "%sstep%06d.csv","/home/jason/Gran/OCT11/build/bin/test/",outputframe);
                std::cout<<"Outputting "<<outputframe << " Frame"<<std::endl;
                msystem.Par_Gran_Outhelper(&msystem, filename);

                char contfilename[100];
                sprintf(contfilename, "%scont%06d.csv","/home/jason/Gran/OCT11/build/bin/test/",outputframe);
                std::cout<<"Outputting "<<outputframe << " Contact Frame"<<std::endl;
                msystem.Par_Gran_Cont_Outhelper(&msystem, contfilename);
                
                outputframe++;

                std::cout<<"Curr Frame Tot KE: "<<msystem.GetTotKineticEnergy(&msystem)<<std::endl;
            }

            if(currframe == 3000){
                top_middle->Accumulate_force(ChVector<>(0,0,-9.81*6),top_middle->GetPos(),false);
            }

            currframe++;
        } else {
            break;
        }
    }
#else
    // Run simulation for specified time
    int num_steps = (int)std::ceil(time_end / time_step);
    double time = 0;
    for (int i = 0; i < num_steps; i++) {
        msystem.DoStepDynamics(time_step);
        time += time_step;
    }
#endif

    return 0;
}
