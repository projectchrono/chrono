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
// Authors: Conlain Kelly & Dan Negrut
// =============================================================================
//
// ChronoParallel demo program using SMC method for frictional contact.
//
// The model simulated here represents a cylindrical Couette flow.
//
// The global reference frame has Z up.
//
// If available, OpenGL is used for run-time rendering. Otherwise, the
// simulation is carried out for a pre-defined duration and output files are
// generated for post-processing with POV-Ray.
// =============================================================================

#include <cstdio>
#include <cstdlib>
#include <vector>
#include "chrono/core/ChTimer.h"
#include "chrono/physics/ChBodyEasy.h"

#include "chrono/utils/ChUtilsGenerators.h"

#include "chrono/physics/ChLinkMotorRotationSpeed.h"
#include "chrono_parallel/physics/ChSystemParallel.h"

#include "chrono/ChConfig.h"
#include "chrono/core/ChFileutils.h"
#include "chrono/utils/ChUtilsCreators.h"

#include "chrono/utils/ChUtilsInputOutput.h"
#include "chrono_thirdparty/SimpleOpt/SimpleOpt.h"

#ifdef CHRONO_OPENGL
#include "chrono_opengl/ChOpenGLWindow.h"
#endif
// #define VISUALIZE 1

using namespace chrono;
using namespace chrono::collision;

double time_step = 1e-5;
double time_end = 6.;
int render_frame = 1;
int fric_steps = 10;
std::string output_prefix = "couette";
std::string output_suffix;

// Tilt angle (about global Y axis) of the container.
bool makeLid = true;
bool lidFall = false;

// Indicate if you want to write out data for rendering
bool write_output = false;

// Inner cylinder radius
double r1 = 1;
// Outer cylinder radius
double r2 = 2 * r1;
// half-height of big cylinder
double hh = 1;

// Material properties (same on bin and balls)
float Y = 2e5f;
float wallY = 1e7f;
float muBall = .8f;
float muWall = 1.0f;
float adh = 0.f;

int threads = 40;

// double adMax = 1.0f;
float cr = 0.4f;

double rotspeed = CH_C_2PI / 2;

double ball_radius = 0.1;

double ballDensity = 100;
double ballMass = 4.0 / 3.0 * ballDensity * CH_C_PI * ball_radius * ball_radius * ball_radius;
double lidMass = ballMass * 200;
double wallMass = 500;

// ID values to identify command line arguments
enum {
    OPT_HELP,
    OPT_SUFFIX,
    OPT_PREFIX,
    OPT_BALL_RADIUS,
    OPT_R1,
    OPT_R2,
    OPT_ROTSPEED,
    OPT_F_BALL,
    OPT_F_WALL,
    OPT_LID_FALL,
    OPT_WRITEOUT,
    OPT_COHESION,
    OPT_THREADS,
    OPT_TIMESTEP,
    OPT_TIMEEND,
    OPT_DENSITY,
    OPT_HEIGHT
};

// Table of CSimpleOpt::Soption structures. Each entry specifies:
// - the ID for the option (returned from OptionId() during processing)
// - the option as it should appear on the command line
// - type of the option
// The last entry must be SO_END_OF_OPTIONS
CSimpleOptA::SOption g_options[] = {{OPT_BALL_RADIUS, "-br", SO_REQ_SEP},
                                    {OPT_THREADS, "-n", SO_REQ_SEP},
                                    {OPT_TIMESTEP, "-t", SO_REQ_SEP},
                                    {OPT_TIMEEND, "-e", SO_REQ_SEP},
                                    {OPT_R1, "--radius1", SO_REQ_CMB},
                                    {OPT_R2, "--radius2", SO_REQ_CMB},
                                    {OPT_ROTSPEED, "-w", SO_REQ_SEP},
                                    {OPT_F_BALL, "--fball", SO_REQ_SEP},
                                    {OPT_F_WALL, "--fwall", SO_REQ_SEP},
                                    {OPT_LID_FALL, "--falling-lid", SO_NONE},
                                    {OPT_WRITEOUT, "--write-output", SO_NONE},
                                    {OPT_COHESION, "--cohesion", SO_REQ_SEP},
                                    {OPT_SUFFIX, "--suffix", SO_REQ_SEP},
                                    {OPT_PREFIX, "--prefix", SO_REQ_SEP},
                                    {OPT_DENSITY, "--density", SO_REQ_SEP},
                                    {OPT_HEIGHT, "--height", SO_REQ_SEP},
                                    {OPT_HELP, "-?", SO_NONE},
                                    {OPT_HELP, "-h", SO_NONE},
                                    {OPT_HELP, "--help", SO_NONE},
                                    SO_END_OF_OPTIONS};

void showUsage() {
    std::cout << "Options:" << std::endl;
    std::cout << "-br <ball_radius>" << std::endl;
    std::cout << "-n <num_threads>" << std::endl;
    std::cout << "-t <timestep>" << std::endl;
    std::cout << "-w <rotspeed>" << std::endl;
    std::cout << "--radius1=<inner_radius>" << std::endl;
    std::cout << "--radius2=<outer_radius>" << std::endl;
    std::cout << "--fball=<mu_ball>" << std::endl;
    std::cout << "--fwall=<mu_wall>" << std::endl;
    std::cout << "--cohesion=<cohesion_value>" << std::endl;
    std::cout << "--prefix=<output_prefix>" << std::endl;
    std::cout << "--suffix=<output_suffix>" << std::endl;
    std::cout << "--density=<density>" << std::endl;
    std::cout << "--height=<height>" << std::endl;
    std::cout << "--falling-lid\t Enable falling lid" << std::endl;
    std::cout << "--write-output\t Enable CSV output" << std::endl;
    std::cout << "-h / --help / -? \t Show this help." << std::endl;
}

bool GetProblemSpecs(int argc, char** argv);

// I stole this from utils to fix the collision shape
std::shared_ptr<ChBody> CreateCylindricalContainerFromBoxes(ChSystem* system,
                                                            int id,
                                                            std::shared_ptr<ChMaterialSurface> mat,
                                                            const ChVector<>& hdim,
                                                            double hthick,
                                                            int numBoxes,
                                                            const ChVector<>& pos,
                                                            const ChQuaternion<>& rot,
                                                            bool collide,
                                                            bool overlap,
                                                            bool closed,
                                                            bool isBoxBase,
                                                            bool partialVisualization) {
    // Verify consistency of input arguments.
    assert(mat->GetContactMethod() == system->GetContactMethod());

    // Create the body and set material
    std::shared_ptr<ChBody> body(system->NewBody());

    body->SetMaterialSurface(mat);

    // Set body properties and geometry.
    body->SetIdentifier(id);
    // body->SetMass(1);
    body->SetPos(pos);
    body->SetRot(rot);
    body->SetCollide(false);
    body->SetBodyFixed(true);

    double box_side = hdim.x() * 2.0 * tan(CH_C_PI / numBoxes);  // side length of cyl
    double o_lap = 0;
    if (overlap) {
        o_lap = hthick * 2;
    }
    double ang = 2.0 * CH_C_PI / numBoxes;
    ChVector<> p_boxSize = ChVector<>((box_side + hthick) / 2.0, hthick, hdim.z() + o_lap);  // size of plates
    ChVector<> p_pos;                                                                        // position of each plate
    ChQuaternion<> p_quat = QUNIT;                                                           // rotation of each plate
    body->GetCollisionModel()->ClearModel();

    for (int i = 0; i < numBoxes; i++) {
        p_pos = pos + ChVector<>(sin(ang * i) * (hthick + hdim.x()), cos(ang * i) * (hthick + hdim.x()), hdim.z());

        p_quat = Angle_to_Quat(AngleSet::RXYZ, ChVector<>(0, 0, ang * i));

        // this is here to make half the cylinder invisible.
        bool m_visualization = true;
        if ((ang * i > CH_C_PI && ang * i < 3.0 * CH_C_PI / 2.0) && partialVisualization) {
            m_visualization = false;
        }
        utils::AddBoxGeometry(body.get(), p_boxSize, p_pos, p_quat, m_visualization);
    }

    // Add ground piece
    if (isBoxBase) {
        utils::AddBoxGeometry(body.get(), Vector(hdim.x() + 2 * hthick, hdim.x() + 2 * hthick, hthick),
                              Vector(0, 0, -hthick), QUNIT, true);
    } else {
        utils::AddCylinderGeometry(body.get(), hdim.x() + 2 * hthick, hthick, ChVector<>(0, 0, -hthick),
                                   Q_from_AngAxis(CH_C_PI / 2, VECT_X));
    }

    if (closed) {
        if (isBoxBase) {
            utils::AddBoxGeometry(body.get(), Vector(hdim.x() + 2 * hthick, hdim.x() + 2 * hthick, hthick),
                                  Vector(0, 0, 2 * hdim.z() + hthick), QUNIT, true);
        } else {
            utils::AddCylinderGeometry(body.get(), hdim.x() + 2 * hthick, hthick,
                                       ChVector<>(0, 0, 2 * hdim.z() + hthick), Q_from_AngAxis(CH_C_PI / 2, VECT_X));
        }
    }

    body->GetCollisionModel()->SetEnvelope(0.2 * hthick);
    body->GetCollisionModel()->SetFamilyMaskNoCollisionWithFamily(5);
    body->GetCollisionModel()->SetFamilyMaskNoCollisionWithFamily(2);
    body->GetCollisionModel()->SetFamily(3);
    body->GetCollisionModel()->BuildModel();

    system->AddBody(body);
    return body;
}

// -----------------------------------------------------------------------------
// Create a bin consisting of five boxes attached to the ground.
// -----------------------------------------------------------------------------
void AddContainer(ChSystemParallelSMC* sys) {
    // IDs for the two bodies
    int binId = -200;

    // Create a common material
    auto mat = std::make_shared<ChMaterialSurfaceSMC>();
    mat->SetYoungModulus(wallY);
    mat->SetFriction(muWall);
    mat->SetRestitution(cr);
    mat->SetAdhesion(adh);

    // std::cout << "making ground body" ;
    std::shared_ptr<ChBody> ground(sys->NewBody());

    sys->AddBody(ground);
    ground->SetMaterialSurface(mat);
    ground->SetBodyFixed(true);
    ground->SetCollide(false);
    ground->SetPos({0, 0, 0});

    // Create the containing bin (4 x 4 x 1)
    std::shared_ptr<ChBody> cylinder1(sys->NewBody());
    cylinder1->SetMaterialSurface(mat);
    // cylinder1->SetIdentifier(-200);
    cylinder1->SetMass(wallMass);
    cylinder1->SetPos({0, 0, 0});
    cylinder1->SetCollide(true);
    cylinder1->SetName("cyl1");
    cylinder1->SetBodyFixed(false);

    cylinder1->GetCollisionModel()->ClearModel();
    utils::AddCylinderGeometry(cylinder1.get(), r1, 2 * hh, ChVector<>(0, 0, 0), Q_from_AngX(CH_C_PI / 2), true);
    cylinder1->GetCollisionModel()->BuildModel();

    cylinder1->GetCollisionModel()->SetFamilyMaskNoCollisionWithFamily(5);
    cylinder1->GetCollisionModel()->SetFamilyMaskNoCollisionWithFamily(3);
    cylinder1->GetCollisionModel()->SetFamily(2);

    sys->AddBody(cylinder1);

    auto motor = std::make_shared<ChLinkEngine>();

    motor->Initialize(cylinder1, ground, cylinder1->GetCoord());
    motor->Set_eng_mode(ChLinkEngine::ENG_MODE_SPEED);
    motor->SetName("cylspin");
    if (auto mfun = std::dynamic_pointer_cast<ChFunction_Const>(motor->Get_spe_funct()))
        mfun->Set_yconst(0);

    sys->AddLink(motor);

    if (makeLid) {
        auto lid = std::make_shared<ChBody>(std::make_shared<ChCollisionModelParallel>(), ChMaterialSurface::SMC);
        lid->SetMaterialSurface(mat);
        lid->SetMass(lidMass);

        // Set lid too high
        lid->SetPos({0, 0, 2.2 * hh + 2 * ball_radius});
        lid->SetPos_dt({0, 0, -.1});
        lid->SetCollide(true);
        lid->SetName("lid");
        lid->SetBodyFixed(!lidFall);

        lid->GetCollisionModel()->ClearModel();
        utils::AddCylinderGeometry(lid.get(), 1.1 * r2, .1, ChVector<>(0, 0, 0), Q_from_AngX(CH_C_PI / 2), true);
        lid->GetCollisionModel()->SetFamilyMaskNoCollisionWithFamily(2);
        lid->GetCollisionModel()->SetFamilyMaskNoCollisionWithFamily(3);
        lid->GetCollisionModel()->SetFamily(5);
        lid->GetCollisionModel()->BuildModel();

        sys->AddBody(lid);
        auto lidJoint = std::make_shared<ChLinkLockPrismatic>();
        lidJoint->Initialize(ground, lid, ground->GetCoord());
        sys->AddLink(lidJoint);
    }

    auto outercyl = CreateCylindricalContainerFromBoxes(sys, -201, mat, ChVector<>(r2, r2, 2 * hh), 2 * ball_radius,
                                                        100, ChVector<>(0, 0, -hh), ChQuaternion<>(1, 0, 0, 0), true,
                                                        true, false, true, true);
    outercyl->SetMass(wallMass);
    outercyl->SetCollide(true);
}

// -----------------------------------------------------------------------------
// Create the falling spherical objects in a uniform rectangular grid.
// -----------------------------------------------------------------------------
void AddFallingBalls(ChSystemParallel* sys) {
    ChVector<> inertia = (2.0 / 5.0) * ballMass * ball_radius * ball_radius * ChVector<>(1, 1, 1);

    // Create the falling balls
    int ballId = 0;

    utils::HCPSampler<> sampler(2.1 * ball_radius);

    ChVector<double> parCenter(0, 0, hh / 2 + ball_radius);
    auto points = sampler.SampleCylinderZ(parCenter, r2 - 1.05 * ball_radius, 1.5 * hh - 1.05 * ball_radius);
    for (int i = 0; i < points.size(); i++) {
        if (pow(points[i].x(), 2) + pow(points[i].y(), 2) < pow(r1 + 1.05 * ball_radius, 2)) {
            continue;
        }
        // Common material
        auto ballMat = std::make_shared<ChMaterialSurfaceSMC>();
        ballMat->SetYoungModulus(Y);
        ballMat->SetFriction(muBall);
        ballMat->SetRestitution(cr);
        ballMat->SetAdhesion(adh);  // Magnitude of the adhesion in Constant adhesion model

        std::shared_ptr<ChBody> ball(sys->NewBody());
        ball->SetMaterialSurface(ballMat);

        ball->SetIdentifier(ballId++);
        ball->SetMass(ballMass);
        ball->SetInertiaXX(inertia);
        ball->SetPos(points.at(i));
        ball->SetPos_dt(ChVector<>(0, 0, 0));
        ball->SetRot(ChQuaternion<>(1, 0, 0, 0));
        ball->SetBodyFixed(false);
        ball->SetCollide(true);
        sys->AddBody(ball);

        ball->GetCollisionModel()->ClearModel();
        utils::AddSphereGeometry(ball.get(), ball_radius);
        // ball->GetCollisionModel()->SetFamily(4);
        ball->GetCollisionModel()->BuildModel();
    }
}

void WriteCSV(const std::string& filename, ChSystemParallelSMC& sys) {
    std::ofstream file{filename};
    std::stringstream outstream;
    outstream << "x,y,z,vx,vy,vz,absv,f,a" << std::endl;
    for (auto body : *sys.Get_bodylist()) {
        auto pos = body->GetPos();
        auto vel = body->GetPos_dt();

        outstream << pos.x() << "," << pos.y() << "," << pos.z() << "," << vel.x() << "," << vel.y() << "," << vel.z()
                  << "," << vel.Length() << "," << body->GetMaterialSurfaceSMC()->GetSfriction() << ","
                  << body->GetMaterialSurfaceSMC()->GetAdhesion() << std::endl;
    }
    file << outstream.str();
    file.close();
}

// -----------------------------------------------------------------------------
// Create the system, specify simulation parameters, and run simulation loop.
// -----------------------------------------------------------------------------
int main(int argc, char* argv[]) {
    GetLog() << "Copyright (c) 2018 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

    if (GetProblemSpecs(argc, argv) == false)
        return 1;

    // Figure out which file the output goes out to
    // ---------------------
    std::string demo_dir(GetChronoOutputPath());
    demo_dir += output_prefix + output_suffix;
    if (write_output)
        std::cout << "outputting to " << demo_dir << std::endl;
    else
        std::cout << "no output generated!\n";

    std::cout << "time step is " << time_step << std::endl;
    std::cout << "ball mu is " << muBall << std::endl;
    std::cout << "rotspeed is " << rotspeed << std::endl;

    if (ChFileutils::MakeDirectory(demo_dir.c_str()) < 0) {
        std::cout << "Error creating directory " << demo_dir << std::endl;
        return 1;
    }

    // Create system
    // -------------
    ChSystemParallelSMC msystem;

    // Set number of threads.
    int max_threads = CHOMPfunctions::GetNumProcs();
    if (threads > max_threads)
        threads = max_threads;
    msystem.SetParallelThreadNumber(threads);
    CHOMPfunctions::SetNumThreads(threads);
    std::cout << "running on " << threads << " threads!" << std::endl;
    // Set gravitational acceleration
    double gravity = 9.81;
    msystem.Set_G_acc(ChVector<>(0, 0, -gravity));

    // Set solver parameters
    uint max_iteration = 100;
    real tolerance = 1e-3;

    msystem.GetSettings()->solver.max_iteration_bilateral = max_iteration;
    msystem.GetSettings()->solver.tolerance = tolerance;

    msystem.GetSettings()->collision.narrowphase_algorithm = NarrowPhaseType::NARROWPHASE_HYBRID_MPR;
    msystem.GetSettings()->collision.bins_per_axis = vec3(10, 10, 10);

    // The following two lines are optional, since they are the default options. They are added for future
    // reference, i.e. when needed to change those models.
    msystem.GetSettings()->solver.contact_force_model = ChSystemSMC::ContactForceModel::Hertz;
    msystem.GetSettings()->solver.adhesion_force_model = ChSystemSMC::AdhesionForceModel::Constant;

    // Create the fixed and moving bodies
    // ----------------------------------
    AddContainer(&msystem);
    AddFallingBalls(&msystem);
    std::cout << "balls added!" << std::endl;
    std::cout << msystem.Get_bodylist()->size() << " bodies!" << std::endl;
    std::cout << "lid Mass is " << lidMass << ", wall mass is " << wallMass << ", ball mass is " << ballMass
              << std::endl;

    // Perform the simulation
    // ----------------------

    int num_steps = (int)std::ceil(time_end / time_step);
    std::cout << "going to " << num_steps << " steps." << std::endl;
    int fps = 100;
    int render_steps = 1 / (fps * time_step);
    std::cout << "rendering every " << render_steps << " steps." << std::endl;

    // Start the motion of the inner cylinder after 2 seconds
    int init_steps = int(2. / time_step);
    std::cout << "init after " << init_steps << std::endl;
    ChTimer<> timer;
    timer.start();
#if defined(CHRONO_OPENGL) && defined(VISUALIZE)
    opengl::ChOpenGLWindow& gl_window = opengl::ChOpenGLWindow::getInstance();
    gl_window.Initialize(1280, 720, "ballsNSC", &msystem);
    gl_window.SetCamera(ChVector<>(0, -6, 0), ChVector<>(0, 0, 0), ChVector<>(0, 0, 1));
    gl_window.SetRenderMode(opengl::WIREFRAME);

    // Uncomment the following two lines for the OpenGL manager to automatically
    // run the simulation in an infinite loop.
    // gl_window.StartDrawLoop(time_step);
    int i = 0;
    while (true) {
        if (gl_window.Active()) {
            // This stops when the lid slows down too much
            if ((init_steps != -1) && (msystem.SearchBody("lid")->GetPos_dtdt().z() > .1) || i == init_steps) {
                init_steps = -1;
                if (auto spinlink = std::dynamic_pointer_cast<ChLinkEngine>(msystem.SearchLink("cylspin"))) {
                    std::cout << "link found" << std::endl;

                    if (auto mfun = std::dynamic_pointer_cast<ChFunction_Const>(spinlink->Get_spe_funct())) {
                        std::cout << "speed set" << std::endl;
                        mfun->Set_yconst(rotspeed);
                    }
                }
                if (makeLid) {
                    std::cout << "lid accel is " << msystem.SearchBody("lid")->GetPos_dtdt().z() << std::endl;
                    std::cout << "lid speed is " << msystem.SearchBody("lid")->GetPos_dt().z() << std::endl;
                    msystem.SearchBody("lid")->SetBodyFixed(true);
                    // msystem.Set_G_acc({0, 0, 0});

                    std::cout << "lid locked, cylinder spinning!" << std::endl;
                }
                std::cout << "gravity frozen at step " << i << std::endl;
            }
            i++;
            gl_window.DoStepDynamics(time_step);
            gl_window.Render();
            // if (gl_window.Running()) {
            //     msystem.CalculateContactForces();
            //     real3 frc = msystem.GetBodyContactForce(-200);
            //     std::cout << frc.x << "  " << frc.y << "  " << frc.z << std::endl;
            // }
        } else {
            break;
        }
    }
#else

    int frames_rendered = 0;
    for (int i = 0; i < num_steps; i++) {
        // This stops when the lid slows down too much
        // This stops when the lid slows down too much
        if ((init_steps != -1) && (msystem.SearchBody("lid")->GetPos_dtdt().z() > .1) || i == init_steps) {
            init_steps = -1;
            if (auto spinlink = std::dynamic_pointer_cast<ChLinkEngine>(msystem.SearchLink("cylspin"))) {
                std::cout << "link found" << std::endl;

                if (auto mfun = std::dynamic_pointer_cast<ChFunction_Const>(spinlink->Get_spe_funct())) {
                    std::cout << "speed set" << std::endl;
                    mfun->Set_yconst(rotspeed);
                }
            }
            if (makeLid) {
                std::cout << "lid accel is " << msystem.SearchBody("lid")->GetPos_dtdt().z() << std::endl;
                std::cout << "lid speed is " << msystem.SearchBody("lid")->GetPos_dt().z() << std::endl;
                msystem.SearchBody("lid")->SetBodyFixed(true);
                // msystem.Set_G_acc({0, 0, 0});

                std::cout << "lid locked, cylinder spinning!" << std::endl;
            }
            std::cout << "gravity frozen at step " << i << std::endl;
        }
        msystem.DoStepDynamics(time_step);

        if (i % render_steps == 0) {
            std::cout << "frame " << frames_rendered++ << std::endl;
            if (write_output) {
                char filename[100];
                sprintf(filename, "%s/data_%05d.csv", demo_dir.c_str(), render_frame++);
                WriteCSV(filename, msystem);
            }
        }
    }
#endif

    timer.stop();
    std::cout << "simulation time [s]: " << timer.GetTimeSeconds() << std::endl;
    return 0;
}

bool GetProblemSpecs(int argc, char** argv) {
    // Create the option parser and pass it the program arguments and the array of valid options.
    CSimpleOptA args(argc, argv, g_options);

    // Then loop for as long as there are arguments to be processed.
    while (args.Next()) {
        // Exit immediately if we encounter an invalid argument.
        if (args.LastError() != SO_SUCCESS) {
            std::cout << "Invalid argument: " << args.OptionText() << std::endl;
            showUsage();
            return false;
        }

        // Process the current argument.
        switch (args.OptionId()) {
            case OPT_HELP:
                showUsage();
                return false;
            case OPT_DENSITY:
                ballDensity = std::stod(args.OptionArg());
                // Recalculate params
                ballMass = 4.0 / 3.0 * ballDensity * CH_C_PI * ball_radius * ball_radius * ball_radius;
                lidMass = ballMass * 200;
                break;
            case OPT_BALL_RADIUS:
                ball_radius = std::stod(args.OptionArg());
                // Recalculate params
                ballMass = 4.0 / 3.0 * ballDensity * CH_C_PI * ball_radius * ball_radius * ball_radius;
                lidMass = ballMass * 200;
                break;
            case OPT_HEIGHT:
                // Store quarter-height
                hh = std::stod(args.OptionArg()) / 2;
                break;
            case OPT_R1:
                r1 = std::stod(args.OptionArg());
                break;
            case OPT_R2:
                r2 = std::stod(args.OptionArg());
                break;
            case OPT_ROTSPEED:
                rotspeed = std::stod(args.OptionArg());
                break;
            case OPT_F_BALL:
                muBall = std::stof(args.OptionArg());
                break;
            case OPT_F_WALL:
                muWall = std::stof(args.OptionArg());
                break;
            case OPT_LID_FALL:
                lidFall = true;
                break;
            case OPT_WRITEOUT:
                write_output = true;
                break;
            case OPT_COHESION:
                adh = std::stof(args.OptionArg());
                break;
            case OPT_THREADS:
                threads = std::stoi(args.OptionArg());
                break;
            case OPT_TIMESTEP:
                time_step = std::stod(args.OptionArg());
                break;
            case OPT_TIMEEND:
                time_end = std::stod(args.OptionArg());
                break;
            case OPT_PREFIX:
                output_prefix = args.OptionArg();
                break;
        }
    }

    return true;
}
