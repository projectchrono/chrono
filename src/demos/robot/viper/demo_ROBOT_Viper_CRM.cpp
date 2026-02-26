// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2023 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Author: Radu Serban
// =============================================================================
//
// Viper rover on CRM terrain (initialized from heightmap image)
//
// =============================================================================

#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChSystemSMC.h"
#include "chrono/input_output/ChWriterCSV.h"
#include "chrono/assets/ChVisualSystem.h"

#include "chrono_fsi/sph/ChFsiSystemSPH.h"

#include "chrono_vehicle/ChVehicleDataPath.h"
#include "chrono_models/robot/viper/Viper.h"

#include "chrono_vehicle/terrain/CRMTerrain.h"

#include "chrono/utils/ChUtilsCreators.h"
#include "chrono/utils/ChUtilsGenerators.h"
#include "chrono/utils/ChUtilsGeometry.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChMassProperties.h"
#include "chrono/geometry/ChTriangleMeshConnected.h"

#include "chrono_thirdparty/filesystem/path.h"

#include "chrono_fsi/sph/visualization/ChSphVisualizationVSG.h"

#include "chrono_sensor/ChSensorManager.h"
#include "chrono_sensor/sensors/ChCameraSensor.h"
#include "chrono_sensor/filters/ChFilterAccess.h"
#include "chrono_sensor/filters/ChFilterVisualize.h"


using namespace chrono;
using namespace chrono::fsi;
using namespace chrono::fsi::sph;
using namespace chrono::viper;
using namespace chrono::vehicle;
using namespace chrono::sensor;

using std::cout;
using std::cin;
using std::endl;

// ===================================================================================================================

// CRM terrain patch type
enum class PatchType { RECTANGULAR, HEIGHT_MAP };
PatchType patch_type = PatchType::HEIGHT_MAP;

// Terrain dimensions (for RECTANGULAR or HEIGHT_MAP patch type)
double terrain_length = 12;
double terrain_width = 3;

// VDB info
bool firstInst = true;
std::vector<int> idList;
int prevActiveVoxels = 0;
std::vector<std::shared_ptr<ChBody>> voxelBodyList = {};
std::vector<float> offsetXList = {};
std::vector<float> offsetYList = {};

int num_meshes = 100;
std::vector<std::shared_ptr<ChVisualShapeTriangleMesh>> regolith_meshes;  // ChVisualShapeTriangleMesh


// ===================================================================================================================

void createVoxelGrid(std::vector<ChVector3d> points,
                     ChSystem* sys,
                     std::shared_ptr<ChVisualMaterial> vis_mat) {
    float slope_angle = 0;
    
    float spacing = 0.01 / 2.f;
    float r = spacing;
    int pointsPerVoxel = 1;
    float voxelSize = spacing;
    int activeVoxels = points.size();

    // std::cout << "Creating CPU Voxel Grid for " <<  activeVoxels << " particles." << std::endl;
    // std::cout << "Previous active voxels: " << prevActiveVoxels << std::endl;
    // std::cout << "VoxelSize=" << voxelSize << std::endl;

    int numVoxelsToAdd = activeVoxels - prevActiveVoxels;
    int numAdds = 0;
    int numUpdates = 0;

    if (!firstInst) {
        thread_local std::mt19937 generator(std::random_device{}());
        std::uniform_int_distribution<int> distribution(0, num_meshes - 1);
        std::uniform_real_distribution<float> randpos(-.005f, .005f);
        std::uniform_real_distribution<float> randscale(1.f, 1.5);
        int voxelCount = 0;
        for (int i = 0; i < activeVoxels; i++) {
            // ChVector3d pos(points[6 * i], points[6 * i + 1], points[6 * i + 2]);
            ChVector3d pos = points[i];
            if (!idList.empty() && ((voxelCount < prevActiveVoxels) || (voxelCount < idList.size()))) {
                numUpdates++;
                auto voxelBody = voxelBodyList[idList[voxelCount]];
                float offsetX = offsetXList[idList[voxelCount]];
                float offsetY = offsetYList[idList[voxelCount]];
                ChVector3d voxelPos(pos.x() + offsetX, pos.y() + offsetY, pos.z());
                double xRot = voxelPos.x() * cos(-slope_angle) + voxelPos.z() * sin(-slope_angle);
                double yRot = voxelPos.y();
                double zRot = -voxelPos.x() * sin(-slope_angle) + voxelPos.z() * cos(-slope_angle);

                // Create a new rotated vector
                ChVector3d rotatedVoxelPos(xRot, yRot, zRot);
                voxelBody->SetPos(rotatedVoxelPos);
                voxelBody->SetRot(QuatFromAngleY(-slope_angle));
                // voxelBody->SetPos({voxelPos.x(), voxelPos.y(), voxelPos.z()});
            }
            // Create a sphere for each point
            else if (numVoxelsToAdd > 0 && voxelCount >= prevActiveVoxels) {
                numAdds++;
                std::shared_ptr<ChBody> voxelBody;
                if (true) {
                    // std::cout << "Adding Mesh " << i << std::endl;
                    int meshIndex = distribution(generator);  // Randomly select a mesh (if needed)
                    auto trimesh_shape = regolith_meshes[meshIndex];
                    trimesh_shape->SetScale(randscale(generator));
                    if (trimesh_shape->GetNumMaterials() == 0) {
                        trimesh_shape->AddMaterial(vis_mat);
                    } else {
                        trimesh_shape->GetMaterials()[0] = vis_mat;
                    }
                    voxelBody = chrono_types::make_shared<ChBody>();
                    voxelBody->AddVisualShape(trimesh_shape);
                } else {
                    auto voxelBody = chrono_types::make_shared<ChBodyEasySphere>(r, 1000, true, false);
                }
                float offsetX = randpos(generator);
                float offsetY = randpos(generator);
                // Set the position and other properties of the voxel body
                ChVector3d voxelPos(pos.x() + offsetX, pos.y() + offsetY, pos.z());
                double xRot = voxelPos.x() * cos(-slope_angle) + voxelPos.z() * sin(-slope_angle);
                double yRot = voxelPos.y();
                double zRot = -voxelPos.x() * sin(-slope_angle) + voxelPos.z() * cos(-slope_angle);

                ChVector3d rotatedVoxelPos(xRot, yRot, zRot);
                voxelBody->SetPos(rotatedVoxelPos);
                voxelBody->SetRot(QuatFromAngleY(-slope_angle));
                voxelBody->SetFixed(true);

                int index = voxelBodyList.size();
                voxelBodyList.push_back(voxelBody);
                {
                    auto shape = voxelBody->GetVisualModel()->GetShapeInstances()[0].shape;
                    if (shape->GetNumMaterials() == 0) {
                        shape->AddMaterial(vis_mat);
                    } else {
                        shape->GetMaterials()[0] = vis_mat;
                    }
                }
                idList.emplace_back(index);
                offsetXList.emplace_back(offsetX);
                offsetYList.emplace_back(offsetY);
            }
            voxelCount++;
        }

    } else {
        voxelBodyList.resize(activeVoxels);
        idList.resize(activeVoxels);
        offsetXList.resize(activeVoxels);
        offsetYList.resize(activeVoxels);

        // std::atomic<int> voxelCount(0);  // Thread-safe counter for the voxels

        // Use std::for_each with parallel execution
        // std::for_each(std::execution::par, points.begin(), points.begin() + activeVoxels, [&](float& point) {
        for (int i = 0; i < points.size(); i++) {
            // Calculate the index based on the position in the loop
            // int i = &point - &points[0];  // Get the current index

            thread_local std::mt19937 generator(std::random_device{}());
            std::uniform_int_distribution<int> distribution(0, num_meshes - 1);
            std::uniform_real_distribution<float> randpos(-.005f, .005f);
            std::uniform_real_distribution<float> randscale(1.f, 1.5);
            // Compute voxel position in world space
            ChVector3d pos = points[i];
            // Create voxelBody if necessary
            if (numVoxelsToAdd > 0 && i >= prevActiveVoxels) {
                std::shared_ptr<ChBody> voxelBody;
                if (true) {
                    // std::cout << "Adding Mesh " << i << std::endl;
                    int meshIndex = distribution(generator);  // Randomly select a mesh (if needed)
                    auto trimesh_shape = regolith_meshes[meshIndex];
                    trimesh_shape->SetScale(randscale(generator));
                    if (trimesh_shape->GetNumMaterials() == 0) {
                        trimesh_shape->AddMaterial(vis_mat);
                    } else {
                        trimesh_shape->GetMaterials()[0] = vis_mat;
                    }
                    voxelBody = chrono_types::make_shared<ChBody>();
                    voxelBody->AddVisualShape(trimesh_shape);

                } else {
                    // Create a sphere voxel
                    voxelBody = chrono_types::make_shared<ChBodyEasySphere>(r, 1000, true, false);

                    auto shape = voxelBody->GetVisualModel()->GetShapeInstances()[0].shape;
                    if (shape->GetNumMaterials() == 0) {
                        shape->AddMaterial(vis_mat);
                    } else {
                        shape->GetMaterials()[0] = vis_mat;
                    }
                }

                float offsetX = randpos(generator);
                float offsetY = randpos(generator);
                // Set the position and other properties of the voxel body
                ChVector3d voxelPos(pos.x() + offsetX, pos.y() + offsetY, pos.z());
                double xRot = voxelPos.x() * cos(-slope_angle) + voxelPos.z() * sin(-slope_angle);
                double yRot = voxelPos.y();
                double zRot = -voxelPos.x() * sin(-slope_angle) + voxelPos.z() * cos(-slope_angle);

                // Create a new rotated vector
                ChVector3d rotatedVoxelPos(xRot, yRot, zRot);
                voxelBody->SetPos(rotatedVoxelPos);
                voxelBody->SetFixed(true);

                // Directly assign the voxelBody and index to the preallocated list positions
                voxelBodyList[i] = voxelBody;
                idList[i] = i;  // Assign index to idList slot
                offsetXList[i] = offsetX;
                offsetYList[i] = offsetY;
            }
            //});
        }
    }
    prevActiveVoxels = activeVoxels;
    // std::cout << "Num Voxels: " << voxelBodyList.size() << std::endl;
    // std::vector<std::shared_ptr<ChBody>> sub_vector(voxelBodyList.begin() + 182254, voxelBodyList.begin() + 364509);
    // std::vector<std::shared_ptr<ChBody>> sub_vector(voxelBodyList.begin() + 0, voxelBodyList.begin() + 182254);
    sys->SetSprites(voxelBodyList);
    firstInst = false;
}



int main(int argc, char* argv[]) {
    double density = 100;
    double cohesion = 5e3;
    double friction = 0.7;
    double youngs_modulus = 0.5e5;
    double poisson_ratio = 0.3;

    double tend = 30;
    double step_size = 5e-4;
    ChVector3d active_box_dim(5.0, 5.0, 5.0);

    bool render = true;       // use run-time visualization
    double render_fps = 200;  // rendering FPS

    bool visualization_sph = true;         // render SPH particles
    bool visualization_bndry_bce = false;  // render boundary BCE markers
    bool visualization_rigid_bce = false;  // render wheel BCE markers

    bool verbose = true;

    // Create the Chrono system and associated collision system
    ChSystemNSC sys;
    sys.SetCollisionSystemType(ChCollisionSystem::Type::BULLET);

    // Create rover
    cout << "Create rover..." << endl;
    ViperWheelType wheel_type = ViperWheelType::RealWheel;
    ChContactMaterialData wheel_mat(0.4f,   // mu
                                    0.2f,   // cr
                                    2e7f,   // Y
                                    0.3f,   // nu
                                    2e5f,   // kn
                                    40.0f,  // gn
                                    2e5f,   // kt
                                    20.0f   // gt
    );
    ChVector3d init_loc(1.25, 0.0, 0.55);

    auto driver = chrono_types::make_shared<ViperDCMotorControl>();
    auto rover = chrono_types::make_shared<Viper>(&sys, wheel_type);
    rover->SetDriver(driver);
    rover->SetWheelContactMaterial(wheel_mat.CreateMaterial(sys.GetContactMethod()));
    rover->Initialize(ChFrame<>(init_loc, QUNIT));

    // Create the CRM terrain system
    double initial_spacing = 0.01;
    CRMTerrain terrain(sys, initial_spacing);
    auto sysFSI = terrain.GetFsiSystemSPH();
    auto sysSPH = terrain.GetFluidSystemSPH();
    sysSPH->EnableCudaErrorCheck(false);
    terrain.SetVerbose(verbose);
    terrain.SetGravitationalAcceleration(ChVector3d(0, 0, -9.81));
    terrain.SetStepSizeCFD(step_size);

    ChFsiFluidSystemSPH::ElasticMaterialProperties mat_props;
    mat_props.Young_modulus = youngs_modulus;
    mat_props.Poisson_ratio = poisson_ratio;
    mat_props.mu_I0 = 0.04;
    mat_props.mu_fric_s = friction;
    mat_props.mu_fric_2 = friction;
    mat_props.average_diam = 0.005;
    mat_props.cohesion_coeff = cohesion;
    terrain.SetElasticSPH(mat_props);

    ChFsiFluidSystemSPH::SPHParameters sph_params;
    sph_params.integration_scheme = IntegrationScheme::RK2;
    sph_params.initial_spacing = initial_spacing;
    sph_params.d0_multiplier = 1.3;
    sph_params.free_surface_threshold = 2.0;
    sph_params.artificial_viscosity = 0.5;
    sph_params.use_consistent_gradient_discretization = false;
    sph_params.use_consistent_laplacian_discretization = false;
    sph_params.viscosity_method = ViscosityMethod::ARTIFICIAL_BILATERAL;
    sph_params.boundary_method = BoundaryMethod::ADAMI;
    sph_params.use_variable_time_step = true;  // This makes the step size irrelevant - now we just make sure we are
                                               // within the max of exchange_info (meta step)
    terrain.SetSPHParameters(sph_params);
    sysSPH->SetDensity(density);

    // Set output level from SPH simulation
    terrain.SetOutputLevel(OutputLevel::STATE);

    // Add rover wheels as FSI bodies
    cout << "Create wheel BCE markers..." << endl;
    std::string mesh_filename = GetChronoDataFile("robot/viper/obj/viper_cylwheel.obj");
    auto geometry = chrono_types::make_shared<utils::ChBodyGeometry>();
    geometry->materials.push_back(ChContactMaterialData());
    geometry->coll_meshes.push_back(utils::ChBodyGeometry::TrimeshShape(VNULL, QUNIT, mesh_filename, VNULL));

    //// TODO: FIX ChFsiProblemSPH to allow rotating geometry on body!!
    for (int i = 0; i < 4; i++) {
        auto wheel_body = rover->GetWheels()[i]->GetBody();
        auto yaw = (i % 2 == 0) ? QuatFromAngleZ(CH_PI) : QUNIT;
        terrain.AddRigidBody(wheel_body, geometry, false);
    }

    terrain.SetActiveDomain(active_box_dim);

    // Construct the terrain
    cout << "Create terrain..." << endl;
    switch (patch_type) {
        case PatchType::RECTANGULAR:
            // Create a rectangular terrain patch
            terrain.Construct({terrain_length, terrain_width, 0.25},  // length X width X height
                              ChVector3d(terrain_length / 2, 0, 0),   // patch center
                              BoxSide::ALL & ~BoxSide::Z_POS          // all boundaries, except top
            );
            break;
        case PatchType::HEIGHT_MAP:
            // Create a patch from a height field map image
            terrain.Construct(GetVehicleDataFile("terrain/height_maps/bump64.bmp"),  // height map image file
                              terrain_length, terrain_width,                           // length (X) and width (Y)
                              {0.25, 0.55},                                            // height range
                              0.25,                                                    // depth
                              true,                                                    // uniform depth
                              ChVector3d(terrain_length / 2, 0, 0),                    // patch center
                              BoxSide::Z_NEG                                           // bottom wall
            );
            break;
    }

    // Initialize the terrain system
    terrain.Initialize();

    auto aabb = terrain.GetSPHBoundingBox();
    cout << "  SPH particles:     " << terrain.GetNumSPHParticles() << endl;
    cout << "  Bndry BCE markers: " << terrain.GetNumBoundaryBCEMarkers() << endl;
    cout << "  SPH AABB:          " << aabb.min << "   " << aabb.max << endl;

    // Create run-time visualization
    std::shared_ptr<ChVisualSystem> vis;
    if (render) {
        // FSI plugin
        auto col_callback = chrono_types::make_shared<ParticleHeightColorCallback>(aabb.min.z(), aabb.max.z());

        auto visFSI = chrono_types::make_shared<ChSphVisualizationVSG>(sysFSI.get());
        visFSI->EnableFluidMarkers(visualization_sph);
        visFSI->EnableBoundaryMarkers(visualization_bndry_bce);
        visFSI->EnableRigidBodyMarkers(visualization_rigid_bce);
        visFSI->SetSPHColorCallback(col_callback, ChColormap::Type::BROWN);

        // VSG visual system (attach visFSI as plugin)
        auto visVSG = chrono_types::make_shared<vsg3d::ChVisualSystemVSG>();
        visVSG->AttachPlugin(visFSI);
        visVSG->AttachSystem(&sys);
        visVSG->SetWindowTitle("Viper rover on CRM deformable terrain");
        visVSG->SetWindowSize(1280, 800);
        visVSG->SetWindowPosition(100, 100);
        visVSG->AddCamera(init_loc + ChVector3d(0, 6, 0.5), init_loc);
        visVSG->SetLightIntensity(0.9f);

        visVSG->Initialize();
        vis = visVSG;
    }

    // Load regolith meshes
    std::string mesh_name_prefix = "sensor/geometries/regolith/particle_";
    for (int i = 1; i <= num_meshes; i++) {
        auto mmesh = ChTriangleMeshConnected::CreateFromWavefrontFile(
            GetChronoDataFile(mesh_name_prefix + std::to_string(i) + ".obj"), false, true);
        mmesh->Transform(ChVector3d(0, 0, 0), ChMatrix33<>(1));  // scale to a different size
        auto trimesh_shape = std::make_shared<ChVisualShapeTriangleMesh>();
        trimesh_shape->SetMesh(mmesh);
        // std::cout << "OK1" << std::endl;
        trimesh_shape->SetName("RegolithMesh" + std::to_string(i));
        trimesh_shape->SetMutable(false);
        regolith_meshes.push_back(trimesh_shape);
    }

    auto regolith_material = chrono_types::make_shared<ChVisualMaterial>();
    regolith_material->SetAmbientColor({1, 1, 1});  // 0.65f,0.65f,0.65f
    regolith_material->SetDiffuseColor({1, 1, 1});  // 0.29f, 0.29f, 0.235f
    regolith_material->SetSpecularColor({1, 1, 1});
    regolith_material->SetUseSpecularWorkflow(true);
    regolith_material->SetRoughness(1.0f);
    // regolith_material->SetBSDF((unsigned int)BSDFType::HAPKE);
    regolith_material->SetHapkeParameters(0.32357f, 0.23955f, 0.30452f, 1.80238f, 0.07145f, 0.3f,
                                          23.4f * (CH_PI / 180));
    regolith_material->SetClassID(30000);
    regolith_material->SetInstanceID(20000);

    float intensity = 1.0;
    auto manager = chrono_types::make_shared<ChSensorManager>(&sys);
    manager->scene->AddPointLight({0.5, 1, 1}, {intensity, intensity, intensity}, 500);
    manager->scene->SetAmbientLight({.1, .1, .1});
    manager->SetVerbose(false);
    manager->SetRayRecursions(4);

    auto floor = chrono_types::make_shared<ChBodyEasyBox>(1, 1, 1, 1000, false, false);
    floor->SetPos({0, 0, 0});
    floor->SetFixed(true);
    sys.Add(floor);

    chrono::ChFrame<double> offset_pose1(
        {0.5, 1, 1}, QuatFromAngleAxis(-CH_PI_2, {0, 0, 1}));  // Q_from_AngAxis(CH_PI_4, {0, 1, 0})  //-1200, -252, 100
    chrono::ChFrame<double> rot = chrono::ChFrame<double>(ChVector3d(0, 0, 0), QuatFromAngleAxis(60.0f / 180.0f * CH_PI, {0, 1, 0}));

    auto cam = chrono_types::make_shared<ChCameraSensor>(floor,         // body camera is attached to
                                                        30,   // update rate in Hz
                                                        offset_pose1 * rot,  // offset pose
                                                        1280,   // image width
                                                        720,  // image height
                                                        1.5707963267948966,           // camera's horizontal field of view
                                                        1,  // super sampling factor
                                                        CameraLensModelType::PINHOLE,    // lens model type
                                                        false);
    cam->PushFilter(chrono_types::make_shared<ChFilterVisualize>(1280, 720, "Third Person Camera"));

    manager->AddSensor(cam);


    // Start the simulation
    double time = 0;
    int sim_frame = 0;
    int render_frame = 0;
    double exchange_info = 5 * step_size;
    std::vector<ChVector3d> h_points;
    // std::vector<float> h_points;
    while (time < tend) {
        rover->Update();

        // Run-time visualization
        if (render && time >= render_frame / render_fps) {
            if (!vis->Run())
                break;
            vis->Render();
            render_frame++;
        }
        if (!render) {
            std::cout << time << "  " << terrain.GetRtfCFD() << "  " << terrain.GetRtfMBD() << std::endl;
        }

        // Advance dynamics of multibody and fluid systems concurrently
        terrain.DoStepDynamics(exchange_info);
        if (time > 0.2) {
            // h_points = sysSPH->GetPositions();
            // h_points = sysSPH->GetParticleData();
            auto all_particles = sysSPH->GetParticlePositions();
            h_points = std::vector<ChVector3d>(all_particles.begin(), all_particles.begin() + sysSPH->GetNumFluidMarkers());
            createVoxelGrid(h_points, &sys, regolith_material);
            // std::cout << "Start rendering" << std::endl;
            manager->Update();
        }

        time += exchange_info;
        sim_frame++;
    }
    std::cout << "Saving data..." << std::endl;
    sysSPH->SaveParticleData("particle_data");

    terrain.PrintStats();
    std::string out_dir = GetChronoOutputPath() + "ROBOT_Viper_CRM/";
    if (!filesystem::create_directory(filesystem::path(out_dir))) {
        std::cerr << "Error creating directory " << out_dir << std::endl;
        return 1;
    }
    terrain.PrintTimeSteps(out_dir + "time_steps.txt");

    return 0;
}
