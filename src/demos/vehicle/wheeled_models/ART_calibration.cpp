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
// Authors: Radu Serban, Asher Elmquist
// =============================================================================
//
// Main driver function for the RCCar model.
//
// The vehicle reference frame has Z up, X towards the front of the vehicle, and
// Y pointing to the left.
//
// =============================================================================

#include "chrono/core/ChStream.h"
#include "chrono/core/ChRealtimeStep.h"
#include "chrono/utils/ChUtilsInputOutput.h"

#include "chrono_vehicle/ChConfigVehicle.h"
#include "chrono_vehicle/wheeled_vehicle/ChWheeledVehicle.h"
#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/terrain/RigidTerrain.h"
#include "chrono_vehicle/driver/ChDataDriver.h"
#include "chrono_vehicle/wheeled_vehicle/utils/ChWheeledVehicleVisualSystemIrrlicht.h"

#include "chrono_models/vehicle/rccar/RCCar.h"
#include "chrono/utils/ChUtilsInputOutput.h"

#include "chrono_thirdparty/filesystem/path.h"

using namespace chrono;
using namespace chrono::irrlicht;
using namespace chrono::vehicle;
using namespace chrono::vehicle::rccar;

// =============================================================================

// Initial vehicle location and orientation
ChVector<> initLoc(0, 0, 0.13);


bool useVis = false;

// Collision type for chassis (PRIMITIVES, MESH, or NONE)
CollisionType chassis_collision_type = CollisionType::NONE;

// Type of tire model (RIGID, TMEASY)
TireModelType tire_model = TireModelType::TMEASY;

// Rigid terrain
RigidTerrain::PatchType terrain_model = RigidTerrain::PatchType::BOX;
double terrainHeight = 0;      // terrain height (FLAT terrain only)
double terrainLength = 100.0;  // size in X direction
double terrainWidth = 100.0;   // size in Y direction

// Point on chassis tracked by the camera
ChVector<> trackPoint(0.0, 0.0, 0.2);

// Contact method
ChContactMethod contact_method = ChContactMethod::SMC;
bool contact_vis = false;

// Simulation step sizes
double step_size = 1e-4;
double tire_step_size = step_size;

// Simulation end time
double t_end;

// Time interval between two render frames
double render_step_size = 0.01;  // FPS = 50

// Output directories
const std::string out_dir = GetChronoOutputPath() + "ART_calibrate";
const std::string pov_dir = out_dir + "/POVRAY";

// Debug logging
bool debug_output = false;
double debug_step_size = 1.0 / 1;  // FPS = 1

// POV-Ray output
bool povray_output = false;

// =============================================================================

int main(int argc, char* argv[]) {
    GetLog() << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

    // get user input for stalling torque and max motor voltage ratio
    if (argc != 4){
        std::cout << "useage: ./ART_calibration <0 - Acc, 1 - Turn Left> <max voltage ratio> <stalling torque> " <<std::endl;

        return -1;
    }

    int test_type = std::stoi(argv[1]);
    double max_voltage_ratio = std::stof(argv[2]);
    double stalling_torque = std::stof(argv[3]);

    std::string driver_filename;
    ChQuaternion<> initRot;
    if (test_type == 0){
        driver_filename = "acc_4sec";
        initRot = ChQuaternion<>(1, 0, 0, 0);
        t_end = 10;
    }

    if (test_type == 1){
        driver_filename = "turn_left";
        initRot = ChQuaternion<>(8.286523e-1, -1.429243e-2, -1.745589e-3, -5.595786e-1);
        t_end = 10;

    }

    if (test_type == 2){
        driver_filename = "ramp_10sec";
        initRot = ChQuaternion<>(1, 0, 0, 0);
        t_end = 12.5;
    }

    std::string driver_file = out_dir + "/" + driver_filename + ".txt";
    std::cout << "dirver file name: " << driver_filename << std::endl;


    // --------------
    // Create systems
    // --------------

    // Create the Sedan vehicle, set parameters, and initialize
    RCCar my_rccar;
    my_rccar.SetContactMethod(contact_method);
    my_rccar.SetChassisCollisionType(chassis_collision_type);
    my_rccar.SetChassisFixed(false);
    my_rccar.SetInitPosition(ChCoordsys<>(initLoc, initRot));
    my_rccar.SetTireType(tire_model);
    my_rccar.SetTireStepSize(tire_step_size);
    my_rccar.SetMaxMotorVoltageRatio(max_voltage_ratio);
    my_rccar.SetStallTorque(stalling_torque);
    my_rccar.Initialize();

    if (useVis){
    // Visualization type for vehicle parts (PRIMITIVES, MESH, or NONE)
        VisualizationType chassis_vis_type = VisualizationType::PRIMITIVES;
        VisualizationType suspension_vis_type = VisualizationType::PRIMITIVES;
        VisualizationType steering_vis_type = VisualizationType::PRIMITIVES;
        VisualizationType wheel_vis_type = VisualizationType::NONE;
        VisualizationType tire_vis_type = VisualizationType::MESH;
        my_rccar.SetChassisVisualizationType(chassis_vis_type);
        my_rccar.SetSuspensionVisualizationType(suspension_vis_type);
        my_rccar.SetSteeringVisualizationType(steering_vis_type);
        my_rccar.SetWheelVisualizationType(wheel_vis_type);
        my_rccar.SetTireVisualizationType(tire_vis_type);
        my_rccar.SetMaxMotorVoltageRatio(0.3);
        my_rccar.SetStallTorque(0.738);
    }


    // Create the terrain
    RigidTerrain terrain(my_rccar.GetSystem());

    MaterialInfo minfo;
    minfo.mu = 0.9f;
    minfo.cr = 0.01f;
    minfo.Y = 2e7f;
    auto patch_mat = minfo.CreateMaterial(contact_method);

    std::shared_ptr<RigidTerrain::Patch> patch;
    switch (terrain_model) {
        case RigidTerrain::PatchType::BOX:
            patch = terrain.AddPatch(patch_mat, ChVector<>(0, 0, 0), ChVector<>(0, 0, 1), terrainLength, terrainWidth);
            patch->SetTexture(vehicle::GetDataFile("terrain/textures/tile4.jpg"), 200, 200);
            break;
        case RigidTerrain::PatchType::HEIGHT_MAP:
            patch = terrain.AddPatch(patch_mat, CSYSNORM, vehicle::GetDataFile("terrain/height_maps/test64.bmp"),
                                     128, 128, 0, 4);
            patch->SetTexture(vehicle::GetDataFile("terrain/textures/grass.jpg"), 16, 16);
            break;
        case RigidTerrain::PatchType::MESH:
            patch = terrain.AddPatch(patch_mat, CSYSNORM, vehicle::GetDataFile("terrain/meshes/test.obj"));
            patch->SetTexture(vehicle::GetDataFile("terrain/textures/grass.jpg"), 100, 100);
            break;
    }
    patch->SetColor(ChColor(0.8f, 0.8f, 0.5f));

    terrain.Initialize();

    // Create the vehicle Irrlicht interface
    auto vis = chrono_types::make_shared<ChWheeledVehicleVisualSystemIrrlicht>();
    if (useVis){
        vis->SetWindowTitle("RCCar Demo");
        vis->SetChaseCamera(trackPoint, 1.5, 0.05);
        vis->Initialize();
        vis->AddTypicalLights();
        vis->AddSkyBox();
        vis->AddLogo();
        my_rccar.GetVehicle().SetVisualSystem(vis);
    }

    // -----------------
    // Initialize output
    // -----------------

    if (!filesystem::create_directory(filesystem::path(out_dir))) {
        std::cout << "Error creating directory " << out_dir << std::endl;
        return 1;
    }
    if (povray_output) {
        if (!filesystem::create_directory(filesystem::path(pov_dir))) {
            std::cout << "Error creating directory " << pov_dir << std::endl;
            return 1;
        }
        terrain.ExportMeshPovray(out_dir);
    }


    // ------------------------
    // Create the driver system
    // ------------------------
    ChDataDriver driver(my_rccar.GetVehicle(), driver_file);
    driver.Initialize();

    // ---------------
    // Simulation loop
    // ---------------
    if (debug_output) {
        GetLog() << "\n\n============ System Configuration ============\n";
        my_rccar.LogHardpointLocations();
    }

    // output vehicle mass and other info
    std::cout << "VEHICLE MASS: " << my_rccar.GetVehicle().GetMass() << std::endl;
    std::cout<<"Front Unsprung Mass :"<<(my_rccar.GetVehicle().GetSuspension(0)->GetMass()/2 + my_rccar.GetVehicle().GetWheel(0,RIGHT)->GetMass()*2 +
    my_rccar.GetVehicle().GetTire(0,RIGHT)->GetMass()*2 + my_rccar.GetVehicle().GetBrake(0,RIGHT)->GetMass()*2)<<std::endl;
    // std::cout<<"Front unsprung mass : "<<(vehicle.GetSuspension(0)->GetMass())<<std::endl;
    std::cout<<"Rear Unsprung Mass :"<<(my_rccar.GetVehicle().GetSuspension(1)->GetMass()/2 + my_rccar.GetVehicle().GetWheel(1,RIGHT)->GetMass()*2 +
    my_rccar.GetVehicle().GetTire(1,RIGHT)->GetMass()*2 + my_rccar.GetVehicle().GetBrake(1,RIGHT)->GetMass()*2)<<std::endl;
    // std::cout<<"Rear unsprung mass : "<<(vehicle.GetSuspension(1)->GetMass())<<std::endl;
    std::cout<<"Unsprung Mass : "<<(my_rccar.GetVehicle().GetChassis()->GetMass() + my_rccar.GetVehicle().GetSteering(0)->GetMass()) <<std::endl;
    std::cout<<"The Vehicle Inertia matrix is "<<my_rccar.GetVehicle().GetInertia()<<std::endl;
    std::cout<<"The Vehicle Maximum Steering angle is "<<my_rccar.GetVehicle().GetMaxSteeringAngle()<<std::endl;
    std::cout<<"The Sprung Mass CG height is "<<my_rccar.GetVehicle().GetChassis()->GetTransform().TransformLocalToParent(my_rccar.GetVehicle().GetChassis()->GetCOMFrame().GetPos())<<std::endl;
    std::cout<<"The Vehicle CG height is "<<my_rccar.GetVehicle().GetCOMFrame().GetPos()<<std::endl;
    std::cout<<"The Vehicle Front Track Width is "<<my_rccar.GetVehicle().GetWheeltrack(0)<<std::endl;
    std::cout<<"The Vehicle Rear Track Width is "<<my_rccar.GetVehicle().GetWheeltrack(1)<<std::endl;
    std::cout<<"Suspension COM front"<<my_rccar.GetVehicle().GetSuspension(0)->GetTransform().GetPos()<<std::endl;    
    std::cout<<"Suspension COM back"<<my_rccar.GetVehicle().GetSuspension(1)->GetTransform().GetPos()<<std::endl;    
    std::cout << "wheel base: " << my_rccar.GetVehicle().GetWheelbase() << std::endl;

    // Number of simulation steps between miscellaneous events
    int render_steps = (int)std::ceil(render_step_size / step_size);
    int debug_steps = (int)std::ceil(debug_step_size / step_size);

    // Initialize simulation frame counters
    int step_number = 0;
    int render_frame = 0;

    if (contact_vis) {
        vis->SetSymbolScale(1e-4);
        vis->EnableContactDrawing(ContactsDrawMode::CONTACT_FORCES);
    }

    ChRealtimeStepTimer realtime_timer;

    
    // Initilaizing the CSV writer to write the output file
    utils::CSV_writer csv(",");
    csv.stream().setf(std::ios::scientific | std::ios::showpos);
    csv.stream().precision(6);

    csv << "time";
    csv << "throttle_input";
    csv << "x";
    csv << "y";
    csv << "vx";
    csv << "vy";
    csv << "ax";
    csv << "ay";
    csv << "motor_speed";
    csv << "motor_torque";
    csv << "FL_n";
    csv << "FL_fr";
    csv << "FL_trq";
    csv << "FR_n";
    csv << "FR_fr";
    csv << "FR_trq";
    csv << "BL_n";
    csv << "BL_fr";
    csv << "BL_trq";
    csv << "BR_n";
    csv << "BR_fr";
    csv << "BR_trq";
    csv << "FL_sptrq";
    csv << "FR_sptrq";
    csv << "BL_sptrq";
    csv << "BR_sptrq";
    csv << "FL_omg";
    csv << "FR_omg";
    csv << "BL_omg";
    csv << "BR_omg";
    csv << "FL_slip";
    csv << "FR_slip";
    csv << "BL_slip";
    csv << "BR_slip";




    csv << std::endl;

    double time = 0;
    while (time < t_end) {

        time = my_rccar.GetSystem()->GetChTime();

        // output data every 0.01 step
        if (step_number % render_steps == 0){            

            // Get the veclocities with respect to the local frame of reference
            auto chassis_vel_abs = my_rccar.GetVehicle().GetPointVelocity(my_rccar.GetVehicle().GetCOMFrame().GetPos());
            auto chassis_vel_veh = my_rccar.GetVehicle().GetTransform().TransformDirectionParentToLocal(chassis_vel_abs);

            // std::cout << chassis_vel_veh[0] << ", " << chassis_vel_veh[1];
            // std::cout << std::endl;


            // Get the vehicle accelerations
            auto chassis_acc_abs = my_rccar.GetVehicle().GetPointAcceleration(my_rccar.GetVehicle().GetChassis()->GetCOMFrame().GetPos());
            auto chassis_acc_veh = my_rccar.GetVehicle().GetTransform().TransformDirectionParentToLocal(chassis_acc_abs);

            // Orientation angles of the vehicle
            // auto rot = vehicle.GetTransform().GetRot();
            auto rot_v = my_rccar.GetVehicle().GetRot();
            auto euler123 = rot_v.Q_to_Euler123();


            // Get the Right wheel state
            auto state = my_rccar.GetVehicle().GetWheel(0,VehicleSide {RIGHT})->GetState();
            // Wheel normal (expressed in global frame)
            ChVector<> wheel_normal = state.rot.GetYaxis();

            // Terrain normal at wheel location (expressed in global frame)
            ChVector<> Z_dir = terrain.GetNormal(state.pos);

            // Longitudinal (heading) and lateral directions, in the terrain plane
            ChVector<> X_dir = Vcross(wheel_normal, Z_dir);
            X_dir.Normalize();
            ChVector<> Y_dir = Vcross(Z_dir, X_dir);

            // Tire reference coordinate system
            ChMatrix33<> rot;
            rot.Set_A_axis(X_dir, Y_dir, Z_dir);
            ChCoordsys<> tire_csys(state.pos, rot.Get_A_quaternion());

            // Express wheel normal in tire frame
            ChVector<> n = tire_csys.TransformDirectionParentToLocal(wheel_normal);

            // Wheel normal in the Vehicle frame
            ChVector<> n_v = my_rccar.GetVehicle().GetTransform().TransformDirectionParentToLocal(wheel_normal);

            // Toe-in
            auto toe_in_r = std::atan2(n_v.x(),n_v.y());


////////////////////////////////////////////////////////////////////////////////LEFT WHEEL - TOE-IN/////////////////////////////////////////////
            // Same process for the left wheel
            auto state_l = my_rccar.GetVehicle().GetWheel(0,VehicleSide {LEFT})->GetState();
            // Wheel normal (expressed in global frame)
            ChVector<> wheel_normal_l = state_l.rot.GetYaxis();
            // Terrain normal at wheel location (expressed in global frame)
            ChVector<> Z_dir_l = terrain.GetNormal(state_l.pos);

            // Longitudinal (heading) and lateral directions, in the terrain plane
            ChVector<> X_dir_l = Vcross(wheel_normal_l, Z_dir_l);
            X_dir_l.Normalize();
            ChVector<> Y_dir_l = Vcross(Z_dir_l, X_dir_l);

            // Tire reference coordinate system
            ChMatrix33<> rot_l;
            rot.Set_A_axis(X_dir_l, Y_dir_l, Z_dir_l);
            ChCoordsys<> tire_csys_l(state_l.pos, rot_l.Get_A_quaternion());

            // Express wheel normal in tire frame
            ChVector<> n_l = tire_csys_l.TransformDirectionParentToLocal(wheel_normal_l);

            // Wheel normal in the Vehicle frame
            ChVector<> n_v_l = my_rccar.GetVehicle().GetTransform().TransformDirectionParentToLocal(wheel_normal_l);


            auto toe_in_l = std::atan2(n_v_l.x(),n_v_l.y());


////////////////////////////////////////////////////////////////////////////////////////////////////////////

            // Slip angle just to check
            double slip_angle = my_rccar.GetVehicle().GetTire(0, VehicleSide {RIGHT})->GetSlipAngle();
            double long_slip = my_rccar.GetVehicle().GetTire(1, VehicleSide {RIGHT})->GetLongitudinalSlip();
            auto omega = my_rccar.GetVehicle().GetChassisBody()->GetWvel_loc();            

            double motor_speed = my_rccar.GetVehicle().GetPowertrain()->GetMotorSpeed();
            double motor_torque = my_rccar.GetVehicle().GetPowertrain()->GetMotorTorque();



            csv << time;
            csv << driver.GetThrottle();
            std::cout << time << ", " << driver.GetThrottle() << std::endl;
            csv << my_rccar.GetVehicle().GetPos().x();
            csv << my_rccar.GetVehicle().GetPos().y();
            csv << chassis_vel_veh[0];
            csv << chassis_vel_veh[1];
            csv << chassis_acc_veh[0];
            csv << chassis_acc_veh[1];
            csv << motor_speed * 60 / CH_C_2PI;
            csv << motor_torque;
            // csv << toe_in_r;
            // csv << (toe_in_l+toe_in_r)/2;
            // csv << toe_in_l;
            // csv << my_rccar.GetVehicle().GetSpindleAngVel(0,LEFT)[1];
            // csv << my_rccar.GetVehicle().GetSpindleAngVel(1,LEFT)[1];
            // csv << my_rccar.GetVehicle().GetSpindleAngVel(0,RIGHT)[1];
            // csv << my_rccar.GetVehicle().GetSpindleAngVel(1,RIGHT)[1];
            // csv << my_rccar.GetVehicle().GetTire(0,RIGHT)->GetDeflection();
            // csv << my_rccar.GetVehicle().GetTire(1,RIGHT)->GetDeflection();
            // csv << my_rccar.GetVehicle().GetTire(0,LEFT)->GetDeflection();
            // csv << my_rccar.GetVehicle().GetTire(1,LEFT)->GetDeflection();
            // csv << my_rccar.GetVehicle().GetDriveline()->GetSpindleTorque(0,VehicleSide {LEFT});

            // get total forces and torque on tire
            for (auto& axle: my_rccar.GetVehicle().GetAxles()){
                for (auto& wheel: axle->GetWheels()){
                    // normal, friction, and torque
                    csv << wheel->GetTire()->ReportTireForce(&terrain).force.z();
                    csv << wheel->GetTire()->ReportTireForce(&terrain).force.x();
                    csv << wheel->GetTire()->ReportTireForce(&terrain).moment.y();                    
                }
            }

            // get spindle torque on wheel
            csv << my_rccar.GetVehicle().GetDriveline()->GetSpindleTorque(0, LEFT);
            csv << my_rccar.GetVehicle().GetDriveline()->GetSpindleTorque(0, RIGHT);
            csv << my_rccar.GetVehicle().GetDriveline()->GetSpindleTorque(1, LEFT);
            csv << my_rccar.GetVehicle().GetDriveline()->GetSpindleTorque(1, RIGHT);


            csv << my_rccar.GetVehicle().GetSpindleAngVel(0,LEFT)[1];
            csv << my_rccar.GetVehicle().GetSpindleAngVel(0,RIGHT)[1];
            csv << my_rccar.GetVehicle().GetSpindleAngVel(1,LEFT)[1];
            csv << my_rccar.GetVehicle().GetSpindleAngVel(1,RIGHT)[1];

            csv << my_rccar.GetVehicle().GetTire(0, LEFT)->GetLongitudinalSlip();
            csv << my_rccar.GetVehicle().GetTire(0, RIGHT)->GetLongitudinalSlip();
            csv << my_rccar.GetVehicle().GetTire(1, LEFT)->GetLongitudinalSlip();
            csv << my_rccar.GetVehicle().GetTire(1, RIGHT)->GetLongitudinalSlip();




            csv << std::endl;            

        }


        // End simulation
        if (time >= t_end)
            break;

        // Render scene and output POV-Ray data
        if (step_number % render_steps == 0) {
            if (useVis) {
                vis->BeginScene();
                vis->DrawAll();
                vis->EndScene();
            }
            render_frame++;
        }

        // Debug logging
        if (debug_output && step_number % debug_steps == 0) {
            GetLog() << "\n\n============ System Information ============\n";
            GetLog() << "Time = " << time << "\n\n";
            my_rccar.DebugLog(OUT_SPRINGS | OUT_SHOCKS | OUT_CONSTRAINTS);
        }

        // Get driver inputs
        DriverInputs driver_inputs = driver.GetInputs();

        // Update modules (process inputs from other modules)
        driver.Synchronize(time);
        terrain.Synchronize(time);
        my_rccar.Synchronize(time, driver_inputs, terrain);

        if (useVis){
            // TODO: how to sync vis here??
            // vis->Synchronize(driver.GetInputModeAsString(), driver_inputs);
        }


        // Advance simulation for one timestep for all modules
        driver.Advance(step_size);
        terrain.Advance(step_size);
        my_rccar.Advance(step_size);
        if (useVis){
            vis->Advance(step_size);
        }

        // Increment frame number
        step_number++;
        // Spin in place for real time to catch up
        realtime_timer.Spin(step_size);
    }

    // csv.write_to_file("acc_test_output.csv");

    char test_name[100];
    sprintf(test_name, "_volt_%.2f_stalling_%.2f", max_voltage_ratio, stalling_torque);

    std::string csv_output = driver_filename + test_name + ".csv";
    csv.write_to_file(csv_output);

    return 0;
}
