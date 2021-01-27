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
// Authors: Alessandro Tasora
// =============================================================================
//
// Demo code about
// - collisions and contacts
// - using the 'barrel' shape to create rollers for building omnidirectional
//   wheels in a mobile robot.
//
// =============================================================================

#include "chrono/core/ChRealtimeStep.h"
#include "chrono/physics/ChLinkMotorRotationSpeed.h"
#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/utils/ChUtilsGeometry.h"
#include "chrono/assets/ChBarrelShape.h"
#include "chrono_irrlicht/ChIrrApp.h"
#include "chrono/physics/ChForce.h"

// Use the namespaces of Chrono
using namespace chrono;
using namespace chrono::irrlicht;

// Use the main namespaces of Irrlicht
using namespace irr;
using namespace irr::core;
using namespace irr::scene;
using namespace irr::video;
using namespace irr::io;
using namespace irr::gui;


// Number of propellers
template <int nop>
class ChCopter {
  public:
	
	ChCopter(ChSystem& sys,				    /// the Chrono physical system
		ChVector<>& cpos,					/// Chassis position
		std::vector<ChVector<>>& ppos,		/// Propeller relative position
		bool clockwise[],					/// i-th propeller rotates clockwise -> true
		bool are_prop_pos_rel = true,		/// if false, propeller axes position has to be given in the abs frame
		bool z_up = false
	) {
		// TODO: ChBodyAuxRef here might be more convenient
		up = (z_up) ? VECT_Z : VECT_Y;
		chassis = chrono_types::make_shared<ChBody>();
		chassis->SetPos(cpos);
		// Data from little hexy, page 132. 
		chassis->SetMass(10.832);
		chassis->SetInertiaXX(ChVector<>(1,1,1.3));
		chassis->SetBodyFixed(false);
		sys.AddBody(chassis);
		h0 = chassis->GetPos() ^ up;
		//26.4 inch propellers
		for (int p = 0; p < nop; p++) {
			auto prop = chrono_types::make_shared<ChBody>();
			props.push_back(prop);
            if (are_prop_pos_rel) {
                prop->SetPos(cpos + ppos[p]);
            } 
			else {
				prop->SetPos(ppos[p]);
			}
			// Data from little hexy, page 132. 
			prop->SetMass(1);
			prop->SetInertiaXX(ChVector<>(0.2,0.2,0.2));
			prop->SetBodyFixed(false);
            sys.AddBody(prop);

			auto propmot = chrono_types::make_shared<ChLinkMotorRotationSpeed>();
			ChQuaternion<> motor_rot = Q_ROTATE_Y_TO_Z;
			if (z_up) { motor_rot = QUNIT; };
            if (clockwise[p]) {
                motor_rot = Q_FLIP_AROUND_X * motor_rot;
            };
			propmot->Initialize(prop, chassis, ChFrame<>(ppos[p], motor_rot) );
			auto speed = chrono_types::make_shared<ChFunction_Const>(0);
			propmot->SetSpeedFunction(speed);
			sys.AddLink(propmot);

			motors.push_back(propmot);
			speeds.push_back(speed);

			u_p[p] = 0;
            auto thrust = chrono_types::make_shared<ChForce>();
            prop->AddForce(thrust);
            thrust->SetMode(ChForce::FORCE);
            thrust->SetMforce(0);
			thrust->SetRelDir(up);
			thrusts.push_back(thrust);

			auto backtorque = std::make_shared<ChForce>();
            backtorque->SetBody(prop.get());
            backtorque->SetMode(ChForce::TORQUE);
			backtorque->SetMforce(0);
			// Resistance Torque direction opposed to omega 
			ChVector<> tdir = (clockwise) ? up : -up;
			backtorque->SetRelDir(tdir);
            backtorques.push_back(backtorque);
			// 

		}

		// linear drag on copter body
		lin_drag = std::make_shared<ChForce>();
		lin_drag->SetBody(chassis.get());
        lin_drag->SetMode(ChForce::FORCE);
		lin_drag->SetMforce(0);
		//thrust->SetDir(up);
	}

	~ChCopter() {

	}

	std::shared_ptr<ChBody> GetChassis() {return chassis;}

	void SetPropellersData(int mass, ChVector<>& inerXX, double diam, double thrust_coeff, double power_coeff, double max_rpm) { 
		Dp = diam;
		Ct = thrust_coeff;
		Cp = power_coeff;
		rps_max = max_rpm/60;
		for (auto& prop : props) {
			prop->SetMass(mass);
			prop->SetInertiaXX(inerXX);
		}
	}
	void SetLinearDragCoeff(double ldc) { Cd = ldc; }

	void SetMeshes(std::string&  chassismesh, std::string&  propellermesh, ChCoordsys<> cor_m1 = CSYSNORM, ChCoordsys<> cor_m2 = CSYSNORM) {
	
		auto chassis_asset_assembly = chrono_types::make_shared<ChAssetLevel>();
		chassis_asset_assembly->GetFrame().SetPos(cor_m1.pos);
		chassis_asset_assembly->GetFrame().SetRot(cor_m1.rot);
		chassis->AddAsset(chassis_asset_assembly);
		auto chassis_mesh = chrono_types::make_shared<ChObjShapeFile>();
		chassis_mesh->SetFilename(chassismesh);
		chassis_asset_assembly->AddAsset(chassis_mesh);
		for (auto propeller : props) {
			auto propeller_asset_assembly = chrono_types::make_shared<ChAssetLevel>();
			propeller_asset_assembly->GetFrame().SetPos(cor_m1.pos);
			propeller_asset_assembly->GetFrame().SetRot(cor_m1.rot);
			propeller->AddAsset(propeller_asset_assembly);
			auto propeller_mesh = chrono_types::make_shared<ChObjShapeFile>();
			propeller_mesh->SetFilename(propellermesh);
			propeller_asset_assembly->AddAsset(propeller_mesh);
		}
	}


	// Increment Propellers omegas
	void ControlIncremental(double inputs[nop]) {
		for (int i = 0; i < nop; i++) {
			u_p[i] = ChClamp(u_p[i] + inputs[i], -1, 1)
			speeds[i]->Set_yconst(u_p[i]*rps_max*CH_C_2PI);
		}
	}

	// Set Propellers omegas
	void ControlAbsolute(double inputs[nop]) {
		for (int i = 0; i < nop; i++) {
            u_p[i] = ChClamp<double>(inputs[i], -1, 1);
            speeds[i]->Set_yconst(u_p[i] * rps_max * CH_C_2PI);
		}
	}
	
	// Update the copter
	void Update(double timestep) {
	// update propeller forces/torques
		for (int i = 0; i < nop; i++) {
			double rpm = motors[i]->GetMotorRot_dt() / CH_C_2PI ;
            thrusts[i]->SetMforce(Ct * rho * pow(rpm, 2) * pow(Dp, 4));
            //backtorques[i]->SetMforce((1 / CH_C_2PI) * Cp * rho * pow(rpm, 4) * pow(Dp, 5) * rpm_max);
		}
	// update linear drag / drag torque
        lin_drag->SetMforce(0.5 * Cd * Surf * rho * chassis->GetPos_dt().Length2());
		lin_drag->SetDir(chassis->GetPos_dt());
	// update rotor internal physics: Magnetic field, air pressure (gravity managed by chrono)
		UpdateAirData();
	// update sensors: gps, camera, magnetometer, altitude

	}
	void SetInitAirData(double rho0, double p0) { rho = rho0; pressure = p0; }

	double GetAirPressure() { return pressure; }

	double GetGroundAirPressure() { return pressure0; }

	void SetGroundPressure(double p0) { pressure0 = p0; }

	double GetAirDensity() {return rho; }

	double GetAltitude() { return Altitude; }

	double GetInitAltitude() { return Altitude0; }

	void SetInitAltitude(double alt) { Altitude0 = alt; }

	double GetTemperature() { return Temp; }

	void SetTemperature(double temp) { Temp = temp; }

	double GetGroundTemperature() { return Temp0; }

	void SetGroundTemperature(double temp) { Temp0 = temp; }

  private:
	// Virtual method. Might be overridden for special condition (e.g. Mars atmosphere)
	// This model holds below 11 km altitude.
	virtual void UpdateAirData(){
		Altitude = Altitude0 + (chassis->GetPos() ^ up - h0);
		Temp = Temp0 - (6.5 * (Altitude / 1000));
		pressure = pressure0 * pow((Temp0 / Temp), -5.255877);
	}

  protected:
	  // Chassis body
	  std::shared_ptr<ChBody> chassis;
	  // Visualization meshes
	  std::string chassis_mesh_file;
	  std::string prop_mesh_file;
	  // Propellers bodies
	  std::vector<std::shared_ptr<ChBody>> props;
	  // Propellers position
	  //std::vector<std::shared_ptr<ChBody>> prop_pos;
	  // Max propeller rotations per SECOND (rps)
      double rps_max;
	  // Propeller diameter [m]
	  double Dp;
	  // Thrust coefficient
	  double Ct;
	  // Power coefficient
	  double Cp;
	  // Propeller rotation as fraction of max rpm, 0<=ui<=1
	  double u_p[nop];
	  // Air density [kg/m^3]
	  double rho = 1.225;
	  // Air pressure [Pa]
	  double pressure = 101325;
	  // Ground Air pressure [Pa]
	  double pressure0 = 101325;
	  // Altitude [m]
	  double Altitude;
	  // Ground  Air Temperature [K]
	  double Temp0 = 298;
	  // Air Temperature [K]
	  double Temp = 298;
	  // Initial Altitude [m]
	  double Altitude0 = 0;
	  // Initial Altitude in simulation [m]
	  double h0 = 0;
	  // Vertical axis
	  ChVector<> up;
	  // Thrust and torques to be applied to propellers
	  // Thrust Forces
	  std::vector<std::shared_ptr<ChForce>> thrusts;
	  // Propeller Resistance torques
	  std::vector<std::shared_ptr<ChForce>> backtorques;
	  // Propeller Motors
	  std::vector<std::shared_ptr<ChLinkMotorRotationSpeed>> motors;
	  // Propeller Motors Speed Functions
	  std::vector<std::shared_ptr<ChFunction_Const>> speeds;
	  // Linear drag
	  double Cd = 0; /// Drag coefficient
	  std::shared_ptr<ChForce> lin_drag;
	  // Drag Surface
	  double Surf = 0;
	  // Drag torque, neglected
	  //std::shared_ptr<ChForce> drag_tor;

	  // Sensors:
	  // gps
	  // magnetometer
};

/// Following class will be used to manage events from the user interface

class MyEventReceiver : public IEventReceiver {
  public:
    MyEventReceiver(ChIrrAppInterface* myapp, ChCopter<6>* mycopter) {
        // store pointer to physical system & other stuff so we can tweak them by user keyboard
        app = myapp;
        copter = mycopter;
    }

    bool OnEvent(const SEvent& event) {
        // check if user presses keys
        if (event.EventType == irr::EET_KEY_INPUT_EVENT && !event.KeyInput.PressedDown) {
            switch (event.KeyInput.Key) {
                case irr::KEY_KEY_W:
                    // STATIC_x_speed -= 1.5;
                    std::cout << "Pressing W\n";

                    return true;
                case irr::KEY_KEY_A:
                    // STATIC_z_speed += 1.5;
                    std::cout << "Pressing A\n";
                    return true;
                default:
                    break;
            }
        }

        return false;
    }

  private:
    ChIrrAppInterface* app;
    ChCopter<6>* copter;
};

int main(int argc, char* argv[]) {
    GetLog() << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

    // Create a ChronoENGINE physical system
    ChSystemNSC mphysicalSystem;
    std::vector<ChVector<>> ppos;
    for (int i = 0; i < 6; i++) {
        double ang = CH_C_PI * (double(i) / 3) + CH_C_PI/6;
        double R = 0.762;
        ChVector<> pos(std::cos(ang) * R, 0, std::sin(ang) * R);
        ppos.push_back(pos);
    };
    bool rotations[] = {false, true, false, true, false, true} ;
	ChCopter<6> mycopter (mphysicalSystem,                             /// the Chrono physical system
							ChVector<>(0,0,0),       /// Chassis position
                            ppos,                                        /// Propeller relative position
                            rotations);  /// i-th propeller rotates clockwise -> true
		 
	mycopter.SetPropellersData(0.126, ChVector<>(0.004739, 0.004739, 0.004739), 0.6718, 0.0587, 0.018734, 4468);
    // Create the Irrlicht visualization (open the Irrlicht device,
    // bind a simple user interface, etc. etc.)
    ChIrrApp application(&mphysicalSystem, L"HexaCopter Test", core::dimension2d<u32>(800, 600), false);

    // create text with info
    IGUIStaticText* textFPS = application.GetIGUIEnvironment()->addStaticText(
        L"Use keys Q,W, A,Z, E,R to move the robot", rect<s32>(150, 10, 430, 40), true);

    // Easy shortcuts to add camera, lights, logo and sky in Irrlicht scene:
    ChIrrWizard::add_typical_Logo(application.GetDevice());
    ChIrrWizard::add_typical_Sky(application.GetDevice());
    ChIrrWizard::add_typical_Lights(application.GetDevice());
    ChIrrWizard::add_typical_Camera(application.GetDevice(), core::vector3df(0, 14, -20));

    // This is for GUI tweaking of system parameters..
    MyEventReceiver receiver(&application, &mycopter);
    // note how to add a custom event receiver to the default interface:
    application.SetUserEventReceiver(&receiver);

    // Create the ground for the collision
    auto ground_mat = chrono_types::make_shared<ChMaterialSurfaceNSC>();
    ground_mat->SetFriction(0.5);

    auto ground = chrono_types::make_shared<ChBodyEasyBox>(200, 1, 200,  // size
                                                           1000,         // density
                                                           true,         // visualize
                                                           true,         // collide
                                                           ground_mat);  // contact material
    ground->SetPos(ChVector<>(0, -5, 0));
    ground->SetBodyFixed(true);
	mphysicalSystem.Add(ground);

	auto mtexture = chrono_types::make_shared<ChTexture>();
    mtexture->SetTextureFilename(GetChronoDataFile("concrete.jpg"));
    mtexture->SetTextureScale(100, 100);

    ground->AddAsset(mtexture);



	// Use this function for adding a ChIrrNodeAsset to all already created items.
    // Otherwise use application.AssetBind(myitem); on a per-item basis.
    application.AssetBindAll();
    application.AssetUpdateAll();

    // Prepare the physical system for the simulation

    mphysicalSystem.SetTimestepperType(ChTimestepper::Type::EULER_IMPLICIT_PROJECTED);

    mphysicalSystem.SetSolverType(ChSolver::Type::PSOR);
    mphysicalSystem.SetSolverMaxIterations(30);

    //
    // THE SOFT-REAL-TIME CYCLE
    //

    application.SetTimestep(0.005);
    application.SetTryRealtime(true);
    double control[] = {.6, .6, .6, .6, .6, .6};
    mycopter.ControlAbsolute(control);
    while (application.GetDevice()->run()) {
        application.BeginScene(true, true, SColor(255, 140, 161, 192));

        application.DrawAll();
        mycopter.Update(0.01);
        // ADVANCE THE SIMULATION FOR ONE TIMESTEP
        application.DoStep();

        // change motor speeds depending on user setpoints from GUI
		
        
			
        application.EndScene();
    }

    return 0;
}
