#include <random>
#include <chrono>
#include <cmath>

#include <chrono_ogre/Core/ChOgreApplication.h>
#include <chrono_ogre/GUI/ChOgreGUIManager.h>
#include <chrono_ogre/GUI/ChOgreGUIText.h>
#include <chrono_ogre/GUI/ChOgreGUIButton.h>
#include "VESuspensionDemo.h"

int main(int argc, char *argv[]) {
		ChOgre::ChOgreApplication app;
		app.createWindow("Test", 1280, 720, 0, false, false);

		ChOgre::ChOgreCamera* DebugCamera = app.getCameraManager()->createCamera("DebugCamera");
		
		DebugCamera->setPosition(100.0f, 20.0f, -100.0f);
		DebugCamera->lookAt(0.0f, 0.0f, 0.0f);
		app.getCameraManager()->makeActive(DebugCamera);

		app.timestep_max = 0.01;
		app.isRealTime = false;

		std::random_device l_rand;

		app.getScene()->setSkyBox("sky");

		VehicleEnvironment::VESuspensionDemo car;
		car.setApp(&app);
		car.build(chrono::ChVector<>(0, 15, 0));


		ChOgre::ChOgreBodyHandle Epsilon = app.getScene()->spawnSphere("Spheere", 1, chrono::ChVector<>(10, 10, 20), 4);
		Epsilon->SetInertiaXX(chrono::ChVector<>(
			((2.0 / 5.0)*Epsilon->GetMass() * 4.0 * 4.0),
			((2.0 / 5.0)*Epsilon->GetMass() * 4.0 * 4.0),
			((2.0 / 5.0)*Epsilon->GetMass() * 4.0 * 4.0)));

		ChOgre::ChOgreBodyHandle Epsilon1 = app.getScene()->spawnSphere("Spheere1", 1, chrono::ChVector<>(30, 10, 20), 4);
		Epsilon1->SetInertiaXX(chrono::ChVector<>(
			((2.0 / 5.0)*Epsilon->GetMass() * 4.0 * 4.0),
			((2.0 / 5.0)*Epsilon->GetMass() * 4.0 * 4.0),
			((2.0 / 5.0)*Epsilon->GetMass() * 4.0 * 4.0)));

		ChOgre::ChOgreBodyHandle Epsilon2 = app.getScene()->spawnSphere("Spheere2", 1, chrono::ChVector<>(0, 10, 20), 4);
		Epsilon2->SetInertiaXX(chrono::ChVector<>(
			((2.0 / 5.0)*Epsilon->GetMass() * 4.0 * 4.0),
			((2.0 / 5.0)*Epsilon->GetMass() * 4.0 * 4.0),
			((2.0 / 5.0)*Epsilon->GetMass() * 4.0 * 4.0)));

		/*EnvironmentCore::ECBody& Epsilon3 = app.getScene()->spawnSphere("Spheere3", 1, chrono::ChVector<>(50, 10, 50), 4);
		Epsilon2->SetInertiaXX(chrono::ChVector<>(
			((2.0 / 5.0)*Epsilon->GetMass() * 4.0 * 4.0),
			((2.0 / 5.0)*Epsilon->GetMass() * 4.0 * 4.0),
			((2.0 / 5.0)*Epsilon->GetMass() * 4.0 * 4.0)));

		EnvironmentCore::ECBody& Epsilon4 = app.getScene()->spawnBox("Spheere4", 10, chrono::ChVector<>(-20, 10, -70), chrono::ChVector<>(4, 4, 4));
		Epsilon2->SetInertiaXX(chrono::ChVector<>(
			((1.0 / 12.0)*Epsilon->GetMass() * (16.0 + 16.0)),
			((1.0 / 12.0)*Epsilon->GetMass() * (16.0 + 16.0)),
			((1.0 / 12.0)*Epsilon->GetMass() * (16.0 + 16.0))));*/

		ChOgre::ChOgreBodyHandle Building = app.getScene()->spawnBox("Building1", 50000, chrono::ChVector<>(0, 490, 100), chrono::ChVector<>(20, 500, 20), chrono::ChQuaternion<>(1, 0, 0, 0), true);

		ChOgre::ChOgreBodyHandle hillsyo = app.getScene()->loadHeightMap("example3.png", chrono::ChVector<>(10, 20, 10));
		hillsyo->SetPos(chrono::ChVector<>(0, 0, 0));
        hillsyo->GetMaterialSurfaceNSC()->SetFriction(0.9);
		//ChOgre::ChOgreBodyHandle Alpha = app.getScene()->spawnBox("Boox", 1, chrono::ChVector<>(0, 0, 0), chrono::ChVector<>(50, 0.5, 50), chrono::ChQuaternion<>(), true);

		/*EnvironmentCore::ECBody& Theta = app.getScene()->spawnEllipsoid("Theta", 1.0, chrono::ChVector<>(0, 30, 0), chrono::ChVector<>(2, 5, 2));
		Theta->SetInertiaXX(chrono::ChVector<>(
			((1.0 / 5.0)*Theta->GetMass() * (std::pow(1, 2) + std::pow(1, 2.5))),
			((1.0 / 5.0)*Theta->GetMass() * (std::pow(1, 2) + std::pow(1, 2.5))),
			((1.0 / 5.0)*Theta->GetMass() * (std::pow(1, 2) + std::pow(1, 2)))));

		EnvironmentCore::ECBody& Alpha = app.getScene()->spawnBox("Boox", 100, chrono::ChVector<>(0, 100, 0), chrono::ChVector<>(2, 2, 2));
		Alpha->SetInertiaXX(chrono::ChVector<>( 
			((1.0 / 12.0)*Alpha->GetMass() * 8.0),
			((1.0 / 12.0)*Alpha->GetMass() * 8.0),
			((1.0 / 12.0)*Alpha->GetMass() * 8.0) ));

		EnvironmentCore::ECBody& Omega = app.getScene()->spawnCylinder("Coone", 3, chrono::ChVector<>(0, 200, 0), chrono::ChVector<>(2, 5, 2));
		Omega->SetInertiaXX(chrono::ChVector<>(
			((1.0 / 12.0)*Omega->GetMass() * 37.0),
			((1.0 / 12.0)*Omega->GetMass() * 37.0),
			((1.0 / 2.0)*Omega->GetMass() * 4.0)));

		for (int i = 0; i < 200; i++) {
			EnvironmentCore::ECBody& Beta = app.getScene()->spawnSphere("", 1, chrono::ChVector<>(((double)l_rand() / (double)l_rand.max()) * 10, 20 + (i * 5), ((double)l_rand() / (double)l_rand.max()) * 10), 1);
			Beta->SetInertiaXX(chrono::ChVector<>(
				((2.5 / 5.0)*Epsilon->GetMass() * 1.0),
				((2.0 / 5.0)*Epsilon->GetMass() * 1.0),
				((2.0 / 5.0)*Epsilon->GetMass() * 1.0)));
		}*/

		

		ChOgre::ChOgreLightHandle yeh = app.getScene()->createLight("Swag");
		yeh->setType(ChOgre::ChOgreLight::POINT);
		yeh->setPosition(0.0f, 100.0f, 0.0f);
		yeh->setDiffuse(1.0f, 1.0f, 0.0f);
		yeh->setSpecular(1.0f, 1.0f, 0.0f);
		yeh->setDirection(0.0f, 0.0f, 0.0f);
		yeh->setIntensity(400.0f);

		ChOgre::ChOgreLightHandle yeh2 = app.getScene()->createLight("Que");
		yeh2->setType(ChOgre::ChOgreLight::POINT);
		yeh2->setPosition(500.0f, 500.0f, 500.0f);
		yeh2->setDiffuse(1.0f, 0.0f, 1.0f);
		yeh2->setSpecular(1.0f, 0.0f, 1.0f);
		yeh2->setDirection(0.0f, 0.0f, 0.0f);
		yeh2->setIntensity(400.0f);

		ChOgre::ChOgreLightHandle yeh3 = app.getScene()->createLight("Holo");
		yeh3->setType(ChOgre::ChOgreLight::POINT);
		yeh3->setPosition(500.0f, 800.0f, -800.0f);
		yeh3->setDiffuse(0.0f, 1.0f, 1.0f);
		yeh3->setSpecular(0.0f, 1.0f, 1.0f);
		yeh3->setDirection(0.0f, 0.0f, 0.0f);
		yeh3->setIntensity(400.0f);

		ChOgre::ChOgreLightHandle follow = app.getScene()->createLight("Follow");
		follow->setType(ChOgre::ChOgreLight::POINT);
		follow->setPosition(1.0f, 1.0f, 1.0f);
		follow->setSpecular(1.0f, 1.0f, 1.0f);


		chrono::ChVector<> direction = chrono::ChVector<>(0, 0, 5);
		chrono::ChQuaternion<> dirRot = car.getChassis()->GetRot();
		dirRot.Normalize();
		direction = dirRot.Rotate(chrono::ChVector<>(0, 0, 5));

		chrono::ChVector<> camera_pos;
		chrono::ChVector<> camera_tpos;
		chrono::ChVector<> look_at;
		chrono::ChVector<> camera_vel;

		//app.getScene()->setAmbientLight(1.0f, 1.0f, 1.0f);

		std::chrono::high_resolution_clock l_time;
		auto l_start = l_time.now();

		double throttle = 0; // actual value 0...1 of gas throttle.

		bool db = true;
		bool db2 = true;
		bool db3 = true;
		bool pshiftdb = true;

		unsigned int deleteSpheres = 0;

		app.getChSystem()->SetMaxItersSolverSpeed(800);
		app.getChSystem()->SetMaxPenetrationRecoverySpeed(100000);
		//app.getChSystem()->SetSolverType(chrono::ChSystem::SOLVER_SYMMSOR);
		app.getChSystem()->SetTol(0);


		app.getInputManager()->AxisThreshold = 0.1;


		ChOgre::ChOgreGUITextPtr p = app.getGUIManager()->createWidget<ChOgre::ChOgreGUIText>(ChOgre::ChFloat3(), ChOgre::ChFloat3());
		p->setPosition(ChOgre::ChFloat3(0, 0, 0));
		p->setColor(1.0, 1.0, 1.0);
		p->setText("");

		ChOgre::ChOgreGUITextPtr p2 = app.getGUIManager()->createWidget<ChOgre::ChOgreGUIText>(ChOgre::ChFloat3(), ChOgre::ChFloat3());
		p2->setPosition(ChOgre::ChFloat3(0, 0.06, 0));
		p2->setColor(1.0, 1.0, 1.0);
		p2->setText("");

		ChOgre::ChOgreGUITextPtr p3 = app.getGUIManager()->createWidget<ChOgre::ChOgreGUIText>(ChOgre::ChFloat3(), ChOgre::ChFloat3());
		p3->setPosition(ChOgre::ChFloat3(0, 0.12, 0));
		p3->setColor(1.0, 1.0, 1.0);
		p3->setText("");

		ChOgre::ChOgreGUITextPtr p4 = app.getGUIManager()->createWidget<ChOgre::ChOgreGUIText>(ChOgre::ChFloat3(), ChOgre::ChFloat3());
		p4->setPosition(ChOgre::ChFloat3(0, 0.18, 0));
		p4->setColor(1.0, 1.0, 1.0);
		p4->setText("");

		ChOgre::ChOgreGUIButtonPtr t = app.getGUIManager()->createWidget<ChOgre::ChOgreGUIButton>(ChOgre::ChFloat3(), ChOgre::ChFloat3());
		t->setPosition(ChOgre::ChFloat3(0, 0.24, 0));
		t->setText("Button");
		t->setTextColor(1.0, 1.0, 1.0);
		t->setColor(0.0, 0.0, 0.0);

		ChOgre::ChOgreGUIClickCallback t_c;
		t_c.call = [&car](MyGUI::WidgetPtr) {
			car.reset(chrono::ChVector<>(0, 0, 0));
		};

		std::chrono::high_resolution_clock l_clock;
		auto start = l_clock.now();

		

		std::chrono::duration<double> end = std::chrono::duration_cast<std::chrono::duration<double>>(l_clock.now() - start);

		app.logMessage("\n\n Loaded heightmap in " + std::to_string(end.count()) + " seconds \n\n");


		bool fire_button = false;
		bool reset_button = false;
		bool rumble_button = false;
		double steer = 0.0;
		double clutch = 0.0;
		double accelerator = 0.0;
		double brake = 0.0;
		bool shift_up = false;
		bool shift_down = false;
		bool quit_button = false;
		bool toggle_realtime = false;
		bool tr_db = true;


		ChOgre::ChOgreApplication::ChOgreLoopCallFunc Loop = ChOgreFunc(void) {

			if (app.getInputManager()->getWheelState().active) {
				fire_button = app.getInputManager()->getWheelState().rwheelb1.down;
				reset_button = app.getInputManager()->getWheelState().red1.down;
				rumble_button = app.getInputManager()->getWheelState().rwheelb2.down;
				steer = 0.07*((double)(-1.0 * app.getInputManager()->getWheelState().wheel.value));
				clutch = app.getInputManager()->getWheelState().clutch.value;
				accelerator = app.getInputManager()->getWheelState().accelerator.value;
				brake = app.getInputManager()->getWheelState().brake.value;
				shift_up = app.getInputManager()->getWheelState().rpaddle.down;
				shift_down = app.getInputManager()->getWheelState().lpaddle.down;
				quit_button = app.getInputManager()->getWheelState().red2.down;
				toggle_realtime = app.getInputManager()->getWheelState().red3.down;
			}
			else {
				fire_button = app.getInputManager()->getControllerState().x.down;
				reset_button = app.getInputManager()->getControllerState().y.down;
				rumble_button = app.getInputManager()->getControllerState().lbumper.down;
				steer = 0.07*((double)(-1.0 * app.getInputManager()->getControllerState().lstickx.value));
				clutch = 0.0;
				accelerator = (app.getInputManager()->getControllerState().rtrigger.value + 1.0000015) / 2;
				brake = (app.getInputManager()->getControllerState().ltrigger.value + 1.0000015) / 2;
				shift_up = app.getInputManager()->getControllerState().d_up.down;
				shift_down = app.getInputManager()->getControllerState().d_down.down;
				quit_button = app.getInputManager()->getControllerState().back.down;
				toggle_realtime = app.getInputManager()->getControllerState().rbumper.down;
			}

			if (abs(brake) < 0.5) {
				brake = 0.0;
			}


			if (toggle_realtime && tr_db) {
				app.isRealTime = app.isRealTime ? false : true;
				tr_db = false;
			}
			
			if (!toggle_realtime) {
				tr_db = true;
			}

			if (fire_button && db) {
				ChOgre::ChOgreBodyHandle Alpha = app.getScene()->spawnSphere("Boox", 50, chrono::ChVector<>(car.getChassis()->GetPos().x, car.getChassis()->GetPos().y + 3, car.getChassis()->GetPos().z), 0.1);
				Alpha->SetInertiaXX(chrono::ChVector<>(
					((2.0 / 5.0)*Alpha->GetMass() * 0.3 * 0.3),
					((2.0 / 5.0)*Alpha->GetMass() * 0.3 * 0.3),
					((2.0 / 5.0)*Alpha->GetMass() * 0.3 * 0.3)));

				auto dir = car.getChassis()->GetRot().Rotate(chrono::ChVector<>(0, 0, 1));

				Alpha->SetPos_dt(dir * 125);

				db = false;
			}

			if (!fire_button) {
				db = true;
			}

			if (reset_button && db3) {
				car.reset(chrono::ChVector<>(4, 2, 4));
				db3 = false;
			}

			if (!reset_button) {
				db3 = true;
			}

			deleteSpheres++;

			if (deleteSpheres > 500) {
				app.getScene()->removeBody("Boox");
				deleteSpheres = 0;
			}


			if (rumble_button && db2) {
				app.getInputManager()->runHapticRumble(1.0f, 1);
				db2 = false;
			}
			if (!rumble_button) {
				db2 = true;
			}

			if (steer > 0.1) {
				steer = 0.1;
			}
			else if (steer < -0.1) {
				steer = -0.1;
			}

			car.steer = steer;


			if (clutch > 0.7) {
				throttle = 0;
				if (app.getInputManager()->getWheelState().gear1.down) {
					car.shift(1);
				}
				else if (app.getInputManager()->getWheelState().gear2.down) {
					car.shift(2);
				}
				else if (app.getInputManager()->getWheelState().gear3.down) {
					car.shift(3);
				}
				else if (app.getInputManager()->getWheelState().gear4.down) {
					car.shift(4);
				}
				else if (app.getInputManager()->getWheelState().gear5.down) {
					car.shift(5);
				}
				else if (app.getInputManager()->getWheelState().gear6.down) {
					car.shift(6);
				}
				else if (app.getInputManager()->getWheelState().reverse.down) {
					car.shift(0);
				}
			}

			if (shift_down && pshiftdb && (car.gear > 0)) {
				car.shift(car.gear-1);
				pshiftdb = false;
			}

			if (shift_up && pshiftdb && (car.gear < 6)) {
				car.shift(car.gear+1);
				pshiftdb = false;
			}

			if (!shift_down && !shift_up) {
				pshiftdb = true;
			}

			if (accelerator) {
				throttle = accelerator;
				if (app.getInputManager()->getWheelState().reverse.down) {
					//throttle *= -1;
				}
			}

			if (brake) {
				car.brake();
			}

			if (app.getInputManager()->getKeyState(SDL_SCANCODE_ESCAPE).down || quit_button) {
				//return 1;
			}

			car.throttle = throttle;

			car.update();

			

			double speed = car.getChassis()->GetPos_dt().Length();

			speed = speed * (3600.0 / 1000.0);

			p->setText("Velocity: " + std::to_string(speed) + " km/h");
			std::string l_gear = std::to_string(car.gear);
			if (car.gear == 0) {
				l_gear = "Reverse";
			}
			p2->setText("Gear: " + l_gear);
			p3->setText("Throttle: " + std::to_string(throttle));
			p4->setText("FPS: " + std::to_string(app.getWindow()->getAverageFPS()));

			follow->setDiffuse(1.0f, 1.0f, 1.0f);
			follow->setSpecular(1.0f, 1.0f, 1.0f);

			dirRot = car.getChassis()->GetRot();
			dirRot.Normalize();
			direction = dirRot.Rotate(chrono::ChVector<>(0, 10, -20));

			look_at = car.getChassis()->GetPos();
			camera_tpos = (car.getChassis()->GetPos() + direction);

			auto camera_dpos = camera_tpos - camera_pos;

			auto fSpring = (camera_dpos * 200) - (camera_vel * 200);

			auto cam_accel = fSpring / 5;

			camera_vel = camera_vel + cam_accel * app.timestep;

			camera_pos = camera_pos + camera_vel * app.timestep + 0.5 * cam_accel * app.timestep * app.timestep;
			
			//DebugCamera->orient(camera_pos.x, camera_pos.y, camera_pos.z, look_at.x, look_at.y+6, look_at.z);
			DebugCamera->setPosition(camera_pos);
			DebugCamera->lookAt(look_at);

			follow->setPosition(car.getChassis()->GetPos().x, car.getChassis()->GetPos().y + 10, car.getChassis()->GetPos().z + 14);

			return 0;
		};

		

		app.startLoop(Loop);
		app.logMessage("end of the program");
	
	return 0;
}
