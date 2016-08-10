#include <chrono_ogre/Core/ChOgreApplication.h>

using namespace chrono::ChOgre;

int main(int argc, char** args) {
	ChOgreApplication app;
	
	app.createWindow("Test", 1280, 720, 0, false, false);

	ChOgreCamera* DebugCamera = app.getCameraManager()->createCamera("DebugCamera");

	DebugCamera->setPosition(50.0f, 20.0f, -50.0f);
	DebugCamera->lookAt(0.0f, 0.0f, 0.0f);
	app.getCameraManager()->makeActive(DebugCamera);

	app.timestep_max = 0.01;
	app.isRealTime = false;

	std::random_device l_rand;

	//ChOgreBodyHandle terrain = app.getScene()->loadHeightMap("example3.png", chrono::ChVector<>(20, 10, 20));
	//terrain->SetPos(chrono::ChVector<>(0, 0, 0));
	//terrain->GetMaterialSurface()->SetFriction(0.9);

	ChOgreBodyHandle Epsilon = app.getScene()->spawnSphere("Sphere", 1, chrono::ChVector<>(0, 20, 0), 3, false);
	Epsilon->SetInertiaXX(chrono::ChVector<>(
		((2.0 / 5.0)*Epsilon->GetMass() * 4.0 * 4.0),
		((2.0 / 5.0)*Epsilon->GetMass() * 4.0 * 4.0),
		((2.0 / 5.0)*Epsilon->GetMass() * 4.0 * 4.0)));

	ChOgreBodyHandle Alpha = app.getScene()->spawnBox("Box", 1, chrono::ChVector<>(0, 0, 0), chrono::ChVector<>(10, 0.5, 10), chrono::ChQuaternion<>(), true);

	ChOgreLightHandle lightHandle = app.getScene()->createLight("Light");
	lightHandle->setType(ChOgreLight::POINT);
	lightHandle->setPosition(0.0f, 100.0f, 0.0f);
	lightHandle->setDiffuse(1.0f, 1.0f, 1.0f);
	lightHandle->setSpecular(1.0f, 1.0f, 1.0f);
	lightHandle->setDirection(0.0f, 0.0f, 0.0f);
	lightHandle->setIntensity(400.0f);

	std::cout<<"about to add skybox"<<std::endl;

	app.getScene()->setSkyBox("sky"); //does not exist in chrono data folder
	
	std::cout<<"finished adding skybox)"<<std::endl;

	ChOgreKeyboardCallback EpsilonCallback;
	EpsilonCallback.call = [&Epsilon](scancode_t ScanCode, keycode_t KeyCode, const ChOgreKeyState& KeyState) {
		if (KeyCode == SDLK_SPACE) {
			if (KeyState.down) {
				Epsilon->SetPos(chrono::ChVector<>(0, 20, 0));
			}
			else if (!KeyState.down) {
				Epsilon->SetPos_dt(chrono::ChVector<>(0, 40, 0));
			}
		}
	};

	ChOgreKeyboardCallback EpsilonCallback2;
	EpsilonCallback2.call = [&Epsilon](scancode_t ScanCode, keycode_t KeyCode, const ChOgreKeyState& KeyState) {
		if (KeyCode == SDLK_w) {
			if (KeyState.down) {
				Epsilon->SetPos_dt(chrono::ChVector<>(10, 0, 0));
			}
			else if (!KeyState.down) {
				Epsilon->SetPos_dt(chrono::ChVector<>(-10, 0, 0));
			}
		}
	};

	std::cout<<"About to do app.getInputManager()"<<std::endl;

	app.getInputManager()->addCallback(EpsilonCallback);
	app.getInputManager()->addCallback(EpsilonCallback2);

	std::cout<<"About to do loop"<<std::endl;

	ChOgreApplication::ChOgreLoopCallFunc Loop = ChOgreFunc(void) {

		return 0;
	};

	app.startLoop(Loop);

	return 0;
}
