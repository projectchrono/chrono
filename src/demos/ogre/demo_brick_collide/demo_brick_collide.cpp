#include <chrono_ogre/Core/ChOgreApplication.h>

using namespace ChOgre;

int main(int argc, char** args) {
	ChOgreApplication app;
	
	app.createWindow("Test", 1280, 720, 0, false, false);

	ChOgreCamera* DebugCamera = app.getCameraManager()->createCamera("DebugCamera");

	DebugCamera->setPosition(100.0f, 20.0f, -100.0f);
	DebugCamera->lookAt(0.0f, 0.0f, 0.0f);
	app.getCameraManager()->makeActive(DebugCamera);

	app.timestep_max = 0.01;
	app.isRealTime = false;

	//app.WriteToFile = true;
	//app.OutputImageFolder = "demo_brick_collide_images";

	std::random_device l_rand;


	ChOgreBodyHandle Epsilon = app.getScene()->spawnSphere("Spheere", 10, chrono::ChVector<>(0, 20, -20), 3, false);
	Epsilon->SetInertiaXX(chrono::ChVector<>(
		((2.0 / 5.0)*Epsilon->GetMass() * 4.0 * 4.0),
		((2.0 / 5.0)*Epsilon->GetMass()  * 4.0 * 4.0),
		((2.0 / 5.0)*Epsilon->GetMass() * 4.0 * 4.0)));

	Epsilon->SetPos_dt(chrono::ChVector<>(0, 10, 80));

	for (float y = 0.f; y < 10.f; y += 1.f) {
		for (float x = 0.f; x < 8.f; x += 1.f) {
			ChOgreBodyHandle Brick = app.getScene()->spawnBox("Brick", 5, chrono::ChVector<>((8.f * x) - 24.f, (4 * y) + 1.25, 5.f), chrono::ChVector<>(4, 2, 2));
			Brick->SetInertiaXX(chrono::ChVector<>(
				((1.0 / 12.0)*Brick->GetMass() * (16 + 4)),
				((1.0 / 12.0)*Brick->GetMass()  * (4 + 4)),
				((1.0 / 12.0)*Brick->GetMass() * (4 + 16))));
            Brick->GetMaterialSurfaceNSC()->SetFriction(1.f);
			Brick.body().deletable = true;
		}
	}

	ChOgreBodyHandle Alpha = app.getScene()->spawnBox("Boox", 1, chrono::ChVector<>(0, 0, 0), chrono::ChVector<>(100, 0.5, 20), chrono::ChQuaternion<>(), true);

	ChOgreLightHandle yeh = app.getScene()->createLight("Swag");
	yeh->setType(ChOgreLight::POINT);
	yeh->setPosition(0.0f, 100.0f, 0.0f);
	yeh->setDiffuse(1.0f, 1.0f, 1.0f);
	yeh->setSpecular(1.0f, 1.0f, 1.0f);
	yeh->setDirection(0.0f, 0.0f, 0.0f);
	yeh->setIntensity(400.0f);

	ChOgreLightHandle yeh2 = app.getScene()->createLight("Yeh");
	yeh2->setType(ChOgreLight::POINT);
	yeh2->setPosition(0.0f, 30.0f, -5.0f);
	yeh2->setDiffuse(1.0f, 1.0f, 1.0f);
	yeh2->setSpecular(1.0f, 1.0f, 1.0f);
	yeh2->setDirection(0.0f, 0.0f, 0.0f);
	yeh2->setIntensity(800.0f);

	app.getScene()->setSkyBox("sky");
	
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

	app.getInputManager()->addCallback(EpsilonCallback);


	ChOgreApplication::ChOgreLoopCallFunc Loop = ChOgreFunc(void) {

		return 0;
	};

	app.startLoop(Loop);

	return 0;
}