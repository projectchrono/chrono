#include <chrono_ogre/Core/ChOgreApplication.h>

using namespace chrono::ChOgre;

int main(int argc, char** args) {
	ChOgreApplication app;

	std::cout << "Window Creation\n";

	auto window = app.createWindow("Test", 1280, 720, 0, false, false);

	ChOgreCamera* DebugCamera = app.getCameraManager()->createCamera("DebugCamera");

	DebugCamera->setPosition(50.0f, 20.0f, -50.0f);
	DebugCamera->lookAt(0.0f, 0.0f, 0.0f);
	app.getCameraManager()->makeActive(DebugCamera);

	app.timestep_max = 0.01;
	app.isRealTime = false;

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

	app.getInputManager()->addCallback(EpsilonCallback);
	app.getInputManager()->addCallback(EpsilonCallback2);

	auto Image = app.getGUIManager()->createWidget<ChOgreGUIImage>(ChFloat3(0.f, 0.f, 0.f), ChFloat3(0.2f, 0.2f, 1.f));
	Image->setImage("logo_projectchrono_alpha.png");

	ChOgreApplication::ChOgreLoopCallFunc Loop = ChOgreFunc(void) {

		return 0;
	};

	app.startLoop(Loop);

	return 0;
}
