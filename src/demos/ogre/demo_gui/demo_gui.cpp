#include <chrono_ogre/Core/ChOgreApplication.h>
#include <cmath>
#include <chrono_ogre/GUI/ChOgreGUIButton.h>
#include <chrono_ogre/GUI/ChOgreGUIText.h>

using namespace ChOgre;

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

	ChOgreGUIPressCallback bca;
	bca.call = [](MyGUI::WidgetPtr Sender, int x, int y, MyGUI::MouseButton b) -> void {
		((MyGUI::Button*)Sender)->setTextColour(MyGUI::Colour(1.f, 0.f, 0.f));
		std::cout << "Button Clicked\n";
	};

	auto button = app.getGUIManager()->createWidget<ChOgreGUIButton>(ChFloat3(0, 0, 0), ChFloat3(0.1, 0.05, 0));
	button->setColor(1.f, 1.f, 1.f);
	button->setText("Button");
	button->setPressCallback(bca);

	auto text = app.getGUIManager()->createWidget<ChOgreGUIText>(ChFloat3(0, 0.2, 0), ChFloat3(0.2, 0.2, 0));
	text->setTextColor(1.f, 0.f, 1.f);
	text->setText("Text");

	ChOgreBodyHandle Epsilon = app.getScene()->spawnSphere("Spheere", 1, chrono::ChVector<>(0, 5, 0), 3, false);
	Epsilon->SetInertiaXX(chrono::ChVector<>(
		((2.0 / 5.0)*Epsilon->GetMass() * 4.0 * 4.0),
		((2.0 / 5.0)*Epsilon->GetMass() * 4.0 * 4.0),
		((2.0 / 5.0)*Epsilon->GetMass() * 4.0 * 4.0)));

	ChOgreBodyHandle Alpha = app.getScene()->spawnBox("Boox", 1, chrono::ChVector<>(0, 0, 0), chrono::ChVector<>(50, 0.5, 50), chrono::ChQuaternion<>(), true);

	ChOgreLight& yeh = app.getScene()->createLight("Swag");
	yeh.setType(ChOgreLightTypes::LT_POINT);
	yeh.setPosition(0.0f, 100.0f, 0.0f);
	yeh.setDiffuseColour(1.0f, 1.0f, 1.0f);
	yeh.setSpecularColour(1.0f, 1.0f, 1.0f);
	yeh.setDirection(0.0f, 0.0f, 0.0f);
	yeh.setPowerScale(400.0f);

	app.getScene()->setSkyBox("sky");

	double throttle = 0;
	double steer = 0;
	typedef double angle;

	bool mouse = false;

	angle direction = 0;

	chrono::ChVector<> mod;
	const double deg_to_rad = std::_Pi / 180.0;

	ChOgreApplication::ChOgreLoopCallFunc Loop = ChOgreFunc(void) {

		steer = app.getInputManager()->getWheelState().wheel.value * 0.25;
		throttle = app.getInputManager()->getWheelState().accelerator.value;

		direction += steer;
		if (direction > 360) {
			direction = 0;
		}
		else if (direction < 0) {
			direction = 360;
		}

		mod.x = std::cos(direction * deg_to_rad);
		mod.z = std::sin(direction * deg_to_rad);


		DebugCamera->setPosition(Epsilon->GetPos().x - (mod.x * 40), 20, Epsilon->GetPos().z - (mod.z * 40));
		DebugCamera->lookAt(Epsilon->GetPos().x, Epsilon->GetPos().y, Epsilon->GetPos().z);

		mod *= throttle;

		Epsilon->SetPos_dt(mod);
		//Epsilon->SetPos_dtdt(mod);
		return 0;
	};

	app.startLoop(Loop);

	return 0;
}