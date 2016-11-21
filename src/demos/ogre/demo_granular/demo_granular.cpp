#include <chrono_ogre/Core/ChOgreApplication.h>

using namespace chrono::ChOgre;

int main(int argc, char** args) {
	ChOgreApplication app;

	app.createWindow("Test", 1280, 720, 0, false, false);

	ChOgreCamera* DebugCamera = app.getCameraManager()->createCamera("DebugCamera");

	DebugCamera->setPosition(-20.0f, 70.0f, -40.0f);
	DebugCamera->lookAt(0.0f, 0.0f, 0.0f);
	app.getCameraManager()->makeActive(DebugCamera);

	app.timestep_max = 0.005;
	app.isRealTime = false;

	ChOgreBodyHandle Floor = app.getScene()->spawnBox("Floor", 1, chrono::ChVector<>(0, 0, 0), chrono::ChVector<>(10, 0.5, 10), chrono::ChQuaternion<>(), true);
	ChOgreBodyHandle Wall1 = app.getScene()->spawnBox("Wall1", 1, chrono::ChVector<>(10, 4.5, 0), chrono::ChVector<>(0.5, 5, 10.5), chrono::ChQuaternion<>(), true);
	ChOgreBodyHandle Wall2 = app.getScene()->spawnBox("Wall2", 1, chrono::ChVector<>(-10, 4.5, 0), chrono::ChVector<>(0.5, 5, 10.5), chrono::ChQuaternion<>(), true);
	ChOgreBodyHandle Wall3 = app.getScene()->spawnBox("Wall3", 1, chrono::ChVector<>(0, 4.5, 10), chrono::ChVector<>(10.5, 5, 0.5), chrono::ChQuaternion<>(), true);
	ChOgreBodyHandle Wall4 = app.getScene()->spawnBox("Wall4", 1, chrono::ChVector<>(0, 4.5, -10), chrono::ChVector<>(10.5, 5, 0.5), chrono::ChQuaternion<>(), true);

	ChOgreLightHandle lightHandle = app.getScene()->createLight("Light");
	lightHandle->setType(ChOgreLight::POINT);
	lightHandle->setPosition(0.0f, 100.0f, 0.0f);
	lightHandle->setDiffuse(1.0f, 1.0f, 1.0f);
	lightHandle->setSpecular(1.0f, 1.0f, 1.0f);
	lightHandle->setDirection(0.0f, 0.0f, 0.0f);
	lightHandle->setIntensity(400.0f);

	app.getScene()->setSkyBox("sky");

	int spheresLeft = 2000;

	ChOgreApplication::ChOgreLoopCallFunc Loop = ChOgreFunc(void) {

		double variance = (double)(spheresLeft % 8);
		variance -= 3.0;

		if (spheresLeft > 0) {
			spheresLeft--;

			ChOgreBodyHandle Ball = app.getScene()->spawnSphere("Sphere", 1, chrono::ChVector<>(7, 20, 7), 0.5, false);
			Ball->SetInertiaXX(chrono::ChVector<>(
				((2.0 / 5.0)*Ball->GetMass() * 0.5 * 0.5),
				((2.0 / 5.0)*Ball->GetMass() * 0.5 * 0.5),
				((2.0 / 5.0)*Ball->GetMass() * 0.5 * 0.5)));

			Ball->SetPos_dt(chrono::ChVector<>(-7.0 + variance, -7.0, -7.0));
		}

		return 0;
	};

	app.startLoop(Loop);

	return 0;
}
