#include <chrono_ogre/Core/ChOgreApplication.h>

#include "chrono/physics/ChSystem.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/geometry/ChTriangleMeshConnected.h"
#include "chrono/assets/ChTexture.h"

using namespace chrono;
using namespace chrono::geometry;
using namespace chrono::ChOgre;

int main(int argc, char* argv[]) {
	// Create a ChronoENGINE physical system
	ChSystem mphysicalSystem;

	
	ChOgreApplication app;
	app.createWindow("Ogre / Irrlicht Comparison", 1280, 720, 4);

	auto Image = app.getGUIManager()->createWidget<ChOgreGUIImage>(chrono::ChVector2<>(0.0, 0.0), chrono::ChVector2<>(0.2, 0.2));
	Image->setImage("logo_projectchrono_alpha.png");

	app.getScene()->setSkyBox("sky");

	ChOgreLightHandle lightHandle = app.getScene()->createLight("Light");
	lightHandle->setType(ChOgreLight::POINT);
	lightHandle->setPosition(30.f, 100.f, 30.f);
	lightHandle->setDiffuse(0.7f, 0.7f, 0.7f);
	lightHandle->setSpecular(0.7f, 0.7f, 0.7f);

	ChOgreLightHandle lightHandle2 = app.getScene()->createLight("Light2");
	lightHandle2->setType(ChOgreLight::POINT);
	lightHandle2->setPosition(30.f, 80.f, -30.f);
	lightHandle2->setDiffuse(0.7f, 0.8f, 0.8f);
	lightHandle2->setSpecular(0.7f, 0.8f, 0.8f);

	ChOgreCamera* Camera = app.getCameraManager()->createCamera("Camera");

	Camera->setPosition(0.f, 10.f, 50.f);
	Camera->lookAt(0.0f, 0.0f, 0.0f);
	app.getCameraManager()->makeActive(Camera);


	collision::ChCollisionModel::SetDefaultSuggestedEnvelope(0.0025);
	collision::ChCollisionModel::SetDefaultSuggestedMargin(0.0025);

	// - Create a floor

	auto floor = std::make_shared<ChBodyEasyBox>(100, 1, 100, 1000, true, true);
	floor->SetBodyFixed(true);
	floor->SetPos(ChVector<>(0.0, 0.0, 0.0));

	mphysicalSystem.Add(floor);


	ChTriangleMeshConnected mmesh;
	mmesh.LoadWavefrontMesh(GetChronoDataFile("vehicle/hmmwv/hmmwv_chassis.obj"), false, true);
	mmesh.Transform(ChVector<>(-0.15, 0, 0), ChMatrix33<>(1.2));
	//	mmesh.RepairDuplicateVertexes(1e-9);

	double mmass;
	ChVector<> mcog;
	ChMatrix33<> minertia;
	mmesh.ComputeMassProperties(true, mmass, mcog, minertia);

	auto body = std::make_shared<ChBody>();

	body->SetPos(ChVector<>(0.0, 10.0, 0.0));
	body->SetMass(mmass);
	body->SetInertia(minertia);

	body->GetCollisionModel()->ClearModel();
	body->GetCollisionModel()->AddTriangleMesh(mmesh, false, false, VNULL, ChMatrix33<>(1), 0.005);
	body->GetCollisionModel()->BuildModel();
	body->SetCollide(true);

	auto masset_mesh = std::make_shared<ChTriangleMeshShape>();
	masset_mesh->SetName(GetChronoDataFile("vehicle/hmmwv/hmmwv_chassis.obj"));
	masset_mesh->SetMesh(mmesh);
	masset_mesh->SetBackfaceCull(true);
	body->AddAsset(masset_mesh);

	mphysicalSystem.Add(body);


	app.initializeFromSystem(mphysicalSystem);

	app.timestep = 0.005;
	app.isRealTime = false;

	while (app.isRunning()) {
		app.pollInput();
		app.drawFrame();

		app.doStep();
	}

	return 0;
}