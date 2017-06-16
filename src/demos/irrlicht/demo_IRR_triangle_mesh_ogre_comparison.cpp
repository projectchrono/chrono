#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChParticlesClones.h"
#include "chrono/assets/ChTexture.h"
#include "chrono/assets/ChTriangleMeshShape.h"
#include "chrono/geometry/ChTriangleMeshConnected.h"

#include "chrono_irrlicht/ChIrrApp.h"

// Use the namespaces of Chrono
using namespace chrono;
using namespace chrono::geometry;
using namespace chrono::irrlicht;

// Use the main namespaces of Irrlicht
using namespace irr;
using namespace irr::core;
using namespace irr::scene;
using namespace irr::video;

int main(int argc, char* argv[]) {
	// Create a ChronoENGINE physical system
	ChSystemNSC mphysicalSystem;

	// Create the Irrlicht visualization (open the Irrlicht device,
	// bind a simple user interface, etc. etc.)
	ChIrrApp application(&mphysicalSystem, L"Ogre / Irrlicht Comparison", core::dimension2d<u32>(1280, 720), false);

	// Easy shortcuts to add camera, lights, logo and sky in Irrlicht scene:
	ChIrrWizard::add_typical_Logo(application.GetDevice());
	ChIrrWizard::add_typical_Sky(application.GetDevice());
	ChIrrWizard::add_typical_Lights(application.GetDevice());
	ChIrrWizard::add_typical_Camera(application.GetDevice(), core::vector3df(0, 10, 50));


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
	masset_mesh->SetMesh(mmesh);
	masset_mesh->SetBackfaceCull(true);
	body->AddAsset(masset_mesh);

	mphysicalSystem.Add(body);


	application.AssetBindAll();

	application.AssetUpdateAll();

	application.SetTimestep(0.005);

	while (application.GetDevice()->run()) {
		application.GetVideoDriver()->beginScene(true, true, SColor(255, 140, 161, 192));

		application.DrawAll();

		application.DoStep();

		application.GetVideoDriver()->endScene();
	}

	return 0;
}