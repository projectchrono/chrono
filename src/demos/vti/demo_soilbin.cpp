///////////////////////////////////////////////////
//
//   Demo code about  
//
//     - Creating a soil bin
//	   - Driving a kinematic roller/tire over the material
//     - analyzing the tire/soil interaction, forces on the wheel joint
//		- Modified demo_tire.cpp, originally by Alessandro Tasora, to use his tire collision geometry
//		- new version by Justin Madsen, interactive demo of a tire in a soil bin
//		- 
///////////////////////////////////////////////////
 
   
#include "physics/ChApidll.h" 
#include "physics/ChSystem.h"
#include "irrlicht_interface/ChBodySceneNode.h"
#include "irrlicht_interface/ChBodySceneNodeTools.h" 
#include "irrlicht_interface/ChIrrAppInterface.h"
#include "core/ChRealtimeStep.h"

#include <irrlicht.h>

using namespace chrono;
using namespace irr;
         
using namespace core;
using namespace scene; 
using namespace video;
using namespace io; 
using namespace gui; 

#include <direct.h>
std::string get_working_path()
{
   char temp[255];
   return ( getcwd(temp, 255) ? std::string( temp ) : std::string("") );
}

class SoilbinWheel {
public:
	//data
	ChBodySceneNode* wheel;

	// functions
	SoilbinWheel(ChVector<> mposition, ChIrrAppInterface& mapplication)
	{
		ChCollisionModel::SetDefaultSuggestedEnvelope(0.005);
		ChCollisionModel::SetDefaultSuggestedMargin  (0.004);

		// the mesh for the visualization (independent from the collision shape)
		IAnimatedMesh*	tireMesh = mapplication.GetSceneManager()->getMesh("../data/tractor_wheel.obj");

		wheel = (ChBodySceneNode*)addChBodySceneNode(
										mapplication.GetSystem(), mapplication.GetSceneManager(),
										tireMesh, // this mesh only for visualization
										50.0, 
										mposition  );

		wheel->GetBody()->SetInertiaXX(ChVector<>(10,10,10));
		wheel->GetBody()->SetFriction(0.5);

		// turn collision ON, otherwise no collide by default	
		wheel->GetBody()->SetCollide(false);	 


			// Clear model. The colliding shape description MUST be between  ClearModel() .. BuildModel() pair.
		wheel->GetBody()->GetCollisionModel()->ClearModel();
			// Describe the (invisible) colliding shape by adding the 'carcass' decomposed shape and the 
			// 'knobs'. Since these decompositions are only for 1/15th of the wheel, use for() to pattern them.

		for (double mangle = 0; mangle < 360.; mangle+= (360./15.))
		{
			ChQuaternion<>myrot;
			ChStreamInAsciiFile myknobs("../data/tractor_wheel_knobs.chulls");
			ChStreamInAsciiFile myslice("../data/tractor_wheel_slice.chulls");
			myrot.Q_from_AngAxis(mangle*(CH_C_PI/180.),VECT_X);
			ChMatrix33<> mm(myrot);
			wheel->GetBody()->GetCollisionModel()->AddConvexHullsFromFile(myknobs,&ChVector<>(0,0,0),&mm);
			wheel->GetBody()->GetCollisionModel()->AddConvexHullsFromFile(myslice,&ChVector<>(0,0,0),&mm);
			//break;
		}
	
			// Complete the description.
		wheel->GetBody()->GetCollisionModel()->BuildModel();
	}

	ChBodySceneNode* getWheelBody() {
		return wheel;
	}

	~SoilbinWheel() {
		// ChSystem* msys = wheel->GetBody()->GetSystem();
		// remove the bodies, joints
		wheel->remove();

	}
};

class TestMech {
public:
	// data
	ChBodySceneNode* truss;
	ChBodySceneNode* floor;
	ChBodySceneNode* wall1;
	ChBodySceneNode* wall2;
	ChBodySceneNode* wall3;
	ChBodySceneNode* wall4;
	ChSharedPtr<ChLinkSpring> spring;

	// functions
	TestMech(ChBodySceneNode* wheelBody, ChIrrAppInterface& mapp,
		double binWidth = 1.0, double binLength = 2.0, 
		double particleRad = 0.02, double particleStd = 0.02, double nParticles = 50)
	{
		// create the soil bin, some particles, and the testing Mechanism
		video::ITexture* cubeMap = mapp.GetVideoDriver()->getTexture("../data/concrete.jpg");
		video::ITexture* rockMap = mapp.GetVideoDriver()->getTexture("../data/rock.jpg");

		ChCollisionModel::SetDefaultSuggestedEnvelope(0.003);
		ChCollisionModel::SetDefaultSuggestedMargin  (0.002);

		ChQuaternion<> rot;
		rot.Q_from_AngAxis(ChRandom()*CH_C_2PI, VECT_Y);

		double bed_x = binWidth;
		double bed_z = binLength;

		// *******
		// Create a soil bin with planes.
		// Note: soil bin depth will always be ~ 1m
		// *******
		double binHeight = 1.0;
		double wallWidth = std::min<double>(binWidth, binLength) / 10.0;	// wall width = 1/10 of min of bin dims
		// create the floor
		floor = (ChBodySceneNode*)addChBodySceneNode_easyBox(
			mapp.GetSystem(), mapp.GetSceneManager(), 
			100.0,	ChVector<>(0, -0.5 - wallWidth / 2.0, 0), ChQuaternion<>(1,0,0,0), 
			ChVector<>( binWidth+wallWidth/2.0, wallWidth, binLength+wallWidth/2.0) );
		floor->GetBody()->SetBodyFixed(true);
		floor->GetBody()->SetFriction(0.5);	// NOTE: May need to change this if the walls have effects on the soil response
		floor->setMaterialTexture(0,	cubeMap);
		// add some transparent walls to the soilBin, w.r.t. width, length of bin
		// wall 1
		wall1 = (ChBodySceneNode*)addChBodySceneNode_easyBox(
			mapp.GetSystem(), mapp.GetSceneManager(), 
			100.0, ChVector<>(-binWidth/2.0 - wallWidth/2.0, 0, 0),	ChQuaternion<>(1,0,0,0), 
			ChVector<>(wallWidth, binHeight, binLength) );
		wall1->GetBody()->SetBodyFixed(true);
		wall1->setMaterialTexture(0,	cubeMap);

		// wall 2
		wall2 = (ChBodySceneNode*)addChBodySceneNode_easyBox(
			mapp.GetSystem(), mapp.GetSceneManager(), 
			100.0,	ChVector<>(binWidth/2.0 + wallWidth/2.0, 0, 0),	ChQuaternion<>(1,0,0,0), 
			ChVector<>(wallWidth,binHeight,binLength) );
		wall2->GetBody()->SetBodyFixed(true);
		wall2->setMaterialTexture(0,	rockMap);

		// grab a few more textures to differentiate the walls
		video::ITexture* wall3tex = mapp.GetVideoDriver()->getTexture("../data/bluwhite.png");
		video::ITexture* wall4tex = mapp.GetVideoDriver()->getTexture("../data/logo_chronoengine_alpha.png");
		// wall 3
		wall3 = (ChBodySceneNode*)addChBodySceneNode_easyBox(
			mapp.GetSystem(), mapp.GetSceneManager(), 
			100.0, ChVector<>(0, 0, -binLength/2.0 - wallWidth/2.0), ChQuaternion<>(1,0,0,0), 
			ChVector<>(binWidth + wallWidth/2.0, binHeight, wallWidth) );
		wall3->GetBody()->SetBodyFixed(true);
		wall3->setMaterialTexture(0,	wall3tex);

		// wall 4
		wall4 = (ChBodySceneNode*)addChBodySceneNode_easyBox(
			mapp.GetSystem(), mapp.GetSceneManager(), 
			100.0,	ChVector<>(0,0, binLength/2.0 + wallWidth/2.0),	ChQuaternion<>(1,0,0,0), 
			ChVector<>(binWidth + wallWidth/2.0, binHeight, wallWidth) );
		wall4->GetBody()->SetBodyFixed(true);
		wall4->setMaterialTexture(0,	wall4tex);

		// ******
		// put some particles in the bin initially
		// ******
		ChBodySceneNode* tempBody;
		for (int bi = 0; bi < nParticles; bi++) 
		{    
			double sphrad = particleRad + particleStd*ChRandom();
			double sphdens = 1;
			double sphmass = (4/3)*CH_C_PI*pow(sphrad,3)*sphdens;
			double sphinertia = pow(sphrad,2) * sphmass;
			ChQuaternion<> randrot(ChRandom(),ChRandom(),ChRandom(),ChRandom());
			randrot.Normalize();

			tempBody = (ChBodySceneNode*)addChBodySceneNode_easySphere(
					mapp.GetSystem(), mapp.GetSceneManager(),
					(4/3)*CH_C_PI*pow(sphrad,3)*sphdens,
					ChVector<>(-0.5*bed_x+ChRandom()*bed_x, 0.01+ 0.04*((double)bi/(double)nParticles), -0.5*bed_z+ChRandom()*bed_z),
					sphrad);

			tempBody->GetBody()->SetInertiaXX(ChVector<>(sphinertia,sphinertia,sphinertia));
			tempBody->GetBody()->SetFriction(0.4f);
			tempBody->GetBody()->SetRot(randrot);
			//tempBody->addShadowVolumeSceneNode();
			tempBody->setMaterialTexture(0,	rockMap);

		} 

		// ******
		// make a truss, allow it to move w.r.t. ground using a translational joint, connect it to the wheel
		// *****
		video::ITexture* bluMap = mapp.GetVideoDriver()->getTexture("../data/blu.jpg");
		ChVector<> trussCM = wheelBody->GetBody()->GetPos();
		ChBodySceneNode* truss = (ChBodySceneNode*)addChBodySceneNode_easyBox(mapp.GetSystem(), mapp.GetSceneManager(),
			1.0, trussCM, QUNIT, ChVector<>(0.2,0.2,0.5));
		truss->setMaterialTexture(0,bluMap);
		// truss shouldn't be used for Collisions
		truss->GetBody()->SetCollide(false);

		// create the translational joint between the truss and ground
		ChSharedPtr<ChLinkLockPrismatic> translational(new ChLinkLockPrismatic);
		translational->Initialize(truss->GetBody(), floor->GetBody(),
			ChCoordsys<>(trussCM) );

		// create a point-line constraint between the wheel and truss

		// create a spring between wheel and truss

	}

	ChBodySceneNode* getTrussBody(){
		return truss;
	}

	~TestMech()
	{
		ChSystem* msys = truss->GetBody()->GetSystem();
		// remove the bodies, joints
		msys->RemoveLink(spring);
		truss->remove();
	}

};


void create_some_falling_items(ChSystem& mphysicalSystem, ISceneManager* msceneManager, IVideoDriver* driver)
{
	 // Make some pebbles, just for fun, under the wheel
	video::ITexture* cubeMap = driver->getTexture("../data/concrete.jpg");
	video::ITexture* rockMap = driver->getTexture("../data/rock.jpg");

	ChBodySceneNode* mrigidBody; 

	ChCollisionModel::SetDefaultSuggestedEnvelope(0.003);
	ChCollisionModel::SetDefaultSuggestedMargin  (0.002);

	ChQuaternion<> rot;
	rot.Q_from_AngAxis(ChRandom()*CH_C_2PI, VECT_Y);

	double bed_x = 0.6;
	double bed_z = 1;

	int n_pebbles = 30;
	for (int bi = 0; bi < n_pebbles; bi++) 
	{    
		double sphrad = 0.02 + 0.02*ChRandom();
		double sphdens = 1;
		double sphmass = (4/3)*CH_C_PI*pow(sphrad,3)*sphdens;
		double sphinertia = pow(sphrad,2) * sphmass;
		ChQuaternion<> randrot(ChRandom(),ChRandom(),ChRandom(),ChRandom());
		randrot.Normalize();

		mrigidBody = (ChBodySceneNode*)addChBodySceneNode_easySphere(
											&mphysicalSystem, msceneManager,
											(4/3)*CH_C_PI*pow(sphrad,3)*sphdens,
											ChVector<>(
												-0.5*bed_x+ChRandom()*bed_x, 
												0.01+ 0.04*((double)bi/(double)n_pebbles), 
												-0.5*bed_z+ChRandom()*bed_z),
											sphrad
											 );

		mrigidBody->GetBody()->SetInertiaXX(ChVector<>(sphinertia,sphinertia,sphinertia));
		mrigidBody->GetBody()->SetFriction(0.4f);
		mrigidBody->GetBody()->SetRot(randrot);
		//mrigidBody->addShadowVolumeSceneNode();
		mrigidBody->setMaterialTexture(0,	rockMap);

	} 


	// Create the a plane using body of 'box' type:

	mrigidBody = (ChBodySceneNode*)addChBodySceneNode_easyBox(
											&mphysicalSystem, msceneManager,
											100.0,
											ChVector<>(0,-0.5,0),
											ChQuaternion<>(1,0,0,0), 
											ChVector<>(10,1,10) );
	mrigidBody->GetBody()->SetBodyFixed(true);
	mrigidBody->GetBody()->SetFriction(0.5); 
	mrigidBody->setMaterialTexture(0,	cubeMap);

} 
     
class MyEventReceiver : public IEventReceiver
{
public:

	MyEventReceiver(ChIrrAppInterface* mapp, ChBodySceneNode* wheelBody, ChBodySceneNode* trussBody,
		double pSize = 0.02, double pDev = 0.02)
	{
		// store pointer to physical system & other stuff so we can tweak them by user keyboard
		app = mapp;
		app->GetDevice()->setEventReceiver(this);
		// any rigid bodies that have their states modified by the GUI need to go here
		wheel = wheelBody;
		truss = trussBody;
		// initial checkbox values
		wheelLocked = true;
		trussLocked = true;
		wheelCollision = false;
		makeParticles = false;
		// initial values for the sliders
		particleSize0 = pSize;
		particleDev0 = pDev;

		// **** ***
		// create the GUI items here

		// ..add a GUI for wheel position lock
		checkbox_wheelLocked = app->GetIGUIEnvironment()->addCheckBox(
			wheelLocked, core::rect<s32>(620, 185, 650, 200), 0, 110);
		text_wheelLocked = app->GetIGUIEnvironment()->addStaticText(
			L"Wheel Position Locked", core::rect<s32>(650,185,900,200), false);

		checkbox_wheelLocked->setVisible(true);
		text_wheelLocked->setVisible(true);

		// ..add a GUI for truss position lock
		checkbox_trussLocked = app->GetIGUIEnvironment()->addCheckBox(
			trussLocked, core::rect<s32>(620, 215, 650, 230), 0, 111);
		text_trussLocked = app->GetIGUIEnvironment()->addStaticText(
			L"Truss Position Locked", core::rect<s32>(650,215,900,230), false);

		checkbox_trussLocked->setVisible(true);
		text_trussLocked->setVisible(true);

		// add a GUI for setting the wheel collision
		checkbox_wheelCollision = app->GetIGUIEnvironment()->addCheckBox(
			wheelCollision, core::rect<s32>(620, 245, 650, 260), 0, 112);
		text_wheelCollision = app->GetIGUIEnvironment()->addStaticText(
			L"wheel Collision On/Off ", core::rect<s32>(650,245,900,260), false);

		checkbox_wheelCollision->setVisible(true);
		text_wheelCollision->setVisible(true);

		// add a GUI for turning particle creation on/off
		checkbox_createParticles = app->GetIGUIEnvironment()->addCheckBox(
			makeParticles, core::rect<s32>(620, 275, 650, 300), 0, 113);
		text_createParticles = app->GetIGUIEnvironment()->addStaticText(
			L"create Particles? ", core::rect<s32>(650,275,900,300), false);

		checkbox_createParticles->setVisible(true);
		text_createParticles->setVisible(true);

		// create sliders to modify particle size/dev
		scrollbar_pSize = mapp->GetIGUIEnvironment()->addScrollBar( true, rect<s32>(520, 315, 650, 330), 0, 101);
		scrollbar_pSize->setMax(100); 
		scrollbar_pSize->setPos(50);
		text_pSize = mapp->GetIGUIEnvironment()->addStaticText(
			L"particle size:", rect<s32>(660,315,760,330), false);

		// particle Dev slider
		scrollbar_pDev = mapp->GetIGUIEnvironment()->addScrollBar(true, rect<s32>(520, 345, 650, 360), 0, 102);
		scrollbar_pDev->setMax(100); 
		scrollbar_pDev->setPos(50);
		text_pDev = mapp->GetIGUIEnvironment()->addStaticText(
			L"particle Dev:", rect<s32>(660,345,760,360), false);
	}

	bool OnEvent(const SEvent& event)
	{
	
		// check if user moved the sliders with mouse..
		if (event.EventType == EET_GUI_EVENT)
		{
			s32 id = event.GUIEvent.Caller->getID();
			gui::IGUIEnvironment* env = app->GetIGUIEnvironment();

			switch(event.GUIEvent.EventType)
			{
				case gui::EGET_CHECKBOX_CHANGED:
				{
					switch(id)
					{
					case 110:
						wheelLocked =  checkbox_wheelLocked->isChecked();
						GetLog() << checkbox_wheelLocked->isChecked() << "\n";
						// activate/deactivate the wheel linklocklock joint
						wheel->GetBody()->SetBodyFixed( wheelLocked);
						return true;
					case 111:
						trussLocked = checkbox_trussLocked->isChecked();
						GetLog() << checkbox_trussLocked->isChecked() << "\n";
						// activate/deactive the truss lock
						truss->GetBody()->SetBodyFixed( trussLocked);
						return true;
					case 112:
						wheelCollision = checkbox_wheelCollision->isChecked();
						GetLog() << checkbox_wheelCollision->isChecked() << "\n";
						// activate/deactivate the wheel collision detection
						wheel->GetBody()->SetCollide(wheelCollision);
						return true;
					default:
						return false;
					}
					// no break? this returns
				}
				case EGET_SCROLL_BAR_CHANGED:
					if( id == 101) // id of particle size slider
					{
						s32 currPos = ((IGUIScrollBar*)event.GUIEvent.Caller)->getPos();
						currParticleSize = particleSize0 + ((currPos - 50)/50.0)*particleSize0;
						char message[50]; sprintf(message,"pSize: %g",currParticleSize);
						text_pSize->setText(core::stringw(message).c_str());
					}
					if( id == 102) // id of particle Dev slider
					{
						s32 currPos = ((IGUIScrollBar*)event.GUIEvent.Caller)->getPos();
						currParticleDev = particleDev0 + ((currPos - 50)/50.0)*particleDev0;
						char message[50]; sprintf(message,"pDev: %g",currParticleDev);
						text_pDev->setText(core::stringw(message).c_str());
					}
			}

			
		} 

		return false;
	}

	// helper functions, these are called in the time step loop
	const double getCurrentPsize(){
		return currParticleSize;
	}
	const double getCurrentPdev(){
		return currParticleDev;
	}
	const bool createParticles(){
		return makeParticles;
	}

private:
	ChIrrAppInterface* app;
	// bodies/joints
	ChBodySceneNode* wheel;
	ChBodySceneNode* truss;
	// for check boxes
	bool wheelLocked;
	bool trussLocked;
	bool wheelCollision;
	bool makeParticles;

	// particle size, deviation
	double particleSize0;		// initial
	double currParticleSize;	// current value
	double particleDev0;		// initial
	double currParticleDev;		// current val

	// menu items
	gui::IGUIContextMenu* menu;
	gui::IGUICheckBox*	 checkbox_wheelLocked;
	gui::IGUIStaticText* text_wheelLocked;
	gui::IGUICheckBox*	checkbox_trussLocked;
	gui::IGUIStaticText* text_trussLocked;
	gui::IGUICheckBox*	checkbox_wheelCollision;
	gui::IGUIStaticText* text_wheelCollision;
	gui::IGUICheckBox*	checkbox_createParticles;
	gui::IGUIStaticText* text_createParticles;

	// scroll bars
	IGUIScrollBar* scrollbar_pSize;	// particle size
	IGUIStaticText* text_pSize;
	IGUIScrollBar* scrollbar_pDev;	// deviation of mean particle size
	IGUIStaticText* text_pDev;	

};

int main(int argc, char* argv[])
{

	// In CHRONO engine, The DLL_CreateGlobals() - DLL_DeleteGlobals(); pair is needed if
	// global functions are needed. 
	DLL_CreateGlobals();

	// Create a ChronoENGINE physical system
	ChSystem mphysicalSystem;
	GetLog() << get_working_path() << "\n";
	chdir("C:/ChronoEngine_svn/ChronoEngine/bin/Win64_VisualStudio/bin/Debug");
	GetLog() << get_working_path() << "\n";

	// Create the Irrlicht visualization (open the Irrlicht device, 
	// bind a simple user interface, etc. etc.)
	ChIrrAppInterface application(&mphysicalSystem, L"soil bin tester",core::dimension2d<u32>(800,600),false);


	// Easy shortcuts to add camera, lights, logo and sky in Irrlicht scene:
	ChIrrWizard::add_typical_Logo(application.GetDevice());
	ChIrrWizard::add_typical_Sky(application.GetDevice());
	ChIrrWizard::add_typical_Lights(application.GetDevice());
	ChIrrWizard::add_typical_Camera(application.GetDevice(), core::vector3df(3.5f,2.5f,-2.4f));


	// create the soil bin, with a few initial particles inside
	// create_some_falling_items(mphysicalSystem, application.GetSceneManager(), application.GetVideoDriver());
	// ChBodySceneNode* ground = create_soil_bin(mphysicalSystem, application.GetSceneManager(), application.GetVideoDriver(),
	//	1.0,2.0, 0.02, 0.02, 50);

	// Create the wheel
	ChVector<> wheelCMpos = ChVector<>(0,1,0);
	SoilbinWheel mwheel = SoilbinWheel(wheelCMpos, application);
	ChBodySceneNode* wheel_ptr = (ChBodySceneNode*)mwheel.getWheelBody();
	wheel_ptr->GetBody()->SetBodyFixed( true );	// change this with GUI
	// now, create the testing mechanism and attach the wheel to it
	TestMech mTestMechanism = TestMech( mwheel.getWheelBody(), application,
		1.0, 2.0, 0.02, 0.02, 50);
	// ChSharedPtr<ChLinkLockLock> wheelLock(new ChLinkLockLock);
	// wheelLock->Initialize(
	
	// Create the User - GUI
	MyEventReceiver receiver(&application, mwheel.getWheelBody(), mTestMechanism.getTrussBody() );
	 // add a custom event receiver to the default interface:
	application.SetUserEventReceiver(&receiver);

	// This will help choosing an integration step which matches the
	// real-time step of the simulation, if possible.

	int nstep = 0;

	application.SetStepManage(true);
	application.SetTimestep(0.01);
	application.SetTryRealtime(true);

	while(application.GetDevice()->run()) 
	{
		application.GetVideoDriver()->beginScene(true, true, SColor(255,140,161,192));

		application.DrawAll();

		application.DoStep();
		

		application.GetVideoDriver()->endScene();  
	}
	
 
 
	// Remember this at the end of the program, if you started
	// with DLL_CreateGlobals();
	DLL_DeleteGlobals();

	return 0;
}
  
