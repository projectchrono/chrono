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

class ParticleGenerator {
public:
	// data

	// functions
	ParticleGenerator(ChSystem* mphysicalSystem, ISceneManager* msceneManager, IVideoDriver* driver,
		double width, double len){
		// just set the data
		this->msys = mphysicalSystem;
		this->mscene = msceneManager;
		this->mdriver = driver;
		this->bedLength = len;
		this->bedWidth = width;
		this->simTime_lastPcreated = 0.0;
		
	}

	~ParticleGenerator() {
		// I have the pointers to each particle, loop over them and remove
		if(particleArray.size() > 0){
			for(int i = 0; i < particleArray.size(); i++){
				particleArray[i]->remove();
			}
		}
	}
	
	const bool create_some_falling_items(double pSize, double pDev, int nParticles)
	{
		double minTime_betweenCreate = 0.2;	// this much simulation time MUST elapse before being allowed to
											// create more particles
		if( (msys->GetChTime() - this->simTime_lastPcreated ) >= minTime_betweenCreate )
		{
			// reset the timer if we get in here
			this->simTime_lastPcreated = msys->GetChTime();

			 // generate some dirt in the bin
			video::ITexture* cubeMap = mdriver->getTexture("../data/concrete.jpg");
			video::ITexture* rockMap = mdriver->getTexture("../data/rock.jpg");

			// I should really check these
			ChCollisionModel::SetDefaultSuggestedEnvelope(0.003);
			ChCollisionModel::SetDefaultSuggestedMargin  (0.002);

			int totalNparticles = particleArray.size();	// how many particles are in the bin so far?
			for (int bi = 0; bi < nParticles; bi++) 
			{    
				ChBodySceneNode* currRigidBody; 
				double sphrad = pSize + pDev*ChRandom();
				double sphdens = 1;
				double sphmass = (4/3)*CH_C_PI*pow(sphrad,3)*sphdens;
				double sphinertia = pow(sphrad,2) * sphmass;
				ChQuaternion<> randrot(ChRandom(),ChRandom(),ChRandom(),ChRandom());
				randrot.Normalize();
				double stackHeight = (totalNparticles / 1000.0) * pSize;	// kind of guess the height of the particle stack
				// randomize spawning position, take stack height into consideration
				ChVector<> currPos = ChVector<>(-0.5*bedWidth + ChRandom()*bedWidth, 
												stackHeight + 2*pSize + 2*pSize*((double)bi/(double)nParticles), 
												-0.5*bedLength + ChRandom()*bedLength );
				currRigidBody = (ChBodySceneNode*)addChBodySceneNode_easySphere(msys, mscene,
													(4/3)*CH_C_PI*pow(sphrad,3)*sphdens,
													currPos,
													sphrad );
				currRigidBody->GetBody()->SetInertiaXX(ChVector<>(sphinertia,sphinertia,sphinertia));
				currRigidBody->GetBody()->SetFriction(0.4f);
				currRigidBody->GetBody()->SetRot(randrot);
				//mrigidBody->addShadowVolumeSceneNode();
				currRigidBody->setMaterialTexture(0,	rockMap);
				// add the particle body to the array
				particleArray.push_back(currRigidBody);
			} 

			GetLog() << " created " << nParticles << " this tstep \n" ;
			return true;	// created particles this step
		}
		else 
			return false;	// did not create particles this time step

	} 
private:
	ChSystem* msys;
	ISceneManager* mscene;
	IVideoDriver* mdriver;
	std::vector<ChBodySceneNode*> particleArray;	// hold onto pointers to the particles as they are created
	double bedLength;
	double bedWidth;
	double simTime_lastPcreated;	// keep track of the sim time when trying to creatye particles
								// so we don't create them at 1000 Hz or something silly like that
};



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

		// turn collision off to start, toggle with checkbox
		wheel->GetBody()->SetCollide(true);	 
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

	~SoilbinWheel() {
		// ChSystem* msys = wheel->GetBody()->GetSystem();
		// remove the bodies, joints
		wheel->remove();

	}
};

// create a test mechanism made up of 2 bodies
// a hub to connect to the wheel spindle and apply a torque through it
// a weight that moves vertically and horizontally w.r.t. the wheel spindle CM location
// spring/damper to apply a vertical load to the tire
// Purpose: only allow the tire to operate In-Plane, to simulate how a soil bin test mechaism works
class TestMech {
public:
	// data
	ChBodySceneNode* truss;
	ChBodySceneNode* weight;
	ChBodySceneNode* floor;
	ChBodySceneNode* wall1;
	ChBodySceneNode* wall2;
	ChBodySceneNode* wall3;
	ChBodySceneNode* wall4;
	ChSharedPtr<ChLinkSpring> spring;
	ChSharedPtr<ChLinkEngine> torqueDriver;

	// functions
	TestMech(ChBodySceneNode* wheelBody, ChIrrAppInterface& mapp,
		double binWidth = 1.0, double binLength = 2.0, 
		double weightMass = 400.0)
	{
		// create the soil bin, some particles, and the testing Mechanism
		video::ITexture* cubeMap = mapp.GetVideoDriver()->getTexture("../data/concrete.jpg");
		video::ITexture* rockMap = mapp.GetVideoDriver()->getTexture("../data/rock.jpg");

		ChCollisionModel::SetDefaultSuggestedEnvelope(0.003);
		ChCollisionModel::SetDefaultSuggestedMargin  (0.002);

		ChQuaternion<> rot;
		rot.Q_from_AngAxis(ChRandom()*CH_C_2PI, VECT_Y);

		// *******
		// Create a soil bin with planes. bin width = x-dir, bin length = z-dir
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
		// make a truss, connect it to the wheel via revolute joint
		// single rotational DOF will be driven with a user-input for torque
		// *****
		video::ITexture* bluMap = mapp.GetVideoDriver()->getTexture("../data/blu.png");
		ChVector<> trussCM = wheelBody->GetBody()->GetPos();
		ChBodySceneNode* truss = (ChBodySceneNode*)addChBodySceneNode_easyBox(mapp.GetSystem(), mapp.GetSceneManager(),
			1.0, trussCM, QUNIT, ChVector<>(0.2,0.2,0.4));
		truss->setMaterialTexture(0,bluMap);
		// truss shouldn't be used for Collisions
		truss->GetBody()->SetCollide(false);
		// create the revolute joint between the wheel and spindle
		ChSharedPtr<ChLinkLockRevolute> spindle(new ChLinkLockRevolute);
		spindle->Initialize(truss->GetBody(), wheelBody->GetBody(), 
			ChCoordsys<>(trussCM, chrono::Q_from_AngAxis(CH_C_PI/2, VECT_Y)) );
		mapp.GetSystem()->AddLink(spindle);
		// create a torque between the truss and wheel
		torqueDriver = ChSharedPtr<ChLinkEngine>(new ChLinkEngine);
		torqueDriver->Initialize(truss->GetBody(), wheelBody->GetBody(), 
			ChCoordsys<>(trussCM, chrono::Q_from_AngAxis(CH_C_PI/2, VECT_Y)) );
		torqueDriver->Set_eng_mode(ChLinkEngine::ENG_MODE_TORQUE);
		mapp.GetSystem()->AddLink(torqueDriver);

		// 

/*
		// ******
		// create a body that will be used as a vehicle weight
		ChVector<> weightCM = ChVector<>(trussCM); 
		weightCM.y += 0.5;
		ChBodySceneNode* weight = (ChBodySceneNode*)addChBodySceneNode_easyBox(mapp.GetSystem(), mapp.GetSceneManager(),
			weightMass, weightCM, QUNIT, ChVector<>(0.2, 0.4, 0.2));
		weight->setMaterialTexture(0,bluMap);
		weight->GetBody()->SetCollide(false);
	

		// create the translational joint between the truss and weight load
		ChSharedPtr<ChLinkLockPrismatic> translational(new ChLinkLockPrismatic);
		translational->Initialize(truss->GetBody(), weight->GetBody(),
			ChCoordsys<>(trussCM, chrono::Q_from_AngAxis(0,VECT_Y)) );
		mapp.GetSystem()->AddLink(translational);

		// create a spring between spindle truss and weight
		spring = ChSharedPtr<ChLinkSpring>(new ChLinkSpring);
		spring->Initialize( truss->GetBody(), weight->GetBody(), false, trussCM, weight->GetBody()->GetPos());
		spring->Set_SpringK(25000);
		spring->Set_SpringR(100);
		mapp.GetSystem()->AddLink(spring);

		// create a point-line constraint between the weight and the ground
		ChSharedPtr<ChLinkLockPointLine> weightLink(new ChLinkLockPointLine);
		weightLink->Initialize(weight->GetBody(), floor->GetBody(), 
			ChCoordsys<>( weightCM, chrono::Q_from_AngAxis(0,VECT_X)) );
		mapp.GetSystem()->AddLink(weightLink);
*/

	}

	// set the spring and damper constants
	void setSpringKD(double k, double d){
		this->spring->Set_SpringK(k);
		this->spring->Set_SpringR(d);
	}

	~TestMech()
	{

		// remove the bodies
		truss->remove();
		weight->remove();
		floor->remove();
		wall1->remove();
		wall2->remove();
		wall3->remove();
		wall4->remove();

		// remove joints
		ChSystem* msys = truss->GetBody()->GetSystem();
		msys->RemoveLink(spring);
		msys->Remove(torqueDriver);

	}

};

     
class MyEventReceiver : public IEventReceiver
{
public:

	// @param pSize particle radius
	// @param pDev multiplier added to ChRandom()
	// @param maxTorque max slider torque applied to wheel
	// @param maxParticles max number of particles to generate each spawning event
	MyEventReceiver(ChIrrAppInterface* mapp, SoilbinWheel* wheel, TestMech* tester, ParticleGenerator* particleGenerator,
		double pSize = 0.02, double pDev = 0.02, double maxTorque = 100, double maxParticles = 25)
	{
		// store pointer to physical system & other stuff so we can tweak them by user keyboard
		this->app = mapp;
		app->GetDevice()->setEventReceiver(this);
		// any rigid bodies that have their states modified by the GUI need to go here
		this->mwheel = wheel;
		this->mtester = tester;
		this->mgenerator = particleGenerator;
		// initial checkbox values
		this->wheelLocked = true;
//		trussLocked = true;
		this->wheelCollision = false;
		this->makeParticles = false;
		// initial values for the sliders
		this->particleSize0 = pSize;
		this->particleDev0 = pDev;
		this->maxTorque = maxTorque;
		this->nParticlesGenMax = maxParticles;

		// **** ***
		// create the GUI items here

		// ..add a GUI for wheel position lock ( id = 110 )
		checkbox_wheelLocked = app->GetIGUIEnvironment()->addCheckBox(
			wheelLocked, core::rect<s32>(620, 185, 650, 200), 0, 110);
		text_wheelLocked = app->GetIGUIEnvironment()->addStaticText(
			L"Wheel Position Locked", core::rect<s32>(650,185,900,200), false);
		checkbox_wheelLocked->setVisible(true);
		text_wheelLocked->setVisible(true);
		this->mwheel->wheel->GetBody()->SetBodyFixed( wheelLocked);	// set IC of checkbox

		// add a GUI for turning particle creation on/off ( id = 111 )
		checkbox_createParticles = app->GetIGUIEnvironment()->addCheckBox(
			makeParticles, core::rect<s32>(620, 215, 650, 230), 0, 111);
		text_createParticles = app->GetIGUIEnvironment()->addStaticText(
			L"create Particles? ", core::rect<s32>(650,215,900,230), false);
		checkbox_createParticles->setVisible(true);
		text_createParticles->setVisible(true);

		// add a GUI for setting the wheel collision ( id = 112 )
		checkbox_wheelCollision = app->GetIGUIEnvironment()->addCheckBox(
			wheelCollision, core::rect<s32>(620, 245, 650, 260), 0, 112);
		text_wheelCollision = app->GetIGUIEnvironment()->addStaticText(
			L"wheel Collision On/Off ", core::rect<s32>(650,245,900,260), false);
		checkbox_wheelCollision->setVisible(true);
		text_wheelCollision->setVisible(true);
		mwheel->wheel->GetBody()->SetCollide(wheelCollision);	// set IC of checkbox

		// ..add a GUI for turning torque on/off ( id = 113 )
		checkbox_applyTorque = app->GetIGUIEnvironment()->addCheckBox(
			applyTorque, core::rect<s32>(620, 275, 650, 300), 0, 113);
		text_applyTorque = app->GetIGUIEnvironment()->addStaticText(
			L"Apply wheel Torque?", core::rect<s32>(650,275,900,300), false);
		checkbox_applyTorque->setVisible(true);
		text_applyTorque->setVisible(true);

		// create sliders to modify particle size/dev ( id = 101)
		scrollbar_pSize = mapp->GetIGUIEnvironment()->addScrollBar( true, rect<s32>(520, 315, 650, 330), 0, 101);
		scrollbar_pSize->setMax(100); 
		scrollbar_pSize->setPos(50);
		char message[50]; sprintf(message,"p Size [m]: %g",particleSize0);
		text_pSize = mapp->GetIGUIEnvironment()->addStaticText(
			core::stringw(message).c_str(), rect<s32>(660,315,760,330), false);
		this->currParticleSize = particleSize0;	// set the IC

		// particle Dev slider	(id = 102)
		scrollbar_pDev = mapp->GetIGUIEnvironment()->addScrollBar(true, rect<s32>(520, 345, 650, 360), 0, 102);
		scrollbar_pDev->setMax(100); 
		scrollbar_pDev->setPos(50);
		char message1[50]; sprintf(message1,"p Dev [m]: %g",particleDev0);
		text_pDev = mapp->GetIGUIEnvironment()->addStaticText(
			core::stringw(message1).c_str(), rect<s32>(660,345,760,360), false);
		this->currParticleDev = particleDev0;	// set the IC for the slider

		// torque slider	(id = 103)
		scrollbar_torque = mapp->GetIGUIEnvironment()->addScrollBar(true, rect<s32>(520, 375, 650, 390), 0, 103);
		scrollbar_torque->setMax(100);
		scrollbar_torque->setPos(50);
		text_torque = mapp->GetIGUIEnvironment()->addStaticText(
			L"Torque [N/m]: 0 ", rect<s32>(660, 375, 760,390), false);
		this->currTorque = 0;	// set the IC of this slider

		// nParticlesGen slider ( id = 104)
		scrollbar_nParticlesGen = mapp->GetIGUIEnvironment()->addScrollBar(true, rect<s32>(520, 405, 650, 420), 0, 104);
		scrollbar_nParticlesGen->setMax(100);
		scrollbar_nParticlesGen->setPos(50);
		this->currNparticlesGen = nParticlesGenMax/2;	// IC of this slider
		char message2[50]; sprintf(message2,"# Gen. particles: %d",currNparticlesGen);
		text_nParticlesGen = mapp->GetIGUIEnvironment()->addStaticText(
			core::stringw(message2).c_str(), rect<s32>(660, 405, 760,420), false);

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
						this->mwheel->wheel->GetBody()->SetBodyFixed( wheelLocked);
						return true;
					case 111:
						makeParticles = checkbox_createParticles->isChecked();
						GetLog() << checkbox_createParticles->isChecked() << "\n";
						return true;
					case 112:
						wheelCollision = checkbox_wheelCollision->isChecked();
						GetLog() << checkbox_wheelCollision->isChecked() << "\n";
						// activate/deactivate the wheel collision detection
						mwheel->wheel->GetBody()->SetCollide(wheelCollision);
						return true;
					case 113:
						applyTorque = checkbox_applyTorque->isChecked();
						GetLog() << checkbox_applyTorque->isChecked() << "\n";
						// apply a torque to the wheel?
						return true;
					default:
						return false;
					}
					// end switch
				}
				case EGET_SCROLL_BAR_CHANGED:
				{
					if( id == 101) // id of particle size slider
					{
						s32 currPos = ((IGUIScrollBar*)event.GUIEvent.Caller)->getPos();
						this->currParticleSize = particleSize0 + ((currPos - 50)/50.0)*particleSize0;
						char message[50]; sprintf(message,"p Size [m]: %g",currParticleSize);
						text_pSize->setText(core::stringw(message).c_str());
					}
					if( id == 102) // id of particle Dev slider
					{
						s32 currPos = ((IGUIScrollBar*)event.GUIEvent.Caller)->getPos();
						this->currParticleDev = particleDev0 + ((currPos - 50)/50.0)*particleDev0;
						char message[50]; sprintf(message,"p Dev [m]: %g",currParticleDev);
						text_pDev->setText(core::stringw(message).c_str());
					}
					if( id == 103) // torque slider
					{
						s32 currPos = ((IGUIScrollBar*)event.GUIEvent.Caller)->getPos();
						this->currTorque = ((currPos - 50.0)/50.0)*maxTorque;
						char message[50]; sprintf(message,"Torque [N/m]: %g",currTorque);
						text_torque->setText(core::stringw(message).c_str());
					}
					if( id == 104) // # particles to generate
					{
						s32 currPos = ((IGUIScrollBar*)event.GUIEvent.Caller)->getPos();
						this->currNparticlesGen =  nParticlesGenMax + ((currPos - 50)/50.0)*nParticlesGenMax;
						char message[50]; sprintf(message,"# Gen. particles: %d",currNparticlesGen);
						text_nParticlesGen->setText(core::stringw(message).c_str());
					}
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

	// try to generate some particles. Returne T/F if anything was created
	const bool genParticles() {
		return mgenerator->create_some_falling_items(currParticleSize, currParticleDev, currNparticlesGen);
	}

private:
	ChIrrAppInterface* app;
	// bodies/joints
	SoilbinWheel* mwheel;
	TestMech* mtester;
	ParticleGenerator* mgenerator;
	// for check boxes
	bool wheelLocked;	// id = 110
	bool makeParticles; // 111
	bool wheelCollision;// 112
	bool applyTorque;	// 113

	// particle size, deviation
	double particleSize0;		// initial
	double currParticleSize;	// current value
	double particleDev0;		// initial
	double currParticleDev;		// current val
	double maxTorque;		// max torque applied to wheel
	double currTorque;
	int nParticlesGenMax;	// max number of particles to generate
	int currNparticlesGen;	// # of particles to generate this step

	// menu items
	gui::IGUIContextMenu* menu;
	gui::IGUICheckBox*	 checkbox_wheelLocked; // ic = 110
	gui::IGUIStaticText* text_wheelLocked;
	gui::IGUICheckBox*	checkbox_createParticles; // id = 111
	gui::IGUIStaticText* text_createParticles;
	gui::IGUICheckBox*	checkbox_wheelCollision;	// id = 112
	gui::IGUIStaticText* text_wheelCollision;
	gui::IGUICheckBox*	checkbox_applyTorque;	// id = 113
	gui::IGUIStaticText* text_applyTorque;


	// scroll bars
	IGUIScrollBar* scrollbar_pSize;	// particle size, id = 101
	IGUIStaticText* text_pSize;
	IGUIScrollBar* scrollbar_pDev;	// deviation of mean particle size, id = 102
	IGUIStaticText* text_pDev;	
	IGUIScrollBar* scrollbar_torque; // torque applied to wheel, id = 103
	IGUIStaticText* text_torque;
	IGUIScrollBar* scrollbar_nParticlesGen;	// particles to spawn, id = 104
	IGUIStaticText* text_nParticlesGen;
};

int main(int argc, char* argv[])
{

	// In CHRONO engine, The DLL_CreateGlobals() - DLL_DeleteGlobals(); pair is needed if
	// global functions are needed. 
	DLL_CreateGlobals();

	// Create a ChronoENGINE physical system
	ChSystem mphysicalSystem;

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
	// mwheel.wheel->GetBody()->SetBodyFixed( true );	// change this with GUI
	// now, create the testing mechanism and attach the wheel to it
	double binWidth = 1.0;
	double binLen = 2.0;
	TestMech mTestMechanism = TestMech( mwheel.wheel, application,
		binWidth, binLen);
	
	// make a particle generator, that the sceneManager can use to easily dump a bunch of dirt in the bin
	ParticleGenerator mParticleGen = ParticleGenerator( application.GetSystem(), application.GetSceneManager(), application.GetVideoDriver(),
		binWidth, binLen);

	
	// Create the User - GUI
	MyEventReceiver receiver(&application, &mwheel, &mTestMechanism, &mParticleGen );
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
		if( !application.GetPaused())
		{
			// add bodies to the system?
			if( receiver.createParticles() )
			{
				receiver.genParticles();
			}
		}
		application.GetVideoDriver()->endScene();  
	}
	
 
	// Remember this at the end of the program, if you started
	// with DLL_CreateGlobals();
	DLL_DeleteGlobals();

	return 0;
}
  
