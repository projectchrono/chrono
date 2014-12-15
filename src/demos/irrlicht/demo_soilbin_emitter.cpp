///////////////////////////////////////////////////
//
//  Demo code about
//    - Similar to demo_soilbin
//    - Uses ChParticleEmitter to control statistical distribution of particles
//    - Uses ChAssets for geometry
//    - classes are all inheriting ChShared
//
//  Author: Justin Madsen, 2014
//
///////////////////////////////////////////////////
 
   
#include "physics/ChSystem.h"
#include "particlefactory/ChParticleEmitter.h"
#include "assets/ChTexture.h"
#include "assets/ChColorAsset.h"
#include "unit_IRRLICHT/ChIrrApp.h"


using namespace chrono;
using namespace chrono::particlefactory;
using namespace chrono::geometry;

// Use the main namespaces of Irrlicht
using namespace irr;    
using namespace core;
using namespace scene; 
using namespace video;
using namespace io; 
using namespace gui; 

struct stats {
  double pRadMean;
  double pRadStdDev;
  double pRad_s1;
  double pRad_s2;
  double pMassMean;
  double pMassStdDev;
  double pMass_s2;
};

/// Creates particles for the soil bin, using the ChParticleEmitter class.
/// Manages saving/loading of particle systems for checkpointing.
/// Exposes a few particle variables used in subsequently created particles for the GUI to tweak.
/// Output data file for particle statistics (?? Is this included in ChParticleEmitter ?? )
class ParticleGenerator: public ChShared {
public:

	ParticleGenerator(ChSystem* mSystem, ChIrrApp* mApp,
		double width, double len, double sphDensity = 100.0, double boxDensity = 100.0, double mu = 0.33)
  : msys( mSystem), mapp(mApp), bedLength(len), bedWidth(width), sphDens(sphDensity), boxDens(boxDensity), mu(mu)

  {
    // zero out some initial values
    simTime_lastPcreated = 0.0;
    totalParticles = 0;
    totalParticleMass = 0.0;

    // keep track of some statistics
    stats zeros = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    m_stats = zeros;

  }

  ~ParticleGenerator() { }

	// turn visibility on/off
	void toggleVisibility(bool togglevis)
	{
		this->particleParent->setVisible(togglevis);

	}


  // --------------------------------------------------------------------------
  //  ACCESSORS
	double get_mu() {	return mu;	}

  double get_sphDensity(){ return sphDens; }

  // return the total # of particles
	int get_totalParticles(){	return totalParticles;	}

	// return the total particle mass
	double get_totalParticleMass(){return totalParticleMass;}

  // return the statistics to use in particle creation
	const stats get_stats() { return m_stats; }


  // --------------------------------------------------------------------------
  // MODIFIERS
	void set_mu(double newMu)
	{
		if(newMu < 0.0) 
		{
			GetLog() << "can't set mu less than 0  \n" ;
			return;
		}
		if(newMu > 1.0)
			GetLog() << "probably shouldn't have mu > 1.0   \n";
		
		// set mu anyway if >1.0
		this->mu = newMu;
	}


	void set_sphDensity(double newDens)
	{
		if(newDens < 0.0) 
			GetLog() << "can't set density less than 0  \n" ;
		else
			this->sphDens = newDens;
	}

	void set_boxDensity(double newDens)
	{
		if(newDens < 0.0) 
			GetLog() << "can't set density less than 0  \n" ;
		else
			this->boxDens = newDens;

	}
  
	// create some spheres with size = pSize + ChRank()*pDev
	// also, you can create boxes too, with the sides being sized in the same sort of manner as the spheres
	const bool create_some_falling_items(double pSize, double pDev, int nParticles,	int nBoxes = 0)
	{
		double minTime_betweenCreate = 0.05;	// this much simulation time MUST elapse before being allowed to
											// create more particles
		if( (msys->GetChTime() - this->simTime_lastPcreated ) >= minTime_betweenCreate )
		{
			// reset the timer if we get in here
			this->simTime_lastPcreated = msys->GetChTime();

			// increment the counters for total # of particles
			this->totalParticles += nParticles;
			this->totalParticles += nBoxes;
			// kind of guess the height of the particle stack
			double stackHeight = (this->totalParticles / 2000.0) * pSize - 0.2;
			double sphdens = this->sphDens;	// kg/m3
			// create the spheres
			for (int bi = 0; bi < nParticles; bi++) 
			{    
				double sphrad = pSize + pDev*ChRandom();
				double sphmass = (4/3)*CH_C_PI*pow(sphrad,3)*sphdens;
				double sphinertia = pow(sphrad,2) * sphmass;
				ChQuaternion<> randrot(ChRandom(),ChRandom(),ChRandom(),ChRandom());
				randrot.Normalize();
				// randomize spawning position, take stack height into consideration
				ChVector<> currPos = ChVector<>(-0.5*bedWidth + ChRandom()*bedWidth, 
												stackHeight + 2*pSize*((double)bi/(20.0*ChRandom() + 50.0)), 
												-0.5*bedLength + ChRandom()*bedLength );



        // use ChParticleEmitter here

        // set Texture here
				// currRigidBody->setMaterialTexture(0,	rockMap);
				// every time we add a body, increment the counter and mass
				totalParticleMass += sphmass;
				m_stats.pMass_s2 += sphmass*sphmass;
				m_stats.pRad_s1 += sphrad;
				m_stats.pRad_s2 += sphrad*sphrad;

			} 

			// create the boxes
			double boxdens = this->boxDens;
			for(int bi = 0; bi < nBoxes; bi++)
			{
				double xscale = 1.5*ChRandom(); // scale 2 of the 3 dimensions
				double yscale = 2.0;
				double zscale = 1.5*ChRandom();
				ChVector<> boxSize = ChVector<>(pSize*xscale,pSize*yscale,pSize*zscale);
				// mass = rho * vol
				double boxmass = boxSize.x * boxSize.y * boxSize.z * sphdens;
				// position found the same way as the spheres
				ChVector<> currPos = ChVector<>(-0.5*bedWidth + ChRandom()*bedWidth, 
												stackHeight + 2*pSize*((double)bi/(20.0*ChRandom() + 20.0)), 
												-0.5*bedLength + ChRandom()*bedLength );
	
				// randomize the initial orientation
				ChQuaternion<> randrot(ChRandom(),ChRandom(),ChRandom(),ChRandom());
				randrot.Normalize();
				// create the body object
				
				this->totalParticles++;
				this->totalParticleMass += boxmass;
				m_stats.pMass_s2 += boxmass*boxmass;

			}


			// update the statistics
			m_stats.pRadMean = m_stats.pRad_s1 / (double)totalParticles;
			m_stats.pRadStdDev = sqrt( (double)totalParticles*m_stats.pRad_s2 - m_stats.pRad_s1*m_stats.pRad_s1) / (double)totalParticles;
			m_stats.pMassMean = this->totalParticleMass / (double)totalParticles;
			m_stats.pMassStdDev = sqrt( (double)totalParticles*m_stats.pMass_s2 - totalParticleMass*totalParticleMass) / (double)totalParticles;
			return true;	// created particles this step
		}
		else 
			return false;	// did not create particles this time step

	} 
  
private:
  // private data members
	ChSystem* msys;
  ChSharedPtr<ChParticleEmitter> m_emitter;
	ChIrrApp* mapp;
  stats m_stats;  // some simple statistics to keep track of
	ISceneNode* particleParent;	// hold onto pointers to the particles as they are created
	int totalParticles;
	double totalParticleMass;	
	double bedLength;
	double bedWidth;
	double simTime_lastPcreated;	// keep track of the sim time when trying to creatye particles
								// so we don't create them at 1000 Hz or something unrealistic

	// density of shape primitives
	double sphDens;
	double boxDens;
	double mu;	// friction coef of particles
  double coh; // cohesion of particles

};



class SoilbinWheel : public ChShared {
public:
	//data
	ChSharedPtr<ChBody> wheel;

	// Use convex decomposition for C-D with the Trelleborg tire
  // a repeated wheel surface and treads
	SoilbinWheel(ChIrrAppInterface& mapplication, const ChVector<>& mposition, 
		double mass, const ChVector<>& inertia, double mu, bool visualize=true)
	{
    wheel = ChSharedPtr<ChBody>(new ChBody);
		wheel->SetInertiaXX(inertia);
		wheel->SetFriction(mu);

		// turn collision off to start, toggle with checkbox
		wheel->SetCollide(true);	 
		// Clear model. The colliding shape description MUST be between  ClearModel() .. BuildModel() pair.
		wheel->GetCollisionModel()->ClearModel();
		// Describe the (invisible) colliding shape by adding the 'carcass' decomposed shape and the 
		// 'knobs'. Since these decompositions are only for 1/15th of the wheel, use for() to pattern them.
		for (double mangle = 0; mangle < 360.; mangle+= (360./15.))
		{
			ChQuaternion<>myrot;
      ChStreamInAsciiFile myknobs(GetChronoDataFile("tractor_wheel_knobs.chulls").c_str());
      ChStreamInAsciiFile myslice(GetChronoDataFile("tractor_wheel_slice.chulls").c_str());
			myrot.Q_from_AngAxis(mangle*(CH_C_PI/180.),VECT_X);
			ChMatrix33<> mm(myrot);
			wheel->GetCollisionModel()->AddConvexHullsFromFile(myknobs, ChVector<>(0,0,0), mm);
			wheel->GetCollisionModel()->AddConvexHullsFromFile(myslice, ChVector<>(0,0,0), mm);
		}
	
			// Complete the description.
		wheel->GetCollisionModel()->BuildModel();
	}

	// use a hollow cylinder as the wheel body. Note, the last input arg doesn't do anything
	// Just needed a different constructor
	SoilbinWheel(ChIrrAppInterface& mapp, ChVector<> mposition, 
		double mass, double cyl_height, double cyl_d_outer, double cyl_d_inner, double mu, bool visualize=true)
	{
		
		double d2 = cyl_d_outer;	// outer radius
		double d1 = cyl_d_inner;	// inner radius
		double h = cyl_height;	// height

		// what axis does Chrono orient a cylinder about? y-axis, I think
		// ChVector<> inertia = ChVector<>( mass*(3*(r1*r1 + r2*r2) + h*h)/12.0, mass*(r1*r1 + r2*r2)/2.0, mass*(3*(r1*r1 + r2*r2) + h*h)/12.0);
		// rotate the wheel for proper initia orientation
		ChQuaternion<> mrot = chrono::Q_from_AngAxis(CH_C_PI/2.0,VECT_Z);

    double density = mass / ((d2*d2 - d1*d1)*h/8.0);
		// create the cylinder body
    wheel = ChSharedPtr<ChBodyEasyCylinder>(new ChBodyEasyCylinder(d2/2.0, h/2.0, density, true, visualize));
		
		// 
		wheel->SetFriction(mu);
    if(visualize)
    {
      ChSharedPtr<ChTexture> rubber_tex(new ChTexture);
      rubber_tex->SetTextureFilename(GetChronoDataFile("rubber.jpg"));
      wheel->AddAsset(rubber_tex);
    }

	}


};

// create a test mechanism made up of 2 bodies
// a hub to connect to the wheel spindle and apply a torque through it
// a weight that moves vertically and horizontally w.r.t. the wheel spindle CM location
// spring/damper to apply a vertical load to the tire
// Purpose: only allow the tire to operate In-Plane, to simulate how a soil bin test mechaism works
class LinearTestRig : public ChShared {
public:
	// data
	ChSharedPtr<ChBody> spindleBody;		// spindle truss
	ChSharedPtr<ChBody> suspweight;	// suspended weight
	ChSharedPtr<ChBody> floor;
	ChSharedPtr<ChBody> wall1;
	ChSharedPtr<ChBody> wall2;
	ChSharedPtr<ChBody> wall3;
	ChSharedPtr<ChBody> wall4;
  // links, force elements
	ChSharedPtr<ChLinkSpring> spring;
	ChSharedPtr<ChLinkEngine> torqueDriver;
	ChSharedPtr<ChLinkLockRevolute> spindle;

	// GUI-tweaked data
	bool isTorqueApplied;
	double currTorque;

	// constructor, attaches to the wheel body via a revolute joint
	LinearTestRig(ChSharedPtr<ChBody>& wheel, ChIrrAppInterface& mapp, ChSystem& m_sys,
		double binWidth = 1.0, double binLength = 2.0, 
		double weightMass = 100.0,
		double springK = 25000, double springD=100,
    double mu_floor = 0.5)
	{
		ChQuaternion<> rot;
		rot.Q_from_AngAxis(ChRandom()*CH_C_2PI, VECT_Y);

		// *******
		// Create a soil bin with planes. bin width = x-dir, bin length = z-dir
		// Note: soil bin depth will always be ~ 1m
		// *******
		double binHeight = 1.0;
		double wallWidth = std::min<double>(binWidth, binLength) / 10.0;	// wall width = 1/10 of min of bin dims

    double density = 1000;
		// create the floor
    floor = ChSharedPtr<ChBodyEasyBox>(new ChBodyEasyBox(binWidth+wallWidth/2.0, wallWidth, binLength+wallWidth/2.0,
      density, true, true));
  
    floor->SetBodyFixed(true);
		floor->SetFriction(mu_floor);	// NOTE: May need to change this if the walls have effects on the soil response
    m_sys.Add(floor);

    // TODO:
    // floor->setMaterialTexture(0,	cubeMap);
		
		// wall 1
		wall1 = ChSharedPtr<ChBodyEasyBox>(new ChBodyEasyBox(wallWidth, binHeight, binLength,
      density, true, true));
      
		wall1->SetBodyFixed(true);

    // TODO:
		// wall1->setMaterialTexture(0,	cubeMap);
    m_sys.Add(wall1);

		// wall 2
		wall2 = ChSharedPtr<ChBodyEasyBox>(new ChBodyEasyBox(wallWidth,binHeight,binLength,
      density, true, true));
		wall2->SetBodyFixed(true);

    // TODO: 
		// wall2->setMaterialTexture(0,	rockMap);
    m_sys.Add(wall2);

	  // wall 3
		wall3 = ChSharedPtr<ChBodyEasyBox>(new ChBodyEasyBox(binWidth + wallWidth/2.0, binHeight, wallWidth,
      density, true, true));
		wall3->SetBodyFixed(true);
		
    // TODO:
		// wall3->setMaterialTexture(0,	wall3tex);
    m_sys.Add(wall3);

		// wall 4
		wall4 =ChSharedPtr<ChBodyEasyBox>(new ChBodyEasyBox(binWidth + wallWidth/2.0, binHeight, wallWidth,
      density, true, true));
		wall4->SetBodyFixed(true);

    // TODO;
		// wall4->setMaterialTexture(0,	wall4tex);
    m_sys.Add(wall4);

		// make a spindle body, connect it to the wheel via revolute joint
		// single rotational DOF will be driven with a user-input for torque
		ChVector<> spindleCM = wheel->GetPos();
		spindleBody = ChSharedPtr<ChBodyEasyBox>(new ChBodyEasyBox(0.2,0.2,0.4,
      density, false, true));
    spindleBody->SetPos(spindleCM);

    ChSharedPtr<ChTexture> blu_tex(new ChTexture);
    blu_tex->SetTextureFilename(GetChronoDataFile("blu.png"));
    spindleBody->AddAsset(blu_tex);
    m_sys.Add(spindleBody);

		// create a torque driver between the spindle and wheel, which also serves as a revolute joint
		torqueDriver = ChSharedPtr<ChLinkEngine>(new ChLinkEngine);
		torqueDriver->Initialize(spindleBody, wheel, 
			ChCoordsys<>(spindleCM, chrono::Q_from_AngAxis(CH_C_PI/2, VECT_Y)) );
		torqueDriver->Set_eng_mode(ChLinkEngine::ENG_MODE_TORQUE);
		m_sys.AddLink(torqueDriver);


		// create a body that will be used as a vehicle weight
		ChVector<> weightCM = ChVector<>(spindleCM); 
		weightCM.y += 1.0;	// note: this will determine the spring free length
		suspweight = ChSharedPtr<ChBodyEasyBox>(new ChBodyEasyBox(0.2, 0.4, 0.2,
      density, false, true));
    suspweight->AddAsset(blu_tex);

		// create the translational joint between the truss and weight load
		ChSharedPtr<ChLinkLockPrismatic> translational(new ChLinkLockPrismatic);
		translational->Initialize(spindleBody, suspweight,
			ChCoordsys<>(spindleCM, chrono::Q_from_AngAxis(CH_C_PI/2,VECT_X)) );	// set trans. axis as vertical
		m_sys.AddLink(translational);

		// create a spring between spindle truss and weight
		spring = ChSharedPtr<ChLinkSpring>(new ChLinkSpring);
		spring->Initialize( spindleBody, suspweight, false, spindleCM, suspweight->GetPos());
		spring->Set_SpringK(springK);
		spring->Set_SpringR(springD);
		mapp.GetSystem()->AddLink(spring);

		// create a prismatic constraint between the weight and the ground
		ChSharedPtr<ChLinkLockOldham> weightLink(new ChLinkLockOldham);
		weightLink->Initialize(suspweight, floor, 
			ChCoordsys<>( weightCM, chrono::Q_from_AngAxis(CH_C_PI/2.0, VECT_Y)) );	// trans. axis should be z-dir by default
		mapp.GetSystem()->AddLink(weightLink);

	}

	// set the spring and damper constants
	void set_SpringKD(double k, double d)
  {
		this->spring->Set_SpringK(k);
		this->spring->Set_SpringR(d);
	}

	// for now, just use the slider value as directly as the torque
	void applyTorque()
  {
		// note: negative sign is to get Trelleborg tire to spin in the correct direction
      if (ChSharedPtr<ChFunction_Const> mfun = torqueDriver->Get_tor_funct().DynamicCastTo<ChFunction_Const>())
			mfun->Set_yconst( -this->currTorque);
	}

	~LinearTestRig() {}

};

     
class MyEventReceiver : public IEventReceiver
{
public:
	// keep the tabs public
	gui::IGUITabControl* gad_tabbed;
	gui::IGUITab*		gad_tab_controls;
	gui::IGUITab*		gad_tab_wheel;
	gui::IGUITab*		gad_tab_soil;

	// @param pSize particle radius
	// @param pDev multiplier added to ChRandom()
	// @param maxTorque max slider torque applied to wheel
	// @param maxParticles max number of particles to generate each spawning event
  MyEventReceiver(ChIrrApp* app, ChSharedPtr<SoilbinWheel>& wheel, ChSharedPtr<LinearTestRig>& testRig,
    ChSharedPtr<ParticleGenerator>& particleGen,
		double pSize = 0.02, double pDev = 0.02,
    double maxTorque = 100.0, int maxParticles = 50)
    : mapp(app), mwheel(wheel), mtester(testRig), mgenerator(particleGen)
	{
		// for getting output from the TM_Module module
		// initial checkbox values
		this->wheelLocked = true;
		this->makeParticles = false;
//		this->applyTorque = false;
		this->wheelCollision = false;
		this->pVisible = true;
		this->wheelVisible = true;

		// initial values for the sliders
		this->particleSize0 = pSize;
		this->particleDev0 = pDev;
		this->maxTorque = maxTorque;
		this->nParticlesGenMax = maxParticles;

		// **** ***
		// create the GUI items here
		irr::s32 x0 = 740;	irr::s32 y0 = 20;	// box0 top left corner
		// create the tabs for the rig output: NOTE: GUI widget locations are all relative to the TabControl!
		gad_tabbed = mapp->GetIGUIEnvironment()->addTabControl(core::rect<s32>(x0,y0,x0+255,y0+440), 0, true, true);
		gad_tab_controls = gad_tabbed->addTab(L"Controls");	// static text will be printed w/ each checkbox or slider
		gad_text_wheelControls = mapp->GetIGUIEnvironment()->addStaticText(L"Wheel Control",
			core::rect<s32>(10,10,245,150), true, true, gad_tab_controls);
		irr::s32 y1 = 165;	// box1 top left corner
		gad_text_pControls = mapp->GetIGUIEnvironment()->addStaticText(L"Particle Control",
			core::rect<s32>(10,y1,245,y1+230), true, true, gad_tab_controls);
		gad_tab_wheel = gad_tabbed->addTab(L"Wheel State");
		gad_text_wheelState = mapp->GetIGUIEnvironment()->addStaticText(L"WS",
			core::rect<s32>(10,10,290,250), true, true, gad_tab_wheel);
		gad_tab_soil = gad_tabbed->addTab(L"Soil State");
		gad_text_soilState = mapp->GetIGUIEnvironment()->addStaticText(L"SS", 
			core::rect<s32>(10,10,290,250), true, true, gad_tab_soil);

		// **** GUI CONTROLS ***
		// -------- Wheel controls
		// ..add a GUI for wheel position lock ( id = 2110 )
		checkbox_wheelLocked = app->GetIGUIEnvironment()->addCheckBox(wheelLocked,
			core::rect<s32>(20, 30, 35, 45), gad_tab_controls, 2110);
		text_wheelLocked = app->GetIGUIEnvironment()->addStaticText(L"Wheel Locked",
			core::rect<s32>(45,30, 125,45), false,false,gad_tab_controls);
		checkbox_wheelLocked->setVisible(true);
		text_wheelLocked->setVisible(true);
		this->mwheel->wheel->SetBodyFixed(wheelLocked);	// set IC of checkbox
		this->mtester->spindleBody->SetBodyFixed(wheelLocked);
		this->mtester->suspweight->SetBodyFixed(wheelLocked);

		// turn wheel visibility on/off, ie = 2115
		checkbox_wheelVisible = app->GetIGUIEnvironment()->addCheckBox(wheelVisible,
			core::rect<s32>(180, 30, 195, 45), gad_tab_controls,2115);
		text_wheelVisible = app->GetIGUIEnvironment()->addStaticText(L"visible?",
			core::rect<s32>(205, 30, 290, 45),false,false,gad_tab_controls);

		// add a GUI for setting the wheel collision ( id = 2112 )
		checkbox_wheelCollision = app->GetIGUIEnvironment()->addCheckBox(wheelCollision,
			core::rect<s32>(20, 60, 35, 75), gad_tab_controls, 2112);
		text_wheelCollision = app->GetIGUIEnvironment()->addStaticText(L"Wheel collide? ",
			core::rect<s32>(45,60,125,75), false,false,gad_tab_controls);
		checkbox_wheelCollision->setVisible(true);
		text_wheelCollision->setVisible(true);
		this->mwheel->wheel->SetCollide(wheelCollision);	// set IC of checkbox

		// torque slider	(id = 1103)
		scrollbar_torque = mapp->GetIGUIEnvironment()->addScrollBar(true, 
			rect<s32>(20, 115, 150, 130), gad_tab_controls, 1103);
		scrollbar_torque->setMax(100);
		scrollbar_torque->setPos(50);
		text_torque = mapp->GetIGUIEnvironment()->addStaticText(L"Torque[N/m]: 0 ",
			rect<s32>(160, 115, 300,130), false,false,gad_tab_controls);
		this->mtester->currTorque = 0;	// set the IC of this slider

		// -------- Particle Controls
		// add a GUI for turning particle creation on/off ( id = 2111 )
		checkbox_createParticles = app->GetIGUIEnvironment()->addCheckBox(makeParticles, 
			core::rect<s32>(20, y1+20, 35, y1+35), gad_tab_controls, 2111);
		text_createParticles = app->GetIGUIEnvironment()->addStaticText(L"create Particles? ", 
			core::rect<s32>(45,y1+20,165,y1+35), false,false,gad_tab_controls);
		checkbox_createParticles->setVisible(true);
		text_createParticles->setVisible(true);

		// add a checkbox to make particle visibility turn on/off, id = 2114
		checkbox_particlesVisible = app->GetIGUIEnvironment()->addCheckBox(pVisible, 
			core::rect<s32>(180,y1+20,195,y1+35),gad_tab_controls, 2114);
		text_particlesVisible = app->GetIGUIEnvironment()->addStaticText( L"visible?", 
			core::rect<s32>(205,y1+20,290,y1+35),false,false,gad_tab_controls);

		// create sliders to modify particle size/dev ( id = 1101)
		scrollbar_pSize = mapp->GetIGUIEnvironment()->addScrollBar( true, 
			rect<s32>(20, y1+50, 150, y1+65), gad_tab_controls, 1101);
		scrollbar_pSize->setMax(100); 
		scrollbar_pSize->setPos(50);
		char message[50]; sprintf(message,"p rad [m]: %g",particleSize0);
		text_pSize = mapp->GetIGUIEnvironment()->addStaticText( core::stringw(message).c_str(), 
			rect<s32>(160,y1+50,300,y1+65), false,false,gad_tab_controls);
		this->currParticleSize = particleSize0;	// set the IC

		// particle rad Deviation slider	(id = 1102)
		scrollbar_pDev = mapp->GetIGUIEnvironment()->addScrollBar(true, 
			rect<s32>(20, y1+80, 150, y1+95), gad_tab_controls, 1102);
		scrollbar_pDev->setMax(100); 
		scrollbar_pDev->setPos(50);
		char message1[50]; sprintf(message1,"p dev.[m]: %g",particleDev0);
		text_pDev = mapp->GetIGUIEnvironment()->addStaticText(core::stringw(message1).c_str(),
			rect<s32>(160,y1+80,300,y1+95), false,false,gad_tab_controls);
		this->currParticleDev = particleDev0;	// set the IC for the slider

		// nParticlesGen slider ( id = 1104)
		scrollbar_nParticlesGen = mapp->GetIGUIEnvironment()->addScrollBar(true, 
			rect<s32>(20, y1+110, 150, y1+125), gad_tab_controls, 1104);
		scrollbar_nParticlesGen->setMax(100);
		scrollbar_nParticlesGen->setPos(50);
		this->currNparticlesGen = nParticlesGenMax/2;	// IC of this slider
		char message2[50]; sprintf(message2,"# p Gen: %d",this->currNparticlesGen);
		text_nParticlesGen = mapp->GetIGUIEnvironment()->addStaticText(core::stringw(message2).c_str(),
			rect<s32>(160, y1+110, 300,y1+125), false,false,gad_tab_controls);

		// friction coefficient of particles, id = 1105
		scrollbar_particleFriction = mapp->GetIGUIEnvironment()->addScrollBar(true,
			rect<s32>(20, y1+140, 150, y1+155), gad_tab_controls, 1105);
		scrollbar_particleFriction->setMax(100);
		scrollbar_particleFriction->setPos(33);
		this->currParticleFriction = 0.33;
		char message3[50]; sprintf(message3,"mu: %g", this->currParticleFriction);
		text_particleFriction = mapp->GetIGUIEnvironment()->addStaticText(core::stringw(message3).c_str(), 
			rect<s32>(160, y1+140, 300,y1+155), false,false,gad_tab_controls);

		// particle density, id = 1106
		scrollbar_particleDensity = mapp->GetIGUIEnvironment()->addScrollBar(true, 
			rect<s32>(20, y1+170, 150, y1+185), gad_tab_controls, 1106);
		scrollbar_particleDensity->setMax(100);
		scrollbar_particleDensity->setPos(50);
		this->avgDensity = this->mgenerator->get_sphDensity();
		char message4[50]; sprintf(message4,"rho [kg/m3]: %g", this->avgDensity);
		text_particleDensity = mapp->GetIGUIEnvironment()->addStaticText(core::stringw(message4).c_str(),
			rect<s32>(160, y1+170, 300,y1+185),false,false,gad_tab_controls);

		// ******* GUI WHEEL STATE
		// Data I care about:
		// wheel CM pos
		ChVector<> cm = mwheel->wheel->GetPos();
		char message5[100]; sprintf(message5,"CM pos, x: %4.4g, y: %4.4g, z: %4.4g",cm.x,cm.y,cm.z);
		text_cmPos = mapp->GetIGUIEnvironment()->addStaticText(core::stringw(message5).c_str(),
			rect<s32>(10,30,280,45),false,false,gad_tab_wheel);
		// wheel CM vel
		ChVector<> cmVel = mwheel->wheel->GetPos_dt();
		char messageV[100]; sprintf(messageV,"CM vel, x: %4.4g, y: %4.4g, z: %4.4g",cmVel.x,cmVel.y,cmVel.z);
		text_cmVel = mapp->GetIGUIEnvironment()->addStaticText(core::stringw(message5).c_str(),
			rect<s32>(10,60,280,75),false,false,gad_tab_wheel);
		// rxn. forces on spindle, in the local coordinate system
		ChVector<> rxnF = mtester->spindle->Get_react_force();
		char messageF[100]; sprintf(messageF,"spindle Rxn. F, x: %4.3g, y: %4.3g, z: %4.3g",rxnF.x,rxnF.y,rxnF.z);
		text_spindleForces = mapp->GetIGUIEnvironment()->addStaticText(core::stringw(message5).c_str(),
			rect<s32>(10,90,280,105),false,false,gad_tab_wheel);		
		// rxn. torques on spindle, in local coordinate system
		ChVector<> rxnT = mtester->spindle->Get_react_torque();
		char messageT[100]; sprintf(messageT,"spindle Rxn. T, x: %4.3g, y: %4.3g, z: %4.3g", rxnT.x, rxnT.y, rxnT.z);
		text_spindleTorque = mapp->GetIGUIEnvironment()->addStaticText(core::stringw(messageT).c_str(),
			rect<s32>(10,120, 280, 135),false,false,gad_tab_wheel);



		// ******* GUI PARTICLE STATE
		// Data I care about:
		//	average particle size: pRadMean
		//	running/continuous std. dev	:  pRadStdDev
		// total particle mass:	totalParticleMass
		// average particle mass: pMassMean
		// running/continuous std. dev of mass: pMassStdDev 
		stats pStats = this->mgenerator->get_stats();

    char messageRad[100]; sprintf(messageRad,"p Rad mean, std. dev: %4.4g, %4.4g",
      pStats.pRadMean, pStats.pRadStdDev);
		text_pRad = mapp->GetIGUIEnvironment()->addStaticText(core::stringw(messageRad).c_str(),
			rect<s32>(10,30, 280, 45),false,false,gad_tab_soil);
		char messageMass[100]; sprintf(messageMass,"p mass mean, std. dev: %4.4g, %4.4g",
      pStats.pMassMean, pStats.pMassStdDev);
		text_pMass = mapp->GetIGUIEnvironment()->addStaticText(core::stringw(messageMass).c_str(),
			rect<s32>(10,60, 280, 75),false,false,gad_tab_soil);
	}

	bool OnEvent(const SEvent& event)
	{
	
		// check if user moved the sliders with mouse..
		if (event.EventType == EET_GUI_EVENT)
		{
			s32 id = event.GUIEvent.Caller->getID();
			gui::IGUIEnvironment* env = mapp->GetIGUIEnvironment();

			switch(event.GUIEvent.EventType)
			{
				case EGET_SCROLL_BAR_CHANGED:
				if( id == 1101) // id of particle size slider
				{
					s32 currPos = ((IGUIScrollBar*)event.GUIEvent.Caller)->getPos();
					currParticleSize = particleSize0 + ((currPos - 50)/50.0)*particleSize0 + 0.001;
					char message[50]; sprintf(message,"p rad [m]: %g",currParticleSize);
					text_pSize->setText(core::stringw(message).c_str());
				}
				if( id == 1102) // id of particle Dev slider
				{
					s32 currPos = ((IGUIScrollBar*)event.GUIEvent.Caller)->getPos();
					currParticleDev = particleDev0 + ((currPos - 50)/50.0)*particleDev0 + 0.001;
					char message[50]; sprintf(message,"p dev.[m]: %g",currParticleDev);
					text_pDev->setText(core::stringw(message).c_str());
				}
				if( id == 1103) // torque slider
				{
					s32 currPos = ((IGUIScrollBar*)event.GUIEvent.Caller)->getPos();
					double torquenew = ((currPos - 50.0)/50.0)*maxTorque;
					char message[50]; sprintf(message,"Torque[N/m]: %g",torquenew);
					text_torque->setText(core::stringw(message).c_str());
					// set the new torque to the tester
					mtester->currTorque = torquenew;	// set the new torque
				}
				if( id == 1104) // # particles to generate
				{
					s32 currPos = ((IGUIScrollBar*)event.GUIEvent.Caller)->getPos();
					currNparticlesGen =  nParticlesGenMax + ((currPos - 50)/50.0)*nParticlesGenMax;
					char message[50]; sprintf(message,"# p Gen: %d",this->currNparticlesGen);
					text_nParticlesGen->setText(core::stringw(message).c_str());
				}
				if( id == 1105) // mu of particlers
				{
					s32 sliderPos = ((IGUIScrollBar*)event.GUIEvent.Caller)->getPos();
					this->currParticleFriction = sliderPos / 100.0;
					char message[50]; sprintf(message,"mu: %g",this->currParticleFriction);
					text_particleFriction->setText(core::stringw(message).c_str());
					// set the friction of the particles generated
					mgenerator->set_mu( sliderPos / 100.0 );
				}
				if( id == 1106) // density of spheres
				{
					s32 sliderPos = ((IGUIScrollBar*)event.GUIEvent.Caller)->getPos();
					double density = this->avgDensity +  ((sliderPos - 50.0) /100.0) * this->avgDensity;
					char message[50]; sprintf(message,"rho [kg/m3]: %g", density);
					text_particleDensity->setText(core::stringw(message).c_str());
					// now, set the Sph density in the particle generator
					mgenerator->set_sphDensity( density );

				}
			break;
			case gui::EGET_CHECKBOX_CHANGED:
				if( id == 2110)
				{
					wheelLocked =  checkbox_wheelLocked->isChecked();
					GetLog() << checkbox_wheelLocked->isChecked() << "\n";
					// activate/deactivate motion for the wheel, truss and suspweight
					mwheel->wheel->SetBodyFixed( wheelLocked);
					mtester->suspweight->SetBodyFixed( wheelLocked);
					mtester->spindleBody->SetBodyFixed( wheelLocked);
					return true;
				}
				if( id == 2111)
				{
					makeParticles = checkbox_createParticles->isChecked();
					GetLog() << checkbox_createParticles->isChecked() << "\n";
					// if checked, report the total # of particles
          char message[50]; sprintf(message,"create Particles?: %d",mgenerator->get_totalParticles() );
					text_createParticles->setText(core::stringw(message).c_str());
          GetLog() << "total particle mass = " << this->mgenerator->get_totalParticleMass() << "\n";
					return true;
				}
				if( id == 2112)
				{
					wheelCollision = checkbox_wheelCollision->isChecked();
					GetLog() << checkbox_wheelCollision->isChecked() << "\n";
					// activate/deactivate the wheel collision detection
					this->mwheel->wheel->SetCollide(wheelCollision);
					return true;
				}
			
				if( id == 2114)
				{
					pVisible = checkbox_particlesVisible->isChecked();
					GetLog() << checkbox_particlesVisible->isChecked() << "\n";
					// turn off the particle visibility
					this->mgenerator->toggleVisibility(pVisible);
				}
			break;
			
			}
			
		} 

		return false;
	}

	void drawSprings()
	{
		std::vector<chrono::ChLink*>::iterator iterlink =  mapp->GetSystem()->Get_linklist()->begin();
		// .. draw the spring constraints as simplified spring helix
		iterlink =  mapp->GetSystem()->Get_linklist()->begin();
		while(iterlink != mapp->GetSystem()->Get_linklist()->end())
		{
			if (ChLinkSpring* mylinkspri = ChDynamicCast(ChLinkSpring,(*iterlink)))
				ChIrrTools::drawSpring(mapp->GetVideoDriver(), 0.05, 
					mylinkspri->GetEndPoint1Abs(),
					mylinkspri->GetEndPoint2Abs(),
					video::SColor(255,   150,20,20),   80,  15,  true);
			iterlink++;
		}
	}

	void drawGrid()
	{	

		// wall 1
		ChCoordsys<> wall1Csys = this->mtester->wall1->GetCoord();
		wall1Csys.rot = chrono::Q_from_AngAxis(CH_C_PI/2.0, VECT_Y);
		wall1Csys.pos.x += .05;
		ChIrrTools::drawGrid(this->mapp->GetVideoDriver(),0.1,0.05,24,20, wall1Csys,
			video::SColor(255,80,130,130),true);
/*
		// wall 2
		ChCoordsys<> wall2Csys = this->mtester->wall2->GetBody()->GetCoord();
		wall2Csys.pos.x -= .05;
		wall2Csys.rot = chrono::Q_from_AngAxis(CH_C_PI/2.0, VECT_Y);
		ChIrrTools::drawGrid(this->app->GetVideoDriver(),0.1,0.05,24,20, wall2Csys,
			video::SColor(255,80,130,130),true);
*/
		// wall 3
		ChCoordsys<> wall3Csys = this->mtester->wall3->GetCoord();
		wall3Csys.pos.z += .05;
		ChIrrTools::drawGrid(this->mapp->GetVideoDriver(),0.1,0.05,10,20, wall3Csys, 
			video::SColor(255,80,130,130),true);

		// wall 4
		ChCoordsys<> wall4Csys = this->mtester->wall4->GetCoord();
		wall4Csys.pos.z -= .05;
		ChIrrTools::drawGrid(this->mapp->GetVideoDriver(),0.1,0.05,10,20, wall4Csys,
			video::SColor(255,80,130,130),true);
	}

	// output any relevant test rig data here
	void drawWheelOutput()
	{
		ChVector<> cm = mwheel->wheel->GetPos();
		char messageCM[100]; sprintf(messageCM,"CM pos, x: %4.4g, y: %4.4g, z: %4.4g",cm.x,cm.y,cm.z);
		text_cmPos->setText(core::stringw(messageCM).c_str());
		// wheel CM vel
		ChVector<> cmVel = mwheel->wheel->GetPos_dt();
		char messageV[100]; sprintf(messageV,"CM vel, x: %4.4g, y: %4.4g, z: %4.4g",cmVel.x,cmVel.y,cmVel.z);
		text_cmVel->setText( core::stringw(messageV).c_str() );
		// rxn. forces on spindle
		ChVector<> rxnF = mtester->spindle->Get_react_force();
		char messageF[100]; sprintf(messageF,"spindle Rxn. F, x: %4.3g, y: %4.3g, z: %4.3g",rxnF.x,rxnF.y,rxnF.z);
		text_spindleForces->setText( core::stringw(messageF).c_str() );
		// rxn. torques on spindle
		ChVector<> rxnT = mtester->spindle->Get_react_torque();
		char messageT[100]; sprintf(messageT,"spindle Rxn. T, x: %4.3g, y: %4.3g, z: %4.3g", rxnT.x, rxnT.y, rxnT.z);
		text_spindleTorque->setText( core::stringw(messageT).c_str() );

		// draw reaction forces on the wheel, to make sure they're the correct output
	//	if( 

	}

	void drawSoilOutput()
	{

    // TODO:
    /*
		std::vector<double> particleStats = this->mgenerator->getStatistics();
		char messageRad[100]; sprintf(messageRad,"p Rad mean, std. dev: %4.4g, %4.4g",particleStats[0],particleStats[1]);
		text_pRad->setText( core::stringw(messageRad).c_str() );

		char messageMass[100]; sprintf(messageMass,"p mass mean, std. dev: %4.4g, %4.4g",
			particleStats[6],particleStats[7]);
		text_pMass->setText( core::stringw(messageMass).c_str() );
    */
	}

	// helper functions, these are called in the time step loop
	double getCurrentPsize(){	return currParticleSize; }
	double getCurrentPdev(){ return currParticleDev; }
	bool createParticles(){	return makeParticles; }

	// try to generate some particles. Returne T/F if anything was created
	bool genParticles() {
    // TODO;
		return mgenerator->create_some_falling_items(currParticleSize, currParticleDev, currNparticlesGen,	0);
	}

private:
	ChIrrApp* mapp;
	// bodies/joints
	ChSharedPtr<SoilbinWheel>& mwheel;
	ChSharedPtr<LinearTestRig>& mtester;
  ChSharedPtr<ParticleGenerator>& mgenerator;

	// for check boxes
	bool wheelLocked;	// id = 2110
	bool makeParticles; // 2111
	bool wheelCollision;// 2112
//	bool applyTorque;	// 2113
	bool pVisible;	//	2114
	bool wheelVisible;	// 2115

	// particle size, deviation
	double particleSize0;		// initial
	double currParticleSize;	// current value
	double particleDev0;		// initial
	double currParticleDev;		// current val
	double maxTorque;		// max torque applied to wheel
	int nParticlesGenMax;	// max number of particles to generate
	int currNparticlesGen;	// # of particles to generate this step
	double currParticleFriction;	// coulomb friction coef, between 0-1
	double avgDensity;		// input/average density for soil particles

	// menu items, checkboxes ids are: 2xxx
//	gui::IGUIContextMenu* menu;
	gui::IGUICheckBox*	 checkbox_wheelLocked; // ic = 2110
	gui::IGUIStaticText* text_wheelLocked;
	gui::IGUICheckBox*	checkbox_createParticles; // id = 2111
	gui::IGUIStaticText* text_createParticles;
	gui::IGUICheckBox*	checkbox_wheelCollision;	// id = 2112
	gui::IGUIStaticText* text_wheelCollision;
//	gui::IGUICheckBox*	checkbox_applyTorque;	// id = 2113
//	gui::IGUIStaticText* text_applyTorque;
	gui::IGUICheckBox*	checkbox_particlesVisible;	// id = 2114
	gui::IGUIStaticText*	text_particlesVisible;
	gui::IGUICheckBox*		checkbox_wheelVisible;	// id = 2115
	gui::IGUIStaticText*	text_wheelVisible;

	// scroll bars, ids are: 1xxx
	IGUIScrollBar* scrollbar_pSize;	// particle size, id = 1101
	IGUIStaticText* text_pSize;
	IGUIScrollBar* scrollbar_pDev;	// deviation of mean particle size, id = 1102
	IGUIStaticText* text_pDev;	
	IGUIScrollBar* scrollbar_torque; // torque applied to wheel, id = 1103
	IGUIStaticText* text_torque;
	IGUIScrollBar* scrollbar_nParticlesGen;	// particles to spawn, id = 1104
	IGUIStaticText* text_nParticlesGen;
	IGUIScrollBar* scrollbar_particleFriction;	// friction coefficient of particles, id = 1105
	IGUIStaticText* text_particleFriction;
	IGUIScrollBar*	scrollbar_particleDensity; 		// particle density, id = 1106
	IGUIStaticText*	text_particleDensity;

	// output tabs, and their text boxes

	gui::IGUIStaticText* gad_text_wheelControls;
	gui::IGUIStaticText* gad_text_pControls;

	gui::IGUIStaticText* gad_text_wheelState;	// panel for all wheel state output data
	gui::IGUIStaticText* text_cmPos;
	gui::IGUIStaticText* text_cmVel;
	gui::IGUIStaticText* text_spindleForces;	// spindle reaction forces, torques
	gui::IGUIStaticText* text_spindleTorque;

	gui::IGUIStaticText* gad_text_soilState;	// panel for all soil state output data
	gui::IGUIStaticText* text_pRad;
	gui::IGUIStaticText* text_pMass;
};


int main(int argc, char* argv[])
{
	// Create a ChronoENGINE physical system
	ChSystem m_System;

	// ** user input
	double wheelMass = 5.0;	// mass of wheel
	double suspMass = 10.0;	// mass of suspended weight

	// Create the Irrlicht visualization (open the Irrlicht device, 
	// bind a simple user interface, etc. etc.)
	char header[150]; sprintf(header,"soil bin, mass wheel/weight = %g, %g ",wheelMass, suspMass);
	ChIrrApp app(&m_System, core::stringw(header).c_str(),core::dimension2d<u32>(1024,768),false);


	// Easy shortcuts to add camera, lights, logo and sky in Irrlicht scene:
	ChIrrWizard::add_typical_Logo(app.GetDevice());
	ChIrrWizard::add_typical_Sky(app.GetDevice());
	ChIrrWizard::add_typical_Lights(app.GetDevice(),
		irr::core::vector3df(20.,30.,25.), irr::core::vector3df(25.,25.,-25.),
		65.0, 75.);
	ChIrrWizard::add_typical_Camera(app.GetDevice(), core::vector3df(3.5f,2.5f,-2.4f));


	// create the soil bin, with a few initial particles inside
	// create_some_falling_items(mphysicalSystem, application.GetSceneManager(), application.GetVideoDriver());

	// Create the soil bin wheel, using convex hulls for collision geometry
	ChVector<> wheelCMpos = ChVector<>(0,0.5,0);
	ChVector<> wheelInertia = ChVector<>(1.0,1.0,1.0);
	// Use Trelleborg tire, with Alessandro's method of using convex hulls
  double muWheel = 0.7;
	ChSharedPtr<SoilbinWheel> binWheel(new SoilbinWheel(app, wheelCMpos, wheelMass, wheelInertia, muWheel));
	
  /*
  // can also use a cylinder for a collision geometry shape
	double wheel_width = 0.6;
	double wheel_d_outer = 1.15;	// outer radius
	double wheel_d_inner = 0.64;	// inner radius, only used for inertia calculation
	ChSharedPtr<SoilbinWheel> mwheel(new SoilbinWheel(application,	wheelCMpos, wheelMass,	wheel_width, wheel_d_outer, wheel_d_inner));
  */


	// Linear soil testing rig mechanism
	// will be attached to the wheel rigid body w/ a revolute joint on the spindleBody
	double binWidth = 1.0;
	double binLen = 2.4;
	ChSharedPtr<LinearTestRig> testRig(new LinearTestRig( binWheel->wheel, app, m_System,
		binWidth, binLen, suspMass, 2500., 10.));
	

  // ChParticleEmitter, to control creation of particles
	// ***** PARTICLE GENERATOR
	// make a particle generator, that the sceneManager can use to easily dump a bunch of dirt in the bin
	
  
  ChSharedPtr<ParticleGenerator> particleGen(new ParticleGenerator(&m_System, &app, binWidth, binLen));

	// ***** Create the User - GUI
	double torqueMax = 50.;
  MyEventReceiver receiver(&app, binWheel, testRig, particleGen, 0.02, 0.02, torqueMax);
	 // add a custom event receiver to the default interface:
	app.SetUserEventReceiver(&receiver);

	//set some integrator settings
	// mphysicalSystem.SetLcpSolverType(ChSystem::LCP_ITERATIVE_APGD);	// see if Toby's works
	m_System.SetLcpSolverType(ChSystem::LCP_ITERATIVE_SOR_MULTITHREAD);
	m_System.SetIterLCPmaxItersSpeed(70);
	m_System.SetIterLCPmaxItersStab(15);
	m_System.SetParallelThreadNumber(4);

	// Use real-time step of the simulation, OR...
	app.SetStepManage(true);
	app.SetTimestep(0.01);
	app.SetTryRealtime(true);

	while(app.GetDevice()->run()) 
	{
		app.GetVideoDriver()->beginScene(true, true, SColor(255,140,161,192));
		app.DrawAll();
		// draw the custom links
		receiver.drawSprings();
		receiver.drawGrid();
		// output relevant soil, wheel data if the tab is selected
		if( receiver.gad_tab_soil->isVisible() )
			receiver.drawSoilOutput();
		if( receiver.gad_tab_wheel->isVisible() )
			receiver.drawWheelOutput();
		receiver.drawWheelOutput();
		// apply torque to the wheel
		testRig->applyTorque();

		app.DoStep();
		if( !app.GetPaused())
		{
			// add bodies to the system?
			if( receiver.createParticles() )
			{
				receiver.genParticles();
			}
		}
		app.GetVideoDriver()->endScene();  
	}
	

	return 0;
}
  
