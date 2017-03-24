// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All right reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Justin Madsen, Radu Serban
// =============================================================================
//
//  Demo code about
//    - Creating a soil bin test rig mechanism
//    - A particle generating class to control some particle creation properties
//	  - Torque drives a rigid tire over the material, in-plane
//    - Irrlicht event receiver, to drive/modify particle properties with the GUI
//
// =============================================================================

#include <algorithm>

#include "chrono/core/ChRealtimeStep.h"
#include "chrono/geometry/ChTriangleMeshConnected.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChSystem.h"

#include "chrono_irrlicht/ChIrrApp.h"

using namespace chrono;
using namespace chrono::geometry;
using namespace chrono::collision;
using namespace chrono::irrlicht;

using namespace irr;
using namespace irr::core;
using namespace irr::scene;
using namespace irr::video;
using namespace irr::io;
using namespace irr::gui;

class ParticleGenerator {
  public:
    ParticleGenerator(ChIrrApp* application,
                      ChSystem* mphysicalSystem,
                      double width,
                      double len,
                      double sphDensity = 100.0,
                      double boxDensity = 100.0,
                      double mu = 0.33) {
        this->app = application;
        this->msys = mphysicalSystem;
        this->bedLength = len;
        this->bedWidth = width;
        this->sphDens = sphDensity;
        this->boxDens = boxDensity;
        this->simTime_lastPcreated = 0.0;
        this->totalParticles = 0;
        this->totalParticleMass = 0.0;
        this->mu = (float)mu;

        // keep track of some statistics
        this->pRadMean = 0.0;
        this->pRadStdDev = 0.0;
        this->pRad_s1 = 0.0;
        this->pRad_s2 = 0.0;
        this->pMass_s2 = 0.0;
        this->pMassMean = 0.0;
        this->pMassStdDev = 0.0;
    }

    ~ParticleGenerator() {}

    // return the total # of particles
    const int nparticles() { return this->totalParticles; }

    // return the total particle mass
    const double particleMass() { return (this->totalParticleMass); }

    const double getMu() { return this->mu; }

    void setMu(double newMu) {
        if (newMu < 0.0) {
            GetLog() << "can't set mu less than 0  \n";
            return;
        }
        if (newMu > 1.0)
            GetLog() << "probably shouldn't have mu > 1.0   \n";

        // set mu anyway if >1.0
        this->mu = (float)newMu;
    }

    const double getSphDensity() { return this->sphDens; }
    void setSphDensity(double newDens) {
        if (newDens < 0.0)
            GetLog() << "can't set density less than 0  \n";
        else
            this->sphDens = newDens;
    }

    void setBoxDensity(double newDens) {
        if (newDens < 0.0)
            GetLog() << "can't set density less than 0  \n";
        else
            this->boxDens = newDens;
    }

    // create some spheres with size = pSize + ChRank()*pDev
    // also, you can create boxes too, with the sides being sized in the same sort of manner as the spheres
    const bool create_some_falling_items(double pSize, double pDev, int nParticles, int nBoxes = 0) {
        double minTime_betweenCreate = 0.05;  // this much simulation time MUST elapse before being allowed to
                                              // create more particles
        if ((msys->GetChTime() - this->simTime_lastPcreated) >= minTime_betweenCreate) {
            // reset the timer if we get in here
            this->simTime_lastPcreated = msys->GetChTime();

            // generate some dirt in the bin
            auto cubeMap = std::make_shared<ChTexture>();
            cubeMap->SetTextureFilename(GetChronoDataFile("concrete.jpg"));
            auto rockMap = std::make_shared<ChTexture>();
            rockMap->SetTextureFilename(GetChronoDataFile("rock.jpg"));

            // I should really check these
            ChCollisionModel::SetDefaultSuggestedEnvelope(0.003);
            ChCollisionModel::SetDefaultSuggestedMargin(0.002);

            // increment the counters for total # of particles
            this->totalParticles += nParticles;
            this->totalParticles += nBoxes;
            // kind of guess the height of the particle stack
            double stackHeight = (this->totalParticles / 2000.0) * pSize - 0.2;

            // create the spheres
            for (int bi = 0; bi < nParticles; bi++) {
                double sphrad = pSize + pDev * ChRandom();
                double sphmass = (4 / 3) * CH_C_PI * pow(sphrad, 3) * this->sphDens;
                ChQuaternion<> randrot(ChRandom(), ChRandom(), ChRandom(), ChRandom());
                randrot.Normalize();
                // randomize spawning position, take stack height into consideration
                ChVector<> currPos = ChVector<>(-0.5 * bedWidth + ChRandom() * bedWidth,
                                                stackHeight + 2 * pSize * ((double)bi / (20.0 * ChRandom() + 50.0)),
                                                -0.5 * bedLength + ChRandom() * bedLength);
                auto currRigidBody = std::make_shared<ChBodyEasySphere>(sphrad, this->sphDens, true, true);
                currRigidBody->SetPos(currPos);
                currRigidBody->GetMaterialSurface()->SetFriction(this->mu);
                currRigidBody->SetRot(randrot);
                currRigidBody->AddAsset(rockMap);

                msys->AddBody(currRigidBody);
                app->AssetBind(currRigidBody);
                app->AssetUpdate(currRigidBody);

                // every time we add a body, increment the counter and mass
                this->totalParticleMass += sphmass;
                this->pMass_s2 += sphmass * sphmass;
                this->pRad_s1 += sphrad;
                this->pRad_s2 += sphrad * sphrad;
            }

            // create the boxes
            for (int bi = 0; bi < nBoxes; bi++) {
                double xscale = 1.5 * ChRandom();  // scale 2 of the 3 dimensions
                double yscale = 2.0;
                double zscale = 1.5 * ChRandom();
                double boxmass = (pSize * xscale) * (pSize * yscale) * (pSize * zscale) * this->boxDens;
                // position found the same way as the spheres
                ChVector<> currPos = ChVector<>(-0.5 * bedWidth + ChRandom() * bedWidth,
                                                stackHeight + 2 * pSize * ((double)bi / (20.0 * ChRandom() + 20.0)),
                                                -0.5 * bedLength + ChRandom() * bedLength);

                // randomize the initial orientation
                ChQuaternion<> randrot(ChRandom(), ChRandom(), ChRandom(), ChRandom());
                randrot.Normalize();
                // create the body object
                auto currRigidBody = std::make_shared<ChBodyEasyBox>(pSize * xscale, pSize * yscale, pSize * zscale, this->boxDens, true, true);
                currRigidBody->SetPos(currPos);
                currRigidBody->SetRot(randrot);
                currRigidBody->GetMaterialSurface()->SetFriction(0.5);
                currRigidBody->AddAsset(cubeMap);

                msys->AddBody(currRigidBody);
                app->AssetBind(currRigidBody);
                app->AssetUpdate(currRigidBody);

                this->totalParticles++;
                this->totalParticleMass += boxmass;
                this->pMass_s2 += boxmass * boxmass;
            }

            // update the statistics
            this->pRadMean = pRad_s1 / (double)totalParticles;
            this->pRadStdDev = sqrt((double)totalParticles * pRad_s2 - pRad_s1 * pRad_s1) / (double)totalParticles;
            this->pMassMean = this->totalParticleMass / (double)totalParticles;
            this->pMassStdDev = sqrt((double)totalParticles * pMass_s2 - totalParticleMass * totalParticleMass) /
                                (double)totalParticles;

            // created particles this step
            return true;
        }

        // did not create particles this time step
        return false;
    }

    // output in the same order as in class list
    std::vector<double> getStatistics() {
        int nStats = 9;
        std::vector<double> out;
        out.resize(9);
        out[0] = pRadMean;
        out[1] = pRadStdDev;
        out[2] = pRad_s1;
        out[3] = pRad_s2;
        out[4] = totalParticleMass;
        out[5] = pMass_s2;
        out[6] = pMassMean;
        out[7] = pMassStdDev;

        return out;
    }

  private:
    ChIrrApp* app;
    ChSystem* msys;
    int totalParticles;
    double totalParticleMass;
    double bedLength;
    double bedWidth;
    double simTime_lastPcreated;  // keep track of the sim time when trying to creatye particles

    // density of shape primitives
    double sphDens;  // material density for spheres
    double boxDens;  // material density for boxes
    float mu;        // friction coef

    // for statistics
    double pRadMean;     // running mean of particle rad
    double pRadStdDev;   // running std. dev. of particle rad
    double pRad_s1;      // running sum of radius
    double pRad_s2;      // running square of radius
    double pMass_s2;     // running square of mass
    double pMassMean;    // running mean of mass
    double pMassStdDev;  // running std. dev. of mass
};

class SoilbinWheel {
  public:
    std::shared_ptr<ChBody> wheel;

    // Use convex decomposition for collision detection with the Trelleborg tire
    SoilbinWheel(ChSystem* system, ChVector<> mposition, double mass, ChVector<>& inertia) {
        ChCollisionModel::SetDefaultSuggestedEnvelope(0.005);
        ChCollisionModel::SetDefaultSuggestedMargin(0.004);

        // Create the wheel body
        wheel = std::make_shared<ChBody>();
        wheel->SetPos(mposition);
        wheel->SetMass(mass);
        wheel->SetInertiaXX(inertia);
        wheel->GetMaterialSurface()->SetFriction(0.4f);
        wheel->SetCollide(true);

        // Visualization mesh
        ChTriangleMeshConnected tireMesh;
        tireMesh.LoadWavefrontMesh(GetChronoDataFile("tractor_wheel.obj"), true, true);
        auto tireMesh_asset = std::make_shared<ChTriangleMeshShape>();
        tireMesh_asset->SetMesh(tireMesh);
        wheel->AddAsset(tireMesh_asset);

        // Contact mesh
        wheel->GetCollisionModel()->ClearModel();
        // Describe the (invisible) colliding shape by adding the 'carcass' decomposed shape and the
        // 'knobs'. Since these decompositions are only for 1/15th of the wheel, use for() to pattern them.
        for (double mangle = 0; mangle < 360.; mangle += (360. / 15.)) {
            ChQuaternion<> myrot;
            ChStreamInAsciiFile myknobs(GetChronoDataFile("tractor_wheel_knobs.chulls").c_str());
            ChStreamInAsciiFile myslice(GetChronoDataFile("tractor_wheel_slice.chulls").c_str());
            myrot.Q_from_AngAxis(mangle * (CH_C_PI / 180.), VECT_X);
            ChMatrix33<> mm(myrot);
            wheel->GetCollisionModel()->AddConvexHullsFromFile(myknobs, ChVector<>(0, 0, 0), mm);
            wheel->GetCollisionModel()->AddConvexHullsFromFile(myslice, ChVector<>(0, 0, 0), mm);
        }
        wheel->GetCollisionModel()->BuildModel();

        // Add wheel body to system
        system->AddBody(wheel);
    }

    ~SoilbinWheel() {}
};

// create a test mechanism made up of 2 bodies
// a hub to connect to the wheel spindle and apply a torque through it
// a weight that moves vertically and horizontally w.r.t. the wheel spindle CM location
// spring/damper to apply a vertical load to the tire
// Purpose: only allow the tire to operate In-Plane, to simulate how a soil bin test mechanism works
class TestMech {
  public:
    // data
    std::shared_ptr<ChBodyEasyBox> truss;       // spindle truss
    std::shared_ptr<ChBodyEasyBox> suspweight;  // suspended weight
    std::shared_ptr<ChBodyEasyBox> floor;
    std::shared_ptr<ChBodyEasyBox> wall1;
    std::shared_ptr<ChBodyEasyBox> wall2;
    std::shared_ptr<ChBodyEasyBox> wall3;
    std::shared_ptr<ChBodyEasyBox> wall4;
    std::shared_ptr<ChLinkSpring> spring;
    std::shared_ptr<ChLinkEngine> torqueDriver;
    std::shared_ptr<ChLinkLockRevolute> spindle;

    // GUI-tweaked data
    bool isTorqueApplied;
    double currTorque;

    // functions
    TestMech(ChSystem* system,
             std::shared_ptr<ChBody> wheelBody,
             double binWidth = 1.0,
             double binLength = 2.0,
             double weightMass = 100.0,
             double springK = 25000,
             double springD = 100) {
        ChCollisionModel::SetDefaultSuggestedEnvelope(0.003);
        ChCollisionModel::SetDefaultSuggestedMargin(0.002);

        ChQuaternion<> rot;
        rot.Q_from_AngAxis(ChRandom() * CH_C_2PI, VECT_Y);

        // *******
        // Create a soil bin with planes. bin width = x-dir, bin length = z-dir
        // Note: soil bin depth will always be ~ 1m
        // *******
        double binHeight = 1.0;
        double wallWidth = std::min<double>(binWidth, binLength) / 10.0;  // wall width = 1/10 of min of bin dims

        // create the floor
        auto cubeMap = std::make_shared<ChTexture>();
        cubeMap->SetTextureFilename(GetChronoDataFile("concrete.jpg"));

        floor = std::make_shared<ChBodyEasyBox>(binWidth + wallWidth / 2.0, wallWidth, binLength + wallWidth / 2.0, 1.0, true, true);
        floor->SetPos(ChVector<>(0, -0.5 - wallWidth / 2.0, 0));
        floor->SetBodyFixed(true);
        floor->GetMaterialSurface()->SetFriction(0.5);
        floor->AddAsset(cubeMap);
        system->AddBody(floor);

        // add some transparent walls to the soilBin, w.r.t. width, length of bin
        wall1 = std::make_shared<ChBodyEasyBox>(wallWidth, binHeight, binLength, 1.0, true, true);
        wall1->SetPos(ChVector<>(-binWidth / 2.0 - wallWidth / 2.0, 0, 0));
        wall1->SetBodyFixed(true);
        system->AddBody(wall1);

        wall2 = std::make_shared<ChBodyEasyBox>(wallWidth, binHeight, binLength, 1.0, true, false);
        wall2->SetPos(ChVector<>(binWidth / 2.0 + wallWidth / 2.0, 0, 0));
        wall2->SetBodyFixed(true);
        system->AddBody(wall2);

        wall3 = std::make_shared<ChBodyEasyBox>(binWidth + wallWidth / 2.0, binHeight, wallWidth, 1.0, true, false);
        wall3->SetPos(ChVector<>(0, 0, -binLength / 2.0 - wallWidth / 2.0));
        wall3->SetBodyFixed(true);
        system->AddBody(wall3);

        // wall 4
        wall4 = std::make_shared<ChBodyEasyBox>(binWidth + wallWidth / 2.0, binHeight, wallWidth, 1.0, true, true);
        wall4->SetPos(ChVector<>(0, 0, binLength / 2.0 + wallWidth / 2.0));
        wall4->SetBodyFixed(true);
        system->AddBody(wall4);

        // ******
        // make a truss, connect it to the wheel via revolute joint
        // single rotational DOF will be driven with a user-input for torque
        // *****
        auto bluMap = std::make_shared<ChTexture>();
        bluMap->SetTextureFilename(GetChronoDataFile("blu.png"));
        ChVector<> trussCM = wheelBody->GetPos();

        truss = std::make_shared<ChBodyEasyBox>(0.2, 0.2, 0.4, 300.0, false, true);
        truss->SetPos(trussCM);
        truss->SetMass(5.0);
        truss->AddAsset(bluMap);
        system->AddBody(truss);

        // create the revolute joint between the wheel and spindle
        spindle = std::make_shared<ChLinkLockRevolute>();
        spindle->Initialize(truss, wheelBody, ChCoordsys<>(trussCM, chrono::Q_from_AngAxis(CH_C_PI / 2, VECT_Y)));
        system->AddLink(spindle);

        // create a torque between the truss and wheel
        torqueDriver = std::make_shared<ChLinkEngine>();
        torqueDriver->Initialize(truss, wheelBody, ChCoordsys<>(trussCM, chrono::Q_from_AngAxis(CH_C_PI / 2, VECT_Y)));
        torqueDriver->Set_eng_mode(ChLinkEngine::ENG_MODE_TORQUE);
        system->AddLink(torqueDriver);

        // ******
        // create a body that will be used as a vehicle weight
        ChVector<> weightCM = ChVector<>(trussCM);
        weightCM.y() += 1.0;  // note: this will determine the spring free length

        suspweight = std::make_shared<ChBodyEasyBox>(0.2, 0.4, 0.2, 5000.0, false, true);
        suspweight->SetPos(weightCM);
        suspweight->SetMass(weightMass);
        suspweight->AddAsset(bluMap);
        system->AddBody(suspweight);

        // create the translational joint between the truss and weight load
        auto translational = std::make_shared<ChLinkLockPrismatic>();
        translational->Initialize(truss, suspweight,
                                  ChCoordsys<>(trussCM, chrono::Q_from_AngAxis(CH_C_PI / 2, VECT_X)));
        system->AddLink(translational);

        // create a spring between spindle truss and weight
        spring = std::make_shared<ChLinkSpring>();
        spring->Initialize(truss, suspweight, false, trussCM, suspweight->GetPos());
        spring->Set_SpringK(springK);
        spring->Set_SpringR(springD);
        system->AddLink(spring);

        // create a prismatic constraint between the weight and the ground
        auto weightLink = std::make_shared<ChLinkLockOldham>();
        weightLink->Initialize(suspweight, floor,
                               ChCoordsys<>(weightCM, chrono::Q_from_AngAxis(CH_C_PI / 2.0, VECT_Y)));
        system->AddLink(weightLink);
    }

    // set the spring and damper constants
    void setSpringKD(double k, double d) {
        this->spring->Set_SpringK(k);
        this->spring->Set_SpringR(d);
    }

    // for now, just use the slider value as directly as the torque
    void applyTorque() {
        // note: negative sign is to get Trelleborg tire to spin in the correct direction
        if (auto mfun = std::dynamic_pointer_cast<ChFunction_Const>(torqueDriver->Get_tor_funct()))
            mfun->Set_yconst(-this->currTorque);
    }

    ~TestMech() {}
};

class MyEventReceiver : public IEventReceiver {
  public:
    // keep the tabs public
    gui::IGUITabControl* gad_tabbed;
    gui::IGUITab* gad_tab_controls;
    gui::IGUITab* gad_tab_wheel;
    gui::IGUITab* gad_tab_soil;

    // @param pSize particle radius
    // @param pDev multiplier added to ChRandom()
    // @param maxTorque max slider torque applied to wheel
    // @param maxParticles max number of particles to generate each spawning event
    MyEventReceiver(ChIrrApp* app,
                    SoilbinWheel* wheel,
                    TestMech* tester,
                    ParticleGenerator* particleGenerator,
                    double pSize = 0.02,
                    double pDev = 0.02,
                    double maxTorque = 100.0,
                    int maxParticles = 50) {
        // store pointer to physical system & other stuff so we can tweak them by user keyboard
        this->mapp = app;
        // any rigid bodies that have their states modified by the GUI need to go here
        this->mwheel = wheel;
        this->mtester = tester;
        this->mgenerator = particleGenerator;
        // for getting output from the TM_Module module
        // initial checkbox values
        this->wheelLocked = true;
        this->makeParticles = false;
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
        irr::s32 x0 = 740;
        irr::s32 y0 = 20;  // box0 top left corner
        // create the tabs for the rig output: NOTE: GUI widget locations are all relative to the TabControl!
        gad_tabbed =
            mapp->GetIGUIEnvironment()->addTabControl(core::rect<s32>(x0, y0, x0 + 255, y0 + 440), 0, true, true);
        gad_tab_controls = gad_tabbed->addTab(L"Controls");  // static text will be printed w/ each checkbox or slider
        gad_text_wheelControls = mapp->GetIGUIEnvironment()->addStaticText(
            L"Wheel Control", core::rect<s32>(10, 10, 245, 150), true, true, gad_tab_controls);
        irr::s32 y1 = 165;  // box1 top left corner
        gad_text_pControls = mapp->GetIGUIEnvironment()->addStaticText(
            L"Particle Control", core::rect<s32>(10, y1, 245, y1 + 230), true, true, gad_tab_controls);
        gad_tab_wheel = gad_tabbed->addTab(L"Wheel State");
        gad_text_wheelState = mapp->GetIGUIEnvironment()->addStaticText(L"WS", core::rect<s32>(10, 10, 290, 250), true,
                                                                        true, gad_tab_wheel);
        gad_tab_soil = gad_tabbed->addTab(L"Soil State");
        gad_text_soilState = mapp->GetIGUIEnvironment()->addStaticText(L"SS", core::rect<s32>(10, 10, 290, 250), true,
                                                                       true, gad_tab_soil);

        // **** GUI CONTROLS ***
        // -------- Wheel controls
        // ..add a GUI for wheel position lock ( id = 2110 )
        checkbox_wheelLocked = app->GetIGUIEnvironment()->addCheckBox(wheelLocked, core::rect<s32>(20, 30, 35, 45),
                                                                      gad_tab_controls, 2110);
        text_wheelLocked = app->GetIGUIEnvironment()->addStaticText(L"Wheel Locked", core::rect<s32>(45, 30, 125, 45),
                                                                    false, false, gad_tab_controls);
        checkbox_wheelLocked->setVisible(true);
        text_wheelLocked->setVisible(true);
        this->mwheel->wheel->SetBodyFixed(wheelLocked);  // set IC of checkbox
        this->mtester->truss->SetBodyFixed(wheelLocked);
        this->mtester->suspweight->SetBodyFixed(wheelLocked);

        // turn wheel visibility on/off, ie = 2115
        ////checkbox_wheelVisible = app->GetIGUIEnvironment()->addCheckBox(wheelVisible, core::rect<s32>(180, 30, 195,
        ///45),
        ////                                                               gad_tab_controls, 2115);
        ////text_wheelVisible = app->GetIGUIEnvironment()->addStaticText(L"visible?", core::rect<s32>(205, 30, 290, 45),
        ////                                                             false, false, gad_tab_controls);

        // add a GUI for setting the wheel collision ( id = 2112 )
        checkbox_wheelCollision = app->GetIGUIEnvironment()->addCheckBox(
            wheelCollision, core::rect<s32>(20, 60, 35, 75), gad_tab_controls, 2112);
        text_wheelCollision = app->GetIGUIEnvironment()->addStaticText(
            L"Wheel collide? ", core::rect<s32>(45, 60, 125, 75), false, false, gad_tab_controls);
        checkbox_wheelCollision->setVisible(true);
        text_wheelCollision->setVisible(true);
        this->mwheel->wheel->SetCollide(wheelCollision);

        // torque slider	(id = 1103)
        scrollbar_torque =
            mapp->GetIGUIEnvironment()->addScrollBar(true, rect<s32>(20, 115, 150, 130), gad_tab_controls, 1103);
        scrollbar_torque->setMax(100);
        scrollbar_torque->setPos(50);
        text_torque = mapp->GetIGUIEnvironment()->addStaticText(L"Torque[N/m]: 0 ", rect<s32>(160, 115, 300, 130),
                                                                false, false, gad_tab_controls);
        this->mtester->currTorque = 0;  // set the IC of this slider

        // -------- Particle Controls
        // add a GUI for turning particle creation on/off ( id = 2111 )
        checkbox_createParticles = app->GetIGUIEnvironment()->addCheckBox(
            makeParticles, core::rect<s32>(20, y1 + 20, 35, y1 + 35), gad_tab_controls, 2111);
        text_createParticles = app->GetIGUIEnvironment()->addStaticText(
            L"create Particles? ", core::rect<s32>(45, y1 + 20, 165, y1 + 35), false, false, gad_tab_controls);
        checkbox_createParticles->setVisible(true);
        text_createParticles->setVisible(true);

        // add a checkbox to make particle visibility turn on/off, id = 2114
        ////checkbox_particlesVisible = app->GetIGUIEnvironment()->addCheckBox(
        ////    pVisible, core::rect<s32>(180, y1 + 20, 195, y1 + 35), gad_tab_controls, 2114);
        ////text_particlesVisible = app->GetIGUIEnvironment()->addStaticText(
        ////    L"visible?", core::rect<s32>(205, y1 + 20, 290, y1 + 35), false, false, gad_tab_controls);

        // create sliders to modify particle size/dev ( id = 1101)
        scrollbar_pSize = mapp->GetIGUIEnvironment()->addScrollBar(true, rect<s32>(20, y1 + 50, 150, y1 + 65),
                                                                   gad_tab_controls, 1101);
        scrollbar_pSize->setMax(100);
        scrollbar_pSize->setPos(50);
        char message[50];
        sprintf(message, "p rad [m]: %g", particleSize0);
        text_pSize = mapp->GetIGUIEnvironment()->addStaticText(
            core::stringw(message).c_str(), rect<s32>(160, y1 + 50, 300, y1 + 65), false, false, gad_tab_controls);
        this->currParticleSize = particleSize0;  // set the IC

        // particle rad Deviation slider	(id = 1102)
        scrollbar_pDev = mapp->GetIGUIEnvironment()->addScrollBar(true, rect<s32>(20, y1 + 80, 150, y1 + 95),
                                                                  gad_tab_controls, 1102);
        scrollbar_pDev->setMax(100);
        scrollbar_pDev->setPos(50);
        char message1[50];
        sprintf(message1, "p dev.[m]: %g", particleDev0);
        text_pDev = mapp->GetIGUIEnvironment()->addStaticText(
            core::stringw(message1).c_str(), rect<s32>(160, y1 + 80, 300, y1 + 95), false, false, gad_tab_controls);
        this->currParticleDev = particleDev0;  // set the IC for the slider

        // nParticlesGen slider ( id = 1104)
        scrollbar_nParticlesGen = mapp->GetIGUIEnvironment()->addScrollBar(true, rect<s32>(20, y1 + 110, 150, y1 + 125),
                                                                           gad_tab_controls, 1104);
        scrollbar_nParticlesGen->setMax(100);
        scrollbar_nParticlesGen->setPos(50);
        this->currNparticlesGen = nParticlesGenMax / 2;  // IC of this slider
        char message2[50];
        sprintf(message2, "# p Gen: %d", this->currNparticlesGen);
        text_nParticlesGen = mapp->GetIGUIEnvironment()->addStaticText(
            core::stringw(message2).c_str(), rect<s32>(160, y1 + 110, 300, y1 + 125), false, false, gad_tab_controls);

        // friction coefficient of particles, id = 1105
        scrollbar_particleFriction = mapp->GetIGUIEnvironment()->addScrollBar(
            true, rect<s32>(20, y1 + 140, 150, y1 + 155), gad_tab_controls, 1105);
        scrollbar_particleFriction->setMax(100);
        scrollbar_particleFriction->setPos(33);
        this->currParticleFriction = 0.33;
        char message3[50];
        sprintf(message3, "mu: %g", this->currParticleFriction);
        text_particleFriction = mapp->GetIGUIEnvironment()->addStaticText(
            core::stringw(message3).c_str(), rect<s32>(160, y1 + 140, 300, y1 + 155), false, false, gad_tab_controls);

        // particle density, id = 1106
        scrollbar_particleDensity = mapp->GetIGUIEnvironment()->addScrollBar(
            true, rect<s32>(20, y1 + 170, 150, y1 + 185), gad_tab_controls, 1106);
        scrollbar_particleDensity->setMax(100);
        scrollbar_particleDensity->setPos(50);
        this->avgDensity = this->mgenerator->getSphDensity();
        char message4[50];
        sprintf(message4, "rho [kg/m3]: %g", this->avgDensity);
        text_particleDensity = mapp->GetIGUIEnvironment()->addStaticText(
            core::stringw(message4).c_str(), rect<s32>(160, y1 + 170, 300, y1 + 185), false, false, gad_tab_controls);

        // ******* GUI WHEEL STATE
        // wheel CM pos
        ChVector<> cm = mwheel->wheel->GetPos();
        char message5[100];
        sprintf(message5, "CM pos, x: %4.4g, y: %4.4g, z: %4.4g", cm.x(), cm.y(), cm.z());
        text_cmPos = mapp->GetIGUIEnvironment()->addStaticText(core::stringw(message5).c_str(),
                                                               rect<s32>(10, 30, 280, 45), false, false, gad_tab_wheel);
        // wheel CM vel
        ChVector<> cmVel = mwheel->wheel->GetPos_dt();
        char messageV[100];
        sprintf(messageV, "CM vel, x: %4.4g, y: %4.4g, z: %4.4g", cmVel.x(), cmVel.y(), cmVel.z());
        text_cmVel = mapp->GetIGUIEnvironment()->addStaticText(core::stringw(message5).c_str(),
                                                               rect<s32>(10, 60, 280, 75), false, false, gad_tab_wheel);
        // rxn. forces on spindle, in the local coordinate system
        ChVector<> rxnF = mtester->spindle->Get_react_force();
        char messageF[100];
        sprintf(messageF, "spindle Rxn. F, x: %4.3g, y: %4.3g, z: %4.3g", rxnF.x(), rxnF.y(), rxnF.z());
        text_spindleForces = mapp->GetIGUIEnvironment()->addStaticText(
            core::stringw(message5).c_str(), rect<s32>(10, 90, 280, 105), false, false, gad_tab_wheel);
        // rxn. torques on spindle, in local coordinate system
        ChVector<> rxnT = mtester->spindle->Get_react_torque();
        char messageT[100];
        sprintf(messageT, "spindle Rxn. T, x: %4.3g, y: %4.3g, z: %4.3g", rxnT.x(), rxnT.y(), rxnT.z());
        text_spindleTorque = mapp->GetIGUIEnvironment()->addStaticText(
            core::stringw(messageT).c_str(), rect<s32>(10, 120, 280, 135), false, false, gad_tab_wheel);

        // ******* GUI PARTICLE STATE
        // average particle size: pRadMean
        // running/continuous std. dev: pRadStdDev
        // total particle mass:	totalParticleMass
        // average particle mass: pMassMean
        // running/continuous std. dev of mass: pMassStdDev
        std::vector<double> particleStats = this->mgenerator->getStatistics();

        char messageRad[100];
        sprintf(messageRad, "p Rad mean, std. dev: %4.4g, %4.4g", particleStats[0], particleStats[1]);
        text_pRad = mapp->GetIGUIEnvironment()->addStaticText(core::stringw(messageRad).c_str(),
                                                              rect<s32>(10, 30, 280, 45), false, false, gad_tab_soil);
        char messageMass[100];
        sprintf(messageMass, "p mass mean, std. dev: %4.4g, %4.4g", particleStats[6], particleStats[7]);
        text_pMass = mapp->GetIGUIEnvironment()->addStaticText(core::stringw(messageMass).c_str(),
                                                               rect<s32>(10, 60, 280, 75), false, false, gad_tab_soil);
    }

    bool OnEvent(const SEvent& event) {
        // check if user moved the sliders with mouse..
        if (event.EventType == EET_GUI_EVENT) {
            s32 id = event.GUIEvent.Caller->getID();
            gui::IGUIEnvironment* env = mapp->GetIGUIEnvironment();

            switch (event.GUIEvent.EventType) {
                case EGET_SCROLL_BAR_CHANGED:
                    if (id == 1101)  // id of particle size slider
                    {
                        s32 currPos = ((IGUIScrollBar*)event.GUIEvent.Caller)->getPos();
                        this->currParticleSize = particleSize0 + ((currPos - 50) / 50.0) * particleSize0 + 0.001;
                        char message[50];
                        sprintf(message, "p rad [m]: %g", currParticleSize);
                        text_pSize->setText(core::stringw(message).c_str());
                    }
                    if (id == 1102)  // id of particle Dev slider
                    {
                        s32 currPos = ((IGUIScrollBar*)event.GUIEvent.Caller)->getPos();
                        this->currParticleDev = particleDev0 + ((currPos - 50) / 50.0) * particleDev0 + 0.001;
                        char message[50];
                        sprintf(message, "p dev.[m]: %g", currParticleDev);
                        text_pDev->setText(core::stringw(message).c_str());
                    }
                    if (id == 1103)  // torque slider
                    {
                        s32 currPos = ((IGUIScrollBar*)event.GUIEvent.Caller)->getPos();
                        double torquenew = ((currPos - 50.0) / 50.0) * maxTorque;
                        char message[50];
                        sprintf(message, "Torque[N/m]: %g", torquenew);
                        text_torque->setText(core::stringw(message).c_str());
                        // set the new torque to the tester
                        this->mtester->currTorque = torquenew;  // set the new torque
                    }
                    if (id == 1104)  // # particles to generate
                    {
                        s32 currPos = ((IGUIScrollBar*)event.GUIEvent.Caller)->getPos();
                        this->currNparticlesGen = nParticlesGenMax + int(double(currPos - 50) / 50.0) * nParticlesGenMax;
                        char message[50];
                        sprintf(message, "# p Gen: %d", this->currNparticlesGen);
                        text_nParticlesGen->setText(core::stringw(message).c_str());
                    }
                    if (id == 1105)  // mu of particlers
                    {
                        s32 sliderPos = ((IGUIScrollBar*)event.GUIEvent.Caller)->getPos();
                        this->currParticleFriction = sliderPos / 100.0;
                        char message[50];
                        sprintf(message, "mu: %g", this->currParticleFriction);
                        text_particleFriction->setText(core::stringw(message).c_str());
                        // set the friction of the particles generated
                        this->mgenerator->setMu(sliderPos / 100.0);
                    }
                    if (id == 1106)  // density of spheres
                    {
                        s32 sliderPos = ((IGUIScrollBar*)event.GUIEvent.Caller)->getPos();
                        double density = this->avgDensity + ((sliderPos - 50.0) / 100.0) * this->avgDensity;
                        char message[50];
                        sprintf(message, "rho [kg/m3]: %g", density);
                        text_particleDensity->setText(core::stringw(message).c_str());
                        // now, set the Sph density in the particle generator
                        this->mgenerator->setSphDensity(density);
                    }
                    break;
                case gui::EGET_CHECKBOX_CHANGED:
                    if (id == 2110) {
                        wheelLocked = checkbox_wheelLocked->isChecked();
                        GetLog() << checkbox_wheelLocked->isChecked() << "\n";
                        // activate/deactivate motion for the wheel, truss and suspweight
                        this->mwheel->wheel->SetBodyFixed(wheelLocked);
                        this->mtester->suspweight->SetBodyFixed(wheelLocked);
                        this->mtester->truss->SetBodyFixed(wheelLocked);
                        return true;
                    }
                    if (id == 2111) {
                        makeParticles = checkbox_createParticles->isChecked();
                        GetLog() << checkbox_createParticles->isChecked() << "\n";
                        // if checked, report the total # of particles
                        char message[50];
                        sprintf(message, "create Particles?: %d", this->mgenerator->nparticles());
                        text_createParticles->setText(core::stringw(message).c_str());
                        GetLog() << "total particle mass = " << this->mgenerator->particleMass() << "\n";
                        return true;
                    }
                    if (id == 2112) {
                        wheelCollision = checkbox_wheelCollision->isChecked();
                        GetLog() << checkbox_wheelCollision->isChecked() << "\n";
                        // activate/deactivate the wheel collision detection
                        this->mwheel->wheel->SetCollide(wheelCollision);
                        return true;
                    }
                    /*
                    if (id == 2114) {
                        pVisible = checkbox_particlesVisible->isChecked();
                        GetLog() << checkbox_particlesVisible->isChecked() << "\n";
                        // turn off the particle visibility
                        this->mgenerator->toggleVisibility(pVisible);
                        return true;
                    }
                    */
                    /*
                    if (id == 2115) {
                        wheelVisible = checkbox_wheelVisible->isChecked();
                        GetLog() << checkbox_wheelVisible->isChecked() << "\n";
                        // turn wheel visibility on/off
                        this->mwheel->toggleVisibility(wheelVisible);
                        return true;
                    }
                    */
                    break;
                default:
                    break;
            }
        }

        return false;
    }

    void drawSprings() {
        auto iterlink = mapp->GetSystem()->Get_linklist()->begin();
        // .. draw the spring constraints as simplified spring helix
        iterlink = mapp->GetSystem()->Get_linklist()->begin();
        while (iterlink != mapp->GetSystem()->Get_linklist()->end()) {
            if (ChLinkSpring* mylinkspri = dynamic_cast<ChLinkSpring*>((*iterlink).get()))
                ChIrrTools::drawSpring(mapp->GetVideoDriver(), 0.05, mylinkspri->GetEndPoint1Abs(),
                                       mylinkspri->GetEndPoint2Abs(), video::SColor(255, 150, 20, 20), 80, 15, true);
            iterlink++;
        }
    }

    void drawGrid() {
        // wall 1
        ChCoordsys<> wall1Csys = this->mtester->wall1->GetCoord();
        wall1Csys.rot = chrono::Q_from_AngAxis(CH_C_PI / 2.0, VECT_Y);
        wall1Csys.pos.x() += .05;
        ChIrrTools::drawGrid(this->mapp->GetVideoDriver(), 0.1, 0.05, 24, 20, wall1Csys,
                             video::SColor(255, 80, 130, 130), true);

        // wall 3
        ChCoordsys<> wall3Csys = this->mtester->wall3->GetCoord();
        wall3Csys.pos.z() += .05;
        ChIrrTools::drawGrid(this->mapp->GetVideoDriver(), 0.1, 0.05, 10, 20, wall3Csys,
                             video::SColor(255, 80, 130, 130), true);

        // wall 4
        ChCoordsys<> wall4Csys = this->mtester->wall4->GetCoord();
        wall4Csys.pos.z() -= .05;
        ChIrrTools::drawGrid(this->mapp->GetVideoDriver(), 0.1, 0.05, 10, 20, wall4Csys,
                             video::SColor(255, 80, 130, 130), true);
    }

    // output any relevant test rig data here
    void drawWheelOutput() {
        ChVector<> cm = mwheel->wheel->GetPos();
        char messageCM[100];
        sprintf(messageCM, "CM pos, x: %4.4g, y: %4.4g, z: %4.4g", cm.x(), cm.y(), cm.z());
        text_cmPos->setText(core::stringw(messageCM).c_str());
        // wheel CM vel
        ChVector<> cmVel = mwheel->wheel->GetPos_dt();
        char messageV[100];
        sprintf(messageV, "CM vel, x: %4.4g, y: %4.4g, z: %4.4g", cmVel.x(), cmVel.y(), cmVel.z());
        text_cmVel->setText(core::stringw(messageV).c_str());
        // rxn. forces on spindle
        ChVector<> rxnF = mtester->spindle->Get_react_force();
        char messageF[100];
        sprintf(messageF, "spindle Rxn. F, x: %4.3g, y: %4.3g, z: %4.3g", rxnF.x(), rxnF.y(), rxnF.z());
        text_spindleForces->setText(core::stringw(messageF).c_str());
        // rxn. torques on spindle
        ChVector<> rxnT = mtester->spindle->Get_react_torque();
        char messageT[100];
        sprintf(messageT, "spindle Rxn. T, x: %4.3g, y: %4.3g, z: %4.3g", rxnT.x(), rxnT.y(), rxnT.z());
        text_spindleTorque->setText(core::stringw(messageT).c_str());
    }

    void drawSoilOutput() {
        std::vector<double> particleStats = this->mgenerator->getStatistics();
        char messageRad[100];
        sprintf(messageRad, "p Rad mean, std. dev: %4.4g, %4.4g", particleStats[0], particleStats[1]);
        text_pRad->setText(core::stringw(messageRad).c_str());

        char messageMass[100];
        sprintf(messageMass, "p mass mean, std. dev: %4.4g, %4.4g", particleStats[6], particleStats[7]);
        text_pMass->setText(core::stringw(messageMass).c_str());
    }

    // helper functions, these are called in the time step loop
    const double getCurrentPsize() { return currParticleSize; }
    const double getCurrentPdev() { return currParticleDev; }
    const bool createParticles() { return makeParticles; }

    // try to generate some particles. Returne T/F if anything was created
    const bool genParticles() {
        return mgenerator->create_some_falling_items(currParticleSize, currParticleDev, currNparticlesGen, 0);
    }

  private:
    ChIrrApp* mapp;

    // bodies/joints
    SoilbinWheel* mwheel;
    TestMech* mtester;
    ParticleGenerator* mgenerator;

    // for check boxes
    bool wheelLocked;     // id = 2110
    bool makeParticles;   // 2111
    bool wheelCollision;  // 2112
    bool pVisible;        // 2114
    bool wheelVisible;    // 2115

    // particle size, deviation
    double particleSize0;         // initial
    double currParticleSize;      // current value
    double particleDev0;          // initial
    double currParticleDev;       // current val
    double maxTorque;             // max torque applied to wheel
    int nParticlesGenMax;         // max number of particles to generate
    int currNparticlesGen;        // # of particles to generate this step
    double currParticleFriction;  // coulomb friction coef, between 0-1
    double avgDensity;            // input/average density for soil particles

    // menu items, checkboxes ids are: 2xxx
    //	gui::IGUIContextMenu* menu;
    gui::IGUICheckBox* checkbox_wheelLocked;  // ic = 2110
    gui::IGUIStaticText* text_wheelLocked;
    gui::IGUICheckBox* checkbox_createParticles;  // id = 2111
    gui::IGUIStaticText* text_createParticles;
    gui::IGUICheckBox* checkbox_wheelCollision;  // id = 2112
    gui::IGUIStaticText* text_wheelCollision;
    gui::IGUICheckBox* checkbox_particlesVisible;  // id = 2114
    gui::IGUIStaticText* text_particlesVisible;
    gui::IGUICheckBox* checkbox_wheelVisible;  // id = 2115
    gui::IGUIStaticText* text_wheelVisible;

    // scroll bars, ids are: 1xxx
    IGUIScrollBar* scrollbar_pSize;  // particle size, id = 1101
    IGUIStaticText* text_pSize;
    IGUIScrollBar* scrollbar_pDev;  // deviation of mean particle size, id = 1102
    IGUIStaticText* text_pDev;
    IGUIScrollBar* scrollbar_torque;  // torque applied to wheel, id = 1103
    IGUIStaticText* text_torque;
    IGUIScrollBar* scrollbar_nParticlesGen;  // particles to spawn, id = 1104
    IGUIStaticText* text_nParticlesGen;
    IGUIScrollBar* scrollbar_particleFriction;  // friction coefficient of particles, id = 1105
    IGUIStaticText* text_particleFriction;
    IGUIScrollBar* scrollbar_particleDensity;  // particle density, id = 1106
    IGUIStaticText* text_particleDensity;

    // output tabs, and their text boxes

    gui::IGUIStaticText* gad_text_wheelControls;
    gui::IGUIStaticText* gad_text_pControls;

    gui::IGUIStaticText* gad_text_wheelState;  // panel for all wheel state output data
    gui::IGUIStaticText* text_cmPos;
    gui::IGUIStaticText* text_cmVel;
    gui::IGUIStaticText* text_spindleForces;  // spindle reaction forces, torques
    gui::IGUIStaticText* text_spindleTorque;

    gui::IGUIStaticText* gad_text_soilState;  // panel for all soil state output data
    gui::IGUIStaticText* text_pRad;
    gui::IGUIStaticText* text_pMass;
};

int main(int argc, char* argv[]) {
    // Create a ChronoENGINE physical system
    ChSystem mphysicalSystem;

    // ** user input
    double wheelMass = 5.0;  // mass of wheel
    double suspMass = 10.0;  // mass of suspended weight

    // Create the Irrlicht visualization (open the Irrlicht device,
    // bind a simple user interface, etc. etc.)
    ChIrrApp application(&mphysicalSystem, L"Soil bin demo", core::dimension2d<u32>(1024, 768), false);
    ChIrrWizard::add_typical_Logo(application.GetDevice());
    ChIrrWizard::add_typical_Sky(application.GetDevice());
    ChIrrWizard::add_typical_Lights(application.GetDevice(), irr::core::vector3df(20., 30., 25.),
                                    irr::core::vector3df(25., 25., -25.), 65.0, 75.);
    ChIrrWizard::add_typical_Camera(application.GetDevice(), core::vector3df(3.5f, 2.5f, -2.4f));

    // ******* SOIL BIN WHEEL
    // Create the wheel
    ChVector<> wheelCMpos = ChVector<>(0, 0.5, 0);
    ChVector<> wheelInertia = ChVector<>(1.0, 1.0, 1.0);
    SoilbinWheel* mwheel = new SoilbinWheel(&mphysicalSystem, wheelCMpos, wheelMass, wheelInertia);

    // ***** TESTING MECHANISM
    // now, create the testing mechanism and attach the wheel to it
    double binWidth = 1.0;
    double binLen = 2.4;
    TestMech* mTestMechanism = new TestMech(&mphysicalSystem, mwheel->wheel, binWidth, binLen, suspMass, 2500., 10.);

    // ***** PARTICLE GENERATOR
    // make a particle generator, that the sceneManager can use to easily dump a bunch of dirt in the bin
    ParticleGenerator* mParticleGen = new ParticleGenerator(&application, &mphysicalSystem, binWidth, binLen);

    // Bind visualization assets.
    application.AssetBindAll();
    application.AssetUpdateAll();

    // ***** Create the User - GUI
    double torqueMax = 50.;
    MyEventReceiver receiver(&application, mwheel, mTestMechanism, mParticleGen, 0.02, 0.02, torqueMax);
    // add a custom event receiver to the default interface:
    application.SetUserEventReceiver(&receiver);

    // Set some integrator settings
    // mphysicalSystem.SetSolverType(ChSolver::Type::APGD);
    mphysicalSystem.SetSolverType(ChSolver::Type::SOR_MULTITHREAD);
    mphysicalSystem.SetMaxItersSolverSpeed(70);
    mphysicalSystem.SetMaxItersSolverStab(15);
    mphysicalSystem.SetParallelThreadNumber(4);

    // Use real-time step of the simulation, OR...
    application.SetStepManage(true);
    application.SetTimestep(0.01);
    application.SetTryRealtime(true);

    while (application.GetDevice()->run()) {
        application.BeginScene(true, true, SColor(255, 140, 161, 192));
        application.DrawAll();

        // draw the custom links
        receiver.drawSprings();
        receiver.drawGrid();

        // output relevant soil, wheel data if the tab is selected
        if (receiver.gad_tab_soil->isVisible())
            receiver.drawSoilOutput();
        if (receiver.gad_tab_wheel->isVisible())
            receiver.drawWheelOutput();
        receiver.drawWheelOutput();

        // apply torque to the wheel
        mTestMechanism->applyTorque();

        application.DoStep();

        if (!application.GetPaused()) {
            // add bodies to the system?
            if (receiver.createParticles()) {
                receiver.genParticles();
            }
        }

        application.EndScene();
    }

    return 0;
}
