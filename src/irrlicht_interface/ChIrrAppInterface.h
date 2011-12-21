#ifndef CHIRRAPPINTERFACE_H
#define CHIRRAPPINTERFACE_H

//////////////////////////////////////////////////
//
//   ChIrrAppInterface.h
//
//   FOR IRRLICHT USERS ONLY!
//
//   Class to add some GUI to Irrlicht+Chrono::Engine
//   applications. 
//   Such basic GUI can be used to monitor solver 
//   timings, to change physical system settings easily, 
//   etc. etc. 
//
//   HEADER file for CHRONO,
//	 Multibody dynamics engine
//
// ------------------------------------------------
// 	 Copyright:Alessandro Tasora / DeltaKnowledge
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////

#include <irrlicht.h>
#include "physics/ChSystem.h"

#include "ChIrrCamera.h"
#include "ChIrrWizard.h"
#include "ChDisplayTools.h"
#include "physics/ChLinkSpring.h"
#include "collision/ChCModelBulletBody.h"
#include "core/ChRealtimeStep.h"


namespace irr
{


/// Class to add some GUI to Irrlicht+Chrono::Engine
/// applications. 
/// Such basic GUI can be used to monitor solver 
/// timings, to change physical system settings easily, 
/// and so on.

class ChIrrAppInterface
{
public: 
	

	class ChIrrAppEventReceiver : public IEventReceiver
	{
	public:

		ChIrrAppEventReceiver(ChIrrAppInterface* m_app)
		{
			app = m_app;
		}

		bool OnEvent(const SEvent& event)
		{
			// See if the user wants to handle some event, first.
			if (this->app)
				if (this->app->user_receiver)
					if (this->app->user_receiver->OnEvent(event))
						return true;
			
			if (event.EventType == irr::EET_KEY_INPUT_EVENT && !event.KeyInput.PressedDown)
			{
				switch (event.KeyInput.Key)
				{
				case irr::KEY_KEY_I: 
					app->SetShowInfos(!app->GetShowInfos());
					return true;
				case irr::KEY_SPACE: 
					app->pause_step = !app->pause_step;
					return true;
				case irr::KEY_KEY_P: 
					app->pause_step = true;
					app->do_single_step = true;
					return true;
				case irr::KEY_SNAPSHOT:
					chrono::GetLog() << "Saving system vector and matrices to dump_xxyy.dat files.\n";
					app->DumpMatrices();
					return true;
				}
			}

			core::dimension2d<u32> ssize = app->GetVideoDriver()->getScreenSize(); 
		   if(event.EventType == EET_MOUSE_INPUT_EVENT) 
		   { 
			  switch(event.MouseInput.Event) 
			  { 
				 case EMIE_MMOUSE_PRESSED_DOWN: 
					 {
						 core::line3d<f32> mline = app->GetSceneManager()->getSceneCollisionManager()->getRayFromScreenCoordinates(app->GetDevice()->getCursorControl()->getPosition());
						 chrono::ChVector<> mfrom(mline.start.X, mline.start.Y, mline.start.Z) ;
						 chrono::ChVector<> mto  (mline.end.X,   mline.end.Y,   mline.end.Z);
						 chrono::ChCollisionSystem::ChRayhitResult mresult;
						 app->GetSystem()->GetCollisionSystem()->RayHit(mfrom, mto, mresult);
						 if (mresult.hit)
							 if (chrono::collision::ChModelBulletBody* mbomod = dynamic_cast<chrono::collision::ChModelBulletBody*>(mresult.hitModel))
						 {
							 app->selectedmover  = new chrono::ChSharedBodyPtr(mbomod->GetBody());
							 app->selectedpoint = (*(app->selectedmover))->Point_World2Body(&mresult.abs_hitPoint);
							 app->selecteddist = (mfrom - mresult.abs_hitPoint).Length();
							 app->selectedspring = new chrono::ChSharedPtr<chrono::ChLinkSpring>(new chrono::ChLinkSpring);
							 app->selectedtruss  = new chrono::ChSharedBodyPtr(new chrono::ChBody);
							 (*(app->selectedtruss))->SetBodyFixed(true);
							 app->GetSystem()->AddBody(*(app->selectedtruss));
							 (*(app->selectedspring))->Initialize(*app->selectedtruss,*app->selectedmover, false, mresult.abs_hitPoint, mresult.abs_hitPoint);
							 app->GetSystem()->AddLink(*(app->selectedspring));
							 
						 }
						break;
					 }
				case EMIE_MMOUSE_LEFT_UP: 
					if (app->selectedtruss)
					 {
						 app->GetSystem()->RemoveBody((*(app->selectedtruss)));
						 app->GetSystem()->RemoveLink((*(app->selectedspring)));
						 delete (app->selectedtruss);
						 delete (app->selectedspring);
						 app->selectedtruss=0;
						 app->selectedspring=0;
					 }
					break;
				case EMIE_MOUSE_MOVED:
					{
						if (app->selectedtruss)
						{
							core::line3d<f32> mline = app->GetSceneManager()->getSceneCollisionManager()->getRayFromScreenCoordinates(app->GetDevice()->getCursorControl()->getPosition());
							chrono::ChVector<> mfrom(mline.start.X, mline.start.Y, mline.start.Z) ;
							chrono::ChVector<> mto  (mline.end.X,   mline.end.Y,   mline.end.Z);
							chrono::ChVector<> mdir = mto-mfrom; mdir.Normalize();
							chrono::ChVector<> springP1 = mfrom + mdir * app->selecteddist;
							chrono::ChVector<> springP2 = (*(app->selectedmover))->Point_Body2World(&app->selectedpoint);
							(*(app->selectedspring))->SetEndPoint1Abs(springP1);
							(*(app->selectedspring))->SetEndPoint2Abs(springP2);
							(*(app->selectedspring))->Set_SpringK( 25* (*(app->selectedmover))->GetMass() );
							(*(app->selectedspring))->Set_SpringR( 3* (*(app->selectedmover))->GetMass() );
						}
						break;
					}
				}
			}
		   

			// check if user moved the sliders with mouse..
			if (event.EventType == EET_GUI_EVENT)
			{
				s32 id = event.GUIEvent.Caller->getID();

				switch(event.GUIEvent.EventType)
				{
				case gui::EGET_SCROLL_BAR_CHANGED:
						if (id == 9904) 
						{
							this->app->GetSystem()->SetIterLCPmaxItersSpeed(( (gui::IGUIScrollBar*)event.GUIEvent.Caller)->getPos() );
							break;
						}
						if (id == 9905) 
						{
							this->app->GetSystem()->SetIterLCPmaxItersStab(( (gui::IGUIScrollBar*)event.GUIEvent.Caller)->getPos() );
							break;
						}
						if (id == 9909) 
						{
							this->app->GetSystem()->SetIterLCPomega( (1.0/50.0)* ((gui::IGUIScrollBar*)event.GUIEvent.Caller)->getPos() );
							break;
						}
						if (id == 9910) 
						{
							this->app->GetSystem()->SetIterLCPsharpnessLambda( (1.0/50.0)* ((gui::IGUIScrollBar*)event.GUIEvent.Caller)->getPos() );
							break;
						}
						if (id == 9911) 
						{
							this->app->GetSystem()->SetMaxPenetrationRecoverySpeed( (3.0/50.0)* ((gui::IGUIScrollBar*)event.GUIEvent.Caller)->getPos() );
							break;
						}
						if (id == 9912) 
						{
							this->app->GetSystem()->SetMinBounceSpeed( (1.0/200.0)* ((gui::IGUIScrollBar*)event.GUIEvent.Caller)->getPos() );
							break;
						}

				case gui::EGET_COMBO_BOX_CHANGED:
						if (id == 9907)
						{		
							int sel = ( (gui::IGUIComboBox*)event.GUIEvent.Caller)->getSelected();
							switch(sel)
							{
								case 0: this->app->GetSystem()->SetLcpSolverType(chrono::ChSystem::LCP_ITERATIVE_SOR); break;
								case 1: this->app->GetSystem()->SetLcpSolverType(chrono::ChSystem::LCP_ITERATIVE_SYMMSOR); break;
								case 2: this->app->GetSystem()->SetLcpSolverType(chrono::ChSystem::LCP_ITERATIVE_JACOBI); break;
								case 3: this->app->GetSystem()->SetLcpSolverType(chrono::ChSystem::LCP_ITERATIVE_SOR_MULTITHREAD); break;
								case 4: this->app->GetSystem()->SetLcpSolverType(chrono::ChSystem::LCP_ITERATIVE_BARZILAIBORWEIN); break;
								case 5: this->app->GetSystem()->SetLcpSolverType(chrono::ChSystem::LCP_ITERATIVE_PCG); break;
								case 6: this->app->GetSystem()->SetLcpSolverType(chrono::ChSystem::LCP_ITERATIVE_PMINRES); break;
							}
							break;
						}
						if (id == 9908)
						{		
							int sel = ( (gui::IGUIComboBox*)event.GUIEvent.Caller)->getSelected();
							switch(sel)
							{
								case 0: this->app->GetSystem()->SetIntegrationType(chrono::ChSystem::INT_ANITESCU); break;
								case 1: this->app->GetSystem()->SetIntegrationType(chrono::ChSystem::INT_TASORA); break;
							}
							break;
						}
				case gui::EGET_CHECKBOX_CHANGED:
						if (id == 9906)
						{	
							this->app->GetSystem()->SetIterLCPwarmStarting( ((gui::IGUICheckBox*)event.GUIEvent.Caller)->isChecked() );
							break;
						}
						if (id == 9913)
						{	
							this->app->GetSystem()->SetUseSleeping( ((gui::IGUICheckBox*)event.GUIEvent.Caller)->isChecked() );
							break;
						}
						if (id == 9916)
						{	
							this->app->SetTryRealtime( ((gui::IGUICheckBox*)event.GUIEvent.Caller)->isChecked() );
							break;
						}
						if (id == 9917)
						{	
							this->app->pause_step =  ((gui::IGUICheckBox*)event.GUIEvent.Caller)->isChecked();
							break;
						}
				case gui::EGET_EDITBOX_ENTER:
						if (id == 9918)
						{	
							double dt = 0.01;
							dt = atof( core::stringc( ((gui::IGUIEditBox*)event.GUIEvent.Caller)->getText() ).c_str() );
							this->app->SetTimestep(dt); 
							break;
						}

				}
			} 

			return false;
		}

	private:
		ChIrrAppInterface* app;
	};




			/// Create the IRRLICHT context (device, etc.)
	ChIrrAppInterface(chrono::ChSystem* mysystem, 
						const wchar_t* title, 
						core::dimension2d<u32> dimens = core::dimension2d<u32>(640,480),
						bool do_fullscreen = false,
						bool do_shadows = false,
						video::E_DRIVER_TYPE mydriver = video::EDT_DIRECT3D9)
		{
			this->step_manage = true;
			this->try_realtime = false;
			this->pause_step = false;
			this->timestep = 0.01;
			this->do_single_step = false;

			this->user_receiver=0;

			this->selectedtruss= 0;
			this->selectedspring = 0;
			this->selectedmover = 0;

			device = createDevice(mydriver, dimens, 
										32, do_fullscreen,		
										do_shadows);			
			if (device == 0) 
			{      
				chrono::GetLog() << "Cannot use default video driver - fall back to OpenGL \n"; 
				device = createDevice(video::EDT_OPENGL, dimens,32,do_fullscreen,do_shadows);
				if (!device) return;
			}

			device->setWindowCaption(title);

			gui::IGUISkin* skin = GetIGUIEnvironment()->getSkin();
			gui::IGUIFont* font = GetIGUIEnvironment()->getFont("../data/fonts/arial8.xml");
			if (font)
				skin->setFont(font);
			skin->setColor(irr::gui::EGDC_BUTTON_TEXT, irr::video::SColor(255,40,50,50));

			gad_tabbed = GetIGUIEnvironment()->addTabControl(core::rect<s32>(2,70,220,476), 0, true, true);
			gad_tab1 = gad_tabbed->addTab(L"Stats");
			gad_tab2 = gad_tabbed->addTab(L"System");

			// create GUI gadgets
			gad_textFPS = GetIGUIEnvironment()->addStaticText(L"FPS", core::rect<s32>(10,10,200,250), true, true, gad_tab1);

			gad_labelcontacts = GetIGUIEnvironment()->addComboBox(core::rect<s32>(10,260, 200,260+20), gad_tab1, 9901);
				gad_labelcontacts->addItem(L"Contact distances");
				gad_labelcontacts->addItem(L"Contact force modulus");
				gad_labelcontacts->addItem(L"Contact force (normal)");
				gad_labelcontacts->addItem(L"Contact force (tangent)");
				gad_labelcontacts->addItem(L"Contact torque modulus");
				gad_labelcontacts->addItem(L"Contact torque (spinning)");
				gad_labelcontacts->addItem(L"Contact torque (rolling)");
				gad_labelcontacts->addItem(L"Don't print contact values");
			gad_labelcontacts->setSelected(7);

			gad_drawcontacts = GetIGUIEnvironment()->addComboBox(core::rect<s32>(10,280, 200,280+20), gad_tab1, 9901);
				gad_drawcontacts->addItem(L"Contact normals");
				gad_drawcontacts->addItem(L"Contact distances");
				gad_drawcontacts->addItem(L"Contact N forces");
				gad_drawcontacts->addItem(L"Contact forces");
				gad_drawcontacts->addItem(L"Don't draw contacts");
			gad_drawcontacts->setSelected(4);

			gad_plot_aabb = GetIGUIEnvironment()->addCheckBox(false,core::rect<s32>(10,310, 200,310+20),
								gad_tab1, 9914, L"Draw AABB");

			gad_plot_cogs = GetIGUIEnvironment()->addCheckBox(false,core::rect<s32>(110,310, 200,310+20),
								gad_tab1, 9915, L"Draw COGs");

			gad_plot_convergence = GetIGUIEnvironment()->addCheckBox(false,core::rect<s32>(10,340, 200,340+20),
								gad_tab1, 9902, L"Plot convergence");

			// --

			gad_speed_iternumber = GetIGUIEnvironment()->addScrollBar(true,      core::rect<s32>(10, 10, 150,10+20), gad_tab2, 9904);
			gad_speed_iternumber->setMax(120);
			gad_speed_iternumber_info = GetIGUIEnvironment()->addStaticText(L"", core::rect<s32>(155,10, 220,10+20), false, false, gad_tab2);

			gad_pos_iternumber   = GetIGUIEnvironment()->addScrollBar(true,    core::rect<s32>(10, 40, 150,40+20), gad_tab2, 9905);
			gad_pos_iternumber->setMax(120);
			gad_pos_iternumber_info = GetIGUIEnvironment()->addStaticText(L"", core::rect<s32>(155,40, 220,40+20), false, false, gad_tab2);

			gad_warmstart = GetIGUIEnvironment()->addCheckBox(false,core::rect<s32>(10,70, 200,70+20),
								gad_tab2, 9906, L"Warm starting");

			gad_usesleep  = GetIGUIEnvironment()->addCheckBox(false,core::rect<s32>(10,100, 200,100+20),
								gad_tab2, 9913, L"Enable sleeping");

			gad_ccpsolver = GetIGUIEnvironment()->addComboBox(core::rect<s32>(10,130, 200,130+20), gad_tab2, 9907);
				gad_ccpsolver->addItem(L"Projected SOR");
				gad_ccpsolver->addItem(L"Projected SSOR");
				gad_ccpsolver->addItem(L"Projected Jacobi");
				gad_ccpsolver->addItem(L"Multithreaded SOR");
				gad_ccpsolver->addItem(L"Projected BB");
				gad_ccpsolver->addItem(L"Projected PCG");
				gad_ccpsolver->addItem(L"Projected MINRES");
				gad_ccpsolver->addItem(L" ");
			gad_ccpsolver->setSelected(5);

			gad_stepper = GetIGUIEnvironment()->addComboBox(core::rect<s32>(10,160, 200,160+20), gad_tab2, 9908);
				gad_stepper->addItem(L"Anitescu stepper");
				gad_stepper->addItem(L"Tasora stepper");
			gad_stepper->setSelected(0);

			gad_omega   = GetIGUIEnvironment()->addScrollBar(true,    core::rect<s32>(10, 190, 150,190+20), gad_tab2, 9909);
			gad_omega->setMax(100);
			gad_omega_info = GetIGUIEnvironment()->addStaticText(L"", core::rect<s32>(155,190, 220,190+20), false, false, gad_tab2);

			gad_lambda   = GetIGUIEnvironment()->addScrollBar(true,    core::rect<s32>(10, 220, 150,220+20), gad_tab2, 9910);
			gad_lambda->setMax(100);
			gad_lambda_info = GetIGUIEnvironment()->addStaticText(L"", core::rect<s32>(155,220, 220,220+20), false, false, gad_tab2);

			gad_clamping   = GetIGUIEnvironment()->addScrollBar(true,    core::rect<s32>(10, 250, 150,250+20), gad_tab2, 9911);
			gad_clamping->setMax(100);
			gad_clamping_info = GetIGUIEnvironment()->addStaticText(L"", core::rect<s32>(155,250, 220,250+20), false, false, gad_tab2);

			gad_minbounce   = GetIGUIEnvironment()->addScrollBar(true,    core::rect<s32>(10, 280, 150,280+20), gad_tab2, 9912);
			gad_minbounce->setMax(100);
			gad_minbounce_info = GetIGUIEnvironment()->addStaticText(L"", core::rect<s32>(155,280, 220,280+20), false, false, gad_tab2);

			gad_timestep      = GetIGUIEnvironment()->addEditBox(L"",core::rect<s32>(140,320, 200,320+15), true,
								gad_tab2, 9918);
			gad_timestep_info = GetIGUIEnvironment()->addStaticText(L"Time step", core::rect<s32>(10,320, 130,320+15), false, false, gad_tab2);

			gad_try_realtime  = GetIGUIEnvironment()->addCheckBox(false,core::rect<s32>(10,340, 200,340+15),
								gad_tab2, 9916, L"Realtime step");
			gad_pause_step    = GetIGUIEnvironment()->addCheckBox(false,core::rect<s32>(10,355, 200,355+15),
								gad_tab2, 9917, L"Pause physics");

			///

			system = mysystem;
			
			show_infos = false;

			// the event receiver, taking care of user interaction
			ChIrrAppEventReceiver* receiver = new ChIrrAppEventReceiver(this);
			device->setEventReceiver(receiver);
		}



			/// This safely delete every Irrlicht item (including the 
			/// scene nodes, so also the encapsulated Chrono bodies will
			/// be deleted)
	~ChIrrAppInterface()
		{
			device->drop();
			//delete (receiver); 
		}


	IrrlichtDevice*			GetDevice() {return device;}
	video::IVideoDriver*	GetVideoDriver() {return device->getVideoDriver();}
	scene::ISceneManager*	GetSceneManager() {return device->getSceneManager();}
	gui::IGUIEnvironment*	GetIGUIEnvironment() {return device->getGUIEnvironment();}
	chrono::ChSystem*		GetSystem()  {return system;};
 
				/// Show the info panel in the 3D view
	void SetShowInfos(bool val) {show_infos= val;}		
	bool GetShowInfos() {return show_infos;}

				/// Set the time step for time integration. It will be used when you 
				/// call myapplication->DoStep() in the loop, that will advance the simulation by 
				/// one timestep. 
	void SetTimestep(double val) 
		{
			this->timestep = chrono::ChMax(10e-9, val);
			char message[50];
			sprintf(message,"%g", this->timestep );
			this->gad_timestep->setText(core::stringw(message).c_str());
		}
	double GetTimestep() {return this->timestep;}

				/// If you set as true, you can use  myapplication->DoStep() in the simulation loop
				/// for advancing the simulation by one timestep. Set to false if you want to handle the 
				/// time stepping by yourself, for example calling ChSystem::DoStepDynamics() in the loop.
	void SetStepManage(bool val) {this->step_manage = val;}
	bool GetStepManage() {return this->step_manage;}

				/// If you set it as true, the function DoStep() will try to use a timestep that
				/// is the same timestep used to refresh the interface and compute physics (it tries to keep	
				/// a soft-realtime performance). Note: the realtime step is upper-limited by the timestep 
				/// that you set by SetTimestep(), anyway! (this clamping will happen if there's too load on the CPU).
	void SetTryRealtime(bool val) {this->try_realtime = val;}
	bool GetTryRealtime() {return this->try_realtime;}

	void SetPaused(bool val) {this->pause_step = val;}
	bool GetPaused() {return this->pause_step;}

		
				/// Use this function to hook a custom event receiver to the application. See examples.
	void SetUserEventReceiver(irr::IEventReceiver* mreceiver) {this->user_receiver = mreceiver;}


			/// Call this important function inside a cycle like
			///    while(application.GetDevice()->run()) {...}
			/// in order to advance the physics by one timestep. The amount of the timestep
			/// can be set via SetTimestep(). Optionally you can use SetTryRealtime(true) if your simulation
			/// can run in realtime because the CPU is fast enough.
			/// Instead, if you want to use ChSystem::DoStepDynamics() directly in your loop, just
			/// do not use this and SetStepManage(false)
	void DoStep()
		{
			if (!this->step_manage)
				return;
			if (this->pause_step)
				if (this->do_single_step)
					this->do_single_step = false;
				else
					return;

			double dt;
			if (this->try_realtime)
				dt = this->m_realtime_timer.SuggestSimulationStep(this->timestep);
			else 
				dt = this->timestep;
			
			this->system->DoStepDynamics(dt);
		}


			/// Call this important function inside a cycle like
			///    while(application.GetDevice()->run()) {...}
			/// in order to get the redrawing of all 3D shapes and all the GUI elements
	void DrawAll()
		{
			core::stringw str = "World time   =";
					str += (int) (1000*system->GetChTime());
					str +=  " s  \n\nCPU step (total)      ="; 
					str += (int) (1000*system->GetTimerStep());
					str +=  " ms \n  CPU Collision time =";
					str += (int) (1000*system->GetTimerCollisionBroad());
					str +=  " ms \n  CPU LCP time         =";
					str += (int) (1000*system->GetTimerLcp());
					str +=  " ms \n\nLCP vel.iters : "; 
					str += system->GetIterLCPmaxItersSpeed();
					str +=  "\nLCP pos.iters : ";
					str += system->GetIterLCPmaxItersStab();
					str +=  "\n\nN.of active bodies  : "; 
					str += system->GetNbodies();
					str +=  "\nN.of sleeping bodies  : "; 
					str += system->GetNbodiesSleeping();
					str +=  "\nN.of contacts  : ";
					str += system->GetNcontacts();
					str +=  "\nN.of coords    : ";
					str += system->GetNcoords_w();
					str +=  "\nN.of constr.   : ";
					str += system->GetNdoc_w();
					str +=  "\nN.of variables : ";
					str += system->GetNsysvars_w();
					gad_textFPS->setText(str.c_str());

			GetSceneManager()->drawAll();

			int dmode = this->gad_drawcontacts->getSelected();
			ChIrrTools::drawAllContactPoints(*system, GetVideoDriver(), 1, (ChIrrTools::eCh_ContactsDrawMode)dmode);

			int lmode = this->gad_labelcontacts->getSelected();
			ChIrrTools::drawAllContactLabels(*system, GetDevice(), (ChIrrTools::eCh_ContactsLabelMode)lmode);

			if (this->gad_plot_aabb->isChecked())
				ChIrrTools::drawAllBoundingBoxes(*system, GetVideoDriver());

			if (this->gad_plot_cogs->isChecked())
				ChIrrTools::drawAllCOGs(*system, GetVideoDriver(), 0.02);

			if (this->gad_plot_convergence->isChecked())
				ChIrrTools::drawHUDviolation(GetVideoDriver(), GetDevice(), *system,  240,370,300,100, 100.0, 500.0);


			gad_tabbed->setVisible(show_infos);

			if (this->gad_speed_iternumber_info->isVisible())
			{
				this->gad_warmstart->setChecked(GetSystem()->GetIterLCPwarmStarting());
				this->gad_usesleep->setChecked(GetSystem()->GetUseSleeping());

				char message[50]; 

				gad_speed_iternumber->setPos(this->GetSystem()->GetIterLCPmaxItersSpeed());
				sprintf(message,"%i vel.iters", this->GetSystem()->GetIterLCPmaxItersSpeed() );
				gad_speed_iternumber_info->setText(core::stringw(message).c_str());

				gad_pos_iternumber->setPos(this->GetSystem()->GetIterLCPmaxItersStab());
				sprintf(message,"%i pos.iters", this->GetSystem()->GetIterLCPmaxItersStab() );
				gad_pos_iternumber_info->setText(core::stringw(message).c_str());

				gad_omega->setPos((s32)(50.0*(this->GetSystem()->GetIterLCPomega())));
				sprintf(message,"%g omega", this->GetSystem()->GetIterLCPomega() );
				gad_omega_info->setText(core::stringw(message).c_str());

				gad_lambda->setPos((s32)(50.0*(this->GetSystem()->GetIterLCPsharpnessLambda())));
				sprintf(message,"%g lambda", this->GetSystem()->GetIterLCPsharpnessLambda() );
				gad_lambda_info->setText(core::stringw(message).c_str());

				gad_clamping->setPos((s32)((50.0/3.0)*(this->GetSystem()->GetMaxPenetrationRecoverySpeed())));
				sprintf(message,"%g stab.clamp", this->GetSystem()->GetMaxPenetrationRecoverySpeed() );
				gad_clamping_info->setText(core::stringw(message).c_str());

				gad_minbounce->setPos((s32)(200.0*(this->GetSystem()->GetMinBounceSpeed())));
				sprintf(message,"%g min.bounce v", this->GetSystem()->GetMinBounceSpeed() );
				gad_minbounce_info->setText(core::stringw(message).c_str());

				switch(this->GetSystem()->GetLcpSolverType())
				{	
					case chrono::ChSystem::LCP_ITERATIVE_SOR: 		gad_ccpsolver->setSelected(0); break;
					case chrono::ChSystem::LCP_ITERATIVE_SYMMSOR: 	gad_ccpsolver->setSelected(1); break;
					case chrono::ChSystem::LCP_ITERATIVE_JACOBI: 	gad_ccpsolver->setSelected(2); break;
					case chrono::ChSystem::LCP_ITERATIVE_SOR_MULTITHREAD: 	gad_ccpsolver->setSelected(3); break;
					case chrono::ChSystem::LCP_ITERATIVE_BARZILAIBORWEIN: 	gad_ccpsolver->setSelected(4); break;
					case chrono::ChSystem::LCP_ITERATIVE_PCG: 		gad_ccpsolver->setSelected(5); break;
					case chrono::ChSystem::LCP_ITERATIVE_PMINRES: 	gad_ccpsolver->setSelected(6); break;
					default: gad_ccpsolver->setSelected(5); break;
				}
				switch(this->GetSystem()->GetIntegrationType())
				{	
					case chrono::ChSystem::INT_ANITESCU: 	gad_stepper->setSelected(0); break;
					case chrono::ChSystem::INT_TASORA: 		gad_stepper->setSelected(1); break;
				}

				this->gad_try_realtime->setChecked(this->GetTryRealtime());
				this->gad_pause_step->setChecked(this->pause_step);

				if (!this->GetStepManage())
				{
					sprintf(message,"%g", this->GetSystem()->GetStep() );
					this->gad_timestep->setText(core::stringw(message).c_str());
				}

				// disable timestep-related gadgets if dt not handled by application object
				this->gad_try_realtime->setEnabled(this->GetStepManage());
				this->gad_pause_step->setEnabled(this->GetStepManage());
				this->gad_timestep->setEnabled(this->GetStepManage());
			}
 
			//if(show_infos)
			GetIGUIEnvironment()->drawAll();
		}

			/// Dump the last used system matrices and vectors in the current directory,
			/// as 'dump_xxxx.dat' files that can be loaded with Matlab for debugging,
			/// benchmarking etc. It saves M mass matrix, Cq jacobians, E compliance
			/// as Matlab sparse matrix format, and known vectors fb, bi as column Matlab matrices.
	void DumpMatrices()
		{
			// For safety
			this->GetSystem()->Setup();
			this->GetSystem()->Update();

			// Save the current speeds, maybe these are needed.
			try
			{
				chrono::ChMatrixDynamic<double> mvold;
				this->GetSystem()->GetLcpSystemDescriptor()->FromVariablesToVector(mvold);
				chrono::ChStreamOutAsciiFile file_vold("dump_v_old.dat");
				mvold.StreamOUTdenseMatlabFormat(file_vold);
			}
			catch(chrono::ChException myexc)
			{	chrono::GetLog() << myexc.what(); }

			// This DoStep() is necessary because we want to get the matrices as they
			// are set-up for the time stepping LCP/CCP problem.
			// (If we avoid this, the previous 'mvold' vector won't be in-sync.)
			this->DoStep(); 

			// Now save the matrices - as they were setup by the previous time stepping scheme.
			try
			{
				chrono::ChSparseMatrix mdM;
				chrono::ChSparseMatrix mdCq;
				chrono::ChSparseMatrix mdE;
				chrono::ChMatrixDynamic<double> mdf;
				chrono::ChMatrixDynamic<double> mdb;
				chrono::ChMatrixDynamic<double> mdfric;
				this->GetSystem()->GetLcpSystemDescriptor()->ConvertToMatrixForm(&mdCq, &mdM, &mdE, &mdf, &mdb, &mdfric);
				chrono::ChStreamOutAsciiFile file_M("dump_M.dat");
				mdM.StreamOUTsparseMatlabFormat(file_M);
				chrono::ChStreamOutAsciiFile file_Cq("dump_Cq.dat");
				mdCq.StreamOUTsparseMatlabFormat(file_Cq);
				chrono::ChStreamOutAsciiFile file_E("dump_E.dat");
				mdE.StreamOUTsparseMatlabFormat(file_E);
				chrono::ChStreamOutAsciiFile file_f("dump_f.dat");
				mdf.StreamOUTdenseMatlabFormat(file_f);
				chrono::ChStreamOutAsciiFile file_b("dump_b.dat");
				mdb.StreamOUTdenseMatlabFormat(file_b);
				chrono::ChStreamOutAsciiFile file_fric("dump_fric.dat");
				mdfric.StreamOUTdenseMatlabFormat(file_fric);
			} 
			catch(chrono::ChException myexc)
			{	chrono::GetLog() << myexc.what(); }

		}


private:
	IrrlichtDevice* device;
		
	chrono::ChSystem* system;
	
	ChIrrAppEventReceiver* receiver;
	
	irr::IEventReceiver* user_receiver;


	bool show_infos;

	bool step_manage;
	bool pause_step;
	bool try_realtime;
	double timestep;
	bool do_single_step;
	chrono::ChRealtimeStepTimer m_realtime_timer;

	gui::IGUITabControl* gad_tabbed;
	gui::IGUITab*		 gad_tab1;
	gui::IGUITab*		 gad_tab2;

	gui::IGUIStaticText* gad_textFPS;
	gui::IGUIComboBox*   gad_drawcontacts;
	gui::IGUIComboBox*   gad_labelcontacts;
	gui::IGUICheckBox*   gad_plot_aabb;
	gui::IGUICheckBox*   gad_plot_cogs;
	gui::IGUICheckBox*   gad_plot_convergence;

	gui::IGUIScrollBar*  gad_speed_iternumber;
	gui::IGUIStaticText* gad_speed_iternumber_info;
	gui::IGUIScrollBar*  gad_pos_iternumber;
	gui::IGUIStaticText* gad_pos_iternumber_info;
	gui::IGUIScrollBar*  gad_omega;
	gui::IGUIStaticText* gad_omega_info;
	gui::IGUIScrollBar*  gad_lambda;
	gui::IGUIStaticText* gad_lambda_info;
	gui::IGUIScrollBar*  gad_clamping;
	gui::IGUIStaticText* gad_clamping_info;
	gui::IGUIScrollBar*  gad_minbounce;
	gui::IGUIStaticText* gad_minbounce_info;
	gui::IGUIScrollBar*  gad_dt;
	gui::IGUIStaticText* gad_dt_info;
	gui::IGUICheckBox*   gad_warmstart;
	gui::IGUICheckBox*   gad_usesleep;
	gui::IGUIComboBox*   gad_ccpsolver;
	gui::IGUIComboBox*   gad_stepper;
	gui::IGUIEditBox*    gad_timestep;
	gui::IGUIStaticText* gad_timestep_info;
	gui::IGUICheckBox*   gad_try_realtime;
	gui::IGUICheckBox*   gad_pause_step;

	public:
	chrono::ChSharedPtr<chrono::ChLinkSpring>* selectedspring;
	chrono::ChSharedBodyPtr* selectedtruss;
	chrono::ChSharedBodyPtr* selectedmover;
	chrono::ChVector<> selectedpoint;
	double selecteddist;

}; // end of  class 









} // END_OF_NAMESPACE____

#endif // END of ChIrrAppInterface.h

