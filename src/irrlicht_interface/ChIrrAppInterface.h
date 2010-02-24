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
								case 4: this->app->GetSystem()->SetLcpSolverType(chrono::ChSystem::LCP_ITERATIVE_GPU); break;
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
				device = createDevice(video::EDT_OPENGL, core::dimension2d<u32>(800, 600));
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
			gad_textFPS = GetIGUIEnvironment()->addStaticText(L"FPS", core::rect<s32>(10,10,200,280), true, true, gad_tab1);

			gad_labelcontacts = GetIGUIEnvironment()->addComboBox(core::rect<s32>(10,290, 200,290+20), gad_tab1, 9901);
				gad_labelcontacts->addItem(L"Contact distances");
				gad_labelcontacts->addItem(L"Contact force modulus");
				gad_labelcontacts->addItem(L"Contact force (normal)");
				gad_labelcontacts->addItem(L"Contact force (tangent)");
				gad_labelcontacts->addItem(L"Contact torque modulus");
				gad_labelcontacts->addItem(L"Contact torque (spinning)");
				gad_labelcontacts->addItem(L"Contact torque (rolling)");
				gad_labelcontacts->addItem(L"Don't print contact values");
			gad_labelcontacts->setSelected(7);

			gad_drawcontacts = GetIGUIEnvironment()->addComboBox(core::rect<s32>(10,310, 200,310+20), gad_tab1, 9901);
				gad_drawcontacts->addItem(L"Contact normals");
				gad_drawcontacts->addItem(L"Contact distances");
				gad_drawcontacts->addItem(L"Contact N forces");
				gad_drawcontacts->addItem(L"Contact forces");
				gad_drawcontacts->addItem(L"Don't draw contacts");
			gad_drawcontacts->setSelected(4);

			gad_plot_convergence = GetIGUIEnvironment()->addCheckBox(false,core::rect<s32>(10,340, 200,340+20),
								gad_tab1, 9902, L"Plot convergence");

			// --

			gad_speed_iternumber = GetIGUIEnvironment()->addScrollBar(true,      core::rect<s32>(10, 10, 150,10+20), gad_tab2, 9904);
			gad_speed_iternumber->setMax(80);
			gad_speed_iternumber_info = GetIGUIEnvironment()->addStaticText(L"", core::rect<s32>(155,10, 220,10+20), false, false, gad_tab2);

			gad_pos_iternumber   = GetIGUIEnvironment()->addScrollBar(true,    core::rect<s32>(10, 40, 150,40+20), gad_tab2, 9905);
			gad_pos_iternumber->setMax(80);
			gad_pos_iternumber_info = GetIGUIEnvironment()->addStaticText(L"", core::rect<s32>(155,40, 220,40+20), false, false, gad_tab2);

			gad_omega   = GetIGUIEnvironment()->addScrollBar(true,    core::rect<s32>(10, 160, 150,160+20), gad_tab2, 9909);
			gad_omega->setMax(100);
			gad_omega_info = GetIGUIEnvironment()->addStaticText(L"", core::rect<s32>(155,160, 220,160+20), false, false, gad_tab2);

			gad_lambda   = GetIGUIEnvironment()->addScrollBar(true,    core::rect<s32>(10, 190, 150,190+20), gad_tab2, 9910);
			gad_lambda->setMax(100);
			gad_lambda_info = GetIGUIEnvironment()->addStaticText(L"", core::rect<s32>(155,190, 220,190+20), false, false, gad_tab2);

			gad_clamping   = GetIGUIEnvironment()->addScrollBar(true,    core::rect<s32>(10, 220, 150,220+20), gad_tab2, 9911);
			gad_clamping->setMax(100);
			gad_clamping_info = GetIGUIEnvironment()->addStaticText(L"", core::rect<s32>(155,220, 220,220+20), false, false, gad_tab2);


			gad_warmstart = GetIGUIEnvironment()->addCheckBox(false,core::rect<s32>(10,70, 200,70+20),
								gad_tab2, 9906, L"Warm starting");

			gad_ccpsolver = GetIGUIEnvironment()->addComboBox(core::rect<s32>(10,100, 200,100+20), gad_tab2, 9907);
				gad_ccpsolver->addItem(L"Projected SOR");
				gad_ccpsolver->addItem(L"Projected SSOR");
				gad_ccpsolver->addItem(L"Projected Jacobi");
				gad_ccpsolver->addItem(L"Multithreaded SOR");
				gad_ccpsolver->addItem(L"GPU");
				gad_ccpsolver->addItem(L" ");
			gad_ccpsolver->setSelected(5);

			gad_stepper = GetIGUIEnvironment()->addComboBox(core::rect<s32>(10,130, 200,130+20), gad_tab2, 9908);
				gad_stepper->addItem(L"Anitescu stepper");
				gad_stepper->addItem(L"Tasora stepper");
			gad_stepper->setSelected(0);


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
 

	bool GetShowInfos() {return show_infos;}
	void SetShowInfos(bool val) {show_infos= val;}

	void SetUserEventReceiver(irr::IEventReceiver* mreceiver) {this->user_receiver = mreceiver;}


			/// Call this important function inside a cycle like
			///    while(ChirrAppInterface->GetDevice()->run()) {...}
			/// in order to get the redrawing of all 3D shapes and all the GUI elements
	void DrawAll()
		{
			core::stringw str = "World time   =";
					str += (int) (1000*system->GetChTime());
					str +=  "   \n\nCPU step (total)      ="; 
					str += (int) (1000*system->GetTimerStep());
					str +=  "ms \n  CPU Collision time =";
					str += (int) (1000*system->GetTimerCollisionBroad());
					str +=  "ms \n  CPU LCP time         =";
					str += (int) (1000*system->GetTimerLcp());
					str +=  "ms \n\nLCP vel.iters : "; 
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

			if (this->gad_plot_convergence->isChecked())
				ChIrrTools::drawHUDviolation(GetVideoDriver(), GetDevice(), *system,  240,370,300,100, 100.0, 500.0);

			gad_tabbed->setVisible(show_infos);

			if (this->gad_speed_iternumber_info->isVisible())
			{
				this->gad_warmstart->setChecked(GetSystem()->GetIterLCPwarmStarting());

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

				gad_clamping->setPos((s32)(50.0*(this->GetSystem()->GetMaxPenetrationRecoverySpeed())));
				sprintf(message,"%g stab.clamp", this->GetSystem()->GetMaxPenetrationRecoverySpeed() );
				gad_clamping_info->setText(core::stringw(message).c_str());

				switch(this->GetSystem()->GetLcpSolverType())
				{	
					case chrono::ChSystem::LCP_ITERATIVE_SOR: 		gad_ccpsolver->setSelected(0); break;
					case chrono::ChSystem::LCP_ITERATIVE_SYMMSOR: 	gad_ccpsolver->setSelected(1); break;
					case chrono::ChSystem::LCP_ITERATIVE_JACOBI: 	gad_ccpsolver->setSelected(2); break;
					case chrono::ChSystem::LCP_ITERATIVE_SOR_MULTITHREAD: 	gad_ccpsolver->setSelected(3); break;
					case chrono::ChSystem::LCP_ITERATIVE_GPU: 		gad_ccpsolver->setSelected(4); break;
					default: gad_ccpsolver->setSelected(5); break;
				}
				switch(this->GetSystem()->GetIntegrationType())
				{	
					case chrono::ChSystem::INT_ANITESCU: 	gad_stepper->setSelected(0); break;
					case chrono::ChSystem::INT_TASORA: 		gad_stepper->setSelected(1); break;
				}

			}
 
			//if(show_infos)
			GetIGUIEnvironment()->drawAll();
		}


private:
	IrrlichtDevice* device;
		
	chrono::ChSystem* system;
	
	ChIrrAppEventReceiver* receiver;
	
	irr::IEventReceiver* user_receiver;


	bool show_infos;

	gui::IGUITabControl* gad_tabbed;
	gui::IGUITab*		 gad_tab1;
	gui::IGUITab*		 gad_tab2;

	gui::IGUIStaticText* gad_textFPS;
	gui::IGUIComboBox*   gad_drawcontacts;
	gui::IGUIComboBox*   gad_labelcontacts;
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
	gui::IGUIScrollBar*  gad_dt;
	gui::IGUIStaticText* gad_dt_info;
	gui::IGUICheckBox*   gad_warmstart;
	gui::IGUIComboBox*   gad_ccpsolver;
	gui::IGUIComboBox*   gad_stepper;

	public:
	chrono::ChSharedPtr<chrono::ChLinkSpring>* selectedspring;
	chrono::ChSharedBodyPtr* selectedtruss;
	chrono::ChSharedBodyPtr* selectedmover;
	chrono::ChVector<> selectedpoint;
	double selecteddist;

}; // end of  class 









} // END_OF_NAMESPACE____

#endif // END of ChIrrAppInterface.h

