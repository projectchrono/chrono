#ifndef HMMWVEVENTRECEIVER_H
#define HMMWVEVENTRECEIVER_H

// Event Reciever for the soil bin simulation,
// only used with visualization via Irrlicht is enabled 
// Allows the user to control the wheel testing mechaism manually
// also, can generate various particles to represent the soil
#include "HMMWV_9body_config.h"
// CE Includes   
#include "physics/ChApidll.h" 
#include "physics/ChSystem.h"
#include "core/ChRealtimeStep.h"

// HMMWV vehicle and terrain module includes
#include "HMMWVTerrain.h"
#include "HMMWV_9body.h"

namespace irr {

 // TODO: move this stuff
double KfreeLenOff_F = 0.2;
double KfreeLenOff_R = 0.2;

class HMMWVEventReceiver : public IEventReceiver
{
public:
	// keep the tabs public
	gui::IGUITabControl* gad_tabbed;
	gui::IGUITab*		gad_tab_controls;	// HMMWV vehicle control
	gui::IGUITab*		gad_tab_carData;	// output pane for wheel state data

	HMMWVEventReceiver(ChIrrApp* app, ChSystem* system, HMMWV_9body* car, HMMWVTerrain* terrain, 
		const int gui_TLC_x = 740, const int gui_TLC_y = 20):
	mapp(app), msys(system), mcar(car), mterrain(terrain)
	{
		// initial checkbox values
		// for saving output data
		this->save_wheelData = false;
		this->save_controls = false;

		// draw normal & shear contact forces that are contacting with the wheel
		// this->draw_norm_shear = false;
		// this->draw_TMaligned = false;

		// **** ***
		// create the GUI items here
		s32 x0 = gui_TLC_x;	s32 y0 = gui_TLC_y;	// box0 top left corner
		s32 y1 = 265;	// box2 top left corner
		s32 cout_y1 = 240;	// y-pos of control output panel
		// create the tabs for the rig output: NOTE: GUI widget locations are all relative to the TabControl!
		gad_tabbed = mapp->GetIGUIEnvironment()->addTabControl(core::rect<s32>(x0,y0,x0+255,y0+660), 0, true, true);
		// tab1 box1: wheel controls
		gad_tab_controls = gad_tabbed->addTab(L"Controls");	// static text will be printed w/ each checkbox or slider
		gad_text_carControls = mapp->GetIGUIEnvironment()->addStaticText(L"Wheel Control", 
			core::rect<s32>(10,10,245,y1-15), true, true, gad_tab_controls);
		
		gad_tab_carData = gad_tabbed->addTab(L"Wheel State");

		// ..add a GUI slider to control gas throttle via mouse
		scrollbar_throttle = mapp->GetIGUIEnvironment()->addScrollBar(
			true, core::rect<s32>(x0, y0, x0+140, y0+15), 0, 100);
		//	true, rect<x32>(10, 85, 150, 100), 0, 100);
		scrollbar_throttle->setMax(100); 
		scrollbar_throttle->setPos(10);
		text_throttle = mapp->GetIGUIEnvironment()->addStaticText(
			L"Throttle", core::rect<s32>(x0+140,y0,x0+240,y0+15), false);

		// ..add a GUI slider to control steering via mouse
		scrollbar_steer = mapp->GetIGUIEnvironment()->addScrollBar(
			true, core::rect<s32>(x0, y0+20, x0+140, y0+35), 0, 101);
		scrollbar_steer->setMax(100); 
		scrollbar_steer->setPos(50);
		text_steerPos = mapp->GetIGUIEnvironment()->addStaticText(
			L"Steer Pos: ",core::rect<s32>(x0+140,y0+20,x0+240,y0+55),false);

		// ..add a GUI text and GUI slider to control the stiffness
		scrollbar_FspringK = mapp->GetIGUIEnvironment()->addScrollBar(
			true, core::rect<s32>(x0, y0+40, x0+140, y0+55), 0, 102);
		scrollbar_FspringK->setMax(100); 
		scrollbar_FspringK->setPos(50);
		text_FspringK = mapp->GetIGUIEnvironment()->addStaticText(
			L"Spring K [lb/in]:", core::rect<s32>(x0+140,y0+40,x0+240,y0+55), false);
		// initial stiffness specified when the subsystem is created. Front differs from rear
		this->K_0_front = mcar->suspension_LF->shock->Get_SpringK();
		this->K_0_rear = mcar->suspension_LB->shock->Get_SpringK();

		// ..add a GUI text and GUI slider to control the damping
		scrollbar_FdamperR = mapp->GetIGUIEnvironment()->addScrollBar(
			true, core::rect<s32>(x0, y0+60, x0+140, y0+75), 0, 103);
		scrollbar_FdamperR->setMax(100); 
		// double Rpos = 50 + 50.0*(acar->link_springRF->Get_SpringR()-5.0)/ 5.0;
		scrollbar_FdamperR->setPos(50);
		text_FdamperR = mapp->GetIGUIEnvironment()->addStaticText(
			L"Damper R [lb s/in]:", core::rect<s32>(x0+140,y0+60,x0+240,y0+75), false);
		this->R_0_front = mcar->suspension_LF->shock->Get_SpringR();
		this->R_0_rear = mcar->suspension_LB->shock->Get_SpringR();

		// ..add a GUI text and GUI slider to control the original undeformed spring length
		scrollbar_FspringL = mapp->GetIGUIEnvironment()->addScrollBar(
			true, core::rect<s32>(x0, y0+80, x0+140, y0+95), 0, 104);
		scrollbar_FspringL->setMax(100); 
		// set scroll bar position, start at 50
		scrollbar_FspringL->setPos(50);
		// get the corresopnding springPos accoring to scroll bar value
		double springPos = mcar->suspension_LB->shock->Get_SpringRestLenght();	// will modify this
		text_FspringL = mapp->GetIGUIEnvironment()->addStaticText(
			L"Spring L [in]:", core::rect<s32>(x0+140,y0+80,x0+240,y0+95), false);
		// keep track of the initial spring length
		this->L_0_F = springPos + KfreeLenOff_F;// + 6.0;
		this->L_0_R = springPos + KfreeLenOff_R;
		// usually like an initial prelength longer than initial value, set it to this.
		this->mcar->suspension_RF->shock->Set_SpringRestLenght(this->L_0_F);
		this->mcar->suspension_LF->shock->Set_SpringRestLenght(this->L_0_F);
		this->mcar->suspension_RB->shock->Set_SpringRestLenght(this->L_0_R);
		this->mcar->suspension_LB->shock->Set_SpringRestLenght(this->L_0_R);

		// applied torque (or rotational velocity) slider	(id = 1103)
		scrollbar_torqueVel = mapp->GetIGUIEnvironment()->addScrollBar(true, core::rect<s32>(20, 60, 150, 75),
			gad_tab_controls, 1103);
		scrollbar_torqueVel->setMax(100);
		scrollbar_torqueVel->setPos(50);
		if( this->mcar->useTorque() )
			text_torqueVel = mapp->GetIGUIEnvironment()->addStaticText(
				L"Torque [N/m]: 0 ", core::rect<s32>(160, 60, 300,75), false,false,gad_tab_controls);
		else
			text_torqueVel = mapp->GetIGUIEnvironment()->addStaticText(
				L"omega [rad/s]: 0 ", core::rect<s32>(160, 60, 300,75), false,false,gad_tab_controls);


		// ******* GUI OUTPUT, Chassis state
		// Data I care about:
		// chassis cm pos
		ChVector<> cm = this->mcar->chassis->GetPos();
		char msg_chassisCMPos[100]; sprintf(msg_chassisCMPos,"CM pos, x: %4.4g, y: %4.4g, z: %4.4g",cm.x,cm.y,cm.z);
		text_chassisCMPos = mapp->GetIGUIEnvironment()->addStaticText(core::stringw(msg_chassisCMPos).c_str(),
			core::rect<s32>(10,30,280,45),false,false,gad_tab_carData);
		// wheel CM vel
		ChVector<> cmVel = this->mcar->chassis->GetPos_dt();
		char message_chassisCMVel[100]; sprintf(message_chassisCMVel,"CM vel, x: %4.4g, y: %4.4g, z: %4.4g",cmVel.x,cmVel.y,cmVel.z);
		text_chassisCMVel = mapp->GetIGUIEnvironment()->addStaticText(core::stringw(message_chassisCMVel).c_str(),
			core::rect<s32>(10,60,280,75),false,false,gad_tab_carData);

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
			case gui::EGET_SCROLL_BAR_CHANGED:
				if( id == 1103) // throttle slider 
				{
					s32 currPos = ((gui::IGUIScrollBar*)event.GUIEvent.Caller)->getPos();
					double throttle = double(currPos)/100.0;
					mcar->driver->setThrottle(throttle);
					// show throttle
					char msg[150]; sprintf(msg,"Throttle: %+5.4g",throttle*100.);
					text_throttle->setText(core::stringw(msg).c_str() );
				}
				if (id == 101) // Steering slider
				{
					s32 pos = ((gui::IGUIScrollBar*)event.GUIEvent.Caller)->getPos();
					this->mcar->driver->setSteer(((double)(pos-50))/50.);
					// show steer position
					char msg[150]; sprintf(msg,"steer pos: %+5.4g",pos);
					text_steerPos->setText(core::stringw(msg).c_str() );
				}
				if (id == 102) // id of 'spring stiffness' slider..
				{
					s32 pos = ((gui::IGUIScrollBar*)event.GUIEvent.Caller)->getPos();
					// newstiff(50) = K_0
					double newstiff_front = K_0_front + K_0_front*( ((double)(pos-50))/50. );
					double newstiff_rear = K_0_rear + K_0_rear*( ((double)(pos-50))/50. );
					// set the stiffness of all 4 springs
					this->mcar->suspension_RF->shock->Set_SpringK(newstiff_front);
					this->mcar->suspension_LF->shock->Set_SpringK(newstiff_front);
					this->mcar->suspension_RB->shock->Set_SpringK(newstiff_rear);
					this->mcar->suspension_LB->shock->Set_SpringK(newstiff_rear);

					// show front stiffness as formatted text in interface screen
					char message[150]; sprintf(message,"front Spring K [lb/in]: %+5.4g",newstiff_front);
					text_FspringK->setText(core::stringw(message).c_str());
				}
				if (id == 103) // id of 'damping' slider..
				{
					s32 pos = ((gui::IGUIScrollBar*)event.GUIEvent.Caller)->getPos();
					double newdamping_front = R_0_front + R_0_front*( ((double)(pos-50))/50. );
					double newdamping_rear = R_0_rear + R_0_rear*( ((double)(pos-50))/50. );
					// set the damping of all 4 dampers
					this->mcar->suspension_RF->shock->Set_SpringR(newdamping_front);
					this->mcar->suspension_LF->shock->Set_SpringR(newdamping_front);
					this->mcar->suspension_RB->shock->Set_SpringR(newdamping_rear);
					this->mcar->suspension_LB->shock->Set_SpringR(newdamping_rear);

					// show front damping as formatted text in interface screen
					char message[150]; sprintf(message,"front Damper R : %+5.4g",newdamping_front);
					text_FdamperR->setText(core::stringw(message).c_str());
				}
				if (id == 104) // id of 'spring rest length' slider..
				{
					s32 pos = ((gui::IGUIScrollBar*)event.GUIEvent.Caller)->getPos();
					double newlength_F =  L_0_F + 12.*( (pos-50.)/50. );
					double newlength_R =  L_0_R + 12.*( (pos-50.)/50. );
					// set the rest length of all 4 springs
					this->mcar->suspension_RF->shock->Set_SpringRestLenght(newlength_F);
					this->mcar->suspension_LF->shock->Set_SpringRestLenght(newlength_F);
					this->mcar->suspension_RB->shock->Set_SpringRestLenght(newlength_R);
					this->mcar->suspension_LB->shock->Set_SpringRestLenght(newlength_R);
					// show stiffness as formatted text in interface screen
					char message[150]; sprintf(message,"Spring L, F/R [in]: %+5.4g / %5.4g ",newlength_F,newlength_R);
					text_FspringL->setText(core::stringw(message).c_str());
				}
			break;
			// case gui::EGET_CHECKBOX_CHANGED:
				/*
				if( id == 2113)
				{
					applyTorque = checkbox_applyTorque->isChecked();
					GetLog() << checkbox_applyTorque->isChecked() << "\n";
					// apply a torque to the wheel?
					this->mtester->isTorqueApplied = applyTorque;
					return true;
				}
				*/
			}
			
		} 

		return false;
	}

	void drawSprings()
	{
		std::list<chrono::ChLink*>::iterator iterlink =  mapp->GetSystem()->Get_linklist()->begin();
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
		// floor
		ChCoordsys<> wall1Csys = this->mterrain->floor->GetCoord();
		wall1Csys.rot = chrono::Q_from_AngAxis(-CH_C_PI_2, VECT_Y);
		wall1Csys.pos.x += .1;
		ChIrrTools::drawGrid(this->mapp->GetVideoDriver(),0.05,0.05,40,20, wall1Csys,
			video::SColor(255,80,130,130),true);
	}

	void drawLinks()
	{
		std::list<chrono::ChLink*>::iterator iterlink =  msys->Get_linklist()->begin();
		while(iterlink !=  msys->Get_linklist()->end())	{
			if (ChLinkDistance* mylinkdis = ChDynamicCast(ChLinkDistance,(*iterlink))) {
				ChIrrTools::drawSegment(mapp->GetVideoDriver(), 
					mylinkdis->GetEndPoint1Abs(), 
					mylinkdis->GetEndPoint2Abs(),
					video::SColor(255,   0,20,0), true	);
			}
			iterlink++;
		}
	}

	// output any relevant test rig data here
	void drawCarDataOutput()
	{
		// wheel CM pos
		ChVector<> cm = this->mcar->chassis->GetPos();
		char messageCM[100]; sprintf(messageCM,"CM pos, x: %4.4g, y: %4.4g, z: %4.4g",cm.x,cm.y,cm.z);
		text_chassisCMPos->setText(core::stringw(messageCM).c_str());
		// wheel CM vel
		ChVector<> cmVel = this->mcar->chassis->GetPos_dt();
		char messageV[100]; sprintf(messageV,"CM vel, x: %4.4g, y: %4.4g, z: %4.4g",cmVel.x,cmVel.y,cmVel.z);
		text_chassisCMVel->setText( core::stringw(messageV).c_str() );
		
	}

	void create_camera(const ChVector<>& camera_pos, const ChVector<>& camera_targ ) {
		// camera is behind and above chassis, its target
		core::vector3df pos = core::vector3df(camera_pos.x, camera_pos.y, camera_pos.z );
		core::vector3df targ = core::vector3df(camera_targ.x, camera_targ.y, camera_targ.z);
		mapp->GetSceneManager()->addCameraSceneNode( mapp->GetSceneManager()->getRootSceneNode(),
			pos, targ);
		mapp->GetSceneManager()->getActiveCamera()->setUpVector( core::vector3df(0,0,1) );


	}

	// move the camera every rendered frame, so it follows the car at a given global offset
	// target is the chassis CM
	void update_cameraPos(const ChVector<>& cam_offset) {
		scene::ICameraSceneNode *mCamera = mapp->GetSceneManager()->getActiveCamera();
		ChVector<> cam_global_pos =  mcar->getCM_pos_chassis() + cam_offset;
		core::vector3df cam_global_irr(cam_global_pos.x, cam_global_pos.y, cam_global_pos.z);
		// set the camera position
		mCamera->setPosition(cam_global_irr);
		chrono::ChVector<> mcar_CMpos = mcar->getCM_pos_chassis();
		// set the camera target: chassis CM
		mCamera->setTarget( core::vector3df( mcar_CMpos.x, mcar_CMpos.y, mcar_CMpos.z) );
	}

private:
	ChIrrAppInterface* mapp;
	// bodies/joints we can control
	HMMWV_9body* mcar;
	HMMWVTerrain* mterrain;

	// the chrono system
	ChSystem* msys;
	// for check boxes

	// save output data
	bool save_wheelData;
	bool save_controls;


	// menu items, checkboxes ids are: 2xxx
	gui::IGUIScrollBar*  scrollbar_steer;
	gui::IGUIStaticText* text_steerPos;	// JCM
	gui::IGUIStaticText* text_FspringK;
	gui::IGUIScrollBar*  scrollbar_FspringK;
	gui::IGUIStaticText* text_FdamperR;
	gui::IGUIScrollBar*  scrollbar_FdamperR;
	gui::IGUIStaticText* text_FspringL;
	gui::IGUIScrollBar*  scrollbar_FspringL;
	gui::IGUIStaticText* text_throttle;
	gui::IGUIScrollBar*  scrollbar_throttle;
	// JCM text output of tire forces/moments
	// info boxes to output Tire Force/Moment parameters
//	IGUIStaticText* text_infoRF;
//	IGUIStaticText* text_infoLF;
//	IGUIStaticText* text_infoLB;
//	IGUIStaticText* text_infoRB;
	// for body data
	gui::IGUIStaticText* text_infoChassis;

	double K_0_front;	// initial shock stiffness, front suspension
	double K_0_rear;	// initial shock stiffness, rear
	double R_0_front;	// initial damping, front
	double R_0_rear;	// initial damping, rear	
	double L_0_F;		// initial spring pre-length, front springs
	double L_0_R;	// rear springs
	// for rendering terrain
    scene::ISceneNode* Terrain;
    scene::ISceneNode* Skybox;
    scene::ISceneNode* Skydome;
	scene::ICameraSceneNode *mCamera;	// follow camera
    bool showBox;
    bool showDebug;

	// scroll bars for wheel control
	gui::IGUIScrollBar* scrollbar_torqueVel;		// torque or linear velocity applied to testMech, id = 1109
	gui::IGUIStaticText* text_chassisCMPos;	// output chassis info
	gui::IGUIStaticText* text_chassisCMVel;
	gui::IGUIStaticText* text_chassisYPR;
	gui::IGUIStaticText* text_torqueVel;	// output applied torque/motion to rear wheels

	// output tabs, and their text boxes
	gui::IGUIStaticText* gad_text_carControls;		// applied controls
	gui::IGUIStaticText* gad_text_carControl_output;	// control output
	gui::IGUIStaticText* gad_text_wheelState;	// panel for all wheel state output data

};


}	// namespace chrono{


#endif	// ifndef HMMWVEVENTRECEIVER_H