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

class HMMWVEventReceiver : public IEventReceiver
{
public:
	// keep the tabs public
	gui::IGUITabControl* gad_tabbed;
	gui::IGUITab*		gad_tab_controls;	// HMMWV vehicle control
	gui::IGUITab*		gad_tab_wheelState;	// output pane for wheel state data

	HMMWVEventReceiver(ChIrrApp* app, ChSystem* system, HMMWV_9body* car, HMMWVTerrain* terrain, 
		const int gui_TLC_x = 740, const int gui_TLC_y = 20):
	mapp(app), msys(system), mcar(car), mterrain(terrain)
	{
		// initial checkbox values
		// for saving output data
		this->save_wheelData = false;
		this->save_controls = false;

		app->SetUserEventReceiver(this);

		// draw normal & shear contact forces that are contacting with the wheel
		// this->draw_norm_shear = false;
		// this->draw_TMaligned = false;

		// **** ***
		// create the GUI tabs
		s32 x0 = gui_TLC_x;	s32 y0 = gui_TLC_y;	// box0 top left corner
		s32 y1 = 265;	// box2 top left corner
		s32 boxH = 660;	// total height of tab box
		s32 cout_y1 = 240;	// y-pos of control output panel
		// create the tabs for the rig output: NOTE: GUI widget locations are all relative to the TabControl!
		gad_tabbed = mapp->GetIGUIEnvironment()->addTabControl(core::rect<s32>(x0,y0,x0+265,y0+boxH),
			0, true, true);
		// tab1 box1: wheel controls
		gad_tab_controls = gad_tabbed->addTab(L"Controls");	
		gad_text_carControls = mapp->GetIGUIEnvironment()->addStaticText(L"Wheel Control", 
			core::rect<s32>(10,10,255,y1-15), true, true, gad_tab_controls);
		// tab1 box2: chassis output
		gad_text_chassisDat = mapp->GetIGUIEnvironment()->addStaticText(L"Chassis State", 
			core::rect<s32>(10,y1,255,boxH-30), true, true, gad_tab_controls);
		// tab2: output wheel data
		gad_tab_wheelState = gad_tabbed->addTab(L"Wheel State");

		// **** ***
		// create the GUI scrollbars
		// ID 1001: throttle
		scrollbar_throttle = mapp->GetIGUIEnvironment()->addScrollBar(true,
			core::rect<s32>(10,30, 150, 45), gad_tab_controls, 1001);
		scrollbar_throttle->setMax(100); 
		scrollbar_throttle->setPos(10);
		text_throttle = mapp->GetIGUIEnvironment()->addStaticText(L"Throttle: 0",
			core::rect<s32>(150,30,250,45), false, false, gad_tab_controls);
		scrollbar_throttle->setVisible(true);

		//ID 1002: steering
		scrollbar_steer = mapp->GetIGUIEnvironment()->addScrollBar(true,
			core::rect<s32>(10, 60, 150, 75), gad_tab_controls, 1002);
		scrollbar_steer->setMax(100); 
		scrollbar_steer->setPos(50);
		text_steerPos = mapp->GetIGUIEnvironment()->addStaticText(L"Tierod Disp: 0",
			core::rect<s32>(150,60,250,75),false, false, gad_tab_controls);
/*
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
*/


		// ******* GUI OUTPUT, Chassis state
		// chassis CM pos
		ChVector<> cm = this->mcar->chassis->GetPos();
		char msg_chassisCMPos[100]; sprintf(msg_chassisCMPos,
			"CM pos, x: %3.3g, y: %3.3g, z: %3.3g",cm.x,cm.y,cm.z);
		text_chassisCMPos = mapp->GetIGUIEnvironment()->addStaticText(core::stringw(msg_chassisCMPos).c_str(),
			core::rect<s32>(10,y1+15,280,y1+30),false,false,gad_text_carControls);
		// chassis CM vel
		ChVector<> cmVel = this->mcar->chassis->GetPos_dt();
		char message_chassisCMVel[100]; sprintf(message_chassisCMVel,
			"CM vel, x: %3.3g, y: %3.3g, z: %3.3g",cmVel.x,cmVel.y,cmVel.z);
		text_chassisCMVel = mapp->GetIGUIEnvironment()->addStaticText(core::stringw(message_chassisCMVel).c_str(),
			core::rect<s32>(10,y1+45,280,y1+90),false,false,gad_text_carControls);

		// *** GUI OUTPUT, LF wheel
		// LF wheel CM pos
		ChVector<> LFcm = this->mcar->wheelLF->wheelBody->GetPos();
		char msg_LF_pos[100]; sprintf(msg_LF_pos,
			"LF pos, x: %3.3g, y: %3.3g, z: %3.3g",LFcm.x,LFcm.y,LFcm.z);
		text_wheelLF_CMPos = mapp->GetIGUIEnvironment()->addStaticText(core::stringw(msg_LF_pos).c_str(),
			core::rect<s32>(10,30,280,45),false,false,gad_tab_wheelState);
		// chassis CM vel
		ChVector<> LFvel = this->mcar->wheelLF->wheelBody->GetPos_dt();
		char msg_LF_vel[100]; sprintf(msg_LF_vel,
			"LF vel, x: %3.3g, y: %3.3g, z: %3.3g",LFvel.x,LFvel.y,LFvel.z);
		text_wheelLF_CMVel = mapp->GetIGUIEnvironment()->addStaticText(core::stringw(msg_LF_vel).c_str(),
			core::rect<s32>(10,60,280,75),false,false,gad_tab_wheelState);

		// LF spindle revolute joint, wheel/upright, position violation
		ChVector<> pos_err = this->mcar->suspension_LF->spindle_revolute->GetRelC().pos;
		char msg_pos_err[100]; sprintf(msg_pos_err,
			"spindle RelC, x: %3.3g, y: %3.3g, z: %3.3g",pos_err.x,pos_err.y,pos_err.z);
		text_spindleLF_RelC = mapp->GetIGUIEnvironment()->addStaticText(core::stringw(msg_pos_err).c_str(),
			core::rect<s32>(10,90,280,105),false,false,gad_tab_wheelState);

	}

	bool OnEvent(const SEvent& event)
	{
		//user input to GUI
		if (event.EventType == EET_GUI_EVENT)
		{
			s32 id = event.GUIEvent.Caller->getID();
			gui::IGUIEnvironment* env = mapp->GetIGUIEnvironment();

			switch(event.GUIEvent.EventType)
			{
			// check if user moved the sliders with mouse
			case gui::EGET_SCROLL_BAR_CHANGED:
				if( id == 1001) // throttle slider 
				{
					s32 currPos = ((gui::IGUIScrollBar*)event.GUIEvent.Caller)->getPos();
					// DEBUG: allow the motor torque to be negative, to slow down
					double throttle = double(currPos-10)/100.0;
					mcar->driver->setThrottle(throttle);
					// show throttle
					char msg[100]; sprintf(msg,"Throttle: %+3.3g",throttle*100.);
					text_throttle->setText(core::stringw(msg).c_str() );
				}
				if (id == 1002) // Steering slider
				{
					s32 pos = ((gui::IGUIScrollBar*)event.GUIEvent.Caller)->getPos();
					double steering = (double)(pos - 50) / 50.;
					this->mcar->driver->setSteer(steering);
					// show steer position
					char msg[100]; sprintf(msg,"Tierod Disp: %+3.3g",steering);
					text_steerPos->setText(core::stringw(msg).c_str() );
				}
			/*
				if (id == 1003) // id of 'spring stiffness' slider..
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
				if (id == 1004) // id of 'damping' slider..
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
				if (id == 1005) // id of 'spring rest length' slider..
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
			*/
			break;
			// case gui::EGET_CHECKBOX_CHANGED:
				/*
				if( id == 2001)
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
		// user hit a key, while not holding it down
		if(event.EventType == EET_KEY_INPUT_EVENT && !event.KeyInput.PressedDown) 
		{
			if(event.KeyInput.Key == KEY_KEY_A) {
				double curr_steer = mcar->driver->getSteer();
				curr_steer -= 0.1;
				this->mcar->driver->setSteer(curr_steer);
				// show steer position
				char msg[100]; sprintf(msg,"Tierod Disp: %+3.3g",curr_steer);
				text_steerPos->setText(core::stringw(msg).c_str() );
				return true;
			}
			if(event.KeyInput.Key == KEY_KEY_D) {
				double curr_steer = mcar->driver->getSteer();
				curr_steer += 0.1;
				this->mcar->driver->setSteer(curr_steer);
				// show steer position
				char msg[100]; sprintf(msg,"Tierod Disp: %+3.3g",curr_steer);
				text_steerPos->setText(core::stringw(msg).c_str() );
				return true;
			}
			if(event.KeyInput.Key == KEY_KEY_W)	{
				// increase throttle by 10
				double curr_throttle =  mcar->driver->getThrottle();
				// DEBUG: allow the motor torque to be negative, to slow down
				curr_throttle += 0.1;
				mcar->driver->setThrottle(curr_throttle);
				// show throttle
				char msg[100]; sprintf(msg,"Throttle: %+3.3g",curr_throttle*100.);
				text_throttle->setText(core::stringw(msg).c_str() );
				return true;
			}
			if(event.KeyInput.Key == KEY_KEY_S) {
				// decrease throttle by 10
				double curr_throttle = mcar->driver->getThrottle();
				// DEBUG: allow the motor torque to be negative, to slow down
				curr_throttle -= 0.1;
				mcar->driver->setThrottle(curr_throttle);
				// show throttle
				char msg[100]; sprintf(msg,"Throttle: %+3.3g",curr_throttle*100.);
				text_throttle->setText(core::stringw(msg).c_str() );
				return true;
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

	// output chassis data to gad_tab_controls, bottom part of tab
	void drawCarDataOutput()
	{
		// wheel CM pos
		ChVector<> cm = this->mcar->chassis->GetPos();
		char messageCM[100]; sprintf(messageCM,"CM pos, x: %3.3g, y: %3.3g, z: %3.3g",cm.x,cm.y,cm.z);
		text_chassisCMPos->setText(core::stringw(messageCM).c_str());
		// wheel CM vel
		ChVector<> cmVel = this->mcar->chassis->GetPos_dt();
		char messageV[100]; sprintf(messageV,"CM vel, x: %3.3g, y: %3.3g, z: %3.3g",cmVel.x,cmVel.y,cmVel.z);
		text_chassisCMVel->setText( core::stringw(messageV).c_str() );
		
	}

	// output any relevant wheel data to gad_tab_wheelState
	void drawWheelData_LF()
	{
		// LF wheel CM pos
		ChVector<> LFcm = this->mcar->wheelLF->wheelBody->GetPos();
		char msg_LF_pos[100]; sprintf(msg_LF_pos,
			"LF pos, x: %3.3g, y: %3.3g, z: %3.3g",LFcm.x,LFcm.y,LFcm.z);
		text_wheelLF_CMPos->setText(core::stringw(msg_LF_pos).c_str() );

		// chassis CM vel
		ChVector<> LFvel = this->mcar->wheelLF->wheelBody->GetPos_dt();
		char msg_LF_vel[100]; sprintf(msg_LF_vel,
			"LF vel, x: %3.3g, y: %3.3g, z: %3.3g",LFvel.x,LFvel.y,LFvel.z);
		text_wheelLF_CMVel->setText(core::stringw(msg_LF_vel).c_str() );

		// LF spindle revolute joint, wheel/upright, position violation
		ChVector<> pos_err = this->mcar->suspension_LF->spindle_revolute->GetRelC().pos;
		char msg_pos_err[100]; sprintf(msg_pos_err,
			"spindle RelC, x: %3.3g, y: %3.3g, z: %3.3g",pos_err.x,pos_err.y,pos_err.z);
		text_spindleLF_RelC->setText(core::stringw(msg_pos_err).c_str() );

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

	// scroll bars
	gui::IGUIScrollBar*  scrollbar_throttle;
	gui::IGUIScrollBar*  scrollbar_steer;

	// text for scroll bar values
	gui::IGUIStaticText* text_throttle;
	gui::IGUIStaticText* text_steerPos;



	// output screen data, chassis state
	gui::IGUIStaticText* text_chassisCMPos;	// output chassis info
	gui::IGUIStaticText* text_chassisCMVel;

	// output screen data,  wheel state
	gui::IGUIStaticText* text_wheelLF_CMPos;	// output chassis info
	gui::IGUIStaticText* text_wheelLF_CMVel;
	gui::IGUIStaticText* text_spindleLF_RelC;	// upright/wheel joint error in position

	// output tabs
	gui::IGUIStaticText* gad_text_carControls;		// applied controls
	gui::IGUIStaticText* gad_text_wheelState;	// panel for all wheel state output data
	gui::IGUIStaticText* gad_text_chassisDat;

	// for rendering terrain
    scene::ISceneNode* Terrain;
    scene::ISceneNode* Skybox;
    scene::ISceneNode* Skydome;
	scene::ICameraSceneNode *mCamera;	// follow camera
    bool showBox;
    bool showDebug;
	// save output data
	bool save_wheelData;
	bool save_controls;

	// unused GUI sliders
//	gui::IGUIStaticText* text_FspringK;
//	gui::IGUIScrollBar*  scrollbar_FspringK;
//	gui::IGUIStaticText* text_FdamperR;
//	gui::IGUIScrollBar*  scrollbar_FdamperR;
//	gui::IGUIStaticText* text_FspringL;
//	gui::IGUIScrollBar*  scrollbar_FspringL;

	// info boxes to output Tire Force/Moment parameters
//	IGUIStaticText* text_infoRF;
//	IGUIStaticText* text_infoLF;
//	IGUIStaticText* text_infoLB;
//	IGUIStaticText* text_infoRB;

//	double K_0_front;	// initial shock stiffness, front suspension
//	double K_0_rear;	// initial shock stiffness, rear
//	double R_0_front;	// initial damping, front
//	double R_0_rear;	// initial damping, rear	
//	double L_0_F;		// initial spring pre-length, front springs
//	double L_0_R;	// rear springs

};


}	// namespace chrono{


#endif	// ifndef HMMWVEVENTRECEIVER_H