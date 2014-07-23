#ifndef HMMWVEVENTRECEIVER_H
#define HMMWVEVENTRECEIVER_H

// Event Reciever for the soil bin simulation,
// only used with visualization via Irrlicht is enabled 
// Allows the user to control the wheel testing mechaism manually
// also, can generate various particles to represent the soil
#include "soilbin_config.h"   

#include "physics/ChApidll.h" 
#include "physics/ChSystem.h"
#include "core/ChRealtimeStep.h"
 #include "irrlicht_interface/CHirrApp.h"
 #include <irrlicht.h>
// includes for a soil bin
#include "HMMWVTerrain.h"
#include "HMMWV.h"

#ifdef USE_TERRAIN
 #include "TerraMech.h"
 #include "TM_TractionElement.h"

#endif
using namespace chrono;
using namespace irr;
         
using namespace core;
using namespace scene; 
using namespace video;
using namespace io; 
using namespace gui; 

 // move this stuff
double KfreeLenOff_F = 0.2;
double KfreeLenOff_R = 0.2;

class HMMWVEventReceiver : public IEventReceiver
{
public:
	// keep the tabs public
	gui::IGUITabControl* gad_tabbed;
	gui::IGUITab*		gad_tab_controls;	// HMMWV vehicle control
	gui::IGUITab*		gad_tab_carData;	// output pane for wheel state data
	gui::IGUITab*		gad_tab_particles;	// output for particle state data statistics
	gui::IGUITab*		gad_tab_TM;		// output related to terramechanics

	HMMWVEventReceiver(ChIrrApp* app, ChSystem* system, HMMWV* car, HMMWVTerrain* terrain, 
		TerraMech* terramech,
		const int gui_TLC_x = 740, const int gui_TLC_y = 20): mapp(app), msys(system), mcar(car), mterrain(terrain), mTerramech(terramech)
	{
		// initial checkbox values
		// for saving output data
		this->save_wheelData = false;
		this->save_soilData = false;
		this->save_controls = false;

		// draw normal & shear contact forces that are contacting with the wheel
		this->draw_wheel_contacts = false;
		this->draw_all_tireForces = false;	// dont draw all forces at the onset
		this->draw_norm_shear = false;
		this->draw_TMaligned = false;

		// **** ***
		// create the GUI items here
		irr::s32 x0 = gui_TLC_x;	irr::s32 y0 = gui_TLC_y;	// box0 top left corner
		irr::s32 y1 = 265;	// box2 top left corner
		irr::s32 cout_y1 = 240;	// y-pos of control output panel
		// create the tabs for the rig output: NOTE: GUI widget locations are all relative to the TabControl!
		gad_tabbed = mapp->GetIGUIEnvironment()->addTabControl(core::rect<s32>(x0,y0,x0+255,y0+660), 0, true, true);
		// tab1 box1: wheel controls
		gad_tab_controls = gad_tabbed->addTab(L"Controls");	// static text will be printed w/ each checkbox or slider
		gad_text_carControls = mapp->GetIGUIEnvironment()->addStaticText(L"Wheel Control", 
			core::rect<s32>(10,10,245,y1-15), true, true, gad_tab_controls);
		// TM output tab
		gad_tab_TM = gad_tabbed->addTab(L"TerraMech");
		gad_text_soilState = mapp->GetIGUIEnvironment()->addStaticText(L"SS", core::rect<s32>(10,10,240,250), true, true, gad_tab_TM);

		gad_tab_carData = gad_tabbed->addTab(L"Wheel State");

		// ..add a GUI slider to control gas throttle via mouse
		scrollbar_throttle = mapp->GetIGUIEnvironment()->addScrollBar(
			true, rect<s32>(x0, y0, x0+140, y0+15), 0, 100);
		//	true, rect<x32>(10, 85, 150, 100), 0, 100);
		scrollbar_throttle->setMax(100); 
		scrollbar_throttle->setPos(10);
		text_throttle = mapp->GetIGUIEnvironment()->addStaticText(
			L"Throttle", rect<s32>(x0+140,y0,x0+240,y0+15), false);

		// ..add a GUI slider to control steering via mouse
		scrollbar_steer = mapp->GetIGUIEnvironment()->addScrollBar(
			true, rect<s32>(x0, y0+20, x0+140, y0+35), 0, 101);
		scrollbar_steer->setMax(100); 
		scrollbar_steer->setPos(50);
		text_steerPos = mapp->GetIGUIEnvironment()->addStaticText(
			L"Steer Pos: ",rect<s32>(x0+140,y0+20,x0+240,y0+55),false);
		this->rel1_steer = this->mcar->link_distRSTEER->GetEndPoint1Rel();
		this->rel2_steer = this->mcar->link_distLSTEER->GetEndPoint1Rel();

		// ..add a GUI text and GUI slider to control the stiffness
		scrollbar_FspringK = mapp->GetIGUIEnvironment()->addScrollBar(
			true, rect<s32>(x0, y0+40, x0+140, y0+55), 0, 102);
		scrollbar_FspringK->setMax(100); 
		scrollbar_FspringK->setPos(50);
		text_FspringK = mapp->GetIGUIEnvironment()->addStaticText(
			L"Spring K [lb/in]:", rect<s32>(x0+140,y0+40,x0+240,y0+55), false);
		this->K_0 = mcar->link_springRF->Get_SpringK();

		// ..add a GUI text and GUI slider to control the damping
		scrollbar_FdamperR = mapp->GetIGUIEnvironment()->addScrollBar(
			true, rect<s32>(x0, y0+60, x0+140, y0+75), 0, 103);
		scrollbar_FdamperR->setMax(100); 
		// double Rpos = 50 + 50.0*(acar->link_springRF->Get_SpringR()-5.0)/ 5.0;
		scrollbar_FdamperR->setPos(50);
		text_FdamperR = mapp->GetIGUIEnvironment()->addStaticText(
			L"Damper R [lb s/in]:", rect<s32>(x0+140,y0+60,x0+240,y0+75), false);
		this->R_0 = mcar->link_springRF->Get_SpringR();

		// ..add a GUI text and GUI slider to control the original undeformed spring length
		scrollbar_FspringL = mapp->GetIGUIEnvironment()->addScrollBar(
			true, rect<s32>(x0, y0+80, x0+140, y0+95), 0, 104);
		scrollbar_FspringL->setMax(100); 
		// set scroll bar position, start at 50
		scrollbar_FspringL->setPos(50);
		// get the corresopnding springPos accoring to scroll bar value
		double springPos = mcar->link_springRF->Get_SpringRestLenght();	// will modify this
		text_FspringL = mapp->GetIGUIEnvironment()->addStaticText(
			L"Spring L [in]:", rect<s32>(x0+140,y0+80,x0+240,y0+95), false);
		// keep track of the initial spring length
		this->L_0_F = springPos + KfreeLenOff_F;// + 6.0;
		this->L_0_R = springPos + KfreeLenOff_R;
		// usually like an initial prelength longer than initial value, set it to this.
		this->mcar->link_springRF->Set_SpringRestLenght(this->L_0_F);
		this->mcar->link_springLF->Set_SpringRestLenght(this->L_0_F);
		this->mcar->link_springRB->Set_SpringRestLenght(this->L_0_R);
		this->mcar->link_springLB->Set_SpringRestLenght(this->L_0_R);

		/*
		// set up the text output for forces/moments on tire CM
		// *** RF ***
		ChVector<> tmpF, tmpM;
		tmpF = mcar->get_wheelF( 0 );
		tmpM = mcar->get_wheelM( 0 );
		char msg1[255]; sprintf(msg1,"RF [lb],[lb-in] \nF_x: %+5.4g\nF_y: %+5.4g\nF_z: %+5.4g\nM_x: %+5.4g\nM_y:%+5.4g\nM_z: %+5.4g",
			tmpF.x, tmpF.y, tmpF.z, tmpF.x, tmpM.y, tmpM.z);
		text_infoRF = mapp->GetIGUIEnvironment()->addStaticText( core::stringw(msg1).c_str(),
			rect<s32>(400,20,490,140),false);
		//	rect<s32>(400,20,490,140),false);
		
		// text_infoRF->setText(core::stringw(msg1).c_str());
		// *** LF
		tmpF = mcar->get_wheelF( 1 );
		tmpM = mcar->get_wheelM( 1 );
		char msg2[255]; sprintf(msg2,"LF [lb],[lb-in] \nF_x: %+5.4g\nF_y: %+5.4g\nF_z: %+5.4g\nM_x: %+5.4g\nM_y: %+5.4g\nM_z: %+5.4g",
			tmpF.x, tmpF.y, tmpF.z, tmpF.x, tmpM.y, tmpM.z);
		text_infoLF = mapp->GetIGUIEnvironment()->addStaticText( core::stringw(msg2).c_str(),
			rect<s32>(500,20,590,140), false);
		//	rect<s32>(500,20,590,140), false);
		// text_infoLF->setText(core::stringw(msg2).c_str());
		// *** RB
		tmpF = mcar->get_wheelF( 2 );
		tmpM = mcar->get_wheelM( 2 );
		char msg3[255]; sprintf(msg3,"LB [lb],[lb-in]\nF_x: %+5.4g\nF_y: %+5.4g\nF_z: %+5.4g\nM_x: %+5.4g\nM_y: %+5.4g\nM_z: %+5.4g",
			tmpF.x, tmpF.y, tmpF.z, tmpF.x, tmpM.y, tmpM.z);
		text_infoRB = mapp->GetIGUIEnvironment()->addStaticText( core::stringw(msg3).c_str(),
			rect<s32>(600,20,690,140),false);
		//	rect<s32>(600,20,690,140),false);
		// *** LB
		tmpF = mcar->get_wheelF( 3 );
		tmpM = mcar->get_wheelM( 3 );
		char msg4[150]; sprintf(msg4,"RB [lb],[lb-in]\nF_x: %+5.4g\nF_y: %+5.4g\nF_z: %+5.4g\nM_x: %+5.4g\nM_y: %+5.4g\nM_z: %+5.4g",
			tmpF.x, tmpF.y, tmpF.z, tmpF.x, tmpM.y, tmpM.z);
		text_infoLB = mapp->GetIGUIEnvironment()->addStaticText( core::stringw(msg4).c_str(),
			rect<s32>(700,20,790,140),false);
		//	rect<s32>(700,20,790,140),false);
		// *** Chassis
		tmpF = mcar->getCM_pos_chassis();
		tmpM = mcar->getCM_vel_chassis();
		char msg5[150]; sprintf(msg5,"Chassis CM Pos./Vel.\n x: %+5.4g\n y: %+5.4g\nz: %+5.4g\nV_x: %+5.4g\nV_y: %+5.4g\nV_z: %+5.4g",
			tmpF.x, tmpF.y, tmpF.z, tmpF.x, tmpM.y, tmpM.z);
		text_infoChassis = mapp->GetIGUIEnvironment()->addStaticText( core::stringw(msg5).c_str(),
			rect<s32>(800,20,890,140),false);
*/


		// torque slider	(id = 1103)
		scrollbar_torqueVel = mapp->GetIGUIEnvironment()->addScrollBar(true, rect<s32>(20, 60, 150, 75), gad_tab_controls, 1103);
		scrollbar_torqueVel->setMax(100);
		scrollbar_torqueVel->setPos(50);
		if( this->mcar->useTorque() )
			text_torqueVel = mapp->GetIGUIEnvironment()->addStaticText(
				L"Torque [N/m]: 0 ", rect<s32>(160, 60, 300,75), false,false,gad_tab_controls);
		else
			text_torqueVel = mapp->GetIGUIEnvironment()->addStaticText(
				L"omega [rad/s]: 0 ", rect<s32>(160, 60, 300,75), false,false,gad_tab_controls);


		// ******* GUI OUTPUT, Chassis state
		// Data I care about:
		// chassis cm pos
		ChVector<> cm = this->mcar->chassis->GetPos();
		char msg_chassisCMPos[100]; sprintf(msg_chassisCMPos,"CM pos, x: %4.4g, y: %4.4g, z: %4.4g",cm.x,cm.y,cm.z);
		text_chassisCMPos = mapp->GetIGUIEnvironment()->addStaticText(core::stringw(msg_chassisCMPos).c_str(),
			rect<s32>(10,30,280,45),false,false,gad_tab_carData);
		// wheel CM vel
		ChVector<> cmVel = this->mcar->chassis->GetPos_dt();
		char message_chassisCMVel[100]; sprintf(message_chassisCMVel,"CM vel, x: %4.4g, y: %4.4g, z: %4.4g",cmVel.x,cmVel.y,cmVel.z);
		text_chassisCMVel = mapp->GetIGUIEnvironment()->addStaticText(core::stringw(message_chassisCMVel).c_str(),
			rect<s32>(10,60,280,75),false,false,gad_tab_carData);

		// ******* GUI OUTPUT, Terramechanics
		
		// id = 3001, draw the normal and shear pressures, separately
		checkbox_draw_normShear = mapp->GetIGUIEnvironment()->addCheckBox(draw_norm_shear,
			rect<s32>(20,30, 35, 45),gad_tab_TM,3001);
		text_draw_normShear = mapp->GetIGUIEnvironment()->addStaticText(L"draw norm/shear pressure ?",
			rect<s32>(50,30, 280, 45),false,false,gad_tab_TM);
		// id = 3002, draw ALL tire node as forces
		checkbox_draw_all_tireForces = mapp->GetIGUIEnvironment()->addCheckBox(draw_all_tireForces,
			rect<s32>(20,60,35, 75),gad_tab_TM, 3002);
		text_draw_all_tireForces = mapp->GetIGUIEnvironment()->addStaticText(L"draw ALL tire forces?",
			rect<s32>(50,60,280, 75), false,false,gad_tab_TM);

		// id = 3003, draw the aligned forces on the terrain surface
		checkbox_draw_TMaligned = mapp->GetIGUIEnvironment()->addCheckBox(draw_TMaligned,
			rect<s32>(20,90,35, 105),gad_tab_TM, 3002);
		text_draw_TMaligned = mapp->GetIGUIEnvironment()->addStaticText(L"draw aligned TM forces??",
			rect<s32>(50,90,280, 105), false,false,gad_tab_TM);


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
				if( id == 1103) // throttle or Motion slider, to drive wheel
				{
					s32 currPos = ((IGUIScrollBar*)event.GUIEvent.Caller)->getPos();
					// either we're driving with a torque
					if( this->mcar->useTorque() ) {
						double throttle = double(currPos)/100.0;
						this->mcar->setThrottle(throttle);
					} else {
						double omegaNew = ((currPos - 20.0)/100.0)*this->mcar->get_max_wheel_omega();
						char message[50]; sprintf(message,"omega [rad/s]: %4.3g",omegaNew);
						this->mcar->set_wheelOmega(omegaNew);
					}
				}
				/*
				if (id == 100) // id of 'throttle' slider..
				{
					s32 pos = ((IGUIScrollBar*)event.GUIEvent.Caller)->getPos();
					double newthrottle =  ((double)(pos-10))/100.0 ;
					this->mcar->throttle = newthrottle;
					// show throttle
					char message[150]; sprintf(message,"Throttle: %+5.4g",newthrottle*100.);
					text_throttle->setText(core::stringw(message).c_str() );

				}
				*/
				if (id == 101) // id of 'steer' slider..
				{
					s32 pos = ((IGUIScrollBar*)event.GUIEvent.Caller)->getPos();
					// steer(50) = 0 steer linkage displacement
					double newsteer = 0.125*( ((double)(pos-50))/50. );
					this->mcar->set_UI_steer(pos);	// hold onto the pos, for output
					// set the steering, moving horizontally the endpoints of the steer rod endpoint on truss.
					this->mcar->link_distRSTEER->SetEndPoint1Rel(ChVector<>(rel1_steer.x, rel1_steer.y+newsteer, rel1_steer.z) );
					this->mcar->link_distLSTEER->SetEndPoint1Rel(ChVector<>(rel2_steer.x, rel2_steer.y+newsteer, rel2_steer.z) );
					char msg[150]; sprintf(msg,"steer pos: %+5.4g",pos);
					text_steerPos->setText(core::stringw(msg).c_str() );
				}
				if (id == 102) // id of 'spring stiffness' slider..
				{
					s32 pos = ((IGUIScrollBar*)event.GUIEvent.Caller)->getPos();
					// newstiff(50) = K_0
					double newstiff = K_0 + K_0*( ((double)(pos-50))/50. );
					// set the stiffness of all 4 springs
					this->mcar->link_springRF->Set_SpringK(newstiff);
					this->mcar->link_springLF->Set_SpringK(newstiff);
					this->mcar->link_springRB->Set_SpringK(newstiff);
					this->mcar->link_springLB->Set_SpringK(newstiff);

					// show stiffness as formatted text in interface screen
					char message[150]; sprintf(message,"Spring K [lb/in]: %+5.4g",newstiff);
					text_FspringK->setText(core::stringw(message).c_str());
				}
				if (id == 103) // id of 'damping' slider..
				{
					s32 pos = ((IGUIScrollBar*)event.GUIEvent.Caller)->getPos();
					double newdamping = R_0 + R_0*( ((double)(pos-50))/50. );
					// set the damping of all 4 dampers
					this->mcar->link_springRF->Set_SpringR(newdamping);
					this->mcar->link_springLF->Set_SpringR(newdamping);
					this->mcar->link_springRB->Set_SpringR(newdamping);
					this->mcar->link_springLB->Set_SpringR(newdamping);

					// show stiffness as formatted text in interface screen
					char message[150]; sprintf(message,"Damper R : %+5.4g",newdamping);
					text_FdamperR->setText(core::stringw(message).c_str());
				}
				if (id == 104) // id of 'spring rest length' slider..
				{
					s32 pos = ((IGUIScrollBar*)event.GUIEvent.Caller)->getPos();
					double newlength_F =  L_0_F + 12.*( (pos-50.)/50. );
					double newlength_R =  L_0_R + 12.*( (pos-50.)/50. );
					// set the rest length of all 4 springs
					this->mcar->link_springRF->Set_SpringRestLenght(newlength_F);
					this->mcar->link_springLF->Set_SpringRestLenght(newlength_F);
					this->mcar->link_springRB->Set_SpringRestLenght(newlength_R);
					this->mcar->link_springLB->Set_SpringRestLenght(newlength_R);
					// show stiffness as formatted text in interface screen
					char message[150]; sprintf(message,"Spring L, F/R [in]: %+5.4g / %5.4g ",newlength_F,newlength_R);
					text_FspringL->setText(core::stringw(message).c_str());
				}
			break;
			case gui::EGET_CHECKBOX_CHANGED:
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
				/*
				if( id == 2114)
				{
					pVisible = checkbox_particlesVisible->isChecked();
					GetLog() << checkbox_particlesVisible->isChecked() << "\n";
					// turn off the particle visibility
					// this->mgenerator->toggleVisibility(pVisible);
				}
				if( id == 2115 )
				{
					this->wheelVisible = checkbox_wheelVisible->isChecked();
					// turn bucket visibility on/off
					this->mwheel->toggleVisibility(wheelVisible);
					this->mwheel->toggleVisibility(wheelVisible);
				}
				*/
				// draw tire debug node force to screen?>
				if( id == 3001)
				{
					this->draw_norm_shear = checkbox_draw_normShear->isChecked();
					return true;
				}
				// draw ALL tire/terrain contact points
				if( id == 3002)
				{
					this->draw_all_tireForces = checkbox_draw_all_tireForces->isChecked();
					return true;
				}
				// draw the aligned forces on the terrain surface
				if( id == 3003 ) {
					this->draw_TMaligned = checkbox_draw_TMaligned->isChecked();
					return true;
				}
			break;
			
			}
			
		} 

		return false;
	}

	// run this to get wheel spindle rxn force, moments
	void OnStep()	{
			
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

		// wall 1
		ChCoordsys<> wall1Csys = this->mterrain->wall1->GetCoord();
		wall1Csys.rot = chrono::Q_from_AngAxis(-CH_C_PI_2, VECT_Y);
		wall1Csys.pos.x += .05;
		ChIrrTools::drawGrid(this->mapp->GetVideoDriver(),0.05,0.05,40,20, wall1Csys,
			video::SColor(255,80,130,130),true);
/*
		// wall 2
		ChCoordsys<> wall2Csys = this->mtester->wall2->GetBody()->GetCoord();
		wall2Csys.pos.x -= .05;
		wall2Csys.rot = chrono::Q_from_AngAxis(CH_C_PI/2.0, VECT_Y);
		ChIrrTools::drawGrid(this->app->GetVideoDriver(),0.1,0.05,24,20, wall2Csys,
			video::SColor(255,80,130,130),true);

		// wall 3
		ChCoordsys<> wall3Csys = this->mtester->wall3->GetBody()->GetCoord();
		wall3Csys.pos.z += .05;
		ChIrrTools::drawGrid(this->mapp->GetVideoDriver(),0.1,0.05,10,20, wall3Csys, 
			video::SColor(255,80,130,130),true);

		// wall 4
		ChCoordsys<> wall4Csys = this->mtester->wall4->GetBody()->GetCoord();
		wall4Csys.pos.z -= .05;
		ChIrrTools::drawGrid(this->mapp->GetVideoDriver(),0.1,0.05,10,20, wall4Csys,
			video::SColor(255,80,130,130),true);
*/
	}

	void drawLinks()
	{
		std::list<chrono::ChLink*>::iterator iterlink =  msys->Get_linklist()->begin();
		while(iterlink !=  msys->Get_linklist()->end())	{
			if (ChLinkDistance* mylinkdis = ChDynamicCast(ChLinkDistance,(*iterlink)))
				ChIrrTools::drawSegment(mapp->GetVideoDriver(), 
					mylinkdis->GetEndPoint1Abs(), 
					mylinkdis->GetEndPoint2Abs(),
					video::SColor(255,   0,20,0), true	);
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

	
	// draw forces on the terrain nodes
	static void draw_all_tire_forces(video::IVideoDriver* driver, const TM_TractionElement* m_ptr, double len ) {
		// draw the resulting normal + shear pressure * area (force) on the tire node
		for(uint64_t idx=0; idx<m_ptr->tireForce.size(); idx++) {
			ChVector<> pos = TerraMech::SAE_to_SI_Coords( m_ptr->contactPos[idx]);
			// TODO: negative sign?
			ChVector<> normForce = TerraMech::SAE_to_SI_Force( m_ptr->tireForce[idx] );	
			video::SColor color(200,255,60,60);	// red - normal
			
			// draw normal force
			ChVector<> v2 = pos + normForce*len;
			driver->draw3DLine( core::vector3dfCH(pos), core::vector3dfCH(v2), color );
		}
	}

	// draw forces algined on the terrain nodes
	static void draw_all_aligned_Forces(video::IVideoDriver* driver, const TM_TractionElement* m_ptr, double len ) {
		// iterate through all the pointers to the DataPoints in contact this step
		for(uint64_t idx=0; idx<m_ptr->mTerrainNodeGrid.alignedForcePtr.size(); idx++) {
			// current DataPoint on the terrain surface
			DataPoint *curr_pt =m_ptr->mTerrainNodeGrid.alignedForcePtr[idx];
			ChVector<> pos = TerraMech::SAE_to_SI_Coords( TMVector(curr_pt->x, curr_pt->y, curr_pt->z));
			// TODO: negative sign?
			ChVector<> curr_force = TerraMech::SAE_to_SI_Force( curr_pt->fTire );	
			video::SColor color(240,225,96,70);	// orange-ish
			
			// draw normal force
			ChVector<> v2 = pos + curr_force*len;
			driver->draw3DLine( core::vector3dfCH(pos), core::vector3dfCH(v2), color );
		}
	}

	// draw 1) the normal pressure, 2) the shear stress as vectors in the global ref frame
	static void draw_all_forces_normalShear( video::IVideoDriver* driver, const TM_TractionElement* m_ptr, double len ) {
		// iterature thru all contacts
		for(uint64_t idx=0; idx<m_ptr->tireForce.size(); idx++) {
			ChVector<> collisionPos = TerraMech::SAE_to_SI_Coords( m_ptr->contactPos[idx]);
			// need to convert N/in2 to kN / m2, (1kN/1000N)*(1in/.0254m)^2 = 1.55
			ChVector<> normP = TerraMech::SAE_to_SI_Force( m_ptr->sigma_n[idx] )*1.55;	
			video::SColor color(200,255,60,60);	// red - normal
			
			// draw normal force
			ChVector<> v2 = collisionPos + normP*len;
			driver->draw3DLine( core::vector3dfCH(collisionPos), core::vector3dfCH(v2), color );

			video::SColor colorshear(100,100,60,60);	// ?? something else for shear
			// TODO: negative sign?
			// need to convert N/in2 to kN / m2, (1kN/1000N)*(1in/.0254m)^2 = 1.55
			ChVector<> shearV = TerraMech::SAE_to_SI_Force( m_ptr->tau[idx] )*1.55;
			// draw the shear force
			v2 = collisionPos + shearV*len;
			driver->draw3DLine( core::vector3dfCH(collisionPos), core::vector3dfCH(v2), colorshear);
			}
	}

	// output the Terramechanics force debug info
	void drawTM_forces(uint64_t tire_idx = 0)
	{
		if( this->draw_norm_shear) {
			TM_TractionElement* tire_ptr = (TM_TractionElement*)(getTElement(tire_idx).tmTire_ptr);
			this->draw_all_forces_normalShear(mapp->GetVideoDriver(), tire_ptr, 0.01);
		}

		if( this->draw_all_tireForces ) {
			TM_TractionElement* tire_ptr = (TM_TractionElement*)(getTElement(tire_idx).tmTire_ptr);
			this->draw_all_tire_forces(mapp->GetVideoDriver(), tire_ptr, 0.01 );
		}

		if( this->draw_TMaligned ) {
			TM_TractionElement* tire_ptr = (TM_TractionElement*)(getTElement(tire_idx).tmTire_ptr);
			this->draw_all_aligned_Forces(mapp->GetVideoDriver(), tire_ptr, 0.01);
		}


	}

	// draw the points on the visualization grid
	// can't draw a point directly in irrlict, rather, use the terrain surface point
	// as the center of two 3d lines that are orthogonal, in the x and z axis
	void drawTMvisGrid(const int tire_idx)
	{
		video::SMaterial material;
		material.ZBuffer = false;
		material.Lighting = false;
		mapp->GetVideoDriver()->setMaterial(material);
		// get a pointer to the pointer for access to the vis grid
		const TM_TractionElement* tire_ptr = (TM_TractionElement*)(getTElement(0).tmTire_ptr);

		IVideoDriver* video = mapp->GetVideoDriver();
		ChVector<> nodeCE_curr = ChVector<>();	// be sure to convert from SAE to SI coords
		ChVector<> l1_p1, l1_p2, l2_p1, l2_p2;
		double half_in = 0.02540005 /2.0;	// 1/2"
		core::vector3df irr_p1, irr_p2;
		video::SColor mcolor(55,100,100,200);
		for( int idx = 0; idx < tire_ptr->mTerrainNodeGrid.vis_node_list.size(); idx++) {
			// grab the next node
			nodeCE_curr = TerraMech::SAE_to_SI_Coords(tire_ptr->mTerrainNodeGrid.vis_node_list[idx]);
			// draw a 3D line thru the point, extent 1/2" in each direction
			l1_p1 = l1_p2 = l2_p1 = l2_p2 = nodeCE_curr;
			// line 1: along the x axis
			l1_p1.x -= half_in;
			l1_p2.x += half_in;
			// line 1: along the z-axis
			l2_p1.z -= half_in;
			l2_p2.z += half_in;
			// x marks the spot: line 1
			irr_p1 = vector3dfCH(l1_p1);
			irr_p2 = vector3dfCH(l1_p2);
			video->draw3DLine( irr_p1, irr_p2, mcolor);
			// line 2
			irr_p1 = vector3dfCH(l2_p1);
			irr_p2 = vector3dfCH(l2_p2);
			video->draw3DLine( irr_p1, irr_p2, mcolor);
		}

	}


private:
	ChIrrAppInterface* mapp;
	// bodies/joints we can control
	HMMWV* mcar;
	HMMWVTerrain* mterrain;
	// the chrono system
	ChSystem* msys;
	// to control terramechanics
	TerraMech* mTerramech;
	// for check boxes

	// save output data
	bool save_wheelData;
	bool save_soilData;
	bool save_controls;

	// drawing controls
	bool draw_wheel_contacts;
	bool draw_norm_shear;	// 3001
	bool draw_all_tireForces;	// 3002
	bool draw_TMaligned;	// 3003

	// menu items, checkboxes ids are: 2xxx
	IGUIScrollBar*  scrollbar_steer;
	IGUIStaticText* text_steerPos;	// JCM
	IGUIStaticText* text_FspringK;
	IGUIScrollBar*  scrollbar_FspringK;
	IGUIStaticText* text_FdamperR;
	IGUIScrollBar*  scrollbar_FdamperR;
	IGUIStaticText* text_FspringL;
	IGUIScrollBar*  scrollbar_FspringL;
	IGUIStaticText* text_throttle;
	IGUIScrollBar*  scrollbar_throttle;
	// JCM text output of tire forces/moments
	// info boxes to output Tire Force/Moment parameters
	IGUIStaticText* text_infoRF;
	IGUIStaticText* text_infoLF;
	IGUIStaticText* text_infoLB;
	IGUIStaticText* text_infoRB;
	// for body data
	IGUIStaticText* text_infoChassis;

//	ChVector<> rel1_steer;
//	ChVector<> rel2_steer;
	double K_0;		// initial spring stiffness, defined in PacejkaTire.cpp
	double R_0;		// initial damping, "
	double L_0_F;		// initial spring pre-length, front springs
	double L_0_R;	// rear springs
	// for rendering terrain
    scene::ISceneNode* Terrain;
    scene::ISceneNode* Skybox;
    scene::ISceneNode* Skydome;
    bool showBox;
    bool showDebug;

	// scroll bars for wheel control
	IGUIScrollBar* scrollbar_torqueVel;		// torque or linear velocity applied to testMech, id = 1109
	IGUIStaticText* text_chassisCMPos;	// output chassis info
	IGUIStaticText* text_chassisCMVel;
	IGUIStaticText* text_chassisYPR;
	IGUIStaticText* text_torqueVel;	// output applied torque/motion to rear wheels

	// output tabs, and their text boxes
	IGUIStaticText* gad_text_carControls;		// applied controls
	IGUIStaticText* gad_text_carControl_output;	// control output
	IGUIStaticText* gad_text_wheelState;	// panel for all wheel state output data
	IGUIStaticText* gad_text_soilState;	// panel for all soil state output data

	// contact force debugging
	IGUICheckBox*	checkbox_draw_normShear;	// if in debug mode, draw the details of that contact
	IGUIStaticText* text_draw_normShear;
	IGUICheckBox*	checkbox_draw_all_tireForces;		// draw the tire/terrain forces?
	IGUIStaticText* text_draw_all_tireForces;	
	IGUICheckBox* checkbox_draw_TMaligned; // draw resulting TM forces
	IGUIStaticText* text_draw_TMaligned;	
};

#endif	// ifndef HMMWVEVENTRECEIVER_H