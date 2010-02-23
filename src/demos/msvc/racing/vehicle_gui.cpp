///////////////////////////////////////////////////
//
// Vehicle user interface
 

#include "physics/ChSystem.h"
#include "physics/ChLinkDistance.h"
 
#include "vehicle_gui.h"
#include "vehicle_simulator.h"
#include <irrlicht.h> 

  

// Use the namespace of Chrono

using namespace chrono;

// Use the main namespaces of Irrlicht
using namespace irr;

using namespace core;
using namespace scene;
using namespace video;
using namespace io;
using namespace gui;



// Define a MyEventReceiver class which will be used to manage input
// from the GUI graphical user interface (the interface will
// be created with the basic -yet flexible- platform
// independent toolset of Irrlicht).
//class MySimulator; // this is a forward reference - MySimulator not yet defined - see below.


MyEventReceiver::MyEventReceiver(MySimulator* asimulator)
{
	// store pointer to physical system & other stuff so we can tweak them by user keyboard
	msimulator = asimulator;

	msimulator->device->setEventReceiver(this);

	// ..add a GUI slider to control gas throttle via mouse
	scrollbar_throttle = msimulator->device->getGUIEnvironment()->addScrollBar(
					true, rect<s32>(10, 85, 150, 100), 0, 100);
	scrollbar_throttle->setMax(100); 
	scrollbar_throttle->setPos(0);
	text_throttle = msimulator->device->getGUIEnvironment()->addStaticText(
					L"Throttle", rect<s32>(150,85,250,100), false);

	// ..add a GUI slider to control steering via mouse
	scrollbar_steer = msimulator->device->getGUIEnvironment()->addScrollBar(
					true, rect<s32>(10, 105, 150, 120), 0, 101);
	scrollbar_steer->setMax(100); 
	scrollbar_steer->setPos(50);
	text_steer = msimulator->device->getGUIEnvironment()->addStaticText(
					L"Steer", rect<s32>(150,105,250,120), false);

		// ..add a GUI text and GUI slider to control the stiffness
	scrollbar_FspringK = msimulator->device->getGUIEnvironment()->addScrollBar(
					true, rect<s32>(10, 125, 150, 140), 0, 102);
	scrollbar_FspringK->setMax(100); 
	scrollbar_FspringK->setPos( FromValToSlider(msimulator->mycar->link_springRF->Get_SpringK(), 20000, 150000) );
	text_FspringK = msimulator->device->getGUIEnvironment()->addStaticText(
					L"Spring K [N/m]:    ", rect<s32>(150,125,250,140), false);

	// ..add a GUI text and GUI slider to control the damping
	scrollbar_FdamperR = msimulator->device->getGUIEnvironment()->addScrollBar(
					true, rect<s32>(10, 145, 150, 160), 0, 103);
	scrollbar_FdamperR->setMax(100); 
	scrollbar_FdamperR->setPos( FromValToSlider(msimulator->mycar->link_springRF->Get_SpringR(), 0, 1900) );
	text_FdamperR = msimulator->device->getGUIEnvironment()->addStaticText(
					L"Damper R [Ns/m]:   ", rect<s32>(150,145,250,160), false);

	// ..add a GUI text and GUI slider to control the original undeformed spring length
	scrollbar_FspringL = msimulator->device->getGUIEnvironment()->addScrollBar(
					true, rect<s32>(10, 165, 150, 180), 0, 104);
	scrollbar_FspringL->setMax(100); 
	scrollbar_FspringL->setPos( FromValToSlider(msimulator->mycar->link_springRF->Get_SpringRestLenght(), 0.300, 0.380) );
	text_FspringL = msimulator->device->getGUIEnvironment()->addStaticText(
					L"Spring L [m]:     ", rect<s32>(150,165,250,180), false);	

		// ..add a GUI text and GUI slider to control the mass
	scrollbar_Mass = msimulator->device->getGUIEnvironment()->addScrollBar(
					true, rect<s32>(10, 185, 150, 200), 0, 105);
	scrollbar_Mass->setMax(100); 
	scrollbar_Mass->setPos( FromValToSlider(msimulator->mycar->truss->GetBody()->GetMass(), 10, 800) ); // map from max min range
	text_Mass = msimulator->device->getGUIEnvironment()->addStaticText(
					L"Mass [kg]:     ", rect<s32>(150,185,250,200), false);

		// ..add a GUI text and GUI slider to control the length of both the rods of the steer
	scrollbar_rods_steer = msimulator->device->getGUIEnvironment()->addScrollBar(
					true, rect<s32>(10, 205, 150, 220), 0, 106);
	scrollbar_rods_steer->setMax(100); 
	scrollbar_rods_steer->setPos( FromValToSlider(msimulator->mycar->link_distLSTEER->GetImposedDistance(), 0.23, 0.26) ); // map from max min range
	text_rods_steer = msimulator->device->getGUIEnvironment()->addStaticText(
					L"Steer rods length [m]:", rect<s32>(150,205,280,220), false);


		// ..add a GUI text and GUI slider to control the length of both the rods of the steer
	scrollbar_brake = msimulator->device->getGUIEnvironment()->addScrollBar(
					true, rect<s32>(10, 225, 150, 240), 0, 107);
	scrollbar_brake->setMax(100); 
	scrollbar_brake->setPos(0);
	text_brake = msimulator->device->getGUIEnvironment()->addStaticText(
					L"Braking", rect<s32>(150,225,280,240), false);


		// ..add a GUI text to show the actual convergence
	text_convergenza_anteriore = msimulator->device->getGUIEnvironment()->addStaticText(
					L"Convergenza ant.: ", rect<s32>(10,250,250,275), false);
		// ..add a GUI text to show the actual convergence
	text_convergenza_posteriore = msimulator->device->getGUIEnvironment()->addStaticText(
					L"Convergenza pos.: ", rect<s32>(10,270,250,295), false);

}




bool MyEventReceiver::OnEvent(const SEvent& event)
{
	if (event.EventType == EET_MOUSE_INPUT_EVENT)
	{
		if (event.MouseInput.Event == EMIE_MOUSE_WHEEL)
		{
			double incbrakethrottle = 0;
			if (event.MouseInput.Wheel == 1)
				incbrakethrottle = 1;
			if (event.MouseInput.Wheel == -1)
				incbrakethrottle = -1;
			if (incbrakethrottle)
			{
				if (msimulator->mycar->throttle>0)
				{
					double oldthrottle = msimulator->mycar->throttle;
					double newthrottle = oldthrottle += incbrakethrottle* 0.1;
					if (newthrottle < 0 ) newthrottle=0;
					if (newthrottle > 1 ) newthrottle=1;
					msimulator->mycar->throttle = newthrottle;
				}
				if (msimulator->mycar->throttle==0)
				{
					double oldbraking = msimulator->mycar->GetBraking();
					double newbraking = oldbraking -= incbrakethrottle* 0.1;
					if (newbraking < 0 ) { newbraking=0; msimulator->mycar->throttle = 0.001;}
					if (newbraking > 1 ) newbraking=1;
					msimulator->mycar->SetBraking(newbraking);
				}
				this->scrollbar_throttle->setPos(100*msimulator->mycar->throttle);
				this->scrollbar_brake->setPos(100*msimulator->mycar->GetBraking());
			}
		}
	}

	// check if user moved the sliders with mouse..
	if (event.EventType == EET_GUI_EVENT)
	{
		s32 id = event.GUIEvent.Caller->getID();

		switch(event.GUIEvent.EventType)
		{
		case EGET_SCROLL_BAR_CHANGED:
				if (id == 101) // id of 'steer' slider..
				{
					s32 pos = ((IGUIScrollBar*)event.GUIEvent.Caller)->getPos();
					double newsteer = FromSliderToVal(pos, -0.039, 0.039);  // map in max min range
					// set the steering
					msimulator->mycar->wanted_steer = newsteer;
					return true;
				}
				if (id == 102) // id of 'spring stiffness' slider..
				{
					s32 pos = ((IGUIScrollBar*)event.GUIEvent.Caller)->getPos();
					double newstiff =  FromSliderToVal(pos, 20000, 150000);  // map in max min range
					// set the stiffness of all 4 springs
					msimulator->mycar->link_springRF->Set_SpringK(newstiff);
					msimulator->mycar->link_springLF->Set_SpringK(newstiff);
					msimulator->mycar->link_springRB->Set_SpringK(newstiff);
					msimulator->mycar->link_springLB->Set_SpringK(newstiff);
					// show stiffness as formatted text in interface screen
					char message[50]; sprintf(message,"Spring K [N/m]: %g",newstiff);
					text_FspringK->setText(core::stringw(message).c_str());
					return true;
				}
				if (id == 103) // id of 'damping' slider..
				{
					s32 pos = ((IGUIScrollBar*)event.GUIEvent.Caller)->getPos();
					double newdamping =   FromSliderToVal(pos, 0, 1900);  // map in max min range
					// set the damping of all 4 springs
					msimulator->mycar->link_springRF->Set_SpringR(newdamping);
					msimulator->mycar->link_springLF->Set_SpringR(newdamping);
					msimulator->mycar->link_springRB->Set_SpringR(newdamping);
					msimulator->mycar->link_springLB->Set_SpringR(newdamping);
					// show stiffness as formatted text in interface screen
					char message[50]; sprintf(message,"Damping R [Ns/m]: %g",newdamping);
					text_FdamperR->setText(core::stringw(message).c_str());
					return true;
				}
				if (id == 104) // id of 'spring rest length' slider..
				{
					s32 pos = ((IGUIScrollBar*)event.GUIEvent.Caller)->getPos();
					double newlength = FromSliderToVal(pos, 0.300, 0.380);  // map in max min range
					// set the rest length of all 4 springs
					msimulator->mycar->link_springRF->Set_SpringRestLenght(newlength);
					msimulator->mycar->link_springLF->Set_SpringRestLenght(newlength);
					msimulator->mycar->link_springRB->Set_SpringRestLenght(newlength);
					msimulator->mycar->link_springLB->Set_SpringRestLenght(newlength);
					// show stiffness as formatted text in interface screen
					char message[50]; sprintf(message,"Spring L [m]: %g",newlength);
					text_FspringL->setText(core::stringw(message).c_str());
					return true;
				}
				if (id == 100) // id of 'throttle' slider..
				{
					s32 pos = ((IGUIScrollBar*)event.GUIEvent.Caller)->getPos();
					double newthrottle =  ((double)(pos))/100.0 ;
					// Set the throttle value of car (the torque transmitted
					// to wheels depends on throttle, speed, transmission gear, so 
					// it will sent to the link_engineR and link_engineL only when
					// computed by MySimplifiedCar::ComputeWheelTorque(),
					msimulator->mycar->throttle=newthrottle;
					return true;
				}
				if (id == 105) // id of 'spring rest length' slider..
				{
					s32 pos = ((IGUIScrollBar*)event.GUIEvent.Caller)->getPos();
					double newmass = FromSliderToVal(pos, 10, 800);  // map to max min range
					// set the rest length of all 4 springs
					msimulator->mycar->truss->GetBody()->SetMass(newmass);
					// show mass as formatted text in interface screen
					char message[50]; sprintf(message,"Mass [kg]: %g",newmass);
					text_Mass->setText(core::stringw(message).c_str());
					return true;
				}
				if (id == 106) // id of 'length of rods of steer' slider..
				{
					s32 pos = ((IGUIScrollBar*)event.GUIEvent.Caller)->getPos();
					double newlen = FromSliderToVal(pos, 0.23, 0.26);  // map to max min range
					// set the rest length of all 4 springs
					msimulator->mycar->link_distLSTEER->SetImposedDistance(newlen);
					msimulator->mycar->link_distRSTEER->SetImposedDistance(newlen);
					// show mass as formatted text in interface screen
					char message[50]; sprintf(message,"Steer rods length [m]:%g",newlen);
					text_rods_steer->setText(core::stringw(message).c_str());
					return true;
				}
				if (id == 107) // id of 'braking' slider..
				{
					s32 pos = ((IGUIScrollBar*)event.GUIEvent.Caller)->getPos();
					double newbraking = FromSliderToVal(pos, 0, 1);  // map to max min range
					// set the braking
					msimulator->mycar->SetBraking(newbraking);
					return true;
				}
				break;
		default:
				break;
		}	
	} 
	
	// check if user presses keys
	if (event.EventType == irr::EET_KEY_INPUT_EVENT && !event.KeyInput.PressedDown)
	{
		switch (event.KeyInput.Key)
		{
		case irr::KEY_F1:	// camera will follow car, from inside car
			msimulator->camera->setParent(this->msimulator->mycar->truss);
			msimulator->camera->setPosition(vector3df(0,0.8,1)); // relative to car COG
			msimulator->camera_view_type = 1;
			return true;
		case irr::KEY_F2:	// camera will follow car from up-side
			msimulator->camera->setParent(this->msimulator->mycar->truss);
			msimulator->camera->setPosition(vector3df(-2,3, 1)); // relative to car COG
			msimulator->camera_view_type = 2;
			return true;
		case irr::KEY_F3:	// camera will follow car, from right
			msimulator->camera->setParent(this->msimulator->mycar->truss);
			msimulator->camera->setPosition(vector3df(3,1,0)); // relative to car COG
			msimulator->camera_view_type = 3; 
			return true;
		case irr::KEY_F4:	// camera will follow car, from behind
			msimulator->camera->setParent(this->msimulator->mycar->truss);
			msimulator->camera->setPosition(vector3df(0,1.5, 2.8)); // relative to car COG
			msimulator->camera_view_type = 4;
			return true;
		case irr::KEY_F5:	// camera will stay up in sky, fixed but looking at car
			msimulator->camera->setParent(this->msimulator->device->getSceneManager()->getRootSceneNode());
			msimulator->camera->setPosition(vector3dfCH((this->msimulator->mycar->truss->GetBody()->GetPos()
				              +ChVector<>(-4,8,-2))) );
			msimulator->camera_view_type = 5;
			return true;
		case irr::KEY_KEY_A:	// sequential gear: DOWN
			msimulator->mycar->ChangeGearDown();
			return true;
		case irr::KEY_KEY_S:	// sequential gear: UP
			msimulator->mycar->ChangeGearUp();
			return true;
		case irr::KEY_KEY_F:	// save a ChFunction to disk
			msimulator->mycar->SaveFunctionsToDisk();
			return true;

		default:
			break;
		}
	}

	return false;
} 

