#ifndef VEHICLE_GUI_H
#define VEHICLE_GUI_H

///////////////////////////////////////////////////
//
//  Vehicle user interface
//
///////////////////////////////////////////////////
 

#include <irrlicht.h>



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

class MySimulator; // this is a forward reference - MySimulator not yet defined - see below.

class MyEventReceiver : public IEventReceiver
{
public:

		// Create the GUI interface (sliders etc.) 
	MyEventReceiver(MySimulator* asimulator);

		// Handle events from the interface
	virtual bool OnEvent(const SEvent& event);

		// Easy conversions from/to slider values to numeric value
	double FromSliderToVal (int    slider, double min_val, double max_val, int maxsteps=100)
			{ return min_val+((double)slider/(double)maxsteps)*(max_val-min_val);};
	int    FromValToSlider (double val   , double min_val, double max_val, int maxsteps=100)
			{ return (int)((double)maxsteps*((val-min_val)/(max_val-min_val)));};
private:
	MySimulator*    msimulator;

public:
	IGUIStaticText* text_steer;
	IGUIScrollBar*  scrollbar_steer;
	IGUIStaticText* text_FspringK;
	IGUIScrollBar*  scrollbar_FspringK;
	IGUIStaticText* text_FdamperR;
	IGUIScrollBar*  scrollbar_FdamperR;
	IGUIStaticText* text_FspringL;
	IGUIScrollBar*  scrollbar_FspringL;
	IGUIStaticText* text_throttle;
	IGUIScrollBar*  scrollbar_throttle;
	IGUIStaticText* text_test;
	IGUIScrollBar*  scrollbar_test;
	IGUIStaticText* text_Mass;
	IGUIScrollBar*  scrollbar_Mass;
	IGUIStaticText* text_rods_steer;
	IGUIScrollBar*  scrollbar_rods_steer;
	IGUIStaticText* text_brake;
	IGUIScrollBar*  scrollbar_brake;

	IGUIStaticText* text_convergenza_anteriore;
	IGUIStaticText* text_convergenza_posteriore;

};



#endif // end of include guard - to avoid double .h inclusion