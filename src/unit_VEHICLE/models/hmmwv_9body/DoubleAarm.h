#ifndef DOUBLEAARM_H
#define DOUBLEAARM_H

#include "physics/ChApidll.h" 
#include "physics/ChSystem.h"
#include "ChSuspension.h"
#include "physics/ChBodyEasy.h"
// #include "ChAssetHelper_initialization.h"

using namespace chrono;

/// a specific type of suspension, a double A-arm.
///	Kinematic mode, the upper and lower A-arms and the upright and wheel spindle are lumped into a single mass.
///	The suspension kinematics are correct through two sets of distance constraints; however, the dynamics will be slightly off.
/// Author: Justin Madsen
class DoubleAarm : public ChSuspension 
{
public:
	// easy access to tie rod
	ChSharedPtr<ChLinkDistance> tierod;	// a distance constraint
	// the shock
	ChSharedPtr<ChLinkSpring> shock;
	// upright body, combines mass/inertia of upper/lower arms, shock
	ChSharedPtr<ChBodyEasyBox> upright;

	// @brief default Constructor 
	DoubleAarm() {}

	// @brief create a Double A-arm suspension using idealized kinematic links. 
	//	A single body, representing combined inertia of the suspension arms, shock and upright is 
	//	connected to the chassis and wheel.
	//	NOTE: applied torque or motion on the wheel needs to be applied outside this function.
	// @param my_system	Chrono system 
	// @param susp_type	which side of the car? lf, rf, lr(LB), and rr(RB) correspond to 0,1,2 and 3, as input ints
	// @param chassis	the Chassis body, whose cm position and orientation is used to locate the wheel spindle location with...
	// @param wheel		the wheel body
	// @param upright_r_bar spindle connection point to wheel,	relative to the chassis C-sys, 
	DoubleAarm(ChSystem&  my_system, const int susp_type, 
		ChSharedPtr<ChBody>& chassis,	ChSharedPtr<ChBody>& wheel,
		const ChVector<>& upright_r_bar, const std::string& data_file_name = "none",
		double uprightMass = 1, ChVector<>& uprightInertia = ChVector<>(1,1,1),
		ChVector<>& uprightSize = ChVector<>(.1, .1, .1) );	// 10 cm cube

	// @brief responsible for calling "ChSystem->RemoveXYZ()" for the links and bodies in the model.
	~DoubleAarm();

	
};

#endif