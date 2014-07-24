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
	// same as in ChSuspension class
	/*
protected:
	// all the bodies
	std::vector<ChSharedPtr<ChBody>> body_list;
	// all the joints/links
	std::vector<ChSharedPtr<ChLinkLock>> link_list;
	// name of the bodies
	std::vector<std::string> body_names;
	// name of the links/joints~IteratorPhysicsItems
	std::vector<std::string> joint_names;
	// suspension name
	std::string subsys_name;
*/
public:
	// easy access to tie rod
	ChSharedPtr<ChLinkDistance> tierod;	// a distance constraint
	// the shock
	ChSharedPtr<ChLinkSpring> shock;

	// @brief default Constructor 
	DoubleAarm() {}

	// @brief suspension constructor. hard code the link and body locations in the suspension,
	//		relative to the chassis. Eventually, read in subsystem info from the file.
	// @param my_system	Chrono system to add bodies, joints, assets, etc. to 
	// @param susp_type	which side of the car? lf, rf, lr(LB), and rr(RB) correspond to 0,1,2 and 3, as input ints
	// @param chassis	the Chassis body, whose cm position and orientation is used to locate the wheel spindle location with...
	// @param wheel		the wheel body
	// @param spindle_r_bar	relative to the chassis C-sys, where the wheel spindle rigid body position is
	DoubleAarm(ChSystem&  my_system, const int susp_type, 
		ChSharedPtr<ChBody>& chassis,	ChSharedPtr<ChBody>& wheel,
		const ChVector<>& spindle_r_bar, const std::string& data_file_name = "none",
		double spindleMass = 1, ChVector<>& spindleInertia = ChVector<>(1,1,1),
		ChVector<>& spindleSize = ChVector<>(.1, .1, .1) );	// 10 cm cube

	// @brief responsible for calling "ChSystem->RemoveXYZ()" for the links and bodies in the model.
	~DoubleAarm();

/*
	// @brief get the list of bodies as const refs
	std::vector<ChSharedPtr<ChBody>>& get_body_ptrs();

	// @brief get the list of joints/links as const refs
	std::vector<ChSharedPtr<ChLinkLock>>& get_link_ptrs();

	// @brief get the names of the bodies
	const std::vector<std::string>& get_body_name_list() {
		return body_names;
	}

	// @brief get the list of names of the links/joints
	const std::vector<std::string>& get_link_name_list() {
		return joint_names;
	}

	// @brief get the name of the subsys
	const std::string& get_subsysName() {
		return subsys_name;
	}
*/

};

#endif