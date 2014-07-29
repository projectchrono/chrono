#ifndef CHSUSPENSION_H
#define CHSUSPENSION_H

#include "physics/ChApidll.h" 
#include "physics/ChSystem.h"

using namespace chrono;

/// Used in a HMMWV (or any other) vehicle model template. Specific implementations
///	can have additional functionality from those inherited from this parent class.
class ChSuspension
{
protected:
	// all the bodies
	std::vector<ChBody*> body_list;
	
	// links: kinematically idealized joints, e.g. massless link distance constraint
	std::vector<ChLinkDistance*> link_list;
	
	// all the general joints
	std::vector<ChLinkLock*> joint_list;

	// suspension name
	std::string subsys_name;

public:

	// Constructors, destruct
	ChSuspension(){}
	virtual ~ChSuspension(){}

	virtual 

	// get the list of bodies as const refs
	const std::vector<ChBody*>& get_body_ptrs(){
		return this->body_list;
	}

	// get the list of geometric constraints (links) as const refs
	const std::vector<ChLinkDistance*>& get_link_ptrs(){
		return this->link_list;
	}

	// get the list of joints in this subsystem
	const std::vector<ChLinkLock*>& get_joint_ptrs(){
		return this->joint_list;
	}

	// get the name of the subsys
	const std::string& get_subsysName() {
		return subsys_name;
	}

};

#endif