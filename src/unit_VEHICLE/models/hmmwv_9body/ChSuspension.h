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
	std::vector<ChSharedPtr<ChBody>*> body_list;
	// links: kinematically idealized joints, e.g. massless link distance constraint
	std::vector<ChSharedPtr<ChLinkDistance>*> link_list;
	// all the general joints
	std::vector<ChSharedPtr<ChLinkLock>*> joint_list;

	// suspension name
	std::string subsys_name;

public:

	// Constructors, destruct
	ChSuspension(){}
	virtual ~ChSuspension(){}

	// get the list of bodies as const refs
	virtual const std::vector<ChSharedPtr<ChBody>*>& get_body_ptrs(){
		return this->body_list;
	}

	// get the list of joints/links as const refs
	virtual const std::vector<ChSharedPtr<ChLinkDistance>*>& get_link_ptrs(){
		return this->link_list;
	}

	// get the name of the subsys
	const std::string& get_subsysName() {
		return subsys_name;
	}

};

#endif