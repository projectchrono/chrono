#ifndef CHPROPLIST_H
#define CHPROPLIST_H

//////////////////////////////////////////////////
//  
//   ChProplist.h
//
//   Property list
//
//   HEADER file for CHRONO,
//	 Multibody dynamics engine
//
// ------------------------------------------------
// 	 Copyright:Alessandro Tasora / DeltaKnowledge
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////


#include "core/ChLists.h"
#include "unit_JS/ChApiJS.h"

namespace chrono 
{


//  To build a tree list of properties for objects

class chjs_propdata {
public:
	char propname[100];
	char label[100];
	ChList<chjs_propdata> children;
	int haschildren;
};

class chjs_fullnamevar {
public:
	char propname[200];
	char label[100];
	int active;
	void* script;		// for example, points to already-compiled Javascript to fetch var
};

} // END_OF_NAMESPACE____


#endif
