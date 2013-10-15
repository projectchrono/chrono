//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2010-2011 Alessandro Tasora
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be 
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//

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
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////


#include "core/ChLists.h"


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
