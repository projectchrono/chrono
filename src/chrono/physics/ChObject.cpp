//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2010, 2012 Alessandro Tasora
// Copyright (c) 2013 Project Chrono
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//

///////////////////////////////////////////////////
//
//   ChObject.cpp
//
// ------------------------------------------------
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////

#include <stdlib.h>
#include <iostream>
#include <math.h>
#include <float.h>
#include <memory.h>

#include "physics/ChObject.h"
#include "physics/ChGlobal.h"
#include "physics/ChExternalObject.h"

namespace chrono {

//////////////////////////////////////
//////////////////////////////////////

// BASE CLASS FOR HANDLING ITEMS
// with linked-list handling functions

// Register into the object factory, to enable run-time
// dynamic creation and persistence
ChClassRegister<ChObj> a_registration_ChObj;

ChObj::ChObj() {
    name.clear();

    ChTime = 0;
    identifier = 0;
}

ChObj::~ChObj() {
}

void ChObj::Copy(ChObj* source) {
    identifier = source->identifier;

    name = source->name;
    ChTime = source->ChTime;
}

//
// OTHER FUNCTIONS
//

const char* ChObj::GetName() const {
    return (char*)this->name.c_str();
}

void ChObj::SetName(const char myname[]) {
    name = myname;
}

std::string ChObj::GetNameString() const {
    return this->name;
}

void ChObj::SetNameString(const std::string& myname) {
    name = myname;
}



}  // END_OF_NAMESPACE____
