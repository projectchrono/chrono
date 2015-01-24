//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2010 Alessandro Tasora
// Copyright (c) 2013 Project Chrono
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be 
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//
    

#include "physics/ChNodeBase.h"

#include "core/ChMemory.h" // must be last include (memory leak debugger). In .cpp only.

namespace chrono
{


ChNodeBase::ChNodeBase()
{
	this->offset_x = 0;
	this->offset_w = 0;
}

ChNodeBase::~ChNodeBase()
{
}

ChNodeBase::ChNodeBase (const ChNodeBase& other) 
{
	this->offset_x = other.offset_x;
	this->offset_w = other.offset_w;
}

ChNodeBase& ChNodeBase::operator= (const ChNodeBase& other)
{
	if (&other == this) 
		return *this;

	this->offset_x = other.offset_x;
	this->offset_w = other.offset_w;

	return *this;
}


} // END_OF_NAMESPACE____


/////////////////////
