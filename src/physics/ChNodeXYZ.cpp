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


#include "physics/ChNodeXYZ.h"

#include "core/ChMemory.h" // must be last include (memory leak debugger). In .cpp only.


namespace chrono
{


ChNodeXYZ::ChNodeXYZ()
{
	this->pos = VNULL;
	this->pos_dt = VNULL;
	this->pos_dtdt = VNULL;
}

ChNodeXYZ::~ChNodeXYZ()
{
}

ChNodeXYZ::ChNodeXYZ (const ChNodeXYZ& other) :
					ChNodeBase(other) 
{
	this->pos = other.pos;
	this->pos_dt = other.pos_dt;
	this->pos_dtdt = other.pos_dtdt;
}

ChNodeXYZ& ChNodeXYZ::operator= (const ChNodeXYZ& other)
{
	if (&other == this) 
		return *this;

	ChNodeBase::operator=(other);

	this->pos = other.pos;
	this->pos_dt = other.pos_dt;
	this->pos_dtdt = other.pos_dtdt;

	return *this;
}







} // END_OF_NAMESPACE____


/////////////////////
