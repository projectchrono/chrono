//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2010 Alessandro Tasora
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be 
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//

///////////////////////////////////////////////////
//
//   ChEvents.cpp
//
// ------------------------------------------------
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////

  
 
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <float.h>
#include <memory.h>

#include "physics/ChEvents.h"

namespace chrono 
{ 
 
/////////////////////////////////////////////////////////
/// 
///   EVENT BUFFER CLASS
///
///


ChEvents::ChEvents (int m_events)
{
	maxevents = m_events;
	current_event = 0;
	ebuffer = (int*) calloc (maxevents, sizeof(int));
	memset(ebuffer, 0, (sizeof(int)*maxevents));
}

ChEvents::ChEvents ()
{
	maxevents = CHCLASS_EVENTS_DEFAULTNUM;
	current_event = 0;
	ebuffer = (int*) calloc (maxevents, sizeof(int));
	memset(ebuffer, 0, (sizeof(int)*maxevents));
}

ChEvents::~ChEvents ()
{
	free(ebuffer);
}

void ChEvents::ResetAllEvents()
{
	memset(ebuffer, 0, (sizeof(int)*maxevents));
}

void ChEvents::Record (int m_event)
{
	// increase pointer 
	current_event++;
	// if end of buffer, repeat cycle
	if (current_event>=maxevents) current_event = 0;
	// store new;
	*(ebuffer+current_event) = m_event; 
}	

int ChEvents::GetN(int n_back)
{
	if (n_back >= maxevents) n_back = maxevents -1; 
	int po = current_event - n_back;
	if (po < 0) po = maxevents + po;
	return *(ebuffer+po);
}

void ChEvents::SetN(int n_back, int m_event)
{
	if (n_back >= maxevents) n_back = maxevents -1; 
	int po = current_event - n_back;
	if (po < 0) po = maxevents + po;
	*(ebuffer+po) = m_event;
}
	
int ChEvents::GetLast()
{
	return *(ebuffer+current_event);
}


 
} // END_OF_NAMESPACE____

////// end
