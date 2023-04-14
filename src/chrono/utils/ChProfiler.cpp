// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Alessandro Tasora
// =============================================================================


/*
CODE MODIFIED FROM:


***************************************************************************************************
**
** profile.cpp
**
** Real-Time Hierarchical Profiling for Game Programming Gems 3
**
** by Greg Hjelstrom & Byon Garrabrant
**
***************************************************************************************************/

// Credits: The Clock class was inspired by the Timer classes in 
// Ogre (www.ogre3d.org).

#include "chrono/core/ChTimer.h"
#include "chrono/utils/ChProfiler.h"

#include <ctime>
#include <ratio>
#include <chrono>
#include <cstdio>

namespace chrono {
namespace utils {

#ifndef CH_NO_PROFILE

static ChTimer gProfileClock;

#define mymin(a,b) (a > b ? a : b)

inline void Profile_Get_Ticks(unsigned long int * ticks)
{
	*ticks = (unsigned long)gProfileClock.GetTimeMicrosecondsIntermediate();
}

inline float Profile_Get_Tick_Rate(void)
{
//	return 1000000.f;
	return 1000.f;

}




/***************************************************************************************************
**
** ChProfileNode
**
***************************************************************************************************/

/***********************************************************************************************
 * INPUT:                                                                                      *
 * name - pointer to a static string which is the name of this profile node                    *
 * parent - parent pointer                                                                     *
 *                                                                                             *
 * WARNINGS:                                                                                   *
 * The name is assumed to be a static pointer, only the pointer is stored and compared for     *
 * efficiency reasons.                                                                         *
 *=============================================================================================*/
ChProfileNode::ChProfileNode( const char * name, ChProfileNode * parent ) :
	Name( name ),
	TotalCalls( 0 ),
	TotalTime( 0 ),
	StartTime( 0 ),
	RecursionCounter( 0 ),
	Parent( parent ),
	Child( NULL ),
	Sibling( NULL )
{
	Reset();
}


void	ChProfileNode::CleanupMemory()
{
	delete ( Child);
	Child = NULL;
	delete ( Sibling);
	Sibling = NULL;
}

ChProfileNode::~ChProfileNode( void )
{
	delete ( Child);
	delete ( Sibling);
}


/***********************************************************************************************
 * INPUT:                                                                                      *
 * name - static string pointer to the name of the node we are searching for                   *
 *                                                                                             *
 * WARNINGS:                                                                                   *
 * All profile names are assumed to be static strings so this function uses pointer compares   *
 * to find the named node.                                                                     *
 *=============================================================================================*/
ChProfileNode * ChProfileNode::Get_Sub_Node( const char * name )
{
	// Try to find this sub node
	ChProfileNode * child = Child;
	while ( child ) {
		if ( child->Name == name ) {
			return child;
		}
		child = child->Sibling;
	}

	// We didn't find it, so add it
	
	ChProfileNode * node = new ChProfileNode( name, this );
	node->Sibling = Child;
	Child = node;
	return node;
}


void	ChProfileNode::Reset( void )
{
	TotalCalls = 0;
	TotalTime = 0.0f;
	

	if ( Child ) {
		Child->Reset();
	}
	if ( Sibling ) {
		Sibling->Reset();
	}
}


void	ChProfileNode::Call( void )
{
	TotalCalls++;
	if (RecursionCounter++ == 0) {
		Profile_Get_Ticks(&StartTime);
	}
}


bool	ChProfileNode::Return( void )
{
	if ( --RecursionCounter == 0 && TotalCalls != 0 ) { 
		unsigned long int time;
		Profile_Get_Ticks(&time);
		time-=StartTime;
		TotalTime += (float)time / Profile_Get_Tick_Rate();
	}
	return ( RecursionCounter == 0 );
}


/***************************************************************************************************
**
** ChProfileIterator
**
***************************************************************************************************/
ChProfileIterator::ChProfileIterator( ChProfileNode * start )
{
	CurrentParent = start;
	CurrentChild = CurrentParent->Get_Child();
}


void	ChProfileIterator::First(void)
{
	CurrentChild = CurrentParent->Get_Child();
}


void	ChProfileIterator::Next(void)
{
	CurrentChild = CurrentChild->Get_Sibling();
}


bool	ChProfileIterator::Is_Done(void)
{
	return CurrentChild == NULL;
}


void	ChProfileIterator::Enter_Child( int index )
{
	CurrentChild = CurrentParent->Get_Child();
	while ( (CurrentChild != NULL) && (index != 0) ) {
		index--;
		CurrentChild = CurrentChild->Get_Sibling();
	}

	if ( CurrentChild != NULL ) {
		CurrentParent = CurrentChild;
		CurrentChild = CurrentParent->Get_Child();
	}
}


void	ChProfileIterator::Enter_Parent( void )
{
	if ( CurrentParent->Get_Parent() != NULL ) {
		CurrentParent = CurrentParent->Get_Parent();
	}
	CurrentChild = CurrentParent->Get_Child();
}


/***************************************************************************************************
**
** ChProfileManager
**
***************************************************************************************************/

ChProfileNode	ChProfileManager::Root( "Root", NULL );
ChProfileNode *	ChProfileManager::CurrentNode = &ChProfileManager::Root;
int				ChProfileManager::FrameCounter = 0;
unsigned long int			ChProfileManager::ResetTime = 0;


/***********************************************************************************************
 * ChProfileManager::Start_Profile -- Begin a named profile                                    *
 *                                                                                             *
 * Steps one level deeper into the tree, if a child already exists with the specified name     *
 * then it accumulates the profiling; otherwise a new child node is added to the profile tree. *
 *                                                                                             *
 * INPUT:                                                                                      *
 * name - name of this profiling record                                                        *
 *                                                                                             *
 * WARNINGS:                                                                                   *
 * The string used is assumed to be a static string; pointer compares are used throughout      *
 * the profiling code for efficiency.                                                          *
 *=============================================================================================*/
void	ChProfileManager::Start_Profile( const char * name )
{
	if (name != CurrentNode->Get_Name()) {
		CurrentNode = CurrentNode->Get_Sub_Node( name );
	} 
	
	CurrentNode->Call();
}


/***********************************************************************************************
 * ChProfileManager::Stop_Profile -- Stop timing and record the results.                       *
 *=============================================================================================*/
void	ChProfileManager::Stop_Profile( void )
{
	// Return will indicate whether we should back up to our parent (we may
	// be profiling a recursive function)
	if (CurrentNode->Return()) {
		CurrentNode = CurrentNode->Get_Parent();
	}
}


/***********************************************************************************************
 * ChProfileManager::Reset -- Reset the contents of the profiling system                       *
 *                                                                                             *
 *    This resets everything except for the tree structure.  All of the timing data is reset.  *
 *=============================================================================================*/
void	ChProfileManager::Reset( void )
{ 
	gProfileClock.reset();
    gProfileClock.start();
	Root.Reset();
    Root.Call();
	FrameCounter = 0;
	Profile_Get_Ticks(&ResetTime);
}


/***********************************************************************************************
 * ChProfileManager::Increment_Frame_Counter -- Increment the frame counter                    *
 *=============================================================================================*/
void ChProfileManager::Increment_Frame_Counter( void )
{
	FrameCounter++;
}


/***********************************************************************************************
 * ChProfileManager::Get_Time_Since_Reset -- returns the elapsed time since last reset         *
 *=============================================================================================*/
float ChProfileManager::Get_Time_Since_Reset( void )
{
	unsigned long int time;
	Profile_Get_Ticks(&time);
	time -= ResetTime;
	return (float)time / Profile_Get_Tick_Rate();
}


void	ChProfileManager::dumpRecursive(ChProfileIterator* profileIterator, int spacing)
{
	profileIterator->First();
	if (profileIterator->Is_Done())
		return;

    float accumulated_time = 0, parent_time = profileIterator->Is_Root()
                                                  ? ChProfileManager::Get_Time_Since_Reset()
                                                  : profileIterator->Get_Current_Parent_Total_Time();
    int frames_since_reset = ChProfileManager::Get_Frame_Count_Since_Reset();
    for (int i = 0; i < spacing; i++)
        printf(".");
    printf("----------------------------------\n");
    for (int i = 0; i < spacing; i++)
        printf(".");
    printf("Profiling: %s (total running time: %.3f ms) ---\n", profileIterator->Get_Current_Parent_Name(),
           parent_time);
    float totalTime = 0.f;

	
	int numChildren = 0;
	
	for (int i = 0; !profileIterator->Is_Done(); i++, profileIterator->Next()) {
        numChildren++;
        float current_total_time = profileIterator->Get_Current_Total_Time();
        accumulated_time += current_total_time;
        float fraction = parent_time > FLT_EPSILON ? (current_total_time / parent_time) * 100 : 0.f;
            for (int j = 0; j < spacing; j++)
                printf(".");
        printf("%d -- %s (%.2f %%) :: %.3f ms / frame (%d calls)\n", i, profileIterator->Get_Current_Name(), fraction,
               (current_total_time / (double)frames_since_reset), profileIterator->Get_Current_Total_Calls());
        totalTime += current_total_time;
        // recurse into children
    }

    if (parent_time < accumulated_time) {
        printf("what's wrong\n");
    }
    for (int i = 0; i < spacing; i++)
        printf(".");
    printf("%s (%.3f %%) :: %.3f ms\n",
           "Unaccounted:", parent_time > FLT_EPSILON ? ((parent_time - accumulated_time) / parent_time) * 100 : 0.f,
           parent_time - accumulated_time);

    for (int i = 0; i < numChildren; i++) {
        profileIterator->Enter_Child(i);
        dumpRecursive(profileIterator, spacing + 3);
        profileIterator->Enter_Parent();
    }
}



void	ChProfileManager::dumpAll()
{
	ChProfileIterator* profileIterator = 0;
	profileIterator = ChProfileManager::Get_Iterator();

	dumpRecursive(profileIterator,0);

	ChProfileManager::Release_Iterator(profileIterator);
}




#endif //CH_NO_PROFILE

}  // end namespace utils
}  // end namespace chrono
