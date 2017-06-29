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

// Credits: The Clock class was inspired by the Timer classes in  Ogre (www.ogre3d.org).


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


#ifndef CHPROFILER_H
#define CHPROFILER_H

//To disable built-in profiling, please comment out next line
//#define CH_NO_PROFILE 1

#ifndef CH_NO_PROFILE

#include <cstdio>
#include <new>
#include <cfloat>
#include <cfloat>
#include <ctime>
#include <ratio>
#include <chrono>


namespace chrono {
namespace utils {



///A node in the Profile Hierarchy Tree
class	ChProfileNode {

public:
	ChProfileNode( const char * name, ChProfileNode * parent );
	~ChProfileNode( void );

	ChProfileNode * Get_Sub_Node( const char * name );

	ChProfileNode * Get_Parent( void )		{ return Parent; }
	ChProfileNode * Get_Sibling( void )		{ return Sibling; }
	ChProfileNode * Get_Child( void )			{ return Child; }

	void				CleanupMemory();
	void				Reset( void );
	void				Call( void );
	bool				Return( void );

	const char *	Get_Name( void )				{ return Name; }
	int				Get_Total_Calls( void )		{ return TotalCalls; }
	float				Get_Total_Time( void )		{ return TotalTime; }

protected:

	const char *	Name;
	int				TotalCalls;
	float				TotalTime;
	unsigned long int			StartTime;
	int				RecursionCounter;

	ChProfileNode *	Parent;
	ChProfileNode *	Child;
	ChProfileNode *	Sibling;
};

///An iterator to navigate through the tree
class ChProfileIterator
{
public:
	// Access all the children of the current parent
	void				First(void);
	void				Next(void);
	bool				Is_Done(void);
	bool                Is_Root(void) { return (CurrentParent->Get_Parent() == 0); }

	void				Enter_Child( int index );		// Make the given child the new parent
	void				Enter_Largest_Child( void );	// Make the largest child the new parent
	void				Enter_Parent( void );			// Make the current parent's parent the new parent

	// Access the current child
	const char *	Get_Current_Name( void )			{ return CurrentChild->Get_Name(); }
	int				Get_Current_Total_Calls( void )	{ return CurrentChild->Get_Total_Calls(); }
	float				Get_Current_Total_Time( void )	{ return CurrentChild->Get_Total_Time(); }

	// Access the current parent
	const char *	Get_Current_Parent_Name( void )			{ return CurrentParent->Get_Name(); }
	int				Get_Current_Parent_Total_Calls( void )	{ return CurrentParent->Get_Total_Calls(); }
	float				Get_Current_Parent_Total_Time( void )	{ return CurrentParent->Get_Total_Time(); }

protected:

	ChProfileNode *	CurrentParent;
	ChProfileNode *	CurrentChild;

	ChProfileIterator( ChProfileNode * start );
	friend	class		ChProfileManager;
};


///The Manager for the Profile system
class	ChProfileManager {
public:
	static	void						Start_Profile( const char * name );
	static	void						Stop_Profile( void );

	static	void						CleanupMemory(void)
	{
		Root.CleanupMemory();
	}

	static	void						Reset( void );
	static	void						Increment_Frame_Counter( void );
	static	int						Get_Frame_Count_Since_Reset( void )		{ return FrameCounter; }
	static	float						Get_Time_Since_Reset( void );

	static	ChProfileIterator *	Get_Iterator( void )	
	{ 
		
		return new ChProfileIterator( &Root ); 
	}
	static	void						Release_Iterator( ChProfileIterator * iterator ) { delete ( iterator); }

	static void	dumpRecursive(ChProfileIterator* profileIterator, int spacing);

	static void	dumpAll();

private:
	static	ChProfileNode			Root;
	static	ChProfileNode *			CurrentNode;
	static	int						FrameCounter;
	static	unsigned long int					ResetTime;
};


///ProfileSampleClass is a simple way to profile a function's scope
///Use the BT_PROFILE macro at the start of scope to time
class	CProfileSample {
public:
	CProfileSample( const char * name )
	{ 
		ChProfileManager::Start_Profile( name ); 
	}

	~CProfileSample( void )					
	{ 
		ChProfileManager::Stop_Profile(); 
	}
};


#define	CH_PROFILE( name )			CProfileSample __profile( name )


}  // end namespace utils
}  // end namespace chrono


#else

#define	CH_PROFILE( name )

#endif //#ifndef CH_NO_PROFILE



#endif //QUICK_PROF_H


