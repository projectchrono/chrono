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

#ifndef CHLINK_H
#define CHLINK_H

///////////////////////////////////////////////////
//
//   ChLink.h
//
//
//   Base class for "links", i.e. 'joints' (mechanical 
//   constraints) between two moving parts - that is,a
//   joint between two ChBody objects.
//
//   A single ChLink is basically a container of multiple
//   scalar constraints of type ChConstraint(), for example
//   a spherical joint uses three scalar constraints 
//   ChConstraint(), etc., so that the ChSystem can 
//   communicate with links through the 'LCP interface' 
//   member methods (see above). 
//   Another important method is the Update() one.
//
//   HEADER file for CHRONO,
//	 Multibody dynamics engine
//
// ------------------------------------------------
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////



#include "core/ChLog.h"
#include "physics/ChPhysicsItem.h"
#include "physics/ChBody.h"



namespace chrono
{


// Forward references
class ChSystem;




// Define the link type identifier.
//
// Some inherited classes from Link class might override the GetType() function
// to return a custom LNK_ identifiers. Create your own LNK_ identifier for new
// link classes. Otherwise the default behaviour would be to return LNK_BASE.
// This ID info is for maximum speed, to avoid to recognize a link type by RTTI.
// Not all link classes must have a LNK_ identifer, but it is good if at least 
// the specialized classes have their one.
// Note that each class may implement more than one type! (see the ChLinkLock case). 
//
// Such LNK_ link identifiers must be _unique_. If you are not sure about uniqueness,
// do not create them and do not use the GetType() function for your own inherited classes.
//

#define LNK_BASE		29





///
/// Base class for joints betwenn two ChBody objects.
///
///  Links are objects which can be created to constrain two rigid
/// bodies (i.e. objects from the ChBody class) in 3D space, like with
/// revolute joints, guides, etc. 
///
///  Note that there are many specializations of this base class,
/// for example the ChLinkEngine class inherits this base class and
/// implements specific functions to represent an engine between two
/// bodies, etc. etc. (In fact, this base ChLink class does basically
/// _nothing_ unless it is specialized by some child class).
///

class ChApi ChLink : public ChPhysicsItem 
{

	CH_RTTI(ChLink,ChPhysicsItem);

protected:

				//
	  			// DATA
				//

	ChBody* Body1;		// body of marker1 (automatically set)
	ChBody* Body2;		// body of marker2 (automatically set)


	Vector react_force;	// store the xyz reactions, expressed in local coordinate system of link;
	Vector react_torque;// store the torque reactions, expressed in local coordinate system of link;

	bool disabled;		// all constraints of link disabled because of user needs
	bool valid;			// link data is valid
	bool broken;		// link is broken because of excessive pulling/pushing.

public:
				//
	  			// CONSTRUCTORS
				//
	ChLink ();
	virtual ~ChLink ();
	virtual void Copy(ChLink* source);
	virtual ChLink* new_Duplicate ();  


public:
				//
	  			// FUNCTIONS
				//


				/// Tells if the link data is currently valid.
				/// (i.e. pointers to other items are correct)
	virtual bool IsValid() {return valid;}
				/// Set the status of link validity
	virtual void SetValid(bool mon) {valid = mon;}

				/// Tells if all constraints of this link are currently turned
				/// on or off by the user.
	virtual bool IsDisabled() {return disabled;}
				/// User can use this to enable/disable all the constraint of
				/// the link as desired.
	virtual void SetDisabled(bool mdis) {disabled = mdis;}


				/// Tells if the link is broken, for excess of pulling/pushing.
	virtual bool IsBroken() {return broken;}
				/// Ex:3rd party software can set the 'broken' status via this method
	virtual void SetBroken(bool mon) {broken = mon;}


				/// An important function!
				/// Tells if the link is currently active, in general,
				/// that is tells if it must be included into the system solver or not.
				/// This method cumulates the effect of various flags (so a link may
				/// be not active either because disabled, or broken, or not valid)
	virtual bool IsActive()
					{
						return ( valid &&
								!disabled &&
								!broken);
					}

				/// If this link has been created automatically by 
				/// collision detection, returns true (false by default). (Was used in the past, now useless)
	virtual bool IsCreatedByCollisionDetection() {return false;};

				/// Get the type identifier of this link. Use if you don't want to use RTTI for performance.
	virtual int GetType	() {return LNK_BASE;}

				/// Get the number of free degrees of freedom left by this link, between two bodies.
	virtual int GetLeftDOF  () {return 6 - GetDOC();}
				/// Get the number of scalar variables affected by constraints in this link 
	int GetNumCoords() {return 14;}

				/// Get the constrained body '1', the 'slave' body.
	ChBody* GetBody1 () {return Body1;}
				/// Get the constrained body '2', the 'master' body.
	ChBody* GetBody2 () {return Body2;}


				/// Get the link coordinate system, expressed relative to Body2 (the 'master'
				/// body). This represents the 'main' reference of the link: reaction forces 
				/// and reaction torques are expressed in this coordinate system.
				/// By default is in the origin of Body2, but child classes should implement this.
	virtual ChCoordsys<> GetLinkRelativeCoords() {return CSYSNORM;}

				/// Get the master coordinate system for the assets (should be implemented 
				/// by children classes)
	virtual ChFrame<> GetAssetsFrame(unsigned int nclone=0) { return ChFrame<>();}


				/// To get reaction force, expressed in link coordinate system:
	ChVector<> Get_react_force() {return react_force;}
				/// To get reaction torque,  expressed in link coordinate system:
	ChVector<> Get_react_torque() {return react_torque;}


				/// If some constraint is redundant, return to normal state  //***OBSOLETE***
	virtual int  RestoreRedundant() {return 0;};  ///< \return number of changed constraints

				// Sets the link to work only in 2D mode  //***OBSOLETE***
				// mode=1 use only constraints for 2D xy plane, mode=0 switch back to 3D.
	virtual void Set2Dmode(int mode);  

				/// Tells if this link requires that the connected ChBody objects
				/// must be waken if they are sleeping. By default =true, i.e. always keep awaken, but
				/// child classes might return false for optimizing sleeping, in case no time-dependant.
	virtual bool IsRequiringWaking() {return true;}

			//
			// UPDATING FUNCTIONS
			//

					/// Given new time, current body state, updates
					/// time-dependant stuff in link state, for example
					/// motion laws, moving markers, etc.
					/// (Default: do nothing but setting new time.)
	virtual void UpdateTime (double mytime);


					// -----------COMPLETE UPDATE.
					// sequence:
					//			UpdateTime;

					/// This is an important function, which is called by the 
					/// owner ChSystem at least once per integration step.
					/// It may update all auxiliary data of the link, such as
					/// matrices if any, etc.
					/// The inherited classes, for example the ChLinkMask, often
					/// implement specialized versions of this Update(time) function,
					/// because they might need to update inner states, forces, springs, etc.
					/// This base version, by default, simply updates the time.
	virtual void Update (double mytime);

					/// As above, but with current time
	virtual void Update();


					/// Tells to the associated external object of class ChExternalObject(),
					/// if any, that its 3D shape must be updated in order to syncronize to
					/// link state (for example, if chrono is a plugin for a 3D modeler, the
					/// wireframe display of the link may change depending on link state.
	virtual void UpdateExternalGeometry ();

					/// Called from a foreign software (i.e. a plugin, a CAD appl.), if any, to report 
					/// that time has changed. Most often you can leave this unimplemented.
	virtual void UpdatedExternalTime (double prevtime, double time){}; 





			//
			// STREAMING
			//

					/// Method to allow deserializing a persistent binary archive (ex: a file)
					/// into transient data.
	virtual void StreamIN(ChStreamInBinary& mstream);

					/// Method to allow serializing transient data into a persistent
					/// binary archive (ex: a file).
	virtual void StreamOUT(ChStreamOutBinary& mstream);

					/// Method to allow serialization of transient data in ascii,
					/// as a readable item, for example   "chrono::GetLog() << myobject;"
	virtual void StreamOUT(ChStreamOutAscii& mstream) {};

};







} // END_OF_NAMESPACE____

#endif
