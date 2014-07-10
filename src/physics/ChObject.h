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

#ifndef CHOBJECT_H
#define CHOBJECT_H

//////////////////////////////////////////////////
//  
//   ChObject.h
//
// Base class for objects which can be renamed, 
// copied, etc. Provides interface to link objects to
// item in hosting applications, like geometric objects 
// in the editor of a 3d modeler.
//
// ------------------------------------------------
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////


#include <stdlib.h>
#include <iostream>
#include <string>
#include <math.h>
#include <float.h>
#include <memory.h>

#include "core/ChLog.h"
#include "core/ChMath.h"
#include "core/ChLists.h"
#include "core/ChShared.h"

#include <vector>



namespace chrono 
{


// Forward references

class ChVar;
class ChTag;
class ChExternalObject;



#define CHOBJ_NAMELENGTH 20


///
/// Base class for items which can be named, deleted,
/// copied. etc. as in the editor of a 3d modeler.
///
/// This class also embeds a simple linked-list mechanism
/// for fast management of linked lists of items, instead
/// of using STL lists or ChList. ***OBSOLETE***
///
/// This class inherits the features of the reference-countable
/// ChShared class, so that  ChObj  instances can be managed
/// easily with the 'intrusive smart pointers' ChSharedPtr
/// (with minimal overhead in performance, yet providing the
/// safe and comfortable automatic deallocation mechanism of
/// shared/smart pointers).
///
/// Each ChObject also has a pointer to user data (for example,
/// the user data can be the encapsulating object in case of
/// implementation as a plugin for 3D modeling software.
///
/// Also, each ChObj object has a 32 bit identifier, in case
/// unique identifiers are used (hash algorithms, etc.)
///

class ChApi ChObj : public virtual ChShared {

						// Chrono simulation of RTTI, needed for serialization
	CH_RTTI(ChObj, ChShared);

private:
			//
			// DATA
			//

				// name of object
	std::string name;

				// ID for referencing
	int identifier;


				// Reference to an external object associated to this item,
				// useful if encapsulating into other 3d engines or apps.
	ChExternalObject* external_obj;


protected: 
				// the time of simulation for the object
	double ChTime;		


public:
			//
			//	CONSTRUCTORS/DELETION
			// 

	ChObj();
	virtual ~ChObj();

	void Copy(ChObj* source);

			//
			// FUNCTIONS
			//

				/// Gets the numerical identifier of the object.
	int  GetIdentifier () const { return identifier; }
				/// Sets the numerical identifier of the object.
	void SetIdentifier (int id) { identifier = id; }

				/// Given a fast list of ChObj, returns the address of the first matching the ID.
	ChObj* GetAddrFromID (ChObj** ChList, int myID);

				/// Gets the simulation time of this object
	double GetChTime () const { return ChTime; }
				/// Sets the simulation time of this object.
	void   SetChTime (double m_time) { ChTime = m_time; }


				/// Gets the name of the object as C Ascii null-terminated string -for reading only!
	char* GetName () const;
				/// Sets the name of this object, as ascii string
	void SetName (const char myname[]);

				/// Gets the name of the object as C Ascii null-terminated string.
	std::string GetNameString () const;
				/// Sets the name of this object, as std::string 
	void SetNameString (std::string& myname);


				/// Returns a reference to the ChExternalObject object associated
				/// to this item. The external object can be used to handle 
				/// external objects when encapsulating other 3d engines or apps.
				/// Return value may be null.
	ChExternalObject* GetExternalObject() const {return external_obj;}

				/// Sets the ChExternalObject of this object (the m_obj will be cloned)
	void   SetExternalObject(ChExternalObject* m_obj);
				/// Sets no external object.
	void   SetNoExternalObject();


		// Set-get generic LONG flags, passed as reference

	void MFlagsSetAllOFF (int& mflag) {mflag = 0;}
	void MFlagsSetAllON (int& mflag) {mflag = 0; mflag = ~ mflag;}
	void MFlagSetON  (int& mflag, int mask) {mflag |= mask ;}
	void MFlagSetOFF (int& mflag, int mask) {mflag &= ~ mask;}
	int  MFlagGet    (int& mflag, int mask) {	return (mflag & mask);};


			//
			// STREAMING
			//

					/// Method to allow serializing transient data into a persistent
					/// binary archive (ex: a file).
	virtual void StreamOUT(ChStreamOutBinary& mstream);

					/// Method to allow deserializing a persistent binary archive (ex: a file)
					/// into transient data.
	virtual void StreamIN(ChStreamInBinary& mstream);


					/// Method to allow serialization of transient data in ascii,
					/// as a readable item, for example   "chrono::GetLog() << myobject;"
	virtual void StreamOUT(ChStreamOutAscii& mstream);


};



// Functions to manipulate STL containers of ChObj objects

template <class T, class Iterator>
T* ChContainerSearchFromName(char* m_name, Iterator from, Iterator to)
{
	Iterator iter = from;
	while (iter != to)
	{
		if (!strcmp(m_name, (*iter)->GetName())) 
			return (*iter);
		iter++;
	}
	return 0; 
}

template <class T, class Iterator>
T* ChContainerSearchFromID(int myID, Iterator from, Iterator to)
{
	Iterator iter = from;
	while (iter != to)
	{
		if (myID == (*iter)->GetIdentifier()) 
			return (*iter);
		iter++;
	}
	return 0;
}




} // END_OF_NAMESPACE____

#endif
