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
// 	 Copyright:Alessandro Tasora / DeltaKnowledge
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


#define CHCLASS_CHOBJ 1

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

class ChObj : public ChShared {

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

				// Pointers for fast linked lists
	ChObj* next;
	ChObj* prev;

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
	int  GetIdentifier () { return identifier; }	
				/// Sets the numerical identifier of the object.
	void SetIdentifier (int id) { identifier = id; }

				/// Given a fast list of ChObj, returns the address of the first matching the ID.
	ChObj* GetAddrFromID (ChObj** ChList, int myID);

				/// Gets the simulation time of this object
	double GetChTime () { return ChTime; }	
				/// Sets the simulation time of this object.
	void   SetChTime (double m_time) { ChTime = m_time; }


				/// Gets the name of the object as C Ascii null-terminated string -for reading only!
	char* GetName ();
				/// Sets the name of this object, as ascii string
	void SetName (char myname[]);

				/// Gets the name of the object as C Ascii null-terminated string.
	std::string GetNameString ();
				/// Sets the name of this object, as std::string 
	void SetNameString (std::string& myname);

				 /// Given a fast list of ChObj, returns the address of the first matching the name
	ChObj* Search (ChObj** ChList, char* m_name);


			//
			// 'Fast list' functions. The list itself is simply a ChObj* pointer
			// to the first element in the list.
			//
	
				/// Get next element in list, if this object is a part of a 'fast list'.
	ChObj* GetNext () { return next;}	
				/// Get previous element in list, if this object is a part of a 'fast list'.
	ChObj* GetPrev () { return prev;}
				/// Set next element in list, if this object is a part of a 'fast list'.
	void SetNext (ChObj* newnext) { next= newnext;}
				/// Set previous element in list, if this object is a part of a 'fast list'.
	void SetPrev (ChObj* newprev) { next= newprev;}

				/// Add this object to a preexisting 'fast list', at the end.
	void AddToList (ChObj** ChList);
				/// Removes this object to a preexisting 'fast list', at the end (do not delete it)
	void RemoveFromList(ChObj** ChList);
				/// Count objects in a 'fast list', given pointer to pointer to list.
	static int  ListCount (ChObj** ChList);
				/// Deletes all objects building a 'fast list', given pointer to pointer to list..
	static void KillList	(ChObj** ChList);
	

				/// Returns a reference to the ChExternalObject object associated
				/// to this item. The external object can be used to handle 
				/// external objects when encapsulating other 3d engines or apps.
				/// Return value may be null.
	ChExternalObject* GetExternalObject() {return external_obj;}

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



				/// Translate is used to set a Ch_Tag structure, given a string with the name of a variable.
				/// (All sub classes should implement this one, and call the up-class Translate() only if fail 
				///  to parse the token. The function TranslateWithThesaurus, can be called to help you in parsing..)
	virtual int Translate (char* string, char* string_end, ChTag& ret_tag);

				/// The ParseTag function executes a function of the type 
				/// retv= fx(param, param->next, etc...) depending on the tag identifier of mtag, with optional params.
				/// If "set" = true, the retv is stored into tag (if possible) instead of "get"ting ret = tag.
				/// (All sub classes should implement this, redirecting to the correct up-class if needed)
	virtual int ParseTag (ChTag mtag, ChVar* retv, ChNode<ChVar>* param, int set);

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
