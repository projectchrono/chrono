///////////////////////////////////////////////////
//
//   ChObject.cpp
//
// ------------------------------------------------
// 	 Copyright:Alessandro Tasora / DeltaKnowledge
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


namespace chrono
{



//////////////////////////////////////
//////////////////////////////////////

// BASE CLASS FOR HANDLING ITEMS
// with linked-list handling functions

// Register into the object factory, to enable run-time
// dynamic creation and persistence
ChClassRegister<ChObj> a_registration_ChObj;


ChObj::ChObj ()						// builder
{
	name.clear();
	//next= 0;
	//prev= 0;

	ChTime = 0;
	identifier = 0;

	external_obj = 0;
}

ChObj::~ChObj()
{
	if (external_obj)
		delete external_obj;
}


void ChObj::Copy(ChObj* source)
{
	identifier = source->identifier;

	name = source->name;
	ChTime = source->ChTime;

	if (source->GetExternalObject())
		SetExternalObject(source->GetExternalObject());
	else
		SetNoExternalObject();
}



void ChObj::SetExternalObject(ChExternalObject* m_obj)
{
	if (external_obj)
		delete external_obj;
	external_obj = m_obj->new_Duplicate();
}

void ChObj::SetNoExternalObject()
{
	if (external_obj)
		delete external_obj;
	external_obj = NULL;
}



//
// OTHER FUNCTIONS
//

char* ChObj::GetName ()
{
	if (this->external_obj)
		return external_obj->GetName();
	else
		return (char*)this->name.c_str();
}

void ChObj::SetName (char myname[])
{
	name = myname;
}


std::string ChObj::GetNameString ()
{
	if (this->external_obj)
		return std::string(external_obj->GetName());
	else
		return this->name;
}

void ChObj::SetNameString (std::string& myname)
{
	name = myname;
}




//
// FILE BINARY I/O
//




void ChObj::StreamOUT(ChStreamOutBinary& mstream)
{
		// class version number
	mstream.VersionWrite(2);
		// stream out all member data
	mstream << identifier;
	mstream << GetName();
}


void ChObj::StreamIN(ChStreamInBinary& mstream)
{
		// class version number
	int version = mstream.VersionRead();
		// stream in all member data
	if (version <= 1)
	{
		int mfoo;
		mstream >> mfoo;
	}
	mstream >> identifier;
	char mbuffer[250];
	mstream >> mbuffer;
	SetName(mbuffer);
}

void ChObj::StreamOUT(ChStreamOutAscii& mstream)
{
	mstream <<"object type:" << this->GetRTTI()->GetName() << "\n";
	mstream << GetName() << "\n";
	mstream << GetIdentifier() << "\n";
}




} // END_OF_NAMESPACE____


