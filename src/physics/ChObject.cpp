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
#include "physics/ChFormule.h"
#include "physics/ChExternalObject.h"


namespace chrono
{


// Thesaurus of functions/datas
// which can be parsed

static ChThesItem chobj_thesaurus[] = {
    {"identifier",		1, CHCLASS_INTEGER,	 0, 0, 0},
	{"name",			1, CHCLASS_STRINGP,	 0, 0, 0},
    {NULL,}
};


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
	next= 0;
	prev= 0;

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

	next = NULL;	// copies not list-linked!
	prev = NULL;
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
// LIST MANAGEMENT FUNCTIONS
//

ChObj* ChObj::GetAddrFromID (ChObj** ChList, int myID)
{
	ChObj* ChPointer = *ChList;
	ChObj* Candidate = NULL;

	while (ChPointer != NULL)
	{
		if (myID == ChPointer->GetIdentifier())
		{
			Candidate = ChPointer;
		}
		ChPointer = (ChObj*) ChPointer->GetNext();
	}
	return (Candidate);
}


ChObj* ChObj::Search (ChObj** ChList, char* m_name)
{
	ChObj* ChPointer = *ChList;
	ChObj* Candidate = NULL;

	while (ChPointer != NULL)
	{
		if (!(strcmp(m_name,ChPointer->GetName())))
		{
			Candidate = ChPointer;
		}
		ChPointer = (ChObj*) ChPointer->GetNext();
	}
	return (Candidate);
}

void ChObj::AddToList (ChObj** ChList)
{
	next= NULL;

	if (*ChList != NULL)
	{
		ChObj* ChPointer = *ChList;
		while (ChPointer->GetNext() != NULL)
		{
			ChPointer = (ChObj*) ChPointer->GetNext();
		}
		ChPointer->SetNext(this);
		prev= ChPointer;
	}
	else
	{
		*ChList= this;
		prev= NULL;
	}
}

void ChObj::RemoveFromList (ChObj** ChList)
{
	// relink the link gap
	if (next)
		next->prev= prev;
	if (prev)
		prev->next= next;

	// if it was first in list
	if (!prev)
		*ChList = NULL;

	// set linked pointers of this obj.
	next = NULL;
	prev = NULL;
}


int ChObj::ListCount (ChObj** ChList)
{
	int count = 0;

	ChObj* ChPointer = *ChList;

	while (ChPointer != NULL)
	{
		++count ;
		ChPointer = (ChObj*) ChPointer->GetNext();
	}

	return count;
}

void ChObj::KillList (ChObj** ChList)
{
	ChObj* ChTemp;
	ChObj* ChPointer = *ChList;

	while (ChPointer != NULL)
	{
		ChTemp= ChPointer;
		ChPointer = (ChObj*) ChPointer->GetNext();
		int iden = ChTemp->GetIdentifier();
		delete (ChTemp);
	}
	*ChList= NULL;
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

//
// PARSING TAGS
//


int ChObj::Translate (char* string, char* string_end,
							  ChTag& ret_tag)
{
	if (TranslateWithThesaurus (string, string_end,
							chobj_thesaurus, this, CHCLASS_ChOBJ,
							ret_tag))
		return TRUE;	// the token has been correctly parsed with own thesaurus, and
						// transformed into tag "ret_tag"
	else
		return FALSE;	// here cannot ask the parent class to parse it, since
						// this is the base class ... return FALSE = insuccess
}

int ChObj::ParseTag (ChTag mtag, ChVar* retv, ChNode<ChVar>* param, int set)
{
	if ((mtag.classID > 0) && (mtag.tagID==0))
	{
		retv->ResetAsPointer(mtag.object);   // was pointer to some object
		return TRUE;
	}

	if (mtag.classID == CHCLASS_ChOBJ)
	{
		if (!set)	// GET values
		{
			switch (mtag.tagID)
			{
			case 1:	// identifier
				retv->ResetAsInt(this->GetIdentifier()); return TRUE;
			case 2: // name
				retv->ResetAsStringPointer(this->GetName()); return TRUE;
			default:
				return FALSE;
			}
		}
		else		// SET values
		{
			switch (mtag.tagID)
			{
			case 1:	// identifier
				this->SetIdentifier((int)retv->mdouble); return TRUE;
			case 2: // name
				this->SetName((char*)retv->varp); return TRUE;
			default:
				return FALSE;
			}
		}
	}
	else
		return FALSE;	// sorry, no parent class, could not parse it.
}



} // END_OF_NAMESPACE____


