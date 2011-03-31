///////////////////////////////////////////////////
//
//   ChFunction_Derive.cpp
//
// ------------------------------------------------
// 	 Copyright:Alessandro Tasora / DeltaKnowledge
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////


#include "ChFunction_Derive.h"


namespace chrono
{


// Register into the object factory, to enable run-time
// dynamic creation and persistence
ChClassRegister<ChFunction_Derive> a_registration_derive;


void ChFunction_Derive::Copy (ChFunction_Derive* source)
{
	order = source->order;
	fa = source->fa->new_Duplicate();
}

ChFunction* ChFunction_Derive::new_Duplicate ()
{
	ChFunction_Derive* m_func;
	m_func = new ChFunction_Derive;
	m_func->Copy(this);
	return (m_func);
}

double ChFunction_Derive::Get_y      (double x)
{
	return fa->Get_y_dx(x);
}

void ChFunction_Derive::Extimate_x_range (double& xmin, double& xmax)
{
	fa->Extimate_x_range(xmin,xmax);
}


int ChFunction_Derive::MakeOptVariableTree(ChList<chjs_propdata>* mtree)
{
	int i=0;

	// inherit parent behaviour
	ChFunction::MakeOptVariableTree(mtree);

	// expand tree for children..

	chjs_propdata* mdataA = new chjs_propdata;
	strcpy(mdataA->propname, "fa");
	strcpy(mdataA->label,    mdataA->propname);
	mdataA->haschildren = TRUE;
	mtree->AddTail(mdataA);

	i += this->fa->MakeOptVariableTree(&mdataA->children);

	return i;
}

void ChFunction_Derive::StreamOUT(ChStreamOutBinary& mstream)
{
		// class version number
	mstream.VersionWrite(1);
		// serialize parent class too
	ChFunction::StreamOUT(mstream);

		// stream out all member data
	mstream << order;
	mstream.AbstractWrite(fa);
}

void ChFunction_Derive::StreamIN(ChStreamInBinary& mstream)
{
		// class version number
	int version = mstream.VersionRead();
		// deserialize parent class too
	ChFunction::StreamIN(mstream);

		// stream in all member data
	mstream >> order;

	if (fa) delete fa; fa=NULL;
	mstream.AbstractReadCreate(&fa);
}

void ChFunction_Derive::StreamOUT(ChStreamOutAscii& mstream)
{
	mstream << "FUNCT_DERIVE  \n";

	//***TO DO***
}





} // END_OF_NAMESPACE____


// eof
