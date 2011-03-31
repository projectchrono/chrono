///////////////////////////////////////////////////
//
//   ChFunction_Repeat.cpp
//
// ------------------------------------------------
// 	 Copyright:Alessandro Tasora / DeltaKnowledge
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////


#include "ChFunction_Repeat.h"


namespace chrono
{


// Register into the object factory, to enable run-time
// dynamic creation and persistence
ChClassRegister<ChFunction_Repeat> a_registration_repeat;


void ChFunction_Repeat::Copy (ChFunction_Repeat* source)
{
	window_start = source->window_start;
	window_length = source->window_length;
	fa = source->fa->new_Duplicate();
}

ChFunction* ChFunction_Repeat::new_Duplicate ()
{
	ChFunction_Repeat* m_func;
	m_func = new ChFunction_Repeat;
	m_func->Copy(this);
	return (m_func);
}

double ChFunction_Repeat::Get_y      (double x)
{
	return fa->Get_y(this->window_start + fmod(x,this->window_length) );
}

void ChFunction_Repeat::Extimate_x_range (double& xmin, double& xmax)
{
	fa->Extimate_x_range(xmin,xmax);
}

int ChFunction_Repeat::MakeOptVariableTree(ChList<chjs_propdata>* mtree)
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

void ChFunction_Repeat::StreamOUT(ChStreamOutBinary& mstream)
{
		// class version number
	mstream.VersionWrite(1);
		// serialize parent class too
	ChFunction::StreamOUT(mstream);

		// stream out all member data
	mstream << window_start;
	mstream << window_length;
	mstream.AbstractWrite(fa);
}

void ChFunction_Repeat::StreamIN(ChStreamInBinary& mstream)
{
		// class version number
	int version = mstream.VersionRead();
		// deserialize parent class too
	ChFunction::StreamIN(mstream);

		// stream in all member data
	mstream >> window_start;
	mstream >> window_length;
	if (fa) delete fa; fa=NULL;
	mstream.AbstractReadCreate(&fa);
}

void ChFunction_Repeat::StreamOUT(ChStreamOutAscii& mstream)
{
	mstream << "FUNCT_REPEAT  \n";

	//***TO DO***
}




} // END_OF_NAMESPACE____


// eof
