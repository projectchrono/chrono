///////////////////////////////////////////////////
//
//   ChFunction_Mirror.cpp
//
// ------------------------------------------------
// 	 Copyright:Alessandro Tasora / DeltaKnowledge
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////


#include "ChFunction_Mirror.h"


namespace chrono
{


// Register into the object factory, to enable run-time
// dynamic creation and persistence
ChClassRegister<ChFunction_Mirror> a_registration_mirror;


void ChFunction_Mirror::Copy (ChFunction_Mirror* source)
{
	mirror_axis = source->mirror_axis;
	fa = source->fa->new_Duplicate();
}

ChFunction* ChFunction_Mirror::new_Duplicate ()
{
	ChFunction_Mirror* m_func;
	m_func = new ChFunction_Mirror;
	m_func->Copy(this);
	return (m_func);
}

double ChFunction_Mirror::Get_y      (double x)
{
	if (x<= this->mirror_axis)
		return fa->Get_y(x);
	return fa->Get_y(2*this->mirror_axis - x);
}

void ChFunction_Mirror::Extimate_x_range (double& xmin, double& xmax)
{
	fa->Extimate_x_range(xmin,xmax);
}

int ChFunction_Mirror::MakeOptVariableTree(ChList<chjs_propdata>* mtree)
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


void ChFunction_Mirror::StreamOUT(ChStreamOutBinary& mstream)
{
		// class version number
	mstream.VersionWrite(1);
		// serialize parent class too
	ChFunction::StreamOUT(mstream);

		// stream out all member data
	mstream << mirror_axis;
	mstream.AbstractWrite(fa);
}

void ChFunction_Mirror::StreamIN(ChStreamInBinary& mstream)
{
		// class version number
	int version = mstream.VersionRead();
		// deserialize parent class too
	ChFunction::StreamIN(mstream);

		// stream in all member data
	mstream >> mirror_axis;
	if (fa) delete fa; fa=NULL;
	mstream.AbstractReadCreate(&fa);
}

void ChFunction_Mirror::StreamOUT(ChStreamOutAscii& mstream)
{
	mstream << "FUNCT_MIRROR  \n";

	//***TO DO***
}





} // END_OF_NAMESPACE____


// eof
