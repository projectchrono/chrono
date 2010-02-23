///////////////////////////////////////////////////
//
//   ChLcpConstraint.cpp
//
//
//    file for CHRONO HYPEROCTANT LCP solver
//
// ------------------------------------------------
// 	 Copyright:Alessandro Tasora / DeltaKnowledge
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////
 

#include "ChLcpVariables.h"
#include "core/ChMemory.h" // must be after system's include (memory leak debugger).


namespace chrono 
{


ChLcpVariables& ChLcpVariables::operator=(const ChLcpVariables& other)
{
	if (&other == this) return *this;

	this->disabled = other.disabled;

	if (other.qb)
	{
		if (qb==NULL)
			qb = new ChMatrixDynamic<>;
		qb->CopyFromMatrix(*other.qb);
	}
	else
	{
		if (qb) delete qb;
		qb=NULL;
	}

	if (other.fb)
	{
		if (fb==NULL)
			fb = new ChMatrixDynamic<>;
		fb->CopyFromMatrix(*other.fb);
	}
	else
	{
		if (fb) delete fb;
		fb=NULL;
	}

	this->ndof = other.ndof;
	this->offset = other.offset;

	return *this;
}


// Register into the object factory, to enable run-time
// dynamic creation and persistence
//ChClassRegister<ChLcpVariables> a_registration_ChLcpVariables;



} // END_OF_NAMESPACE____

