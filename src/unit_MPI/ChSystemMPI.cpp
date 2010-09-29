///////////////////////////////////////////////////
//
//   ChSystemMPI.cpp
//
// ------------------------------------------------
// 	 Copyright:Alessandro Tasora / DeltaKnowledge
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////


#include "mpi.h"
#include "ChMpi.h"
#include "ChSystemMPI.h"

using namespace std;


namespace chrono
{


/*
void ChSystemMPI::LCPprepare_inject(ChLcpSystemDescriptor& mdescriptor)
{
	mdescriptor.BeginInsertion(); // This resets the vectors of constr. and var. pointers.

	HIER_LINK_INIT
	while HIER_LINK_NOSTOP
	{
		Lpointer->InjectConstraints(mdescriptor);
		HIER_LINK_NEXT
	}
	HIER_BODY_INIT
	while HIER_BODY_NOSTOP
	{
		Bpointer->InjectVariables(mdescriptor);
		HIER_BODY_NEXT
	}
	HIER_OTHERPHYSICS_INIT
	while HIER_OTHERPHYSICS_NOSTOP
	{
		PHpointer->InjectVariables(mdescriptor);
		PHpointer->InjectConstraints(mdescriptor);
		HIER_OTHERPHYSICS_NEXT
	}
	this->contact_container->InjectConstraints(mdescriptor);


	mdescriptor.EndInsertion(); 
}
*/






} // END_OF_NAMESPACE____


////// end
