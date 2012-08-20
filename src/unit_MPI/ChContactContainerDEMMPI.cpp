///////////////////////////////////////////////////
//
//   ChContactContainerDEMMPI.cpp
//
// ------------------------------------------------
// 	 Copyright:Alessandro Tasora / DeltaKnowledge
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////
 
  
#include "unit_MPI/ChContactContainerDEMMPI.h"
#include "physics/ChSystem.h"
#include "unit_MPI/ChSystemMPI.h"
#include "physics/ChIndexedNodes.h"
#include "physics/ChBodyDEM.h"
#include "collision/ChCModelBulletDEM.h"

#include "core/ChMemory.h" // must be last include (memory leak debugger). In .cpp only.


namespace chrono
{


using namespace collision;
using namespace geometry;





// Register into the object factory, to enable run-time
// dynamic creation and persistence
ChClassRegister<ChContactContainerDEMMPI> a_registration_ChContactContainerDEMMPI;
 

ChContactContainerDEMMPI::ChContactContainerDEMMPI ()
{ 
	contactlist.clear();
	n_added = 0;

}


ChContactContainerDEMMPI::~ChContactContainerDEMMPI ()
{
	/*
	std::list<ChContactDEM*>::iterator itercontact = contactlist.begin();
	while(itercontact != contactlist.end())
	{
		delete (*itercontact);
		(*itercontact) = 0;
		++itercontact;
		//contactlist.erase(itercontact); //no! do later with clear(), all together
	}
	contactlist.clear();
	*/
	//already done in ChContactContainerDEM class destructor
}



void ChContactContainerDEMMPI::ConstraintsFbLoadForces(double factor)
{
	ChContactDEM* cntct;
	bool apply_force=true;

	std::list<ChContactDEM*>::iterator itercontact = contactlist.begin();
	while(itercontact != contactlist.end())
	{
		apply_force=true;
		cntct = (ChContactDEM*)(*itercontact);
		if (ChSystemMPI* syss = dynamic_cast<ChSystemMPI*>(this->GetSystem()))
		{
			if( ~(*syss).nodeMPI.IsInto( (cntct->GetContactP1() + cntct->GetContactP2())/2.0 ) )
			{
				apply_force=false;
			}
		}
		if ( apply_force && (cntct->GetContactDistance()<0) )
		{
			(*itercontact)->ConstraintsFbLoadForces(factor);
		}
		++itercontact;
	}
}


} // END_OF_NAMESPACE____



