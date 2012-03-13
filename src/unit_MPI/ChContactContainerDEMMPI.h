#ifndef CHCONTACTCONTAINERDEMMPI_H
#define CHCONTACTCONTAINERDEMMPI_H

///////////////////////////////////////////////////
//
//   ChContactContainerDEMMPI.h
//
//   Class for container of many contacts, as CPU
//   typical linked list of ChContactDEM objects
//
//   HEADER file for CHRONO,
//	 Multibody dynamics engine
//
// ------------------------------------------------
// 	 Copyright:Alessandro Tasora / DeltaKnowledge
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////


#include "unit_MPI/ChApiMPI.h"
#include "physics/ChContactContainerDEM.h"


namespace chrono
{


///
/// Class representing a container of many contacts, 
/// implemented as a typical linked list of ChContactNode
/// objects (contacts between 3DOF nodes and 6DOF bodies)
///

class ChApiMPI ChContactContainerDEMMPI : public ChContactContainerDEM {

	CH_RTTI(ChContactContainerDEMMPI,ChContactContainerDEM);


public:
				//
	  			// CONSTRUCTORS
				//

	ChContactContainerDEMMPI ();

	virtual ~ChContactContainerDEMMPI ();


				//
	  			// FUNCTIONS
				//
	

	virtual void ConstraintsFbLoadForces(double factor);
};




//////////////////////////////////////////////////////
//////////////////////////////////////////////////////


} // END_OF_NAMESPACE____

#endif

