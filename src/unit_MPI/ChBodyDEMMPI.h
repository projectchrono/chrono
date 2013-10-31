//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2011-2012 Alessandro Tasora
// Copyright (c) 2013 Project Chrono
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be 
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//

#ifndef CHBODYDEMMPI_H
#define CHBODYDEMMPI_H

//////////////////////////////////////////////////
//
//   ChBodyDEMMPI.h
//
//   Class for rigid bodies to be used in MPI
//
//   HEADER file for CHRONO,
//	 Multibody dynamics engine
//
// ------------------------------------------------
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////

#include "ChMpi.h"
#include "physics/ChBodyDEM.h"
#include "ChDomainNodeMPI.h"


namespace chrono
{

/// Class for rigid bodies that can be used in MPI.
/// These are able to cross boundaries of domain decomposition.

class ChApiMPI ChBodyDEMMPI : public ChBodyDEM 
{
public:
						// Chrono simulation of RTTI, needed for serialization
	CH_RTTI(ChBodyDEMMPI,ChBodyDEM);



			//
	  		// CONSTRUCTORS
			//

				/// Build a rigid body.
	ChBodyDEMMPI ();
				/// Destructor
	~ChBodyDEMMPI ();


				/// Tell to a system descriptor that there are variables of type
				/// ChLcpVariables in this object (for further passing it to a LCP solver).
				/// When the ChLcpSystemDescriptor is a ChSystemDescriptorMPIlattice3D special
				/// type, it also sets the 'shared variables' information into it.
	virtual void InjectVariables(ChLcpSystemDescriptor& mdescriptor);

				/// Update all auxiliary data of the rigid body and of
				/// its children (markers, forces..), at given time
	virtual void Update (double mytime);

				/// Update all auxiliary data of the rigid body and of
				/// its children (markers, forces..)
	virtual void Update ();

				/// Update all children forces of the rigid body, at current body state.
				/// If incr=true, add the body forces (gravity, forces, accumulators, scripts)
				/// If incr=false, reset the forces to 0.
	void UpdateForces (double mytime, bool incr);

				/// Gets the 27 bits about which boundary is overlapping with, and returns it.
	int GetOverlapFlags() {return last_shared;};
				

				/// Sets the 27 bits about which boundary is overlapping with, and returns it.
	int ComputeOverlapFlags(ChDomainNodeMPIlattice3D& mnode);


			//
			// STREAMING
			//


				/// Method to allow deserializing a persistent binary archive (ex: a file)
				/// into transient data.
	void StreamIN(ChStreamInBinary& mstream);

				/// Method to allow serializing transient data into a persistent
				/// binary archive (ex: a file).
	void StreamOUT(ChStreamOutBinary& mstream);
				
				/// Access the flag telling which of surrounding domains already cloned it
	int& LastShared() {return last_shared;}

private:

	int last_shared;

};




typedef ChSharedPtr<ChBodyDEMMPI> ChSharedBodyDEMMPIPtr;



} // END_OF_NAMESPACE____


#endif
