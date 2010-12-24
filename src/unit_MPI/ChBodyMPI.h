#ifndef CHBODYMPI_H
#define CHBODYMPI_H

//////////////////////////////////////////////////
//
//   ChBodyMPI.h
//
//   Class for rigid bodiesto be used in MPI
//
//   HEADER file for CHRONO,
//	 Multibody dynamics engine
//
// ------------------------------------------------
// 	 Copyright:Alessandro Tasora / DeltaKnowledge
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////

#include "ChMpi.h"
#include "physics/ChBody.h"
#include "ChDomainNodeMPI.h"


namespace chrono
{

/// Class for rigid bodies that can be used in MPI.
/// These are able to cross boundaries of domain decomposition.

class ChApiMPI ChBodyMPI : public ChBody 
{
public:
						// Chrono simulation of RTTI, needed for serialization
	CH_RTTI(ChBodyMPI,ChBody);



			//
	  		// CONSTRUCTORS
			//

				/// Build a rigid body.
	ChBodyMPI ();
				/// Destructor
	~ChBodyMPI ();


				/// Tell to a system descriptor that there are variables of type
				/// ChLcpVariables in this object (for further passing it to a LCP solver).
				/// When the ChLcpSystemDescriptor is a ChSystemDescriptorMPIlattice3D special
				/// type, it also sets the 'shared variables' information into it.
	virtual void InjectVariables(ChLcpSystemDescriptor& mdescriptor);

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




typedef ChSharedPtr<ChBodyMPI> ChSharedBodyMPIPtr;



} // END_OF_NAMESPACE____


#endif
