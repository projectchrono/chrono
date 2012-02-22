#ifndef CHASSEMBLYMPI_H
#define CHASSEMBLYMPI_H

//////////////////////////////////////////////////
//
//   ChAssemblyMPI.h
//
//   Class for an assembly of constrained bodies
//   to be considered together.
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
#include "physics/ChAssembly.h"
#include "ChDomainNodeMPI.h"


namespace chrono
{

/// Class for an assembly of rigid bodies and constraints that can be used in MPI.
/// These are able to cross boundaries of domain decomposition.

class ChApiMPI ChAssemblyMPI : public ChAssembly 
{
public:
						// Chrono simulation of RTTI, needed for serialization
	CH_RTTI(ChAssemblyMPI,ChAssembly);



			//
	  		// CONSTRUCTORS
			//

				/// Build an assembly.
	ChAssemblyMPI ();
				/// Destructor
	~ChAssemblyMPI ();


				/// Gets the 27 bits about which boundary is overlapping with, and returns it.
	int GetOverlapFlags() {return last_shared;};
				

				/// Sets the 27 bits about which boundary is overlapping with, and returns it.
	int ComputeOverlapFlags(ChDomainNodeMPIlattice3D& mnode);

				/// Update all auxiliary data of the assembly at given time
	virtual void Update (double mytime);

				/// Update all auxiliary data of the assembly
	virtual void Update ();


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




typedef ChSharedPtr<ChAssemblyMPI> ChSharedAssemblyMPIPtr;



} // END_OF_NAMESPACE____


#endif

