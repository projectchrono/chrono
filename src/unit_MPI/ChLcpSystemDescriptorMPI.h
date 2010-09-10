#ifndef CHLCPSYSTEMDESCRIPTORMPI_H
#define CHLCPSYSTEMDESCRIPTORMPI_H

//////////////////////////////////////////////////
//
//   ChLcpSystemDescriptorMPI.h
//
//    Base class for collecting objects inherited 
//   from ChLcpConstraint or ChLcpVariables. 
//
//   HEADER file for CHRONO HYPEROCTANT LCP solver
//
// ------------------------------------------------
// 	 Copyright:Alessandro Tasora / DeltaKnowledge
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////



#include "lcp/ChLcpSystemDescriptor.h"


namespace chrono
{



/// Class for defining a variable of a domain that is
/// shared with another domain (that can be reached via MPI).
/// It contais also an integer unique ID, that is used if
/// a vector of shared variables must be sorted before sending
/// it via MPI (so that the other domain can remap in its shared vars).

class ChLcpSharedVarMPI
{
public:
	ChLcpVariables*	var;
	int				uniqueID;
	//ChMatrixDynamic<> b;
	
	// operator overloading is needed to enable the stl sorting
	//bool operator<=(const ChVector<Real>&other) const { return x<=other.x && y<=other.y && z<=other.z;};
	//bool operator>=(const ChVector<Real>&other) const { return x>=other.x && y>=other.y && z>=other.z;};
	//bool operator<(const ChVector<Real>&other) const { return x<other.x && y<other.y && z<other.z;};
	bool operator < (const ChLcpSharedVarMPI& other) const { return uniqueID < other.uniqueID;}
	//bool operator ==(const ChLcpSharedVarMPI& other) const { return uniqueID == other.uniqueID;}	
};



/// Class for defining a vector of variables of a domain that are
/// shared with another domain (that can be reached via MPI)

class ChLcpSharedInterfaceMPI
{
protected:
	std::vector<ChLcpSharedVarMPI>  sharedvariables;
	int id_MPI;
	ChMatrixDynamic<> shared_vector;

public:
	ChLcpSharedInterfaceMPI()
					{
						sharedvariables.clear();
					};


	virtual ~ChLcpSharedInterfaceMPI()
					{
						sharedvariables.clear();
					};

		/// Access the vector of variables
	std::vector<ChLcpSharedVarMPI>& GetSharedVariablesList() {return sharedvariables;};

		/// Begin insertion of items
	virtual void BeginInsertion()
					{
						sharedvariables.clear();
					}
		/// End insertion of items: do some preprocessing optimization,
		/// such as allocate the proper size of the shared vector and sort shared variables
		/// according to their unique IDs.
	virtual void EndInsertion();

		/// Insert reference to a ChLcpVariables object
	virtual void InsertSharedVariable(ChLcpSharedVarMPI& mv) { sharedvariables.push_back(mv); }

		/// Set the MPI rank of the other domain that shares this set of variables. 
	virtual void SetMPIfriend(int mid) {id_MPI = mid;}
		/// Get the MPI rank of the other domain that shares this set of variables. 
	virtual int  GetMPIfriend() {return id_MPI;}

		/// Sends a single block of data to the friend domain that shares 
		/// this set of variables. 
		/// Note: this can be called only after EndInsertion() has been done, 
		/// because of needed preoptimizations.
	virtual void SendMPI ();

		/// Receives a single block of data from the friend domain that shares 
		/// this set of variables, and set to 'fb' sparse vectors, by ADDING values.
		/// Note: this can be called only after EndInsertion() has been done, 
		/// because of needed preoptimizations.
	virtual void ReceiveMPIandAdd ();

};


/// System descriptor for domain decomposition.
/// This class collects objects inherited 
/// from ChLcpConstraint or ChLcpVariables, with
/// some of the ChLcpVariables shared between other 
/// ChLcpSystemDescriptorMPI descriptors. 
///	Many ChLcpSystemDescriptorMPI descriptors 
/// can communicate with MPI in a cluster of
/// computers, if some Lcp solver that supports
/// domain decomposition is used.

class ChLcpSystemDescriptorMPI : public ChLcpSystemDescriptor
{

protected:
			//
			// DATA
			//
		std::vector<ChLcpSharedInterfaceMPI>  shared_interfaces;

public:

			//
			// CONSTRUCTORS
			//

	ChLcpSystemDescriptorMPI()
					{
						shared_interfaces.clear();
					};


	virtual ~ChLcpSystemDescriptorMPI()
					{
						shared_interfaces.clear();
					};

	
		/// Begin insertion of items
	virtual void BeginInsertion()
					{
						ChLcpSystemDescriptor::BeginInsertion();
						for (unsigned int j = 0; j < shared_interfaces.size(); j++)
							shared_interfaces[j].BeginInsertion();
					}
		/// End insertion of items
	virtual void EndInsertion()
					{
						ChLcpSystemDescriptor::EndInsertion();
						for (unsigned int j = 0; j < shared_interfaces.size(); j++)
							shared_interfaces[j].EndInsertion();
					}

		/// Access the vector of interfaces
	std::vector<ChLcpSharedInterfaceMPI>& GetSharedInterfacesList() {return shared_interfaces;};

		/// Insert reference to a shared interface
//	virtual void InsertSharedInterface(ChLcpSharedInterfaceMPI* mv) { shared_interfaces.push_back(mv); }

		/// Perform the intercommunication between the 
		/// other domains, using MPI.
		/// In detail, all ChLcpSharedInterfaceMPI of this descriptor
		/// will send shared values to their matching domains, then 
		/// data from matching domains is received and set to variables.
	virtual void PerformCommunication();

};





} // END_OF_NAMESPACE____






#endif  
