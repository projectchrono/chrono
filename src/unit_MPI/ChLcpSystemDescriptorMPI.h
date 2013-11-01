//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2010, 2012 Alessandro Tasora
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be 
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//

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
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////


#include "ChApiMPI.h"
#include "lcp/ChLcpSystemDescriptor.h"
#include "ChDomainNodeMPI.h"


namespace chrono
{



/// Class for defining a variable of a domain that is
/// shared with another domain (that can be reached via MPI).
/// It contais also an integer unique ID, that is used if
/// a vector of shared variables must be sorted before sending
/// it via MPI (so that the other domain can remap in its shared vars).

class ChApiMPI ChLcpSharedVarMPI
{
public:
	ChLcpVariables*	var;
	int				uniqueID;
	bool			master;	
	
	// operator overloading is needed to enable the stl sorting
	bool operator < (const ChLcpSharedVarMPI& other) const { return uniqueID < other.uniqueID;}
};



/// Class for defining a vector of variables of a domain that are
/// shared with another domain (that can be reached via MPI)

class ChApiMPI ChLcpSharedInterfaceMPI
{
protected:
	std::vector<ChLcpSharedVarMPI>  sharedvariables;
	int id_MPI;
	ChMatrixDynamic<> shared_vector_out;
	ChMatrixDynamic<> shared_vector_in;
	ChMPIrequest mpi_request;

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

	virtual ChMatrixDynamic<>* GetSharedVectorIn()  {return &shared_vector_in;}
	virtual ChMatrixDynamic<>* GetSharedVectorOut() {return &shared_vector_out;}
	virtual ChMPIrequest*  GetMPIrequest() {return &mpi_request;}

		/// Sends a single block of data to the friend domain that shares 
		/// this set of variables. 
		/// Note: this can be called only after EndInsertion() has been done, 
		/// because of needed preoptimizations.
	virtual void SendMPI ();

		/// Receives a single block of data from the friend domain that shares 
		/// this set of variables, and set to 'fb' sparse vectors, by ADDING values.
		/// Note: this can be called only after EndInsertion() has been done, 
		/// because of needed preoptimizations.
	virtual void ReceiveMPI ();

	virtual void AddReceivedMPI ();


		/// Testing, for now.
	virtual void SendForcesMPI ();

	virtual void ReceiveForcesMPI ();

	virtual void AddReceivedForcesMPI ();

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

class ChApiMPI ChLcpSystemDescriptorMPI : public ChLcpSystemDescriptor
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

		/// Testing, for now.
	virtual void PerformForcesCommunication();

};



/// System descriptor for domain decomposition of 
/// complementarity problems, where the decomposition
/// corresponds to a 3D cubic lattice partitioning of the space.

class ChApiMPI ChSystemDescriptorMPIlattice3D : public ChLcpSystemDescriptorMPI
{

public:
			//
			// DATA
			//
		ChDomainNodeMPIlattice3D* lattice_node;

public:

			//
			// CONSTRUCTORS
			//

	ChSystemDescriptorMPIlattice3D(ChDomainNodeMPI* mnode)
					{
						// make sure the domain node is lattice type, like the descriptor
						ChDomainNodeMPIlattice3D *lnode = dynamic_cast<ChDomainNodeMPIlattice3D*>(mnode);
						if (lnode==0){
							GetLog() << "ERROR: domain node not of lattice type\n";
						}
						lattice_node = lnode;
						this->shared_interfaces.resize(27);

						for (int i = 0; i < 27; i++)
							this->shared_interfaces[i].SetMPIfriend (lnode->interfaces[i].id_MPI);
					};

};


/// System descriptor for domain decomposition of
/// complementarity problems, where the decomposition
/// corresponds to a 3D grid, this is more general than a lattice

class ChApiMPI ChSystemDescriptorMPIgrid3D : public ChLcpSystemDescriptorMPI
{

public:
			//
			// DATA
			//
		ChDomainNodeMPIgrid3D* grid_node;

public:

			//
			// CONSTRUCTORS
			//

	ChSystemDescriptorMPIgrid3D(ChDomainNodeMPI* mnode)
					{
						// make sure the domain node is grid type, like the descriptor
						ChDomainNodeMPIgrid3D *gnode = dynamic_cast<ChDomainNodeMPIgrid3D*>(mnode);
						if (gnode==0){
							GetLog() << "ERROR: domain node not of grid type\n";
						}
						grid_node = gnode;
						int num_interfaces = gnode->interfaces.size();

						this->shared_interfaces.resize(num_interfaces);

						for (int i = 0; i < num_interfaces; i++)
							this->shared_interfaces[i].SetMPIfriend (gnode->interfaces[i].id_MPI);
					};

};



} // END_OF_NAMESPACE____






#endif  
