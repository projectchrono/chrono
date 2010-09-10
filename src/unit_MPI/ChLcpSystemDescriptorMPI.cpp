///////////////////////////////////////////////////
//
//   ChLcpSystemDescriptor.cpp
//
//
//    file for CHRONO HYPEROCTANT LCP solver
//
// ------------------------------------------------
// 	 Copyright:Alessandro Tasora / DeltaKnowledge
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////

#include <algorithm>
#include "ChLcpSystemDescriptorMPI.h"
#include "ChMpi.h"
 
namespace chrono 
{


void ChLcpSharedInterfaceMPI::EndInsertion()
{
	// 1) Sort the shared variables according to their unique IDs
	std::sort(sharedvariables.begin(), sharedvariables.end());

	// 2) Count active scalar variables and resize the vector.
	int n_q=0;
	for (unsigned int iv = 0; iv< this->sharedvariables.size(); iv++)
	{
		if (sharedvariables[iv].var->IsActive())
		{
			n_q += sharedvariables[iv].var->Get_ndof();
		}
	}

	shared_vector.Resize(n_q, 1);

}

void ChLcpSharedInterfaceMPI::SendMPI ()
{
	// We send via MPI a single vector that contains, queued, all the
	// single vectors of all the variable.
	// This has the drawback of creating a temporary large vector that
	// might waste some more memory in very large simulations, and waste 
	// small time in copy&bookeeping operations.
	// (other options could be doing thousands of small MPI sends directly
	// from the small vectors of the variables, or exploiting some single custom
	// MPI datatype equipped with a vector of offsets).
	// This approach is based on an idea: all the sent values of shared variables must be
	// queued so that their unique ID (that is unique also among domains!) 
	// is sorted in ascending order; this happens also on the receiving domain,
	// so the remapping values->variable objects on the receiving side is unique and quick.


	// 1) Sort the shared variables according to their unique IDs
	//    ... should be already done in EndInsertion()

	// 2) Count active scalar variables and resize the vector to assembly.
	//    ... should be already done in EndInsertion()
		
	// 3) Fill the vector with actual value in 'q' sparse vectors of the variables

	//ChMatrixDynamic<> m_ids(shared_vector); //***DEBUG

	GetLog() << "ID=" << ChMPI::CommRank() << "     sharedvariables.size()=" << sharedvariables.size() << "\n";

 	int s_q=0;
	for (unsigned int iv = 0; iv< sharedvariables.size(); iv++)
	{
		if (sharedvariables[iv].var->IsActive())
		{
			shared_vector.PasteMatrix(&sharedvariables[iv].var->Get_qb(), s_q, 0);
			GetLog() << "ID=" << ChMPI::CommRank() << "     send dfb=" << shared_vector(s_q,0) << "\n";
			s_q += sharedvariables[iv].var->Get_ndof();
			//m_ids(sq,0)= sharedvariables[iv].uniqueID; //***DEBUG
		}
	}

	// 4) Send the vector using MPI
	ChMPI::SendMatrix(this->id_MPI, shared_vector, ChMPI::MPI_STANDARD);
	//ChMPI::SendMatrix(this->id_MPI, m_ids, ChMPI::MPI_STANDARD); //***DEBUG
}




void ChLcpSharedInterfaceMPI::ReceiveMPIandAdd ()
{
	// [read comments in SendMPI() code above.

	// 1) Sort the shared variables according to their unique IDs
	//    ... should be already done in EndInsertion()

	// 2) Count active scalar variables and resize the vector to assembly.
	//    ... should be already done in EndInsertion()
		
	// 3) Receive the vector using MPI
	ChMPIstatus mstatus;
	ChMPI::ReceiveMatrix(this->id_MPI, shared_vector, &mstatus);

	// 4) Get data from the vector and sets to variable objects by adding
	//    to their force sparse vectors 'f'.

	int s_q=0;
	for (unsigned int iv = 0; iv< sharedvariables.size(); iv++)
	{
		if (sharedvariables[iv].var->IsActive())
		{
			GetLog() << "ID=" << ChMPI::CommRank() << "     receive dfb=" << shared_vector(s_q,0) << "\n";
			sharedvariables[iv].var->Get_qb().PasteSumClippedMatrix(&shared_vector, s_q, 0,  sharedvariables[iv].var->Get_ndof(),1,  0,0);
			s_q += sharedvariables[iv].var->Get_ndof();
		}
	}

	
}



void ChLcpSystemDescriptorMPI::PerformCommunication()
{
	//    ***Trick: backup & zero all the sparse vars (qb vectors) of entire domain,
	//    so that the effect of the actual lambdas df = [invM]*[Cq_i]'*l can be computed via 
	//    the method Increment_q() that all ChConstraint surely implement; 

	GetLog() << "ID=" << ChMPI::CommRank() << "  PerformCommunication \n";

	ChMatrixDynamic<> q_backup;
	this->FromVariablesToVector(q_backup);

	//    For the abovementioned trick, set to zero all the qb vectors (will be restored at the end)
	for (unsigned int iv = 0; iv< vvariables.size(); iv++)
		if (vvariables[iv]->IsActive())
			vvariables[iv]->Get_qb().FillElem(0);

	//    Compute the df = [invM]*[Cq_i]'*l computation for all contacts. The 
	//    result is automatically in sparse qb vectors, in all variables.
	//    ***TO DO*** optimize this tricky approach, because this computation might
	//    have been just computed by last iteration of an iterative solver, already...

	for (unsigned int ic = 0; ic< vconstraints.size(); ic++)
		if (vconstraints[ic]->IsActive())
			vconstraints[ic]->Increment_q(vconstraints[ic]->Get_l_i());

	//    MPI!!! Now it is possible to let all shared interfaces to send
	//    qb informations to their matching domains.
	GetLog() << "ID=" << ChMPI::CommRank() << "    shared_interfaces.size()=" << shared_interfaces.size() << "\n";
	for (unsigned int is = 0; is < shared_interfaces.size(); is++)
		shared_interfaces[is].SendMPI();

	//    ***Trick end: restore backup-ed sparse vars (qb vectors) of entire domain,
	//    because we used the qb vectors for sparse computation of df = [invM]*[Cq_i]'*l
	this->FromVectorToVariables(q_backup);

	//    MPI!! Now it is possible to let all shared interfaces to receive
	//    informations from their matching domains and to add to qb vectors in variables.
	for (unsigned int is = 0; is < shared_interfaces.size(); is++)
		shared_interfaces[is].ReceiveMPIandAdd();

	

	GetLog() << "ID=" << ChMPI::CommRank() << "  ...end PerformCommunication \n";
}



} // END_OF_NAMESPACE____


