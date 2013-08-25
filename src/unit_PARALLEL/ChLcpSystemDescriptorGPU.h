#ifndef CHLCPSYSTEMDESCRIPTORGPU_H
#define CHLCPSYSTEMDESCRIPTORGPU_H
//////////////////////////////////////////////////
//
//   ChLcpSystemDescriptorGPU.h
//
//   Contains GPU Memory shared between the Collision Detection and LCP Solver Algorithms
//
//
//   HEADER file for CHRONO HYPEROCTANT LCP solver
//
// ------------------------------------------------
//   Copyright:Alessandro Tasora / DeltaKnowledge
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////
#include "ChCudaMath.h"
#include "ChCudaDefines.h"
#include "ChLcpSolverGPU.h"
#include "lcp/ChLcpSystemDescriptor.h"
namespace chrono {
class ChApiGPU ChLcpSystemDescriptorGPU: public ChLcpSystemDescriptor {
public:
	ChLcpSystemDescriptorGPU() {
	}
	~ChLcpSystemDescriptorGPU() {
	}

	void ConvertToMatrixForm(ChSparseMatrix* Cq, ChSparseMatrix* M, ChSparseMatrix* E, ChMatrix<>* Fvector, ChMatrix<>* Bvector, ChMatrix<>* Frict) {

		std::vector<ChLcpConstraint*>& mconstraints = this->GetConstraintsList();
			std::vector<ChLcpVariables*>&  mvariables	= this->GetVariablesList();


		int n_c = data_container->number_of_rigid_rigid * 3 + data_container->number_of_bilaterals;

		int n_q = Thrust_Count(data_container->host_data.active_data,1)*6;
		//cout << " " << n_q << " " << data_container->number_of_objects << " " << endl;
		if (Cq)
			Cq->Reset(n_c, n_q);
		if (M)
			M->Reset(n_q, n_q);
		if (E)
			E->Reset(n_c, n_c);
		if (Fvector)
			Fvector->Reset(n_q, 1);
		if (Bvector)
			Bvector->Reset(n_c, 1);
		if (Frict)
			Frict->Reset(n_c, 1);

		int s_q = 0;
		for (unsigned int iv = 0; iv< mvariables.size(); iv++)
			{
				if (mvariables[iv]->IsActive())
				{
					if (M)
						mvariables[iv]->Build_M(*M, s_q, s_q);				// .. fills  M
					if (Fvector)
						Fvector->PasteMatrix(&vvariables[iv]->Get_fb(), s_q, 0);// .. fills  'f'
					s_q += mvariables[iv]->Get_ndof();
				}
			}
	}

	ChGPUDataManager* data_container;
};
} // END_OF_NAMESPACE____

#endif

