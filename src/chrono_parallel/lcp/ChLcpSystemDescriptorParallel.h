#ifndef CHLCPSYSTEMDESCRIPTORPARALLEL_H
#define CHLCPSYSTEMDESCRIPTORPARALLEL_H

#include <thrust/count.h>

#include "lcp/ChLcpSystemDescriptor.h"

#include "chrono_parallel/ChParallelDefines.h"
#include "chrono_parallel/lcp/ChLcpSolverParallel.h"
#include "chrono_parallel/math/ChParallelMath.h"

namespace chrono {

class CH_PARALLEL_API ChLcpSystemDescriptorParallel : public ChLcpSystemDescriptor {
 public:
  ChLcpSystemDescriptorParallel(ChParallelDataManager* dc) : data_manager(dc) {}
  ~ChLcpSystemDescriptorParallel() {}

  void ConvertToMatrixForm(ChSparseMatrix* Cq,
                           ChSparseMatrix* M,
                           ChSparseMatrix* E,
                           ChMatrix<>* Fvector,
                           ChMatrix<>* Bvector,
                           ChMatrix<>* Frict) {
    std::vector<ChLcpConstraint*>& mconstraints = this->GetConstraintsList();
    std::vector<ChLcpVariables*>& mvariables = this->GetVariablesList();

    int n_c = data_manager->num_rigid_contacts * 3 + data_manager->num_bilaterals;
    int n_q = Thrust_Count(data_manager->host_data.active_rigid, 1) * 6;

    // cout << " " << n_q << " " << data_manager->num_bodies << " " << endl;

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
    for (unsigned int iv = 0; iv < mvariables.size(); iv++) {
      if (mvariables[iv]->IsActive()) {
        if (M)
          mvariables[iv]->Build_M(*M, s_q, s_q);
        if (Fvector)
          Fvector->PasteMatrix(&vvariables[iv]->Get_fb(), s_q, 0);

        s_q += mvariables[iv]->Get_ndof();
      }
    }
  }

 private:
  ChParallelDataManager* data_manager;
};

}  // END_OF_NAMESPACE____

#endif
