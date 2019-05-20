// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2016 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Hammad Mazhar, Radu Serban
// =============================================================================

#pragma once

//#include <thrust/count.h>

#include "chrono/solver/ChSystemDescriptor.h"

#include "chrono_parallel/ChParallelDefines.h"
#include "chrono_parallel/solver/ChIterativeSolverParallel.h"
#include "chrono_parallel/math/ChParallelMath.h"

namespace chrono {

/// @addtogroup parallel_solver
/// @{

/// System descriptor for Chrono::Parallel.
class CH_PARALLEL_API ChSystemDescriptorParallel : public ChSystemDescriptor {
  public:
    ChSystemDescriptorParallel(ChParallelDataManager* dc) : data_manager(dc) {}
    ~ChSystemDescriptorParallel() {}

    // void ConvertToMatrixForm(ChSparseMatrix* Cq,
    //                         ChSparseMatrix* M,
    //                         ChSparseMatrix* E,
    //                         ChMatrix<>* Fvector,
    //                         ChMatrix<>* Bvector,
    //                         ChMatrix<>* Frict) {
    //    std::vector<ChConstraint*>& mconstraints = this->GetConstraintsList();
    //    std::vector<ChVariables*>& mvariables = this->GetVariablesList();

    //    int n_c = data_manager->num_rigid_contacts * 3 + data_manager->num_bilaterals;
    //    int n_q = Thrust_Count(data_manager->host_data.active_rigid, 1) * 6;

    //    // cout << " " << n_q << " " << data_manager->num_bodies << " " << endl;

    //    if (Cq)
    //        Cq->Reset(n_c, n_q);
    //    if (M)
    //        M->Reset(n_q, n_q);
    //    if (E)
    //        E->Reset(n_c, n_c);
    //    if (Fvector)
    //        Fvector->Reset(n_q, 1);
    //    if (Bvector)
    //        Bvector->Reset(n_c, 1);
    //    if (Frict)
    //        Frict->Reset(n_c, 1);

    //    int s_q = 0;
    //    for (unsigned int iv = 0; iv < mvariables.size(); iv++) {
    //        if (mvariables[iv]->IsActive()) {
    //            if (M)
    //                mvariables[iv]->Build_M(*M, s_q, s_q, this->c_a);
    //            if (Fvector)
    //                Fvector->PasteMatrix(vvariables[iv]->Get_fb(), s_q, 0);

    //            s_q += mvariables[iv]->Get_ndof();
    //        }
    //    }
    //}

  private:
    ChParallelDataManager* data_manager;  ///< Pointer to the system's data manager
};

/// @} parallel_solver

} // end namespace chrono
