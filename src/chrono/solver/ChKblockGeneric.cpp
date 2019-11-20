// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Alessandro Tasora, Radu Serban
// =============================================================================

#include "chrono/solver/ChKblockGeneric.h"

namespace chrono {

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChKblockGeneric)

ChKblockGeneric::ChKblockGeneric(std::vector<ChVariables*> mvariables) {
    SetVariables(mvariables);
}

ChKblockGeneric::ChKblockGeneric(ChVariables* mvariableA, ChVariables* mvariableB) {
    std::vector<ChVariables*> mvars;
    mvars.push_back(mvariableA);
    mvars.push_back(mvariableB);
    SetVariables(mvars);
}

ChKblockGeneric& ChKblockGeneric::operator=(const ChKblockGeneric& other) {
    if (&other == this)
        return *this;

    this->variables = other.variables;
    this->K = other.K;

    return *this;
}

void ChKblockGeneric::SetVariables(std::vector<ChVariables*> mvariables) {
    assert(mvariables.size() > 0);

    variables = mvariables;

    int msize = 0;
    for (unsigned int iv = 0; iv < variables.size(); iv++)
        msize += variables[iv]->Get_ndof();

    K.resize(msize, msize);
}

void ChKblockGeneric::MultiplyAndAdd(ChVectorRef result, ChVectorConstRef vect) const {
    int kio = 0;
    for (unsigned int iv = 0; iv < this->GetNvars(); iv++) {
        int io = this->GetVariableN(iv)->GetOffset();
        int in = this->GetVariableN(iv)->Get_ndof();
        if (this->GetVariableN(iv)->IsActive()) {
            int kjo = 0;
            for (unsigned int jv = 0; jv < this->GetNvars(); jv++) {
                int jo = this->GetVariableN(jv)->GetOffset();
                int jn = this->GetVariableN(jv)->Get_ndof();
                if (this->GetVariableN(jv)->IsActive()) {
                    for (int r = 0; r < in; r++) {
                        double tot = 0;
                        for (int c = 0; c < jn; c++) {
                            tot += K(kio + r, kjo + c) * vect(jo + c);
                        }
                        result(io + r) += tot;
                    }
                    //// RADU: using Eigen as below leads to *significant* performance drop!
                    ////result.segment(io, in) += K.block(kio, kjo, in, jn) * vect.segment(jo, jn);
                }
                kjo += jn;
            }
        }
        kio += in;
    }
}

void ChKblockGeneric::DiagonalAdd(ChVectorRef result) {
    int kio = 0;
    for (unsigned int iv = 0; iv < this->GetNvars(); iv++) {
        int io = this->GetVariableN(iv)->GetOffset();
        int in = this->GetVariableN(iv)->Get_ndof();
        if (this->GetVariableN(iv)->IsActive()) {
            for (int r = 0; r < in; r++) {
                result(io + r) += K(kio + r, kio + r);
            }
            //// RADU: using Eigen as below leads to *noticeable* performance drop!
            ////result.segment(io, in) += K.diagonal().segment(kio, in);
        }
        kio += in;
    }
}

void ChKblockGeneric::Build_K(ChSparseMatrix& storage, bool add) {
    if (K.rows() == 0)
        return;

    int kio = 0;
    for (unsigned int iv = 0; iv < this->GetNvars(); iv++) {
        int io = this->GetVariableN(iv)->GetOffset();
        int in = this->GetVariableN(iv)->Get_ndof();

        if (this->GetVariableN(iv)->IsActive()) {
            int kjo = 0;
            for (unsigned int jv = 0; jv < this->GetNvars(); jv++) {
                int jo = this->GetVariableN(jv)->GetOffset();
                int jn = this->GetVariableN(jv)->Get_ndof();

                if (this->GetVariableN(jv)->IsActive()) {
                    if (add)
                        PasteMatrix(storage, K.block(kio, kjo, in, jn), io, jo, false);
                    else
                        PasteMatrix(storage, K.block(kio, kjo, in, jn), io, jo, true);
                }

                kjo += jn;
            }
        }

        kio += in;
    }
}


}  // end namespace chrono
