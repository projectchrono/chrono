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

#include "chrono/solver/ChKRMBlock.h"

namespace chrono {

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChKRMBlock)

ChKRMBlock::ChKRMBlock(std::vector<ChVariables*> mvariables) {
    SetVariables(mvariables);
}

ChKRMBlock::ChKRMBlock(ChVariables* mvariableA, ChVariables* mvariableB) {
    std::vector<ChVariables*> mvars;
    mvars.push_back(mvariableA);
    mvars.push_back(mvariableB);
    SetVariables(mvars);
}

ChKRMBlock& ChKRMBlock::operator=(const ChKRMBlock& other) {
    if (&other == this)
        return *this;

    variables = other.variables;
    KRM = other.KRM;

    return *this;
}

void ChKRMBlock::SetVariables(std::vector<ChVariables*> mvariables) {
    assert(mvariables.size() > 0);

    variables = mvariables;

    int msize = 0;
    for (unsigned int iv = 0; iv < variables.size(); iv++)
        msize += variables[iv]->Get_ndof();

    KRM.resize(msize, msize);
}

void ChKRMBlock::MultiplyAndAdd(ChVectorRef result, ChVectorConstRef vect) const {
    int kio = 0;
    for (unsigned int iv = 0; iv < GetNumVariables(); iv++) {
        int io = GetVariable(iv)->GetOffset();
        int in = GetVariable(iv)->Get_ndof();
        if (GetVariable(iv)->IsActive()) {
            int kjo = 0;
            for (unsigned int jv = 0; jv < GetNumVariables(); jv++) {
                int jo = GetVariable(jv)->GetOffset();
                int jn = GetVariable(jv)->Get_ndof();
                if (GetVariable(jv)->IsActive()) {
                    for (int r = 0; r < in; r++) {
                        double tot = 0;
                        for (int c = 0; c < jn; c++) {
                            tot += KRM(kio + r, kjo + c) * vect(jo + c);
                        }
                        result(io + r) += tot;
                    }
                    //// RADU: using Eigen as below leads to *significant* performance drop!
                    ////result.segment(io, in) += KRM.block(kio, kjo, in, jn) * vect.segment(jo, jn);
                }
                kjo += jn;
            }
        }
        kio += in;
    }
}

void ChKRMBlock::DiagonalAdd(ChVectorRef result) {
    int kio = 0;
    for (unsigned int iv = 0; iv < GetNumVariables(); iv++) {
        int io = GetVariable(iv)->GetOffset();
        int in = GetVariable(iv)->Get_ndof();
        if (GetVariable(iv)->IsActive()) {
            for (int r = 0; r < in; r++) {
                result(io + r) += KRM(kio + r, kio + r);
            }
            //// RADU: using Eigen as below leads to *noticeable* performance drop!
            ////result.segment(io, in) += KRM.diagonal().segment(kio, in);
        }
        kio += in;
    }
}

void ChKRMBlock::PasteInto(ChSparseMatrix& storage, bool add) {
    if (KRM.rows() == 0)
        return;

    int kio = 0;
    for (unsigned int iv = 0; iv < GetNumVariables(); iv++) {
        int io = GetVariable(iv)->GetOffset();
        int in = GetVariable(iv)->Get_ndof();

        if (GetVariable(iv)->IsActive()) {
            int kjo = 0;
            for (unsigned int jv = 0; jv < GetNumVariables(); jv++) {
                int jo = GetVariable(jv)->GetOffset();
                int jn = GetVariable(jv)->Get_ndof();

                if (GetVariable(jv)->IsActive()) {
                    if (add)
                        PasteMatrix(storage, KRM.block(kio, kjo, in, jn), io, jo, false);
                    else
                        PasteMatrix(storage, KRM.block(kio, kjo, in, jn), io, jo, true);
                }

                kjo += jn;
            }
        }

        kio += in;
    }
}

}  // end namespace chrono
