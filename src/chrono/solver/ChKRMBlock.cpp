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

ChKRMBlock::ChKRMBlock() : m_has_KR(true) {}

ChKRMBlock::ChKRMBlock(std::vector<ChVariables*> variables) : m_has_KR(true) {
    SetVariables(variables);
}

ChKRMBlock::ChKRMBlock(ChVariables* variableA, ChVariables* variableB) : m_has_KR(true) {
    std::vector<ChVariables*> variables;
    variables.push_back(variableA);
    variables.push_back(variableB);
    SetVariables(variables);
}

ChKRMBlock& ChKRMBlock::operator=(const ChKRMBlock& other) {
    if (&other == this)
        return *this;

    m_has_KR = other.m_has_KR;
    m_variables = other.m_variables;
    m_matrix = other.m_matrix;

    return *this;
}

void ChKRMBlock::SetVariables(std::vector<ChVariables*> variables) {
    assert(variables.size() > 0);

    m_variables = variables;

    unsigned int msize = 0;
    for (unsigned int iv = 0; iv < m_variables.size(); iv++)
        msize += m_variables[iv]->GetDOF();

    m_matrix.resize(msize, msize);
}

void ChKRMBlock::AddMatrixTimesVectorInto(ChVectorRef result, ChVectorConstRef vect) const {
    unsigned int kio = 0;
    for (unsigned int iv = 0; iv < GetNumVariables(); iv++) {
        unsigned int io = GetVariable(iv)->GetOffset();
        unsigned int in = GetVariable(iv)->GetDOF();
        if (GetVariable(iv)->IsActive()) {
            unsigned int kjo = 0;
            for (unsigned int jv = 0; jv < GetNumVariables(); jv++) {
                unsigned int jo = GetVariable(jv)->GetOffset();
                unsigned int jn = GetVariable(jv)->GetDOF();
                if (GetVariable(jv)->IsActive()) {
                    for (unsigned int r = 0; r < in; r++) {
                        double tot = 0;
                        for (unsigned int c = 0; c < jn; c++) {
                            tot += m_matrix(kio + r, kjo + c) * vect(jo + c);
                        }
                        result(io + r) += tot;
                    }
                    //// RADU: using Eigen as below leads to *significant* performance drop!
                    ////result.segment(io, in) += m_matrix.block(kio, kjo, in, jn) * vect.segment(jo, jn);
                }
                kjo += jn;
            }
        }
        kio += in;
    }
}

void ChKRMBlock::DiagonalAdd(ChVectorRef result) const {
    unsigned int kio = 0;
    for (unsigned int iv = 0; iv < GetNumVariables(); iv++) {
        unsigned int io = GetVariable(iv)->GetOffset();
        unsigned int in = GetVariable(iv)->GetDOF();
        if (GetVariable(iv)->IsActive()) {
            for (unsigned int r = 0; r < in; r++) {
                result(io + r) += m_matrix(kio + r, kio + r);
            }
            //// RADU: using Eigen as below leads to *noticeable* performance drop!
            ////result.segment(io, in) += m_matrix.diagonal().segment(kio, in);
        }
        kio += in;
    }
}

void ChKRMBlock::PasteMatrixInto(ChSparseMatrix& mat,
                                 unsigned int start_row,
                                 unsigned int start_col,
                                 bool overwrite) const {
    if (m_matrix.rows() == 0)
        return;

    unsigned int kio = 0;
    for (unsigned int iv = 0; iv < GetNumVariables(); iv++) {
        unsigned int io = GetVariable(iv)->GetOffset();
        unsigned int in = GetVariable(iv)->GetDOF();

        if (GetVariable(iv)->IsActive()) {
            unsigned int kjo = 0;
            for (unsigned int jv = 0; jv < GetNumVariables(); jv++) {
                unsigned int jo = GetVariable(jv)->GetOffset();
                unsigned int jn = GetVariable(jv)->GetDOF();

                if (GetVariable(jv)->IsActive()) {
                    PasteMatrix(mat, m_matrix.block(kio, kjo, in, jn), io + start_row, jo + start_col, overwrite);
                }

                kjo += jn;
            }
        }

        kio += in;
    }
}

}  // end namespace chrono
