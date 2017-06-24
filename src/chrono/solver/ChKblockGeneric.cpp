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

ChKblockGeneric::ChKblockGeneric(std::vector<ChVariables*> mvariables) : K(NULL) {
    SetVariables(mvariables);
}

ChKblockGeneric::ChKblockGeneric(ChVariables* mvariableA, ChVariables* mvariableB) : K(NULL) {
    std::vector<ChVariables*> mvars;
    mvars.push_back(mvariableA);
    mvars.push_back(mvariableB);
    SetVariables(mvars);
}

ChKblockGeneric::~ChKblockGeneric() {
    if (K)
        delete K;
    K = 0;
}

ChKblockGeneric& ChKblockGeneric::operator=(const ChKblockGeneric& other) {
    if (&other == this)
        return *this;

    // copy parent class data
    // ChKblock::operator=(other);

    this->variables = other.variables;

    if (other.K) {
        if (K == 0)
            K = new ChMatrixDynamic<double>;
        K->CopyFromMatrix(*other.K);
    } else {
        if (K)
            delete K;
        K = NULL;
    }

    return *this;
}

void ChKblockGeneric::SetVariables(std::vector<ChVariables*> mvariables) {
    assert(mvariables.size() > 0);

    variables = mvariables;

    // destroy the K matrix if needed
    if (K)
        delete K;
    K = 0;

    int msize = 0;
    for (unsigned int iv = 0; iv < variables.size(); iv++)
        msize += variables[iv]->Get_ndof();

    // reallocate the K matrix
    K = new ChMatrixDynamic<double>(msize, msize);
}

void ChKblockGeneric::MultiplyAndAdd(ChMatrix<double>& result, const ChMatrix<double>& vect) const {
    assert(K);

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
                    // Multiply the iv,jv sub block of K
                    for (int r = 0; r < in; r++) {
                        double tot = 0;
                        for (int c = 0; c < jn; c++) {
                            tot += (*this->K)(kio + r, kjo + c) * vect(jo + c);
                        }
                        result(io + r) += tot;
                    }
                }

                kjo += jn;
            }
        }

        kio += in;
    }
}

void ChKblockGeneric::DiagonalAdd(ChMatrix<double>& result) {
    assert(result.GetColumns() == 1);

    int kio = 0;
    for (unsigned int iv = 0; iv < this->GetNvars(); iv++) {
        int io = this->GetVariableN(iv)->GetOffset();
        int in = this->GetVariableN(iv)->Get_ndof();

        if (this->GetVariableN(iv)->IsActive()) {
            for (int r = 0; r < in; r++) {
                // GetLog() << "Summing" << result(io+r) << " to " << (*this->K)(kio+r,kio+r) << "\n";
                result(io + r) += (*this->K)(kio + r, kio + r);
            }
        }
        kio += in;
    }
}

void ChKblockGeneric::Build_K(ChSparseMatrix& storage, bool add) {
    if (!K)
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
                        storage.PasteSumClippedMatrix(*K, kio, kjo, in, jn, io, jo);
                    else
                        storage.PasteClippedMatrix(*K, kio, kjo, in, jn, io, jo);
                }

                kjo += jn;
            }
        }

        kio += in;
    }
}

/*
void ChKblockGeneric::StreamOUT(ChStreamOutBinary& mstream)
{
        // class version number
    mstream.VersionWrite(1);

        // serialize parent class too
    ChKblock::StreamOUT(mstream);

        // stream out all member data
    // NOTHING INTERESTING TO SERIALIZE (the Cq jacobians are not so
    // important to waste disk space.. they may be recomputed run-time,
    // and pointers to variables must be rebound in run-time.)
    // mstream << Cq_a;
    // mstream << Cq_b;
}

void ChKblockGeneric::StreamIN(ChStreamInBinary& mstream)
{
        // class version number
    int version = mstream.VersionRead();

        // deserialize parent class too
    ChKblock::StreamIN(mstream);

        // stream in all member data
    // NOTHING INTERESTING TO DESERIALIZE (the Cq jacobians are not so
    // important to waste disk space.. they may be recomputed run-time,
    // and pointers to variables must be rebound in run-time.)
    // mstream >> Cq_a;
    // mstream >> Cq_b;
}
*/

}  // end namespace chrono
