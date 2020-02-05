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

#include "chrono/fea/ChElementGeneric.h"

namespace chrono {
namespace fea {

void ChElementGeneric::EleIntLoadResidual_F(ChVectorDynamic<>& R, const double c) {
    ChVectorDynamic<> mFi(this->GetNdofs());
    this->ComputeInternalForces(mFi);
    // GetLog() << "EleIntLoadResidual_F , mFi=" << mFi << "  c=" << c << "\n";
    mFi *= c;

    //// RADU
    //// Attention: this is called from within a parallel OMP for loop.
    //// Must use atomic increment when updating the global vector R.

    int stride = 0;
    for (int in = 0; in < this->GetNnodes(); in++) {
        int nodedofs = GetNodeNdofs(in);
        // GetLog() << "  in=" << in << "  stride=" << stride << "  nodedofs=" << nodedofs << " offset=" <<
        // GetNodeN(in)->NodeGetOffset_w() << "\n";
        if (!GetNodeN(in)->GetFixed()) {
            for (int j = 0; j < nodedofs; j++)
#pragma omp atomic
                R(GetNodeN(in)->NodeGetOffset_w() + j) += mFi(stride + j);
        }
        stride += nodedofs;
    }
    // GetLog() << "EleIntLoadResidual_F , R=" << R << "\n";
}

void ChElementGeneric::EleIntLoadResidual_Mv(ChVectorDynamic<>& R, const ChVectorDynamic<>& w, const double c) {
    // This is a default (VERY UNOPTIMAL) book keeping so that in children classes you can avoid
    // implementing this EleIntLoadResidual_Mv function, unless you need faster code)

    ChMatrixDynamic<> mMi(this->GetNdofs(), this->GetNdofs());
    this->ComputeMmatrixGlobal(mMi);

    ChVectorDynamic<> mqi(this->GetNdofs());
    int stride = 0;
    for (int in = 0; in < this->GetNnodes(); in++) {
        int nodedofs = GetNodeNdofs(in);
        if (GetNodeN(in)->GetFixed()) {
            for (int i = 0; i < nodedofs; ++i)
                mqi(stride + i) = 0;
        } else {
            mqi.segment(stride, nodedofs) = w.segment(GetNodeN(in)->NodeGetOffset_w(), nodedofs);
        }
        stride += nodedofs;
    }

    ChVectorDynamic<> mFi = c * mMi * mqi;

    stride = 0;
    for (int in = 0; in < this->GetNnodes(); in++) {
        int nodedofs = GetNodeNdofs(in);
        if (!GetNodeN(in)->GetFixed())
            R.segment(GetNodeN(in)->NodeGetOffset_w(), nodedofs) += mFi.segment(stride, nodedofs);
        stride += nodedofs;
    }
}

void ChElementGeneric::VariablesFbLoadInternalForces(double factor) {
    throw(ChException("ChElementGeneric::VariablesFbLoadInternalForces is deprecated"));
}

void ChElementGeneric::VariablesFbIncrementMq() {
    throw(ChException("ChElementGeneric::VariablesFbIncrementMq is deprecated"));
}

}  // end namespace fea
}  // end namespace chrono
