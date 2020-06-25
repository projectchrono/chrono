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
#include "chrono/physics/ChLoadable.h"
#include "chrono/physics/ChLoad.h"

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


void ChElementGeneric::EleIntLoadResidual_F_gravity(ChVectorDynamic<>& R, const ChVector<>& G_acc, const double c) {
    
    ChVectorDynamic<> mFg(this->GetNdofs());
    this->ComputeGravityForces(mFg, G_acc);
    mFg *= c;


    int stride = 0;
    for (int in = 0; in < this->GetNnodes(); in++) {
        int nodedofs = GetNodeNdofs(in);
        if (!GetNodeN(in)->GetFixed()) {
            for (int j = 0; j < nodedofs; j++)
                //***ATOMIC*** as called from an OMP parallel loop: this is here to avoid race conditions when writing to R
                #pragma omp atomic  
                R(GetNodeN(in)->NodeGetOffset_w() + j) += mFg(stride + j);
        }
        stride += nodedofs;
    }

}

/*
void ChElementGeneric::EleIntLoadResidual_F_gravity(ChVectorDynamic<>& R, const ChVector<>& G_acc, const double c) {
    // (This is a default (VERY UNOPTIMAL) book keeping so that in children classes you can avoid
    // implementing this EleIntLoadResidual_F_gravity function, unless you need faster code.

    // fallback to the previous implementation, that used ChLoaderGravity and Gauss uadrature for 
    // elements inherited from ChLoadableUVW:

    std::shared_ptr<ChElementGeneric> this_wrapper{this, [](ChElementGeneric*){} }; //  A null deleter: this is a hack to wrap "this" raw ptr

    if (auto mloadable = std::dynamic_pointer_cast<ChLoadableUVW>(this_wrapper)) {
        if (G_acc != VNULL) {
                auto common_gravity_loader = chrono_types::make_shared<ChLoad<ChLoaderGravity>>(mloadable);
                common_gravity_loader->loader.Set_G_acc(G_acc);
                common_gravity_loader->loader.SetNumIntPoints(1); //***TODO*** n. gauss points as parameter?
                if (mloadable->GetDensity()) {
                    // temporary set loader target and compute generalized forces term
                    common_gravity_loader->loader.loadable = mloadable;
                    common_gravity_loader->ComputeQ(nullptr, nullptr);
                    common_gravity_loader->LoadIntLoadResidual_F(R, c);
                }
        }
    }
}
*/

void ChElementGeneric::ComputeGravityForces(ChVectorDynamic<>& Fg, const ChVector<>& G_acc) {

    // A default fall-back implementation of the ComputeGravityForces that will work for all elements
    // inherited from ChLoadableUVW and with nonzero GetDensity(). You might override with more efficient implementations.

    Fg.setZero();

    // A null deleter: this is a hack to wrap "this" raw ptr, but we really need this 
    // because the ChLoaderGravity needs it as a shared pointer
    std::shared_ptr<ChElementGeneric> this_wrapper{this, [](ChElementGeneric*){} }; 

    if (auto mloadable = std::dynamic_pointer_cast<ChLoadableUVW>(this_wrapper)) {
        if (G_acc != VNULL) {
                auto common_gravity_loader = chrono_types::make_shared<ChLoad<ChLoaderGravity>>(mloadable);
                common_gravity_loader->loader.Set_G_acc(G_acc);
                common_gravity_loader->loader.SetNumIntPoints(1); //***TODO*** n. gauss points as parameter?
                if (mloadable->GetDensity()) {
                    // temporary set loader target and compute generalized forces term
                    common_gravity_loader->loader.loadable = mloadable;
                    common_gravity_loader->ComputeQ(nullptr, nullptr);
                    Fg = common_gravity_loader->loader.Q;
                }
        }
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
