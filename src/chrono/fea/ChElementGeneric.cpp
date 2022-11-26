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
    ChVectorDynamic<> Fi(GetNdofs());
    ComputeInternalForces(Fi);
    Fi *= c;

    //// Attention: this is called from within a parallel OMP for loop.
    //// Must use atomic increment when updating the global vector R.

    int stride = 0;
    for (int in = 0; in < GetNnodes(); in++) {
        int node_dofs = GetNodeNdofs_active(in);
        if (!GetNodeN(in)->IsFixed()) {
            for (int j = 0; j < node_dofs; j++)
#pragma omp atomic
                R(GetNodeN(in)->NodeGetOffsetW() + j) += Fi(stride + j);
        }
        stride += GetNodeNdofs(in);
    }
    // GetLog() << "EleIntLoadResidual_F , R=" << R << "\n";
}

void ChElementGeneric::EleIntLoadResidual_Mv(ChVectorDynamic<>& R, const ChVectorDynamic<>& w, const double c) {
    ChMatrixDynamic<> Mi(GetNdofs(), GetNdofs());
    ComputeMmatrixGlobal(Mi);

    ChVectorDynamic<> mqi(GetNdofs());
    mqi.setZero();
    int stride = 0;
    for (int in = 0; in < GetNnodes(); in++) {
        int node_dofs = GetNodeNdofs_active(in);
        if (!GetNodeN(in)->IsFixed()) {
            mqi.segment(stride, node_dofs) = w.segment(GetNodeN(in)->NodeGetOffsetW(), node_dofs);
        }
        stride += GetNodeNdofs(in);
    }

    ChVectorDynamic<> Fi = c * Mi * mqi;

    stride = 0;
    for (int in = 0; in < GetNnodes(); in++) {
        int node_dofs = GetNodeNdofs_active(in);
        if (!GetNodeN(in)->IsFixed())
            R.segment(GetNodeN(in)->NodeGetOffsetW(), node_dofs) += Fi.segment(stride, node_dofs);
        stride += GetNodeNdofs(in);
    }
}

void ChElementGeneric::EleIntLoadResidual_F_gravity(ChVectorDynamic<>& R, const ChVector<>& G_acc, const double c) {
    ChVectorDynamic<> Fg(GetNdofs());
    ComputeGravityForces(Fg, G_acc);
    Fg *= c;

    //// Attention: this is called from within a parallel OMP for loop.
    //// Must use atomic increment when updating the global vector R.

    int stride = 0;
    for (int in = 0; in < GetNnodes(); in++) {
        int node_dofs = GetNodeNdofs_active(in);
        if (!GetNodeN(in)->IsFixed()) {
            for (int j = 0; j < node_dofs; j++)
#pragma omp atomic
                R(GetNodeN(in)->NodeGetOffsetW() + j) += Fg(stride + j);
        }
        stride += GetNodeNdofs(in);
    }
}

// A default fall-back implementation of the ComputeGravityForces that will work for all elements inherited from
// ChLoadableUVW and with nonzero GetDensity().
void ChElementGeneric::ComputeGravityForces(ChVectorDynamic<>& Fg, const ChVector<>& G_acc) {
    Fg.setZero();

    // A null deleter: this is a hack to wrap "this" raw ptr, because the ChLoaderGravity needs it as a shared pointer
    std::shared_ptr<ChElementGeneric> this_wrapper{this, [](ChElementGeneric*) {}};

    if (auto loadable = std::dynamic_pointer_cast<ChLoadableUVW>(this_wrapper)) {
        if (G_acc != VNULL) {
            auto common_gravity_loader = chrono_types::make_shared<ChLoad<ChLoaderGravity>>(loadable);
            common_gravity_loader->loader.Set_G_acc(G_acc);
            common_gravity_loader->loader.SetNumIntPoints(1);  //// TODO n. gauss points as parameter?
            if (loadable->GetDensity()) {
                // temporary set loader target and compute generalized forces term
                common_gravity_loader->loader.loadable = loadable;
                common_gravity_loader->ComputeQ(nullptr, nullptr);
                Fg = common_gravity_loader->loader.Q;
            }
        }
    }
}

void ChElementGeneric::ComputeMmatrixGlobal(ChMatrixRef M) {
    ComputeKRMmatricesGlobal(M, 0, 0, 1.0);
}

void ChElementGeneric::InjectKRMmatrices(ChSystemDescriptor& descriptor) {
    descriptor.InsertKblock(&Kmatr);
}

void ChElementGeneric::KRMmatricesLoad(double Kfactor, double Rfactor, double Mfactor) {
    ComputeKRMmatricesGlobal(Kmatr.Get_K(), Kfactor, Rfactor, Mfactor);
}

void ChElementGeneric::VariablesFbLoadInternalForces(double factor) {
    throw(ChException("ChElementGeneric::VariablesFbLoadInternalForces is deprecated"));
}

void ChElementGeneric::VariablesFbIncrementMq() {
    throw(ChException("ChElementGeneric::VariablesFbIncrementMq is deprecated"));
}

}  // end namespace fea
}  // end namespace chrono
