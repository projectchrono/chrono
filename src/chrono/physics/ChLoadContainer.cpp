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

#include "chrono/physics/ChLoadContainer.h"

namespace chrono {

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChLoadContainer)

ChLoadContainer::ChLoadContainer(const ChLoadContainer& other) : ChPhysicsItem(other) {
    loadlist = other.loadlist;
}

void ChLoadContainer::Add(std::shared_ptr<ChLoadBase> newload) {
    //TODO Radu: I don't think find can be used on a container of shared pointers which does not support the == operator.
    //TODO Radu: check if this is still true, now that we switched to std::shared_ptr

    //assert(std::find<std::vector<std::shared_ptr<ChLoadBase>>::iterator>(loadlist.begin(), loadlist.end(), newload)
    ///== loadlist.end());
    loadlist.push_back(newload);
}

void ChLoadContainer::Update(double mytime, bool update_assets) {
    for (size_t i = 0; i < loadlist.size(); ++i) {
        loadlist[i]->Update(mytime);
    }
    // Overloading of base class:
    ChPhysicsItem::Update(mytime, update_assets);
}

void ChLoadContainer::IntLoadResidual_F(const unsigned int off,  // offset in R residual
                                        ChVectorDynamic<>& R,    // result: the R residual, R += c*F
                                        const double c           // a scaling factor
                                        ) {
    for (size_t i = 0; i < loadlist.size(); ++i) {
        loadlist[i]->LoadIntLoadResidual_F(R, c);
    }
}

void ChLoadContainer::IntLoadResidual_Mv(const unsigned int off,      ///< offset in R residual
                                   ChVectorDynamic<>& R,        ///< result: the R residual, R += c*M*v
                                   const ChVectorDynamic<>& w,  ///< the w vector
                                   const double c               ///< a scaling factor
                                   ) {
    for (size_t i = 0; i < loadlist.size(); ++i) {
        loadlist[i]->LoadIntLoadResidual_Mv(R, w, c);
    }
}

void ChLoadContainer::InjectKRMmatrices(ChSystemDescriptor& mdescriptor) {
    for (size_t i = 0; i < loadlist.size(); ++i) {
        loadlist[i]->InjectKRMmatrices(mdescriptor);
    }
}

void ChLoadContainer::KRMmatricesLoad(double Kfactor, double Rfactor, double Mfactor) {
    for (size_t i = 0; i < loadlist.size(); ++i) {
        loadlist[i]->KRMmatricesLoad(Kfactor, Rfactor, Mfactor);
    }
}

void ChLoadContainer::ArchiveOut(ChArchiveOut& marchive) {
    //***TODO***
}

void ChLoadContainer::ArchiveIn(ChArchiveIn& marchive) {
    //***TODO***
}

}  // end namespace chrono
