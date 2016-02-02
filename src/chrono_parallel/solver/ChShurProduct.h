// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All right reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Hammad Mazhar
// =============================================================================
//
// This file contains a base implementation of shur product,
// it can be customzied as needed for different solve stages
// =============================================================================
#pragma once
#include "chrono_parallel/ChDataManager.h"

namespace chrono {

class CH_PARALLEL_API ChShurProduct {
  public:
    ChShurProduct();
    virtual ~ChShurProduct() {}

    virtual void Setup(ChParallelDataManager* data_container_) { data_manager = data_container_; }

    // Perform M_invDx=M^-1*D*x
    void shurA(DynamicVector<real>& x, DynamicVector<real>& out);  // Vector that N is multiplied by
    // Perform the Shur Product
    virtual void operator()(const DynamicVector<real>& x, DynamicVector<real>& AX);

    // Pointer to the system's data manager
    ChParallelDataManager* data_manager;
};

class CH_PARALLEL_API ChShurProductBilateral : public ChShurProduct {
  public:
    ChShurProductBilateral() {}
    virtual ~ChShurProductBilateral() {}
    virtual void Setup(ChParallelDataManager* data_container_);

    // Perform the Shur Product
    virtual void operator()(const DynamicVector<real>& x, DynamicVector<real>& AX);

    CompressedMatrix<real> NshurB;
};
}
