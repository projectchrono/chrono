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
// Authors: Jibril B. Coulibaly
// =============================================================================

#include <cmath>
#include <memory>
#include "chrono/core/ChMatrix.h"
#include "chrono/core/ChMatrix33.h"
#include "chrono/core/ChQuaternion.h"
#include "chrono/core/ChTypes.h"
#include "chrono/core/ChVector3.h"
#include "chrono/fea/ChMesh.h"
#include "chrono/physics/ChSystemSMC.h"
#include "chrono_wood/ChBeamSectionCBLCON.h"
#include "chrono_wood/ChBuilderCBLCON.h"
#include "gtest/gtest.h"

#include "chrono_wood/ChWoodMaterialVECT.h"

#include <chrono>

using namespace chrono;
using namespace wood;

// TODO: Write a Fixture to avoid repeating the code for setting up the test

TEST(WoodMaterialVECTTest, stress_no_eigenstrain){
    double rho = 5e-8;
    double E0 = 8000;
    double alpha = 0.2373;
    double sigmat = 30.0;
    double sigmac = 120.0;
    double sigmas = 78.0;
    double nt = 0.2;   
    double lt = 5.0;
    double rs = 0.0;
    double couple_multiplier = 0.761;


    
    auto my_mat = chrono_types::make_shared<ChWoodMaterialVECT>();
    my_mat->Set_density(rho);
    my_mat->Set_E0(E0);
    my_mat->Set_alpha(alpha);
    my_mat->Set_sigmat(sigmat);
    my_mat->Set_sigmac(sigmac);
    my_mat->Set_sigmas(sigmas);
    my_mat->Set_nt(nt);
    my_mat->Set_lt(lt);
    my_mat->Set_rs(rs);
    my_mat->SetCoupleMultiplier(couple_multiplier);

    // TODO JBC: test skeleton, not finished, not supposed to work
    double length = 3.0;
    double facet_width = 3.165;
    double facet_height = 12.4864;
    double facet_area = facet_height * facet_width;
    double epsv = 0;
    ChVectorN<double, 18> statev;
    statev.setZero();
    ChVector3d strain(4.186, 1.16, -86.48);
    ChVector3d curvature(0.123, -0.864, 0.793);

    // CBL stress calculation
    ChVector3d stress;
    ChVector3d couple;
    double random_field = 1.0; // NO RANDOM FIELD
    my_mat->ComputeStress(strain,curvature, length, epsv, statev, facet_area, facet_width, facet_height, random_field, stress, couple);

}

