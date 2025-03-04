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

#include <memory>
#include "chrono/core/ChMatrix.h"
#include "chrono/core/ChQuaternion.h"
#include "chrono/core/ChTypes.h"
#include "chrono/core/ChVector3.h"
#include "chrono/fea/ChMesh.h"
#include "chrono/physics/ChSystemSMC.h"
#include "chrono_wood/ChBeamSectionCBLCON.h"
#include "chrono_wood/ChBuilderCBLCON.h"
#include "gtest/gtest.h"

#include "chrono_wood/ChElementCBLCON.h"


using namespace chrono;
using namespace wood;

TEST(CBLConnectorTest, Amatrix) {
    ChElementCBLCON connector;
    ChMatrixNM<double,3,6> A;
    connector.ComputeAmatrix(A, ChVector3d(0.0, 0.0, 0.0), ChVector3d(1.0, 2.0, 3.0));
    ASSERT_DOUBLE_EQ(A(0,0), 1.0);
    ASSERT_DOUBLE_EQ(A(1,1), 1.0);
    ASSERT_DOUBLE_EQ(A(2,2), 1.0);
    ASSERT_DOUBLE_EQ(A(0,4), -3.0);
    ASSERT_DOUBLE_EQ(A(0,5), 2.0);
    ASSERT_DOUBLE_EQ(A(1,3), 3.0);
    ASSERT_DOUBLE_EQ(A(1,5), -1.0);
    ASSERT_DOUBLE_EQ(A(2,3), -2.0);
    ASSERT_DOUBLE_EQ(A(2,4), 1.0);
}

TEST(CBLConnectorTest, compute_strain){
    ChSystemSMC sys;
    auto my_mesh = chrono_types::make_shared<ChMesh>();
    my_mesh->SetAutomaticGravity(false);
    sys.Add(my_mesh);

    auto my_mat = chrono_types::make_shared<ChWoodMaterialVECT>(5e-8,
                                                                0.0, // Zero stiffness to get zero force ?
                                                                0.2372,
                                                                30,
                                                                78.,
                                                                0.2,
                                                                5.0,
                                                                3000,
                                                                120,
                                                                0,
                                                                9900,
                                                                3000,
                                                                3,
                                                                0.5,
                                                                5,
                                                                0.1,
                                                                0.2,
                                                                0.2,
                                                                600,
                                                                1.0); // Not sure what this value of kt should be, not set in Wisdom's demo
    my_mat->SetCoupleMultiplier(1.0);
    my_mat->SetElasticAnalysisFlag(true);

    double length = 2.0;
    ChVector3d posNodeA(0, 0, 0);
    ChVector3d posNodeB(length, 0, 0);
    ChVector3d Ydir(0, 1, 0);
    double facet_area = 1.0;
    double center_from_A = 0.6;
    ChMatrix33<double> facetFrame(ChQuaterniond(1, 0, 0, 0));
    auto my_section = chrono_types::make_shared<ChBeamSectionCBLCON>(my_mat, facet_area, posNodeA + center_from_A * (posNodeB-posNodeA), facetFrame);

    ChBuilderCBLCON builder;
    int num_elem = 1;
    builder.BuildBeam(my_mesh, my_section, num_elem, posNodeA, posNodeB, Ydir);
    auto connector = builder.GetLastBeamElements()[0];
    auto nodeA = connector->GetNodeA();
    auto nodeB = connector->GetNodeB();


    // Performing some kind of step / analysis is required for chrono to run the appropriate setup
    // Functions setupInitial() etc, are all private functions and inaccessible from here, otherwise we use them to make a minimal setup

    // Small deflection can be used since the void ChElementCBLCON::ComputeStrain function only really performs the
    // computation of the strain according to https://doi.org/10.1016/j.cemconcomp.2011.02.011 + TODO: FIND REF FOR CURVATURE
    ChElementCBLCON::LargeDeflection=false;
    ChElementCBLCON::EnableCoupleForces=true;

    sys.DoStepDynamics(0);

    double disp = 0.01;
    double rot = 0.01;
    ChVector3d strain;
    ChVector3d curvature;
    ChVectorN<double, 12> displ_incr;
    displ_incr.setZero(12);

    // Translation of node A
    for (int i = 0; i<3 ; i++) {
        displ_incr(i) = disp; // X, Y, Z direction
        connector->ComputeStrainIncrement(displ_incr, strain, curvature);
        ASSERT_DOUBLE_EQ(strain[i], -disp / length);
        ASSERT_DOUBLE_EQ(strain[(i+1)%3], 0.0);
        ASSERT_DOUBLE_EQ(strain[(i+2)%3], 0.0);
        displ_incr(i) = 0.0;
    }
        // XYZ direction
        displ_incr(0) = displ_incr(1) = displ_incr(2) = disp;
        connector->ComputeStrainIncrement(displ_incr, strain, curvature);
        ASSERT_DOUBLE_EQ(strain[0], -disp / length);
        ASSERT_DOUBLE_EQ(strain[1], -disp / length);
        ASSERT_DOUBLE_EQ(strain[2], -disp / length);
        displ_incr(0) = displ_incr(1) = displ_incr(2) = 0.0;

    // Rotation of node A
    double dir[3][3] = {{0, 0, 0}, {0, 0, 1}, {0, -1, 0}}; // Direction of vector product: rot x (x_A - X_Center)
    for (int i = 0; i<3 ; i++) {
        displ_incr(3+i) = rot; // X, Y, Z direction
        connector->ComputeStrainIncrement(displ_incr, strain, curvature);
        ASSERT_DOUBLE_EQ(strain[0], rot * center_from_A * dir[i][0]);
        ASSERT_DOUBLE_EQ(strain[1], rot * center_from_A * dir[i][1]);
        ASSERT_DOUBLE_EQ(strain[2], rot * center_from_A * dir[i][2]);
        displ_incr(3+i) = 0.0;
    }

    // Translation of node B
    for (int i = 0; i<3 ; i++) {
        displ_incr(6+i) = disp; // X, Y, Z direction
        connector->ComputeStrainIncrement(displ_incr, strain, curvature);
        ASSERT_DOUBLE_EQ(strain[i], disp / length);
        ASSERT_DOUBLE_EQ(strain[(i+1)%3], 0.0);
        ASSERT_DOUBLE_EQ(strain[(i+2)%3], 0.0);
        displ_incr(6+i) = 0.0;
    }
        // XYZ direction
        displ_incr(6) = displ_incr(7) = displ_incr(8) = disp;
        connector->ComputeStrainIncrement(displ_incr, strain, curvature);
        ASSERT_DOUBLE_EQ(strain[0], disp / length);
        ASSERT_DOUBLE_EQ(strain[1], disp / length);
        ASSERT_DOUBLE_EQ(strain[2], disp / length);
        displ_incr(6) = displ_incr(7) = displ_incr(8) = 0.0;

    // Rotation of node B
    // Direction of vector product: rot x (x_B - X_Center) is the same as for node A
    for (int i = 0; i<3 ; i++) {
        displ_incr(9+i) = rot; // X, Y, Z direction
        connector->ComputeStrainIncrement(displ_incr, strain, curvature);
        ASSERT_DOUBLE_EQ(strain[0], rot * (1 - center_from_A) * dir[i][0]);
        ASSERT_DOUBLE_EQ(strain[1], rot * (1 - center_from_A) * dir[i][1]);
        ASSERT_DOUBLE_EQ(strain[2], rot * (1 - center_from_A) * dir[i][2]);
        displ_incr(9+i) = 0.0;
    }

    // // Analytical strain
    // ASSERT_DOUBLE_EQ(mstrain(0), 0.0);
    // ASSERT_DOUBLE_EQ(mstrain(1), 0.0);
    // ASSERT_DOUBLE_EQ(mstrain(2), 0.0);
}

TEST(CBLConnectorTest, update_rotation){
    // TODO when we implement large displacement
}