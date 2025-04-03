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
#include "chrono_wood/ChBasisToolsBeziers.h"
#include "chrono_wood/ChBeamSectionCBLCON.h"
#include "chrono_wood/ChBuilderCBLCON.h"
#include "gtest/gtest.h"

#include "chrono_wood/ChElementCBLCON.h"

#include <chrono>

using Moment = std::chrono::high_resolution_clock::time_point;
using FloatSecs = std::chrono::duration<double>;

inline Moment now()
{
    return std::chrono::high_resolution_clock::now();
}

using namespace chrono;
using namespace wood;

// TODO: Write a Fixture to avoid repeating the code for setting up the test
// TODO: Branch and duplicate test with / without largeDeflection (when implemented) and coupled forces

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
    // CBL expects the facet frame matrix to be stored as the transpose of the rotation matrix:
    // - n on the first row
    // - m on the second row
    // - l on the second row
    // TODO JBC: I think this should eventually be changed as it might be confusing
    // that this is different from all the other "frames" defined as ChMatrix33 in Chrono
    auto my_section = chrono_types::make_shared<ChBeamSectionCBLCON>(my_mat, facet_area, posNodeA + center_from_A * (posNodeB-posNodeA), facetFrame.transpose());

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
}



TEST(CBLConnectorTest, elastic_stiffness_matrix){
    ChSystemSMC sys;
    auto my_mesh = chrono_types::make_shared<ChMesh>();
    my_mesh->SetAutomaticGravity(false);
    sys.Add(my_mesh);

    double E0 = 8000;
    double alpha = 0.2373;
    double couple_multiplier = 0.761;
    auto my_mat = chrono_types::make_shared<ChWoodMaterialVECT>(5e-8,
                                                                E0,
                                                                alpha,
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

    // Small deflection can be used since the void ChElementCBLCON::ComputeStrain function only really performs the
    // computation of the strain according to https://doi.org/10.1016/j.cemconcomp.2011.02.011
    my_mat->SetElasticAnalysisFlag(true);
    ChElementCBLCON::LargeDeflection=false;
    // Bending stiffness computed according to assumptions detailed below
    my_mat->SetCoupleMultiplier(couple_multiplier);
    ChElementCBLCON::EnableCoupleForces=true;

    // Test input
    double length = 3.0;
    ChVector3d trans(12.0, -3.0, 6.0);
    double rot_angle = -22.0 * CH_DEG_TO_RAD;
    ChVector3d rot_axis(1.0, 1.0, -0.5);
    rot_axis.Normalize(); // Must be normalized
    double facet_width = 0.648;
    double facet_height = 0.176;
    double facet_area = facet_height * facet_width;
    double facet_center_from_A = 0.178;

    // Connector setup
    ChQuaterniond q;
    q.SetFromAngleAxis(rot_angle, rot_axis);
    ChMatrix33<double> facetFrame(q);
    ChVector3d posNodeA = q.Rotate(ChVector3d(0.0, 0.0, 0.0)) + trans;
    ChVector3d posNodeB = q.Rotate(ChVector3d(length, 0.0, 0.0)) + trans;
    ChVector3d Ydir = q.Rotate(ChVector3d(0.0, 1.0, 0.0));
    ChVector3d center = posNodeA + facet_center_from_A * (posNodeB-posNodeA);

    // CBL expects the facet frame matrix to be stored as the transpose of the rotation matrix:
    // - n on the first row
    // - m on the second row
    // - l on the second row
    // TODO JBC: I think this should eventually be changed as it might be confusing
    // that this is different from all the other "frames" defined as ChMatrix33 in Chrono
    auto my_section = chrono_types::make_shared<ChBeamSectionCBLCON>(my_mat, facet_area, center, facetFrame.transpose());
    my_section->SetHeight(facet_height);
    my_section->SetWidth(facet_width);

    ChBuilderCBLCON builder;
    int num_elem = 1;
    builder.BuildBeam(my_mesh, my_section, num_elem, posNodeA, posNodeB, Ydir);
    auto connector = builder.GetLastBeamElements()[0];

    // Performing some kind of step / analysis is required for chrono to run the appropriate setup
    // Functions setupInitial() etc, are all private functions and inaccessible from here, otherwise we use them to make a minimal setup
    sys.DoStepDynamics(0);

    // Chrono-CBL calculation of the stiffness matrix
    connector->ComputeStiffnessMatrix();


    // Analytical calculation of the stiffness matrix
    ChMatrixNM<double, 12, 12> analytical_matrix;

    ChMatrix33<> stiff_local;
    stiff_local << facet_area * E0 / length , 0.0                             , 0.0                             ,
                   0.0                      , facet_area * E0 * alpha / length, 0.0                             ,
                   0.0                      , 0.0                             , facet_area * E0 * alpha / length;

    ChMatrix33<> Rf = facetFrame;

    ChVector3<> xc_xi = center - posNodeA;
    ChVector3<> xc_xj = center - posNodeB;
    ChMatrixNM<double,3,6> Ai, Aj;
    Ai << 1.0, 0.0, 0.0, 0.0      ,  xc_xi[2], -xc_xi[1],
          0.0, 1.0, 0.0, -xc_xi[2],  0.0     ,  xc_xi[0],
          0.0, 0.0, 1.0,  xc_xi[1], -xc_xi[0],  0.0     ;
    Aj << 1.0, 0.0, 0.0, 0.0      ,  xc_xj[2], -xc_xj[1],
          0.0, 1.0, 0.0, -xc_xj[2],  0.0     ,  xc_xj[0],
          0.0, 0.0, 1.0,  xc_xj[1], -xc_xj[0],  0.0     ;

    // Stiffness from facet stresses
    // Chain rule Derivatives of (12) (13) from https://doi.org/10.1016/j.cemconcomp.2011.02.011
    analytical_matrix.block<6,6>(0,0) =  Ai.transpose() * Rf * stiff_local * Rf.transpose() * Ai;
    analytical_matrix.block<6,6>(0,6) = -Ai.transpose() * Rf * stiff_local * Rf.transpose() * Aj;
    analytical_matrix.block<6,6>(6,0) = -Aj.transpose() * Rf * stiff_local * Rf.transpose() * Ai;
    analytical_matrix.block<6,6>(6,6) =  Aj.transpose() * Rf * stiff_local * Rf.transpose() * Aj;

    // Stiffness from facet bending
    if (ChElementCBLCON::EnableCoupleForces) {
        // Current bending stiffness calculations assume:
        // - Linear shape functions for the angles
        // - No offset of the section centroid
        // - Section height/width in direction of 2nd/3rd axis M/L of local frame, respectively
        // - Polar moment of area for rectangular cross section
        double I_M = facet_height * (facet_width * facet_width * facet_width) / 12.0;
        double I_L = facet_width * (facet_height * facet_height * facet_height) / 12.0;
        double I_N = I_M + I_L;
        ChMatrix33<> stiff_bending_local;
        stiff_bending_local << E0 * alpha * I_N / length  , 0.0               , 0.0               ,
                               0.0                         , E0 * I_M / length, 0.0               ,
                               0.0                         , 0.0               , E0 * I_L / length;
        stiff_bending_local *= couple_multiplier;
        ChMatrix33<> analytical_bending_block = Rf * stiff_bending_local * Rf.transpose();

        analytical_matrix.block<3,3>(3,3) += analytical_bending_block;
        analytical_matrix.block<3,3>(3,9) -= analytical_bending_block;
        analytical_matrix.block<3,3>(9,3) -= analytical_bending_block;
        analytical_matrix.block<3,3>(9,9) += analytical_bending_block;
    }

    double tol = 1e-10;

    for (int i = 0; i < 12; i++){
        for (int j = 0 ; j < 12 ; j++) {
            ASSERT_NEAR(connector->Km(i, j), analytical_matrix(i, j), tol);
        }
    }
}


TEST(CBLConnectorTest, internal_forces){
    ChSystemSMC sys;
    auto my_mesh = chrono_types::make_shared<ChMesh>();
    my_mesh->SetAutomaticGravity(false);
    sys.Add(my_mesh);

    double E0 = 8000;
    double alpha = 0.2373;

    auto my_mat = chrono_types::make_shared<ChWoodMaterialVECT>(5e-8,
                                                                E0,
                                                                alpha,
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

    // Computing internal forces requires computing the stress
    // To have simple analytical results for this test simple, we enforce linear elasticity
    my_mat->SetElasticAnalysisFlag(true);
    // TODO JBC: Not sure what this does yet
    my_mat->SetCoupleMultiplier(0.0);

    // Test input
    double length = 3.0;
    ChVector3d trans(12.0, -3.0, 6.0);
    double rot_angle = -22.0 * CH_DEG_TO_RAD;
    ChVector3d rot_axis(1.0, 1.0, -0.5);
    rot_axis.Normalize(); // Must be normalized
    double facet_width = 0.648;
    double facet_height = 0.176;
    double facet_area = facet_height * facet_width;
    double facet_center_from_A = 0.178;

    // Connector setup
    ChQuaterniond q;
    q.SetFromAngleAxis(rot_angle, rot_axis);
    ChMatrix33<double> facetFrame(q);
    ChVector3d posNodeA = q.Rotate(ChVector3d(0.0, 0.0, 0.0)) + trans;
    ChVector3d posNodeB = q.Rotate(ChVector3d(length, 0.0, 0.0)) + trans;
    ChVector3d Ydir = q.Rotate(ChVector3d(0.0, 1.0, 0.0));
    ChVector3d center = posNodeA + facet_center_from_A * (posNodeB-posNodeA);

    // CBL expects the facet frame matrix to be stored as the transpose of the rotation matrix:
    // - n on the first row
    // - m on the second row
    // - l on the second row
    // TODO JBC: I think this should eventually be changed as it might be confusing
    // that this is different from all the other "frames" defined as ChMatrix33 in Chrono
    auto my_section = chrono_types::make_shared<ChBeamSectionCBLCON>(my_mat, facet_area, center, facetFrame.transpose());
    my_section->SetHeight(facet_height);
    my_section->SetWidth(facet_width);

    ChBuilderCBLCON builder;
    int num_elem = 1;
    builder.BuildBeam(my_mesh, my_section, num_elem, posNodeA, posNodeB, Ydir);
    auto connector = builder.GetLastBeamElements()[0];

    // Performing some kind of step / analysis is required for chrono to run the appropriate setup
    // Functions setupInitial() etc, are all private functions and inaccessible from here, otherwise we use them to make a minimal setup

    // Small deflection can be used since the void ChElementCBLCON::ComputeStrain function only really performs the
    // computation of the strain according to https://doi.org/10.1016/j.cemconcomp.2011.02.011 + TODO: FIND REF FOR CURVATURE
    ChElementCBLCON::LargeDeflection=false;
    ChElementCBLCON::EnableCoupleForces=false; // TODO: make it work with and without couple Forces

    sys.DoStepDynamics(0);

    // Displace node B to cause imposed local strain
    ChVector3d imposed_local_strain(0.398, -0.67, 0.495);
    ChVectorDynamic<> pos1;
    connector->GetNodeB()->SetPos(posNodeB + length * (facetFrame * imposed_local_strain));

    // TODO JBC: Perform the test for the bending moments once we have a reference for curvature calculations
    // Rotate node B to cause curvature


    // Chrono CBL calculation of the internal forces
    ChVectorDynamic<> Fi;
    connector->ComputeInternalForces(Fi);


    // Analytical calculation of the internal forces
    // Equation (12) (13) of https://doi.org/10.1016/j.cemconcomp.2011.02.011
    ChVectorN<double, 12> Fi_analytical;
    ChVector3<> stress = imposed_local_strain * E0 * ChVector3d(1.0, alpha, alpha);
    ChVector3<> xc_xi = center - posNodeA;
    ChVector3<> xc_xj = center - posNodeB;
    ChMatrixNM<double,3,6> Ai, Aj;
    Ai << 1.0, 0.0, 0.0, 0.0      ,  xc_xi[2], -xc_xi[1],
          0.0, 1.0, 0.0, -xc_xi[2],  0.0     ,  xc_xi[0],
          0.0, 0.0, 1.0,  xc_xi[1], -xc_xi[0],  0.0     ;
    Aj << 1.0, 0.0, 0.0, 0.0      ,  xc_xj[2], -xc_xj[1],
          0.0, 1.0, 0.0, -xc_xj[2],  0.0     ,  xc_xj[0],
          0.0, 0.0, 1.0,  xc_xj[1], -xc_xj[0],  0.0     ;
    ChVectorN<double, 6> Bn_i_tr, Bm_i_tr, Bl_i_tr;
	ChVectorN<double, 6> Bn_j_tr, Bm_j_tr, Bl_j_tr;
    Bn_i_tr = Ai.transpose() * facetFrame.GetAxisX().eigen() / length;
    Bm_i_tr = Ai.transpose() * facetFrame.GetAxisY().eigen() / length;
    Bl_i_tr = Ai.transpose() * facetFrame.GetAxisZ().eigen() / length;
    Bn_j_tr = Aj.transpose() * facetFrame.GetAxisX().eigen() / length;
    Bm_j_tr = Aj.transpose() * facetFrame.GetAxisY().eigen() / length;
    Bl_j_tr = Aj.transpose() * facetFrame.GetAxisZ().eigen() / length;
    Fi_analytical.segment(0,6) = - length * facet_area * (stress[0] * Bn_i_tr + stress[1] * Bm_i_tr + stress[2] * Bl_i_tr);
    Fi_analytical.segment(6,6) =   length * facet_area * (stress[0] * Bn_j_tr + stress[1] * Bm_j_tr + stress[2] * Bl_j_tr);


    Fi_analytical *= -1.0;
    // // Stiffness from facet bending
    // if (ChElementCBLCON::EnableCoupleForces) {
    //     // TODO
    //     // Code below copied from ChElementCBLCON::ComputeInternalForces for reference
    //     // Until I get a paper that references that calculation
    // }


    double tol = 1e-10;
    for (int i = 0; i < 12; i++) {
        ASSERT_NEAR(Fi(i), Fi_analytical(i), tol);
    }
}

TEST(CBLConnectorTest, update_rotation){
    // TODO when we implement large displacement
}
