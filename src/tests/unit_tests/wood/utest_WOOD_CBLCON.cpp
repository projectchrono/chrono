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
}



TEST(CBLConnectorTest, elastic_stiffness_matrix){
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
    my_mat->SetCoupleMultiplier(1.0);
    my_mat->SetElasticAnalysisFlag(true);

    // Test input
    double length = 3.0;
    ChVector3d trans(12.0, -3.0, 6.0);
    double rot_angle = -22.0 * CH_DEG_TO_RAD;
    ChVector3d rot_axis(1.0, 1.0, -0.5);
    rot_axis.Normalize(); // Must be normalized
    double facet_area = 0.459;
    double facet_center_from_A = 0.178;

    // Connector setup
    ChQuaterniond q;
    q.SetFromAngleAxis(rot_angle, rot_axis);
    ChMatrix33<double> facetFrame(q);
    ChVector3d posNodeA = q.Rotate(ChVector3d(0.0, 0.0, 0.0)) + trans;
    ChVector3d posNodeB = q.Rotate(ChVector3d(length, 0.0, 0.0)) + trans;
    ChVector3d Ydir = q.Rotate(ChVector3d(0.0, 1.0, 0.0));
    ChVector3d center = posNodeA + facet_center_from_A * (posNodeB-posNodeA);

    auto my_section = chrono_types::make_shared<ChBeamSectionCBLCON>(my_mat, facet_area, center, facetFrame);

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
    connector->ComputeStiffnessMatrix();


    // Analytical calculation of the stiffness matrix
    ChMatrixNM<double, 12, 12> analytical_matrix;

    ChMatrix33<> stiff_local;
    double AEL_N = facet_area * E0 / length;
	double AEL_T = AEL_N * alpha;
    stiff_local << AEL_N , 0.0  , 0.0  ,
                   0.0   , AEL_T, 0.0  ,
                   0.0   , 0.0  , AEL_T;

    // The face frame matrix is stored as the transpose of the rotation matrix:
    // - n on the first row
    // - m on the second row
    // - l on the second row
    // TODO JBC: I think this should eventually be changed as it might be confusing
    // that this is different from all the other "frames" defined as ChMatrix33 in Chrono
    ChMatrix33<> Rf = connector->section->Get_facetFrame().transpose();

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
        // TODO
        // Code below copied from ChElementCBLCON::ComputeStiffnessMatrix for reference
        // Until I get a paper that references that calculation


		// if (ChElementCBLCON::EnableCoupleForces){
		//         //nmL=this->section->Get_facetFrame();
		// 	double multiplier=this->section->Get_material()->GetCoupleMultiplier()*area*E0/length;
		// 	double w=this->section->GetWidth()/2.; 
    	// 		double h=this->section->GetHeight()/2.; 
    	// 		double onethird=1./3.;
    	// 		double rM=w*w*onethird;
    	// 		double rL=h*h*onethird;
    	// 		double rN=rM+rL;
    	// 		//std::cout<<"nmL: "<<nmL<<std::endl;
    	// 		//std::cout<<"rN: "<<rN<<"\t"<<"rM: "<<rM<<"\t"<<"rL: "<<rL<<"\n";
    	// 		//std::cout<<"W: "<<w<<"\t"<<"h: "<<h<<"\n";
    	// 		//std::cout<<"alpha: "<<alpha<<"\t"<<"beta: "<<this->section->Get_material()->GetCoupleMultiplier()<<"\n";
		//         //ChMatrix33<double> rotmat(q_delta);		        
		// 	//nmL=rotmat*nmL;
		// 	//std::cout<<"rot-nmL: "<<nmL<<std::endl;
		// 	ChMatrix33<double> Dmat;
		// 	Dmat.setZero();			
		// 	Dmat(1,1)=rM*multiplier;
		// 	Dmat(2,2)=rL*multiplier;
		// 	Dmat(0,0)=alpha*rN*multiplier;
		// 	//std::cout<<"Dmat: \n"<<Dmat<<std::endl;
		// 	ChMatrix33<double> kmu=nmL.transpose()*Dmat*nmL;
		// 	Km.block<3,3>(3,3)+=kmu;
		// 	Km.block<3,3>(9,3)-=kmu;
		// 	Km.block<3,3>(9,9)+=kmu;
		// 	Km.block<3,3>(3,9)-=kmu;
		// 	//std::cout<<"kmu\n"<<kmu<<std::endl;
		// 	//exit(1);
			
		// }
    // std::cout<< analytical_matrix << std::endl << std::endl << std::endl;
    // std::cout<< connector->Km << std::endl;
    }

    double tol = 1e-10;

    for (int i = 0; i < 12; i++){
        for (int j = 0 ; j < 12 ; j++) {
            ASSERT_NEAR(connector->Km(i, j), analytical_matrix(i, j), tol);
        }
    }
}

TEST(CBLConnectorTest, update_rotation){
    // TODO when we implement large displacement
}
