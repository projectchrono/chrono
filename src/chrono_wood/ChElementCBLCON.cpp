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
// Authors: Erol Lale, Jibril B. Coulibaly
// =============================================================================
// Class for CBLCON elements:  
//
//  i)   Internal forces
//  ii)  Stiffness matrix
//  iii) Mass matrix  
//  iv)  Body forces
//
// Formulation of the CBLCON element can be found in: https://doi.org/10.1016/j.cemconcomp.2011.02.011
// =============================================================================

//#define BEAM_VERBOSE

#include "chrono_wood/ChElementCBLCON.h"
#include <Eigen/src/Core/DiagonalMatrix.h>
#include <cstdlib>
#include <iomanip>
#include <ostream>
#include "chrono/core/ChMatrix.h"
#include "chrono/core/ChMatrix33.h"
#include "chrono/core/ChVector3.h"

namespace chrono {
namespace wood {

bool ChElementCBLCON::LargeDeflection=true;
bool ChElementCBLCON::EnableCoupleForces=true;
bool ChElementCBLCON::LumpedMass=false;



ChElementCBLCON::ChElementCBLCON()
    : q_refrotA(QUNIT),
      q_refrotB(QUNIT),
      q_element_abs_rot(QUNIT),
      q_element_ref_rot(QUNIT),
      disable_corotate(false),
      force_symmetric_stiffness(false),
      use_geometric_stiffness(false)
{
    nodes.resize(2);

    Km.setZero(this->GetNumCoordsPosLevel(), this->GetNumCoordsPosLevel());
    Kg.setZero(this->GetNumCoordsPosLevel(), this->GetNumCoordsPosLevel());
}

void ChElementCBLCON::SetNodes(std::shared_ptr<ChNodeFEAxyzrot> nodeA, std::shared_ptr<ChNodeFEAxyzrot> nodeB) {
    assert(nodeA);
    assert(nodeB);

    nodes[0] = nodeA;
    nodes[1] = nodeB;
    std::vector<ChVariables*> mvars;
    mvars.push_back(&nodes[0]->Variables());
    mvars.push_back(&nodes[1]->Variables());
    Kmatr.SetVariables(mvars);
}

void ChElementCBLCON::ShapeFunctions(ShapeVector& N, double eta) {
    double Nx1 = (1. / 2.) * (1 - eta);
    double Nx2 = (1. / 2.) * (1 + eta);
    double Ny1 = (1. / 4.) * pow((1 - eta), 2) * (2 + eta);
    double Ny2 = (1. / 4.) * pow((1 + eta), 2) * (2 - eta);
    double Nr1 = (this->length / 8.) * pow((1 - eta), 2) * (1 + eta);
    double Nr2 = (this->length / 8.) * pow((1 + eta), 2) * (eta - 1);
    /*
    N(0) = Nx1;
    N(1) = Ny1;
    N(2) = Ny1;
    N(3) = Nx1;
    N(4) = -Nr1;
    N(5) = Nr1;
    N(6) = Nx2;
    N(7) = Ny2;
    N(8) = Ny2;
    N(9) = Nx2;
    N(10) = -Nr2;
    N(11) = Nr2;
    */
    double dN_ua = (1. / (2. * this->length)) * (-3. + 3 * eta * eta);
    double dN_ub = (1. / (2. * this->length)) * (3. - 3 * eta * eta);
    double dN_ra = (1. / 4.) * (-1. - 2 * eta + 3 * eta * eta);
    double dN_rb = -(1. / 4.) * (1. - 2 * eta - 3 * eta * eta);
    N(0) = Nx1;
    N(1) = Ny1;
    N(2) = Nr1;
    N(3) = Nx2;
    N(4) = Ny2;
    N(5) = Nr2;
    N(6) = dN_ua;
    N(7) = dN_ub;
    N(8) = dN_ra;
    N(9) = dN_rb;
}


void ChElementCBLCON::Update() {
    // parent class update:
    ChElementGeneric::Update();

    // always keep updated the rotation matrix A:
    // TODO JBC: Because we inherit from a corotational element, I think we should keep that updated
    //  and use the disable_corotational on top / instead of LargeDeflection flag we defiend here
	if (ChElementCBLCON::LargeDeflection){
		this->UpdateRotation();
		this->length = (nodes[1]->GetPos() - nodes[0]->GetPos()).Length();
	}	
	//
	if(this->macro_strain){		
		this->section->ComputeEigenStrain(this->macro_strain);
	}
    // get displacement increment between current step and previous step
    // update total displacement increment 
    //ChVectorDynamic<> displ(12);
    //this->GetStateBlock(displ);
    //dofs_increment=displ-dofs_old;
    //dofs_old=displ;
}

void ChElementCBLCON::UpdateRotation() {
    ChMatrix33<> A0(this->q_element_ref_rot);	
    ChMatrix33<> Aabs;
    if (!LargeDeflection) {
        Aabs = A0;
        q_element_abs_rot = q_element_ref_rot;
    } else {
    	/*
        ChVector3d mXele_w = nodes[1]->Frame().GetPos() - nodes[0]->Frame().GetPos();
        // propose Y_w as absolute dir of the Y axis of A node, removing the effect of Aref-to-A rotation if any:
        //    Y_w = [R Aref->w ]*[R Aref->A ]'*{0,1,0}
        ChVector3d myele_wA = nodes[0]->Frame().GetRot().Rotate(q_refrotA.RotateBack(ChVector3d(0, 1, 0)));
        // propose Y_w as absolute dir of the Y axis of B node, removing the effect of Bref-to-B rotation if any:
        //    Y_w = [R Bref->w ]*[R Bref->B ]'*{0,1,0}
        ChVector3d myele_wB = nodes[1]->Frame().GetRot().Rotate(q_refrotB.RotateBack(ChVector3d(0, 1, 0)));
        // Average the two Y directions to have midpoint torsion (ex -30?torsion A and +30?torsion B= 0?
        ChVector3d myele_w = (myele_wA + myele_wB).GetNormalized();
        Aabs.SetFromAxisX(mXele_w, myele_w);
        q_element_abs_rot = Aabs.GetQuaternion();
        */
        
        // TODO JBC: I think the code below does not work for large displacement.
        // the Y axis of the nodes are not necessarily aligned (even on average) with the y axis of the beam
        // This approach may rotate the L and M directions of the facet by the wrong amount,
        // causing the strain increment not to be added in the correct direction
        // I believe the commented code above for beams should be used instead
    	ChVector3d mXele = nodes[1]->Frame().GetPos() - nodes[0]->Frame().GetPos();
    	ChVector3d myele =
        (nodes[0]->Frame().GetRotMat().GetAxisY() + nodes[1]->Frame().GetRotMat().GetAxisY()).GetNormalized();
    	Aabs.SetFromAxisX(mXele, myele);
    	q_element_abs_rot = Aabs.GetQuaternion();
    	
    	
    	
    }

    //this->A = A0.transpose() * Aabs;
}





void ChElementCBLCON::GetStateBlock(ChVectorDynamic<>& mD) {
    mD.resize(12);

    // Node displacement
    // If large deflection disabled, return displacement in global frame
    // TODO: Go back to this function when we decide how to enable and formulate large deflection
    auto getDisplacement = [&](std::shared_ptr<ChNodeFEAxyzrot> node) {
        ChVector3d local_disp;
        if (!LargeDeflection)
            local_disp = node->Frame().GetPos() - node->GetX0().GetPos();
        else
            local_disp = node->Frame().GetPos() - node->GetX0().GetPos(); // TODO: temporary
        return local_disp;
    };
    
    // Node rotations
    // If Large deflection disabled, return rotation in global frame
    // TODO: Go back to this function when we decide how to enable and formulate large deflection
    auto getRotation = [&](std::shared_ptr<ChNodeFEAxyzrot> node) {
        ChVector3d delta_rot_dir;
        double delta_rot_angle;
        if (!LargeDeflection)
            node->Frame().GetRot().GetAngleAxis(delta_rot_angle, delta_rot_dir);
        else
            node->Frame().GetRot().GetAngleAxis(delta_rot_angle, delta_rot_dir); // TODO: temporary
        // TODO: GetAngleAxis already returns in the range [-PI..PI], no need to change range
        // TODO: the previous range was only shifted if delta_rot_angle > CH_PI. How about delta_rot_angle < - CH_PI ? Was that a bug?
        // TODO: We may want to use GetRotVec directly, but the angle is not within [-PI .. PI]
        // TODO: Consider changing GetRotVec() to return within [-PI .. PI]
        return delta_rot_angle * delta_rot_dir;
    };
    
    mD.segment(0, 3) = getDisplacement(nodes[0]).eigen();
    mD.segment(3, 3) = getRotation(nodes[0]).eigen();

    mD.segment(6, 3) = getDisplacement(nodes[1]).eigen();
    mD.segment(9, 3) = getRotation(nodes[1]).eigen();
}


void ChElementCBLCON::GetField_dt(ChVectorDynamic<>& mD_dt) {
    mD_dt.resize(12);

    // Node 0, velocity (in local element frame, corotated back by A' )
    mD_dt.segment(0, 3) = q_element_abs_rot.RotateBack(nodes[0]->Frame().GetPosDt()).eigen();

    // Node 0, x,y,z ang.velocity (in local element frame, corotated back by A' )
    mD_dt.segment(3, 3) = q_element_abs_rot.RotateBack(nodes[0]->Frame().GetAngVelParent ()).eigen();

    // Node 1, velocity (in local element frame, corotated back by A' )
    mD_dt.segment(6, 3) = q_element_abs_rot.RotateBack(nodes[1]->Frame().GetPosDt()).eigen();

    // Node 1, x,y,z ang.velocity (in local element frame, corotated back by A' )
    mD_dt.segment(9, 3) = q_element_abs_rot.RotateBack(nodes[1]->Frame().GetAngVelParent ()).eigen();
}


void ChElementCBLCON::ComputeStrainIncrement(ChVectorN<double, 12>& displ_incr, ChVector3d& mstrain, ChVector3d& curvature) {
	// 
    ChVector3d ui = displ_incr.segment(0,3);
    ChVector3d ri = displ_incr.segment(3,3);
    ChVector3d uj = displ_incr.segment(6,3);
    ChVector3d rj = displ_incr.segment(9,3);
    ChVector3d xc_xi;
    ChVector3d xc_xj;
    // For small deflection, use the initial position of the nodes and facet centers
    // to determine the displacement induced by the node rotation
    if (!LargeDeflection) {
        xc_xi = this->section->Get_center() - this->nodes[0]->GetX0().GetPos();
        xc_xj = this->section->Get_center() - this->nodes[1]->GetX0().GetPos();
    }
    else { // TODO: Find out how to evolve position of the center for large displacement
        xc_xi = this->section->Get_center() - this->nodes[0]->GetX0().GetPos(); // TEMPORARY
        xc_xj = this->section->Get_center() - this->nodes[1]->GetX0().GetPos(); // TEMPORARY
    }

	ChMatrix33<double> nmL=this->section->Get_facetFrame();
	if(ChElementCBLCON::LargeDeflection){	// TODO: JBC: I think that should go into Update()
		ChQuaternion<> q_delta=(this->q_element_abs_rot *  this->q_element_ref_rot.GetConjugate());
		for (int id=0; id<3; id++){				
			nmL.block<1,3>(id,0)=q_delta.Rotate(nmL.block<1,3>(id,0)).eigen();
		}	
	}

    // Strain
    ChVector3d strain_increment = (uj + rj.Cross(xc_xj) - (ui + ri.Cross(xc_xi))) / this->length;
    mstrain = nmL * strain_increment;

    // Curvature
	if (ChElementCBLCON::EnableCoupleForces){
		curvature = nmL * (rj - ri) / this->length;
	}
}



// TODO: this is not used consider deleting
void ChElementCBLCON::ComputeStress(ChVector3d& mstress) {
	ChVector3d mstrain;
	ChVector3d curvature;
    // Displacement and rotation increment of nodes:
	this->ComputeStrainIncrement(dofs_increment, mstrain, curvature);
	//std::cout<<"\nmstrain:\n"<<mstrain<<std::endl;	
	//
	double E0=this->section-> Get_material()->Get_E0();
	double alpha=this->section-> Get_material()->Get_alpha();	
	//
	double epsQ=pow(mstrain[0]*mstrain[0]+alpha*(mstrain[1]*mstrain[1]+mstrain[2]*mstrain[2]), 0.5);
	double strsQ=E0*epsQ;	
	//
	if (epsQ!=0) {
		mstress[0]=strsQ*mstrain[0]/epsQ;
		mstress[1]=alpha*strsQ*mstrain[1]/epsQ;
		mstress[2]=alpha*strsQ*mstrain[2]/epsQ;
	}else{
		mstress.Set(0.0);
	}	
}


void ChElementCBLCON::ComputeMmatrixGlobal(ChMatrixRef M) {
    // Mass Matrix

    // TODO JBC: Consider making the lumped matrix with L/2 on each node and no extra-diagonal un lower-left block for simplicity
    // no need to compute the nodes-to-center vectors and distances
    ChVector3d pNA;
    ChVector3d pNB;
    ChVector3d pC;

    if (!ChElementCBLCON::LargeDeflection) {
        pNA = this->GetNodeA()->GetX0().GetPos();
        pNB = this->GetNodeB()->GetX0().GetPos();
        pC = this->section->Get_center();
    } else {
        // TODO JBC: Technically, if the connector is stretched, at constant mass, the moment of inertia changes
        // Decide whether or not we want to take this into account in the calculation of the length or ignored it. error ~ 2*max_strain
        // Currently use the same length for simplicity
        pNA = this->GetNodeA()->GetX0().GetPos(); // TEMPORARY
        pNB = this->GetNodeB()->GetX0().GetPos(); // TEMPORARY
        pC = this->section->Get_center(); // TEMPORARY
    }
    double LA=(pC-pNA).Length();
    double LB=(pC-pNB).Length(); // TODO JBC: This could be length - LA, but will likely change when we refactor the center from a Vector into a scalar along the line

    // // TODO JBC:
    // // intermediate results of calculate_MI could be stored in the section, similar to other section models with inertia
    // // content of calculate_MI could be done directly in here
    // // There should be an option to use a lumped mass matrix, in which case some of the expensive rotations might be avoided


    M.setZero();

    double w=this->section->GetWidth();
    double h=this->section->GetHeight();
    double Area=this->section->Get_area();
    double rho=this->section->Get_material()->Get_density();

    ChMatrix33<double> nmL=this->section->Get_facetFrame();
    auto computeMass = [&](double L, int pos) {
        double mass = Area * rho * std::abs(L);
        double MJxx = mass * L * L / 3.0; // eccentricity in x direction equals to L/2  ----->  Jxx=(mass*L*L/12+mass*(L/2)*(L/2)
        double MJyy = mass * w * w / 12.0;
        double MJzz = mass * h * h / 3.0; // Modified if transverse generic connector


        M(0 + pos, 0 + pos) = mass;
        M(1 + pos, 1 + pos) = mass;
        M(2 + pos, 2 + pos) = mass;

        ChMatrix33<> block_upper_right, block_lower_right;
        block_upper_right.setZero();
        block_lower_right.setZero();
        if (this->section->GetSectionType() == this->section->transverse_generic) {
            MJzz *= 0.25; // TODO JBC: For some reason this is 12 for the default (transverse generic?) connector
        } else {
            double sign = (this->section->GetSectionType() == this->section->transverse_top) ? 1.0 : -1.0;
            block_upper_right(0, 2) = sign * 0.5 * mass * h;
            block_upper_right(2, 0) = -sign * 0.5 * mass * h;

            block_lower_right(0, 1) = sign * 0.25 * mass * h * L;
            block_lower_right(1, 0) = sign * 0.25 * mass * h * L;

            // TODO JBC: I don't understand the geometry:
            // why the longitudinal connector has MJzz = m * h * h /3.0 ? and not 1/12 ?
            // the 1/3 factor seems to indicate the node is at the end of the section instead of centered
            // My gut feeling is that longitudinal connectors would be centered, like the default one. TBD
            //
        }

        block_lower_right(0, 0) = MJyy + MJzz;
        block_lower_right(1, 1) = MJyy + MJxx;
        block_lower_right(2, 2) = MJzz + MJxx;

        //double cm_x=L/2.;
        // TODO JBC: I do not understand why we are adding these. there is no x-excentricity of the midline, right ?
        // I think these should be zero?
        block_upper_right(1, 2) =  0.5 * mass * L;
        block_upper_right(2, 1) = -0.5 * mass * L;

        M.block<3,3>(pos, pos + 3) = nmL * block_upper_right * nmL.transpose();
        M.block<3,3>(pos + 3, pos) = M.block<3,3>(pos, pos + 3).transpose();
        M.block<3,3>(pos + 3, pos + 3) = nmL * block_lower_right * nmL.transpose();
    };

    computeMass(LA, 0);
    computeMass(-LB, 6);
}




void ChElementCBLCON::ComputeStiffnessMatrix() {
        assert(section);
        Km.resize(12,12);

        double normal_stiff = this->section->Get_material()->Get_E0() * this->section->Get_area() / this->length;
        double tangent_stiff = normal_stiff * this->section->Get_material()->Get_alpha();
        Eigen::DiagonalMatrix<double, 3> K_diag(normal_stiff, tangent_stiff, tangent_stiff);

        ChMatrix33<double> nmL = this->section->Get_facetFrame();
        //
        if(ChElementCBLCON::LargeDeflection){// TODO JBC: If the facet orientation is made to coincide with the quaternion, we could save a lot and move this to Update()
            ChQuaternion<> q_delta=(this->q_element_abs_rot *  this->q_element_ref_rot.GetConjugate());
            for (int id=0; id<3; id++){
                nmL.block<1,3>(id,0)=q_delta.Rotate(nmL.block<1,3>(id,0)).eigen();
            }
        }
        ChMatrix33<double> nmL_tr = nmL.transpose();
        ChMatrix33<> K_local = nmL_tr * K_diag * nmL;

        // For small deflection, use the initial position of the nodes and facet centers
        // to determine the displacement induced by the node rotation
        ChVector3d xc_xi;
        ChVector3d xc_xj;
        if (!LargeDeflection) {
            xc_xi = this->section->Get_center() - this->nodes[0]->GetX0().GetPos();
            xc_xj = this->section->Get_center() - this->nodes[1]->GetX0().GetPos();
        }
        else { // TODO: Find out how to evolve position of the center for large displacement
            xc_xi = this->section->Get_center() - this->nodes[0]->GetX0().GetPos(); // TEMPORARY
            xc_xj = this->section->Get_center() - this->nodes[1]->GetX0().GetPos(); // TEMPORARY
        }
        ChMatrix33<> Ai_cross; // 3x3 right sub-block of Ai matrix for skew-symmetric cross-product vector
        ChMatrix33<> Aj_cross; // 3x3 right sub-block of Aj matrix for skew-symmetric cross-product vector
        Ai_cross << 0.0      ,  xc_xi[2], -xc_xi[1],
                    -xc_xi[2],  0.0     ,  xc_xi[0],
                     xc_xi[1], -xc_xi[0],  0.0     ;
        Aj_cross << 0.0      ,  xc_xj[2], -xc_xj[1],
                    -xc_xj[2],  0.0     ,  xc_xj[0],
                     xc_xj[1], -xc_xj[0],  0.0     ;

        ChMatrix33<> K_local_Id_Ai = K_local * Ai_cross;
        ChMatrix33<> K_local_Ai_Id = K_local_Id_Ai.transpose();
        ChMatrix33<> K_local_Ai_Ai = K_local_Ai_Id * Ai_cross;
        ChMatrix33<> K_local_Id_Aj = K_local * Aj_cross;
        ChMatrix33<> K_local_Aj_Id = K_local_Id_Aj.transpose();
        ChMatrix33<> K_local_Aj_Aj = K_local_Aj_Id * Aj_cross;
        ChMatrix33<> K_local_Ai_Aj = K_local_Ai_Id * Aj_cross;
        ChMatrix33<> K_local_Aj_Ai = K_local_Ai_Aj.transpose();

        // dFi / dQi
        Km.block<3,3>(0,0) =  K_local;
        Km.block<3,3>(0,3) =  K_local_Id_Ai;
        Km.block<3,3>(3,0) =  K_local_Ai_Id;
        Km.block<3,3>(3,3) =  K_local_Ai_Ai;
        // dFi / dQj
        Km.block<3,3>(0,6) = -K_local;
        Km.block<3,3>(0,9) = -K_local_Id_Aj;
        Km.block<3,3>(3,6) = -K_local_Ai_Id;
        Km.block<3,3>(3,9) = -K_local_Ai_Aj;
        // dFj / dQi
        Km.block<3,3>(6,0) = -K_local;
        Km.block<3,3>(6,3) = -K_local_Id_Ai;
        Km.block<3,3>(9,0) = -K_local_Aj_Id;
        Km.block<3,3>(9,3) = -K_local_Aj_Ai;
        // dFj / dQj
        Km.block<3,3>(6,6) =  K_local;
        Km.block<3,3>(6,9) =  K_local_Id_Aj;
        Km.block<3,3>(9,6) =  K_local_Aj_Id;
        Km.block<3,3>(9,9) =  K_local_Aj_Aj;

        if (ChElementCBLCON::EnableCoupleForces){
            double mult = this->section->Get_material()->GetCoupleMultiplier();
            double w = this->section->GetWidth();
            double h = this->section->GetHeight();
            // Current bending stiffness calculations assumes:
            // - Linear variations of the angle along the connector
            // - No offset of the section centroid
            // - Section height/width in direction of 2nd/3rd axis M/L of local frame, respectively
            // - Polar moment of area for rectangular cross section
            // TODO JBC: Should we take into account the offset for the top and bottom connectors? It is taken into account in the mass matrix, why not here in the stiffness matrix?
            // TODO JBC: Polar moment of area is an approximation assuming stress grows linearly from axis, which is not correct for rectangular profiles, should we change to more exact or good enough for this?
            double rM = w * w / 12.0;
            double rL = h * h / 12.0;
            double rN = rM + rL;

            Eigen::DiagonalMatrix<double, 3> Kb_diag(rN * tangent_stiff * mult, rM * normal_stiff * mult, rL * normal_stiff * mult);
            ChMatrix33<double> Kb_local = nmL_tr * Kb_diag * nmL;
            Km.block<3,3>(3,3) += Kb_local;
            Km.block<3,3>(3,9) -= Kb_local;
            Km.block<3,3>(9,3) -= Kb_local;
            Km.block<3,3>(9,9) += Kb_local;
        }
}


void ChElementCBLCON::ComputeGeometricStiffnessMatrix() {
    assert(section);    
    
}


void ChElementCBLCON::SetupInitial(ChSystem* system) {
    assert(section);

    // Compute rest length, mass:
    this->length = (nodes[1]->GetX0().GetPos() - nodes[0]->GetX0().GetPos()).Length();
	//this->mass = this->length * this->section->GetMassPerUnitLength();
    //this->mass = this->length * 1.0;
    // Compute initial rotation
    ChMatrix33<> A0;
    ChVector3d mXele = nodes[1]->GetX0().GetPos() - nodes[0]->GetX0().GetPos();
    ChVector3d myele =
        (nodes[0]->GetX0().GetRotMat().GetAxisY() + nodes[1]->GetX0().GetRotMat().GetAxisY()).GetNormalized();
    A0.SetFromAxisX(mXele, myele);
    q_element_ref_rot = A0.GetQuaternion();
    //
    // WARNNING: Check updating rotation here is a good idea
    //
    this->UpdateRotation();
    //
    // Initialize total displacement increment
    //
    ChVectorDynamic<> displ(12);
    this->GetStateBlock(displ);
    dofs_old=displ;
    //
    // Initiliaze state variables:
    //
	// All state variables initialized at zero, which is the value in the Section constructor
    // Nothing to do here

    //
	if(this->macro_strain){
			this->section->ComputeProjectionMatrix();
			this->section->ComputeEigenStrain(this->macro_strain);			
	}

    //
    // Compute local stiffness matrix:
    //
    ComputeStiffnessMatrix();
    //
    // Compute local geometric stiffness matrix normalized by pull force P: Kg/P
    //
    ComputeGeometricStiffnessMatrix();   
}

void ChElementCBLCON::ComputeKRMmatricesGlobal(ChMatrixRef H, double Kfactor, double Rfactor, double Mfactor) {
    assert((H.rows() == 12) && (H.cols() == 12));
    assert(section);
	
    //
    // The K stiffness matrix and R damping matrix of this element:
    //
    if (Kfactor || Rfactor) {    

        if (use_numerical_diff_for_KR) {
	    //std::cout<<"////////////////////////////////////////////////////////\n";
            // numerical evaluation of the K R  matrices
            double delta_p = 1e-5;
            double delta_r = 1e-3;

            ChVectorDynamic<> Fi0(12);
            ChVectorDynamic<> FiD(12);
            this->ComputeInternalForces(Fi0);	    
            ChMatrixDynamic<> H_num(12, 12);

            // K
            ChVector3d     pa0 = this->GetNodeA()->GetPos();
            ChQuaternion<> qa0 = this->GetNodeA()->GetRot();
            ChVector3d     pb0 = this->GetNodeB()->GetPos();
            ChQuaternion<> qb0 = this->GetNodeB()->GetRot();
            for (int i = 0; i < 3; ++i) {            	
                ChVector3d paD = pa0; 
                paD[i] += delta_p;
                this->GetNodeA()->SetPos(paD);
                this->ComputeInternalForces(FiD);              
                H_num.block<12,1>(0, i) = (FiD - Fi0) / delta_p;
                this->GetNodeA()->SetPos(pa0);
            }
            for (int i = 0; i < 3; ++i) {            	
                ChVector3d rotator(VNULL);  rotator[i] = delta_r;
                ChQuaternion<> mdeltarotL;  mdeltarotL.SetFromRotVec(rotator); // rot.in local basis - as in system wide vectors
                ChQuaternion<> qaD = qa0 * mdeltarotL;
                this->GetNodeA()->SetRot(qaD);
                this->ComputeInternalForces(FiD);                
                H_num.block<12,1>(0, i+3) = (FiD - Fi0) / delta_r;
                this->GetNodeA()->SetRot(qa0);
            }
            for (int i = 0; i < 3; ++i) {            	
                ChVector3d pbD = pb0; 
                pbD[i] += delta_p;
                this->GetNodeB()->SetPos(pbD);
                this->ComputeInternalForces(FiD);
                H_num.block<12,1>(0, i+6) = (FiD - Fi0) / delta_p;                
                this->GetNodeB()->SetPos(pb0);
            }
            for (int i = 0; i < 3; ++i) {            	
                ChVector3d rotator(VNULL);  rotator[i] = delta_r;
                ChQuaternion<> mdeltarotL;  mdeltarotL.SetFromRotVec(rotator); // rot.in local basis - as in system wide vectors
                ChQuaternion<> qbD = qb0 * mdeltarotL;
                this->GetNodeB()->SetRot(qbD);
                this->ComputeInternalForces(FiD);
                H_num.block<12,1>(0, i+9) = (FiD - Fi0) / delta_r;                
                this->GetNodeB()->SetRot(qb0);
            }
            H.block(0, 0, 12, 12) =  -H_num * Kfactor;			
			//exit(2);
            /*
            // R
            ChVector3d va0 = this->GetNodeA()->GetPosDt();
            ChVector3d wa0 = this->GetNodeA()->GetAngVelLocal();
            ChVector3d vb0 = this->GetNodeB()->GetPosDt();
            ChVector3d wb0 = this->GetNodeB()->GetAngVelLocal();
            for (int i = 0; i < 3; ++i) {
                ChVector3d vaD = va0; 
                vaD[i] += delta_p;
                this->GetNodeA()->SetPos_dt(vaD);
                this->ComputeInternalForces(FiD);
                H_num.block<12,1>(0, i) = (FiD - Fi0) / delta_p;
                this->GetNodeA()->SetPos_dt(va0);
            }
            for (int i = 0; i < 3; ++i) {
                ChVector3d waD = wa0; 
                waD[i] += delta_r;
                this->GetNodeA()->SetWvel_loc(waD);
                this->ComputeInternalForces(FiD);
                H_num.block<12,1>(0, i+3) = (FiD - Fi0) / delta_r;
                this->GetNodeA()->SetWvel_loc(wa0);
            }
            for (int i = 0; i < 3; ++i) {
                ChVector3d vbD = vb0; 
                vbD[i] += delta_p;
                this->GetNodeB()->SetPos_dt(vbD);
                this->ComputeInternalForces(FiD);
                H_num.block<12,1>(0, i+6) = (FiD - Fi0) / delta_p;
                this->GetNodeB()->SetPos_dt(vb0);
            }
            for (int i = 0; i < 3; ++i) {
                ChVector3d wbD = wb0; 
                wbD[i] += delta_r;
                this->GetNodeB()->SetWvel_loc(wbD);
                this->ComputeInternalForces(FiD);
                H_num.block<12,1>(0, i+9) = (FiD - Fi0) / delta_r;
                this->GetNodeB()->SetWvel_loc(wb0);
            }
            H.block(0, 0, 12, 12) += - H_num * Rfactor;
		*/
        }
        else {

            // K stiffness:
            


            ChMatrixDynamic<> H_local;
            

            if (this->use_geometric_stiffness) {
                // K = Km+Kg

                // For Kg, compute Px tension of the beam along centerline, using temporary but fast data structures:
                ChVectorDynamic<> displ(this->GetNumCoordsPosLevel());
                this->GetStateBlock(displ);
                double Px = -this->Km.row(0) * displ;

                // Rayleigh damping (stiffness proportional part)  [R] = beta*[Km] , so H = kf*[Km+Kg]+rf*[R] = (kf+rf*beta)*[Km] + kf*Kg
                //H_local = this->Km * (Kfactor + Rfactor * this->section->GetBeamRaleyghDampingBeta()) + this->Kg * Px * Kfactor;
		H_local = this->Km * ( Kfactor ) + this->Kg * Px * Kfactor;
            }
            else {
                // K = Km

                // Rayleigh damping (stiffness proportional part)  [R] = beta*[Km] , so H = kf*[Km]+rf*[R] = (kf+rf*beta)*[K]
                //H_local = this->Km * (Kfactor + Rfactor * this->section->GetBeamRaleyghDampingBeta());
		H_local = this->Km * Kfactor;
				
            }

            //H.block(0, 0, 12, 12) = CKCt;
			
	    H.block(0, 0, 12, 12) = H_local;
           
        }

    } 
    else
		H.setZero();

    //
    // The M mass matrix of this element:  
    //

    if (Mfactor || (Rfactor) ) {

        ChMatrixDynamic<> Mloc(12, 12);
        Mloc.setZero();
        //ChMatrix33<> Mxw;
	this->ComputeMmatrixGlobal(Mloc);
        H.block(0, 0, 12, 12) += Mloc* Mfactor;

        /*
        //  The following would be needed if consistent mass matrix is used, but...
		// Corotational M mass:
         ChMatrixDynamic<> CK(12, 12);
         ChMatrixDynamic<> CKCt(12, 12);  // the global, corotated, K matrix
         ChMatrix33<> Atoabs(this->q_element_abs_rot);
         ChMatrix33<> AtolocwelA(this->GetNodeA()->Frame().GetRot().GetConjugate() * this->q_element_abs_rot);
         ChMatrix33<> AtolocwelB(this->GetNodeB()->Frame().GetRot().GetConjugate() * this->q_element_abs_rot);
         std::vector< ChMatrix33<>* > R;
         R.push_back(&Atoabs);
         R.push_back(&AtolocwelA);
         R.push_back(&Atoabs);
         R.push_back(&AtolocwelB);

         ChMatrixCorotation::ComputeCK(Mloc, R, 4, CK);
         ChMatrixCorotation::ComputeKCt(CK, R, 4, CKCt);

         H.block(0,0,12,12) += CKCt; 
	*/	 
		
		/*
        // 
		//
        // "lumped" M mass matrix
        //
        ChMatrixNM<double, 6, 6> sectional_mass;
        this->section->ComputeInertiaMatrix(sectional_mass);
        // ..rather do this because lumped mass matrix does not need rotation transf.
		
        H.block(0, 0, 12, 12) += Mloc;

        //// TODO better per-node lumping, or 4x4 consistent mass matrices, maybe with integration if not uniform
        // materials.
		*/
    }
	
}

void ChElementCBLCON::ComputeInternalForces(ChVectorDynamic<>& Fi) {
    assert(Fi.size() == 12);
    assert(section);
    
    // Displacement and rotation increment of nodes:
    ChVectorDynamic<> displ(12);
    this->GetStateBlock(displ);
    dofs_increment = displ - dofs_old;
    dofs_old = displ;

    //
    // Get facet area and length of beam
    //
    double area=this->section->Get_area();
    ChMatrix33<double> nmL=this->section->Get_facetFrame();
    if(ChElementCBLCON::LargeDeflection){// TODO JBC: If the facet orientation is made to coincide with the quaternion, we could save a lot and move this to Update()
        ChQuaternion<> q_delta=(this->q_element_abs_rot *  this->q_element_ref_rot.GetConjugate());
        for (int id=0; id<3; id++){
            nmL.block<1,3>(id,0)=q_delta.Rotate(nmL.block<1,3>(id,0)).eigen();
        }
    }

    auto mysection=this->section;
    ChVector3d mstress;
    ChVector3d dmstrain;
    ChVector3d mcouple;
    ChVector3d dcurvature;
    StateVarVector statev;
    this->ComputeStrainIncrement(dofs_increment, dmstrain, dcurvature);
    statev=mysection->Get_StateVar();
    double epsV=0; // TODO JBC: Volumetric strain seems to be a remnant of LDPM. remove it when refactoring ComputeStress
    double width=mysection->GetWidth()/2;
    double height=mysection->GetHeight()/2;  

    auto nonMechanicalStrain=mysection->Get_nonMechanicStrain();
    if (nonMechanicalStrain.size()){
        mysection->Get_material()->ComputeStress( dmstrain, dcurvature, nonMechanicalStrain, length, epsV, statev, area, width, height, mstress, mcouple);
    }else{
        mysection->Get_material()->ComputeStress( dmstrain, dcurvature, length, epsV, statev, area, width, height, mstress, mcouple);
    }

    mysection->Set_StateVar(statev);


    ChVector3d force = area * (nmL.transpose() * mstress);
    ChVector3d xc_xi;
    ChVector3d xc_xj;
    // For small deflection, use the initial position of the nodes and facet centers
    // to determine the displacement induced by the node rotation
    if (!LargeDeflection) {
        xc_xi = this->section->Get_center() - this->nodes[0]->GetX0().GetPos();
        xc_xj = this->section->Get_center() - this->nodes[1]->GetX0().GetPos();
    }
    else { // TODO: Find out how to evolve position of the center for large displacement
        xc_xi = this->section->Get_center() - this->nodes[0]->GetX0().GetPos(); // TEMPORARY
        xc_xj = this->section->Get_center() - this->nodes[1]->GetX0().GetPos(); // TEMPORARY
    }
    ChVectorDynamic<> Fi_local(12);

    Fi_local.segment(0,3) = -force.eigen();
    Fi_local.segment(3,3) = -xc_xi.Cross(force).eigen();
    Fi_local.segment(6,3) =  force.eigen();
    Fi_local.segment(9,3) =  xc_xj.Cross(force).eigen();

    
    //std::cout<<"\nFi_local-1:\n"<<Fi_local<<std::endl;
    ChVectorDynamic<> Fmu(6);
    if(ChElementCBLCON::EnableCoupleForces){
        //ChMatrix33<double> rotmat(q_delta);
        //nmL=rotmat*nmL;
        //nmL=this->section->Get_facetFrame();
        ChMatrixNM<double, 6, 3> Bx;
        Bx.block<3,3>(3,0)=nmL.transpose();
        Bx.block<3,3>(0,0)=-Bx.block<3,3>(3,0);

        //ChVectorDynamic<> Fmu(6);
        Fmu=area*(Bx*mcouple.eigen());
        //std::cout<<"length:\t"<<length<<"area:\t"<<area<<std::endl;
        //std::cout<<"mcouple:\t"<<mcouple<<std::endl;
        //std::cout<<"nmL:\n"<<nmL<<std::endl;
        Fi_local.segment(3,3)+= Fmu.segment(0,3); //area*(nmL*mcouple);
        Fi_local.segment(9,3)+= Fmu.segment(3,3); //area*(nmL*mcouple);
        //std::cout<<"Fmu:\n"<<area*(nmL*mcouple)<<std::endl;
    }

    Fi=-Fi_local;

#ifdef BEAM_VERBOSE
    GetLog() << "\nInternal forces (local): \n";
    for (int c = 0; c < 6; c++)
        GetLog() << FiK_local(c) << "  ";
    GetLog() << "\n";
    for (int c = 6; c < 12; c++)
        GetLog() << FiK_local(c) << "  ";
    GetLog() << "\n\nInternal forces (ABS) : \n";
    for (int c = 0; c < 6; c++)
        GetLog() << Fi(c) << "  ";
    GetLog() << "\n";
    for (int c = 6; c < 12; c++)
        GetLog() << Fi(c) << "  ";
    GetLog() << "\n";
#endif
 
}



void ChElementCBLCON::ComputeGravityForces(ChVectorDynamic<>& Fg, const ChVector3d& G_acc) {
    
    // no so efficient... a temporary mass matrix here:
    ChMatrixDynamic<> mM(12, 12);
    this->ComputeMmatrixGlobal(mM);

    // a vector of G accelerations for the two nodes (for translation degrees of freedom)
    ChVectorDynamic<> mG(12);
    mG.setZero();
    mG.segment(0, 3) = G_acc.eigen();
    mG.segment(6, 3) = G_acc.eigen();

    // Gravity forces as M*g, always works, regardless of the way M 
    // is computed (lumped or consistent, with offset center of mass or centered, etc.)
    // [Maybe one can replace this function with a faster ad-hoc implementation in case of lumped masses.]
    Fg = mM * mG;

    //// TODO for the lumped mass matrix case, the mM * mG product can be unrolled into few multiplications as mM mostly zero, and same for mG
}


void ChElementCBLCON::EvaluateSectionDisplacement(const double eta, ChVector3d& u_displ, ChVector3d& u_rotaz) {
    ChVectorDynamic<> displ(this->GetNumCoordsPosLevel());
    this->GetStateBlock(displ);
    ChVector3d ui = displ.segment(0,3);
    ChVector3d ri = displ.segment(3,3);
    ChVector3d uj = displ.segment(6,3);
    ChVector3d rj = displ.segment(9,3);
    ChVector3d xc_xi;
    ChVector3d xc_xj;
    // For small deflection, use the initial position of the nodes and facet centers
    // to determine the displacement induced by the node rotation
    if (!LargeDeflection) {
        xc_xi = this->section->Get_center() - this->nodes[0]->GetX0().GetPos();
        xc_xj = this->section->Get_center() - this->nodes[1]->GetX0().GetPos();
    }
    else { // TODO: Find out how to evolve position of the center for large displacement
        xc_xi = this->section->Get_center() - this->nodes[0]->GetX0().GetPos(); // TEMPORARY
        xc_xj = this->section->Get_center() - this->nodes[1]->GetX0().GetPos(); // TEMPORARY
    }

    // TODO JBC: This assumes small displacements
    // If the connector is broken and stretched, length will be very large eta_center poorly estimated
    // If we want to display the broken connector, I think a much bigger refactor would be needed because it does not really make sense
    // to call this function over normalized eta, and for many points uniformly distributed along the beam at a given resolution.
    // We should instead call over the actual distance, and evaluate only at the ends and facet discontinuities. Dispaly it as 2 elements, not one.
    double eta_center = -1.0 + 2.0 * xc_xi.Length() / this->length;

    // TODO JBC: I think using the rotation matrix rather, than the cross product, would give the true displacement for large displacement without other changes necessary
    if (eta <= eta_center) {
        double factor = (eta + 1.0) / (eta_center + 1.0);
        u_displ = ui + ri.Cross(xc_xi) * factor;
        u_rotaz = ri;
    } else {
        double factor = (1.0 - eta) / (1.0 - eta_center);
        u_displ = uj + rj.Cross(xc_xj) * factor;
        u_rotaz = rj;
    }
}

void ChElementCBLCON::EvaluateSectionFrame(const double eta, ChVector3d& point, ChQuaternion<>& rot) {
    ChVector3d u_displ;
    ChVector3d u_rotaz;
    double Nx1 = (1. / 2.) * (1 - eta);
    double Nx2 = (1. / 2.) * (1 + eta);
    //std::cout<<"\n\neta: "<<eta<<"\t";
    //std::cout<< "nodeA: "<< this->nodes[0]->Frame().GetPos() <<   "  nodeB: "<< this->nodes[1]->Frame().GetPos()<<std::endl;
    this->EvaluateSectionDisplacement(eta, u_displ, u_rotaz);
    
    
    // Since   d = [Atw]' Xt - [A0w]'X0   , so we have
    //        Xt = [Atw] (d +  [A0w]'X0)

    //point = this->q_element_abs_rot.Rotate(u_displ +
    //                                       this->q_element_ref_rot.RotateBack(Nx1 * this->nodes[0]->GetX0().GetPos() +
     //                                                                         Nx2 * this->nodes[1]->GetX0().GetPos()));
    point = Nx1 * this->nodes[0]->Frame().GetPos() +  Nx2 * this->nodes[1]->Frame().GetPos();
    ChQuaternion<> msectionrot;
    msectionrot.SetFromAngleAxis(u_rotaz.Length(), u_rotaz.GetNormalized());
    rot = this->q_element_abs_rot * msectionrot;    
    //rot = this->q_element_abs_rot * this->q_element_ref_rot;
    //rot=  this->nodes[0]->Frame().GetRot() * Nx1 +  this->nodes[1]->Frame().GetRot() * Nx2;  
    //std::cout<<"point: "<<point<<"\t"<<"rot: "<<rot<<std::endl;     
}

void ChElementCBLCON::EvaluateSectionForceTorque(const double eta, ChVector3d& Fforce, ChVector3d& Mtorque) {
    assert(section);
	
}

void ChElementCBLCON::LoadableGetStateBlockPosLevel(int block_offset, ChState& mD) {
    mD.segment(block_offset + 0, 3) = nodes[0]->GetPos().eigen();
    mD.segment(block_offset + 3, 4) = nodes[0]->GetRot().eigen();

    mD.segment(block_offset + 7, 3) = nodes[1]->GetPos().eigen();
    mD.segment(block_offset + 10, 4) = nodes[1]->GetRot().eigen();
}

void ChElementCBLCON::LoadableGetStateBlockVelLevel(int block_offset, ChStateDelta& mD) {
    mD.segment(block_offset + 0, 3) = nodes[0]->GetPosDt().eigen();
    mD.segment(block_offset + 3, 3) = nodes[0]->GetAngVelLocal().eigen();

    mD.segment(block_offset + 6, 3) = nodes[1]->GetPosDt().eigen();
    mD.segment(block_offset + 9, 3) = nodes[1]->GetAngVelLocal().eigen();
}

void ChElementCBLCON::LoadableStateIncrement(const unsigned int off_x,
                                                ChState& x_new,
                                                const ChState& x,
                                                const unsigned int off_v,
                                                const ChStateDelta& Dv) {
    nodes[0]->NodeIntStateIncrement(off_x, x_new, x, off_v, Dv);
    nodes[1]->NodeIntStateIncrement(off_x + 7, x_new, x, off_v + 6, Dv);
}

void ChElementCBLCON::LoadableGetVariables(std::vector<ChVariables*>& mvars) {
    mvars.push_back(&this->nodes[0]->Variables());
    mvars.push_back(&this->nodes[1]->Variables());
}

void ChElementCBLCON::ComputeNF(const double U,
                                   ChVectorDynamic<>& Qi,
                                   double& detJ,
                                   const ChVectorDynamic<>& F,
                                   ChVectorDynamic<>* state_x,
                                   ChVectorDynamic<>* state_w) {
    ShapeVector N;
    ShapeFunctions(N, U);  // evaluate shape functions (in compressed vector), btw. not dependant on state

    detJ = this->GetRestLength() / 2.0;

    Qi(0) = N(0) * F(0);
    Qi(1) = N(1) * F(1) + N(6) * F(5);
    Qi(2) = N(1) * F(2) - N(6) * F(4);
    Qi(3) = N(0) * F(3);
    Qi(4) = -N(2) * F(2) + N(8) * F(4);
    Qi(5) = N(2) * F(1) + N(8) * F(5);

    Qi(6) = N(3) * F(0);
    Qi(7) = N(4) * F(1) + N(7) * F(5);
    Qi(8) = N(4) * F(2) - N(7) * F(4);
    Qi(9) = N(3) * F(3);
    Qi(10) = -N(5) * F(2) + N(9) * F(4);
    Qi(11) = N(5) * F(1) + N(9) * F(5);
}

void ChElementCBLCON::ComputeNF(const double U,
                                   const double V,
                                   const double W,
                                   ChVectorDynamic<>& Qi,
                                   double& detJ,
                                   const ChVectorDynamic<>& F,
                                   ChVectorDynamic<>* state_x,
                                   ChVectorDynamic<>* state_w) {
    this->ComputeNF(U, Qi, detJ, F, state_x, state_w);
    detJ /= 4.0;  // because volume
}

double ChElementCBLCON::GetDensity() {
	return 1;
    //return this->section->GetMassPerUnitLength();
}



ChMatrixNM<double, 1, 9> ChElementCBLCON::ComputeMacroStressContribution(){
	ChMatrixNM<double, 1, 9> macro_stress;
	double area =this->section->Get_area();	
	auto statev= this->section->Get_StateVar();		
	macro_stress =(this->section->GetProjectionMatrix()).transpose()*statev.segment(3,3)*area*this->length;
	
	return macro_stress;
}



}  // end namespace wood
}  // end namespace chrono
