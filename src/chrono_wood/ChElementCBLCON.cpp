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
// Authors: Erol Lale,Jibril B. Coulibaly, Wisdom Akpan
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
#include "chrono/core/ChQuaternion.h"
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
        // TODO JBC: I think this should also update the projection matrix for eigenstrains. TBD when large deflection is actually implemented
        this->section->ComputeEigenStrain(this->macro_strain);
    } else { // JBC: This might not be necessary since the value of 0.0 in SetupInitial() should remain if macro_strain is nullptr
        this->section->Set_nonMechanicStrain(ChVector3d(0.0, 0.0, 0.0));
    }
}

void ChElementCBLCON::UpdateRotation() {
    ChMatrix33<> Aabs;
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

    // TODO JBC: This looks fragile and relies on assumptions and current behavior of other parts of the code
    //           - If the Y-axes are aligned with the node-to-node direction, or cancel out, Gram-Schmidt will fail and use an arbitrary direction
    //              This arbitrary direction is repeatable, that is the only thing that guarantee this works, otherwise, the initial quaternion determined in SetupInitial() might be different
    //              Any change to the fallback behavior of ChMatrix33::SetFromAxisX() will break this code !
    //           - I believe the commented code above for beams should be used instead, together with the initial quaternion being aligned with nmL matrix, similar to the beam element
    // TODO JBC: This quaternion update actually does not work under large displacement
    //           If I only comment line 154 (i.e., not update old quaternion), everything runs fine
    ChVector3d mXele = nodes[1]->Frame().GetPos() - nodes[0]->Frame().GetPos();
    ChVector3d mYele =
    (nodes[0]->Frame().GetRotMat().GetAxisY() + nodes[1]->Frame().GetRotMat().GetAxisY()).GetNormalized();
    Aabs.SetFromAxisX(mXele, mYele);
    q_element_abs_rot = Aabs.GetQuaternion();
    this->A.SetFromQuaternion(q_element_ref_rot.GetConjugate() * q_element_abs_rot);

    // New center position

    // 1. Projection `P` of center `C` on the edge should remain in the same relative position along the edge
    //    For CBL, the relative position is currently always 1/2
    ChVector3d center = 0.5 * (nodes[0]->GetPos() + nodes[1]->GetPos());
    // 2. Vector `PC` keeps the same length but rotates with the edge
    //    The centroid is aligned with the nodes for several connector types:
    //       - ConSectionType::transverse_regular_bot
    //       - ConSectionType::transverse_regular_gen
    //       - ConSectionType::transverse_regular_top
    //       - TODO: test the other connectors / ask Wisdom about the geometry
    //    For those, rotation is unnecessary
    if (section->GetSectionType() == ChBeamSectionCBLCON::ConSectionType::longitudinal) { // TODO JBC: why is this not aligned?
        ChQuaternion<> q_delta = q_element_abs_rot * q_element_ref_rot.GetConjugate();
        ChVector3d xp_xc0 = section->Get_center_ref() - 0.5 * (nodes[0]->GetX0().GetPos() + nodes[1]->GetX0().GetPos());
        center += q_delta.Rotate(xp_xc0);
    }

    section->Set_center(center);
}


void ChElementCBLCON::ComputeBranchVectors(ChVector3d& xi_xc, ChVector3d& xj_xc) {
    if (!ChElementCBLCON::LargeDeflection) {
        // For small deflection, use the initial position of the nodes and facet centers
        xi_xc = section->Get_center_ref() - nodes[0]->GetX0().GetPos();
        xj_xc = section->Get_center_ref() - nodes[1]->GetX0().GetPos();
    } else {
        // For large deflection, use the current position of the nodes and facet centers
        xi_xc = section->Get_center() - nodes[0]->GetPos();
        xj_xc = section->Get_center() - nodes[1]->GetPos();
    }
}


void ChElementCBLCON::GetStateBlock(ChVectorDynamic<>& statev) {
    // Copy paste of ChElementLDPM::LoadableGetStateBlockPosLevel because this function
    // takes ChVectorDynamic as an argument, not a ChState
    unsigned index_start = GetDOFOffset();
    statev.segment(index_start + 0, 3) = nodes[0]->GetPos().eigen();
    statev.segment(index_start + 3, 4) = nodes[0]->GetRot().eigen();
    //
    statev.segment(index_start + 7, 3) = nodes[1]->GetPos().eigen();
    statev.segment(index_start + 10, 4) = nodes[1]->GetRot().eigen();
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


void ChElementCBLCON::ComputeStrainIncrement(ChVectorN<double, 12>& dofs_increment, ChVector3d& strain_increment, ChVector3d& curvature_increment) {
    //
    ChVector3d dui = dofs_increment.segment(0,3);
    ChVector3d dri = dofs_increment.segment(3,3);
    ChVector3d duj = dofs_increment.segment(6,3);
    ChVector3d drj = dofs_increment.segment(9,3);

    double linv = 1.0 / this->length;

    ChMatrix33<double> nmL=this->section->Get_facetFrame();
    if(ChElementCBLCON::LargeDeflection){	// TODO JBC: If the facet orientation is made to coincide with the quaternion, we could save a lot and move this to Update()
        ChQuaternion<> q_delta=(this->q_element_abs_rot *  this->q_element_ref_rot.GetConjugate());
        for (int id=0; id<3; id++){
            nmL.block<1,3>(id,0)=q_delta.Rotate(nmL.block<1,3>(id,0)).eigen();
        }
    }

    ChVector3d xi_xc, xj_xc;
    ComputeBranchVectors(xi_xc, xj_xc);

    // Strain
    strain_increment = nmL * (duj + drj.Cross(xj_xc) - (dui + dri.Cross(xi_xc))) * linv;

    // Curvature
    if (ChElementCBLCON::EnableCoupleForces){
        curvature_increment = nmL * (drj - dri) * linv;
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
        pC = this->section->Get_center_ref();
    } else {
        // JBC: Technically, if the connector is stretched, at constant mass, the moment of inertia changes
        pNA = this->GetNodeA()->GetPos();
        pNB = this->GetNodeB()->GetPos();
        pC = this->section->Get_center();
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
    ChMatrix33<double> nmL_tr=nmL.transpose();
    
    auto computeMass = [&](double L, int pos) {
        double mass = Area * rho * std::abs(L);
        double MJxx = mass * L * L / 3.0; // eccentricity in x direction equals to L/2  ----->  Jxx=(mass*L*L/12+mass*(L/2)*(L/2)
        double MJyy = mass * w * w / 12.0;
        double MJzz = mass * h * h / 3.0; // Modified if transverse generic connector


        M(0 + pos, 0 + pos) = mass;
        M(1 + pos, 1 + pos) = mass;
        M(2 + pos, 2 + pos) = mass;

        ChMatrix33<> block_UR_local, block_LR_local;
        block_UR_local.setZero();
        block_LR_local.setZero();

        auto type = this->section->GetSectionType();
        if (type == this->section->transverse_regular_gen ||
            type == this->section->tangential_ray_gen ||
            type == this->section->radial_ray_gen) {

        MJzz *= 0.25; // TODO JBC: For some reason this is 12 for the default (transverse generic?) connector
        } else {
        double sign = (type == this->section->transverse_regular_top ||
                   type == this->section->tangential_ray_top ||
                   type == this->section->radial_ray_top) ? 1.0 : -1.0;
            block_UR_local(0, 2) = sign * 0.5 * mass * h;
            block_UR_local(2, 0) = -sign * 0.5 * mass * h;

            block_LR_local(0, 1) = sign * 0.25 * mass * h * L;
            block_LR_local(1, 0) = sign * 0.25 * mass * h * L;

            // TODO JBC: I don't understand the geometry:
            // why the longitudinal connector has MJzz = m * h * h /3.0 ? and not 1/12 ?
            // the 1/3 factor seems to indicate the node is at the end of the section instead of centered
            // My gut feeling is that longitudinal connectors would be centered, like the default one. TBD
            //
        }

        block_LR_local(0, 0) = MJyy + MJzz;
        block_LR_local(1, 1) = MJyy + MJxx;
        block_LR_local(2, 2) = MJzz + MJxx;

        //double cm_x=L/2.;
        // TODO JBC: I do not understand why we are adding these. there is no x-excentricity of the midline, right ?
        // I think these should be zero?
        block_UR_local(1, 2) =  0.5 * mass * L;
        block_UR_local(2, 1) = -0.5 * mass * L;

        ChMatrix33<> block_UR = nmL * block_UR_local * nmL_tr;
        M.block<3,3>(pos, pos + 3) = block_UR;
        M.block<3,3>(pos + 3, pos) = block_UR.transpose();
        M.block<3,3>(pos + 3, pos + 3) = nmL * block_LR_local * nmL_tr;
    };
    computeMass(LA, 0);
    computeMass(-LB, 6);
}




void ChElementCBLCON::ComputeStiffnessMatrixGlobal(ChMatrixRef Km) {
        assert(section);

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


        ChVector3d xi_xc, xj_xc;
        ComputeBranchVectors(xi_xc, xj_xc);
        // 3x3 right sub-block of Ai matrix for skew-symmetric cross-product vector
        ChMatrix33<> Ai_cross(0.0);
        Ai_cross(0,1) =  xi_xc[2]; Ai_cross(1,0) = -xi_xc[2];
        Ai_cross(0,2) = -xi_xc[1]; Ai_cross(2,0) =  xi_xc[1];
        Ai_cross(1,2) =  xi_xc[0]; Ai_cross(2,1) = -xi_xc[0];
        // 3x3 right sub-block of Aj matrix for skew-symmetric cross-product vector
        ChMatrix33<> Aj_cross(0.0);
        Aj_cross(0,1) =  xj_xc[2]; Aj_cross(1,0) = -xj_xc[2];
        Aj_cross(0,2) = -xj_xc[1]; Aj_cross(2,0) =  xj_xc[1];
        Aj_cross(1,2) =  xj_xc[0]; Aj_cross(2,1) = -xj_xc[0];

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


void ChElementCBLCON::ComputeGeometricStiffnessMatrixGlobal(ChMatrixRef Kg) {
    assert(section);
    // Geometric stiffness matrix not implemented

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
    q_element_abs_rot = q_element_ref_rot; // Initialize the current quaternion as the reference quaternion

    section->Set_center(section->Get_center_ref());
    //
    // Initiliaze state variables:
    //
    statevar_old.setZero(GetNumStateVar());
    GetStateBlock(statevar_old); // Store DOFs at the end of state variables vector (temporary, to do updated Lagrangian until these are stored at the nodes in Tasora's new design)
    statevar = statevar_old;
    //
    if(this->macro_strain){
            this->section->ComputeProjectionMatrix();
            this->section->ComputeEigenStrain(this->macro_strain);
    } else {
        this->section->Set_nonMechanicStrain(ChVector3d(0.0, 0.0, 0.0));
    }
}

void ChElementCBLCON::ComputeKRMmatricesGlobal(ChMatrixRef H, double Kfactor, double Rfactor, double Mfactor) {
    assert((H.rows() == 12) && (H.cols() == 12));
    assert(section);

    //
    // The K stiffness matrix and R damping matrix of this element:
    //
    if (Kfactor || Rfactor) {

        if (use_numerical_diff_for_KR) {
            // TODO JBC: numerical differentiation not used for this elements / material
            //           I think the previous code that was copy-pasted here was wrong
            //           with the current implementation of CBL due to displacement update
            //           not being reset when calling this->ComputeInternalForces
        }
        else {
            // CBL currently uses the initial elastic stiffness matrix
            ChMatrixNM<double, 12, 12> Km;
            ComputeStiffnessMatrixGlobal(Km);

            if (this->use_geometric_stiffness) {
                ChMatrixNM<double, 12, 12> Kg;
                ComputeGeometricStiffnessMatrixGlobal(Kg); // TODO: not currently implemented
                H.block(0, 0, 12, 12) = (Km + Kg) * Kfactor;
            } else {
                H.block(0, 0, 12, 12) = Km * Kfactor;
            }
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

    // Compute increment of DOFs in global frame
    this->GetStateBlock(statevar); // Store current DOFs at the end of state variables vector (temporary, to do updated Lagrangian until these are stored at the nodes in Tasora's new design)
    ChVectorN<double, 12> dofs_increment;
    for (int inode = 0 ; inode < 2 ; inode++) {
        unsigned int index_start = GetDOFOffset();
        // Displacement in global frame
        dofs_increment.segment(inode*6,3) = statevar.segment(index_start + inode*7,3) - statevar_old.segment(index_start + inode*7,3);

        // Rotation in global frame
        ChQuaterniond q_curr(statevar.segment(index_start + inode*7 + 3,4));
        ChQuaterniond q_prev(statevar_old.segment(index_start + inode*7 + 3,4));
        ChQuaterniond dq = q_curr * q_prev.GetConjugate();
        ChVector3d delta_rot_dir;
        double delta_rot_angle;
        dq.GetAngleAxis(delta_rot_angle, delta_rot_dir);
        // TODO: We may want to use GetRotVec directly, but the angle is not within [-PI .. PI]
        // TODO: Consider changing GetRotVec() to return within [-PI .. PI]
        dofs_increment.segment(inode*6+3, 3) = (delta_rot_angle * delta_rot_dir).eigen();
    }

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
    ChMatrix33<double> nmL_tr = nmL.transpose();

    ChVector3d strain_increment, curvature_increment;
    ComputeStrainIncrement(dofs_increment, strain_increment, curvature_increment);

    double width=section->GetWidth()/2;
    double height=section->GetHeight()/2;
    double random_field =section->GetRandomField();
    auto nonMechanicalStrain=section->Get_nonMechanicStrain();
    ChVector3d mstress, mcouple;
    section->Get_material()->ComputeStress(strain_increment, curvature_increment, nonMechanicalStrain, length, statevar_old, statevar, area, width, height, random_field, mstress, mcouple);

    ChVector3d force = area * (nmL_tr * mstress);

    ChVector3d xi_xc, xj_xc;
    ComputeBranchVectors(xi_xc, xj_xc);

    Fi.segment(0,3) =  force.eigen();
    Fi.segment(3,3) =  xi_xc.Cross(force).eigen();
    Fi.segment(6,3) = -force.eigen();
    Fi.segment(9,3) = -xj_xc.Cross(force).eigen();

    if(ChElementCBLCON::EnableCoupleForces){
        ChVector3d couple_global = area * (nmL_tr * mcouple);
        Fi.segment(3,3) += couple_global.eigen();
        Fi.segment(9,3) -= couple_global.eigen();
    }
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
    // TODO JBC: had to do the hack below after redefining GetStateBlock()
    //           Not sure what this function does, it's called by ChVisualShapeWood.
    double angle;
    ChVector3d axis;
    ChVector3d ui = nodes[0]->GetPos() - nodes[0]->GetX0().GetPos();
    (nodes[0]->GetRot() * nodes[0]->GetX0().GetRot().GetConjugate()).GetAngleAxis(angle, axis);
    ChVector3d ri = axis * angle;
    ChVector3d uj = nodes[1]->GetPos() - nodes[1]->GetX0().GetPos();
    (nodes[1]->GetRot() * nodes[1]->GetX0().GetRot().GetConjugate()).GetAngleAxis(angle, axis);
    ChVector3d rj = axis*angle;

    ChVector3d xi_xc, xj_xc;
    ComputeBranchVectors(xi_xc, xj_xc);

    // TODO JBC: This assumes small displacements
    // If the connector is broken and stretched, length will be very large eta_center poorly estimated
    // If we want to display the broken connector, I think a much bigger refactor would be needed because it does not really make sense
    // to call this function over normalized eta, and for many points uniformly distributed along the beam at a given resolution.
    // We should instead call over the actual distance, and evaluate only at the ends and facet discontinuities. Dispaly it as 2 elements, not one.
    double eta_center = -1.0 + 2.0 * xi_xc.Length() / this->length;

    // TODO JBC: I think using the rotation matrix rather, than the cross product, would give the true displacement for large displacement without other changes necessary
    if (eta <= eta_center) {
        double factor = (eta + 1.0) / (eta_center + 1.0);
        u_displ = ui + ri.Cross(xi_xc) * factor;
        u_rotaz = ri;
    } else {
        double factor = (1.0 - eta) / (1.0 - eta_center);
        u_displ = uj + rj.Cross(xj_xc) * factor;
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
    macro_stress =(this->section->GetProjectionMatrix()).transpose()*statevar.segment(3,3)*area*this->length;

    return macro_stress;
}



}  // end namespace wood
}  // end namespace chrono
