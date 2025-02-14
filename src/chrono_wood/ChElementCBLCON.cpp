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
// Authors: Erol Lale
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
#include <iomanip>

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

void ChElementCBLCON:: ComputeAmatrix( Amatrix& Amat, chrono::ChVector3d X , chrono::ChVector3d XI ){
		
		Amat.setZero();
		Amat(0,0)=1.0;
		Amat(1,1)=1.0;
		Amat(2,2)=1.0;
		//
		Amat(0,4)=X[2]-XI[2];
		Amat(0,5)=XI[1]-X[1];
		//
		Amat(1,3)=XI[2]-X[2];
		Amat(1,5)=X[0]-XI[0];
		//
		Amat(2,3)=X[1]-XI[1];
		Amat(2,4)=XI[0]-X[0];
		
	}

void ChElementCBLCON::Update() {
    // parent class update:
    ChElementGeneric::Update();

    // always keep updated the rotation matrix A:
	if (ChElementCBLCON::LargeDeflection){
		this->UpdateRotation();    
		this->section->Set_Length((nodes[1]->GetPos() - nodes[0]->GetPos()).Length());
	}	
	//
	if(this->macro_strain){		
		this->section->ComputeEigenStrain(this->macro_strain);
	}
    // get displacement increment between current step and previous step
    // update total displacement increment 
    //ChVectorDynamic<> displ(12);
    //this->GetStateBlock(displ);
    //DUn_1=displ-Un_1;
    //Un_1=displ;  
    //
    // Compute local stiffness matrix:
    //
    //ComputeStiffnessMatrix();  
  
	
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

    ChVector3d delta_rot_dir;
    double delta_rot_angle;

    // Node 0, displacement (in local element frame, corotated back)
    //     d = [Atw]' Xt - [A0w]'X0	
	
    ChVector3d displ = nodes[0]->Frame().GetPos() - nodes[0]->GetX0().GetPos();
	mD.segment(0, 3) = displ.eigen();    
    // Node 0, x,y,z small rotations (in local element frame)
    ChQuaternion<> q_delta0 =  nodes[0]->Frame().GetRot() ;    
    q_delta0.GetAngleAxis(delta_rot_angle, delta_rot_dir);	
    if (delta_rot_angle > CH_PI) 
        delta_rot_angle -= CH_2PI;  // no 0..360 range, use -180..+180	
    mD.segment(3, 3) = delta_rot_angle * delta_rot_dir.eigen();
    
	

    // Node 1, displacement (in local element frame, corotated back)
    //     d = [Atw]' Xt - [A0w]'X0
    displ = nodes[1]->Frame().GetPos() - nodes[1]->GetX0().GetPos();	
    mD.segment(6, 3) = displ.eigen();

    // Node 1, x,y,z small rotations (in local element frame)
    ChQuaternion<> q_delta1 =  nodes[1]->Frame().GetRot() ;    
    q_delta1.GetAngleAxis(delta_rot_angle, delta_rot_dir);	
    if (delta_rot_angle > CH_PI)
        delta_rot_angle -= CH_2PI;  // no 0..360 range, use -180..+180    
    mD.segment(9, 3) = delta_rot_angle * delta_rot_dir.eigen();
	
	
}



void ChElementCBLCON::GetLatticeStateBlock(ChVectorDynamic<>& mD) {
    mD.setZero(12);
    mD.segment(0, 6)=DUn_1.segment(0,6); 
    mD.segment(6, 6)=DUn_1.segment(6,6); 
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


void ChElementCBLCON::ComputeStrain(ChVectorDynamic<>& mstrain, ChVectorDynamic<>& curvature) {
    mstrain.resize(3);
	// 
	// Displacement of nodes:
	ChVectorDynamic<> displ(this->GetNumCoordsPosLevel());
	//this->GetStateBlock(displ);
	this->GetLatticeStateBlock(displ);
	auto dispN1=displ.segment(0,6);
	auto dispN2=displ.segment(6,6);	
	//dispN1.setConstant(1E-4);
	//dispN2.setConstant(1.2E-4);
	//
	// Length is calculated 
	double length;
	ChMatrix33<double> nmL=this->section->Get_facetFrame();
	if(ChElementCBLCON::LargeDeflection){	
		length = (this->GetNodeA()->Frame().GetPos() - this->GetNodeB()->Frame().GetPos()).Length();
		ChQuaternion<> q_delta=(this->q_element_abs_rot *  this->q_element_ref_rot.GetConjugate());
		for (int id=0; id<3; id++){				
			nmL.block<1,3>(id,0)=q_delta.Rotate(nmL.block<1,3>(id,0)).eigen();
		}	
	}else{
		length = (this->GetNodeA()->GetX0().GetPos() - this->GetNodeB()->GetX0().GetPos()).Length();
	}
	//
	Amatrix AI; 	Amatrix AJ;
	// A matrix is calculated based on initial coordinates
	this->ComputeAmatrix(AI, this->section->Get_center() , 
						     this->nodes[0]->GetX0().GetPos());
									
	this->ComputeAmatrix( AJ, this->section->Get_center() , 
							  this->nodes[1]->GetX0().GetPos());
									
			
	ChVectorN<double,3> n; //
	for (int id=0;id<3;id++) {			
			//
			// NodeA
			//			
			n<< nmL(id,0), nmL(id,1), nmL(id,2);
			//std::cout<<id<<" n: \n"<<n<<std::endl;
			//n<< nmL(id,0), nmL(id,1), nmL(id,2);			
			ChMatrixNM<double,1,6> BI= n.transpose()*AI/length;
			//std::cout << "matrix BI:\n" << BI << std::endl; 	
			//
			// NodeB
			//		
			ChMatrixNM<double,1,6> BJ= n.transpose()*AJ/length;			
			//std::cout << "matrix BJ:\n" << BJ << std::endl; 
			//
			mstrain(id)=double (BJ*dispN2)+ double (-BI*dispN1); 

		}
		
	if (ChElementCBLCON::EnableCoupleForces){
		//nmL=this->section->Get_facetFrame();
		curvature=(displ.segment(9,3)-displ.segment(3,3))/length;
		curvature=nmL*curvature;		
		
	}	
	
}




void ChElementCBLCON::ComputeStress(ChVectorDynamic<>& mstress) {
    mstress.resize(3);
	ChVectorDynamic<> mstrain;
	ChVectorDynamic<> curvature;
	this->ComputeStrain(mstrain, curvature);
	//std::cout<<"\nmstrain:\n"<<mstrain<<std::endl;	
	//
	double E0=this->section-> Get_material()->Get_E0();
	double alpha=this->section-> Get_material()->Get_alpha();	
	//
	double epsQ=pow(mstrain(0)*mstrain(0)+alpha*(mstrain(1)*mstrain(1)+mstrain(2)*mstrain(2)), 0.5);
	double strsQ=E0*epsQ;	
	//
	if (epsQ!=0) {
		mstress(0)=strsQ*mstrain(0)/epsQ;
		mstress(1)=alpha*strsQ*mstrain(1)/epsQ;
		mstress(2)=alpha*strsQ*mstrain(2)/epsQ;
	}else{
		mstress<< 0.0, 0.0, 0.0;
		//mstress.setZero();
	}	
}




/*
void ChElementCBLCON::ComputeMmatrixGlobal(ChMatrixRef M) {
    M.setZero();
    // Mass Matrix 
	ChVector3d pNA=this->GetNodeA()->GetX0().GetPos();
	ChVector3d pNB=this->GetNodeB()->GetX0().GetPos();
	
	auto vertices=this->V_vert_nodes;
	for(auto verts: vertices){
		auto pC=verts[0]->GetX0().GetPos();
		auto pA=verts[1]->GetX0().GetPos();
		auto pB=verts[2]->GetX0().GetPos();		
		
		ChMatrixNM<double,6,6> mA=this->ComputeTetMassN(pNA, pC, pA, pB);
		ChMatrixNM<double,6,6> mB=this->ComputeTetMassN(pNB, pC, pA, pB);
		M.block<6,6>(0,0)+=mA;
		M.block<6,6>(6,6)+=mB;
	}
	
}
*/


void ChElementCBLCON::ComputeMmatrixGlobal(ChMatrixRef M) {
    M.setZero();
    // Mass Matrix 
	ChVector3d pNA=this->GetNodeA()->GetX0().GetPos();
	ChVector3d pNB=this->GetNodeB()->GetX0().GetPos();
	ChVector3d pC=this->section->Get_center();
	double LA=(pC-pNA).Length();
	double LB=(pC-pNB).Length();
	ChMatrixNM<double,6,6> mA=this->section->calculate_MI(LA, pNA, pC);
	ChMatrixNM<double,6,6> mB=this->section->calculate_MI(-LB, pNB, pC);
	//ChMatrixNM<double,6,6> mA=this->ComputeTetMassN(pNA, pC, pA, pB);
	//ChMatrixNM<double,6,6> mB=this->ComputeTetMassN(pNB, pC, pA, pB);
	ChMatrix33<double> nmL=this->section->Get_facetFrame();
	//std::cout<<"nmL: "<<nmL<<std::endl;
	//M.block<6,6>(0,0) = mA;
	//M.block<6,6>(6,6) = mB;
	ChMatrixNM<double,6,6> Rot;
	Rot.setZero();
	Rot.block<3,3>(0,0)=nmL;
	Rot.block<3,3>(3,3)=nmL;
	M.block<6,6>(0,0) = Rot*mA*Rot.transpose();		
	M.block<6,6>(6,6) = Rot*mB*Rot.transpose();	
	//std::cout<<"PNA: \n"<<pNA<<"\t"<<"PNB: \n"<<pNB<<std::endl;
	//std::cout<<"M: \n"<<M<<std::endl;
	//exit(0);	
	
}




void ChElementCBLCON::ComputeStiffnessMatrix() {
		assert(section);
		Km.resize(12,12);
		
		double E0= this->section->Get_material()->Get_E0();
		double alpha=this->section-> Get_material()->Get_alpha();    	
		//
		double ET=alpha*E0;
		//
		// Get facet area and length of beam
		double area=this->section->Get_area();
		//
		// Get initial section frame
		ChMatrix33<double> nmL=this->section->Get_facetFrame();
		//
		double length;
		if(ChElementCBLCON::LargeDeflection){	
			length = (this->GetNodeA()->Frame().GetPos() - this->GetNodeB()->Frame().GetPos()).Length();
			ChQuaternion<> q_delta=(this->q_element_abs_rot *  this->q_element_ref_rot.GetConjugate());
			for (int id=0; id<3; id++){				
				nmL.block<1,3>(id,0)=q_delta.Rotate(nmL.block<1,3>(id,0)).eigen();
			}	
		}else{
			length = (this->GetNodeA()->GetX0().GetPos() - this->GetNodeB()->GetX0().GetPos()).Length();
		}
		// 
		Amatrix AI; 	
		Amatrix AJ;
		// A matrix is calculated based on initial coordinates
		this->ComputeAmatrix(AI, this->section->Get_center() , 
								 this->nodes[0]->GetX0().GetPos());
										
		this->ComputeAmatrix( AJ, this->section->Get_center() , 
								  this->nodes[1]->GetX0().GetPos());
		//	
		ChMatrixNM<double,3,6> mBI;
		ChMatrixNM<double,3,6> mBJ;
		ChVectorN<double,3> n;
		for (int id=0;id<3;id++) {				
				n<< nmL(id,0), nmL(id,1), nmL(id,2);
				//
				// NodeA
				//
				mBI.block<1,6>(id,0) = n.transpose()*AI/length;				
				//
				// NodeB
				//				
				mBJ.block<1,6>(id,0) = n.transpose()*AJ/length;

		}
		//
		double aLE_N=area*length*E0;
		double aLE_T=aLE_N*alpha;
		//std::cout<<"mBI:\n"<<mBI<<std::endl;
		//std::cout<<"mBJ:\n"<<mBJ<<std::endl;
		// Kii
		Km.block<6,6>(0,0)=aLE_N*mBI.block<1,6>(0,0).transpose()*mBI.block<1,6>(0,0)+
						   aLE_T*(mBI.block<1,6>(1,0).transpose()*mBI.block<1,6>(1,0)+
							mBI.block<1,6>(2,0).transpose()*mBI.block<1,6>(2,0) );
		// Kjj
		Km.block<6,6>(6,6)=aLE_N*mBJ.block<1,6>(0,0).transpose()*mBJ.block<1,6>(0,0)+
						   aLE_T*(mBJ.block<1,6>(1,0).transpose()*mBJ.block<1,6>(1,0)+
							mBJ.block<1,6>(2,0).transpose()*mBJ.block<1,6>(2,0) );
		// Kij
		Km.block<6,6>(0,6)=-aLE_N*mBI.block<1,6>(0,0).transpose()*mBJ.block<1,6>(0,0)-
						   aLE_T*(mBI.block<1,6>(1,0).transpose()*mBJ.block<1,6>(1,0)+
							mBI.block<1,6>(2,0).transpose()*mBJ.block<1,6>(2,0) );
		// Kji
		Km.block<6,6>(6,0)=-aLE_N*mBJ.block<1,6>(0,0).transpose()*mBI.block<1,6>(0,0)-
						   aLE_T*(mBJ.block<1,6>(1,0).transpose()*mBI.block<1,6>(1,0)+
							mBJ.block<1,6>(2,0).transpose()*mBI.block<1,6>(2,0) );
							
		
		if (ChElementCBLCON::EnableCoupleForces){
		        //nmL=this->section->Get_facetFrame();
			double multiplier=this->section->Get_material()->GetCoupleMultiplier()*area*E0/length;
			double w=this->section->GetWidth()/2.; 
    			double h=this->section->GetHeight()/2.; 
    			double onethird=1./3.;
    			double rM=w*w*onethird;
    			double rL=h*h*onethird;
    			double rN=rM+rL;
    			//std::cout<<"nmL: "<<nmL<<std::endl;
    			//std::cout<<"rN: "<<rN<<"\t"<<"rM: "<<rM<<"\t"<<"rL: "<<rL<<"\n";
    			//std::cout<<"W: "<<w<<"\t"<<"h: "<<h<<"\n";
    			//std::cout<<"alpha: "<<alpha<<"\t"<<"beta: "<<this->section->Get_material()->GetCoupleMultiplier()<<"\n";
		        //ChMatrix33<double> rotmat(q_delta);		        
			//nmL=rotmat*nmL;
			//std::cout<<"rot-nmL: "<<nmL<<std::endl;
			ChMatrix33<double> Dmat;
			Dmat.setZero();			
			Dmat(1,1)=rM*multiplier;
			Dmat(2,2)=rL*multiplier;
			Dmat(0,0)=alpha*rN*multiplier;
			//std::cout<<"Dmat: \n"<<Dmat<<std::endl;
			ChMatrix33<double> kmu=nmL.transpose()*Dmat*nmL;
			Km.block<3,3>(3,3)+=kmu;
			Km.block<3,3>(9,3)-=kmu;
			Km.block<3,3>(9,9)+=kmu;
			Km.block<3,3>(3,9)-=kmu;
			//std::cout<<"kmu\n"<<kmu<<std::endl;
			//exit(1);
			
		}
		
		
		//std::cout<<"PNA: "<<this->GetNodeA()->Frame().GetPos()<<"\t"<<"PNB: "<<this->GetNodeB()->Frame().GetPos()<<std::endl;
		//std::cout<<"center: "<<this->section->Get_center()<<std::endl;
		//std::cout<<"nmL:\n"<<nmL<<std::endl;
		//std::cout<<"AI: \n"<< AI <<std::endl;
		//std::cout<<"AJ: \n"<< AJ <<std::endl;
		//for (int ik=0; ik<12; ik++)
		//	if (Km(ik,ik)==0)
		//		Km(ik,ik)+=aLE_T/1000;
		//std::cout<<"Km: \n"<<Km<<std::endl;
		
		
}


void ChElementCBLCON::ComputeGeometricStiffnessMatrix() {
    assert(section);    
    
}


void ChElementCBLCON::SetupInitial(ChSystem* system) {
    assert(section);

    // Compute rest length, mass:
    this->section->Set_Length((nodes[1]->GetX0().GetPos() - nodes[0]->GetX0().GetPos()).Length());
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
    Un_1=displ;
    //
    // Initiliaze state variables:
    //
	
    auto statev= this->section->Get_StateVar();
    statev.resize(18);
    statev.setZero();
    this->section->Set_StateVar(statev); 
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
    
    ChVectorDynamic<> displ(12);
    this->GetStateBlock(displ);
    DUn_1=displ-Un_1;
    Un_1=displ;	
	//
	// Get facet area and length of beam
	//
	double area=this->section->Get_area();	
	ChMatrix33<double> nmL=this->section->Get_facetFrame();	
	double length;
	if(ChElementCBLCON::LargeDeflection){	
		length = (this->GetNodeA()->Frame().GetPos() - this->GetNodeB()->Frame().GetPos()).Length();
		ChQuaternion<> q_delta=(this->q_element_abs_rot *  this->q_element_ref_rot.GetConjugate());		
		for (int id=0; id<3; id++){				
			nmL.block<1,3>(id,0)=q_delta.Rotate(nmL.block<1,3>(id,0)).eigen();
		}
		
		//std::cout<<"nmL: "<<nmL<<std::endl;
	}else{
		length = (this->GetNodeA()->GetX0().GetPos() - this->GetNodeB()->GetX0().GetPos()).Length();
	 }   			
	// 
	Amatrix AI; 	
	Amatrix AJ;
	// A matrix is calculated based on initial coordinates
	this->ComputeAmatrix(AI, this->section->Get_center() , 
						     this->nodes[0]->GetX0().GetPos());
									
	this->ComputeAmatrix( AJ, this->section->Get_center() , 
							  this->nodes[1]->GetX0().GetPos());
									
	//ChMatrix33<double> nmL=this->section->Get_facetFrame();
	//ChQuaternion<> q_delta=(this->q_element_abs_rot *  this->q_element_ref_rot.GetConjugate());		
	//	
	ChMatrixNM<double,3,6> mBI;
	ChMatrixNM<double,3,6> mBJ;
	ChVectorN<double,3> n;
	for (int id=0;id<3;id++) {			
			n<< nmL(id,0), nmL(id,1), nmL(id,2);
			//
			// NodeA
			//
			mBI.block<1,6>(id,0) = n.transpose()*AI/length;				
			//
			// NodeB
			//				
			mBJ.block<1,6>(id,0) = n.transpose()*AJ/length;

		}
	//
	// Get Stress values at facet center
	//ChVectorDynamic<> mstress;
	//this->ComputeStress(mstress);
	
	auto mysection=this->section;
	ChVectorDynamic<> mstress;	
	ChVectorDynamic<> dmstrain;
	ChVectorDynamic<> mcouple;
	ChVectorDynamic<> dcurvature;
	ChVectorDynamic<> statev;
	this->ComputeStrain(dmstrain,dcurvature);
	statev=mysection->Get_StateVar();
	// compute volumetric strain
	double epsV=0;
	/*for (int itet:this->tetIDs){ 
    		auto vols=ChElementCBLCON::mTetVols[itet]; 
    		if (vols[1]!=0)
    			epsV+=(vols[1]-vols[0])/vols[0];   			
    	}
    	epsV=epsV/(3.0*this->tetIDs.size());*/  
    	//    	
    double width=mysection->GetWidth()/2;
    double height=mysection->GetHeight()/2;  
	
	auto nonMechanicalStrain=mysection->Get_nonMechanicStrain();	
    if (nonMechanicalStrain.size()){
		mysection->Get_material()->ComputeStress( dmstrain, dcurvature, nonMechanicalStrain, length, epsV, statev, area, width, height, mstress, mcouple);
	}else{
		mysection->Get_material()->ComputeStress( dmstrain, dcurvature, length, epsV, statev, area, width, height, mstress, mcouple);
	}	
		
	mysection->Set_StateVar(statev);	
	
    			//std::cout<<"W: "<<width<<"\t"<<"h: "<<height<<"\n";
    			
	//std::cout<<"\nmstress:\n"<<mstress<<std::endl;	
    // [local Internal Forces] = [Klocal] * displ + [Rlocal] * displ_dt
    ChVectorDynamic<> Fi_local(12);
	
	Fi_local.segment(0,6)=-area*length*(mstress.transpose()*mBI);
	Fi_local.segment(6,6)= area*length*(mstress.transpose()*mBJ);	
	
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
		Fmu=area*(Bx*mcouple);
		//std::cout<<"length:\t"<<length<<"area:\t"<<area<<std::endl;
		//std::cout<<"mcouple:\t"<<mcouple<<std::endl;
		//std::cout<<"nmL:\n"<<nmL<<std::endl;	
		Fi_local.segment(3,3)+= Fmu.segment(0,3); //area*(nmL*mcouple);
		Fi_local.segment(9,3)+= Fmu.segment(3,3); //area*(nmL*mcouple);
		//std::cout<<"Fmu:\n"<<area*(nmL*mcouple)<<std::endl;
		
	}
	//std::cout<<"\nFi_local:\n"<<Fi_local<<std::endl;
	
	//std::cout<< "Fi_local : "<< Fi_local << std::endl;
    //// set up vector of nodal velocities (in local element system)
    //ChVectorDynamic<> displ_dt(12);
    //this->GetField_dt(displ_dt);
	/*
    // Rayleigh damping - stiffness proportional
    ChMatrixDynamic<> FiR_local = section->GetBeamRaleyghDampingBeta() * Km * displ_dt;

    Fi_local += FiR_local;

    // Rayleigh damping - mass proportional
    if (this->section->GetBeamRaleyghDampingAlpha()) {
        ChMatrixDynamic<> Mloc(12, 12);
        Mloc.setZero();
        ChMatrix33<> Mxw;

        // the "lumped" M mass matrix must be computed
        ChMatrixNM<double, 6, 6> sectional_mass;
        this->section->ComputeInertiaMatrix(sectional_mass);

        // Rayleigh damping (stiffness proportional part)  [Rm] = alpha*[M] 
        double node_multiplier_fact = 0.5 * length * (this->section->GetBeamRaleyghDampingAlpha());
        for (int i = 0; i < nodes.size(); ++i) {
            int stride = i * 6; 
            Mloc.block<6, 6>(stride, stride) += sectional_mass * node_multiplier_fact;
        }
        FiR_local = Mloc * displ_dt;
        Fi_local += FiR_local;
    }
    
	*/
	
     //Fi_local *= -1.0;
    //
    // Corotate local internal loads
    //
	
    /*
    // Fi = C * Fi_local  with C block-diagonal rotations A  , for nodal forces in abs. frame
    ChMatrix33<> Atoabs(this->q_element_abs_rot);
    ChMatrix33<> AtolocwelA(this->GetNodeA()->Frame().GetRot().GetConjugate() * this->q_element_abs_rot);
    ChMatrix33<> AtolocwelB(this->GetNodeB()->Frame().GetRot().GetConjugate() * this->q_element_abs_rot);
    std::vector<ChMatrix33<>*> R;
    R.push_back(&Atoabs);
    R.push_back(&AtolocwelA);
    R.push_back(&Atoabs);
    R.push_back(&AtolocwelB);
    ChMatrixCorotation::ComputeCK(Fi_local, R, 4, Fi);*/
     
    Fi=-Fi_local;
       
    	
    	//}
    //Fi=-Km*displ;
	/*
    // Add also inertial quadratic terms: gyroscopic and centrifugal
    
    // CASE OF LUMPED MASS - fast
    double node_multiplier = 0.5 * length;
    ChVector3d mFcent_i;
    ChVector3d mTgyro_i;
    for (int i = 0; i < nodes.size(); ++i) {
        this->section->ComputeQuadraticTerms(mFcent_i, mTgyro_i, nodes[i]->GetAngVelLocal());
        Fi.segment(i * 6, 3)     -= node_multiplier * (nodes[i]->GetA() * mFcent_i).eigen();
        Fi.segment(3 + i * 6, 3) -= node_multiplier * mTgyro_i.eigen();
    }
    */

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
	/*	
    ShapeVector N;
    ShapeFunctions(N, eta);  // Evaluate compressed shape functions

    u_displ.x() = N(0) * displ(0) + N(3) * displ(6);     // x_a   x_b
    u_displ.y() = N(1) * displ(1) + N(4) * displ(7)      // y_a   y_b
                  + N(2) * displ(5) + N(5) * displ(11);  // Rz_a  Rz_b
    u_displ.z() = N(1) * displ(2) + N(4) * displ(8)      // z_a   z_b
                  - N(2) * displ(4) - N(5) * displ(10);  // Ry_a  Ry_b

    u_rotaz.x() = N(0) * displ(3) + N(3) * displ(9);    // Rx_a  Rx_b
    u_rotaz.y() = -N(6) * displ(2) - N(7) * displ(8) +  // z_a   z_b   note - sign
                  N(8) * displ(4) + N(9) * displ(10);   // Ry_a  Ry_b
    u_rotaz.z() = N(6) * displ(1) + N(7) * displ(7) +   // y_a   y_b
                  N(8) * displ(5) + N(9) * displ(11);   // Rz_a  Rz_b
    	*/
    Amatrix Amat;
    double Nx1 = (1. / 2.) * (1 - eta);
    double Nx2 = (1. / 2.) * (1 + eta);
    ChVector3d point = Nx1 * this->nodes[0]->Frame().GetPos() +  Nx2 * this->nodes[1]->Frame().GetPos();
    if (eta<=0.5) {
    	
    	this->ComputeAmatrix(Amat, point , this->nodes[0]->GetX0().GetPos());
    	u_displ=Amat*displ.segment(0, 3);
    	u_rotaz=Amat*displ.segment(3, 3);
    }else{
    	this->ComputeAmatrix(Amat, point , this->nodes[1]->GetX0().GetPos());
    	u_displ=Amat*displ.segment(6, 3);
    	u_rotaz=Amat*displ.segment(9, 3);
    }
    //std::cout<<"u_displ: "<<u_displ<<"\t"<<"u_rotaz:\t"<<u_rotaz<<std::endl;
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

double ChElementCBLCON::ComputeTetVol(ChVector3d p1, ChVector3d p2, ChVector3d p3, ChVector3d p4){
	double tetvol=0;
	tetvol = p2[0]*(p3[1]*p4[2]-p4[1]*p3[2])-p3[0]*(p2[1]*p4[2]-p4[1]*p2[2])+p4[0]*(p2[1]*p3[2]-p3[1]*p2[2]);
    tetvol = tetvol-(p3[0]*(p4[1]*p1[2]-p1[1]*p4[2])-p4[0]*(p3[1]*p1[2]-p1[1]*p3[2])+p1[0]*(p3[1]*p4[2]-p4[1]*p3[2]));
    tetvol = tetvol+p4[0]*(p1[1]*p2[2]-p2[1]*p1[2])-p1[0]*(p4[1]*p2[2]-p2[1]*p4[2])+p2[0]*(p4[1]*p1[2]-p1[1]*p4[2]);
    tetvol = tetvol-(p1[0]*(p2[1]*p3[2]-p3[1]*p2[2])-p2[0]*(p1[1]*p3[2]-p3[1]*p1[2])+p3[0]*(p1[1]*p2[2]-p2[1]*p1[2]));
    tetvol = abs(tetvol)/6.0;
	return tetvol;
}

ChMatrixNM<double,6,6> ChElementCBLCON::ComputeTetMassN(ChVector3d pN, ChVector3d pC, ChVector3d pA, ChVector3d pB){
	
	double density=this->section->Get_material()->Get_density();
	//
	ChMatrix33<double> coef={{1.0, 0.05, 0.05},{0.05, 0.1, 0.05},{0.05, 0.05, 0.1}};
	//
	double vol=this->ComputeTetVol(pN, pC, pA, pB);
	double tetmass=vol*density;
	ChVector3d pG=(pN+pC+pA+pB)/4.0;
	ChVectorN<double,3> X={pA[0]-pN[0], pB[0]-pN[0], pC[0]-pN[0]};
	ChVectorN<double,3> Y={pA[1]-pN[1], pB[1]-pN[1], pC[1]-pN[1]};
	ChVectorN<double,3> Z={pA[2]-pN[2], pB[2]-pN[2], pC[2]-pN[2]};
	// first moment of area
	double Sx=(pG[0]-pN[0])*tetmass;
	double Sy=(pG[1]-pN[1])*tetmass;
	double Sz=(pG[2]-pN[2])*tetmass;
	// Moment of inertia
	double Ixx=double (X.transpose()*(coef*X))*tetmass;
	double Iyy=double(Y.transpose()*(coef*Y))*tetmass;
	double Izz=double(Z.transpose()*(coef*Z))*tetmass;
	double Ixy=double(X.transpose()*(coef*Y))*tetmass;
	double Ixz=double(X.transpose()*(coef*Z))*tetmass;
	double Iyz=double(Y.transpose()*(coef*Z))*tetmass;
	//
	ChMatrixNM<double,6,6> MN;
	MN.setZero();
	if (ChElementCBLCON::LumpedMass){ //lumped mass
		MN(0,0)=tetmass; 
		MN(1,1)=tetmass; 
		MN(2,2)=tetmass; 
		MN(3,3)=Iyy+Izz;
		MN(4,4)=Ixx+Izz; 
		MN(5,5)=Ixx+Iyy; 
	}else{ //consistent mass
		MN(0,0)=tetmass; MN(0,4)=Sz; MN(0,5)=-Sy;
		MN(1,1)=tetmass; MN(1,3)=-Sz; MN(1,5)=Sx;
		MN(2,2)=tetmass; MN(2,3)=Sy; MN(2,4)=-Sx;
		MN(3,1)=-Sz; MN(3,2)=Sy; MN(3,3)=Iyy+Izz; MN(3,4)=-Ixy; MN(3,5)=-Ixz;
		MN(4,0)=Sz; MN(4,2)=-Sx; MN(4,3)=-Ixy; 	MN(4,4)=Ixx+Izz; MN(4,5)=-Iyz;
		MN(5,0)=-Sy; MN(5,1)= Sx; MN(5,3)=-Ixz; MN(5,4)=-Iyz; MN(5,5)=Ixx+Iyy; 
	}
	//
	return MN;
}



ChMatrixNM<double, 1, 9> ChElementCBLCON::ComputeMacroStressContribution(){
	ChMatrixNM<double, 1, 9> macro_stress;
	double length=this->section->Get_Length();		 	
	double area =this->section->Get_area();	
	auto statev= this->section->Get_StateVar();		
	macro_stress =(this->section->GetProjectionMatrix()).transpose()*statev.segment(3,3)*area*length;		
	
	return macro_stress;
}



}  // end namespace wood
}  // end namespace chrono
