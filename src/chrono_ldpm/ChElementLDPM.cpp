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
// Class for LDPM elements:  
//
//  i)   Internal forces
//  ii)  Stiffness matrix
//  iii) Mass matrix  
//  iv)  Body forces
//
// Formulation of the LDPM element can be found in: https://doi.org/10.1016/j.cemconcomp.2011.02.011
// =============================================================================

#include "chrono_ldpm/ChElementLDPM.h"


namespace chrono {
namespace ldpm {



ChElementLDPM::ChElementLDPM() : Volume(0) {
    nodes.resize(4);
    this->MatrB.setZero(6, 24);
    this->StiffnessMatrix.setZero(24, 24);    
}

ChElementLDPM::~ChElementLDPM() {}

void ChElementLDPM::SetNodes(std::shared_ptr<fea::ChNodeFEAxyzrot> nodeA,
                                     std::shared_ptr<fea::ChNodeFEAxyzrot> nodeB,
                                     std::shared_ptr<fea::ChNodeFEAxyzrot> nodeC,
                                     std::shared_ptr<fea::ChNodeFEAxyzrot> nodeD) {
    nodes[0] = nodeA;
    nodes[1] = nodeB;
    nodes[2] = nodeC;
    nodes[3] = nodeD;
    std::vector<ChVariables*> mvars;
    mvars.push_back(&nodes[0]->Variables());
    mvars.push_back(&nodes[1]->Variables());
    mvars.push_back(&nodes[2]->Variables());
    mvars.push_back(&nodes[3]->Variables());    
    Kmatr.SetVariables(mvars);    
    
}

void ChElementLDPM::Update() {
    // parent class update:
    ChElementGeneric::Update();
    // always keep updated the rotation matrix A:
    this->UpdateRotation();
    //this->ComputeStiffnessMatrix();
	/*
    ChVectorDynamic<> displ(24);
    this->GetStateBlock(displ);
    DUn_1=displ-Un_1;
    Un_1=displ;
    */
}

void ChElementLDPM::ShapeFunctions(ShapeVector& N, double r, double s, double t) {
    N(0) = 1.0 - r - s - t;
    N(1) = r;
    N(2) = s;
    N(3) = t;
}


void ChElementLDPM:: ComputeAmatrix( Amatrix& Amat, chrono::ChVector3d X , chrono::ChVector3d XI ){
		
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


void ChElementLDPM::GetStateBlock(ChVectorDynamic<>& mD) {
    mD.setZero(this->GetNumCoordsPosLevel());
    
    //
    ChVector3d delta_rot_dir;
    double delta_rot_angle;
    ChQuaternion<> q_delta;
    //
    // First node
    //
    // displacement dofs:
    mD.segment(0, 3) = (nodes[0]->Frame().GetPos() - nodes[0]->GetX0().GetPos()).eigen();
    // rotational dofs:
    q_delta =  nodes[0]->Frame().GetRot() ;    
    q_delta.GetAngleAxis(delta_rot_angle, delta_rot_dir);	
    if (delta_rot_angle > CH_PI) 
        delta_rot_angle -= CH_2PI;  // no 0..360 range, use -180..+180	
    mD.segment(3, 3) = delta_rot_angle * delta_rot_dir.eigen();
    //
    // Second node
    //
    // displacement dofs:
    mD.segment(6, 3) = ( nodes[1]->Frame().GetPos() - nodes[1]->GetX0().GetPos()).eigen();    
    // rotational dofs:
    q_delta =  nodes[1]->Frame().GetRot() ;    
    q_delta.GetAngleAxis(delta_rot_angle, delta_rot_dir);	
    if (delta_rot_angle > CH_PI) 
        delta_rot_angle -= CH_2PI;  // no 0..360 range, use -180..+180	
    mD.segment(9, 3) = delta_rot_angle * delta_rot_dir.eigen();
    //
    // third node
    //
    // displacement dofs:
    //
    mD.segment(12, 3) = ( nodes[2]->Frame().GetPos() - nodes[2]->GetX0().GetPos()).eigen();
    // rotational dofs:
    q_delta =  nodes[2]->Frame().GetRot() ;    
    q_delta.GetAngleAxis(delta_rot_angle, delta_rot_dir);	
    if (delta_rot_angle > CH_PI) 
        delta_rot_angle -= CH_2PI;  // no 0..360 range, use -180..+180	
    mD.segment(15, 3) = delta_rot_angle * delta_rot_dir.eigen();
    //
    // fourth node
    //
    // displacement dofs:
    //
    mD.segment(18, 3) = ( nodes[3]->Frame().GetPos() - nodes[3]->GetX0().GetPos()).eigen();
    // rotational dofs:
    q_delta =  nodes[3]->Frame().GetRot() ;    
    q_delta.GetAngleAxis(delta_rot_angle, delta_rot_dir);	
    if (delta_rot_angle > CH_PI) 
        delta_rot_angle -= CH_2PI;  // no 0..360 range, use -180..+180	
    mD.segment(21, 3) = delta_rot_angle * delta_rot_dir.eigen();
    
    
}




void ChElementLDPM::GetLatticeStateBlock(unsigned int& ind, unsigned int& jnd, ChVectorDynamic<>& mD) {
    mD.setZero(12);
	mD.segment(0, 6)=DUn_1.segment(ind*6,6); 
	mD.segment(6, 6)=DUn_1.segment(jnd*6,6);
    /*
    //
    ChVector3d delta_rot_dir;
    double delta_rot_angle;
    ChQuaternion<> q_delta;
    //
    // First node
    //
    // displacement dofs:
    mD.segment(0, 3) = (nodes[ind]->Frame().GetPos() - nodes[ind]->GetX0().GetPos()).eigen();
    // rotational dofs:
    q_delta =  nodes[ind]->Frame().GetRot() ;    
    q_delta.GetAngleAxis(delta_rot_angle, delta_rot_dir);	
    if (delta_rot_angle > CH_PI) 
        delta_rot_angle -= CH_2PI;  // no 0..360 range, use -180..+180	
    mD.segment(3, 3) = delta_rot_angle * delta_rot_dir.eigen();
    //
    // Second node
    //
    // displacement dofs:
    mD.segment(6, 3) = ( nodes[jnd]->Frame().GetPos() - nodes[jnd]->GetX0().GetPos()).eigen();    
    // rotational dofs:
    q_delta =  nodes[jnd]->Frame().GetRot() ;    
    q_delta.GetAngleAxis(delta_rot_angle, delta_rot_dir);	
    if (delta_rot_angle > CH_PI) 
        delta_rot_angle -= CH_2PI;  // no 0..360 range, use -180..+180	
    mD.segment(9, 3) = delta_rot_angle * delta_rot_dir.eigen();
    */
    
}



/*
void ChElementLDPM::GetField_dt(ChVectorDynamic<>& mD_dt) {
    mD_dt.resize(12);

    // Node 0, velocity (in local element frame, corotated back by A' )
    mD_dt.segment(0, 3) = (nodes[0]->Frame().GetPosDt()).eigen();

    // Node 0, x,y,z ang.velocity (in local element frame, corotated back by A' )
    mD_dt.segment(3, 3) = (nodes[0]->Frame().GetAngVelParent()).eigen();

    // Node 1, velocity (in local element frame, corotated back by A' )
    mD_dt.segment(6, 3) = (nodes[1]->Frame().GetPosDt()).eigen();

    // Node 1, x,y,z ang.velocity (in local element frame, corotated back by A' )
    mD_dt.segment(9, 3) = (nodes[1]->Frame().GetAngVelParent()).eigen();
    
     // Node 2, velocity (in local element frame, corotated back by A' )
    mD_dt.segment(12, 3) = (nodes[2]->Frame().GetPosDt()).eigen();

    // Node 2, x,y,z ang.velocity (in local element frame, corotated back by A' )
    mD_dt.segment(15, 3) = (nodes[2]->Frame().GetAngVelParent()).eigen();

    // Node 3, velocity (in local element frame, corotated back by A' )
    mD_dt.segment(18, 3) = (nodes[3]->Frame().GetPosDt()).eigen();

    // Node 3, x,y,z ang.velocity (in local element frame, corotated back by A' )
    mD_dt.segment(21, 3) = (nodes[3]->Frame().GetAngVelParent()).eigen();
}
*/



void ChElementLDPM::GetLatticeField_dt(unsigned int& ind, unsigned int& jnd, ChVectorDynamic<>& mD) {
    mD.setZero(12);    
    //
    //
    // First node
    //
    // displacement dofs:
    mD.segment(0, 3) = (nodes[ind]->Frame().GetPosDt()).eigen();
    // rotational dofs:   	
    mD.segment(3, 3) = (nodes[ind]->Frame().GetAngVelParent()).eigen();
    //
    // Second node
    //
    // displacement dofs:
    mD.segment(6, 3) = (nodes[jnd]->Frame().GetPosDt()).eigen();
    // rotational dofs:   	
    mD.segment(9, 3) = (nodes[jnd]->Frame().GetAngVelParent()).eigen();
        
}

/*
void ChElementLDPM::GetLatticeStateBlock(unsigned int& ind, unsigned int& jnd, ChVectorDynamic<>& mD) {
    mD.setZero(12);    
    double dT=this->GetCurrentTimeIncrement(this->mysystem);
    std::cout<<"Step time: "<<	 this->mysystem->GetChTime()<<"  dT: "<<dT<<"\t";
    this->GetLatticeField_dt(ind, jnd, mD);
	printf("mD-1: %f %f %f %f %f %f %f %f %f %f %f %f\n", mD(0), mD(1), mD(2), mD(3), mD(4), mD(5),mD(6), mD(7), mD(8), mD(9), mD(10), mD(11));
    mD*=dT;	
	printf("mD-2: %f %f %f %f %f %f %f %f %f %f %f %f\n", mD(0), mD(1), mD(2), mD(3), mD(4), mD(5),mD(6), mD(7), mD(8), mD(9), mD(10), mD(11));
}
*/



void ChElementLDPM::ComputeStrain(std::shared_ptr<ChSectionLDPM> section, unsigned int& ind, unsigned int& jnd, ChVectorDynamic<>& mstrain) {
    mstrain.resize(3);    	
	// 
	// Displacement of nodes:
	ChVectorDynamic<> displ(12);
	this->GetLatticeStateBlock(ind, jnd, displ);	
	auto dispN1=displ.segment(0,6);
	auto dispN2=displ.segment(6,6);
	//std::cout<<"dispN1:\n"<<dispN1<<std::endl;
	//std::cout<<"dispN2:\n"<<dispN2<<std::endl;
	//std::cout<<"TotaldispN1:\n"<<(this->nodes[ind]->Frame().GetPos() - this->nodes[ind]->GetX0().GetPos())<<std::endl;
	//std::cout<<"TotaldispN2:\n"<<(this->nodes[jnd]->Frame().GetPos() - this->nodes[jnd]->GetX0().GetPos())<<std::endl;
	//
	double length;
	if(LargeDeflection){	
		length = (this->nodes[jnd]->Frame().GetPos() - this->nodes[ind]->Frame().GetPos()).Length();
	}else{
		length = (this->nodes[jnd]->GetX0().GetPos() - this->nodes[ind]->GetX0().GetPos()).Length();
	}
	Amatrix AI; 	Amatrix AJ;
	// A matrix is calculated based on initial coordinates
	this->ComputeAmatrix(AI, section->Get_center(), this->nodes[ind]->GetX0().GetPos());
									
	this->ComputeAmatrix( AJ, section->Get_center(), this->nodes[jnd]->GetX0().GetPos());
									
	ChMatrix33<double> nmL=section->Get_facetFrame();
	ChQuaternion<> q_delta=(section->Get_abs_rot() * section->Get_ref_rot().GetConjugate()) ;
		
	ChVectorN<double,3> n; //
	for (int id=0;id<3;id++) {			
			//
			// NodeA
			//
			ChVector3d nn{nmL(id,0), nmL(id,1), nmL(id,2)};	
			nn=q_delta.Rotate(nn);
			//nn=chrono::ChTransform<>::TransformParentToLocal(nn, ChVector3d(0, 0, 0) ,  q_delta);
			n<< nn[0], nn[1], nn[2];
			ChMatrixNM<double,1,6> BI= n.transpose()*AI/length;				
			//
			// NodeB
			//		
			ChMatrixNM<double,1,6> BJ= n.transpose()*AJ/length;
			//
			mstrain(id)=double (BJ*dispN2)+ double (-BI*dispN1); 

		}	
	
}




void ChElementLDPM::ComputeStress(std::shared_ptr<ChSectionLDPM> section, unsigned int& ind, unsigned int& jnd, ChVectorDynamic<>& mstress) {
    mstress.resize(3);
	ChVectorDynamic<> mstrain;
	this->ComputeStrain(section, ind, jnd, mstrain);
	//std::cout<<"\nmstrain:\n"<<mstrain<<std::endl;	
	//
	double E0=section-> Get_material()->Get_E0();
	double alpha=section-> Get_material()->Get_alpha();	
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






double ChElementLDPM::ComputeVolume() {
    ChVector3d B1, C1, D1;
    B1.Sub(nodes[1]->Frame().GetPos(), nodes[0]->Frame().GetPos());
    C1.Sub(nodes[2]->Frame().GetPos(), nodes[0]->Frame().GetPos());
    D1.Sub(nodes[3]->Frame().GetPos(), nodes[0]->Frame().GetPos());
    ChMatrixDynamic<> M(3, 3);
    M.col(0) = B1.eigen();
    M.col(1) = C1.eigen();
    M.col(2) = D1.eigen();
    M.transposeInPlace();
    double Volume = std::abs(M.determinant() / 6);
    return Volume;
}

void ChElementLDPM::ComputeStiffnessMatrix() {
    //assert(section);    
    unsigned int iface=0; 
    StiffnessMatrix.setZero();     
    for (auto facet:this->GetSection()){
    	//std::cout<< "face: "<<iface+1<<"-------------------------------\n";
    	unsigned int ind=facetNodeNums(iface,0);
    	unsigned int jnd=facetNodeNums(iface,1);    	
    	double E0= facet->Get_material()->Get_E0();
    	double alpha=facet-> Get_material()->Get_alpha(); 
    	double ET=alpha*E0;
    	//    	
    	//
    	// Get facet area and length of beam
    	double area=facet->Get_area();
    	double length;
	if(LargeDeflection){	
		length = (this->GetTetrahedronNode(ind)->Frame().GetPos() - this->GetTetrahedronNode(jnd)->Frame().GetPos()).Length();
	}else{
		length = (this->GetTetrahedronNode(ind)->GetX0().GetPos() - this->GetTetrahedronNode(jnd)->GetX0().GetPos()).Length();
	}    	
    	double aLE_N=area*length*E0;
	double aLE_T=aLE_N*alpha;	
    	// 
    	Amatrix AI; 
    	Amatrix AJ;
    	// A matrix is calculated based on initial coordinates
    	this->ComputeAmatrix(AI, facet->Get_center(),  this->nodes[ind]->GetX0().GetPos());
    	this->ComputeAmatrix(AJ, facet->Get_center(), this->nodes[jnd]->GetX0().GetPos());    	
        //
    	ChMatrix33<double> nmL=facet->Get_facetFrame(); 
    	ChQuaternion<> q_delta=(facet->Get_abs_rot() *  facet->Get_ref_rot().GetConjugate());
    	//	
    	ChMatrixNM<double,3,6> mBI;
    	ChMatrixNM<double,3,6> mBJ;
    	ChVectorN<double,3> n;
    	for (int id=0;id<3;id++) {
	    	ChVector3d nn{nmL(id,0), nmL(id,1), nmL(id,2)};		    	
	    	nn=q_delta.Rotate(nn);
	    	//nn=chrono::ChTransform<>::TransformParentToLocal(nn, ChVector3d(0, 0, 0) ,  q_delta);
	    	n<< nn[0], nn[1], nn[2]; 
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
	    
	    //std::cout<<"mBI:\n"<<mBI<<std::endl;
	    //std::cout<<"mBJ:\n"<<mBJ<<std::endl;	    
	    
	    // Kii
	    StiffnessMatrix.block<6,6>(ind*6,ind*6)+=( aLE_N*mBI.block<1,6>(0,0).transpose()*mBI.block<1,6>(0,0)+
	    			aLE_T*(mBI.block<1,6>(1,0).transpose()*mBI.block<1,6>(1,0)+
	    			mBI.block<1,6>(2,0).transpose()*mBI.block<1,6>(2,0) ) );
	    // Kjj
	    StiffnessMatrix.block<6,6>(jnd*6,jnd*6)+=( aLE_N*mBJ.block<1,6>(0,0).transpose()*mBJ.block<1,6>(0,0)+
	    			aLE_T*(mBJ.block<1,6>(1,0).transpose()*mBJ.block<1,6>(1,0)+
	    			mBJ.block<1,6>(2,0).transpose()*mBJ.block<1,6>(2,0) ) );
	    // Kij
	    StiffnessMatrix.block<6,6>(ind*6,jnd*6)+=( -aLE_N*mBI.block<1,6>(0,0).transpose()*mBJ.block<1,6>(0,0)-
	    			aLE_T*(mBI.block<1,6>(1,0).transpose()*mBJ.block<1,6>(1,0)+
	    			mBI.block<1,6>(2,0).transpose()*mBJ.block<1,6>(2,0) ) );
	    // Kji
	    StiffnessMatrix.block<6,6>(jnd*6,ind*6)+=( -aLE_N*mBJ.block<1,6>(0,0).transpose()*mBI.block<1,6>(0,0)-
	    			aLE_T*(mBJ.block<1,6>(1,0).transpose()*mBI.block<1,6>(1,0)+
	    			mBJ.block<1,6>(2,0).transpose()*mBI.block<1,6>(2,0) ) );
	    //
	    iface++;	    
    }    
    //    
}

void ChElementLDPM::SetupInitial(ChSystem* system) {
    //
    //this->mysystem=system;
    // set initial orientation of facets
    unsigned int iface=0;    
    for( auto facet:this->GetSection()){ 
    	    unsigned int ind=facetNodeNums(iface,0);
    	    unsigned int jnd=facetNodeNums(iface,1);
	    
	    ChMatrix33<> A0;
	    ChVector3d mXele = nodes[jnd]->GetX0().GetPos() - nodes[ind]->GetX0().GetPos();
	    ChVector3d myele =
		(nodes[ind]->GetX0().GetRotMat().GetAxisY() + nodes[jnd]->GetX0().GetRotMat().GetAxisY()).GetNormalized();
	    A0.SetFromAxisX(mXele, myele);
	    facet->Set_ref_rot( A0.GetQuaternion() );
	    ///
	    ///
	    ///
	    auto statev= facet->Get_StateVar();
	    statev.resize(16);
	    statev.setZero();
	    facet->Set_StateVar(statev); 
	    iface++;
     }	
    
	ChVectorDynamic<> displ(24);
    this->GetStateBlock(displ);
	Un_1=displ;
	
    this->UpdateRotation();
    //
    double vol=ComputeVolume();
    this->SetVolume(vol);
    //
    ComputeStiffnessMatrix();
}

void ChElementLDPM::UpdateRotation() {
    //
    //    
    unsigned int iface=0;    
    for( auto facet:this->GetSection()){
    	    unsigned int ind=facetNodeNums(iface,0);
    	    unsigned int jnd=facetNodeNums(iface,1);   
    	          
	    ChMatrix33<> A0(facet->Get_ref_rot());	
	    ChMatrix33<> Aabs;
	    ChQuaternion<> q_lattice_abs_rot;
	    if (!LargeDeflection) {
		Aabs = A0;
		q_lattice_abs_rot =facet->Get_ref_rot();
	    }else {
	    			
	    	ChVector3d mXele = nodes[jnd]->Frame().GetPos() - nodes[ind]->Frame().GetPos();
	    	ChVector3d myele = (nodes[ind]->Frame().GetRotMat().GetAxisY() + nodes[jnd]->Frame().GetRotMat().GetAxisY()).GetNormalized();
		
	    	Aabs.SetFromAxisX(mXele, myele);        
	    	q_lattice_abs_rot = Aabs.GetQuaternion();	    	
	    	
	    }
		
	    facet->Set_abs_rot(q_lattice_abs_rot);	    
	    iface++;
    
    }
	
	//DUn_1
    
   
}

void ChElementLDPM::ComputeKRMmatricesGlobal(ChMatrixRef H, double Kfactor, double Rfactor, double Mfactor) {
    assert((H.rows() == 24) && (H.cols() == 24));     
    // For K stiffness matrix and R damping matrix:
    double RayleighDampingK=this->GetFacetI(0)->Get_material()->GetRayleighDampingK();
    double RayleighDampingM=this->GetFacetI(0)->Get_material()->GetRayleighDampingM();
    //
    if (Kfactor || Rfactor) { 
    double mkfactor = Kfactor + Rfactor * RayleighDampingK;
    H.block(0, 0, 24, 24) = mkfactor*this->StiffnessMatrix;   
    }
    // For M mass matrix:
    if (Mfactor || Rfactor) {      	    
        ChMatrixDynamic<> Mloc(24, 24); 
	this->ComputeMmatrixGlobal(Mloc);	
	double amfactor = Mfactor + Rfactor * RayleighDampingM;
	H.block(0, 0, 24, 24)+= amfactor*Mloc;        
    }
     
    //std::cout<<"H:\n"<<H<<std::endl;
    //***TO DO*** better per-node lumping, or 12x12 consistent mass matrix.
}

void ChElementLDPM::ComputeInternalForces(ChVectorDynamic<>& Fi) {
    assert(Fi.size() == 24);
    //assert(section);
    // set up vector of nodal displacements (in local element system) u_l = R*p - p0
    //ChVectorDynamic<> mD(24);
    //this->GetStateBlock(mD);  // nodal displacements, local
    ChVectorDynamic<> displ(24);
    this->GetStateBlock(displ);
    DUn_1=displ-Un_1;
    Un_1=displ;
    //
    //ChVectorDynamic<> Fi(24);
    Fi.setZero();
    //
    // compute volumetric strain
    double V0=this->GetVolume();
    double V=this->ComputeVolume();	
    double epsV=(V-V0)/(3.0*V0);   
    //
    //
    unsigned int iface=0;
    for (auto facet:this->GetSection()){    
    	//std::cout<<"iface: "<<iface<<"--------";
    	unsigned int ind=facetNodeNums(iface,0);
    	unsigned int jnd=facetNodeNums(iface,1);     	
    	//
	// Get facet area and length of beam
	//
	double area=facet->Get_area();
	double length;
	if(LargeDeflection){	
		length = (this->GetTetrahedronNode(ind)->Frame().GetPos() - this->GetTetrahedronNode(jnd)->Frame().GetPos()).Length();
	}else{
		length = (this->GetTetrahedronNode(ind)->GetX0().GetPos() - this->GetTetrahedronNode(jnd)->GetX0().GetPos()).Length();
	}    		
	//std::cout<<" L: "<<length<<"\t";
	// 
	Amatrix AI; 	
	Amatrix AJ;
	// A matrix is calculated based on initial coordinates
	this->ComputeAmatrix(AI, facet->Get_center(), this->nodes[ind]->GetX0().GetPos());
									
	this->ComputeAmatrix(AJ, facet->Get_center(), this->nodes[jnd]->GetX0().GetPos());
									
	ChMatrix33<double> nmL=facet->Get_facetFrame();
	ChQuaternion<> q_delta=(facet->Get_abs_rot() *  facet->Get_ref_rot().GetConjugate());		
	//	
	ChMatrixNM<double,3,6> mBI;
	ChMatrixNM<double,3,6> mBJ;
	ChVectorN<double,3> n;	
	for (int id=0;id<3;id++) {
		ChVector3d nn{nmL(id,0), nmL(id,1), nmL(id,2)};	
		nn=q_delta.Rotate(nn);
		//nn=chrono::ChTransform<>::TransformParentToLocal(nn, ChVector3d(0, 0, 0) ,  q_delta);
		n<< nn[0], nn[1], nn[2];				
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
	//
	/*
	ChVectorDynamic<> mstress;
	this->ComputeStress(facet, ind, jnd, mstress);
	//std::cout<<"stress: \n"<<mstress<<std::endl;
	*/
	ChVectorDynamic<> mstress;	
	ChVectorDynamic<> dmstrain;
	ChVectorDynamic<> statev;
	this->ComputeStrain(facet, ind, jnd, dmstrain);
	//std::cout<<"strain_INC: "<<dmstrain(0)<<"\t"<<dmstrain(1)<<"\t"<<dmstrain(2)<<"\t";
	statev=facet->Get_StateVar();	
    	//    	  	
	facet->Get_material()->ComputeStress( dmstrain, length,  epsV, statev, mstress, area);
	facet->Set_StateVar(statev);	
	
	Fi.segment(ind*6,6)+= area*length*(mstress.transpose()*mBI);
	Fi.segment(jnd*6,6)+= -area*length*(mstress.transpose()*mBJ);
	//
	//
	//	
	iface++;
	
    }
    
    // Fi = C * Fi_local  with C block-diagonal rotations A
    //ChMatrixCorotation::ComputeCK(FiK_local, this->A, 4, Fi);
}

ChStrainTensor<> ChElementLDPM::GetStrain() {
    // set up vector of nodal displacements (in local element system) u_l = R*p - p0
    ChVectorDynamic<> displ(12);
    this->GetStateBlock(displ);  // nodal displacements, local

    ChStrainTensor<> mstrain = MatrB * displ;
    return mstrain;
}

ChStressTensor<> ChElementLDPM::GetStress() {
     ChMatrixDynamic<> StressStrainMatrix;
     StressStrainMatrix.setZero(6, 6);
     //StressStrainMatrix=this->Material->Get_StressStrainMatrix()
    ChStressTensor<> mstress =  StressStrainMatrix * this->GetStrain();
    return mstress;
}

void ChElementLDPM::ComputeNodalMass() {
    nodes[0]->m_TotalMass += this->GetVolume() * this->Material->Get_density() / 4.0;
    nodes[1]->m_TotalMass += this->GetVolume() * this->Material->Get_density() / 4.0;
    nodes[2]->m_TotalMass += this->GetVolume() * this->Material->Get_density() / 4.0;
    nodes[3]->m_TotalMass += this->GetVolume() * this->Material->Get_density() / 4.0;
}

void ChElementLDPM::LoadableGetStateBlockPosLevel(int block_offset, ChState& mD) {
    mD.segment(block_offset + 0, 3) = nodes[0]->GetPos().eigen();
    mD.segment(block_offset + 3, 4) = nodes[0]->GetRot().eigen();
    //
    mD.segment(block_offset + 7, 3) = nodes[1]->GetPos().eigen();
    mD.segment(block_offset + 10, 4) = nodes[1]->GetRot().eigen();
    //
    mD.segment(block_offset + 14, 3) = nodes[2]->GetPos().eigen();
    mD.segment(block_offset + 17, 4) = nodes[2]->GetRot().eigen();
    //
    mD.segment(block_offset + 21, 3) = nodes[3]->GetPos().eigen();
    mD.segment(block_offset + 24, 4) = nodes[3]->GetRot().eigen();
}

void ChElementLDPM::LoadableGetStateBlockVelLevel(int block_offset, ChStateDelta& mD) {
    mD.segment(block_offset + 0, 3) = nodes[0]->GetPosDt().eigen();
    mD.segment(block_offset + 3, 3) = nodes[0]->GetAngVelLocal().eigen();
    //
    mD.segment(block_offset + 6, 3) = nodes[1]->GetPosDt().eigen();
    mD.segment(block_offset + 9, 3) = nodes[1]->GetAngVelLocal().eigen();
    //
    mD.segment(block_offset + 12, 3) = nodes[2]->GetPosDt().eigen();
    mD.segment(block_offset + 15, 3) = nodes[2]->GetAngVelLocal().eigen();
    //
    mD.segment(block_offset + 18, 3) = nodes[3]->GetPosDt().eigen();
    mD.segment(block_offset + 21, 3) = nodes[3]->GetAngVelLocal().eigen();
}



void ChElementLDPM::LoadableStateIncrement(const unsigned int off_x,
                                                   ChState& x_new,
                                                   const ChState& x,
                                                   const unsigned int off_v,
                                                   const ChStateDelta& Dv) {
    for (int i = 0; i < 4; ++i) {
        nodes[i]->NodeIntStateIncrement(off_x + i * 7, x_new, x, off_v + i * 6, Dv);        
    }
}

void ChElementLDPM::LoadableGetVariables(std::vector<ChVariables*>& mvars) {
    for (int i = 0; i < nodes.size(); ++i)
        mvars.push_back(&this->nodes[i]->Variables());
}

void ChElementLDPM::ComputeNF(const double U,
                                      const double V,
                                      const double W,
                                      ChVectorDynamic<>& Qi,
                                      double& detJ,
                                      const ChVectorDynamic<>& F,
                                      ChVectorDynamic<>* state_x,
                                      ChVectorDynamic<>* state_w) {
    // evaluate shape functions (in compressed vector), btw. not dependent on state
    // note: U,V,W in 0..1 range, thanks to IsTetrahedronIntegrationNeeded() {return true;}
    ShapeVector N;
    this->ShapeFunctions(N, U, V, W);

    detJ = 6 * this->GetVolume();

    Qi(0) = N(0) * F(0);
    Qi(1) = N(0) * F(1);
    Qi(2) = N(0) * F(2);
    Qi(3) = N(1) * F(0);
    Qi(4) = N(1) * F(1);
    Qi(5) = N(1) * F(2);
    Qi(6) = N(2) * F(0);
    Qi(7) = N(2) * F(1);
    Qi(8) = N(2) * F(2);
    Qi(9) = N(3) * F(0);
    Qi(10) = N(3) * F(1);
    Qi(11) = N(3) * F(2);
}


double ChElementLDPM::ComputeTetVol(ChVector3d p1, ChVector3d p2, ChVector3d p3, ChVector3d p4){
    double tetvol=0;
    tetvol = p2[0]*(p3[1]*p4[2]-p4[1]*p3[2])-p3[0]*(p2[1]*p4[2]-p4[1]*p2[2])+p4[0]*(p2[1]*p3[2]-p3[1]*p2[2]);
    tetvol = tetvol-(p3[0]*(p4[1]*p1[2]-p1[1]*p4[2])-p4[0]*(p3[1]*p1[2]-p1[1]*p3[2])+p1[0]*(p3[1]*p4[2]-p4[1]*p3[2]));
    tetvol = tetvol+p4[0]*(p1[1]*p2[2]-p2[1]*p1[2])-p1[0]*(p4[1]*p2[2]-p2[1]*p4[2])+p2[0]*(p4[1]*p1[2]-p1[1]*p4[2]);
    tetvol = tetvol-(p1[0]*(p2[1]*p3[2]-p3[1]*p2[2])-p2[0]*(p1[1]*p3[2]-p3[1]*p1[2])+p3[0]*(p1[1]*p2[2]-p2[1]*p1[2]));
    tetvol = abs(tetvol)/6.0;
	return tetvol;
}

ChMatrixNM<double,6,6> ChElementLDPM::ComputeTetMassN(std::shared_ptr<ChSectionLDPM> section, ChVector3d pN, ChVector3d pC, ChVector3d pA, ChVector3d pB){
	ChMatrixNM<double,6,6> MN;
	MN.setZero();
	double density=section->Get_material()->Get_density();
	//
	ChMatrix33<double> coef={{0.1, 0.05, 0.05},{0.05, 0.1, 0.05},{0.05, 0.05, 0.1}};
	//
	
	double vol=this->ComputeTetVol(pN, pC, pA, pB);
	double tetmass=vol*density;
	ChVector3d  pG=(pN+pC+pA+pB)/4.0;
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
	if (false){ //lumped mass
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


void ChElementLDPM::ComputeMmatrixGlobal(ChMatrixRef M) {
    M.setZero();
    // Mass Matrix 
    
	
	
	auto vertices=this->V_vert_nodes;
	unsigned int iface=0;
	for(auto verts: vertices){	
		unsigned int ind=facetNodeNums(iface,0);
    		unsigned int jnd=facetNodeNums(iface,1);
    		//
    		ChVector3d pNA=this->nodes[ind]->GetX0().GetPos();
		ChVector3d pNB=this->nodes[jnd]->GetX0().GetPos();
    		//
		auto pC=verts[0]->GetX0().GetPos();
		auto pA=verts[1]->GetX0().GetPos();
		auto pB=verts[2]->GetX0().GetPos();		
		//
		ChMatrixNM<double,6,6> mA=this->ComputeTetMassN(this->GetFacetI(iface), pNA, pC, pA, pB);
		ChMatrixNM<double,6,6> mB=this->ComputeTetMassN(this->GetFacetI(iface), pNB, pC, pA, pB);
		M.block<6,6>(ind*6,ind*6)+=mA;
		M.block<6,6>(jnd*6,jnd*6)+=mB;
		//
		iface++;
	}
	
}


void ChElementLDPM::EleIntLoadLumpedMass_Md(ChVectorDynamic<>& Md, double& error, const double c){
        ChMatrixDynamic<> Mloc(GetNumCoordsPosLevel(), GetNumCoordsPosLevel()); 
	this->ComputeMmatrixGlobal(Mloc);			
	
	ChVectorDynamic<> dMi = c * Mloc.diagonal();
    
    	error = Mloc.sum() - Mloc.diagonal().sum();	
	
    	int stride = 0;
    	for (int in = 0; in < GetNumNodes(); in++) {
		int node_dofs = GetNodeNumCoordsPosLevelActive(in);		
		if (!GetNode(in)->IsFixed()){		    
		    Md.segment(GetNode(in)->NodeGetOffsetVelLevel(), node_dofs) += dMi.segment(stride, node_dofs);
		}
		stride += GetNodeNumCoordsPosLevel(in);
    	}      	
    	         
    
}


chrono::ChMatrixNM<int,12,2> fill(){
chrono::ChMatrixNM<int,12,2> facetNodeNums;
facetNodeNums << 0, 1, 0, 1, 0, 2, 0, 2, 0, 3, 0, 3, 1, 2, 1, 2, 1, 3, 1, 3, 2, 3, 2, 3;
return facetNodeNums;
}

chrono::ChMatrixNM<int,12,2> ChElementLDPM::facetNodeNums=fill();



}  // end namespace ldpm
}  // end namespace chrono
