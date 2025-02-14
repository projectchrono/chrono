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

//#define BEAM_VERBOSE

//#include "chrono/fea/ChElementCurvilinearBeamIGA.h"
#include "chrono_wood/ChElementCurvilinearBeamBezier.h"
#include "chrono/core/ChVector3.h"
#include <iterator>

//#include <chrono>
//using namespace std::chrono;


namespace chrono {
namespace wood {

// for testing and debugging
ChElementCurvilinearBeamBezier::QuadratureType ChElementCurvilinearBeamBezier::quadrature_type = ChElementCurvilinearBeamBezier::QuadratureType::FULL_OVER;
double ChElementCurvilinearBeamBezier::Delta = 1e-10;
bool ChElementCurvilinearBeamBezier::LumpedMass = false;
bool ChElementCurvilinearBeamBezier::add_gyroscopic_terms = false;
bool ChElementCurvilinearBeamBezier::LargeDeflection=false;

ChElementCurvilinearBeamBezier::ChElementCurvilinearBeamBezier() {
    order = 3;
    nodes.resize(4);  // controllare se ordine = -> 2 nodi, 2 control points, o di pi?
    knots.resize(8);
    int_order_s = 1;
    int_order_b = 1;
}

void ChElementCurvilinearBeamBezier::SetNodesCubic(std::shared_ptr<ChNodeFEAxyzrot> nodeA,
                                     std::shared_ptr<ChNodeFEAxyzrot> nodeB,
                                     std::shared_ptr<ChNodeFEAxyzrot> nodeC,
                                     std::shared_ptr<ChNodeFEAxyzrot> nodeD,
                                     double knotA1,
                                     double knotA2,
                                     double knotB1,
                                     double knotB2,
                                     double knotB3,
                                     double knotB4,
                                     double knotB5,
                                     double knotB6) {
    nodes.resize(4);
    nodes[0] = nodeA;
    nodes[1] = nodeB;
    nodes[2] = nodeC;
    nodes[3] = nodeD;
    knots.resize(8);
    knots(0) = knotA1;
    knots(1) = knotA2;
    knots(2) = knotB1;
    knots(3) = knotB2;
    knots(4) = knotB3;
    knots(5) = knotB4;
    knots(6) = knotB5;
    knots(7) = knotB6;
    std::vector<ChVariables*> mvars;
    mvars.push_back(&nodes[0]->Variables());
    mvars.push_back(&nodes[1]->Variables());
    mvars.push_back(&nodes[2]->Variables());
    mvars.push_back(&nodes[3]->Variables());
    Kmatr.SetVariables(mvars);

    int_order_s = 1;
    int_order_b = 1;
}


void ChElementCurvilinearBeamBezier::SetNodesGenericOrder(std::vector<std::shared_ptr<ChNodeFEAxyzrot>> mynodes,
                                            std::vector<double> myknots,
                                            int myorder) {
    this->order = myorder;

    nodes.resize(myorder + 1);
    for (int i = 0; i < mynodes.size(); ++i) {
        nodes[i] = mynodes[i];
    }
    knots.resize(nodes.size() + myorder + 1);
    for (int i = 0; i < myknots.size(); ++i) {
        knots(i) = myknots[i];
    }

    std::vector<ChVariables*> mvars;
    for (int i = 0; i < mynodes.size(); ++i) {
        mvars.push_back(&nodes[i]->Variables());
    }
    Kmatr.SetVariables(mvars);

    // integration for in-between elements:
    // int_order_s = 1;

    // FULL OVER INTEGRATION:
    // int_order_b = myorder+1;
    // FULL EXACT INTEGRATION:
    int_order_b = (int)std::ceil((this->order + 1.0) / 2.0);
    // REDUCED INTEGRATION:
    // int_order_b = myorder;
    // SELECTIVE INTEGRATION:
    // int_order_b = 1;

    // ensure integration order is odd, to have a centered gauss point
    // if (int_order_b % 2 == 0) {
    //	int_order_b += 1;
    //}

    bool is_middle = false;
    bool is_end_A = false;
    bool is_end_B = false;

    double u1 = knots(order);
    double u2 = knots(knots.size() - order - 1);

    if (u1 < 0.5 && u2 >= 0.5) {
        // GetLog() << " -- IS_MIDDLE ";
        is_middle = true;
    }
    // Full integration for end elements:
    int multiplicity_a = 1;
    int multiplicity_b = 1;
    for (int im = order - 1; im >= 0; --im) {
        if (knots(im) == knots(order))  // extreme of span
            ++multiplicity_a;
    }
    for (int im = (int)knots.size() - order; im < (int)knots.size(); ++im) {
        if (knots(im) == knots(knots.size() - order - 1))  // extreme of span
            ++multiplicity_b;
    }
    if (multiplicity_a > 1)
        is_end_A = true;
    if (multiplicity_b > 1)
        is_end_B = true;

    if (this->quadrature_type == QuadratureType::FULL_EXACT) {
        int_order_b = (int)std::ceil((this->order + 1.0) / 2.0);
        int_order_s = (int)std::ceil((this->order + 1.0) / 2.0);
    }
    if (this->quadrature_type == QuadratureType::FULL_OVER) {
        int_order_b = myorder + 1;
        int_order_s = myorder + 1;
    }
    if (this->quadrature_type == QuadratureType::REDUCED) {
        int_order_b = myorder;
        int_order_s = myorder;
    }
    if (this->quadrature_type == QuadratureType::SELECTIVE) {
        int_order_b = 1;  // int_order_b = my_order;
        int_order_s = 1;
        if (is_end_A || is_end_B) {
            int_order_b = (int)std::ceil((this->order + 1.0) / 2.0);
            int_order_s = (int)std::ceil((this->order + 1.0) / 2.0);
        }
    }
    if (this->quadrature_type == QuadratureType::CUSTOM1) {
        int_order_b = 1;
        int_order_s = 1;
        if (order == 2) {
            if (is_middle) {
                int_order_s = 2;
                int_order_b = 2;
            }
        }
        if (order >= 3) {
            if (is_end_A || is_end_B) {
                int_order_b = order - 1;
                int_order_s = order - 1;
            }
        }
    }
    if (this->quadrature_type == QuadratureType::URI2) {
        int_order_b = 1;
        int_order_s = 1;
        if (order == 2) {
            if (is_end_B) {
                int_order_s = 2;
                int_order_b = 2;
            }
        }
        if (order >= 3) {
            if (is_end_A || is_end_B) {
                int_order_b = order - 1;
                int_order_s = order - 1;
            }
        }
    }

    // TRICK - FOR THE MOMENT ENSURE NO SELECTIVE CAUSE NOT YET IMPLEMENTED DIFFERENT GAUSS PTS FOR BEND/SHEAR
    int_order_s = int_order_b;

    /*
    GetLog() << "Element order p=" << this->order << ",   u in (" << u1 << ", " << u2 << ")";
    GetLog() << "   orders:  b=" << int_order_b << "   s=" << int_order_s;
    GetLog() << "\n";
    */
}





void ChElementCurvilinearBeamBezier::Update() {
    // parent class update:
    ChElementGeneric::Update();
	
    // always keep updated the rotation matrix A:
    this->UpdateRotation();
    this->ComputeStiffnessMatrix();
	
	if(this->macro_strain){
		for (int ig = 0; ig < int_order_b; ++ig){ 
			this->ComputeEigenStrain(ig, this->macro_strain);				
		}
	}
}


ChMatrix33<> ChElementCurvilinearBeamBezier::FrenetSerret(std::vector<std::shared_ptr<ChNodeFEAxyzrot>>& nodes, ChVector3d& jacob, double invJacob, ChVectorDynamic<>& RR, ChVectorDynamic<>& dRdxi, ChVectorDynamic<>& ddRddxi, ChVectorDynamic<>& dddRdddxi, double& kappa, double& tau){
	//
	//
	//std::cout<<"jacob: "<<jacob<<" invJacob: "<<invJacob<<std::endl;
	//std::cout<<"ddRddxi: "<<ddRddxi<<std::endl;
	//std::cout<<"dddRdddxi: "<<dddRdddxi<<std::endl;
	//
	//
	ChVector3d tngt;
	ChVector3d nrml;
	ChVector3d bnrml;	
	//	
	ChVector3d dxxdxixi;
	ChVector3d dxxxdxixixi;
	int ind=0;
	for (auto node:nodes){
		ChVector3d pos=node->GetPos();
		//std::cout<<"Node pos: "<<pos<<std::endl;
		for (int jnd=0; jnd<3; jnd++){
		//std::cout<<"pos X dRdx: "<<pos*dRdx<<std::endl;
		dxxdxixi[jnd]+=pos[jnd]*ddRddxi[ind];
		dxxxdxixixi[jnd]+=pos[jnd]*dddRdddxi[ind];
		}
		ind++;
	}
	
	//std::cout<<"dxxdxixi: "<<dxxdxixi<<std::endl;
	//std::cout<<"dxxxdxixixi: "<<dxxxdxixixi<<std::endl;
	
    tngt=invJacob*jacob; 
    tngt.Normalize();
    double invJacob2=invJacob*invJacob;
	//std::cout<<"tngt: "<<tngt<<std::endl;
    //    
    nrml=invJacob2*(dxxdxixi-(jacob^dxxdxixi)*jacob*invJacob2);
	
	//curvature of the mid curve:
    kappa=nrml.Length();	
    //std::cout<<"kappa: "<<kappa<<std::endl;

	
	ChVector3d Vx, Vy, Vz;
		XdirToDxDyDz(tngt, VECT_Y, Vx, bnrml, nrml);
	//std::cout<<"Vx: "<<Vx<<"\nVy: "<<nrml<<"\nVz: "<<bnrml<<std::endl;
	
	/*
	if (kappa > 1.0E-10){  // Curved beam    
    	// normalize the vector
    	nrml.Normalize();   
        bnrml=tngt.Cross(nrml);		
        bnrml.Normalize();			
    }else{ // Straight beam
    	//int endN=nodes.size()-1;    	 
	//ChVector3d dL=nodes[endN]->GetPos()-nodes[0]->GetPos();	 
        //tngt=dL/dL.Length();                
        //
        // Choose an arbitrary vector perpendicular to the tangent
    	nrml = tngt.Cross(VECT_Y); //Vcross(tngt,VECT_X); //VECT_X.Cross(tngt);    	
    	if (nrml.Length() < 1e-10) {
        	nrml = tngt.Cross(VECT_X); // Vcross(tngt,VECT_Y); //VECT_Y.Cross(tngt);
    	}
    	nrml.Normalize();
    	// Complete the basis by taking the cross product of the tangent and normal
    	bnrml =tngt.Cross(nrml);
    }*/
	

    //kappa=abs(jacob(1)*hessian(2)-jacob(2)*hessian(1))*invJacob^3;
   /*if (kappa > 1.0E-12){  // Curved beam    
    	// normalize the vector
		//std::cout<<"curved nrml: "<<nrml<<std::endl;
    	nrml.Normalize();   
        bnrml=tngt.Cross(nrml);		
        bnrml.Normalize();	
		
    }else{ // Straight beam
		//kappa = 0;
    	//int endN=nodes.size()-1;    	 
	    //ChVector3d dL=nodes[endN]->GetPos()-nodes[0]->GetPos();	 
        //tngt=dL/dL.Length();                
        //
				
		
		ChVector3d reference_vector;
		if (std::abs(tngt.x())<1 ) {
			reference_vector=VECT_X;
		}else{
			reference_vector=VECT_Y;
		}
		
		nrml = tngt.Cross(reference_vector);
        
		if (nrml.Length()<1E-10 ) {
			reference_vector=VECT_Z;
			nrml=tngt.Cross(reference_vector);
		}
		
        //std::cout<<"New nrml:  "<<nrml<<std::endl;
        
		//std::cout<<"Straight nrml: "<<nrml<<std::endl;			
    	// nrml = VECT_Y.Cross(tngt); //tngt.Cross(VECT_Y); //Vcross(tngt,VECT_X); //VECT_X.Cross(tngt);  
    	// if (nrml.Length() < 1e-10) {
        	// nrml = tngt.Cross(VECT_Z); // Vcross(tngt,VECT_Y); //VECT_Y.Cross(tngt);
    	// }
    	nrml.Normalize();
    	// Complete the basis by taking the cross product of the tangent and normal
    	bnrml =tngt.Cross(nrml);		
    } 
	*/
		 
	// torsion of the mid curve
    ChVector3d m1=jacob%dxxdxixi;
    double Lm1=m1.Length();
    if (Lm1==0)
        tau=0;
    else
        tau=(m1^dxxxdxixixi)/(Lm1*Lm1);  	
	
	ChMatrix33<> tnb; 
    tnb.block<1, 3>(0, 0) << tngt[0], tngt[1], tngt[2];
	tnb.block<1, 3>(1, 0) << nrml[0], nrml[1], nrml[2];
	tnb.block<1, 3>(2, 0) << bnrml[0], bnrml[1], bnrml[2];	
	/*std::cout<<"tnb: "<<tngt<<"\t";
	std::cout<<nrml<<"\t";
	std::cout<<bnrml<<"\t";
	std::cout<<"kappa: "<<kappa<<"  tau: "<<tau<<std::endl;*/
	
	return tnb;
}


void ChElementCurvilinearBeamBezier::UpdateRotation() {
    //
    // 
    for (int ig = 0; ig < int_order_b; ++ig) {
    
    	ChMatrix33<> A0(Qpoint_ref_rot[ig]);	
	ChMatrix33<> Aabs;
	ChQuaternion<> q_gp_abs_rot;	
	if (!LargeDeflection) {		
		Aabs = A0;
		Qpoint_abs_rot[ig] = Qpoint_ref_rot[ig];
	}else {
	    	
	//std::cout<<"\n---------------------------------------igauss: "<<ig<<std::endl;
        // absyssa in typical -1,+1 range:
        double eta = ChQuadrature::GetStaticTables()->Lroots[int_order_b - 1][ig];
        // absyssa in span range:
        //double u = (c1 * eta + c2);
        // scaling = gauss weight
        double w = ChQuadrature::GetStaticTables()->Weight[int_order_b - 1][ig];
	//std::cout<<"eta: "<<eta<<" w: "<< w <<std::endl;
        // Jacobian Jsu = ds/du
        double Jsu = this->Jacobian_b[ig];
	
        // Jacobian Jue = du/deta
        //double Jue = c1;

        // compute the basis functions N(u) at given u:
        int nspan = order;

        ChVectorDynamic<> N((int)nodes.size());  
	ChVectorDynamic<> dNdu((int)nodes.size());  

        ChBasisToolsBeziers::BasisEvaluateDeriv(this->order, eta, N, dNdu);  ///< here return N and dN/du
		
		
       // compute abs. spline gradient r'  = dr/ds
       // compute abs. spline gradient r'  = dr/ds
       // Maybe this part can be moved to update function
        ChVector3d mxele;
        ChVector3d myele;
        for (int i = 0; i < nodes.size(); ++i) {
            ChVector3d r_i = nodes[i]->GetPos();
            mxele += r_i * dNdu(i);  // dr/du = N_i'*r_i
            myele += nodes[i]->GetX0().GetRotMat().GetAxisY() * N(i);
        }
	mxele=mxele.GetNormalized();	
	myele=myele.GetNormalized();       
	
	Aabs.SetFromAxisX (mxele, myele);	
	Qpoint_abs_rot[ig] = Aabs.GetQuaternion();	
	
		
    }			   
   
    
    }
	
	//DUn_1
    
   
}


///
ChMatrixNM<double, 6, 6> ChElementCurvilinearBeamBezier::GetCurvedBeamSecDmat(int ig, std::shared_ptr<ChElasticityCosseratSimple>& melasticity){
	double A0=melasticity->A;
	double Sy=A0*this->sectionModifiers[ig][1];
	double Sz=A0*this->sectionModifiers[ig][2];
	double Iyy=A0*this->sectionModifiers[ig][3];
	double Izz=A0*this->sectionModifiers[ig][5];
	double Iyz=A0*this->sectionModifiers[ig][4];
	double J=Iyy+Izz;		
	double A=A0*this->sectionModifiers[ig][0];
	double G=melasticity->G;
	double E=melasticity->E;
	//
	ChMatrixNM<double, 6, 6> Dmat;		
	Dmat.setZero();
	Dmat(0,0)=A*E; Dmat(0,4)=-Sz*E; Dmat(0,5)=Sy*E;
	Dmat(1,1)=0.85*A*G; Dmat(1,3)=G*Sz;
	Dmat(2,2)=0.85*A*G; Dmat(2,3)=-G*Sy;
	Dmat(3,1)=Sz*G; Dmat(3,2)=-G*Sy; Dmat(3,3)=G*J;
	Dmat(4,0)=-Sz*E; Dmat(4,4)=Izz*E; Dmat(4,5)=-Iyz*E;
	Dmat(5,0)=Sy*E; Dmat(5,4)=-Iyz*E; Dmat(5,5)=Iyy*E;
	
	return Dmat;	
	
};



/// Set the integration points, for shear components and for bending components:

void ChElementCurvilinearBeamBezier::SetIntegrationPoints(int npoints_s, int npoints_b) {
    int_order_s = npoints_s;
    int_order_b = npoints_b;
}


ChMatrixDynamic<> ChElementCurvilinearBeamBezier::ComputeBMatrix(ChVectorDynamic<>& R, ChVector3d& dRdx, ChMatrix33<>& tnb ){
	int nelnod=nodes.size();	
	ChMatrixDynamic<> B(6,6*nelnod);
	B.setZero();
	
    for (int inode=0;inode<nelnod; inode++){
            int count=inode*6;
            B(0,count)=dRdx[inode]*tnb(0,0);
            B(0,count+1)=dRdx[inode]*tnb(0,1);
            B(0,count+2)=dRdx[inode]*tnb(0,2);			 			
            //
            B(1,count)=dRdx[inode]*tnb(1,0);
            B(1,count+1)=dRdx[inode]*tnb(1,1);
            B(1,count+2)=dRdx[inode]*tnb(1,2);  
            B(1,count+3)=-R(inode)*tnb(2,0);
            B(1,count+4)=-R(inode)*tnb(2,1);
            B(1,count+5)=-R(inode)*tnb(2,2);
            //
            B(2,count)=dRdx[inode]*tnb(2,0);
            B(2,count+1)=dRdx[inode]*tnb(2,1);
            B(2,count+2)=dRdx[inode]*tnb(2,2);
            B(2,count+3)=R(inode)*tnb(1,0);
            B(2,count+4)=R(inode)*tnb(1,1);
            B(2,count+5)=R(inode)*tnb(1,2);
            //
            B(3,count+3)=dRdx[inode]*tnb(0,0);
            B(3,count+4)=dRdx[inode]*tnb(0,1);
            B(3,count+5)=dRdx[inode]*tnb(0,2);
            //
            B(4,count+3)=dRdx[inode]*tnb(1,0);
            B(4,count+4)=dRdx[inode]*tnb(1,1);
            B(4,count+5)=dRdx[inode]*tnb(1,2);    
            //
            B(5,count+3)=dRdx[inode]*tnb(2,0);
            B(5,count+4)=dRdx[inode]*tnb(2,1);
            B(5,count+5)=dRdx[inode]*tnb(2,2); 
    }
	return B;
	
}


/*
ChMatrixDynamic<> ChElementCurvilinearBeamBezier::ComputeBMatrix(ChVectorDynamic<>& R, ChVector3d& dRdx, ChMatrix33<>& tnb ){
	int nelnod=nodes.size();	
	ChMatrixDynamic<> B(6,6*nelnod);
	B.setZero();
	
    for (int inode=0;inode<nelnod; inode++){
            int count=inode*6;
            B(0,count)=dRdx[inode]*tnb(0,0);
            B(0,count+1)=dRdx[inode]*tnb(0,1);
            B(0,count+2)=dRdx[inode]*tnb(0,2);			 			
            //
            B(2,count)=dRdx[inode]*tnb(2,0);
            B(2,count+1)=dRdx[inode]*tnb(2,1);
            B(2,count+2)=dRdx[inode]*tnb(2,2);  
            B(2,count+3)=-R(inode)*tnb(1,0);
            B(2,count+4)=-R(inode)*tnb(1,1);
            B(2,count+5)=-R(inode)*tnb(1,2);
            //
            B(1,count)=dRdx[inode]*tnb(1,0);
            B(1,count+1)=dRdx[inode]*tnb(1,1);
            B(1,count+2)=dRdx[inode]*tnb(1,2);
            B(1,count+3)=R(inode)*tnb(2,0);
            B(1,count+4)=R(inode)*tnb(2,1);
            B(1,count+5)=R(inode)*tnb(2,2);
            //
            B(3,count+3)=dRdx[inode]*tnb(0,0);
            B(3,count+4)=dRdx[inode]*tnb(0,1);
            B(3,count+5)=dRdx[inode]*tnb(0,2);
            //
            B(5,count+3)=dRdx[inode]*tnb(2,0);
            B(5,count+4)=dRdx[inode]*tnb(2,1);
            B(5,count+5)=dRdx[inode]*tnb(2,2);    
            //
            B(4,count+3)=dRdx[inode]*tnb(1,0);
            B(4,count+4)=dRdx[inode]*tnb(1,1);
            B(4,count+5)=dRdx[inode]*tnb(1,2); 
    }
	return B;
	
}
*/

void ChElementCurvilinearBeamBezier::GetStateBlock_NEW(ChVectorDynamic<>& mD) {
    mD.resize(nodes.size()*6);

    ChVector3d delta_rot_dir;
    double delta_rot_angle;
    for (int ind=0;ind<nodes.size();ind++){
    // Node 0, displacement (in local element frame, corotated back)
    //     d = [Atw]' Xt - [A0w]'X0
    ChVector3d displ = (nodes[ind]->GetPos()) - (nodes[ind]->GetX0().GetPos());
    mD.segment(ind*6, 3) = displ.eigen();

    // Node 0, x,y,z small rotations (in local element frame)
    
    ChQuaternion<> q_delta0 = nodes[ind]->GetRot().eigen();    
    q_delta0.GetAngleAxis(delta_rot_angle, delta_rot_dir);	
    if (delta_rot_angle > CH_PI) 
        delta_rot_angle -= CH_2PI;  // no 0..360 range, use -180..+180

    mD.segment(ind*6+3, 3) = delta_rot_angle * delta_rot_dir.eigen();
    }    
    
}




void ChElementCurvilinearBeamBezier::ComputeStiffnessMatrix() {
    assert(section);
    double h = this->length;
	
    int mrows_w = this->GetLoadableNumCoordsVelLevel();
    int mrows_x = this->GetLoadableNumCoordsPosLevel();
	//std::cout<<"mrows_w: "<<mrows_w<<" mrows_x: "<<mrows_x<<std::endl;
	//std::cout<<"int_order_b: "<<int_order_b<<std::endl;
	ChMatrixDynamic<> DB;
	stiffness.resize(mrows_w, mrows_w);
	stiffness.setZero(mrows_w, mrows_w);
    //std::cout<<"Stiffness calculation:\n";  
    // Do quadrature over the Gauss points - reuse the "b" bend Gauss points
    double ba_curvature=0;
    double ba_torsion=0;
    for (int ig = 0; ig < int_order_b; ++ig) {
	//std::cout<<"\n---------------------------------------igauss: "<<ig<<std::endl;
        // absyssa in typical -1,+1 range:
        double eta = ChQuadrature::GetStaticTables()->Lroots[int_order_b - 1][ig];
        // absyssa in span range:
        //double u = (c1 * eta + c2);
        // scaling = gauss weight
        double w = ChQuadrature::GetStaticTables()->Weight[int_order_b - 1][ig];
	//std::cout<<"eta: "<<eta<<" w: "<< w <<std::endl;
        // Jacobian Jsu = ds/du
        double Jsu = this->Jacobian_b[ig];
	
        // Jacobian Jue = du/deta
        //double Jue = c1;

        // compute the basis functions N(u) at given u:
        int nspan = order;

        ChVectorDynamic<> N((int)nodes.size());  
		ChVectorDynamic<> dNdu((int)nodes.size());  

        ChBasisToolsBeziers::BasisEvaluateDeriv(this->order, eta, N, dNdu);  ///< here return N and dN/du
		
		
		// compute abs. spline gradient r'  = dr/ds
       // compute abs. spline gradient r'  = dr/ds
	   // Maybe this part can be moved to update function
        ChVector3d dr;
        for (int i = 0; i < nodes.size(); ++i) {
            ChVector3d r_i = nodes[i]->GetPos();
            dr += r_i * dNdu(i);  // dr/du = N_i'*r_i
        }
		
	
        //ChVector3d dRdx = dNdu / Jsu;
        ChVector3d dRdx=dNdu/Jsu;
	//std::cout<<" Jacobian: "<<Jsu<<" w: "<< w <<std::endl;	
		double dmultiplier=Jsu*w;
			
		///
		///
		///
		ChVectorDynamic<> RR(order + 1), dRdu(order + 1), ddRddu(order + 1), dddRdddu(order + 1);
		ChBasisToolsBeziers::BasisEvaluateDeriv( order,                    ///< order
                eta,                 	///< parameter                
                RR,           		///< here return basis functions R evaluated at u, that is: R(u)
                dRdu,        		///< here return basis functions derivatives dR/du evaluated at u
                ddRddu,       		///< here return basis functions derivatives ddR/ddu evaluated at u
		dddRdddu);		///< here return basis functions derivatives dddR/dddu evaluated at u
				
		///
		///
		///
		//std::cout<<"RR: "<<RR<<std::endl;
		//std::cout<<"dRdu: "<<dRdu<<std::endl;
		//ChMatrix33<> tnb= FrenetSerret(nodes, dr, 1./Jsu, ddRddu, dddRdddu, ba_curvature, ba_torsion);	
		ChMatrix33<> tnb=this->GetLocalSystemOfReference(ig);
		//std::cout<<	"tnb: "<<tnb<<std::endl;	
		if(LargeDeflection){					
			ChQuaternion<> q_delta=(Qpoint_abs_rot[ig] *  Qpoint_ref_rot[ig].GetConjugate()) ;
			ChMatrix33<double> rotmat(q_delta);
			tnb=rotmat*tnb;
		}
		
		//std::cout<<"Stiff--ba_curvature: "<<ba_curvature<<std::endl;
		//std::cout<<"tnb: "<<tnb<<std::endl;
		ChMatrixDynamic<> Bmat= ComputeBMatrix(N, dRdx, tnb);
		
		//auto myelasticity=std::dynamic_pointer_cast<ChElasticityCosseratSimple>(this->section->GetElasticity());
		auto myelasticity=std::dynamic_pointer_cast<ChElasticityCosseratAdvancedGenericFPM>(this->section->GetElasticity());		
		ChMatrixNM<double, 6, 6> Dmat=myelasticity->GetStiffnessMatrix();		
			
		DB=dmultiplier*(Dmat*Bmat);		
		//std::cout<<"Bmat:\n"<<Bmat<<std::endl;		
		//std::cout<<"Dmat\n"<<Dmat<<std::endl;
		
		//std::cout<<"DB\n"<<(Dmat*Bmat)<<std::endl;
				
			
		for (int id=0;id<mrows_w;id++){			
			for (int jd=0;jd<mrows_w;jd++){				
				stiffness(id,jd)+=(Bmat.transpose().block<1, 6>(id, 0)*DB.block<6, 1>(0, jd));
				
			}
		}
		
		
		
       
	} /// End of Gauss integration loop
	//this->stiffness=Kloc;
	//std::cout<<"stiffness\n"<<stiffness<<std::endl;
	//exit(1);	
			
}



void ChElementCurvilinearBeamBezier::ComputeMmatrixGlobal(ChMatrixRef M) {
    M.setZero();
    // Mass Matrix 
    //ChMatrixDynamic<> M(6 * nodes.size(), 6 * nodes.size());    
    ChMatrix33<> Mxw;	
	
        /*if (ChElementCurvilinearBeamIGA::LumpedMass == true) {
            //
            // "lumped" M mass matrix
            //
            ChMatrixNM<double, 6, 6> sectional_mass;
            this->section->GetInertia()->ComputeInertiaMatrix(sectional_mass);
	    
            double node_multiplier = length / (double)nodes.size();
            for (int i = 0; i < nodes.size(); ++i) {
                int stride = i * 6;
                // if there is no mass center offset, the upper right and lower left blocks need not be rotated,
                // hence it can be the simple (constant) expression
                //   Mloc.block<6, 6>(stride, stride) += sectional_mass * (node_multiplier * Mfactor);
                // but the more general case needs the rotations, hence:
                M.block<3, 3>(stride, stride) += sectional_mass.block<3, 3>(0, 0) * node_multiplier ;
                M.block<3, 3>(stride + 3, stride + 3) +=
                    sectional_mass.block<3, 3>(3, 3) * node_multiplier;
                Mxw = nodes[i]->GetA() * sectional_mass.block<3, 3>(0, 3) * node_multiplier ;
                M.block<3, 3>(stride, stride + 3) += Mxw;
                M.block<3, 3>(stride + 3, stride) += Mxw.transpose();
            }
        } else {*/
            //
            // "consistent" M mass matrix, via Gauss quadrature
            //
            ChMatrixNM<double, 6, 6> sectional_mass;
            this->section->GetInertia()->ComputeInertiaMatrix(sectional_mass);
            //std::cout<< "Sectional_mass:\n"<<sectional_mass<<std::endl;
            // Do quadrature over the Gauss points - reuse the "b" bend Gauss points
	    double ba_curvature=0;
    	    double ba_torsion=0;
            for (int ig = 0; ig < int_order_b; ++ig) {
            	//std::cout<< "\n------------------------------------------------ ig: "<<ig<<std::endl;
                // absyssa in typical -1,+1 range:
                double eta = ChQuadrature::GetStaticTables()->Lroots[int_order_b - 1][ig];
                // absyssa in span range:
                //double u = (c1 * eta + c2);
                // scaling = gauss weight
                double w = ChQuadrature::GetStaticTables()->Weight[int_order_b - 1][ig];

                // Jacobian Jsu = ds/du
                double Jsu = this->Jacobian_b[ig];
                // Jacobian Jue = du/deta
                //double Jue = c1;

                // compute the basis functions N(u) at given u:
                int nspan = order;

                ChVectorDynamic<> N((int)nodes.size());
                ChVectorDynamic<> dNdu((int)nodes.size());               
                ChBasisToolsBeziers::BasisEvaluateDeriv(this->order, eta, N, dNdu);  ///< here return N and dN/du
                //std::cout<<"N: "<<N<<std::endl;
                // compute abs. spline gradient r'  = dr/ds
		ChVector3d dr;
		for (int i = 0; i < nodes.size(); ++i) {
		    ChVector3d r_i(nodes[i]->GetX0().GetPos());
		    dr += r_i * dNdu(i);  // dr/du = N_i'*r_i
		}
		
		///
		///
		///
		ChVectorDynamic<> RR(order + 1), dRdu(order + 1), ddRddu(order + 1), dddRdddu(order + 1);		
		ChBasisToolsBeziers::BasisEvaluateDeriv( 
		order,                  ///< order
                eta,                 	///< parameter                
                RR,           		///< here return basis functions R evaluated at u, that is: R(u)
                dRdu,        		///< here return basis functions derivatives dR/du evaluated at u
                ddRddu,       		///< here return basis functions derivatives ddR/ddu evaluated at u
		dddRdddu);		///< here return basis functions derivatives dddR/dddu evaluated at u
					
		///
		///
		///		
			
		//std::cout<< "w , Jsu , Jue: \t"<<w <<"\t"<<Jsu <<std::endl;
		//std::cout<< "dr: "<<dr<<std::endl;	
		//ChMatrix33<> tnb= FrenetSerret(nodes, dr, 1/Jsu, ddRddu, dddRdddu, ba_curvature, ba_torsion);
		ChMatrix33<> tnb=this->GetLocalSystemOfReference(ig);		
		if(LargeDeflection){						
			ChQuaternion<> q_delta=(Qpoint_abs_rot[ig] * Qpoint_ref_rot[ig].GetConjugate()) ;
			ChMatrix33<double> rotmat(q_delta);
			tnb=rotmat*tnb;
		}
		//std::cout<<"tnb: "<<tnb<<std::endl;
		
		//std::cout<<"sectional_mass:\n"<<sectional_mass<<std::endl;
		
		ChMatrixNM<double, 6, 6> rot_sectional_mass;
		rot_sectional_mass.setZero();		
		//rot_sectional_mass = sectional_mass;
		rot_sectional_mass.block<3, 3>(0, 0)=tnb.transpose()*sectional_mass.block<3, 3>(0, 0)*tnb;
		rot_sectional_mass.block<3, 3>(3, 3)=tnb.transpose()*sectional_mass.block<3, 3>(3, 3)*tnb;
		rot_sectional_mass.block<3, 3>(0, 3)=tnb.transpose()*sectional_mass.block<3, 3>(0, 3)*tnb;
		rot_sectional_mass.block<3, 3>(3, 0)=rot_sectional_mass.block<3, 3>(0, 3).transpose();
		
		//std::cout<<"rot_sectional_mass:\n"<<rot_sectional_mass<<std::endl;
                
                for (int i = 0; i < nodes.size(); ++i) {
                    int istride = i * 6;
                    for (int j = 0; j < nodes.size(); ++j) {                        
                        int jstride = j * 6;

                        double Ni_Nj = N(i) * N(j);
                        double gmassfactor = w * Jsu * Ni_Nj;

                        // if there is no mass center offset, the upper right and lower left blocks need not be rotated,
                        // hence it can be the simple (constant) expression
                        //   Mloc.block<6, 6>(istride + 0, jstride + 0) += gmassfactor * sectional_mass;
                        // but the more general case needs the rotations, hence:                        
                        M.block<3, 3>(istride, jstride) += rot_sectional_mass.block<3, 3>(0, 0) * gmassfactor;
                        M.block<3, 3>(istride + 3, jstride + 3) += rot_sectional_mass.block<3, 3>(3, 3) * gmassfactor;
                        //Mxw = nodes[i]->GetA() * sectional_mass.block<3, 3>(0, 3) * gmassfactor;
                        //Mxw = sectional_mass.block<3, 3>(0, 3) * gmassfactor;
                        M.block<3, 3>(istride, jstride + 3) += rot_sectional_mass.block<3, 3>(0, 3) * gmassfactor;;
                        M.block<3, 3>(istride + 3, jstride) += rot_sectional_mass.block<3, 3>(3, 0) * gmassfactor;;
                    }
                }
		//std::cout<<ig<<" Mass matrix:\n"<<M<<std::endl;
            }  // end loop on gauss points
        //}     
        
        
       if (ChElementCurvilinearBeamBezier::LumpedMass == true) {
		    int ndof;		    
			
			for (int i=0; i<nodes.size()*6; i++){
				ndof=i%6;
				for (int j=0; j<nodes.size(); j++){
					int k=j*6+ndof;
					if (i!=k){
						M(i,i) += M(k,i);        				      			
					}
				}
			}
			
        	for (int i=0; i<nodes.size()*6; i++){
        		for (int j=0; j<nodes.size()*6; j++){
        			if (i!=j){
        				//M(i,i) += M(j,i);
        				M(j,i)=0;        			
        			}
        		}
        	}
        }
        				
        		
        
        //std::cout<<" Mass matrix:\n"<<M<<std::endl;	
	
}



/// Sets H as the global stiffness matrix K, scaled  by Kfactor. Optionally, also
/// superimposes global damping matrix R, scaled by Rfactor, and global mass matrix M multiplied by Mfactor.

void ChElementCurvilinearBeamBezier::ComputeKRMmatricesGlobal(ChMatrixRef H, double Kfactor, double Rfactor, double Mfactor) {
    assert((H.rows() == 6 * nodes.size()) && (H.cols() == 6 * nodes.size()));
    assert(section);

    // BRUTE FORCE METHOD: USE NUMERICAL DIFFERENTIATION!

    //
    // The K stiffness matrix of this element span:
    //

    ChState state_x(this->GetLoadableNumCoordsPosLevel(), nullptr);
    ChStateDelta state_w(this->GetLoadableNumCoordsVelLevel(), nullptr);
    this->LoadableGetStateBlockPosLevel(0, state_x);
    this->LoadableGetStateBlockVelLevel(0, state_w);

    int mrows_w = this->GetLoadableNumCoordsVelLevel();
    int mrows_x = this->GetLoadableNumCoordsPosLevel();

    // compute Q at current speed & position, x_0, v_0
    //ChVectorDynamic<> Q0(mrows_w);
    //this->ComputeInternalForces_impl(Q0, state_x, state_w, true);  // Q0 = Q(x, v)

    //ChVectorDynamic<> Q1(mrows_w);
    //ChVectorDynamic<> Jcolumn(mrows_w);

    if (Kfactor) {
        //ChMatrixDynamic<> K(mrows_w, mrows_w);
		/*
        ChState state_x_inc(mrows_x, nullptr);
        ChStateDelta state_delta(mrows_w, nullptr);

        // Compute K=-dQ(x,v)/dx by backward differentiation
        state_delta.setZero(mrows_w, nullptr);

        for (int i = 0; i < mrows_w; ++i) {
            state_delta(i) += Delta;
            this->LoadableStateIncrement(0, state_x_inc, state_x, 0,
                                         state_delta);  // exponential, usually state_x_inc(i) = state_x(i) + Delta;

            Q1.setZero(mrows_w);
            this->ComputeInternalForces_impl(Q1, state_x_inc, state_w, true);  // Q1 = Q(x+Dx, v)
            state_delta(i) -= Delta;

            Jcolumn = (Q1 - Q0) * (-1.0 / Delta);  // - sign because K=-dQ/dx
            K.col(i) = Jcolumn;
        }
		*/
		
        // finally, store K into H:
        H.block(0, 0, mrows_w, mrows_w) = Kfactor * this->stiffness;
    } else
        H.setZero();

    //
    // The R damping matrix of this element:
    //

    if (Rfactor && this->section->GetDamping()) {
        // Compute R=-dQ(x,v)/dv by backward differentiation
        /*ChStateDelta state_w_inc(mrows_w, nullptr);
        state_w_inc = state_w;
        ChMatrixDynamic<> R(mrows_w, mrows_w);

        for (int i = 0; i < mrows_w; ++i) {
            Q1.setZero(mrows_w);

            state_w_inc(i) += Delta;
            this->ComputeInternalForces_impl(Q1, state_x, state_w_inc, true);  // Q1 = Q(x, v+Dv)
            state_w_inc(i) -= Delta;

            Jcolumn = (Q1 - Q0) * (-1.0 / Delta);  // - sign because R=-dQ/dv
            R.col(i) = Jcolumn;
        }
        H.block(0, 0, mrows_w, mrows_w) += Rfactor * R;*/
    }

    // Add inertial stiffness matrix and inertial damping matrix (gyroscopic damping),
    // if enabled in section material.
    // These matrices are not symmetric.
    // A lumped version of the inertial damping/stiffness matrix computation is used here,
    // on a per-node basis. In future one could implement also a consistent version of it, optionally, depending on
    // ChElementCurvilinearBeamIGA::LumpedMass.
    if ((Rfactor && this->section->GetInertia()->compute_inertia_damping_matrix) ||
        (Kfactor && this->section->GetInertia()->compute_inertia_stiffness_matrix)) {
        ChMatrixNM<double, 6, 6> matr_loc;
        ChMatrixNM<double, 6, 6> KRi_loc;
        KRi_loc.setZero();
	/*
        double node_multiplier_fact_R = 0.5 * length * Rfactor;
        double node_multiplier_fact_K = 0.5 * length * Kfactor;
        for (int i = 0; i < nodes.size(); ++i) {
            int stride = i * 6;
            if (this->section->GetInertia()->compute_inertia_damping_matrix) {
                this->section->GetInertia()->ComputeInertiaDampingMatrix(matr_loc, nodes[i]->GetWvel_loc());
                KRi_loc += matr_loc * node_multiplier_fact_R;
            }
            if (this->section->GetInertia()->compute_inertia_stiffness_matrix) {
                this->section->GetInertia()->ComputeInertiaStiffnessMatrix(
                    matr_loc, nodes[i]->GetWvel_loc(), nodes[i]->GetWacc_loc(),
                    (nodes[i]->GetA().transpose()) * nodes[i]->GetPosDt2());  // assume x_dtdt in local frame!
                KRi_loc += matr_loc * node_multiplier_fact_K;
            }
            // corotate the local damping and stiffness matrices (at once, already scaled) into absolute one
            // H.block<3, 3>(stride,   stride  ) += nodes[i]->GetA() * KRi_loc.block<3, 3>(0,0) *
            // (nodes[i]->GetA().transpose()); // NOTE: not needed as KRi_loc.block<3, 3>(0,0) is null by construction
            H.block<3, 3>(stride + 3, stride + 3) += KRi_loc.block<3, 3>(3, 3);
            H.block<3, 3>(stride, stride + 3) += nodes[i]->GetA() * KRi_loc.block<3, 3>(0, 3);
            // H.block<3, 3>(stride+3, stride)   +=                    KRi_loc.block<3, 3>(3,0) *
            // (nodes[i]->GetA().transpose());  // NOTE: not needed as KRi_loc.block<3, 3>(3,0) is null by construction
        }*/
    }

    //
    // The M mass matrix of this element span:
    //

    if (Mfactor) {
        ChMatrixDynamic<> Mloc(6 * nodes.size(), 6 * nodes.size());
        Mloc.setZero();
        this->ComputeMmatrixGlobal(Mloc);
        H.block(0, 0, Mloc.rows(), Mloc.cols()) += Mfactor*Mloc;
    }
}

/// Computes the internal forces (ex. the actual position of nodes is not in relaxed reference position) and set
/// values in the Fi vector.

void ChElementCurvilinearBeamBezier::ComputeInternalForces(ChVectorDynamic<>& Fi) {
    ChState mstate_x(this->GetLoadableNumCoordsPosLevel(), nullptr);
    ChStateDelta mstate_w(this->GetLoadableNumCoordsVelLevel(), nullptr);
    this->LoadableGetStateBlockPosLevel(0, mstate_x);
    this->LoadableGetStateBlockVelLevel(0, mstate_w);
    ComputeInternalForces_impl(Fi, mstate_x, mstate_w);
    //std::cout<<"Fi:\n"<<Fi<<std::endl;
}

/*
void ChElementCurvilinearBeamIGA::ComputeInternalForces_impl(ChVectorDynamic<>& Fi,
                                                  ChState& state_x,
                                                  ChStateDelta& state_w,
                                                  bool used_for_differentiation) {
    // get two values of absyssa at extreme of span
    double u1 = knots(order);
    double u2 = knots(knots.size() - order - 1);

    double c1 = (u2 - u1) / 2;
    double c2 = (u2 + u1) / 2;

    // zero the Fi accumulator
    Fi.setZero();

    // Do quadrature over the "s" shear Gauss points
    // (only if int_order_b != int_order_s, otherwise do a single loop later over "b" bend points also for shear)

    //***TODO*** maybe not needed: separate bending and shear integration points.

    // Do quadrature over the "b" bend Gauss points	
    for (int ig = 0; ig < int_order_b; ++ig) {		
        // absyssa in typical -1,+1 range:
        double eta = ChQuadrature::GetStaticTables()->Lroots[int_order_b - 1][ig];
        // absyssa in span range:
        double u = (c1 * eta + c2);
        // scaling = gauss weight
        double w = ChQuadrature::GetStaticTables()->Weight[int_order_b - 1][ig];

        // Jacobian Jsu = ds/du
        double Jsu = this->Jacobian_b[ig];
        // Jacobian Jue = du/deta
        double Jue = c1;

        // compute the basis functions N(u) at given u:
        int nspan = order;

        ChVectorDynamic<> N((int)nodes.size());  
		ChVectorDynamic<> dNdu((int)nodes.size());  
		
        ChBasisToolsBeziers::BasisEvaluateDeriv(this->order, u, N, dNdu);  ///< here return N and dN/du

        // interpolate rotation of section at given u, to compute R.
        // Note: this is approximate.
        // A more precise method: use quaternion splines, as in Kim,Kim and Shin, 1995 paper.
        ChQuaternion<> q_delta;
        ChVector3d da = VNULL;
        ChVector3d delta_rot_dir;
        double delta_rot_angle;
        for (int i = 0; i < nodes.size(); ++i) {
            ChQuaternion<> q_i(state_x.segment(i * 7 + 3, 4));
            q_delta = nodes[0]->GetRot().GetConjugate() * q_i;
            q_delta.GetAngleAxis(delta_rot_angle,
                                 delta_rot_dir);  // a_i = dir_i*angle_i (in spline local reference, -PI..+PI)
            da += delta_rot_dir * delta_rot_angle * N(i);  // a = N_i*a_i
        }
        ChQuaternion<> qda;
        qda.Q_from_Rotv(da);
        ChQuaternion<> qR = nodes[0]->GetRot() * qda;

        // compute the 3x3 rotation matrix R equivalent to quaternion above
        ChMatrix33<> R(qR);

        // compute abs. spline gradient r'  = dr/ds
        ChVector3d dr;
        for (int i = 0; i < nodes.size(); ++i) {
            ChVector3d r_i(state_x.segment(i * 7, 3));
            dr += r_i * dNdu(i);  // dr/du = N_i'*r_i
        }
        // (note r'= dr/ds = dr/du du/ds = dr/du * 1/Jsu   where Jsu computed in SetupInitial)
        dr *= 1.0 / Jsu;

        // compute abs. time rate of spline gradient  dr'/dt
        ChVector3d drdt;
        for (int i = 0; i < nodes.size(); ++i) {
            ChVector3d drdt_i(state_w.segment(i * 6, 3));
            drdt += drdt_i * dNdu(i);
        }
        drdt *= 1.0 / Jsu;

        // compute abs spline rotation gradient q' = dq/ds
        // But.. easier to compute local gradient of spin vector a' = da/ds
        da = VNULL;
        for (int i = 0; i < nodes.size(); ++i) {
            ChQuaternion<> q_i(state_x.segment(i * 7 + 3, 4));
            q_delta = qR.GetConjugate() * q_i;
            q_delta.GetAngleAxis(delta_rot_angle,
                                 delta_rot_dir);  // a_i = dir_i*angle_i (in spline local reference, -PI..+PI)
            da += delta_rot_dir * delta_rot_angle * dNdu(i);  // da/du = N_i'*a_i
        }
        // (note a= da/ds = da/du du/ds = da/du * 1/Jsu   where Jsu computed in SetupInitial)
        da *= 1.0 / Jsu;

        // compute abs rate of spline rotation gradient da'/dt
        ChVector3d dadt;
        for (int i = 0; i < nodes.size(); ++i) {
            ChQuaternion<> q_i(state_x.segment(i * 7 + 3, 4));
            ChVector3d wl_i(state_w.segment(i * 6 + 3, 3));  //  w in node csys
            ChVector3d w_i = q_i.Rotate(wl_i);               // w in absolute csys
            dadt += w_i * dNdu(i);
        }
        dadt *= 1.0 / Jsu;

        // compute local epsilon strain:  strain_e= R^t * r' - {1, 0, 0}
        ChVector3d astrain_e = R.transpose() * dr - VECT_X - this->strain_e_0[ig];

        // compute local time rate of strain:  strain_e_dt = R^t * dr'/dt
        ChVector3d astrain_e_dt = R.transpose() * drdt;

        // compute local curvature strain:  strain_k= 2*[F(q*)(+)] * q' = 2*[F(q*)(+)] * N_i'*q_i = R^t * a' = a'_local
        ChVector3d astrain_k = da - this->strain_k_0[ig];

        // compute local time rate of curvature strain:
        ChVector3d astrain_k_dt = R.transpose() * dadt;

        // compute stress n  (local cut forces)
        // compute stress_m  (local cut torque)
        ChVector3d astress_n;
        ChVector3d astress_m;
        ChBeamMaterialInternalData* aplastic_data_old = nullptr;
        ChBeamMaterialInternalData* aplastic_data = nullptr;
        std::vector<std::unique_ptr<ChBeamMaterialInternalData>> foo_plastic_data;
        if (this->section->GetPlasticity()) {
            aplastic_data_old = this->plastic_data_old[ig].get();
            aplastic_data = this->plastic_data[ig].get();
            if (used_for_differentiation) {
                // Avoid updating plastic_data_new if ComputeInternalForces_impl() called for computing
                // stiffness matrix by backward differentiation. Otherwise pollutes the plastic_data integration.
                this->section->GetPlasticity()->CreatePlasticityData(1, foo_plastic_data);
                aplastic_data = foo_plastic_data[0].get();
            }
        }
        this->section->ComputeStress(astress_n, astress_m, astrain_e, astrain_k, aplastic_data, aplastic_data_old);

        if (!used_for_differentiation) {
            this->stress_n[ig] = astress_n;
            this->stress_m[ig] = astress_m;
            this->strain_e[ig] = astrain_e;
            this->strain_k[ig] = astrain_k;
        }

        // add viscous damping
        if (this->section->GetDamping()) {
            ChVector3d n_sp;
            ChVector3d m_sp;
            this->section->GetDamping()->ComputeStress(n_sp, m_sp, astrain_e_dt, astrain_k_dt);
            astress_n += n_sp;
            astress_m += m_sp;
        }

        // compute internal force, in generalized coordinates:

        ChVector3d stress_n_abs = R * astress_n;
        ChVector3d stress_m_abs = R * astress_m;

        for (int i = 0; i < nodes.size(); ++i) {
            // -Force_i = w * Jue * Jsu * Jsu^-1 * N' * R * C * (strain_e - strain_e0)
            //          = w * Jue                * N'         * stress_n_abs
            ChVector3d Force_i = stress_n_abs * dNdu(i) * (-w * Jue);
            Fi.segment(i * 6, 3) += Force_i.eigen();

            // -Torque_i =   w * Jue * Jsu * Jsu^-1 * R_i^t * N'               * R * D * (strain_k - strain_k0) +
            //             + w * Jue * Jsu *          R_i^t * N * skew(r')^t   * R * C * (strain_e - strain_e0)
            //           =   w * Jue * R_i^t                * N'               * stress_m_abs +
            //             + w * Jue * Jsu * R_i^t          * N * skew(r')^t   * stress_n_abs
            ChQuaternion<> q_i(state_x.segment(i * 7 + 3, 4));
            ChVector3d Torque_i = q_i.RotateBack(stress_m_abs * dNdu(i) * (-w * Jue) -
                                                 Vcross(dr, stress_n_abs) * N(i) * (-w * Jue * Jsu));
            Fi.segment(3 + i * 6, 3) += Torque_i.eigen();
        }

        // GetLog() << "     gp n." << ig <<   "  J=" << this->Jacobian[ig] << "   strain_e= " << strain_e << "\n";
        // GetLog() << "                    stress_n= " << stress_n << "\n";
    }

    // Add also inertial quadratic terms: gyroscopic and centrifugal

    if (ChElementCurvilinearBeamIGA::add_gyroscopic_terms == true) {
        if (ChElementCurvilinearBeamIGA::LumpedMass == true) {
            // CASE OF LUMPED MASS - faster
            double node_multiplier = length / (double)nodes.size();
            ChVector3d mFcent_i;
            ChVector3d mTgyro_i;
            for (int i = 0; i < nodes.size(); ++i) {
                this->section->GetInertia()->ComputeQuadraticTerms(mFcent_i, mTgyro_i, state_w.segment(3 + i * 6, 3));
                ChQuaternion<> q_i(state_x.segment(i * 7 + 3, 4));
                Fi.segment(i * 6, 3) -= node_multiplier * q_i.Rotate(mFcent_i).eigen();
                Fi.segment(3 + i * 6, 3) -= node_multiplier * mTgyro_i.eigen();
            }
        } else {
            // CASE OF CONSISTENT MASS
            ChVector3d mFcent_i;
            ChVector3d mTgyro_i;
            // evaluate inertial quadratic forces using same gauss points used for bending
            for (int ig = 0; ig < int_order_b; ++ig) {
                double eta = ChQuadrature::GetStaticTables()->Lroots[int_order_b - 1][ig];
                double u = (c1 * eta + c2);
                double w = ChQuadrature::GetStaticTables()->Weight[int_order_b - 1][ig];

                double Jsu = this->Jacobian_b[ig];
                double Jue = c1;
                int nspan = order;

                ChVectorDynamic<> N((int)nodes.size());  
				ChVectorDynamic<> dNdu((int)nodes.size());  
	
                ChBasisToolsBeziers::BasisEvaluateDeriv(this->order, u, N, dNdu);  ///< here return N and dN/du

                // interpolate rotation of section at given u, to compute R.
                // Note: this is approximate.
                // A more precise method: use quaternion splines, as in Kim,Kim and Shin, 1995 paper.
                ChQuaternion<> q_delta;
                ChVector3d da = VNULL;
                ChVector3d delta_rot_dir;
                double delta_rot_angle;
                for (int i = 0; i < nodes.size(); ++i) {
                    ChQuaternion<> q_i(state_x.segment(i * 7 + 3, 4));
                    q_delta = nodes[0]->GetRot().GetConjugate() * q_i;
                    q_delta.GetAngleAxis(delta_rot_angle,
                                         delta_rot_dir);  // a_i = dir_i*angle_i (in spline local reference, -PI..+PI)
                    da += delta_rot_dir * delta_rot_angle * N(i);  // a = N_i*a_i
                }
                ChQuaternion<> qda;
                qda.Q_from_Rotv(da);
                ChQuaternion<> qR = nodes[0]->GetRot() * qda;

                ChVector3d w_sect;
                for (int i = 0; i < nodes.size(); ++i) {
                    ChQuaternion<> q_i(state_x.segment(i * 7 + 3, 4));
                    ChVector3d wl_i(state_w.segment(i * 6 + 3, 3));  //  w in node csys
                    ChVector3d w_i = q_i.Rotate(wl_i);               // w in absolute csys
                    w_sect += w_i * N(i);
                }
                w_sect = qR.RotateBack(w_sect);  // w in sectional csys

                this->section->GetInertia()->ComputeQuadraticTerms(mFcent_i, mTgyro_i, w_sect);

                for (int i = 0; i < nodes.size(); ++i) {
                    Fi.segment(i * 6, 3) -= w * Jsu * Jue * N(i) * qR.Rotate(mFcent_i).eigen();
                    Fi.segment(3 + i * 6, 3) -= w * Jsu * Jue * N(i) * mTgyro_i.eigen();
                }
            }
        }
    }  // end quadratic inertial force terms
	
}
*/

void ChElementCurvilinearBeamBezier::ComputeInternalForces_impl(ChVectorDynamic<>& Fi,
                                                  ChState& state_x,
                                                  ChStateDelta& state_w,
                                                  bool used_for_differentiation) {
    
    
    ChVectorDynamic<> displ;   
    this->GetStateBlock_NEW(displ);
    DUn_1=displ-Un_1;
    Un_1=displ;   
    // zero the Fi accumulator
    Fi.setZero();
    
    
    
    //std::cout<<"state_x: "<<state_x<<std::endl;
    //std::cout<<"stiffness: \n"<<this->GetStiffnessMatrix()<<std::endl; 
    
    //
    //auto myelasticity=std::dynamic_pointer_cast<ChElasticityCosseratSimple>(this->section->GetElasticity());   
    auto myelasticity=std::dynamic_pointer_cast<ChElasticityCosseratAdvancedGenericFPM>(this->section->GetElasticity());
    //ChVector3d strain_e, strain_k;
    // Do quadrature over the "s" shear Gauss points
    // (only if int_order_b != int_order_s, otherwise do a single loop later over "b" bend points also for shear)

    //***TODO*** maybe not needed: separate bending and shear integration points.

    // Do quadrature over the "b" bend Gauss points
    double ba_curvature=0;
    double ba_torsion=0;	
    for (int ig = 0; ig < int_order_b; ++ig) {	
    	//std::cout<<"igauss----------------------------------------------------------:\t "<<ig<<std::endl;	
    	//std::cout<<"Initial astrain_e, astrain_k: "<< this->strain_e[ig] <<"  " <<this->strain_k[ig]<<std::endl;
        // absyssa in typical -1,+1 range:
        double eta = ChQuadrature::GetStaticTables()->Lroots[int_order_b - 1][ig];
        // absyssa in span range:
        //double u = (c1 * eta + c2);
        // scaling = gauss weight
        double w = ChQuadrature::GetStaticTables()->Weight[int_order_b - 1][ig];

        // Jacobian Jsu = ds/du
        double Jsu = this->Jacobian_b[ig];
        // Jacobian Jue = du/deta
        //double Jue = c1;
		double dmultiplier=Jsu*w;
		//std::cout<<"dmultiplier: "<<dmultiplier<<std::endl;
        // compute the basis functions N(u) at given u:
        int nspan = order;

        ChVectorDynamic<> N((int)nodes.size());  
		ChVectorDynamic<> dNdu((int)nodes.size());  
		
        ChBasisToolsBeziers::BasisEvaluateDeriv(this->order, eta, N, dNdu);  ///< here return N and dN/du

        
        // compute abs. spline gradient r'  = dr/ds
        ChVector3d dr;
        for (int i = 0; i < nodes.size(); ++i) {
            ChVector3d r_i(state_x.segment(i * 7, 3));
            dr += r_i * dNdu(i);  // dr/du = N_i'*r_i
        }
        // (note r'= dr/ds = dr/du du/ds = dr/du * 1/Jsu   where Jsu computed in SetupInitial)
        //dr *= 1.0 / Jsu;
        ChVector3d dRdx=dNdu/Jsu;
        ///
		///
		///
		ChVectorDynamic<> RR(order + 1), dRdu(order + 1), ddRddu(order + 1), dddRdddu(order + 1);		
		ChBasisToolsBeziers::BasisEvaluateDeriv( order,                    ///< order
					eta,                 	///< parameter                
					RR,           		///< here return basis functions R evaluated at u, that is: R(u)
					dRdu,        		///< here return basis functions derivatives dR/du evaluated at u
					ddRddu,       		///< here return basis functions derivatives ddR/ddu evaluated at u
			dddRdddu);		///< here return basis functions derivatives dddR/dddu evaluated at u
					
		///
		///
		///
		
		//ChMatrix33<> tnb= FrenetSerret(nodes, dr, 1/Jsu, ddRddu, dddRdddu, ba_curvature, ba_torsion);	
		ChMatrix33<> tnb=this->GetLocalSystemOfReference(ig);	
		if(LargeDeflection){				
				ChQuaternion<> q_delta=(Qpoint_abs_rot[ig] *  Qpoint_ref_rot[ig].GetConjugate()) ;
				ChMatrix33<double> rotmat(q_delta);
				tnb=rotmat*tnb;
		}	
		ChMatrixDynamic<> Bmat= ComputeBMatrix(N, dRdx, tnb);
	
		//std::cout<<"tnb: "<<tnb<<std::endl;
		//std::cout<<"N: "<<N<<std::endl;
		//std::cout<<"dRdx: "<<dRdx<<std::endl;
		//std::cout<<"Bmat:\n "<<Bmat<<std::endl;
		//std::cout<<"ba_curvature: "<<ba_curvature<<std::endl;
        //
        //
        ChMatrixNM<double, 6, 6> Dmat=myelasticity->GetStiffnessMatrix();        
    	//  
		ChVectorDynamic<> dstrain=Bmat*DUn_1;
		//std::cout<<"dstrain: "<<dstrain<<std::endl;
        // compute local epsilon strain:  strain_e= R^t * r' - {1, 0, 0}
        //ChVector3d astrain_e = R.transpose() * dr - VECT_X - this->strain_e_0[ig];
		//
        ChVector3d astrain_e =dstrain.segment(0, 3);
		astrain_e += this->strain_e[ig];       
		//
        // compute local curvature strain:  strain_k= 2*[F(q*)(+)] * q' = 2*[F(q*)(+)] * N_i'*q_i = R^t * a' = a'_local
        //ChVector3d astrain_k = da - this->strain_k_0[ig];
        ChVector3d astrain_k =dstrain.segment(3, 3);
        astrain_k += this->strain_k[ig];
		
		if (!used_for_differentiation) {
            //this->stress_n[ig] = astress_n;
            //this->stress_m[ig] = astress_m;
            this->strain_e[ig] = astrain_e;
            this->strain_k[ig] = astrain_k;
        }
		
		//		
		if(this->macro_strain){
			auto nonMechanicalStrain=this->Get_nonMechanicStrain(ig);	
			astrain_e -= nonMechanicalStrain;
			//astrain_k -= nonMechanicalStrain;
			//std::cout<<"macro_strain: "<<*this->macro_strain<<"   astrain_e: "<<astrain_e<<" nonMechanicalStrain: "<<nonMechanicalStrain<<std::endl;
		}
        // compute stress n  (local cut forces)
        // compute stress_m  (local cut torque)
        ChVector3d astress_n;
        ChVector3d astress_m;
        ChBeamMaterialInternalData* aplastic_data_old = nullptr;
        ChBeamMaterialInternalData* aplastic_data = nullptr;
        std::vector<std::unique_ptr<ChBeamMaterialInternalData>> foo_plastic_data;
        if (this->section->GetPlasticity()) {
            aplastic_data_old = this->plastic_data_old[ig].get();
            aplastic_data = this->plastic_data[ig].get();
            if (used_for_differentiation) {
                // Avoid updating plastic_data_new if ComputeInternalForces_impl() called for computing
                // stiffness matrix by backward differentiation. Otherwise pollutes the plastic_data integration.
                this->section->GetPlasticity()->CreatePlasticityData(1, foo_plastic_data);
                aplastic_data = foo_plastic_data[0].get();
            }
        }
        //std::cout<<"astrain_e, astrain_k: "<< astrain_e <<"  " <<astrain_k<<std::endl;
        //std::cout<<"Dmat: "<< Dmat<<std::endl;        
        this->section->ComputeStress(astress_n, astress_m, astrain_e, astrain_k, Dmat, aplastic_data, aplastic_data_old);
	//std::cout<<"astress_n, astress_m: "<< astress_n <<"  "<< astress_m<<std::endl;
        
        if (!used_for_differentiation) {
            this->stress_n[ig] = astress_n;
            this->stress_m[ig] = astress_m;
            //this->strain_e[ig] = astrain_e;
            //this->strain_k[ig] = astrain_k;
        }
       
        // compute internal force, in generalized coordinates:

        //ChVector3d stress_n_abs = R * astress_n;
        //ChVector3d stress_m_abs = R * astress_m;
        
        //ChVector3d Force_i =Bmat.Transpose()* (stress_n_abs+stress_m_abs)
	   //std::cout<<" Bmat\n "<< Bmat <<std::endl;
	   //std::cout<<" Stress\n "<< astress_n <<" "<< astress_m <<std::endl;
        for (int i = 0; i < nodes.size(); ++i) {            
            ChVector3d Force_i =(Bmat.block<3, 3>(0, i*6)).transpose()*astress_n.eigen()*dmultiplier;            
            Fi.segment(i * 6, 3) -= Force_i.eigen();
            //
            ChVector3d Torque_i = (Bmat.block<3, 3>(0, i*6+3).transpose()*astress_n.eigen()+ Bmat.block<3, 3>(3, i*6+3).transpose()*astress_m.eigen() )*dmultiplier;
            Fi.segment(3 + i * 6, 3) -= Torque_i.eigen();            
        }
	
    }
    	/*std::cout<<"Fi_norml:\n"; 
    for (int i = 0; i < nodes.size()*6; ++i){
		std::cout<<Fi(i)<<"\t";
	}
	std::cout<<std::endl;		
    	
    	ChVectorDynamic<> Fi_st=-this->GetStiffnessMatrix()*displ;
	std::cout<<"Fi_stiff:\n";
	for (int i = 0; i < nodes.size()*6; ++i){
		std::cout<<Fi_st(i)<<"\t";
	}
	std::cout<<std::endl;
	//if (Fi(0)>0)	
	exit(1);
	//std::cout<<"\n\n";*/
	
    // Add also inertial quadratic terms: gyroscopic and centrifugal	
	
}




void ChElementCurvilinearBeamBezier::ComputeGravityForces(ChVectorDynamic<>& Fg, const ChVector3d& G_acc) {
    // no so efficient... a temporary mass matrix here:
    ChMatrixDynamic<> mM(this->GetNumCoordsPosLevel(), this->GetNumCoordsPosLevel());
    this->ComputeMmatrixGlobal(mM);

    // a vector of G accelerations (for translation degrees of freedom ie 3 xyz every 6 values)
    ChVectorDynamic<> mG(this->GetNumCoordsPosLevel());
    mG.setZero();
    for (int i = 0; i < nodes.size(); ++i) {
        mG.segment(i * 6, 3) = G_acc.eigen();
    }

    // Gravity forces as M*g, always works, regardless of the way M
    // is computed (lumped or consistent, with offset center of mass or centered, etc.)
    // [Maybe one can replace this function with a faster ad-hoc implementation in case of lumped masses.]
    Fg = mM * mG;
}

/// Evaluate N'*F , where N is some type of shape function
/// evaluated at U coordinates of the line, ranging in -1..+1
/// F is a load, N'*F is the resulting generalized load
/// Returns also det[J] with J=[dx/du,..], that might be useful in gauss quadrature.

void ChElementCurvilinearBeamBezier::ComputeNF(const double U,
                                 ChVectorDynamic<>& Qi,
                                 double& detJ,
                                 const ChVectorDynamic<>& F,
                                 ChVectorDynamic<>* state_x,
                                 ChVectorDynamic<>* state_w) {
    
    // compute the basis functions N(u) at given u:    

    ChVectorDynamic<> N((int)nodes.size());  
	ChVectorDynamic<> dNdu((int)nodes.size());  

    ChBasisToolsBeziers::BasisEvaluateDeriv(this->order, U, N, dNdu);  ///< h
	
    ChVector3d dr0;
    for (int i = 0; i < nodes.size(); ++i) {
        dr0 += nodes[i]->GetX0ref().GetPos() * dNdu(i);
    }
    detJ = dr0.Length();

    for (int i = 0; i < nodes.size(); ++i) {
        int stride = i * 6;
        Qi(stride + 0) = N(i) * F(0);
        Qi(stride + 1) = N(i) * F(1);
        Qi(stride + 2) = N(i) * F(2);
        Qi(stride + 3) = N(i) * F(3);
        Qi(stride + 4) = N(i) * F(4);
        Qi(stride + 5) = N(i) * F(5);
    }
}

/// Evaluate N'*F , where N is some type of shape function
/// evaluated at U,V,W coordinates of the volume, each ranging in -1..+1
/// F is a load, N'*F is the resulting generalized load
/// Returns also det[J] with J=[dx/du,..], that might be useful in gauss quadrature.

void ChElementCurvilinearBeamBezier::ComputeNF(const double U,
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






std::vector<double> ChElementCurvilinearBeamBezier::ComputeSectionModifiers( const double kappa, ///< curvature
                            const int order)               ///< order of integration
    {
        ChQuadratureTables* mtables = 0;
        std::vector<double>* lroots;
        std::vector<double>* weight;
        bool static_tables;

        if ((unsigned int)order <= ChQuadrature::GetStaticTables()->Lroots.size()) {
            mtables = ChQuadrature::GetStaticTables();
            lroots = &mtables->Lroots[order - 1];
            weight = &mtables->Weight[order - 1];
            static_tables = true;
        } else {
            mtables = new ChQuadratureTables(order, order);
            mtables->PrintTables();
            lroots = &mtables->Lroots[0];
            weight = &mtables->Weight[0];
            static_tables = false;
        }

        //double Xc1 = (Xb - Xa) / 2;
        //double Xc2 = (Xb + Xa) / 2;
        //double Yc1 = (Yb - Ya) / 2;
        //double Yc2 = (Yb + Ya) / 2;

       
        //
	double w;
	double mult_A=0; double mult_Sn=0; double mult_Sb=0; 
    	double mult_Inn=0; double mult_Inb=0; double mult_Ibb=0;
    	double D = this->section->GetWidth_y();
    	double Bw = this->section->GetWidth_z();    	
    	double alpha = 0; //this->section->alpha;    	
    	//
        for (unsigned int ix = 0; ix < lroots->size(); ix++){           
            for (unsigned int iy = 0; iy < lroots->size(); iy++) {                                          
                w = (weight->at(ix) * weight->at(iy))*0.250;                     
                double pnprime = D/2. * lroots->at(ix) ;
		double pbprime = Bw/2. * lroots->at(iy);  
		//
		double pn = std::cos(alpha)*pnprime - std::sin(alpha)*pbprime;
		double pb = std::sin(alpha)*pnprime + std::cos(alpha)*pbprime;  
		      
                //
                mult_A   +=  1./(1.-kappa*pn)*w;
        	mult_Sn  +=  pb/(1.-kappa*pn)*w;
        	mult_Sb  +=  pn/(1.-kappa*pn)*w;
        	mult_Inn +=  pb*pb/(1.-kappa*pn)*w;
        	mult_Inb +=  pn*pb/(1.-kappa*pn)*w;
        	mult_Ibb +=  pn*pn/(1.-kappa*pn)*w;
            }
        }      
	std::vector<double> Mmult{mult_A, mult_Sn, mult_Sb, mult_Inn, mult_Inb, mult_Ibb};
	
	
        if (!static_tables)
            delete mtables;
        
        return Mmult;
    }




/// Initial setup. Precompute mass and matrices that do not change during the simulation. In particular, compute the
/// arc-length parametrization.

void ChElementCurvilinearBeamBezier::SetupInitial(ChSystem* system) {
    assert(section);
	
    if (this->section->GetPlasticity()) {
        this->section->GetPlasticity()->CreatePlasticityData(int_order_b, this->plastic_data_old);
        this->section->GetPlasticity()->CreatePlasticityData(int_order_b, this->plastic_data);
    }
	
	

    this->length = 0;

    // get two values of absyssa at extreme of span
    double u1 = knots(order);
    double u2 = knots(knots.size() - order - 1);

    double c1 = (u2 - u1) / 2;
    double c2 = (u2 + u1) / 2;

    // Gauss points for "s" shear components:

    this->Jacobian_s.resize(int_order_s);

    for (int ig = 0; ig < int_order_s; ++ig) {
        // absyssa in typical -1,+1 range:
        double eta = ChQuadrature::GetStaticTables()->Lroots[int_order_s - 1][ig];
        // absyssa in span range:
        //double u = (c1 * eta + c2);

        // compute the basis functions N(u) at given u:
        int nspan = order;

        ChVectorDynamic<> N((int)nodes.size());  
	    ChVectorDynamic<> dNdu((int)nodes.size());  

        ChBasisToolsBeziers::BasisEvaluateDeriv(this->order, eta, N, dNdu);  ///< here return N and dN/du

        // compute reference spline gradient \dot{dr_0} = dr0/du
        ChVector3d dr0;
        for (int i = 0; i < nodes.size(); ++i) {
            dr0 += nodes[i]->GetX0ref().GetPos() * dNdu(i);
        }
        this->Jacobian_s[ig] = dr0.Length();  // J = |dr0/du|   
        
    }

    // Gauss points for "b" bend components:

    this->Jacobian_b.resize(int_order_b);
    this->strain_e_0.resize(int_order_b);
    this->strain_k_0.resize(int_order_b);
    this->stress_m.resize(int_order_b);
    this->stress_n.resize(int_order_b);
    this->strain_e.resize(int_order_b);
    this->strain_k.resize(int_order_b);

    std::vector<std::vector<double>> sectionMultiplier;			
    ChMatrix33<> Aabs;
    for (int ig = 0; ig < int_order_b; ++ig) {
        // absyssa in typical -1,+1 range:
        double eta = ChQuadrature::GetStaticTables()->Lroots[int_order_b - 1][ig];
        // absyssa in span range:
        //double u = (c1 * eta + c2);
        // scaling = gauss weight * change of range:
        double w = ChQuadrature::GetStaticTables()->Weight[int_order_b - 1][ig];

        // compute the basis functions N(u) at given u:
        int nspan = order;

        ChVectorDynamic<> N((int)nodes.size());  
		ChVectorDynamic<> dNdu((int)nodes.size());  

        ChBasisToolsBeziers::BasisEvaluateDeriv(this->order, eta, N, dNdu);  ///< here return N and dN/du

        // compute reference spline gradient \dot{dr_0} = dr0/du
        ChVector3d dr0;
        for (int i = 0; i < nodes.size(); ++i) {
            dr0 += nodes[i]->GetPos() * dNdu(i);
        }
        this->Jacobian_b[ig] = dr0.Length();  // J = |dr0/du|
		//std::cout<<"Jacobian_b[ig]: "<<Jacobian_b[ig]<<std::endl;
        // From now on, compute initial strains as in ComputeInternalForces

        // Jacobian Jsu = ds/du
        double Jsu = this->Jacobian_b[ig];
        // Jacobian Jue = du/deta
        //double Jue = c1;
        double ba_curvature, ba_torsion;
        ChVectorDynamic<> RR(order + 1), dRdu(order + 1), ddRddu(order + 1), dddRdddu(order + 1);
		ChBasisToolsBeziers::BasisEvaluateDeriv( order,                    ///< order
                eta,                 	///< parameter                
                RR,           		///< here return basis functions R evaluated at u, that is: R(u)
                dRdu,        		///< here return basis functions derivatives dR/du evaluated at u
                ddRddu,       		///< here return basis functions derivatives ddR/ddu evaluated at u
		dddRdddu);		///< here return basis functions derivatives dddR/dddu evaluated at u
        
		//std::cout<<"RR: "<<RR<<"\ndRdu: "<<dRdu<<std::endl;
		//std::cout<<"igauss: "<<ig<<"\t";
        ChMatrix33<> tnb= FrenetSerret(nodes, dr0, 1/Jsu, RR, dRdu, ddRddu, dddRdddu, ba_curvature, ba_torsion);
		this->LocalSysOfReference.push_back(tnb);
         
        // compute local epsilon strain:  strain_e= R^t * r' - {1, 0, 0}
        this->strain_e_0[ig] = {0, 0, 0};
		this->strain_e[ig] = {0, 0, 0};
        // compute local curvature strain:  strain_k= 2*[F(q*)(+)] * q' = 2*[F(q*)(+)] * N_i'*q_i = R^t * a' =
        // a'_local
        this->strain_k_0[ig] = {0, 0, 0};
		this->strain_k[ig] = {0, 0, 0};
        // as a byproduct, also compute length
        this->length += w * Jsu ;
        //
        // initial local system of reference for gauss points
        //
        ChVector3d mxele;
        ChVector3d myele;
        for (int i = 0; i < nodes.size(); ++i) {
            ChVector3d r_i = nodes[i]->GetPos();
            mxele += r_i * dNdu(i);  // dr/du = N_i'*r_i
            myele += nodes[i]->GetX0().GetRotMat().GetAxisY() * N(i);
        }
	mxele=mxele.GetNormalized();	
	myele=myele.GetNormalized();       
	
	Aabs.SetFromAxisX (mxele, myele);	
	Qpoint_ref_rot.push_back(Aabs.GetQuaternion());
	Qpoint_abs_rot.push_back(Qpoint_ref_rot[ig]);
        
    }
    this->sectionModifiers=sectionMultiplier;
    //
    ChVectorDynamic<> displ(6*nodes.size());
    this->GetStateBlock_NEW(displ);
    Un_1=displ;
    DUn_1.setZero(6*nodes.size());
    //exit(9);
    // as a byproduct, also compute total mass
    this->mass = this->length * this->section->GetInertia()->GetMassPerUnitLength();
	//	
	if(this->macro_strain){
		this->m_nonMechanicStrain.resize(int_order_b);
		this->mprojection_matrix.resize(int_order_b);
		this->ComputeProjectionMatrix();
	}
	//
    this->ComputeStiffnessMatrix();
	
	//exit(0);
}


void ChElementCurvilinearBeamBezier::ComputeProjectionMatrix() {	
	for (int ig=0; ig< int_order_b; ig++){
		ChMatrixNM<double,3,9> projection_matrix;
		//	
		ChMatrix33<double> nmL=this->GetLocalSystemOfReference(ig);
		//	
		//
		//		
		for (int i=0; i<3; i++){
			projection_matrix(i,0)=nmL(0,0)*nmL(i,0);  			// e_xx
			projection_matrix(i,1)=nmL(0,1)*nmL(i,1);			// e_yy
			projection_matrix(i,2)=nmL(0,2)*nmL(i,2);			// e_zz
			projection_matrix(i,3)=(nmL(0,0)*nmL(i,1));			// e_xy
			projection_matrix(i,4)=(nmL(0,0)*nmL(i,2));			// e_xz
			projection_matrix(i,5)=(nmL(0,1)*nmL(i,2));			// e_yz	
			projection_matrix(i,6)=(nmL(0,1)*nmL(i,0));			// e_yx
			projection_matrix(i,7)=(nmL(0,2)*nmL(i,0));			// e_zx
			projection_matrix(i,8)=(nmL(0,2)*nmL(i,1));			// e_zy	
			
		}	
		mprojection_matrix[ig]=projection_matrix;		
	}
		
};

void ChElementCurvilinearBeamBezier::ComputeEigenStrain(int ig, std::shared_ptr<ChMatrixNM<double,1,9>> macro_strain){
	ChVectorDynamic<> eigen_strain;					
	auto pp=this->GetProjectionMatrix(ig);
	eigen_strain = -pp * macro_strain->transpose();	
	ChMatrix33<double> nmL=this->GetLocalSystemOfReference(ig);		
	this->Set_nonMechanicStrain(ig, eigen_strain);				
			
}



ChMatrixNM<double, 1, 9> ChElementCurvilinearBeamBezier::ComputeMacroStressContribution(){
	ChMatrixNM<double, 1, 9> macro_stress;
	macro_stress.setZero();	
	ChVector3d stress, couples;
	for (int ig = 0; ig < int_order_b; ++ig){
		double w = ChQuadrature::GetStaticTables()->Weight[int_order_b - 1][ig];
		ChMatrix33<double> nmL=this->GetLocalSystemOfReference(ig);
		//double area=this->section->GetArea();
        double vol = this->Jacobian_b[ig]*w;
		Eigen::Matrix<double, 3, 1> stress;
		stress << this->stress_n[ig].x(), this->stress_n[ig].y(), this->stress_n[ig].z();
        //couples=this->stress_m[ig];  
		macro_stress +=(this->GetProjectionMatrix(ig)).transpose()*stress*vol;				
	}
	return macro_stress;
}

/*
void ChElementCurvilinearBeamBezier::BasisEvaluateDeriv(
                const int p,                    ///< order
                const double u,                 ///< parameter
                const ChVectorDynamic<>& Weights,///< weights
                const ChVectorDynamic<>& Knots, ///< knots
                ChVectorDynamic<>& R,           ///< here return basis functions R evaluated at u, that is: R(u)
                ChVectorDynamic<>& dRdu,        ///< here return basis functions derivatives dR/du evaluated at u
                ChVectorDynamic<>& ddRddu,       ///< here return basis functions derivatives ddR/ddu evaluated at u
				ChVectorDynamic<>& dddRdddu       ///< here return basis functions derivatives dddR/dddu evaluated at u
                ) {
        int spanU = geometry::ChBasisToolsBspline::FindSpan(p, u, Knots);

        // Compute bases N and first derivatives dNdu and second derivatives ddNddu:
        ChMatrixDynamic<>  N  (4, p + 1);
        geometry::ChBasisToolsBspline::BasisEvaluateDeriv(p, spanU, u, Knots, N);

        int i;
        int uind = spanU - p;
        double W = 0.0;
        double dWdu  = 0.0; 
        double ddWddu = 0.0; 
		double dddWdddu=0.0;

        for (i = 0; i <= p; i++) {
            double wi = Weights(uind + i);
            W     +=    N(0,i) * wi;
            dWdu  +=    N(1,i) * wi;
            ddWddu+=    N(2,i) * wi;
			dddWdddu+=    N(3,i) * wi;
        }

        double fac;
        
        for (i = 0; i <= p; i++) {
            double wi = Weights(uind + i);
            fac      = wi/(W*W);
            R(i)     = N(0,i) * wi / W;
            dRdu(i)  = (N(1,i)*W - N(0,i)*dWdu) * fac;  
            ddRddu(i)= wi*(N(2,i)/W - 2*N(1,i)*dWdu/W/W - N(0,i)*ddWddu/W/W + 2*N(0,i)*dWdu*dWdu/W/W/W) ; 
			dddRdddu(i)= wi*(N(2,i)/W - 2*N(1,i)*dWdu/W/W - N(0,i)*ddWddu/W/W + 2*N(0,i)*dWdu*dWdu/W/W/W) ; 
			
			dddRdddu(i)=N(3,i)/W+(-3*dWdu*N(2,i)-dddWdddu*W*R(i)+ddWddu*W*dRdu(i)+
			5.*R(i)*ddWddu*dWdu-2.*ddWddu*ddWddu+2.*dRdu(i)*dWdu*dWdu)/W/W+
            4.*dWdu*dWdu*(ddWddu-R(i)*dWdu)/(W*W*W);
        }
		
		
		//std::cout<<"R: "<<R<<std::endl;
		//std::cout<<"dRdu: "<<dRdu<<std::endl;
		//std::cout<<"ddRddu: "<<ddRddu<<std::endl;
		//std::cout<<"dddRdddu: "<<dddRdddu<<std::endl;
    }

*/


}  // end namespace wood
}  // end namespace chrono



