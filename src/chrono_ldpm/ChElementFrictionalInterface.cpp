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

#include "chrono_ldpm/ChElementFrictionalInterface.h"
#include "chrono/utils/ChUtilsGeometry.h"

using namespace chrono;
using namespace chrono::fea;


namespace chrono {
namespace ldpm {

ChElementFrictionalInterface::ChElementFrictionalInterface() : spring_k(1.0), damper_r(0.01) {
    nodes.resize(4);
	Un_1.setZero(12);
	DUn_1.setZero(12);
}

ChElementFrictionalInterface::~ChElementFrictionalInterface() {}

void ChElementFrictionalInterface::SetNodes(std::shared_ptr<ChNodeFEAxyz> nodeA, std::shared_ptr<ChNodeFEAxyz> nodeB, 
							std::shared_ptr<ChNodeFEAxyz> nodeC, std::shared_ptr<ChNodeFEAxyz> nodeP) {
    nodes[0] = nodeA;
    nodes[1] = nodeB;
	nodes[2] = nodeC;
    nodes[3] = nodeP;
    std::vector<ChVariables*> mvars;
    mvars.push_back(&nodes[0]->Variables());
    mvars.push_back(&nodes[1]->Variables());
	mvars.push_back(&nodes[2]->Variables());
    mvars.push_back(&nodes[3]->Variables());
    Kmatr.SetVariables(mvars);
	
	bool is_into;
    ChVector3d p_projected;
    double d = utils::PointTriangleDistance(nodes[3]->GetPos(), nodes[0]->pos, nodes[1]->pos,  nodes[2]->pos, N2, N3, is_into, p_projected);
	N1=1.-N2-N3;
	Un_1.setZero(12);
	DUn_1.setZero(12);	
}


void ChElementFrictionalInterface::Update() {
    // parent class update:
    ChElementGeneric::Update();

    // always keep updated the rotation matrix A:
    //this->UpdateRotation();
}



void ChElementFrictionalInterface::LoadableGetStateBlockPosLevel(int block_offset, ChState& mD) {
    mD.segment(block_offset + 0, 3) = nodes[0]->GetPos().eigen();
    mD.segment(block_offset + 3, 3) = nodes[1]->GetPos().eigen();
    mD.segment(block_offset + 6, 3) = nodes[2]->GetPos().eigen();
    mD.segment(block_offset + 9, 3) = nodes[3]->GetPos().eigen();
    
}

void ChElementFrictionalInterface::LoadableGetStateBlockVelLevel(int block_offset, ChStateDelta& mD) {
    mD.segment(block_offset + 0, 3) = nodes[0]->GetPosDt().eigen();
    mD.segment(block_offset + 3, 3) = nodes[1]->GetPosDt().eigen();
    mD.segment(block_offset + 6, 3) = nodes[2]->GetPosDt().eigen();
    mD.segment(block_offset + 9, 3) = nodes[3]->GetPosDt().eigen();
}

void ChElementFrictionalInterface::LoadableStateIncrement(const unsigned int off_x,
                                                ChState& x_new,
                                                const ChState& x,
                                                const unsigned int off_v,
                                                const ChStateDelta& Dv) {
    for (int i = 0; i < 4; ++i) {
        nodes[i]->NodeIntStateIncrement(off_x + i * 3, x_new, x, off_v + i * 3, Dv);
    }
}

void ChElementFrictionalInterface::LoadableGetVariables(std::vector<ChVariables*>& mvars) {
    for (int i = 0; i < nodes.size(); ++i)
        mvars.push_back(&this->nodes[i]->Variables());
}


void ChElementFrictionalInterface::GetStateBlock(ChVectorDynamic<>& mD) {
    mD.setZero(this->GetNumCoordsPosLevel());
    mD.segment(0, 3) = this->nodes[0]->GetPos().eigen();
    mD.segment(3, 3) = this->nodes[1]->GetPos().eigen();
	mD.segment(6, 3) = this->nodes[2]->GetPos().eigen();
    mD.segment(9, 3) = this->nodes[3]->GetPos().eigen();
}

void ChElementFrictionalInterface::ComputeKRMmatricesGlobal(ChMatrixRef H, double Kfactor, double Rfactor, double Mfactor) {
    assert((H.rows() == 12) && (H.cols() == 12));
	H.block(0, 0, 12, 12).setZero();
	if (Kfactor){
		double commonfactor = this->spring_k*Kfactor;	
		ChVector3d posA = nodes[0]->GetPos();
		ChVector3d posB = nodes[1]->GetPos();
		ChVector3d posC = nodes[2]->GetPos();
		ChVector3d posP = nodes[3]->GetPos();
		
		// Triangle normal and tangents
		ChVector3d e1 = posB - posA;
		ChVector3d e2 = posC - posA;
		ChVector3d n = Vcross(e2, e1).GetNormalized();
		ChVector3d t1 = e1.GetNormalized();
		ChVector3d t2 = Vcross(n, t1).GetNormalized();
		
		Eigen::Matrix3d P = commonfactor * (
			n.eigen() * n.eigen().transpose() +
			t1.eigen() * t1.eigen().transpose() +
			t2.eigen() * t2.eigen().transpose()    
		);			
		
		// Fill the 3x3 blocks
		std::vector<double> N = {N1, N2, N3};
		for (int i = 0; i < 3; ++i) {
			for (int j = 0; j < 3; ++j) {
				H.block<3,3>(i*3, j*3) += N[i] * N[j] * P;
			}
			H.block<3,3>(i*3, 9) -= N[i] * P;     // interaction with P
			H.block<3,3>(9, i*3) -= N[i] * P;     // interaction with P
		}
		H.block<3,3>(9,9) += P;		
				
	}	
   
}


void ChElementFrictionalInterface::ComputeInternalForces(ChVectorDynamic<>& Fi) {
		assert(Fi.size() == 12);
		ChVectorDynamic<> mD;
		GetStateBlock(mD);
		//std::cout<<"mD: "<<mD<<std::endl;
		DUn_1=mD-Un_1;
		Un_1=mD;
		// Get the positions of the nodes
        ChVector3d posA = nodes[0]->GetPos();
        ChVector3d posB = nodes[1]->GetPos();
        ChVector3d posC = nodes[2]->GetPos();
        ChVector3d posP = nodes[3]->GetPos();
		
		//std::cout<<"posA: "<<posA<<"\t";
		//std::cout<<"posB: "<<posB<<"\t";
		//std::cout<<"posC: "<<posC<<"\t";
		//
		/*ChVector3d posA0 = nodes[0]->GetX0();
        ChVector3d posB0 = nodes[1]->GetX0();
        ChVector3d posC0 = nodes[2]->GetX0();
        ChVector3d posP0 = nodes[3]->GetX0();*/
		
        // Triangle normal and tangents
        ChVector3d e1 = posB - posA;
        ChVector3d e2 = posC - posA;
        ChVector3d n = Vcross(e1, e2).GetNormalized();
		//
		
			ChVector3d t1 = e1.GetNormalized();
			ChVector3d t2 = Vcross(n, t1).GetNormalized();		
		   // Relative displacement at the center node (P)
			ChVector3d vP = DUn_1.segment(9, 3); //(posP-posP0);
			ChVector3d vABC = (DUn_1.segment(0, 3) * N1 + DUn_1.segment(3, 3) * N2 + DUn_1.segment(6, 3) * N3 ) ;		
			ChVector3d v_rel = (vP - vABC);		
			// Displacement components in local link directions		
			double vn = Vdot(v_rel, n);
			double vt1 = Vdot(v_rel, t1);
			double vt2 = Vdot(v_rel, t2);	
			slip_t += pow(vt1*vt1+vt2*vt2,0.5);
			//std::cout<<"N1: "<<N1<<"\tN2: "<<N2<<"\tN3: "<<N3<<std::endl;
			//std::cout<<"n: "<<n<<"\tt1: "<<t1<<"\tt2: "<<t2<<std::endl;
			//std::cout<<"vP: "<<vP<<"\tvABC: "<<vABC<<"\tv_rel: "<<v_rel<<std::endl;
			
			// Normal force estimation 
			double F_normal = std::min(0.0,Vdot(Force, n) + spring_k * vn );
			//ChVector3d reaction=mconstraint-> GetReactionOnNode();			
			// tangential forces estimation
			double F_t1 =  Vdot(Force, t1) + spring_k  * vt1;
			double F_t2 =  Vdot(Force, t2) + spring_k  * vt2;
			//
			double mu = GetCurrentFrictionCoefficient(slip_t);	
			//Length of tangential forces
			double ft = pow(F_t1*F_t1+F_t2*F_t2, 0.5);
			// Allowed maximum tangential force
			double ft_max = mu * std::max(0.,-F_normal);			
			//
			// Apply Coulomb's Law for friction
			if (ft > ft_max && ft > 1e-10) {								
				F_t1 *= ft_max / ft;
				F_t2 *= ft_max / ft;			
			}
			//
			// convert interaction forces from local to global axis
			Force = F_normal *n + F_t1 * t1 +  F_t2 * t2;
			//
			Fi.segment(0, 3) = N1 * Force.eigen();
			Fi.segment(3, 3) = N2 * Force.eigen();	
			Fi.segment(6, 3) = N3 * Force.eigen();
			Fi.segment(9, 3) = -Force.eigen();
		
}



ChVector3d ChElementFrictionalInterface::GetCurrentForce() {    
    return Force;
}

bool ChElementFrictionalInterface::CreateInteractionNodeToTriFace(ChSystem& sys, std::shared_ptr<ChMesh>& my_mesh, std::shared_ptr<ChMeshSurface> m_surface, std::shared_ptr<ChNodeFEAxyz> m_node, double max_dist=1e-4){
	int iface=0;	
	for (const auto& face : m_surface->GetFaces()) {  
		auto face_tetra = std::dynamic_pointer_cast<ChTetrahedronFace>(face);		
        if ( face_tetra ) {
				iface++;
				
				double val, u, v, w;
				bool is_into;
				ChVector3d p_projected;
				//
				// Get face nodes
				//
				auto node0 = std::static_pointer_cast<ChNodeFEAxyz>(face_tetra->GetNode(0));
				auto node1 = std::static_pointer_cast<ChNodeFEAxyz>(face_tetra->GetNode(1));
				auto node2 = std::static_pointer_cast<ChNodeFEAxyz>(face_tetra->GetNode(2)); 
				//
				// coordinate of the nodes
				//
				ChVector3d p0 = node0->GetPos();
				ChVector3d p1 = node1->GetPos();
				ChVector3d p2 = node2->GetPos();            
				//
				//std::cout<< "pN: "<<m_node->GetPos()<<std::endl;
				ChVector3d pN = m_node->GetPos(); 
				
				// check if node is on the surface
				val = chrono::utils::PointTriangleDistance(
					pN, p0, p1, p2, u, v, is_into, p_projected);
				
				val = fabs(val);
				
				w = 1 - u - v;
				if (!is_into)                
					val += std::max(0.0, -u) + std::max(0.0, -v) + std::max(0.0, -w);
				
				if (val < max_dist) {  					
									
					//
					// create fritional interface element between nodeP and nodes of triangular surface
					//
					auto interface1 = chrono_types::make_shared<ChElementFrictionalInterface>();	
					interface1->SetNodes(node0, node1, node2, m_node);	
					interface1->SetSpringCoefficient(spring_k);  
					interface1->SetInitialFrictionCoefficient(mu0);
					interface1->SetDynamicFrictionCoefficient(mudyn);
					my_mesh->AddElement(interface1);
					
					
					//
					return true;
					
				}   	    	
    	    	continue;
    	    }
            
        }
        //// TODO: other types of elements
    
	
	
	//std::cout<<"Unseccesful "<<m_node->GetPos()<<"\n";				
    return false; 

}

void ChElementFrictionalInterface::CreateInteractionNodeToBody(ChSystem& sys, std::shared_ptr<ChMesh>& my_mesh, std::vector<std::shared_ptr<ChNodeFEAbase>> node_list, std::shared_ptr<ChBody> m_body){
	auto mmeshsurf_bot = chrono_types::make_shared<ChMeshSurface>();
		my_mesh->AddMeshSurface(mmeshsurf_bot);
		mmeshsurf_bot->AddFacesFromNodeSet(node_list);
		
		std::vector< std::shared_ptr<ChNodeFEAxyz> > bot_mid_nodes;
		for (const auto& face : mmeshsurf_bot->GetFaces()) {
			if (face) {
				auto Lface=std::dynamic_pointer_cast<ChTetrahedronFace>(face);
				// Access the vertices of the face				
				ChVector3d v1 = Lface->GetNode(0)->GetPos();  // First vertex
				ChVector3d v2 = Lface->GetNode(1)->GetPos();  // Second vertex
				ChVector3d v3 = Lface->GetNode(2)->GetPos();  // Third vertex
				//
				// create a node at the center of triangle and insert into system
				//
				ChVector3d P = (v1+v2+v3)/3;
				auto nodeP = chrono_types::make_shared<ChNodeFEAxyz>(P);				
				my_mesh->AddNode(nodeP);
				// tie the nodeP to Chbody surface				
				auto constraintP = chrono_types::make_shared<ChLinkNodeFrame>();
				constraintP->Initialize(nodeP, m_body);  // body to be connected to
				sys.Add(constraintP);
				//
				// create fritional interface element between nodeP and nodes of triangular surface
				//
				auto interface1 = chrono_types::make_shared<ChElementFrictionalInterface>();	
				interface1->SetNodes(Lface->GetNode(0), Lface->GetNode(1), Lface->GetNode(2), nodeP);	
				interface1->SetSpringCoefficient(spring_k);  
				interface1->SetInitialFrictionCoefficient(mu0);
				interface1->SetDynamicFrictionCoefficient(mudyn);
				interface1->SetConstraint(constraintP);
				my_mesh->AddElement(interface1);
				//
				
			}
		}
}


////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////

ChElementFrictionalInterfaceRot::ChElementFrictionalInterfaceRot() : spring_k(1.0), damper_r(0.01) {
    nodes.resize(4);
	Un_1.setZero(24);
	DUn_1.setZero(24);
}

ChElementFrictionalInterfaceRot::~ChElementFrictionalInterfaceRot() {}

void ChElementFrictionalInterfaceRot::SetNodes(std::shared_ptr<ChNodeFEAxyzrot> nodeA, std::shared_ptr<ChNodeFEAxyzrot> nodeB, 
							std::shared_ptr<ChNodeFEAxyzrot> nodeC, std::shared_ptr<ChNodeFEAxyzrot> nodeP) {
    nodes[0] = nodeA;
    nodes[1] = nodeB;
	nodes[2] = nodeC;
    nodes[3] = nodeP;
    std::vector<ChVariables*> mvars;
    mvars.push_back(&nodes[0]->Variables());
    mvars.push_back(&nodes[1]->Variables());
	mvars.push_back(&nodes[2]->Variables());
    mvars.push_back(&nodes[3]->Variables());
    Kmatr.SetVariables(mvars);
	
	bool is_into;
    ChVector3d p_projected;
    double d = utils::PointTriangleDistance(nodes[3]->GetPos(), nodes[0]->GetPos(), nodes[1]->GetPos(),  nodes[2]->GetPos(), N2, N3, is_into, p_projected);
	N1=1-N2-N3;
	Un_1.setZero(24);
	DUn_1.setZero(24);
	//std::cout<<"N1: "<<N1<<"\tN2: "<<N2<<"\tN3: "<<N3<<std::endl;
}


void ChElementFrictionalInterfaceRot::Update() {
    // parent class update:
    ChElementGeneric::Update();

    // always keep updated the rotation matrix A:
    //this->UpdateRotation();
}




void ChElementFrictionalInterfaceRot::LoadableGetStateBlockPosLevel(int block_offset, ChState& mD) {
   
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

void ChElementFrictionalInterfaceRot::LoadableGetStateBlockVelLevel(int block_offset, ChStateDelta& mD) {
    
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

void ChElementFrictionalInterfaceRot::LoadableStateIncrement(const unsigned int off_x,
                                                ChState& x_new,
                                                const ChState& x,
                                                const unsigned int off_v,
                                                const ChStateDelta& Dv) {
    for (int i = 0; i < 4; ++i) {
        nodes[i]->NodeIntStateIncrement(off_x + i * 7, x_new, x, off_v + i * 6, Dv);
    }
}

void ChElementFrictionalInterfaceRot::LoadableGetVariables(std::vector<ChVariables*>& mvars) {
    for (int i = 0; i < nodes.size(); ++i)
        mvars.push_back(&this->nodes[i]->Variables());
}


void ChElementFrictionalInterfaceRot::GetStateBlock(ChVectorDynamic<>& mD) {
    mD.setZero(this->GetNumCoordsPosLevel());
    mD.segment(0, 3) = this->nodes[0]->GetPos().eigen();
    mD.segment(6, 3) = this->nodes[1]->GetPos().eigen();
	mD.segment(12, 3) = this->nodes[2]->GetPos().eigen();
    mD.segment(18, 3) = this->nodes[3]->GetPos().eigen();
}

void ChElementFrictionalInterfaceRot::ComputeKRMmatricesGlobal(ChMatrixRef H, double Kfactor, double Rfactor, double Mfactor) {
    assert((H.rows() == 24) && (H.cols() == 24));
	H.block(0, 0, 24, 24).setZero();
	if (Kfactor){
		double commonfactor = this->spring_k * Kfactor;
		ChVector3d posA = nodes[0]->GetPos();
		ChVector3d posB = nodes[1]->GetPos();
		ChVector3d posC = nodes[2]->GetPos();
		ChVector3d posP = nodes[3]->GetPos();
		
		// Triangle normal and tangents
		ChVector3d e1 = posB - posA;
		ChVector3d e2 = posC - posA;
		ChVector3d n = Vcross(e2, e1).GetNormalized();
		ChVector3d t1 = e1.GetNormalized();
		ChVector3d t2 = Vcross(n, t1).GetNormalized();
		
		Eigen::Matrix3d P = commonfactor * (
			n.eigen() * n.eigen().transpose() +
			t1.eigen() * t1.eigen().transpose() +
			t2.eigen() * t2.eigen().transpose()    
		);			
		
		// Fill the 3x3 blocks
		std::vector<double> N = {N1, N2, N3};
		for (int i = 0; i < 3; ++i) {
			for (int j = 0; j < 3; ++j) {
				H.block<3,3>(i*6, j*6) += N[i] * N[j] * P;					
			}
			H.block<3,3>(i*6, 18) -= N[i] * P;     
			H.block<3,3>(18, i*6) -= N[i] * P;     
		}
		H.block<3,3>(18,18) += P;			
	}
	
	
	
   
}


void ChElementFrictionalInterfaceRot::ComputeInternalForces(ChVectorDynamic<>& Fi) {
		assert(Fi.size() == 24);
		ChVectorDynamic<> mD;
		GetStateBlock(mD);
		DUn_1=mD-Un_1;
		Un_1=mD;
			
		// Get the positions of the nodes
        ChVector3d posA = nodes[0]->GetPos();
        ChVector3d posB = nodes[1]->GetPos();
        ChVector3d posC = nodes[2]->GetPos();
        ChVector3d posP = nodes[3]->GetPos();
		//std::cout<<"PosP: "<<posP<<std::endl;
		ChVector3d posA0 = nodes[0]->GetX0().GetPos();
        ChVector3d posB0 = nodes[1]->GetX0().GetPos();
        ChVector3d posC0 = nodes[2]->GetX0().GetPos();
        ChVector3d posP0 = nodes[3]->GetX0().GetPos();
				
        // Triangle normal and tangents
        ChVector3d e1 = posB - posA;
        ChVector3d e2 = posC - posA;
        ChVector3d n = Vcross(e2, e1).GetNormalized();		
        ChVector3d t1 = e1.GetNormalized();
        ChVector3d t2 = Vcross(n, t1).GetNormalized();		
        // Relative displacement at the center node (P)
        ChVector3d vP = DUn_1.segment(18, 3); //(posP-posP0);
        ChVector3d vABC = (DUn_1.segment(0, 3) * N1 + DUn_1.segment(6, 3) * N2 + DUn_1.segment(12, 3) * N3 ) ;		
        ChVector3d v_rel = (vP - vABC);
		
        // Displacement components in local link directions		
		double vn = Vdot(v_rel, n);
        double vt1 = Vdot(v_rel, t1);
        double vt2 = Vdot(v_rel, t2);
		slip_t += pow(vt1*vt1+vt2*vt2,0.5);		
		// Normal force estimation 
        double F_normal = std::min(0.0,Vdot(Force, n) + spring_k * vn );
		//ChVector3d reaction=mconstraint-> GetReaction1().force;
		// tangential forces estimation 
		double F_t1 =  Vdot(Force, t1) + spring_k  * vt1;
		double F_t2 =  Vdot(Force, t2) + spring_k  * vt2;
		
		double mu = GetCurrentFrictionCoefficient(slip_t);
		//Length of tangential forces
        double ft = pow(F_t1*F_t1+F_t2*F_t2, 0.5);
        double ft_max = mu * std::max(0.,-F_normal); // allowed maximum tangential force
		//
        // Apply Coulomb's Law for friction
        if (ft > ft_max && ft > 1e-10) {	
            F_t1 *= ft_max / ft;
			F_t2 *= ft_max / ft;			
        }
		
		// convert interaction forces from local to global axis
		Force = F_normal *n + F_t1 * t1 +  F_t2 * t2;
		//
		Fi.segment(0, 3) = N1 * Force.eigen();
		Fi.segment(3, 3).setZero();
		Fi.segment(6, 3) = N2 * Force.eigen();	
		Fi.segment(9, 3).setZero();
		Fi.segment(12, 3) = N3 * Force.eigen();
		Fi.segment(15, 3).setZero();
		Fi.segment(18, 3) = -Force.eigen();	
		Fi.segment(21, 3).setZero();
}

ChVector3d ChElementFrictionalInterfaceRot::GetCurrentForce() {    
    return Force;
}

bool ChElementFrictionalInterfaceRot::CreateInteractionNodeToTriFace(ChSystem& sys, std::shared_ptr<ChMesh>& my_mesh, std::shared_ptr<ChMeshSurface> m_surface, std::shared_ptr<ChNodeFEAxyzrot> m_node, double max_dist=1e-4){
	int iface=0;
	for (const auto& face : m_surface->GetFaces()) {  
		auto face_tetra = std::dynamic_pointer_cast<chrono::ldpm::ChLDPMFace>(face);		
        if ( face_tetra ) {
				double val, u, v, w;
				bool is_into;
				ChVector3d p_projected;
				//
				iface++;
				//
				// Get face nodes
				//
				auto node0 = std::static_pointer_cast<ChNodeFEAxyzrot>(face_tetra->GetNode(0));
				auto node1 = std::static_pointer_cast<ChNodeFEAxyzrot>(face_tetra->GetNode(1));
				auto node2 = std::static_pointer_cast<ChNodeFEAxyzrot>(face_tetra->GetNode(2)); 
				//
				// coordinate of the nodes
				//
				ChVector3d p0 = node0->GetPos();
				ChVector3d p1 = node1->GetPos();
				ChVector3d p2 = node2->GetPos();            
				//
				//std::cout<< "pN: "<<m_node->GetPos()<<std::endl;
				ChVector3d pN = m_node->GetPos();            
				// check if node is on the surface
				val = chrono::utils::PointTriangleDistance(
					pN, p0, p1, p2, u, v, is_into, p_projected);				
				
				val = fabs(val);
				
				w = 1 - u - v;
				if (!is_into)                
					val += std::max(0.0, -u) + std::max(0.0, -v) + std::max(0.0, -w);
				
				if (val < max_dist) {  					
					
					//
					// create fritional interface element between nodeP and nodes of triangular surface
					//
					auto interface1 = chrono_types::make_shared<ChElementFrictionalInterfaceRot>();	
					interface1->SetNodes(node0, node1, node2, m_node);	
					interface1->SetSpringCoefficient(spring_k);  
					interface1->SetInitialFrictionCoefficient(mu0);
					interface1->SetDynamicFrictionCoefficient(mudyn);
					my_mesh->AddElement(interface1);
					//
					
					
				}   	    	
    	    	return true;
    	    }
            
        }
        //// TODO: other types of elements
    
	
	
	//std::cout<<"Unseccesful "<<m_node->GetPos()<<"\n";				
    return false; 

}

void ChElementFrictionalInterfaceRot::CreateInteractionNodeToBody(ChSystem& sys, std::shared_ptr<ChMesh>& my_mesh, std::vector<std::shared_ptr<ChNodeFEAbase>> node_list, std::shared_ptr<ChBody> m_body){
	auto mmeshsurf_bot = chrono_types::make_shared<chrono::ldpm::ChMeshSurfaceLDPM>();
		my_mesh->AddMeshSurface(mmeshsurf_bot);
		mmeshsurf_bot->AddFacesFromNodeSet(node_list);
		int constnum=0;
		std::vector< std::shared_ptr<ChNodeFEAxyzrot> > bot_mid_nodes;
		for (const auto& face : mmeshsurf_bot->GetFaces()) {
			if (face) {
				auto Lface=std::dynamic_pointer_cast<chrono::ldpm::ChLDPMFace>(face);
				// Access the vertices of the face
				ChVector3d v1 = Lface->GetNode(0)->GetPos();  // First vertex
				ChVector3d v2 = Lface->GetNode(1)->GetPos();  // Second vertex
				ChVector3d v3 = Lface->GetNode(2)->GetPos();  // Third vertex
				//
				// create a node at the center of triangle and insert into system
				//
				ChVector3d P = (v1+v2+v3)/3;
				auto nodeP = chrono_types::make_shared<ChNodeFEAxyzrot>(ChFrame<>(P,QUNIT));
				my_mesh->AddNode(nodeP);
				// tie the nodeP to Chbody surface				
				auto constraintP = chrono_types::make_shared<ChLinkMateGeneric>(true, true, true, true, true, true);
				constraintP->Initialize(nodeP, m_body, false, nodeP->Frame(), nodeP->Frame());  // body to be connected to
				sys.Add(constraintP);
				constnum++;
				//
				// create fritional interface element between nodeP and nodes of triangular surface
				//
				auto interface1 = chrono_types::make_shared<ChElementFrictionalInterfaceRot>();	
				interface1->SetNodes(Lface->GetNode(0), Lface->GetNode(1), Lface->GetNode(2), nodeP);				
				interface1->SetSpringCoefficient(spring_k);  
				//interface1->SetConstraint(constraintP);
				interface1->SetInitialFrictionCoefficient(mu0);
				interface1->SetDynamicFrictionCoefficient(mudyn);
				my_mesh->AddElement(interface1);
				//
				
			}
		}
		std::cout<<"constnum: "<<constnum<<std::endl;
}






}  // end namespace fea
}  // end namespace chrono
