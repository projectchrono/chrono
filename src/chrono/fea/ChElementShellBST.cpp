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
// Authors: Alessandro Tasora
// =============================================================================
// A Kirchhoff triangle thin shell element with 6 node (3 for triangle, 3 for 
// neighbouring triangles) as in the BST Basic Shell Triangle (Onate et al.)
// =============================================================================

#include "chrono/core/ChException.h"
#include "chrono/physics/ChSystem.h"
#include "chrono/timestepper/ChState.h"
#include "chrono/fea/ChElementShellBST.h"
#include <cmath>


namespace chrono {
namespace fea {



ChElementShellBST::ChElementShellBST() : tot_thickness(0) {
    m_nodes.resize(6);

	n_usednodes = 6;

}

ChElementShellBST::~ChElementShellBST() {}

void ChElementShellBST::SetNodes(std::shared_ptr<ChNodeFEAxyz> node0,
                  std::shared_ptr<ChNodeFEAxyz> node1,
                  std::shared_ptr<ChNodeFEAxyz> node2,
                  std::shared_ptr<ChNodeFEAxyz> node3,
				  std::shared_ptr<ChNodeFEAxyz> node4,
			      std::shared_ptr<ChNodeFEAxyz> node5) {
    assert(node0);
    assert(node1);
    assert(node2);


    m_nodes[0] = node0;
    m_nodes[1] = node1;
    m_nodes[2] = node2;
    m_nodes[3] = node3;
	m_nodes[4] = node4;
	m_nodes[5] = node5;

	this->n_usednodes = 3;
	nodes_used_to_six[0] = 0;
	nodes_used_to_six[1] = 1;
	nodes_used_to_six[2] = 2;
	if (node3 != nullptr) {
		nodes_used_to_six[n_usednodes] = 3;
		++n_usednodes;
	}
	if (node4 != nullptr) {
		nodes_used_to_six[n_usednodes] = 4;
		++n_usednodes;
	}
	if (node5 != nullptr) {
		nodes_used_to_six[n_usednodes] = 5;
		++n_usednodes;
	}

    std::vector<ChVariables*> mvars;
    mvars.push_back(&m_nodes[0]->Variables());
    mvars.push_back(&m_nodes[1]->Variables());
    mvars.push_back(&m_nodes[2]->Variables());
	if (node3 != nullptr)
		mvars.push_back(&m_nodes[3]->Variables());
	if (node4 != nullptr)
		mvars.push_back(&m_nodes[4]->Variables());
	if (node5 != nullptr)
		mvars.push_back(&m_nodes[5]->Variables());
    Kmatr.SetVariables(mvars);
}



// -----------------------------------------------------------------------------
// Add a layer.
// -----------------------------------------------------------------------------

void ChElementShellBST::AddLayer(double thickness,
                                       double theta,
                                       std::shared_ptr<ChMaterialShellKirchhoff> material) {
    m_layers.push_back(Layer(this, thickness, theta, material));
    SetLayerZreferenceCentered();
}

void ChElementShellBST::SetLayerZreferenceCentered() {
    // accumulate element thickness.
    tot_thickness = 0;
    for (size_t kl = 0; kl < m_layers.size(); kl++) {
        tot_thickness += m_layers[kl].Get_thickness();
    }

    // Loop again over the layers and calculate the z levels of layers, by centering them
    m_layers_z.clear();
    m_layers_z.push_back(-0.5 * this->GetThickness());
    for (size_t kl = 0; kl < m_layers.size(); kl++) {
        m_layers_z.push_back(m_layers_z[kl] + m_layers[kl].Get_thickness());
    }
}

void ChElementShellBST::SetLayerZreference(double z_from_bottom) {
    // accumulate element thickness.
    tot_thickness = 0;
    for (size_t kl = 0; kl < m_layers.size(); kl++) {
        tot_thickness += m_layers[kl].Get_thickness();
    }

    // Loop again over the layers and calculate the z levels of layers, by centering them
    m_layers_z.clear();
    m_layers_z.push_back(z_from_bottom);
    for (size_t kl = 0; kl < m_layers.size(); kl++) {
        m_layers_z.push_back(m_layers_z[kl] + m_layers[kl].Get_thickness());
    }
}

// -----------------------------------------------------------------------------
// Set as neutral position
// -----------------------------------------------------------------------------

// Initial element setup.
void ChElementShellBST::SetAsNeutral() {
	for (int i = 0; i < 6; ++i) {
		if (m_nodes[i]!=nullptr)
			m_nodes[i]->SetX0(m_nodes[i]->GetPos());
	}
}

// -----------------------------------------------------------------------------
// Interface to ChElementBase base class
// -----------------------------------------------------------------------------

// Initial element setup.
void ChElementShellBST::SetupInitial(ChSystem* system) {

    // Align initial pos/rot of nodes to actual pos/rot
    SetAsNeutral();

	// Compute element area
	this->area = 0.5 * Vlength( Vcross(m_nodes[1]->GetX0() - m_nodes[0]->GetX0(), m_nodes[2]->GetX0() - m_nodes[0]->GetX0()) );
	
	// Compute cM and cI coefficients
	ChVector<> Li[3];
	Li[0] = m_nodes[2]->GetX0() - m_nodes[1]->GetX0();
	Li[1] = m_nodes[0]->GetX0() - m_nodes[2]->GetX0();
	Li[2] = m_nodes[1]->GetX0() - m_nodes[0]->GetX0();

	for (int i = 0; i < 3; ++i) {
		this->cM[i][0] = Vdot(Li[0], Li[i].GetNormalized());
		this->cM[i][1] = Vdot(Li[1], Li[i].GetNormalized());
		this->cM[i][2] = Vdot(Li[2], Li[i].GetNormalized());
	}
	ChVector<> LIi[3];
	for (int i = 0; i < 3; ++i) {
		if (m_nodes[i + 3] != nullptr) {
			LIi[0] = m_nodes[(i + 1) % 3]->GetX0() - m_nodes[(i + 2) % 3]->GetX0();
			LIi[1] = m_nodes[i + 3]->GetX0() - m_nodes[(i + 1) % 3]->GetX0();
			LIi[2] = m_nodes[(i + 2) % 3]->GetX0() - m_nodes[i + 3]->GetX0();
			this->cI[i][0] = -Vdot(LIi[0], Li[i].GetNormalized());
			this->cI[i][1] = -Vdot(LIi[1], Li[i].GetNormalized());
			this->cI[i][2] = -Vdot(LIi[2], Li[i].GetNormalized());
		}
		else {
			cI[i][0] = cI[i][1] = cI[i][2] = 0;
		}
	}

	// Compute initial side lengths
	this->l0[0] = Li[0].Length();
	this->l0[1] = Li[1].Length();
	this->l0[2] = Li[2].Length();

	// Compute orthogonal system at triangle in carthesian basis
	ChVector<> t0_1 = m_nodes[1]->GetX0() - m_nodes[0]->GetX0();
	ChVector<> t0_2 = m_nodes[2]->GetX0() - m_nodes[0]->GetX0();
	// Compute normal - and orthogonalize others
	ChVector<> t0_3 = Vcross(t0_1, t0_2).GetNormalized(); // The normal
	t0_2 = (Vcross(t0_3, t0_1)).GetNormalized();
	t0_1.Normalize();

	// Compute jacobian
	ShapeVector  N;
	this->ShapeFunctions(N, 1./3., 1./3.);
	ShapeVector  Nu;
	this->ShapeFunctionsDerivativeU(Nu, 1./3., 1./3.);
	ShapeVector  Nv;
	this->ShapeFunctionsDerivativeV(Nv, 1./3., 1./3.);

	//GetLog() << "Nu: " << Nu(0,0) << "  " << Nu(0,1) << "  " << Nu(0,2) << "\n" << "Nv: " << Nv(0,0) << "  " << Nv(0,1) << "  " << Nv(0,2) << "\n";

	ChVector<> P0_u = Nu(0,0) * m_nodes[0]->GetX0() + Nu(0,1) * m_nodes[1]->GetX0() + Nu(0,2) * m_nodes[2]->GetX0();
	ChVector<> P0_v = Nv(0,0) * m_nodes[0]->GetX0() + Nv(0,1) * m_nodes[1]->GetX0() + Nv(0,2) * m_nodes[2]->GetX0();
	//GetLog() << "t0_1= \n" << t0_1 <<  "\n";
	//GetLog() << "t0_2= \n" << t0_2 <<  "\n";
	//GetLog() << "P0_u= \n" << P0_u <<  "\n";
	//GetLog() << "P0_v= \n" << P0_v <<  "\n";

	ChMatrixNM<double, 2, 2> Jxu;
	Jxu(0, 0) = Vdot(P0_u, t0_1);
	Jxu(0, 1) = Vdot(P0_v, t0_1);
	Jxu(1, 0) = Vdot(P0_u, t0_2);
	Jxu(1, 1) = Vdot(P0_v, t0_2);
	//GetLog() << "Jxu= \n" << "  " << Jxu(0, 0) << "  " << Jxu(0, 1) << "\n" << "  " << Jxu(1, 0) << "  " << Jxu(1, 1) << "\n";

	bool invertible;
	Jxu.computeInverseWithCheck(this->Jux, invertible, 1e-10);
	if (!invertible)
		GetLog() << "Warning: a ChElementShellBST is not invertible (degenerate triangle? \n";
	//GetLog() << "Jux= \n" << "  " << Jux(0, 0) << "  " << Jux(0, 1) << "\n" << "  " << Jux(1, 0) << "  " << Jux(1, 1) << "\n";

	ShapeVector  Nx;
	this->ShapeFunctionsDerivativeX(Nx, this->Jux, 1./3., 1./3.);
	ShapeVector  Ny;
	this->ShapeFunctionsDerivativeY(Ny, this->Jux, 1./3., 1./3.);

	//GetLog() << "Nx: " << Nx(0) << "  " << Nx(1) << "  " << Nx(2) << "\n" << "Ny: " << Ny(0) << "  " << Ny(1) << "  " << Ny(2) << "\n";
	

	// Compute initial strain e0
	this->e0 = VNULL;

	// Compute initial edge bendings phi[i]
	// Use the BST method in "kinked and branched shells"
	this->k0 = VNULL;
	for (int i = 0; i < 3; ++i) {
		if (m_nodes[i + 3] != nullptr) {
			// normal of ith neighbouring triangle, opposed to ith triangle vertex:
			ChVector<> t0i_3 = Vcross(m_nodes[i + 3]->GetX0() - m_nodes[(i + 1) % 3]->GetX0(), m_nodes[(i + 2) % 3]->GetX0() - m_nodes[(i + 1) % 3]->GetX0()).GetNormalized();
			// boundary normal to shared edge:
			ChVector<> n_i = Vcross(m_nodes[(i + 1) % 3]->GetX0() - m_nodes[(i + 2) % 3]->GetX0(), t0_3).GetNormalized();
			// edge angle
			double mcos_fi = Vdot(t0_3, t0i_3);
			double msin_fi = Vdot(n_i, t0i_3);
			this->phi0[i] = atan2(msin_fi, mcos_fi);
			// curvature
			// NOT NEEDED? k here as 'change' of curvature in bent meshes, only phi0 matters.
			//this->k0 += VNULL;
			// precompute rI coefficients = RIi/(RMi/RIi) = (1/hIi)/((1/hMi)(1/hIi)) assuming heights h remains not changed
			double hIi = Vcross(m_nodes[i + 3]->GetX0() - m_nodes[(i + 1) % 3]->GetX0(), m_nodes[(i + 2) % 3]->GetX0() - m_nodes[(i + 1) % 3]->GetX0()).Length() / this->l0[i];
			double hMi = 2.0 * this->area / this->l0[i];
			this->rI[i] = (1.0 / hIi) / ((1.0 / hMi) + (1.0 / hIi));
		}
		else {
			this->phi0[i] = 0;
			this->rI[i] = 0.5;
			//this->k0 += VNULL;
		}
	}
	this->k0 *= this->area * 4.0;

    // Perform layer initialization
    for (size_t kl = 0; kl < m_layers.size(); kl++) {
        m_layers[kl].SetupInitial();
    }

}

// State update.
void ChElementShellBST::Update() {
    ChElementGeneric::Update();
}


void ChElementShellBST::EleIntLoadLumpedMass_Md(ChVectorDynamic<>& Md, double& error, const double c){

	double avg_density = this->GetDensity();
	double mass_per_area = avg_density * tot_thickness;

	// Heuristic, simplified 'lumped' mass matrix to central three nodes.
	// This is simpler than the stiffness-consistent mass matrix that would require
	// integration over gauss points.

	double nodemass = (1.0 / 3.0) * (this->area * mass_per_area);
	for (int n = 0; n < 3; n++) {
		// xyz masses of first 3 nodes of BST
		Md(m_nodes[n]->NodeGetOffsetW()  ) += nodemass * c;
		Md(m_nodes[n]->NodeGetOffsetW()+1) += nodemass * c;
		Md(m_nodes[n]->NodeGetOffsetW()+2) += nodemass * c;
	}
}



// Fill the D vector with the current field values at the element nodes.
void ChElementShellBST::GetStateBlock(ChVectorDynamic<>& mD) {
    mD.resize(this->n_usednodes * 3, 1);

	for (int i = 0; i < this->n_usednodes; ++i) {
		mD.segment(i*3, 3) = m_nodes[nodes_used_to_six[i]]->GetPos().eigen();
	}
   
}

// Calculate the global matrix H as a linear combination of K, R, and M:
//   H = Mfactor * [M] + Kfactor * [K] + Rfactor * [R]
// NOTE! we assume that this function is computed after one computed
// ComputeInternalForces(), that updates inner data for the given node states.

void ChElementShellBST::ComputeKRMmatricesGlobal(ChMatrixRef H, double Kfactor, double Rfactor, double Mfactor) {

	assert((H.rows() == this->GetNdofs() ) && (H.cols() == this->GetNdofs()));

	// BRUTE FORCE METHOD: USE NUMERICAL DIFFERENTIATION!

	//
	// The K stiffness matrix of this element:  
	//

	ChState      state_x(this->LoadableGet_ndof_x(), nullptr);
	ChStateDelta state_w(this->LoadableGet_ndof_w(), nullptr);
	this->LoadableGetStateBlock_x(0, state_x);
	this->LoadableGetStateBlock_w(0, state_w);

	double Delta = 1e-10;

	int mrows_w = this->LoadableGet_ndof_w();
	int mrows_x = this->LoadableGet_ndof_x();

	
	// compute Q at current speed & position, x_0, v_0
	ChVectorDynamic<> Q0(mrows_w);
	this->ComputeInternalForces_impl(Q0, state_x, state_w, true);     // Q0 = Q(x, v)

	ChVectorDynamic<> Q1(mrows_w);
	ChVectorDynamic<> Jcolumn(mrows_w);
	

	if (Kfactor) {
		ChMatrixDynamic<> K(mrows_w, mrows_w);
		
		ChState       state_x_inc(mrows_x, nullptr);
		ChStateDelta  state_delta(mrows_w, nullptr);

		// Compute K=-dQ(x,v)/dx by backward differentiation
		state_delta.setZero(mrows_w, nullptr);

		for (int i = 0; i < mrows_w; ++i) {
			state_delta(i) += Delta;
			this->LoadableStateIncrement(0, state_x_inc, state_x, 0, state_delta);  // exponential, usually state_x_inc(i) = state_x(i) + Delta;

			Q1.setZero(mrows_w);
			this->ComputeInternalForces_impl(Q1, state_x_inc, state_w, true);   // Q1 = Q(x+Dx, v)
			state_delta(i) -= Delta;

			Jcolumn = (Q1 - Q0) * (-1.0 / Delta);   // - sign because K=-dQ/dx
			K.col(i) = Jcolumn;
		}

		// finally, store K into H:
		H.block(0, 0, mrows_w, mrows_w) = Kfactor * K;
	}
	else
		H.setZero();


	//
	// The R damping matrix of this element: 
	//

	if (Rfactor) {
		// Compute R=-dQ(x,v)/dv by backward differentiation
		//if (this->section->GetDamping()) {
		ChStateDelta  state_w_inc(mrows_w, nullptr);
		state_w_inc = state_w;
		ChMatrixDynamic<> R(mrows_w, mrows_w);
		
		for (int i = 0; i < mrows_w; ++i) {
			Q1.setZero(mrows_w);

			state_w_inc(i) += Delta;
			this->ComputeInternalForces_impl(Q1, state_x, state_w_inc, true); // Q1 = Q(x, v+Dv)
			state_w_inc(i) -= Delta;

			Jcolumn = (Q1 - Q0) * (-1.0 / Delta);   // - sign because R=-dQ/dv
			R.col(i) = Jcolumn;
		}

		H.block(0, 0, mrows_w, mrows_w) += Rfactor * R;
	}

	//
	// The M mass matrix of this element: (lumped version)
	//

	if (Mfactor) {
		// loop over all layers, to compute total "mass per area" = sum(rho_i*thickness_i) = average_rho * sum(thickness_i)
		double avg_density = this->GetDensity();
		double mass_per_area = avg_density * tot_thickness;

		// Heuristic, simplified 'lumped' mass matrix to central three nodes.
		// This is simpler than the stiffness-consistent mass matrix that would require
		// integration over gauss points.
		
		double nodemass = (1.0 / 3.0) * (this->area * mass_per_area);

		for (int n = 0; n < 3; n++) {

			int node_off = n * 3;
			H(node_off + 0, node_off + 0) += Mfactor * nodemass;
			H(node_off + 1, node_off + 1) += Mfactor * nodemass;
			H(node_off + 2, node_off + 2) += Mfactor * nodemass;

		}
	}
}




// -----------------------------------------------------------------------------
// Internal force calculation
// -----------------------------------------------------------------------------

void ChElementShellBST::ComputeInternalForces(ChVectorDynamic<>& Fi) {

	ChState mstate_x(this->LoadableGet_ndof_x(), nullptr);
	ChStateDelta mstate_w(this->LoadableGet_ndof_w(), nullptr);
    this->LoadableGetStateBlock_x(0,mstate_x);
    this->LoadableGetStateBlock_w(0,mstate_w);
    ComputeInternalForces_impl(Fi, mstate_x, mstate_w);
}

void ChElementShellBST::ComputeInternalForces_impl(ChVectorDynamic<>& Fi,
								ChState& state_x,       ///< state position to evaluate Fi
								ChStateDelta& state_w,  ///< state speed to evaluate Fi
								bool used_for_differentiation) {

	Fi.setZero();

	// only for readability .. make rest of the function easier to read
	std::array<ChVector<>,6> np;
	int istride = 0;
	for (int i = 0; i < 6; ++i) {
		if (m_nodes[i] != nullptr) {
			np[i] = state_x.segment(istride * 3, 3);
			++istride;
		}
		else
			np[i] = VNULL;
	}

	// Compute orthogonal system at triangle in carthesian basis
	ChVector<> t_1 = np[1] - np[0];
	ChVector<> t_2 = np[2] - np[0];
	// Compute normal 
	ChVector<> t_3 = Vcross(t_1, t_2).GetNormalized(); // The normal
	// no need to do t_1.Normalize() and to orthogonalize t_2 respect to t_1 because not needed later

	ShapeVector  Nx;
	this->ShapeFunctionsDerivativeX(Nx, this->Jux, 1./3., 1./3.);
	ShapeVector  Ny;
	this->ShapeFunctionsDerivativeY(Ny, this->Jux, 1./3., 1./3.);

	//GetLog() << "Nx: " << Nx(0) << "  " << Nx(1) << "  " << Nx(2) << "\n" << "Ny: " << Ny(0) << "  " << Ny(1) << "  " << Ny(2) << "\n";

	// Compute strain 
	ChVector<> P_1 = Nx(0) * np[0] + Nx(1) * np[1] + Nx(2) * np[2];
	ChVector<> P_2 = Ny(0) * np[0] + Ny(1) * np[1] + Ny(2) * np[2];

	//GetLog() << "P_1: " << P_1 << "\n" << "P_2: " << P_2 << "\n";

	this->e[0] = 0.5 * (Vdot(P_1, P_1) - 1.0);
	this->e[1] = 0.5 * (Vdot(P_2, P_2) - 1.0);
	this->e[2] =       (Vdot(P_1, P_2));

	// Compute curvature k  
	// Use the BST method in "kinked and branched shells"
	this->k = VNULL;
	ChVector<> ti_3[3];
	for (int i = 0; i < 3; ++i) {
		if (m_nodes[i + 3] != nullptr) {
			// normal of ith neighbouring triangle, opposed to ith triangle vertex:
			ti_3[i] = Vcross(np[i + 3] - np[(i + 1) % 3], np[(i + 2) % 3] - np[(i + 1) % 3]).GetNormalized();
			// boundary normal to shared edge:
			ChVector<> n_i = Vcross(np[(i + 1) % 3] - np[(i + 2) % 3], t_3).GetNormalized();
			// edge angle
			double mcos_fi = Vdot(t_3, ti_3[i]);
			double msin_fi = Vdot(n_i, ti_3[i]);
			ChVector<> Dphi;
			phi[i] = atan2(msin_fi, mcos_fi);
			Dphi[i] = phi[i] - this->phi0[i];
			// curvature
			double lambda = 1.0;
			//m_r[i] = 1.0 / 2.0; // to check - to improve in case of coarse mesh, make proportional to triangle height: r=R_i/(R_i+R_m), R_ = 1/h_
			ChVector<> LLi(Nx(i) * Nx(i),
				Ny(i) * Ny(i),
				-2.0 * Nx(i) * Ny(i)); // TODO: constant LLi[i] if 1 gauss point, linear triangle, move to precomputed?
			this->k += ((lambda * this->rI[i] * Dphi[i]) / this->l0[i]) * LLi;
		}
		else {
			phi[i] = 0;
			ti_3[i] = VNULL;
		}
	}
	this->k *= this->area * 4.0; 

	// Compute stretching and bending strains 
	// Loop on layers:
	ChVector<> l_n, l_m;
	this->n = VNULL;
	this->m = VNULL;
    for (size_t il = 0; il < this->m_layers.size(); ++il) {
        // compute layer stresses (per-unit-length forces and torques), and accumulate
        m_layers[il].GetMaterial()->ComputeStress(	l_n, 
													l_m, 
													this->e, 
													this->k, 
                                                    m_layers_z[il], m_layers_z[il + 1], m_layers[il].Get_theta());
        this->n += l_n;
		this->m += l_m;

		// add viscous damping 
		//***TO DO*** this require (still not computed) time derivative of this->e and this->k from state_w. Ex. see in IGA beam etc.
		/*
		if (m_layers[il].GetMaterial()->GetDamping()) {
			ChVector<> n_sp;
			ChVector<> m_sp;
			m_layers[il].GetMaterial()->GetDamping()->ComputeStress(
					n_sp,
					m_sp,
					e_dt, ???
					k_dt, ???
					m_layers_z[il], m_layers_z[il + 1], m_layers[il].Get_theta());
			this->n += n_sp;
			this->n += m_sp;
		}
		*/
    }
	

	// Compute forces by computing variations. One gauss point integration.

	// Membrane, BST

	//  f_n^T= n^T*[Bn] ,   with  de = [Bn]*dp
	for (int iv = 0; iv < 3; ++iv) {
		Fi.segment(iv * 3, 3) +=  -this->area * 1.0 * //***TO CHECK*** // gauss weight = 1.0 ? or 0.5 ?
								 (n[0] * Nx(iv) * P_1 
								+ n[1] * Ny(iv) * P_2 
							    + n[2] * (Nx(iv) * P_2 + Ny(iv) * P_1) ).eigen();
	}

	// Bending, BST

	int i_used_tri = 0;

	for (int it = 0; it < 3; ++it) {

		if (m_nodes[it + 3] != nullptr) {

			ChVector<> LLi(	Nx(it) * Nx(it),
							Ny(it) * Ny(it),
							-2.0 * Nx(it) * Ny(it)); // TODO: constant LLi[i] if 1 gauss point, linear triangle, move to precomputed?

			double mtL_coeff = this->area * 1.0  // gauss weight  = 1.0? or 0.5 ?
				* 2.0 * this->rI[it] * (1.0 / this->l0[it])
				* Vdot(this->m, LLi);

			int iI[3] = { (i_used_tri + 3), (it + 2) % 3, (it + 1) % 3 }; // state indexes of used vertexes in neighbouring triangle it, opposed to vertex iv=it; ccwise

			for (int iv = 0; iv < 3; ++iv) {

				Fi.segment(iv * 3, 3) +=  -mtL_coeff *  // * lambda_M/lambda_M  (=1)
					this->cM[it][iv] * (t_3).eigen();

				Fi.segment(iI[iv] * 3, 3) += -mtL_coeff *  // * lambda_M/lambda_I  (approx=1)
					this->cI[it][iv] * (ti_3[it]).eigen();
			}

			++i_used_tri;
		}
	}


}





// -----------------------------------------------------------------------------
// Shape functions
// -----------------------------------------------------------------------------

void ChElementShellBST::ShapeFunctions(ShapeVector& N, const double u, const double v) {

    N(0) = u;
    N(1) = v;
    N(2) = 1-u-v;
}

void ChElementShellBST::ShapeFunctionsDerivativeU(ShapeVector& Nu, const double u, const double v) {
    Nu(0) =  1.0;
    Nu(1) =  0.0;
    Nu(2) = -1.0;
}

void ChElementShellBST::ShapeFunctionsDerivativeV(ShapeVector& Nv, const double u, const double v) {
    Nv(0) = 0.0;
    Nv(1) = 1.0;
    Nv(2) =-1.0;
}

void  ChElementShellBST::ShapeFunctionsDerivativeX(ShapeVector& Nx, const ChMatrixNM<double, 2, 2> & Jux, const double u, const double v) {
	// compute [Nx;Ny]=[Jux]^T*[Nu;Nv], here without calling ShapeFunctionsDerivativeU ShapeFunctionsDerivativeV, in fact unrolling it simplifies to:
	Nx(0) = Jux(0, 0);					// J(0,0)*Nu(0)+J(1,0)*Nv(0)
	Nx(1) = Jux(1, 0);					// J(0,0)*Nu(1)+J(1,0)*Nv(1)
	Nx(2) = -Jux(0, 0) - Jux(1, 0) ;	// J(0,0)*Nu(3)+J(1,0)*Nv(3)
}

void  ChElementShellBST::ShapeFunctionsDerivativeY(ShapeVector& Ny, const ChMatrixNM<double, 2, 2> & Jux, const double u, const double v) {
	// compute [Nx;Ny]=[Jux]^T*[Nu;Nv], here without calling ShapeFunctionsDerivativeU ShapeFunctionsDerivativeV, in fact unrolling it simplifies to:
	Ny(0) = Jux(0, 1);					// J(0,1)*Nu(0)+J(1,1)*Nv(0)
	Ny(1) = Jux(1, 1);					// J(0,1)*Nu(1)+J(1,1)*Nv(1)
	Ny(2) = -Jux(0, 1) - Jux(1, 1) ;	// J(0,1)*Nu(3)+J(1,1)*Nv(3)
}




// -----------------------------------------------------------------------------
// Interface to ChElementShell base class
// -----------------------------------------------------------------------------

void ChElementShellBST::EvaluateSectionDisplacement(const double u,
                                                          const double v,
                                                          ChVector<>& u_displ,
                                                          ChVector<>& u_rotaz) {
    // this is not a corotational element, so just do:
    EvaluateSectionPoint(u, v, u_displ);
    u_rotaz = VNULL;  // no angles.. (or maybe return here the slope derivatives?)
}

void ChElementShellBST::EvaluateSectionFrame(const double u,
                                                   const double v,
                                                   ChVector<>& point,
                                                   ChQuaternion<>& rot) {
    // this is not a corotational element, so just do:
    EvaluateSectionPoint(u, v, point);
    rot = QUNIT;  // or maybe use gram-schmidt to get csys of section from slopes?
}

void ChElementShellBST::EvaluateSectionPoint(const double u,
                                                   const double v,
                                                   ChVector<>& point) {
    ShapeVector N;
    this->ShapeFunctions(N, u, v);

    const ChVector<>& pA = m_nodes[0]->GetPos();
    const ChVector<>& pB = m_nodes[1]->GetPos();
    const ChVector<>& pC = m_nodes[2]->GetPos();

    point = N(0) * pA + N(1) * pB + N(2) * pC;
}

// -----------------------------------------------------------------------------
// Functions for ChLoadable interface
// -----------------------------------------------------------------------------


// Gets all the DOFs packed in a single vector (position part).
void ChElementShellBST::LoadableGetStateBlock_x(int block_offset, ChState& mD) {
	for (int i = 0; i < n_usednodes; ++i) {
		mD.segment(block_offset + 3 * i, 3) = m_nodes[nodes_used_to_six[i]]->GetPos().eigen();
	}
}

// Gets all the DOFs packed in a single vector (velocity part).
void ChElementShellBST::LoadableGetStateBlock_w(int block_offset, ChStateDelta& mD) {
	for (int i = 0; i < n_usednodes; ++i) {
		mD.segment(block_offset + 3 * i, 3) = m_nodes[nodes_used_to_six[i]]->GetPos_dt().eigen();
	}
}

void ChElementShellBST::LoadableStateIncrement(const unsigned int off_x, ChState& x_new, const ChState& x, const unsigned int off_v, const ChStateDelta& Dv)  {
    for (int i = 0; i < n_usednodes; i++) {
        this->m_nodes[nodes_used_to_six[i]]->NodeIntStateIncrement(off_x  + 3 * i  , x_new, x, off_v  + 3 * i  , Dv);
    }
}

void ChElementShellBST::EvaluateSectionVelNorm(double U, double V, ChVector<>& Result) {
    ShapeVector N;
    ShapeFunctions(N, U, V);
    for (unsigned int ii = 0; ii < 3; ii++) {
        Result += N(ii) * m_nodes[ii]->GetPos_dt();
    }
}

// Get the pointers to the contained ChVariables, appending to the mvars vector.
void ChElementShellBST::LoadableGetVariables(std::vector<ChVariables*>& mvars) {
    for (int i = 0; i < n_usednodes; ++i) {
        mvars.push_back(&m_nodes[nodes_used_to_six[i]]->Variables());
    }
}

// Evaluate N'*F , where N is the shape function evaluated at (U,V) coordinates of the surface.
void ChElementShellBST::ComputeNF(
    const double U,              // parametric coordinate in surface
    const double V,              // parametric coordinate in surface
    ChVectorDynamic<>& Qi,       // Return result of Q = N'*F  here
    double& detJ,                // Return det[J] here
    const ChVectorDynamic<>& F,  // Input F vector, size is =n. field coords.
    ChVectorDynamic<>* state_x,  // if != 0, update state (pos. part) to this, then evaluate Q
    ChVectorDynamic<>* state_w   // if != 0, update state (speed part) to this, then evaluate Q
    ) {
    ShapeVector N;
    ShapeFunctions(N, U, V);

    // det of jacobian at u,v
    detJ = this->area*2.0;  

	Qi.setZero();

    Qi.segment(0, 3) = N(0) * F.segment(0, 3);

    Qi.segment(3, 3) = N(1) * F.segment(0, 3);

    Qi.segment(6, 3) = N(2) * F.segment(0, 3);
}

// Evaluate N'*F , where N is the shape function evaluated at (U,V,W) coordinates of the surface.
// Note that quadrature will be done with 0..1 coords for U,V (triangle natural coords) and -1..+1 for W, along thickness.
void ChElementShellBST::ComputeNF(
    const double U,              // parametric coordinate in volume
    const double V,              // parametric coordinate in volume
    const double W,              // parametric coordinate in volume
    ChVectorDynamic<>& Qi,       // Return result of N'*F  here, maybe with offset block_offset
    double& detJ,                // Return det[J] here
    const ChVectorDynamic<>& F,  // Input F vector, size is = n.field coords.
    ChVectorDynamic<>* state_x,  // if != 0, update state (pos. part) to this, then evaluate Q
    ChVectorDynamic<>* state_w   // if != 0, update state (speed part) to this, then evaluate Q
    ) {
    ShapeVector N;
    ShapeFunctions(N, U, V);

	detJ = GetThickness() * this->area; 
	// simplified from: 
	//   detJ = GetThickness() * this->area * 2 * 1/2
	// where ..* 2  stems from the fact that for triangles in natural coordinates [0..+1] the det_tri[J] is area*2
	// where ..*1/2 stems from the fact that the thickness integration runs in [-1..+1], so det_thick[J] is thickness*1/2

	Qi.setZero();

    Qi.segment(0,3) = N(0) * F.segment(0,3);

    Qi.segment(3, 3) = N(1) * F.segment(0, 3);

    Qi.segment(6, 3) = N(2) * F.segment(0, 3);
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------

// Calculate average element density (needed for ChLoaderVolumeGravity).
double ChElementShellBST::GetDensity() {
    double tot_density = 0;
    for (size_t kl = 0; kl < m_layers.size(); kl++) {
		double rho = m_layers[kl].GetMaterial()->GetDensity();
        double layerthick = m_layers[kl].Get_thickness();
        tot_density += rho * layerthick;
    }
    return tot_density / tot_thickness;
}

// Calculate normal to the surface at (U,V) coordinates.
ChVector<> ChElementShellBST::ComputeNormal(const double U, const double V) {

    const ChVector<>& pA = m_nodes[0]->GetPos();
    const ChVector<>& pB = m_nodes[1]->GetPos();
    const ChVector<>& pC = m_nodes[2]->GetPos();

    ChVector<> mnorm = Vcross(pB-pA, pC-pA);
    return mnorm.GetNormalized();
}


// Private constructor (a layer can be created only by adding it to an element)
ChElementShellBST::Layer::Layer(ChElementShellBST* element,
                                      double thickness,
                                      double theta,
                                      std::shared_ptr<ChMaterialShellKirchhoff> material)
    : m_element(element), m_thickness(thickness), m_theta(theta), m_material(material) {}

// Initial setup for this layer:
void ChElementShellBST::Layer::SetupInitial() {}

}  // end namespace fea
}  // end namespace chrono
