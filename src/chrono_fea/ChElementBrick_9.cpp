// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All right reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban
// =============================================================================
// Brick element with 9 nodes (central node for curvature)
// =============================================================================

#include "chrono/core/ChException.h"
#include "chrono/core/ChQuadrature.h"
#include "chrono_fea/ChElementBrick_9.h"
#include "chrono_fea/ChUtilsFEA.h"

namespace chrono {
	namespace fea {

		// -----------------------------------------------------------------------------
		// Constructor
		// -----------------------------------------------------------------------------

		ChElementBrick_9::ChElementBrick_9() : m_gravity_on(false) {
			m_nodes.resize(8);
		}

		enum StrainFormulation { GreenLagrange, Hencky };

		// -----------------------------------------------------------------------------
		// Initializations and initial setup
		// -----------------------------------------------------------------------------

		// Specify the nodes for this element
		void ChElementBrick_9::SetNodes(std::shared_ptr<ChNodeFEAxyz> node1,
			std::shared_ptr<ChNodeFEAxyz> node2,
			std::shared_ptr<ChNodeFEAxyz> node3,
			std::shared_ptr<ChNodeFEAxyz> node4,
			std::shared_ptr<ChNodeFEAxyz> node5,
			std::shared_ptr<ChNodeFEAxyz> node6,
			std::shared_ptr<ChNodeFEAxyz> node7,
			std::shared_ptr<ChNodeFEAxyz> node8,
			std::shared_ptr<ChNodeFEAcurv> nodeC) {
			assert(node1);
			assert(node2);
			assert(node3);
			assert(node4);
			assert(node5);
			assert(node6);
			assert(node7);
			assert(node8);
			assert(nodeC);

			m_nodes[0] = node1;
			m_nodes[1] = node2;
			m_nodes[2] = node3;
			m_nodes[3] = node4;
			m_nodes[4] = node5;
			m_nodes[5] = node6;
			m_nodes[6] = node7;
			m_nodes[7] = node8;
			m_central_node = nodeC;

			std::vector<ChLcpVariables*> mvars;
			mvars.push_back(&m_nodes[0]->Variables());
			mvars.push_back(&m_nodes[1]->Variables());
			mvars.push_back(&m_nodes[2]->Variables());
			mvars.push_back(&m_nodes[3]->Variables());
			mvars.push_back(&m_nodes[4]->Variables());
			mvars.push_back(&m_nodes[5]->Variables());
			mvars.push_back(&m_nodes[6]->Variables());
			mvars.push_back(&m_nodes[7]->Variables());
			mvars.push_back(&m_central_node->Variables());

			Kmatr.SetVariables(mvars);

			// Initial positions and slopes of the element nodes
			CalcCoordMatrix(m_d0);
			m_d0d0T.MatrMultiplyT(m_d0, m_d0);
		}

		// Initial element setup
		void ChElementBrick_9::SetupInitial(ChSystem* system) {
			//// TODO any other initializations go here

			m_GaussScaling = (GetDimensions().x * GetDimensions().y * GetDimensions().z) / 8;

			ComputeMassMatrix();
			ComputeGravityForce(system->Get_G_acc());
			

		}

		// -----------------------------------------------------------------------------
		// Calculation of shape functions and their derivatives
		// -----------------------------------------------------------------------------

		void ChElementBrick_9::ShapeFunctions(ChMatrix<>& N, double x, double y, double z) {
			double a = GetDimensions().x;
			double b = GetDimensions().y;
			double c = GetDimensions().z;
            N(0) = 0.125 * (1 - x) * (1 - y) * (1 - z);
            N(1) = 0.125 * (1 + x) * (1 - y) * (1 - z);
            N(2) = 0.125 * (1 + x) * (1 + y) * (1 - z);
            N(3) = 0.125 * (1 - x) * (1 + y) * (1 - z);
            N(4) = 0.125 * (1 - x) * (1 - y) * (1 + z);
            N(5) = 0.125 * (1 + x) * (1 - y) * (1 + z);
            N(6) = 0.125 * (1 + x) * (1 + y) * (1 + z);
            N(7) = 0.125 * (1 - x) * (1 + y) * (1 + z);
			N(8) = (a*a)*(-1.0 / 8.0) + (a*a)*(x*x)*(1.0 / 8.0);
			N(9) = (b*b)*(-1.0 / 8.0) + (b*b)*(y*y)*(1.0 / 8.0);
			N(10) = (c*c)*(-1.0 / 8.0) + (c*c)*(z*z)*(1.0 / 8.0);
		}

		void ChElementBrick_9::ShapeFunctionsDerivativeX(ChMatrix<>& Nx, double x, double y, double z) {
			double a = GetDimensions().x;
            Nx(0) = 0.25 / a * (-1) * (1 - y) * (1 - z);
            Nx(1) = 0.25 / a * (+1) * (1 - y) * (1 - z);
            Nx(2) = 0.25 / a * (+1) * (1 + y) * (1 - z);
            Nx(3) = 0.25 / a * (-1) * (1 + y) * (1 - z);
            Nx(4) = 0.25 / a * (-1) * (1 - y) * (1 + z);
            Nx(5) = 0.25 / a * (+1) * (1 - y) * (1 + z);
            Nx(6) = 0.25 / a * (+1) * (1 + y) * (1 + z);
            Nx(7) = 0.25 / a * (-1) * (1 + y) * (1 + z);
			//
			Nx(8) = a*x*(1.0 / 2.0);
			Nx(9) = 0;
			Nx(10) = 0;
		}

		void ChElementBrick_9::ShapeFunctionsDerivativeY(ChMatrix<>& Ny, double x, double y, double z) {
			double b = GetDimensions().y;
            Ny(0) = 0.25 / b * (1 - x) * (-1) * (1 - z);
            Ny(1) = 0.25 / b * (1 + x) * (-1) * (1 - z);
            Ny(2) = 0.25 / b * (1 + x) * (+1) * (1 - z);
            Ny(3) = 0.25 / b * (1 - x) * (+1) * (1 - z);
            Ny(4) = 0.25 / b * (1 - x) * (-1) * (1 + z);
            Ny(5) = 0.25 / b * (1 + x) * (-1) * (1 + z);
            Ny(6) = 0.25 / b * (1 + x) * (+1) * (1 + z);
            Ny(7) = 0.25 / b * (1 - x) * (+1) * (1 + z);
			//
			Ny(8) = 0;
			Ny(9) = b*y*(1.0 / 2.0);
			Ny(10) = 0;
		}

		void ChElementBrick_9::ShapeFunctionsDerivativeZ(ChMatrix<>& Nz, double x, double y, double z) {
			double c = GetDimensions().z;
            Nz(0) = 0.25 / c * (1 - x) * (1 - y) * (-1);
            Nz(1) = 0.25 / c * (1 + x) * (1 - y) * (-1);
            Nz(2) = 0.25 / c * (1 + x) * (1 + y) * (-1);
            Nz(3) = 0.25 / c * (1 - x) * (1 + y) * (-1);
            Nz(4) = 0.25 / c * (1 - x) * (1 - y) * (+1);
            Nz(5) = 0.25 / c * (1 + x) * (1 - y) * (+1);
            Nz(6) = 0.25 / c * (1 + x) * (1 + y) * (+1);
            Nz(7) = 0.25 / c * (1 - x) * (1 + y) * (+1);
			//
			Nz(8) = 0;
			Nz(9) = 0;
			Nz(10) = c*z*(1.0 / 2.0);
		}

		// -----------------------------------------------------------------------------
		// Calculation of the mass matrix
		// -----------------------------------------------------------------------------

		// Private class for quadrature of the mass matrix.
		class MyMassBrick9 : public ChIntegrable3D<ChMatrixNM<double, 33, 33>> {
		public:
			MyMassBrick9(ChElementBrick_9* element) : m_element(element) {}
			~MyMassBrick9() {}

		private:
			ChElementBrick_9* m_element;

			virtual void Evaluate(ChMatrixNM<double, 33, 33>& result, const double x, const double y, const double z) override;
		};

		void MyMassBrick9::Evaluate(ChMatrixNM<double, 33, 33>& result, const double x, const double y, const double z) {
			ChMatrixNM<double, 1, 11> N;
			m_element->ShapeFunctions(N, x, y, z);
			ChMatrixNM<double, 3, 33> S;
			ChMatrix33<> Si;
			Si.FillDiag(N(0));
			S.PasteMatrix(&Si, 0, 0);
			Si.FillDiag(N(1));
			S.PasteMatrix(&Si, 0, 3);
			Si.FillDiag(N(2));
			S.PasteMatrix(&Si, 0, 6);
			Si.FillDiag(N(3));
			S.PasteMatrix(&Si, 0, 9);
			Si.FillDiag(N(4));
			S.PasteMatrix(&Si, 0, 12);
			Si.FillDiag(N(5));
			S.PasteMatrix(&Si, 0, 15);
			Si.FillDiag(N(6));
			S.PasteMatrix(&Si, 0, 18);
			Si.FillDiag(N(7));
			S.PasteMatrix(&Si, 0, 21);
			Si.FillDiag(N(8));
			S.PasteMatrix(&Si, 0, 24);
			Si.FillDiag(N(9));
			S.PasteMatrix(&Si, 0, 27);
			Si.FillDiag(N(10));
			S.PasteMatrix(&Si, 0, 30);

			double detJ0 = m_element->Calc_detJ0(x, y, z);

			// perform  r = S'*S
			result.MatrTMultiply(S, S);

			// multiply integration weights
			result *= detJ0 * (m_element->m_GaussScaling);
		}

		// Compute the mass matrix of the element.
		void ChElementBrick_9::ComputeMassMatrix() {
			m_MassMatrix.Reset();

			MyMassBrick9 myformula(this);

			ChQuadrature::Integrate3D<ChMatrixNM<double, 33, 33>>(m_MassMatrix,  // result of integration will go there
				myformula,     // formula to integrate
				-1, 1,         // limits in x direction
				-1, 1,         // limits in y direction
				-1, 1,         // limits in z direction
				2              // order of integration
				);

			m_MassMatrix *= m_material->Get_density();
			
		}

		// -----------------------------------------------------------------------------
		// Calculation of gravitational forces
		// -----------------------------------------------------------------------------

		// Private class for quadrature of gravitational forces.
		class MyGravityBrick9 : public ChIntegrable3D<ChMatrixNM<double, 33, 1>> {
		public:
			MyGravityBrick9(ChElementBrick_9* element, const ChVector<>& gacc) : m_element(element), m_gacc(gacc) {}
			~MyGravityBrick9() {}

		private:
			ChElementBrick_9* m_element;
			ChVector<> m_gacc;

			virtual void Evaluate(ChMatrixNM<double, 33, 1>& result, const double x, const double y, const double z) override;
		};

		// Evaluate integrand at the specified point
		void MyGravityBrick9::Evaluate(ChMatrixNM<double, 33, 1>& result, const double x, const double y, const double z) {
			ChMatrixNM<double, 1, 11> N;
			m_element->ShapeFunctions(N, x, y, z);

			double detJ0 = m_element->Calc_detJ0(x, y, z);

			for (int i = 0; i < 11; i++) {
				result(i * 3 + 0, 0) = N(0, i) * m_gacc.x;
				result(i * 3 + 1, 0) = N(0, i) * m_gacc.y;
				result(i * 3 + 2, 0) = N(0, i) * m_gacc.z;
			}

			result *= detJ0 * m_element->m_GaussScaling;
		}

		// Compute the gravitational forces.
		void ChElementBrick_9::ComputeGravityForce(const ChVector<>& g_acc) {
			m_GravForce.Reset();

			MyGravityBrick9 myformula(this, g_acc);
			ChQuadrature::Integrate3D<ChMatrixNM<double, 33, 1>>(m_GravForce,  // result of integration will go there
				myformula,    // formula to integrate
				-1, 1,        // limits in x direction
				-1, 1,        // limits in y direction
				-1, 1,        // limits in z direction
				2             // order of integration
				);

			m_GravForce *= m_material->Get_density();
		}

		// -----------------------------------------------------------------------------
		// Calculation of the internal forces
		// -----------------------------------------------------------------------------

		// Private class for quadrature of internal forces
		class MyForceBrick9 : public ChIntegrable3D<ChMatrixNM<double, 33, 1>> {
		public:
			MyForceBrick9(ChElementBrick_9* element) : m_element(element) {}
			~MyForceBrick9() {}

		private:
			ChElementBrick_9* m_element;
			virtual void Evaluate(ChMatrixNM<double, 33, 1>& result, const double x, const double y, const double z) override;
		};

		// Evaluate integrand at the specified point
		void MyForceBrick9::Evaluate(ChMatrixNM<double, 33, 1>& result, const double x, const double y, const double z) {
			ChMatrixNM<double, 1, 11> N;
			m_element->ShapeFunctions(N, x, y, z);

			// Determinant of position vector gradient matrix: Initial configuration
			ChMatrixNM<double, 1, 11> Nx;
			ChMatrixNM<double, 1, 11> Ny;
			ChMatrixNM<double, 1, 11> Nz;
			ChMatrixNM<double, 1, 3> Nx_d0;
			ChMatrixNM<double, 1, 3> Ny_d0;
			ChMatrixNM<double, 1, 3> Nz_d0;
			ChMatrixNM<double, 1, 3> Nx_d;
			ChMatrixNM<double, 1, 3> Ny_d;
			ChMatrixNM<double, 1, 3> Nz_d;
			double detJ0 = m_element->Calc_detJ0(x, y, z, Nx, Ny, Nz, Nx_d0, Ny_d0, Nz_d0);

			Nx_d = Nx * m_element->m_d;
			Ny_d = Ny * m_element->m_d;
			Nz_d = Nz * m_element->m_d;

			double detJ = Nx_d(0, 0) * Ny_d(0, 1) * Nz_d(0, 2) + Ny_d(0, 0) * Nz_d(0, 1) * Nx_d(0, 2) +
				Nz_d(0, 0) * Nx_d(0, 1) * Ny_d(0, 2) - Nx_d(0, 2) * Ny_d(0, 1) * Nz_d(0, 0) -
				Ny_d(0, 2) * Nz_d(0, 1) * Nx_d(0, 0) - Nz_d(0, 2) * Nx_d(0, 1) * Ny_d(0, 0);

			ChMatrixNM<double, 3, 3> j0;
			// Calculates inverse of rd0 (j0) (position vector gradient: Initial Configuration)
			j0(0, 0) = Ny_d0(0,1) * Nz_d0(0,2) - Nz_d0(0,1) * Ny_d0(0,2);
			j0(0, 1) = Ny_d0(0,2) * Nz_d0(0,0) - Ny_d0(0,0) * Nz_d0(0,2);
			j0(0, 2) = Ny_d0(0,0) * Nz_d0(0,1) - Nz_d0(0,0) * Ny_d0(0,1);
			j0(1, 0) = Nz_d0(0,1) * Nx_d0(0,2) - Nx_d0(0,1) * Nz_d0(0,2);
			j0(1, 1) = Nz_d0(0,2) * Nx_d0(0,0) - Nx_d0(0,2) * Nz_d0(0,0);
			j0(1, 2) = Nz_d0(0,0) * Nx_d0(0,1) - Nz_d0(0,1) * Nx_d0(0,0);
			j0(2, 0) = Nx_d0(0,1) * Ny_d0(0,2) - Ny_d0(0,1) * Nx_d0(0,2);
			j0(2, 1) = Ny_d0(0,0) * Nx_d0(0,2) - Nx_d0(0,0) * Ny_d0(0,2);
			j0(2, 2) = Nx_d0(0,0) * Ny_d0(0,1) - Ny_d0(0,0) * Nx_d0(0,1);
			j0.MatrDivScale(detJ0);

			ChMatrixNM<double, 3, 3> DefF;
			DefF(0, 0) = Nx_d(0, 0); DefF(1, 0) = Nx_d(0, 1); DefF(2, 0) = Nx_d(0, 2);
			DefF(0, 1) = Ny_d(0, 0); DefF(1, 1) = Ny_d(0, 1); DefF(2, 1) = Ny_d(0, 2);
			DefF(0, 2) = Nz_d(0, 0); DefF(1, 2) = Nz_d(0, 1); DefF(2, 2) = Nz_d(0, 2);

			double E = m_element->GetMaterial()->Get_E();
			double nu = m_element->GetMaterial()->Get_v();
			double C1 = E*nu / ((1.0 + nu)*(1.0 - 2.0*nu));
			double C2 = m_element->GetMaterial()->Get_G();

			// Matrix of elastic coefficients
			ChMatrixNM<double, 6, 6> E_eps;
			E_eps(0, 0) = C1 + 2.0* C2;
			E_eps(1, 1) = C1 + 2.0* C2;
			E_eps(3, 3) = C1 + 2.0* C2;
			E_eps(0, 1) = C1;
			E_eps(0, 3) = C1;
			E_eps(1, 3) = C1;
			E_eps(1, 0) = E_eps(0, 1);
			E_eps(3, 0) = E_eps(0, 3);
			E_eps(3, 1) = E_eps(1, 3);
			E_eps(2, 2) = C2;
			E_eps(4, 4) = C2;
			E_eps(5, 5) = C2;

			//if (m_element->m_strain_form == GreenLagrange)
			if (!m_element->m_Hencky)
			{

				ChMatrixNM<double, 11, 1> ddNx;
				ChMatrixNM<double, 11, 1> ddNy;
				ChMatrixNM<double, 11, 1> ddNz;
				ddNx.MatrMultiplyT(m_element->m_ddT, Nx);
				ddNy.MatrMultiplyT(m_element->m_ddT, Ny);
				ddNz.MatrMultiplyT(m_element->m_ddT, Nz);

				ChMatrixNM<double, 11, 1> d0d0Nx;
				ChMatrixNM<double, 11, 1> d0d0Ny;
				ChMatrixNM<double, 11, 1> d0d0Nz;
				d0d0Nx.MatrMultiplyT(m_element->m_d0d0T, Nx);
				d0d0Ny.MatrMultiplyT(m_element->m_d0d0T, Ny);
				d0d0Nz.MatrMultiplyT(m_element->m_d0d0T, Nz);

				// Strain component
				ChMatrixNM<double, 6, 1> strain;
				strain(0, 0) = 0.5 * ((Nx * ddNx)(0, 0) - (Nx * d0d0Nx)(0, 0));
				strain(1, 0) = 0.5 * ((Ny * ddNy)(0, 0) - (Ny * d0d0Ny)(0, 0));
				strain(2, 0) = (Nx * ddNy)(0, 0) - (Nx * d0d0Ny)(0, 0);
				strain(3, 0) = 0.5 * ((Nz * ddNz)(0, 0) - (Nz * d0d0Nz)(0, 0));
				strain(4, 0) = (Nx * ddNz)(0, 0) - (Nx * d0d0Nz)(0, 0);
				strain(5, 0) = (Ny * ddNz)(0, 0) - (Ny * d0d0Nz)(0, 0);


				ChMatrixNM<double, 6, 33> strainD;
				ChMatrixNM<double, 3, 3> Id3;
				ChMatrixNM<double, 3, 3> Dm;
				ChMatrixNM<double, 6, 9> DepsDDm;
				Id3(0, 0) = 1.0; Id3(1, 1) = 1.0; Id3(2, 2) = 1.0;
				Dm = DefF - Id3;
				DepsDDm(0, 0) = 1.0 + Dm(0, 0);
				DepsDDm(0, 1) = Dm(1, 0);
				DepsDDm(0, 2) = Dm(2, 0);
				DepsDDm(1, 3) = Dm(0, 1);
				DepsDDm(1, 4) = 1.0 + Dm(1, 1);
				DepsDDm(1, 5) = Dm(2, 1);

				DepsDDm(2, 0) = Dm(0, 1);
				DepsDDm(2, 1) = (1.0 + Dm(1, 1));
				DepsDDm(2, 2) = Dm(2, 1);
				DepsDDm(2, 3) = (1.0 + Dm(0, 0));
				DepsDDm(2, 4) = Dm(1, 0);
				DepsDDm(2, 5) = Dm(2, 0);

				DepsDDm(3, 6) = Dm(0, 2);
				DepsDDm(3, 7) = Dm(1, 2);
				DepsDDm(3, 8) = 1.0 + Dm(2, 2);

				DepsDDm(4, 0) = Dm(0, 2);
				DepsDDm(4, 1) = Dm(1, 2);
				DepsDDm(4, 2) = (1.0 + Dm(2, 2));
				DepsDDm(4, 6) = (1.0 + Dm(0, 0));
				DepsDDm(4, 7) = Dm(1, 0);
				DepsDDm(4, 8) = Dm(2, 0);

				DepsDDm(5, 3) = Dm(0, 2);
				DepsDDm(5, 4) = Dm(1, 2);
				DepsDDm(5, 5) = (1.0 + Dm(2, 2));
				DepsDDm(5, 6) = Dm(0, 1);
				DepsDDm(5, 7) = (1.0 + Dm(1, 1));
				DepsDDm(5, 8) = Dm(2, 1);

				ChMatrixNM<double, 9, 33> Gd;
				for (int ii = 0; ii < 11; ii++) {
					Gd(0, 3 * (ii)) = j0(0, 0) * Nx(0, ii) + j0(1, 0) * Ny(0, ii) + j0(2, 0) * Nz(0, ii);
					Gd(1, 3 * (ii)+1) = j0(0, 0) * Nx(0, ii) + j0(1, 0) * Ny(0, ii) + j0(2, 0) * Nz(0, ii);
					Gd(2, 3 * (ii)+2) = j0(0, 0) * Nx(0, ii) + j0(1, 0) * Ny(0, ii) + j0(2, 0) * Nz(0, ii);

					Gd(3, 3 * (ii)) = j0(0, 1) * Nx(0, ii) + j0(1, 1) * Ny(0, ii) + j0(2, 1) * Nz(0, ii);
					Gd(4, 3 * (ii)+1) = j0(0, 1) * Nx(0, ii) + j0(1, 1) * Ny(0, ii) + j0(2, 1) * Nz(0, ii);
					Gd(5, 3 * (ii)+2) = j0(0, 1) * Nx(0, ii) + j0(1, 1) * Ny(0, ii) + j0(2, 1) * Nz(0, ii);

					Gd(6, 3 * (ii)) = j0(0, 2) * Nx(0, ii) + j0(1, 2) * Ny(0, ii) + j0(2, 2) * Nz(0, ii);
					Gd(7, 3 * (ii)+1) = j0(0, 2) * Nx(0, ii) + j0(1, 2) * Ny(0, ii) + j0(2, 2) * Nz(0, ii);
					Gd(8, 3 * (ii)+2) = j0(0, 2) * Nx(0, ii) + j0(1, 2) * Ny(0, ii) + j0(2, 2) * Nz(0, ii);
				}

				strainD.MatrMultiply(DepsDDm, Gd);

				ChMatrixNM<double, 6, 1> DEPS;
				DEPS.Reset();
				for (int ii = 0; ii < 33; ii++) {
					DEPS(0, 0) = DEPS(0, 0) + strainD(0, ii) * m_element->m_d_dt(ii, 0);
					DEPS(1, 0) = DEPS(1, 0) + strainD(1, ii) * m_element->m_d_dt(ii, 0);
					DEPS(2, 0) = DEPS(2, 0) + strainD(2, ii) * m_element->m_d_dt(ii, 0);
					DEPS(3, 0) = DEPS(3, 0) + strainD(3, ii) * m_element->m_d_dt(ii, 0);
					DEPS(4, 0) = DEPS(4, 0) + strainD(4, ii) * m_element->m_d_dt(ii, 0);
					DEPS(5, 0) = DEPS(5, 0) + strainD(5, ii) * m_element->m_d_dt(ii, 0);
				}

				// Add structural damping
				strain += DEPS * m_element->m_Alpha;

				// Internal force calculation
				ChMatrixNM<double, 33, 6> tempC;
				tempC.MatrTMultiply(strainD, E_eps);
				ChMatrixNM<double, 33, 1> Fint;
				Fint.MatrMultiply(tempC, strain);
				Fint.MatrScale(detJ0 * m_element->m_GaussScaling);
				result = Fint;
			}
			//else if (m_element->m_strain_form == Hencky)
			else if (m_element->m_Hencky)
			{
				ChMatrixNM<double, 3, 3> Temp33;
				ChMatrixNM<double, 3, 3> CCPinv;
				ChMatrix33<double> BETRI;
				double BETRI_eig[3] = { 0.0 };
				ChMatrix33<double> BETRI_eigv;
				ChMatrixNM<double, 3, 1> e1;
				ChMatrixNM<double, 3, 1> e2;
				ChMatrixNM<double, 3, 1> e3;
				ChMatrixNM<double, 3, 3> MM1;
				ChMatrixNM<double, 3, 3> MM2;
				ChMatrixNM<double, 3, 3> MM3;
				CCPinv(0, 0) = 1.0;
				CCPinv(1, 1) = 1.0;
				CCPinv(2, 2) = 1.0;
				if (m_element->m_Plasticity)
				{
					CCPinv(0, 0) = m_element->m_CCPinv_Plast(0, m_element->m_InteCounter);
					CCPinv(0, 1) = m_element->m_CCPinv_Plast(1, m_element->m_InteCounter);
					CCPinv(0, 2) = m_element->m_CCPinv_Plast(2, m_element->m_InteCounter);
					CCPinv(1, 0) = m_element->m_CCPinv_Plast(3, m_element->m_InteCounter);
					CCPinv(1, 1) = m_element->m_CCPinv_Plast(4, m_element->m_InteCounter);
					CCPinv(1, 2) = m_element->m_CCPinv_Plast(5, m_element->m_InteCounter);
					CCPinv(2, 0) = m_element->m_CCPinv_Plast(6, m_element->m_InteCounter);
					CCPinv(2, 1) = m_element->m_CCPinv_Plast(7, m_element->m_InteCounter);
					CCPinv(2, 2) = m_element->m_CCPinv_Plast(8, m_element->m_InteCounter);
				}
				
				Temp33.MatrMultiply(DefF, CCPinv);
				BETRI.MatrMultiplyT(Temp33, DefF);
				BETRI.FastEigen(BETRI_eigv, BETRI_eig);
				for (int i = 0; i < 3; i++)
				{
					e3(i, 0) = BETRI_eigv(i, 0);
					e1(i, 0) = BETRI_eigv(i, 1);
					e2(i, 0) = BETRI_eigv(i, 2);
				}
				MM1.MatrMultiplyT(e1, e1);
				MM2.MatrMultiplyT(e2, e2);
				MM3.MatrMultiplyT(e3, e3);
				//ChMatrix33<double> test;
				//double test_eig[3] = { 0.0 };
				//ChMatrix33<double> test_eigv;
				//ChVector<double> e11;
				//ChVector<double> e21;
				//ChVector<double> e31;
				//test(0, 0) = 1.3; test(0, 1) = -3.3; test(0, 2) = 1.4;
				//test(1, 0) = -3.3; test(1, 1) = 1.11; test(1, 2) = 2.1;
				//test(2, 0) = 1.4; test(2, 1) = 2.1; test(2, 2) = 1.7;

				//test.FastEigen(test_eigv, test_eig);

				//for (int i = 0; i < 3; i++)
				//{
				//	e3(i) = test_eigv(i, 0);
				//	e1(i) = test_eigv(i, 1);
				//	e2(i) = test_eigv(i, 2);
				//}
				//
				//double testorth;
				//testorth = (e11.x*e21.x + e11.y*e21.y + e11.z*e21.z);
				//testorth = (e31.x*e21.x + e31.y*e21.y + e31.z*e21.z);
				//testorth = (e11.x*e31.x + e11.y*e31.y + e11.z*e31.z);

				ChMatrixNM<double, 3, 1> LogStrain;
				LogStrain(0, 0) = 0.5*log(BETRI_eig[1]);
				LogStrain(1, 0) = 0.5*log(BETRI_eig[2]);
				LogStrain(2, 0) = 0.5*log(BETRI_eig[0]);

				ChMatrixNM<double, 3, 3> FI;
				ChMatrixNM<double, 6, 33> strainD;
				FI = DefF;
				FI.MatrInverse();
				m_element->EPSP_Euerian_SolidANCF33(strainD, Nx, Ny, Nz, FI, j0);

				ChMatrixNM<double, 3, 1> StressK_eig;
				StressK_eig(0, 0) = E_eps(0, 0)*LogStrain(0, 0) + E_eps(0, 1)*LogStrain(1, 0) + E_eps(0, 3)*LogStrain(2, 0);
				StressK_eig(1, 0) = E_eps(1, 0)*LogStrain(0, 0) + E_eps(1, 1)*LogStrain(1, 0) + E_eps(1, 3)*LogStrain(2, 0);
				StressK_eig(2, 0) = E_eps(3, 0)*LogStrain(0, 0) + E_eps(3, 1)*LogStrain(1, 0) + E_eps(3, 3)*LogStrain(2, 0);

				if (m_element->m_Plasticity)
				{
					double G = E / (2.0*(1 + nu));
					double K = E / (3.0*(1 - 2.0*nu));

					double EEVD3 = (LogStrain(0, 0) + LogStrain(1, 0) + LogStrain(2, 0)) / 3.0;
					ChVector<double> EETD;
					EETD.x = LogStrain(0, 0) - EEVD3;
					EETD.y = LogStrain(1, 0) - EEVD3;
					EETD.z = LogStrain(2, 0) - EEVD3;
					double ETDNorm = sqrt(EETD.x*EETD.x + EETD.y*EETD.y + EETD.z*EETD.z);

					double hydroP = (StressK_eig(0, 0) + StressK_eig(1, 0) + StressK_eig(2, 0)) / 3.0;
					ChVector<double> devStress;
					devStress.x = StressK_eig(0, 0) - hydroP;
					devStress.y = StressK_eig(1, 0) - hydroP;
					devStress.z = StressK_eig(2, 0) - hydroP;
					double NormSn = sqrt(devStress.x*devStress.x + devStress.y*devStress.y + devStress.z*devStress.z);

					double J2Rt = NormSn / sqrt(2.0);
					double qtrial = sqrt(3.0)*J2Rt;

					double YieldFunc = qtrial - (m_element->m_YieldStress + m_element->m_HardeningSlope*m_element->m_Alpha_Plast(m_element->m_InteCounter, 0));
					int YieldFlag = 0;
					if (YieldFunc > 0.0)
					{
						YieldFlag = 1;

						double DeltaGamma = YieldFunc / (3.0*G + m_element->m_HardeningSlope);
						ChVector<double> devStressUp;
						if (qtrial != 0.0)
						{
							devStressUp = (1.0 - G*DeltaGamma*3.0 / qtrial)*devStress;
						}
						else{
							devStressUp = devStress;
						}

						StressK_eig(0, 0) = devStressUp.x + hydroP;
						StressK_eig(1, 0) = devStressUp.y + hydroP;
						StressK_eig(2, 0) = devStressUp.z + hydroP;

						LogStrain(0, 0) = devStressUp.x / (2.0*G) + EEVD3;
						LogStrain(1, 0) = devStressUp.y / (2.0*G) + EEVD3;
						LogStrain(2, 0) = devStressUp.z / (2.0*G) + EEVD3;

						ChVector<double> lambda;
						lambda.x = exp(2.0*LogStrain(0, 0));
						lambda.y = exp(2.0*LogStrain(1, 0));
						lambda.z = exp(2.0*LogStrain(2, 0));
						ChMatrixNM<double, 3, 3> BEUP;
						MM1.MatrScale(lambda.x);
						MM2.MatrScale(lambda.y);
						MM3.MatrScale(lambda.z);
						BEUP = MM1 + MM2 + MM3;
						MM1.MatrScale(1 / lambda.x);
						MM2.MatrScale(1 / lambda.y);
						MM3.MatrScale(1 / lambda.z);
						m_element->m_Alpha_Plast(m_element->m_InteCounter, 0) = m_element->m_Alpha_Plast(m_element->m_InteCounter, 0) + DeltaGamma;
						Temp33.MatrMultiply(FI, BEUP);
						CCPinv.MatrMultiplyT(Temp33,FI);
						// Store CPPinv
						m_element->m_CCPinv_Plast(0, m_element->m_InteCounter) = CCPinv(0, 0);
						m_element->m_CCPinv_Plast(1, m_element->m_InteCounter) = CCPinv(0, 1);
						m_element->m_CCPinv_Plast(2, m_element->m_InteCounter) = CCPinv(0, 2);
						m_element->m_CCPinv_Plast(3, m_element->m_InteCounter) = CCPinv(1, 0);
						m_element->m_CCPinv_Plast(4, m_element->m_InteCounter) = CCPinv(1, 1);
						m_element->m_CCPinv_Plast(5, m_element->m_InteCounter) = CCPinv(1, 2);
						m_element->m_CCPinv_Plast(6, m_element->m_InteCounter) = CCPinv(2, 0);
						m_element->m_CCPinv_Plast(7, m_element->m_InteCounter) = CCPinv(2, 1);
						m_element->m_CCPinv_Plast(8, m_element->m_InteCounter) = CCPinv(2, 2);

					}

				}

				ChMatrixNM<double, 3, 3> StressK;
				MM1.MatrScale(StressK_eig(0, 0));
				MM2.MatrScale(StressK_eig(1, 0));
				MM3.MatrScale(StressK_eig(2, 0));
				StressK = MM1 + MM2 + MM3;

				ChMatrixNM<double, 6, 1> DEPS;
				DEPS.Reset();
				for (int ii = 0; ii < 33; ii++) {
					DEPS(0, 0) = DEPS(0, 0) + strainD(0, ii) * m_element->m_d_dt(ii, 0);
					DEPS(1, 0) = DEPS(1, 0) + strainD(1, ii) * m_element->m_d_dt(ii, 0);
					DEPS(2, 0) = DEPS(2, 0) + strainD(2, ii) * m_element->m_d_dt(ii, 0);
					DEPS(3, 0) = DEPS(3, 0) + strainD(3, ii) * m_element->m_d_dt(ii, 0);
					DEPS(4, 0) = DEPS(4, 0) + strainD(4, ii) * m_element->m_d_dt(ii, 0);
					DEPS(5, 0) = DEPS(5, 0) + strainD(5, ii) * m_element->m_d_dt(ii, 0);
				}

				ChMatrixNM<double, 6, 1> Stress_damp;
				// Add structural damping
				Stress_damp.MatrMultiply(E_eps, DEPS);
				Stress_damp.MatrScale(m_element->m_Alpha);

				ChMatrixNM<double, 6, 1> Stress;
				Stress(0, 0) = StressK(0, 0) + Stress_damp(0, 0);
				Stress(1, 0) = StressK(1, 1) + Stress_damp(1, 0);
				Stress(2, 0) = 0.5*(StressK(1, 0) + StressK(0, 1)) + Stress_damp(2, 0);
				Stress(3, 0) = StressK(2, 2) + Stress_damp(3, 0);
				Stress(4, 0) = 0.5*(StressK(2, 0) + StressK(0, 2)) + Stress_damp(4, 0);
				Stress(5, 0) = 0.5*(StressK(2, 1) + StressK(1, 2)) + Stress_damp(5, 0);

				ChMatrixNM<double, 33, 1> Fint;
				Fint.MatrTMultiply(strainD, Stress);
				Fint.MatrScale(detJ0*m_element->m_GaussScaling);
				result = Fint;
				m_element->m_InteCounter++;
			}
		}

		// Compute internal forces and load them in the Fi vector.
		void ChElementBrick_9::ComputeInternalForces(ChMatrixDynamic<>& Fi) {
			CalcCoordMatrix(m_d);
			CalcCoordDerivMatrix(m_d_dt);
			m_ddT.MatrMultiplyT(m_d, m_d);


			Fi.Reset();
			m_InteCounter = 0;
			ChMatrixNM<double, 33, 1> result;
			MyForceBrick9 formula(this);
			ChQuadrature::Integrate3D<ChMatrixNM<double, 33, 1> >(result,   // result of integration
				formula,  // integrand formula
				-1, 1,    // x limits
				-1, 1,    // y limits
				-1, 1,  // z limits
				2  // order of integration
				);
			Fi -= result;
			if (m_gravity_on) {
				Fi += m_GravForce;
			}
		}

		// -----------------------------------------------------------------------------
		// Calculation of the Jacobian of internal forces
		// -----------------------------------------------------------------------------

		// Private class for quadrature of the Jacobian of internal forces
		class MyJacobianBrick9 : public ChIntegrable3D<ChMatrixNM<double, 33, 33>> {
		public:
			MyJacobianBrick9(ChElementBrick_9* element,  // Associated element
				double Kfactor,             // Scaling coefficient for stiffness component
				double Rfactor              // Scaling coefficient for damping component
				)
				: m_element(element), m_Kfactor(Kfactor), m_Rfactor(Rfactor) {}

		private:
			ChElementBrick_9* m_element;
			double m_Kfactor;
			double m_Rfactor;

			virtual void Evaluate(ChMatrixNM<double, 33, 33>& result, const double x, const double y, const double z) override;
		};

		// Evaluate integrand at the specified point
		void MyJacobianBrick9::Evaluate(ChMatrixNM<double, 33, 33>& result, const double x, const double y, const double z) {
			ChMatrixNM<double, 1, 11> N;
			m_element->ShapeFunctions(N, x, y, z);

			// Determinant of position vector gradient matrix: Initial configuration
			ChMatrixNM<double, 1, 11> Nx;
			ChMatrixNM<double, 1, 11> Ny;
			ChMatrixNM<double, 1, 11> Nz;
			ChMatrixNM<double, 1, 3> Nx_d0;
			ChMatrixNM<double, 1, 3> Ny_d0;
			ChMatrixNM<double, 1, 3> Nz_d0;
			ChMatrixNM<double, 1, 3> Nx_d;
			ChMatrixNM<double, 1, 3> Ny_d;
			ChMatrixNM<double, 1, 3> Nz_d;
			double detJ0 = m_element->Calc_detJ0(x, y, z, Nx, Ny, Nz, Nx_d0, Ny_d0, Nz_d0);

			Nx_d = Nx * m_element->m_d;
			Ny_d = Ny * m_element->m_d;
			Nz_d = Nz * m_element->m_d;

			double detJ = Nx_d(0, 0) * Ny_d(0, 1) * Nz_d(0, 2) + Ny_d(0, 0) * Nz_d(0, 1) * Nx_d(0, 2) +
				Nz_d(0, 0) * Nx_d(0, 1) * Ny_d(0, 2) - Nx_d(0, 2) * Ny_d(0, 1) * Nz_d(0, 0) -
				Ny_d(0, 2) * Nz_d(0, 1) * Nx_d(0, 0) - Nz_d(0, 2) * Nx_d(0, 1) * Ny_d(0, 0);

			ChMatrixNM<double, 3, 3> j0;
			// Calculates inverse of rd0 (j0) (position vector gradient: Initial Configuration)
			j0(0, 0) = Ny_d0(0, 1) * Nz_d0(0, 2) - Nz_d0(0, 1) * Ny_d0(0, 2);
			j0(0, 1) = Ny_d0(0, 2) * Nz_d0(0, 0) - Ny_d0(0, 0) * Nz_d0(0, 2);
			j0(0, 2) = Ny_d0(0, 0) * Nz_d0(0, 1) - Nz_d0(0, 0) * Ny_d0(0, 1);
			j0(1, 0) = Nz_d0(0, 1) * Nx_d0(0, 2) - Nx_d0(0, 1) * Nz_d0(0, 2);
			j0(1, 1) = Nz_d0(0, 2) * Nx_d0(0, 0) - Nx_d0(0, 2) * Nz_d0(0, 0);
			j0(1, 2) = Nz_d0(0, 0) * Nx_d0(0, 1) - Nz_d0(0, 1) * Nx_d0(0, 0);
			j0(2, 0) = Nx_d0(0, 1) * Ny_d0(0, 2) - Ny_d0(0, 1) * Nx_d0(0, 2);
			j0(2, 1) = Ny_d0(0, 0) * Nx_d0(0, 2) - Nx_d0(0, 0) * Ny_d0(0, 2);
			j0(2, 2) = Nx_d0(0, 0) * Ny_d0(0, 1) - Ny_d0(0, 0) * Nx_d0(0, 1);
			j0.MatrDivScale(detJ0);

			ChMatrixNM<double, 3, 3> DefF;
			DefF(0, 0) = Nx_d(0, 0); DefF(1, 0) = Nx_d(0, 1); DefF(2, 0) = Nx_d(0, 2);
			DefF(0, 1) = Ny_d(0, 0); DefF(1, 1) = Ny_d(0, 1); DefF(2, 1) = Ny_d(0, 2);
			DefF(0, 2) = Nz_d(0, 0); DefF(1, 2) = Nz_d(0, 1); DefF(2, 2) = Nz_d(0, 2);

			double E = m_element->GetMaterial()->Get_E();
			double nu = m_element->GetMaterial()->Get_v();
			double C1 = E*nu / ((1.0 + nu)*(1.0 - 2.0*nu));
			double C2 = m_element->GetMaterial()->Get_G();

			// Matrix of elastic coefficients
			ChMatrixNM<double, 6, 6> E_eps;
			E_eps(0, 0) = C1 + 2.0* C2;
			E_eps(1, 1) = C1 + 2.0* C2;
			E_eps(3, 3) = C1 + 2.0* C2;
			E_eps(0, 1) = C1;
			E_eps(0, 3) = C1;
			E_eps(1, 3) = C1;
			E_eps(1, 0) = E_eps(0, 1);
			E_eps(3, 0) = E_eps(0, 3);
			E_eps(3, 1) = E_eps(1, 3);
			E_eps(2, 2) = C2;
			E_eps(4, 4) = C2;
			E_eps(5, 5) = C2;

			//if (m_element->m_strain_form == GreenLagrange)
			if (!m_element->m_Hencky)
			{

				ChMatrixNM<double, 11, 1> ddNx;
				ChMatrixNM<double, 11, 1> ddNy;
				ChMatrixNM<double, 11, 1> ddNz;
				ddNx.MatrMultiplyT(m_element->m_ddT, Nx);
				ddNy.MatrMultiplyT(m_element->m_ddT, Ny);
				ddNz.MatrMultiplyT(m_element->m_ddT, Nz);

				ChMatrixNM<double, 11, 1> d0d0Nx;
				ChMatrixNM<double, 11, 1> d0d0Ny;
				ChMatrixNM<double, 11, 1> d0d0Nz;
				d0d0Nx.MatrMultiplyT(m_element->m_d0d0T, Nx);
				d0d0Ny.MatrMultiplyT(m_element->m_d0d0T, Ny);
				d0d0Nz.MatrMultiplyT(m_element->m_d0d0T, Nz);

				// Strain component
				ChMatrixNM<double, 6, 1> strain_til;
				strain_til(0, 0) = 0.5 * ((Nx * ddNx)(0, 0) - (Nx * d0d0Nx)(0, 0));
				strain_til(1, 0) = 0.5 * ((Ny * ddNy)(0, 0) - (Ny * d0d0Ny)(0, 0));
				strain_til(2, 0) = (Nx * ddNy)(0, 0) - (Nx * d0d0Ny)(0, 0);
				strain_til(3, 0) = 0.5 * ((Nz * ddNz)(0, 0) - (Nz * d0d0Nz)(0, 0));
				strain_til(4, 0) = (Nx * ddNz)(0, 0) - (Nx * d0d0Nz)(0, 0);
				strain_til(5, 0) = (Ny * ddNz)(0, 0) - (Ny * d0d0Nz)(0, 0);

				ChMatrixNM<double, 6, 1> strain;
				strain = strain_til;

				ChMatrixNM<double, 6, 33> strainD;
				ChMatrixNM<double, 6, 33> strainDT;
				ChMatrixNM<double, 3, 3> Id3;
				ChMatrixNM<double, 3, 3> Dm;
				ChMatrixNM<double, 6, 9> DepsDDm;
				Id3(0, 0) = 1.0; Id3(1, 1) = 1.0; Id3(2, 2) = 1.0;
				Dm = DefF - Id3;
				DepsDDm(0, 0) = 1.0 + Dm(0, 0);
				DepsDDm(0, 1) = Dm(1, 0);
				DepsDDm(0, 2) = Dm(2, 0);
				DepsDDm(1, 3) = Dm(0, 1);
				DepsDDm(1, 4) = 1.0 + Dm(1, 1);
				DepsDDm(1, 5) = Dm(2, 1);

				DepsDDm(2, 0) = Dm(0, 1);
				DepsDDm(2, 1) = (1.0 + Dm(1, 1));
				DepsDDm(2, 2) = Dm(2, 1);
				DepsDDm(2, 3) = (1.0 + Dm(0, 0));
				DepsDDm(2, 4) = Dm(1, 0);
				DepsDDm(2, 5) = Dm(2, 0);

				DepsDDm(3, 6) = Dm(0, 2);
				DepsDDm(3, 7) = Dm(1, 2);
				DepsDDm(3, 8) = 1.0 + Dm(2, 2);

				DepsDDm(4, 0) = Dm(0, 2);
				DepsDDm(4, 1) = Dm(1, 2);
				DepsDDm(4, 2) = (1.0 + Dm(2, 2));
				DepsDDm(4, 6) = (1.0 + Dm(0, 0));
				DepsDDm(4, 7) = Dm(1, 0);
				DepsDDm(4, 8) = Dm(2, 0);

				DepsDDm(5, 3) = Dm(0, 2);
				DepsDDm(5, 4) = Dm(1, 2);
				DepsDDm(5, 5) = (1.0 + Dm(2, 2));
				DepsDDm(5, 6) = Dm(0, 1);
				DepsDDm(5, 7) = (1.0 + Dm(1, 1));
				DepsDDm(5, 8) = Dm(2, 1);

				ChMatrixNM<double, 9, 33> Gd;
				for (int ii = 0; ii < 11; ii++) {
					Gd(0, 3 * (ii)) = j0(0, 0) * Nx(0, ii) + j0(1, 0) * Ny(0, ii) + j0(2, 0) * Nz(0, ii);
					Gd(1, 3 * (ii)+1) = j0(0, 0) * Nx(0, ii) + j0(1, 0) * Ny(0, ii) + j0(2, 0) * Nz(0, ii);
					Gd(2, 3 * (ii)+2) = j0(0, 0) * Nx(0, ii) + j0(1, 0) * Ny(0, ii) + j0(2, 0) * Nz(0, ii);

					Gd(3, 3 * (ii)) = j0(0, 1) * Nx(0, ii) + j0(1, 1) * Ny(0, ii) + j0(2, 1) * Nz(0, ii);
					Gd(4, 3 * (ii)+1) = j0(0, 1) * Nx(0, ii) + j0(1, 1) * Ny(0, ii) + j0(2, 1) * Nz(0, ii);
					Gd(5, 3 * (ii)+2) = j0(0, 1) * Nx(0, ii) + j0(1, 1) * Ny(0, ii) + j0(2, 1) * Nz(0, ii);

					Gd(6, 3 * (ii)) = j0(0, 2) * Nx(0, ii) + j0(1, 2) * Ny(0, ii) + j0(2, 2) * Nz(0, ii);
					Gd(7, 3 * (ii)+1) = j0(0, 2) * Nx(0, ii) + j0(1, 2) * Ny(0, ii) + j0(2, 2) * Nz(0, ii);
					Gd(8, 3 * (ii)+2) = j0(0, 2) * Nx(0, ii) + j0(1, 2) * Ny(0, ii) + j0(2, 2) * Nz(0, ii);
				}

				strainD.MatrMultiply(DepsDDm, Gd);

				//for (int i = 0; i < 33; i++)
				//{
				//	GetLog() << strainD(0, i) << " " << strainD(1, i) << " " << strainD(2, i) << "\n" << strainD(3, i) << " " << strainD(4, i) << " " << strainD(5, i) << "\n";
				//}
				//system("pause");

				ChMatrixNM<double, 6, 1> DEPS;
				DEPS.Reset();
				for (int ii = 0; ii < 33; ii++) {
					DEPS(0, 0) = DEPS(0, 0) + strainD(0, ii) * m_element->m_d_dt(ii, 0);
					DEPS(1, 0) = DEPS(1, 0) + strainD(1, ii) * m_element->m_d_dt(ii, 0);
					DEPS(2, 0) = DEPS(2, 0) + strainD(2, ii) * m_element->m_d_dt(ii, 0);
					DEPS(3, 0) = DEPS(3, 0) + strainD(3, ii) * m_element->m_d_dt(ii, 0);
					DEPS(4, 0) = DEPS(4, 0) + strainD(4, ii) * m_element->m_d_dt(ii, 0);
					DEPS(5, 0) = DEPS(5, 0) + strainD(5, ii) * m_element->m_d_dt(ii, 0);
				}

				// Add structural damping
				strain += DEPS * m_element->m_Alpha;

				// Stress tensor calculation
				ChMatrixNM<double, 6, 1> stress;
				stress.MatrMultiply(E_eps, strain);

				// Declaration and computation of Sigm, to be removed
				ChMatrixNM<double, 9, 9> Sigm;
				Sigm(0, 0) = stress(0, 0);  // XX
				Sigm(1, 1) = stress(0, 0);
				Sigm(2, 2) = stress(0, 0);

				Sigm(0, 3) = stress(2, 0);  // XY
				Sigm(1, 4) = stress(2, 0);
				Sigm(2, 5) = stress(2, 0);

				Sigm(0, 6) = stress(4, 0);  // XZ
				Sigm(1, 7) = stress(4, 0);
				Sigm(2, 8) = stress(4, 0);

				Sigm(3, 0) = stress(2, 0);  // XY
				Sigm(4, 1) = stress(2, 0);
				Sigm(5, 2) = stress(2, 0);

				Sigm(3, 3) = stress(1, 0);  // YY
				Sigm(4, 4) = stress(1, 0);
				Sigm(5, 5) = stress(1, 0);

				Sigm(3, 6) = stress(5, 0);  // YZ
				Sigm(4, 7) = stress(5, 0);
				Sigm(5, 8) = stress(5, 0);

				Sigm(6, 0) = stress(4, 0);  // XZ
				Sigm(7, 1) = stress(4, 0);
				Sigm(8, 2) = stress(4, 0);

				Sigm(6, 3) = stress(5, 0);  // YZ
				Sigm(7, 4) = stress(5, 0);
				Sigm(8, 5) = stress(5, 0);

				Sigm(6, 6) = stress(3, 0);  // ZZ
				Sigm(7, 7) = stress(3, 0);
				Sigm(8, 8) = stress(3, 0);

				// Jacobian of internal forces (excluding the EAS contribution).
				ChMatrixNM<double, 33, 6> temp336;
				ChMatrixNM<double, 33, 9> temp339;
				temp336.MatrTMultiply(strainD, E_eps);
				temp339.MatrTMultiply(Gd, Sigm);
				ChMatrixNM<double, 33, 33> KTE;

				/*temp36.MatrMultiply(E_eps, strainD);
				ChMatrixNM<double, 33, 33> KT1;
				KT1 = temp336*strainD* (m_Kfactor + m_Rfactor * m_element->m_Alpha);*/

				KTE = (temp336 * strainD) * (m_Kfactor + m_Rfactor * m_element->m_Alpha) + (temp339 * Gd) * m_Kfactor;
				KTE.MatrScale(detJ0 * m_element->m_GaussScaling);
				result = KTE;
			}
			//else if (m_element->m_strain_form == Hencky)
			else if (m_element->m_Hencky)
			{
				ChMatrixNM<double, 3, 3> Temp33;
				ChMatrixNM<double, 3, 3> CCPinv;
				ChMatrix33<double> BETRI;
				double BETRI_eig[3] = { 0.0 };
				ChMatrix33<double> BETRI_eigv;
				ChMatrixNM<double, 3, 1> e1;
				ChMatrixNM<double, 3, 1> e2;
				ChMatrixNM<double, 3, 1> e3;
				ChMatrixNM<double, 3, 3> MM1;
				ChMatrixNM<double, 3, 3> MM2;
				ChMatrixNM<double, 3, 3> MM3;
				CCPinv(0, 0) = 1.0;
				CCPinv(1, 1) = 1.0;
				CCPinv(2, 2) = 1.0;
				if (m_element->m_Plasticity)
				{
					CCPinv(0, 0) = m_element->m_CCPinv_Plast(0, m_element->m_InteCounter);
					CCPinv(0, 1) = m_element->m_CCPinv_Plast(1, m_element->m_InteCounter);
					CCPinv(0, 2) = m_element->m_CCPinv_Plast(2, m_element->m_InteCounter);
					CCPinv(1, 0) = m_element->m_CCPinv_Plast(3, m_element->m_InteCounter);
					CCPinv(1, 1) = m_element->m_CCPinv_Plast(4, m_element->m_InteCounter);
					CCPinv(1, 2) = m_element->m_CCPinv_Plast(5, m_element->m_InteCounter);
					CCPinv(2, 0) = m_element->m_CCPinv_Plast(6, m_element->m_InteCounter);
					CCPinv(2, 1) = m_element->m_CCPinv_Plast(7, m_element->m_InteCounter);
					CCPinv(2, 2) = m_element->m_CCPinv_Plast(8, m_element->m_InteCounter);
				}
				Temp33.MatrMultiply(DefF, CCPinv);
				BETRI.MatrMultiplyT(Temp33, DefF);
				BETRI.FastEigen(BETRI_eigv, BETRI_eig);
				for (int i = 0; i < 3; i++)
				{
					e3(i, 0) = BETRI_eigv(i, 0);
					e1(i, 0) = BETRI_eigv(i, 1);
					e2(i, 0) = BETRI_eigv(i, 2);
				}
				MM1.MatrMultiplyT(e1, e1);
				MM2.MatrMultiplyT(e2, e2);
				MM3.MatrMultiplyT(e3, e3);

				ChMatrixNM<double, 3, 1> LogStrain;
				LogStrain(0, 0) = 0.5*log(BETRI_eig[1]);
				LogStrain(1, 0) = 0.5*log(BETRI_eig[2]);
				LogStrain(2, 0) = 0.5*log(BETRI_eig[0]);

				ChMatrixNM<double, 3, 3> FI;
				ChMatrixNM<double, 6, 33> strainD;
				FI = DefF;
				FI.MatrInverse();
				m_element->EPSP_Euerian_SolidANCF33(strainD, Nx, Ny, Nz, FI, j0);

				ChMatrixNM<double, 3, 1> StressK_eig;
				StressK_eig(0, 0) = E_eps(0, 0)*LogStrain(0, 0) + E_eps(0, 1)*LogStrain(1, 0) + E_eps(0, 3)*LogStrain(2, 0);
				StressK_eig(1, 0) = E_eps(1, 0)*LogStrain(0, 0) + E_eps(1, 1)*LogStrain(1, 0) + E_eps(1, 3)*LogStrain(2, 0);
				StressK_eig(2, 0) = E_eps(3, 0)*LogStrain(0, 0) + E_eps(3, 1)*LogStrain(1, 0) + E_eps(3, 3)*LogStrain(2, 0);
				int YieldFlag = 0;
				ChMatrixNM<double, 6, 6> Dep;
				if (m_element->m_Plasticity)
				{
					double G = E / (2.0*(1 + nu));
					double K = E / (3.0*(1 - 2.0*nu));

					double EEVD3 = (LogStrain(0, 0) + LogStrain(1, 0) + LogStrain(2, 0)) / 3.0;
					ChVector<double> EETD;
					EETD.x = LogStrain(0, 0) - EEVD3;
					EETD.y = LogStrain(1, 0) - EEVD3;
					EETD.z = LogStrain(2, 0) - EEVD3;
					double ETDNorm = sqrt(EETD.x*EETD.x + EETD.y*EETD.y + EETD.z*EETD.z);

					double hydroP = (StressK_eig(0, 0) + StressK_eig(1, 0) + StressK_eig(2, 0)) / 3.0;
					ChVector<double> devStress;
					devStress.x = StressK_eig(0, 0) - hydroP;
					devStress.y = StressK_eig(1, 0) - hydroP;
					devStress.z = StressK_eig(2, 0) - hydroP;
					double NormSn = sqrt(devStress.x*devStress.x + devStress.y*devStress.y + devStress.z*devStress.z);

					double J2Rt = NormSn / sqrt(2.0);
					double qtrial = sqrt(3.0)*J2Rt;

					double YieldFunc = qtrial - (m_element->m_YieldStress + m_element->m_HardeningSlope*m_element->m_Alpha_Plast(m_element->m_InteCounter, 0));
					if (YieldFunc > 0.0)
					{
						YieldFlag = 1;

						double DeltaGamma = YieldFunc / (3.0*G + m_element->m_HardeningSlope);
						ChVector<double> devStressUp;
						if (qtrial != 0.0)
						{
							devStressUp = (1.0 - G*DeltaGamma*3.0 / qtrial)*devStress;
						}
						else{
							devStressUp = devStress;
						}

						StressK_eig(0, 0) = devStressUp.x + hydroP;
						StressK_eig(1, 0) = devStressUp.y + hydroP;
						StressK_eig(2, 0) = devStressUp.z + hydroP;

						LogStrain(0, 0) = devStressUp.x / (2.0*G) + EEVD3;
						LogStrain(1, 0) = devStressUp.y / (2.0*G) + EEVD3;
						LogStrain(2, 0) = devStressUp.z / (2.0*G) + EEVD3;

						ChVector<double> lambda;
						lambda.x = exp(2.0*LogStrain(0, 0));
						lambda.y = exp(2.0*LogStrain(1, 0));
						lambda.z = exp(2.0*LogStrain(2, 0));
						ChMatrixNM<double, 3, 3> BEUP;
						MM1.MatrScale(lambda.x);
						MM2.MatrScale(lambda.y);
						MM3.MatrScale(lambda.z);
						BEUP = MM1 + MM2 + MM3;
						MM1.MatrScale(1 / lambda.x);
						MM2.MatrScale(1 / lambda.y);
						MM3.MatrScale(1 / lambda.z);
						m_element->m_Alpha_Plast(m_element->m_InteCounter, 0) = m_element->m_Alpha_Plast(m_element->m_InteCounter, 0) + DeltaGamma;
						Temp33.MatrMultiply(FI, BEUP);
						CCPinv.MatrMultiplyT(Temp33, FI);
						// Store CPPinv
						m_element->m_CCPinv_Plast(0, m_element->m_InteCounter) = CCPinv(0, 0);
						m_element->m_CCPinv_Plast(1, m_element->m_InteCounter) = CCPinv(0, 1);
						m_element->m_CCPinv_Plast(2, m_element->m_InteCounter) = CCPinv(0, 2);
						m_element->m_CCPinv_Plast(3, m_element->m_InteCounter) = CCPinv(1, 0);
						m_element->m_CCPinv_Plast(4, m_element->m_InteCounter) = CCPinv(1, 1);
						m_element->m_CCPinv_Plast(5, m_element->m_InteCounter) = CCPinv(1, 2);
						m_element->m_CCPinv_Plast(6, m_element->m_InteCounter) = CCPinv(2, 0);
						m_element->m_CCPinv_Plast(7, m_element->m_InteCounter) = CCPinv(2, 1);
						m_element->m_CCPinv_Plast(8, m_element->m_InteCounter) = CCPinv(2, 2);

						//// ONLY FOR JACOBIAN!!!!!
						qtrial = sqrt(3.0 / 2.0)*NormSn + 3.0*G*DeltaGamma;
						double AFACT = 2.0*G*(1.0 - 3.0*G*DeltaGamma / qtrial);
						double BFACT = 6.0*G*G*(DeltaGamma / qtrial - 1.0 / (3.0*G + m_element->m_HardeningSlope)) / (NormSn*NormSn);

						ChMatrixNM<double, 6, 1> devStressVec;
						devStressVec(0, 0) = devStress.x;
						devStressVec(1, 0) = devStress.y;
						devStressVec(3, 0) = devStress.z;

						ChMatrixNM<double, 6, 6> FOID;
						ChMatrixNM<double, 6, 1> SOID;

						FOID(0, 0) = 1.0;
						FOID(1, 1) = 1.0;
						FOID(2, 2) = 0.5;
						FOID(3, 3) = 1.0;
						FOID(4, 4) = 0.5;
						FOID(5, 5) = 0.5;

						SOID(0, 0) = 1.0;
						SOID(1, 0) = 1.0;
						SOID(3, 0) = 1.0;

						ChMatrixNM<double, 6, 6> DEVPRJ;

						for (int i = 0; i < 6; i++)
						{
							for (int j = i; j < 6; j++)
							{
								DEVPRJ(i, j) = FOID(i, j) - SOID(i, 0)*SOID(j, 0) / 3.0;
								Dep(i, j) = AFACT*DEVPRJ(i, j) + BFACT*devStressVec(i,0)*devStressVec(j,0) + K*SOID(i,0)*SOID(j,0);
							}
						}

						for (int j = 0; j < 5; j++)
						{
							for (int i = j + 1; i < 6; i++)
							{
								Dep(i, j) = Dep(j, i);
							}
						}

					}

				}

				ChMatrixNM<double, 3, 3> StressK;
				MM1.MatrScale(StressK_eig(0, 0));
				MM2.MatrScale(StressK_eig(1, 0));
				MM3.MatrScale(StressK_eig(2, 0));
				StressK = MM1 + MM2 + MM3;

				ChMatrixNM<double, 6, 1> DEPS;
				DEPS.Reset();
				for (int ii = 0; ii < 33; ii++) {
					DEPS(0, 0) = DEPS(0, 0) + strainD(0, ii) * m_element->m_d_dt(ii, 0);
					DEPS(1, 0) = DEPS(1, 0) + strainD(1, ii) * m_element->m_d_dt(ii, 0);
					DEPS(2, 0) = DEPS(2, 0) + strainD(2, ii) * m_element->m_d_dt(ii, 0);
					DEPS(3, 0) = DEPS(3, 0) + strainD(3, ii) * m_element->m_d_dt(ii, 0);
					DEPS(4, 0) = DEPS(4, 0) + strainD(4, ii) * m_element->m_d_dt(ii, 0);
					DEPS(5, 0) = DEPS(5, 0) + strainD(5, ii) * m_element->m_d_dt(ii, 0);
				}

				ChMatrixNM<double, 6, 1> Stress_damp;
				// Add structural damping
				Stress_damp.MatrMultiply(E_eps, DEPS);
				Stress_damp.MatrScale(m_element->m_Alpha);

				ChMatrixNM<double, 6, 1> Stress;
				Stress(0, 0) = StressK(0, 0) + Stress_damp(0, 0);
				Stress(1, 0) = StressK(1, 1) + Stress_damp(1, 0);
				Stress(2, 0) = 0.5*(StressK(1, 0) + StressK(0, 1)) + Stress_damp(2, 0);
				Stress(3, 0) = StressK(2, 2) + Stress_damp(3, 0);
				Stress(4, 0) = 0.5*(StressK(2, 0) + StressK(0, 2)) + Stress_damp(4, 0);
				Stress(5, 0) = 0.5*(StressK(2, 1) + StressK(1, 2)) + Stress_damp(5, 0);

				// Declaration and computation of Sigm, to be removed
				ChMatrixNM<double, 9, 9> Sigm;
				Sigm(0, 0) = Stress(0, 0);  // XX
				Sigm(1, 1) = Stress(0, 0);
				Sigm(2, 2) = Stress(0, 0);

				Sigm(0, 3) = Stress(2, 0);  // XY
				Sigm(1, 4) = Stress(2, 0);
				Sigm(2, 5) = Stress(2, 0);

				Sigm(0, 6) = Stress(4, 0);  // XZ
				Sigm(1, 7) = Stress(4, 0);
				Sigm(2, 8) = Stress(4, 0);

				Sigm(3, 0) = Stress(2, 0);  // XY
				Sigm(4, 1) = Stress(2, 0);
				Sigm(5, 2) = Stress(2, 0);

				Sigm(3, 3) = Stress(1, 0);  // YY
				Sigm(4, 4) = Stress(1, 0);
				Sigm(5, 5) = Stress(1, 0);

				Sigm(3, 6) = Stress(5, 0);  // YZ
				Sigm(4, 7) = Stress(5, 0);
				Sigm(5, 8) = Stress(5, 0);

				Sigm(6, 0) = Stress(4, 0);  // XZ
				Sigm(7, 1) = Stress(4, 0);
				Sigm(8, 2) = Stress(4, 0);

				Sigm(6, 3) = Stress(5, 0);  // YZ
				Sigm(7, 4) = Stress(5, 0);
				Sigm(8, 5) = Stress(5, 0);

				Sigm(6, 6) = Stress(3, 0);  // ZZ
				Sigm(7, 7) = Stress(3, 0);
				Sigm(8, 8) = Stress(3, 0);

				ChMatrixNM<double, 9, 33> Gd;
				for (int ii = 0; ii < 11; ii++) {
					Gd(0, 3 * (ii)) = j0(0, 0) * Nx(0, ii) + j0(1, 0) * Ny(0, ii) + j0(2, 0) * Nz(0, ii);
					Gd(1, 3 * (ii)+1) = j0(0, 0) * Nx(0, ii) + j0(1, 0) * Ny(0, ii) + j0(2, 0) * Nz(0, ii);
					Gd(2, 3 * (ii)+2) = j0(0, 0) * Nx(0, ii) + j0(1, 0) * Ny(0, ii) + j0(2, 0) * Nz(0, ii);

					Gd(3, 3 * (ii)) = j0(0, 1) * Nx(0, ii) + j0(1, 1) * Ny(0, ii) + j0(2, 1) * Nz(0, ii);
					Gd(4, 3 * (ii)+1) = j0(0, 1) * Nx(0, ii) + j0(1, 1) * Ny(0, ii) + j0(2, 1) * Nz(0, ii);
					Gd(5, 3 * (ii)+2) = j0(0, 1) * Nx(0, ii) + j0(1, 1) * Ny(0, ii) + j0(2, 1) * Nz(0, ii);

					Gd(6, 3 * (ii)) = j0(0, 2) * Nx(0, ii) + j0(1, 2) * Ny(0, ii) + j0(2, 2) * Nz(0, ii);
					Gd(7, 3 * (ii)+1) = j0(0, 2) * Nx(0, ii) + j0(1, 2) * Ny(0, ii) + j0(2, 2) * Nz(0, ii);
					Gd(8, 3 * (ii)+2) = j0(0, 2) * Nx(0, ii) + j0(1, 2) * Ny(0, ii) + j0(2, 2) * Nz(0, ii);
				}

				// Jacobian of internal forces (excluding the EAS contribution).
				ChMatrixNM<double, 33, 6> temp336;
				ChMatrixNM<double, 33, 9> temp339;
				if (m_element->m_Plasticity&&YieldFlag==1)
				{
					temp336.MatrTMultiply(strainD, Dep);
				}
				else
				{
					temp336.MatrTMultiply(strainD, E_eps);
				}
				
				temp339.MatrTMultiply(Gd, Sigm);
				ChMatrixNM<double, 33, 33> KTE;

				KTE = (temp336 * strainD) * (m_Kfactor + m_Rfactor * m_element->m_Alpha) + (temp339 * Gd) * m_Kfactor;
				KTE.MatrScale(detJ0 * m_element->m_GaussScaling);
				result = KTE;
				m_element->m_InteCounter++;
			}
		}

		// Compute the Jacobian of the internal forces
		void ChElementBrick_9::ComputeInternalJacobians(double Kfactor, double Rfactor) {
			m_JacobianMatrix.Reset();
			m_InteCounter = 0;
			ChMatrixNM<double, 33, 33> result;
			MyJacobianBrick9 formula(this, Kfactor, Rfactor);
			ChQuadrature::Integrate3D<ChMatrixNM<double, 33, 33> >(result,                         // result of integration
				formula,                         // integrand formula
				-1, 1,                           // x limits
				-1, 1,                           // y limits
				-1, 1,						   // z limits
				2                                // order of integration
				);
			// Accumulate Jacobian
			m_JacobianMatrix += result;
			//for (int i = 0; i < 33; i++)
			//{
			//	GetLog() << result(i,0) << "\n";
			//	system("pause");
			//}

		}

		// -----------------------------------------------------------------------------
		// Interface to implicit integrators
		// -----------------------------------------------------------------------------

		// Update element at new time
		void ChElementBrick_9::Update() {
			ChElementGeneric::Update();
		}

		// Fill the D vector (column matrix) with the current states at the nodes of
		// the element, with proper ordering.
		void ChElementBrick_9::GetStateBlock(ChMatrixDynamic<>& mD) {
			for (int i = 0; i < 8; i++) {
				mD.PasteVector(m_nodes[i]->GetPos(), 3 * i, 0);
			}
			mD.PasteVector(m_central_node->GetCurvatureXX(), 24, 0);
			mD.PasteVector(m_central_node->GetCurvatureYY(), 27, 0);
			mD.PasteVector(m_central_node->GetCurvatureZZ(), 30, 0);
		}

		// Return the mass matrix.
		void ChElementBrick_9::ComputeMmatrixGlobal(ChMatrix<>& M) {
			M = m_MassMatrix;
		}

		// Calculate the global matrix H as a linear combination of K, R, and M:
		//   H = Mfactor * (M) + Kfactor * (K) + Rfactor * (R)
		void ChElementBrick_9::ComputeKRMmatricesGlobal(ChMatrix<>& H, double Kfactor, double Rfactor, double Mfactor) {
			assert((H.GetRows() == 33) && (H.GetColumns() == 33));

			// Calculate the linear combination Kfactor*(K) + Rfactor*(R)
			ComputeInternalJacobians(Kfactor, Rfactor);

			// Load Jac + Mfactor*(M) into H
			for (int i = 0; i < 33; i++)
			for (int j = 0; j < 33; j++)
				H(i, j) = m_JacobianMatrix(i, j) + Mfactor * m_MassMatrix(i, j);
		}

		// -----------------------------------------------------------------------------
		// Implementation of interface to ChLoadableUVW
		// -----------------------------------------------------------------------------

		// Get all the DOFs packed in a single vector (position part).
		void ChElementBrick_9::LoadableGetStateBlock_x(int block_offset, ChVectorDynamic<>& mD) {
			for (int i = 0; i < 8; i++) {
				mD.PasteVector(m_nodes[i]->GetPos(), block_offset + 3 * i, 0);
			}
			mD.PasteVector(m_central_node->GetCurvatureXX(), block_offset + 24, 0);
			mD.PasteVector(m_central_node->GetCurvatureYY(), block_offset + 27, 0);
			mD.PasteVector(m_central_node->GetCurvatureZZ(), block_offset + 30, 0);
		}

		// Get all the DOFs packed in a single vector (speed part).
		void ChElementBrick_9::LoadableGetStateBlock_w(int block_offset, ChVectorDynamic<>& mD) {
			for (int i = 0; i < 8; i++) {
				mD.PasteVector(this->m_nodes[i]->GetPos_dt(), block_offset + 3 * i, 0);
			}
			mD.PasteVector(m_central_node->GetCurvatureXX_dt(), block_offset + 24, 0);
			mD.PasteVector(m_central_node->GetCurvatureYY_dt(), block_offset + 27, 0);
			mD.PasteVector(m_central_node->GetCurvatureZZ_dt(), block_offset + 30, 0);
		}

		// Get the pointers to the contained ChLcpVariables, appending to the mvars vector.
		void ChElementBrick_9::LoadableGetVariables(std::vector<ChLcpVariables*>& mvars) {
			for (int i = 0; i < 8; ++i) {
				mvars.push_back(&m_nodes[i]->Variables());
			}
			mvars.push_back(&m_central_node->Variables());
		}

		// Evaluate N'*F , where N is some type of shape function evaluated at (U,V,W).
		// Here, U,V,W are coordinates of the volume, each ranging in -1..+1
		// F is a load, N'*F is the resulting generalized load
		// Returns also det(J) with J=(dx/du,..), that might be useful in gauss quadrature.
		void ChElementBrick_9::ComputeNF(
			const double U,              // parametric coordinate in volume
			const double V,              // parametric coordinate in volume
			const double W,              // parametric coordinate in volume
			ChVectorDynamic<>& Qi,       // Return result of N'*F  here, maybe with offset block_offset
			double& detJ,                // Return det(J) here
			const ChVectorDynamic<>& F,  // Input F vector, size is = n.field coords.
			ChVectorDynamic<>* state_x,  // if != 0, update state (pos. part) to this, then evaluate Q
			ChVectorDynamic<>* state_w   // if != 0, update state (speed part) to this, then evaluate Q
			) {
			ChMatrixNM<double, 1, 11> N;
			ShapeFunctions(N, U, V, W);

			detJ = Calc_detJ0(U, V, W);
			detJ *= m_dimensions.x * m_dimensions.y * m_dimensions.z / 8;

			ChVector<> Fv = F.ClipVector(0, 0);
			for (int i = 0; i < 11; i++) {
				Qi.PasteVector(N(i) * Fv, 3 * i, 0);
			}
		}


		void ChElementBrick_9::EPSP_Euerian_SolidANCF33(ChMatrixNM<double, 6, 33> &strainD,
			ChMatrixNM<double, 1, 11> Nx,
			ChMatrixNM<double, 1, 11> Ny,
			ChMatrixNM<double, 1, 11> Nz,
			ChMatrixNM<double, 3, 3> FI,
			ChMatrixNM<double, 3, 3> J0I){

			strainD.Reset();

			strainD(0, 0) = Nx(0, 0)* (FI(0, 0) * J0I(0, 0) + FI(1, 0) * J0I(0, 1) + FI(2, 0) * J0I(0, 2)) + Ny(0, 0)* (FI(0, 0) * J0I(1, 0) + FI(1, 0) * J0I(1, 1) + FI(2, 0) * J0I(1, 2)) + Nz(0, 0)* (FI(0, 0) * J0I(2, 0) + FI(1, 0) * J0I(2, 1) + FI(2, 0) * J0I(2, 2));
			strainD(0, 3) = Nx(0, 1)* (FI(0, 0) * J0I(0, 0) + FI(1, 0) * J0I(0, 1) + FI(2, 0) * J0I(0, 2)) + Ny(0, 1)* (FI(0, 0) * J0I(1, 0) + FI(1, 0) * J0I(1, 1) + FI(2, 0) * J0I(1, 2)) + Nz(0, 1)* (FI(0, 0) * J0I(2, 0) + FI(1, 0) * J0I(2, 1) + FI(2, 0) * J0I(2, 2));
			strainD(0, 6) = Nx(0, 2)* (FI(0, 0) * J0I(0, 0) + FI(1, 0) * J0I(0, 1) + FI(2, 0) * J0I(0, 2)) + Ny(0, 2)* (FI(0, 0) * J0I(1, 0) + FI(1, 0) * J0I(1, 1) + FI(2, 0) * J0I(1, 2)) + Nz(0, 2)* (FI(0, 0) * J0I(2, 0) + FI(1, 0) * J0I(2, 1) + FI(2, 0) * J0I(2, 2));
			strainD(0, 9) = Nx(0, 3)* (FI(0, 0) * J0I(0, 0) + FI(1, 0) * J0I(0, 1) + FI(2, 0) * J0I(0, 2)) + Ny(0, 3)* (FI(0, 0) * J0I(1, 0) + FI(1, 0) * J0I(1, 1) + FI(2, 0) * J0I(1, 2)) + Nz(0, 3)* (FI(0, 0) * J0I(2, 0) + FI(1, 0) * J0I(2, 1) + FI(2, 0) * J0I(2, 2));
			strainD(0, 12) = Nx(0, 4)* (FI(0, 0) * J0I(0, 0) + FI(1, 0) * J0I(0, 1) + FI(2, 0) * J0I(0, 2)) + Ny(0, 4)* (FI(0, 0) * J0I(1, 0) + FI(1, 0) * J0I(1, 1) + FI(2, 0) * J0I(1, 2)) + Nz(0, 4)* (FI(0, 0) * J0I(2, 0) + FI(1, 0) * J0I(2, 1) + FI(2, 0) * J0I(2, 2));
			strainD(0, 15) = Nx(0, 5)* (FI(0, 0) * J0I(0, 0) + FI(1, 0) * J0I(0, 1) + FI(2, 0) * J0I(0, 2)) + Ny(0, 5)* (FI(0, 0) * J0I(1, 0) + FI(1, 0) * J0I(1, 1) + FI(2, 0) * J0I(1, 2)) + Nz(0, 5)* (FI(0, 0) * J0I(2, 0) + FI(1, 0) * J0I(2, 1) + FI(2, 0) * J0I(2, 2));
			strainD(0, 18) = Nx(0, 6)* (FI(0, 0) * J0I(0, 0) + FI(1, 0) * J0I(0, 1) + FI(2, 0) * J0I(0, 2)) + Ny(0, 6)* (FI(0, 0) * J0I(1, 0) + FI(1, 0) * J0I(1, 1) + FI(2, 0) * J0I(1, 2)) + Nz(0, 6)* (FI(0, 0) * J0I(2, 0) + FI(1, 0) * J0I(2, 1) + FI(2, 0) * J0I(2, 2));
			strainD(0, 21) = Nx(0, 7)* (FI(0, 0) * J0I(0, 0) + FI(1, 0) * J0I(0, 1) + FI(2, 0) * J0I(0, 2)) + Ny(0, 7)* (FI(0, 0) * J0I(1, 0) + FI(1, 0) * J0I(1, 1) + FI(2, 0) * J0I(1, 2)) + Nz(0, 7)* (FI(0, 0) * J0I(2, 0) + FI(1, 0) * J0I(2, 1) + FI(2, 0) * J0I(2, 2));
			strainD(0, 24) = Nx(0, 8)* (FI(0, 0) * J0I(0, 0) + FI(1, 0) * J0I(0, 1) + FI(2, 0) * J0I(0, 2)) + Ny(0, 8)* (FI(0, 0) * J0I(1, 0) + FI(1, 0) * J0I(1, 1) + FI(2, 0) * J0I(1, 2)) + Nz(0, 8)* (FI(0, 0) * J0I(2, 0) + FI(1, 0) * J0I(2, 1) + FI(2, 0) * J0I(2, 2));
			strainD(0, 27) = Nx(0, 9)* (FI(0, 0) * J0I(0, 0) + FI(1, 0) * J0I(0, 1) + FI(2, 0) * J0I(0, 2)) + Ny(0, 9)* (FI(0, 0) * J0I(1, 0) + FI(1, 0) * J0I(1, 1) + FI(2, 0) * J0I(1, 2)) + Nz(0, 9)* (FI(0, 0) * J0I(2, 0) + FI(1, 0) * J0I(2, 1) + FI(2, 0) * J0I(2, 2));
			strainD(0, 30) = Nx(0, 10)* (FI(0, 0) * J0I(0, 0) + FI(1, 0) * J0I(0, 1) + FI(2, 0) * J0I(0, 2)) + Ny(0, 10)* (FI(0, 0) * J0I(1, 0) + FI(1, 0) * J0I(1, 1) + FI(2, 0) * J0I(1, 2)) + Nz(0, 10)* (FI(0, 0) * J0I(2, 0) + FI(1, 0) * J0I(2, 1) + FI(2, 0) * J0I(2, 2));
			strainD(1, 1) = Nx(0, 0)* (FI(0, 1) * J0I(0, 0) + FI(1, 1) * J0I(0, 1) + FI(2, 1) * J0I(0, 2)) + Ny(0, 0)* (FI(0, 1) * J0I(1, 0) + FI(1, 1) * J0I(1, 1) + FI(2, 1) * J0I(1, 2)) + Nz(0, 0)* (FI(0, 1) * J0I(2, 0) + FI(1, 1) * J0I(2, 1) + FI(2, 1) * J0I(2, 2));
			strainD(1, 4) = Nx(0, 1)* (FI(0, 1) * J0I(0, 0) + FI(1, 1) * J0I(0, 1) + FI(2, 1) * J0I(0, 2)) + Ny(0, 1)* (FI(0, 1) * J0I(1, 0) + FI(1, 1) * J0I(1, 1) + FI(2, 1) * J0I(1, 2)) + Nz(0, 1)* (FI(0, 1) * J0I(2, 0) + FI(1, 1) * J0I(2, 1) + FI(2, 1) * J0I(2, 2));
			strainD(1, 7) = Nx(0, 2)* (FI(0, 1) * J0I(0, 0) + FI(1, 1) * J0I(0, 1) + FI(2, 1) * J0I(0, 2)) + Ny(0, 2)* (FI(0, 1) * J0I(1, 0) + FI(1, 1) * J0I(1, 1) + FI(2, 1) * J0I(1, 2)) + Nz(0, 2)* (FI(0, 1) * J0I(2, 0) + FI(1, 1) * J0I(2, 1) + FI(2, 1) * J0I(2, 2));
			strainD(1, 10) = Nx(0, 3)* (FI(0, 1) * J0I(0, 0) + FI(1, 1) * J0I(0, 1) + FI(2, 1) * J0I(0, 2)) + Ny(0, 3)* (FI(0, 1) * J0I(1, 0) + FI(1, 1) * J0I(1, 1) + FI(2, 1) * J0I(1, 2)) + Nz(0, 3)* (FI(0, 1) * J0I(2, 0) + FI(1, 1) * J0I(2, 1) + FI(2, 1) * J0I(2, 2));
			strainD(1, 13) = Nx(0, 4)* (FI(0, 1) * J0I(0, 0) + FI(1, 1) * J0I(0, 1) + FI(2, 1) * J0I(0, 2)) + Ny(0, 4)* (FI(0, 1) * J0I(1, 0) + FI(1, 1) * J0I(1, 1) + FI(2, 1) * J0I(1, 2)) + Nz(0, 4)* (FI(0, 1) * J0I(2, 0) + FI(1, 1) * J0I(2, 1) + FI(2, 1) * J0I(2, 2));
			strainD(1, 16) = Nx(0, 5)* (FI(0, 1) * J0I(0, 0) + FI(1, 1) * J0I(0, 1) + FI(2, 1) * J0I(0, 2)) + Ny(0, 5)* (FI(0, 1) * J0I(1, 0) + FI(1, 1) * J0I(1, 1) + FI(2, 1) * J0I(1, 2)) + Nz(0, 5)* (FI(0, 1) * J0I(2, 0) + FI(1, 1) * J0I(2, 1) + FI(2, 1) * J0I(2, 2));
			strainD(1, 19) = Nx(0, 6)* (FI(0, 1) * J0I(0, 0) + FI(1, 1) * J0I(0, 1) + FI(2, 1) * J0I(0, 2)) + Ny(0, 6)* (FI(0, 1) * J0I(1, 0) + FI(1, 1) * J0I(1, 1) + FI(2, 1) * J0I(1, 2)) + Nz(0, 6)* (FI(0, 1) * J0I(2, 0) + FI(1, 1) * J0I(2, 1) + FI(2, 1) * J0I(2, 2));
			strainD(1, 22) = Nx(0, 7)* (FI(0, 1) * J0I(0, 0) + FI(1, 1) * J0I(0, 1) + FI(2, 1) * J0I(0, 2)) + Ny(0, 7)* (FI(0, 1) * J0I(1, 0) + FI(1, 1) * J0I(1, 1) + FI(2, 1) * J0I(1, 2)) + Nz(0, 7)* (FI(0, 1) * J0I(2, 0) + FI(1, 1) * J0I(2, 1) + FI(2, 1) * J0I(2, 2));
			strainD(1, 25) = Nx(0, 8)* (FI(0, 1) * J0I(0, 0) + FI(1, 1) * J0I(0, 1) + FI(2, 1) * J0I(0, 2)) + Ny(0, 8)* (FI(0, 1) * J0I(1, 0) + FI(1, 1) * J0I(1, 1) + FI(2, 1) * J0I(1, 2)) + Nz(0, 8)* (FI(0, 1) * J0I(2, 0) + FI(1, 1) * J0I(2, 1) + FI(2, 1) * J0I(2, 2));
			strainD(1, 28) = Nx(0, 9)* (FI(0, 1) * J0I(0, 0) + FI(1, 1) * J0I(0, 1) + FI(2, 1) * J0I(0, 2)) + Ny(0, 9)* (FI(0, 1) * J0I(1, 0) + FI(1, 1) * J0I(1, 1) + FI(2, 1) * J0I(1, 2)) + Nz(0, 9)* (FI(0, 1) * J0I(2, 0) + FI(1, 1) * J0I(2, 1) + FI(2, 1) * J0I(2, 2));
			strainD(1, 31) = Nx(0, 10)* (FI(0, 1) * J0I(0, 0) + FI(1, 1) * J0I(0, 1) + FI(2, 1) * J0I(0, 2)) + Ny(0, 10)* (FI(0, 1) * J0I(1, 0) + FI(1, 1) * J0I(1, 1) + FI(2, 1) * J0I(1, 2)) + Nz(0, 10)* (FI(0, 1) * J0I(2, 0) + FI(1, 1) * J0I(2, 1) + FI(2, 1) * J0I(2, 2));
			strainD(2, 0) = Nx(0, 0)* (FI(0, 1) * J0I(0, 0) + FI(1, 1) * J0I(0, 1) + FI(2, 1) * J0I(0, 2)) + Ny(0, 0)* (FI(0, 1) * J0I(1, 0) + FI(1, 1) * J0I(1, 1) + FI(2, 1) * J0I(1, 2)) + Nz(0, 0)* (FI(0, 1) * J0I(2, 0) + FI(1, 1) * J0I(2, 1) + FI(2, 1) * J0I(2, 2));
			strainD(2, 1) = Nx(0, 0)* (FI(0, 0) * J0I(0, 0) + FI(1, 0) * J0I(0, 1) + FI(2, 0) * J0I(0, 2)) + Ny(0, 0)* (FI(0, 0) * J0I(1, 0) + FI(1, 0) * J0I(1, 1) + FI(2, 0) * J0I(1, 2)) + Nz(0, 0)* (FI(0, 0) * J0I(2, 0) + FI(1, 0) * J0I(2, 1) + FI(2, 0) * J0I(2, 2));
			strainD(2, 3) = Nx(0, 1)* (FI(0, 1) * J0I(0, 0) + FI(1, 1) * J0I(0, 1) + FI(2, 1) * J0I(0, 2)) + Ny(0, 1)* (FI(0, 1) * J0I(1, 0) + FI(1, 1) * J0I(1, 1) + FI(2, 1) * J0I(1, 2)) + Nz(0, 1)* (FI(0, 1) * J0I(2, 0) + FI(1, 1) * J0I(2, 1) + FI(2, 1) * J0I(2, 2));
			strainD(2, 4) = Nx(0, 1)* (FI(0, 0) * J0I(0, 0) + FI(1, 0) * J0I(0, 1) + FI(2, 0) * J0I(0, 2)) + Ny(0, 1)* (FI(0, 0) * J0I(1, 0) + FI(1, 0) * J0I(1, 1) + FI(2, 0) * J0I(1, 2)) + Nz(0, 1)* (FI(0, 0) * J0I(2, 0) + FI(1, 0) * J0I(2, 1) + FI(2, 0) * J0I(2, 2));
			strainD(2, 6) = Nx(0, 2)* (FI(0, 1) * J0I(0, 0) + FI(1, 1) * J0I(0, 1) + FI(2, 1) * J0I(0, 2)) + Ny(0, 2)* (FI(0, 1) * J0I(1, 0) + FI(1, 1) * J0I(1, 1) + FI(2, 1) * J0I(1, 2)) + Nz(0, 2)* (FI(0, 1) * J0I(2, 0) + FI(1, 1) * J0I(2, 1) + FI(2, 1) * J0I(2, 2));
			strainD(2, 7) = Nx(0, 2)* (FI(0, 0) * J0I(0, 0) + FI(1, 0) * J0I(0, 1) + FI(2, 0) * J0I(0, 2)) + Ny(0, 2)* (FI(0, 0) * J0I(1, 0) + FI(1, 0) * J0I(1, 1) + FI(2, 0) * J0I(1, 2)) + Nz(0, 2)* (FI(0, 0) * J0I(2, 0) + FI(1, 0) * J0I(2, 1) + FI(2, 0) * J0I(2, 2));
			strainD(2, 9) = Nx(0, 3)* (FI(0, 1) * J0I(0, 0) + FI(1, 1) * J0I(0, 1) + FI(2, 1) * J0I(0, 2)) + Ny(0, 3)* (FI(0, 1) * J0I(1, 0) + FI(1, 1) * J0I(1, 1) + FI(2, 1) * J0I(1, 2)) + Nz(0, 3)* (FI(0, 1) * J0I(2, 0) + FI(1, 1) * J0I(2, 1) + FI(2, 1) * J0I(2, 2));
			strainD(2, 10) = Nx(0, 3)* (FI(0, 0) * J0I(0, 0) + FI(1, 0) * J0I(0, 1) + FI(2, 0) * J0I(0, 2)) + Ny(0, 3)* (FI(0, 0) * J0I(1, 0) + FI(1, 0) * J0I(1, 1) + FI(2, 0) * J0I(1, 2)) + Nz(0, 3)* (FI(0, 0) * J0I(2, 0) + FI(1, 0) * J0I(2, 1) + FI(2, 0) * J0I(2, 2));
			strainD(2, 12) = Nx(0, 4)* (FI(0, 1) * J0I(0, 0) + FI(1, 1) * J0I(0, 1) + FI(2, 1) * J0I(0, 2)) + Ny(0, 4)* (FI(0, 1) * J0I(1, 0) + FI(1, 1) * J0I(1, 1) + FI(2, 1) * J0I(1, 2)) + Nz(0, 4)* (FI(0, 1) * J0I(2, 0) + FI(1, 1) * J0I(2, 1) + FI(2, 1) * J0I(2, 2));
			strainD(2, 13) = Nx(0, 4)* (FI(0, 0) * J0I(0, 0) + FI(1, 0) * J0I(0, 1) + FI(2, 0) * J0I(0, 2)) + Ny(0, 4)* (FI(0, 0) * J0I(1, 0) + FI(1, 0) * J0I(1, 1) + FI(2, 0) * J0I(1, 2)) + Nz(0, 4)* (FI(0, 0) * J0I(2, 0) + FI(1, 0) * J0I(2, 1) + FI(2, 0) * J0I(2, 2));
			strainD(2, 15) = Nx(0, 5)* (FI(0, 1) * J0I(0, 0) + FI(1, 1) * J0I(0, 1) + FI(2, 1) * J0I(0, 2)) + Ny(0, 5)* (FI(0, 1) * J0I(1, 0) + FI(1, 1) * J0I(1, 1) + FI(2, 1) * J0I(1, 2)) + Nz(0, 5)* (FI(0, 1) * J0I(2, 0) + FI(1, 1) * J0I(2, 1) + FI(2, 1) * J0I(2, 2));
			strainD(2, 16) = Nx(0, 5)* (FI(0, 0) * J0I(0, 0) + FI(1, 0) * J0I(0, 1) + FI(2, 0) * J0I(0, 2)) + Ny(0, 5)* (FI(0, 0) * J0I(1, 0) + FI(1, 0) * J0I(1, 1) + FI(2, 0) * J0I(1, 2)) + Nz(0, 5)* (FI(0, 0) * J0I(2, 0) + FI(1, 0) * J0I(2, 1) + FI(2, 0) * J0I(2, 2));
			strainD(2, 18) = Nx(0, 6)* (FI(0, 1) * J0I(0, 0) + FI(1, 1) * J0I(0, 1) + FI(2, 1) * J0I(0, 2)) + Ny(0, 6)* (FI(0, 1) * J0I(1, 0) + FI(1, 1) * J0I(1, 1) + FI(2, 1) * J0I(1, 2)) + Nz(0, 6)* (FI(0, 1) * J0I(2, 0) + FI(1, 1) * J0I(2, 1) + FI(2, 1) * J0I(2, 2));
			strainD(2, 19) = Nx(0, 6)* (FI(0, 0) * J0I(0, 0) + FI(1, 0) * J0I(0, 1) + FI(2, 0) * J0I(0, 2)) + Ny(0, 6)* (FI(0, 0) * J0I(1, 0) + FI(1, 0) * J0I(1, 1) + FI(2, 0) * J0I(1, 2)) + Nz(0, 6)* (FI(0, 0) * J0I(2, 0) + FI(1, 0) * J0I(2, 1) + FI(2, 0) * J0I(2, 2));
			strainD(2, 21) = Nx(0, 7)* (FI(0, 1) * J0I(0, 0) + FI(1, 1) * J0I(0, 1) + FI(2, 1) * J0I(0, 2)) + Ny(0, 7)* (FI(0, 1) * J0I(1, 0) + FI(1, 1) * J0I(1, 1) + FI(2, 1) * J0I(1, 2)) + Nz(0, 7)* (FI(0, 1) * J0I(2, 0) + FI(1, 1) * J0I(2, 1) + FI(2, 1) * J0I(2, 2));
			strainD(2, 22) = Nx(0, 7)* (FI(0, 0) * J0I(0, 0) + FI(1, 0) * J0I(0, 1) + FI(2, 0) * J0I(0, 2)) + Ny(0, 7)* (FI(0, 0) * J0I(1, 0) + FI(1, 0) * J0I(1, 1) + FI(2, 0) * J0I(1, 2)) + Nz(0, 7)* (FI(0, 0) * J0I(2, 0) + FI(1, 0) * J0I(2, 1) + FI(2, 0) * J0I(2, 2));
			strainD(2, 24) = Nx(0, 8)* (FI(0, 1) * J0I(0, 0) + FI(1, 1) * J0I(0, 1) + FI(2, 1) * J0I(0, 2)) + Ny(0, 8)* (FI(0, 1) * J0I(1, 0) + FI(1, 1) * J0I(1, 1) + FI(2, 1) * J0I(1, 2)) + Nz(0, 8)* (FI(0, 1) * J0I(2, 0) + FI(1, 1) * J0I(2, 1) + FI(2, 1) * J0I(2, 2));
			strainD(2, 25) = Nx(0, 8)* (FI(0, 0) * J0I(0, 0) + FI(1, 0) * J0I(0, 1) + FI(2, 0) * J0I(0, 2)) + Ny(0, 8)* (FI(0, 0) * J0I(1, 0) + FI(1, 0) * J0I(1, 1) + FI(2, 0) * J0I(1, 2)) + Nz(0, 8)* (FI(0, 0) * J0I(2, 0) + FI(1, 0) * J0I(2, 1) + FI(2, 0) * J0I(2, 2));
			strainD(2, 27) = Nx(0, 9)* (FI(0, 1) * J0I(0, 0) + FI(1, 1) * J0I(0, 1) + FI(2, 1) * J0I(0, 2)) + Ny(0, 9)* (FI(0, 1) * J0I(1, 0) + FI(1, 1) * J0I(1, 1) + FI(2, 1) * J0I(1, 2)) + Nz(0, 9)* (FI(0, 1) * J0I(2, 0) + FI(1, 1) * J0I(2, 1) + FI(2, 1) * J0I(2, 2));
			strainD(2, 28) = Nx(0, 9)* (FI(0, 0) * J0I(0, 0) + FI(1, 0) * J0I(0, 1) + FI(2, 0) * J0I(0, 2)) + Ny(0, 9)* (FI(0, 0) * J0I(1, 0) + FI(1, 0) * J0I(1, 1) + FI(2, 0) * J0I(1, 2)) + Nz(0, 9)* (FI(0, 0) * J0I(2, 0) + FI(1, 0) * J0I(2, 1) + FI(2, 0) * J0I(2, 2));
			strainD(2, 30) = Nx(0, 10)* (FI(0, 1) * J0I(0, 0) + FI(1, 1) * J0I(0, 1) + FI(2, 1) * J0I(0, 2)) + Ny(0, 10)* (FI(0, 1) * J0I(1, 0) + FI(1, 1) * J0I(1, 1) + FI(2, 1) * J0I(1, 2)) + Nz(0, 10)* (FI(0, 1) * J0I(2, 0) + FI(1, 1) * J0I(2, 1) + FI(2, 1) * J0I(2, 2));
			strainD(2, 31) = Nx(0, 10)* (FI(0, 0) * J0I(0, 0) + FI(1, 0) * J0I(0, 1) + FI(2, 0) * J0I(0, 2)) + Ny(0, 10)* (FI(0, 0) * J0I(1, 0) + FI(1, 0) * J0I(1, 1) + FI(2, 0) * J0I(1, 2)) + Nz(0, 10)* (FI(0, 0) * J0I(2, 0) + FI(1, 0) * J0I(2, 1) + FI(2, 0) * J0I(2, 2));
			strainD(3, 2) = Nx(0, 0)* (FI(0, 2) * J0I(0, 0) + FI(1, 2) * J0I(0, 1) + FI(2, 2) * J0I(0, 2)) + Ny(0, 0)* (FI(0, 2) * J0I(1, 0) + FI(1, 2) * J0I(1, 1) + FI(2, 2) * J0I(1, 2)) + Nz(0, 0)* (FI(0, 2) * J0I(2, 0) + FI(1, 2) * J0I(2, 1) + FI(2, 2) * J0I(2, 2));
			strainD(3, 5) = Nx(0, 1)* (FI(0, 2) * J0I(0, 0) + FI(1, 2) * J0I(0, 1) + FI(2, 2) * J0I(0, 2)) + Ny(0, 1)* (FI(0, 2) * J0I(1, 0) + FI(1, 2) * J0I(1, 1) + FI(2, 2) * J0I(1, 2)) + Nz(0, 1)* (FI(0, 2) * J0I(2, 0) + FI(1, 2) * J0I(2, 1) + FI(2, 2) * J0I(2, 2));
			strainD(3, 8) = Nx(0, 2)* (FI(0, 2) * J0I(0, 0) + FI(1, 2) * J0I(0, 1) + FI(2, 2) * J0I(0, 2)) + Ny(0, 2)* (FI(0, 2) * J0I(1, 0) + FI(1, 2) * J0I(1, 1) + FI(2, 2) * J0I(1, 2)) + Nz(0, 2)* (FI(0, 2) * J0I(2, 0) + FI(1, 2) * J0I(2, 1) + FI(2, 2) * J0I(2, 2));
			strainD(3, 11) = Nx(0, 3)* (FI(0, 2) * J0I(0, 0) + FI(1, 2) * J0I(0, 1) + FI(2, 2) * J0I(0, 2)) + Ny(0, 3)* (FI(0, 2) * J0I(1, 0) + FI(1, 2) * J0I(1, 1) + FI(2, 2) * J0I(1, 2)) + Nz(0, 3)* (FI(0, 2) * J0I(2, 0) + FI(1, 2) * J0I(2, 1) + FI(2, 2) * J0I(2, 2));
			strainD(3, 14) = Nx(0, 4)* (FI(0, 2) * J0I(0, 0) + FI(1, 2) * J0I(0, 1) + FI(2, 2) * J0I(0, 2)) + Ny(0, 4)* (FI(0, 2) * J0I(1, 0) + FI(1, 2) * J0I(1, 1) + FI(2, 2) * J0I(1, 2)) + Nz(0, 4)* (FI(0, 2) * J0I(2, 0) + FI(1, 2) * J0I(2, 1) + FI(2, 2) * J0I(2, 2));
			strainD(3, 17) = Nx(0, 5)* (FI(0, 2) * J0I(0, 0) + FI(1, 2) * J0I(0, 1) + FI(2, 2) * J0I(0, 2)) + Ny(0, 5)* (FI(0, 2) * J0I(1, 0) + FI(1, 2) * J0I(1, 1) + FI(2, 2) * J0I(1, 2)) + Nz(0, 5)* (FI(0, 2) * J0I(2, 0) + FI(1, 2) * J0I(2, 1) + FI(2, 2) * J0I(2, 2));
			strainD(3, 20) = Nx(0, 6)* (FI(0, 2) * J0I(0, 0) + FI(1, 2) * J0I(0, 1) + FI(2, 2) * J0I(0, 2)) + Ny(0, 6)* (FI(0, 2) * J0I(1, 0) + FI(1, 2) * J0I(1, 1) + FI(2, 2) * J0I(1, 2)) + Nz(0, 6)* (FI(0, 2) * J0I(2, 0) + FI(1, 2) * J0I(2, 1) + FI(2, 2) * J0I(2, 2));
			strainD(3, 23) = Nx(0, 7)* (FI(0, 2) * J0I(0, 0) + FI(1, 2) * J0I(0, 1) + FI(2, 2) * J0I(0, 2)) + Ny(0, 7)* (FI(0, 2) * J0I(1, 0) + FI(1, 2) * J0I(1, 1) + FI(2, 2) * J0I(1, 2)) + Nz(0, 7)* (FI(0, 2) * J0I(2, 0) + FI(1, 2) * J0I(2, 1) + FI(2, 2) * J0I(2, 2));
			strainD(3, 26) = Nx(0, 8)* (FI(0, 2) * J0I(0, 0) + FI(1, 2) * J0I(0, 1) + FI(2, 2) * J0I(0, 2)) + Ny(0, 8)* (FI(0, 2) * J0I(1, 0) + FI(1, 2) * J0I(1, 1) + FI(2, 2) * J0I(1, 2)) + Nz(0, 8)* (FI(0, 2) * J0I(2, 0) + FI(1, 2) * J0I(2, 1) + FI(2, 2) * J0I(2, 2));
			strainD(3, 29) = Nx(0, 9)* (FI(0, 2) * J0I(0, 0) + FI(1, 2) * J0I(0, 1) + FI(2, 2) * J0I(0, 2)) + Ny(0, 9)* (FI(0, 2) * J0I(1, 0) + FI(1, 2) * J0I(1, 1) + FI(2, 2) * J0I(1, 2)) + Nz(0, 9)* (FI(0, 2) * J0I(2, 0) + FI(1, 2) * J0I(2, 1) + FI(2, 2) * J0I(2, 2));
			strainD(3, 32) = Nx(0, 10)* (FI(0, 2) * J0I(0, 0) + FI(1, 2) * J0I(0, 1) + FI(2, 2) * J0I(0, 2)) + Ny(0, 10)* (FI(0, 2) * J0I(1, 0) + FI(1, 2) * J0I(1, 1) + FI(2, 2) * J0I(1, 2)) + Nz(0, 10)* (FI(0, 2) * J0I(2, 0) + FI(1, 2) * J0I(2, 1) + FI(2, 2) * J0I(2, 2));
			strainD(4, 0) = Nx(0, 0)* (FI(0, 2) * J0I(0, 0) + FI(1, 2) * J0I(0, 1) + FI(2, 2) * J0I(0, 2)) + Ny(0, 0)* (FI(0, 2) * J0I(1, 0) + FI(1, 2) * J0I(1, 1) + FI(2, 2) * J0I(1, 2)) + Nz(0, 0)* (FI(0, 2) * J0I(2, 0) + FI(1, 2) * J0I(2, 1) + FI(2, 2) * J0I(2, 2));
			strainD(4, 2) = Nx(0, 0)* (FI(0, 0) * J0I(0, 0) + FI(1, 0) * J0I(0, 1) + FI(2, 0) * J0I(0, 2)) + Ny(0, 0)* (FI(0, 0) * J0I(1, 0) + FI(1, 0) * J0I(1, 1) + FI(2, 0) * J0I(1, 2)) + Nz(0, 0)* (FI(0, 0) * J0I(2, 0) + FI(1, 0) * J0I(2, 1) + FI(2, 0) * J0I(2, 2));
			strainD(4, 3) = Nx(0, 1)* (FI(0, 2) * J0I(0, 0) + FI(1, 2) * J0I(0, 1) + FI(2, 2) * J0I(0, 2)) + Ny(0, 1)* (FI(0, 2) * J0I(1, 0) + FI(1, 2) * J0I(1, 1) + FI(2, 2) * J0I(1, 2)) + Nz(0, 1)* (FI(0, 2) * J0I(2, 0) + FI(1, 2) * J0I(2, 1) + FI(2, 2) * J0I(2, 2));
			strainD(4, 5) = Nx(0, 1)* (FI(0, 0) * J0I(0, 0) + FI(1, 0) * J0I(0, 1) + FI(2, 0) * J0I(0, 2)) + Ny(0, 1)* (FI(0, 0) * J0I(1, 0) + FI(1, 0) * J0I(1, 1) + FI(2, 0) * J0I(1, 2)) + Nz(0, 1)* (FI(0, 0) * J0I(2, 0) + FI(1, 0) * J0I(2, 1) + FI(2, 0) * J0I(2, 2));
			strainD(4, 6) = Nx(0, 2)* (FI(0, 2) * J0I(0, 0) + FI(1, 2) * J0I(0, 1) + FI(2, 2) * J0I(0, 2)) + Ny(0, 2)* (FI(0, 2) * J0I(1, 0) + FI(1, 2) * J0I(1, 1) + FI(2, 2) * J0I(1, 2)) + Nz(0, 2)* (FI(0, 2) * J0I(2, 0) + FI(1, 2) * J0I(2, 1) + FI(2, 2) * J0I(2, 2));
			strainD(4, 8) = Nx(0, 2)* (FI(0, 0) * J0I(0, 0) + FI(1, 0) * J0I(0, 1) + FI(2, 0) * J0I(0, 2)) + Ny(0, 2)* (FI(0, 0) * J0I(1, 0) + FI(1, 0) * J0I(1, 1) + FI(2, 0) * J0I(1, 2)) + Nz(0, 2)* (FI(0, 0) * J0I(2, 0) + FI(1, 0) * J0I(2, 1) + FI(2, 0) * J0I(2, 2));
			strainD(4, 9) = Nx(0, 3)* (FI(0, 2) * J0I(0, 0) + FI(1, 2) * J0I(0, 1) + FI(2, 2) * J0I(0, 2)) + Ny(0, 3)* (FI(0, 2) * J0I(1, 0) + FI(1, 2) * J0I(1, 1) + FI(2, 2) * J0I(1, 2)) + Nz(0, 3)* (FI(0, 2) * J0I(2, 0) + FI(1, 2) * J0I(2, 1) + FI(2, 2) * J0I(2, 2));
			strainD(4, 11) = Nx(0, 3)* (FI(0, 0) * J0I(0, 0) + FI(1, 0) * J0I(0, 1) + FI(2, 0) * J0I(0, 2)) + Ny(0, 3)* (FI(0, 0) * J0I(1, 0) + FI(1, 0) * J0I(1, 1) + FI(2, 0) * J0I(1, 2)) + Nz(0, 3)* (FI(0, 0) * J0I(2, 0) + FI(1, 0) * J0I(2, 1) + FI(2, 0) * J0I(2, 2));
			strainD(4, 12) = Nx(0, 4)* (FI(0, 2) * J0I(0, 0) + FI(1, 2) * J0I(0, 1) + FI(2, 2) * J0I(0, 2)) + Ny(0, 4)* (FI(0, 2) * J0I(1, 0) + FI(1, 2) * J0I(1, 1) + FI(2, 2) * J0I(1, 2)) + Nz(0, 4)* (FI(0, 2) * J0I(2, 0) + FI(1, 2) * J0I(2, 1) + FI(2, 2) * J0I(2, 2));
			strainD(4, 14) = Nx(0, 4)* (FI(0, 0) * J0I(0, 0) + FI(1, 0) * J0I(0, 1) + FI(2, 0) * J0I(0, 2)) + Ny(0, 4)* (FI(0, 0) * J0I(1, 0) + FI(1, 0) * J0I(1, 1) + FI(2, 0) * J0I(1, 2)) + Nz(0, 4)* (FI(0, 0) * J0I(2, 0) + FI(1, 0) * J0I(2, 1) + FI(2, 0) * J0I(2, 2));
			strainD(4, 15) = Nx(0, 5)* (FI(0, 2) * J0I(0, 0) + FI(1, 2) * J0I(0, 1) + FI(2, 2) * J0I(0, 2)) + Ny(0, 5)* (FI(0, 2) * J0I(1, 0) + FI(1, 2) * J0I(1, 1) + FI(2, 2) * J0I(1, 2)) + Nz(0, 5)* (FI(0, 2) * J0I(2, 0) + FI(1, 2) * J0I(2, 1) + FI(2, 2) * J0I(2, 2));
			strainD(4, 17) = Nx(0, 5)* (FI(0, 0) * J0I(0, 0) + FI(1, 0) * J0I(0, 1) + FI(2, 0) * J0I(0, 2)) + Ny(0, 5)* (FI(0, 0) * J0I(1, 0) + FI(1, 0) * J0I(1, 1) + FI(2, 0) * J0I(1, 2)) + Nz(0, 5)* (FI(0, 0) * J0I(2, 0) + FI(1, 0) * J0I(2, 1) + FI(2, 0) * J0I(2, 2));
			strainD(4, 18) = Nx(0, 6)* (FI(0, 2) * J0I(0, 0) + FI(1, 2) * J0I(0, 1) + FI(2, 2) * J0I(0, 2)) + Ny(0, 6)* (FI(0, 2) * J0I(1, 0) + FI(1, 2) * J0I(1, 1) + FI(2, 2) * J0I(1, 2)) + Nz(0, 6)* (FI(0, 2) * J0I(2, 0) + FI(1, 2) * J0I(2, 1) + FI(2, 2) * J0I(2, 2));
			strainD(4, 20) = Nx(0, 6)* (FI(0, 0) * J0I(0, 0) + FI(1, 0) * J0I(0, 1) + FI(2, 0) * J0I(0, 2)) + Ny(0, 6)* (FI(0, 0) * J0I(1, 0) + FI(1, 0) * J0I(1, 1) + FI(2, 0) * J0I(1, 2)) + Nz(0, 6)* (FI(0, 0) * J0I(2, 0) + FI(1, 0) * J0I(2, 1) + FI(2, 0) * J0I(2, 2));
			strainD(4, 21) = Nx(0, 7)* (FI(0, 2) * J0I(0, 0) + FI(1, 2) * J0I(0, 1) + FI(2, 2) * J0I(0, 2)) + Ny(0, 7)* (FI(0, 2) * J0I(1, 0) + FI(1, 2) * J0I(1, 1) + FI(2, 2) * J0I(1, 2)) + Nz(0, 7)* (FI(0, 2) * J0I(2, 0) + FI(1, 2) * J0I(2, 1) + FI(2, 2) * J0I(2, 2));
			strainD(4, 23) = Nx(0, 7)* (FI(0, 0) * J0I(0, 0) + FI(1, 0) * J0I(0, 1) + FI(2, 0) * J0I(0, 2)) + Ny(0, 7)* (FI(0, 0) * J0I(1, 0) + FI(1, 0) * J0I(1, 1) + FI(2, 0) * J0I(1, 2)) + Nz(0, 7)* (FI(0, 0) * J0I(2, 0) + FI(1, 0) * J0I(2, 1) + FI(2, 0) * J0I(2, 2));
			strainD(4, 24) = Nx(0, 8)* (FI(0, 2) * J0I(0, 0) + FI(1, 2) * J0I(0, 1) + FI(2, 2) * J0I(0, 2)) + Ny(0, 8)* (FI(0, 2) * J0I(1, 0) + FI(1, 2) * J0I(1, 1) + FI(2, 2) * J0I(1, 2)) + Nz(0, 8)* (FI(0, 2) * J0I(2, 0) + FI(1, 2) * J0I(2, 1) + FI(2, 2) * J0I(2, 2));
			strainD(4, 26) = Nx(0, 8)* (FI(0, 0) * J0I(0, 0) + FI(1, 0) * J0I(0, 1) + FI(2, 0) * J0I(0, 2)) + Ny(0, 8)* (FI(0, 0) * J0I(1, 0) + FI(1, 0) * J0I(1, 1) + FI(2, 0) * J0I(1, 2)) + Nz(0, 8)* (FI(0, 0) * J0I(2, 0) + FI(1, 0) * J0I(2, 1) + FI(2, 0) * J0I(2, 2));
			strainD(4, 27) = Nx(0, 9)* (FI(0, 2) * J0I(0, 0) + FI(1, 2) * J0I(0, 1) + FI(2, 2) * J0I(0, 2)) + Ny(0, 9)* (FI(0, 2) * J0I(1, 0) + FI(1, 2) * J0I(1, 1) + FI(2, 2) * J0I(1, 2)) + Nz(0, 9)* (FI(0, 2) * J0I(2, 0) + FI(1, 2) * J0I(2, 1) + FI(2, 2) * J0I(2, 2));
			strainD(4, 29) = Nx(0, 9)* (FI(0, 0) * J0I(0, 0) + FI(1, 0) * J0I(0, 1) + FI(2, 0) * J0I(0, 2)) + Ny(0, 9)* (FI(0, 0) * J0I(1, 0) + FI(1, 0) * J0I(1, 1) + FI(2, 0) * J0I(1, 2)) + Nz(0, 9)* (FI(0, 0) * J0I(2, 0) + FI(1, 0) * J0I(2, 1) + FI(2, 0) * J0I(2, 2));
			strainD(4, 30) = Nx(0, 10)* (FI(0, 2) * J0I(0, 0) + FI(1, 2) * J0I(0, 1) + FI(2, 2) * J0I(0, 2)) + Ny(0, 10)* (FI(0, 2) * J0I(1, 0) + FI(1, 2) * J0I(1, 1) + FI(2, 2) * J0I(1, 2)) + Nz(0, 10)* (FI(0, 2) * J0I(2, 0) + FI(1, 2) * J0I(2, 1) + FI(2, 2) * J0I(2, 2));
			strainD(4, 32) = Nx(0, 10)* (FI(0, 0) * J0I(0, 0) + FI(1, 0) * J0I(0, 1) + FI(2, 0) * J0I(0, 2)) + Ny(0, 10)* (FI(0, 0) * J0I(1, 0) + FI(1, 0) * J0I(1, 1) + FI(2, 0) * J0I(1, 2)) + Nz(0, 10)* (FI(0, 0) * J0I(2, 0) + FI(1, 0) * J0I(2, 1) + FI(2, 0) * J0I(2, 2));
			strainD(5, 1) = Nx(0, 0)* (FI(0, 2) * J0I(0, 0) + FI(1, 2) * J0I(0, 1) + FI(2, 2) * J0I(0, 2)) + Ny(0, 0)* (FI(0, 2) * J0I(1, 0) + FI(1, 2) * J0I(1, 1) + FI(2, 2) * J0I(1, 2)) + Nz(0, 0)* (FI(0, 2) * J0I(2, 0) + FI(1, 2) * J0I(2, 1) + FI(2, 2) * J0I(2, 2));
			strainD(5, 2) = Nx(0, 0)* (FI(0, 1) * J0I(0, 0) + FI(1, 1) * J0I(0, 1) + FI(2, 1) * J0I(0, 2)) + Ny(0, 0)* (FI(0, 1) * J0I(1, 0) + FI(1, 1) * J0I(1, 1) + FI(2, 1) * J0I(1, 2)) + Nz(0, 0)* (FI(0, 1) * J0I(2, 0) + FI(1, 1) * J0I(2, 1) + FI(2, 1) * J0I(2, 2));
			strainD(5, 4) = Nx(0, 1)* (FI(0, 2) * J0I(0, 0) + FI(1, 2) * J0I(0, 1) + FI(2, 2) * J0I(0, 2)) + Ny(0, 1)* (FI(0, 2) * J0I(1, 0) + FI(1, 2) * J0I(1, 1) + FI(2, 2) * J0I(1, 2)) + Nz(0, 1)* (FI(0, 2) * J0I(2, 0) + FI(1, 2) * J0I(2, 1) + FI(2, 2) * J0I(2, 2));
			strainD(5, 5) = Nx(0, 1)* (FI(0, 1) * J0I(0, 0) + FI(1, 1) * J0I(0, 1) + FI(2, 1) * J0I(0, 2)) + Ny(0, 1)* (FI(0, 1) * J0I(1, 0) + FI(1, 1) * J0I(1, 1) + FI(2, 1) * J0I(1, 2)) + Nz(0, 1)* (FI(0, 1) * J0I(2, 0) + FI(1, 1) * J0I(2, 1) + FI(2, 1) * J0I(2, 2));
			strainD(5, 7) = Nx(0, 2)* (FI(0, 2) * J0I(0, 0) + FI(1, 2) * J0I(0, 1) + FI(2, 2) * J0I(0, 2)) + Ny(0, 2)* (FI(0, 2) * J0I(1, 0) + FI(1, 2) * J0I(1, 1) + FI(2, 2) * J0I(1, 2)) + Nz(0, 2)* (FI(0, 2) * J0I(2, 0) + FI(1, 2) * J0I(2, 1) + FI(2, 2) * J0I(2, 2));
			strainD(5, 8) = Nx(0, 2)* (FI(0, 1) * J0I(0, 0) + FI(1, 1) * J0I(0, 1) + FI(2, 1) * J0I(0, 2)) + Ny(0, 2)* (FI(0, 1) * J0I(1, 0) + FI(1, 1) * J0I(1, 1) + FI(2, 1) * J0I(1, 2)) + Nz(0, 2)* (FI(0, 1) * J0I(2, 0) + FI(1, 1) * J0I(2, 1) + FI(2, 1) * J0I(2, 2));
			strainD(5, 10) = Nx(0, 3)* (FI(0, 2) * J0I(0, 0) + FI(1, 2) * J0I(0, 1) + FI(2, 2) * J0I(0, 2)) + Ny(0, 3)* (FI(0, 2) * J0I(1, 0) + FI(1, 2) * J0I(1, 1) + FI(2, 2) * J0I(1, 2)) + Nz(0, 3)* (FI(0, 2) * J0I(2, 0) + FI(1, 2) * J0I(2, 1) + FI(2, 2) * J0I(2, 2));
			strainD(5, 11) = Nx(0, 3)* (FI(0, 1) * J0I(0, 0) + FI(1, 1) * J0I(0, 1) + FI(2, 1) * J0I(0, 2)) + Ny(0, 3)* (FI(0, 1) * J0I(1, 0) + FI(1, 1) * J0I(1, 1) + FI(2, 1) * J0I(1, 2)) + Nz(0, 3)* (FI(0, 1) * J0I(2, 0) + FI(1, 1) * J0I(2, 1) + FI(2, 1) * J0I(2, 2));
			strainD(5, 13) = Nx(0, 4)* (FI(0, 2) * J0I(0, 0) + FI(1, 2) * J0I(0, 1) + FI(2, 2) * J0I(0, 2)) + Ny(0, 4)* (FI(0, 2) * J0I(1, 0) + FI(1, 2) * J0I(1, 1) + FI(2, 2) * J0I(1, 2)) + Nz(0, 4)* (FI(0, 2) * J0I(2, 0) + FI(1, 2) * J0I(2, 1) + FI(2, 2) * J0I(2, 2));
			strainD(5, 14) = Nx(0, 4)* (FI(0, 1) * J0I(0, 0) + FI(1, 1) * J0I(0, 1) + FI(2, 1) * J0I(0, 2)) + Ny(0, 4)* (FI(0, 1) * J0I(1, 0) + FI(1, 1) * J0I(1, 1) + FI(2, 1) * J0I(1, 2)) + Nz(0, 4)* (FI(0, 1) * J0I(2, 0) + FI(1, 1) * J0I(2, 1) + FI(2, 1) * J0I(2, 2));
			strainD(5, 16) = Nx(0, 5)* (FI(0, 2) * J0I(0, 0) + FI(1, 2) * J0I(0, 1) + FI(2, 2) * J0I(0, 2)) + Ny(0, 5)* (FI(0, 2) * J0I(1, 0) + FI(1, 2) * J0I(1, 1) + FI(2, 2) * J0I(1, 2)) + Nz(0, 5)* (FI(0, 2) * J0I(2, 0) + FI(1, 2) * J0I(2, 1) + FI(2, 2) * J0I(2, 2));
			strainD(5, 17) = Nx(0, 5)* (FI(0, 1) * J0I(0, 0) + FI(1, 1) * J0I(0, 1) + FI(2, 1) * J0I(0, 2)) + Ny(0, 5)* (FI(0, 1) * J0I(1, 0) + FI(1, 1) * J0I(1, 1) + FI(2, 1) * J0I(1, 2)) + Nz(0, 5)* (FI(0, 1) * J0I(2, 0) + FI(1, 1) * J0I(2, 1) + FI(2, 1) * J0I(2, 2));
			strainD(5, 19) = Nx(0, 6)* (FI(0, 2) * J0I(0, 0) + FI(1, 2) * J0I(0, 1) + FI(2, 2) * J0I(0, 2)) + Ny(0, 6)* (FI(0, 2) * J0I(1, 0) + FI(1, 2) * J0I(1, 1) + FI(2, 2) * J0I(1, 2)) + Nz(0, 6)* (FI(0, 2) * J0I(2, 0) + FI(1, 2) * J0I(2, 1) + FI(2, 2) * J0I(2, 2));
			strainD(5, 20) = Nx(0, 6)* (FI(0, 1) * J0I(0, 0) + FI(1, 1) * J0I(0, 1) + FI(2, 1) * J0I(0, 2)) + Ny(0, 6)* (FI(0, 1) * J0I(1, 0) + FI(1, 1) * J0I(1, 1) + FI(2, 1) * J0I(1, 2)) + Nz(0, 6)* (FI(0, 1) * J0I(2, 0) + FI(1, 1) * J0I(2, 1) + FI(2, 1) * J0I(2, 2));
			strainD(5, 22) = Nx(0, 7)* (FI(0, 2) * J0I(0, 0) + FI(1, 2) * J0I(0, 1) + FI(2, 2) * J0I(0, 2)) + Ny(0, 7)* (FI(0, 2) * J0I(1, 0) + FI(1, 2) * J0I(1, 1) + FI(2, 2) * J0I(1, 2)) + Nz(0, 7)* (FI(0, 2) * J0I(2, 0) + FI(1, 2) * J0I(2, 1) + FI(2, 2) * J0I(2, 2));
			strainD(5, 23) = Nx(0, 7)* (FI(0, 1) * J0I(0, 0) + FI(1, 1) * J0I(0, 1) + FI(2, 1) * J0I(0, 2)) + Ny(0, 7)* (FI(0, 1) * J0I(1, 0) + FI(1, 1) * J0I(1, 1) + FI(2, 1) * J0I(1, 2)) + Nz(0, 7)* (FI(0, 1) * J0I(2, 0) + FI(1, 1) * J0I(2, 1) + FI(2, 1) * J0I(2, 2));
			strainD(5, 25) = Nx(0, 8)* (FI(0, 2) * J0I(0, 0) + FI(1, 2) * J0I(0, 1) + FI(2, 2) * J0I(0, 2)) + Ny(0, 8)* (FI(0, 2) * J0I(1, 0) + FI(1, 2) * J0I(1, 1) + FI(2, 2) * J0I(1, 2)) + Nz(0, 8)* (FI(0, 2) * J0I(2, 0) + FI(1, 2) * J0I(2, 1) + FI(2, 2) * J0I(2, 2));
			strainD(5, 26) = Nx(0, 8)* (FI(0, 1) * J0I(0, 0) + FI(1, 1) * J0I(0, 1) + FI(2, 1) * J0I(0, 2)) + Ny(0, 8)* (FI(0, 1) * J0I(1, 0) + FI(1, 1) * J0I(1, 1) + FI(2, 1) * J0I(1, 2)) + Nz(0, 8)* (FI(0, 1) * J0I(2, 0) + FI(1, 1) * J0I(2, 1) + FI(2, 1) * J0I(2, 2));
			strainD(5, 28) = Nx(0, 9)* (FI(0, 2) * J0I(0, 0) + FI(1, 2) * J0I(0, 1) + FI(2, 2) * J0I(0, 2)) + Ny(0, 9)* (FI(0, 2) * J0I(1, 0) + FI(1, 2) * J0I(1, 1) + FI(2, 2) * J0I(1, 2)) + Nz(0, 9)* (FI(0, 2) * J0I(2, 0) + FI(1, 2) * J0I(2, 1) + FI(2, 2) * J0I(2, 2));
			strainD(5, 29) = Nx(0, 9)* (FI(0, 1) * J0I(0, 0) + FI(1, 1) * J0I(0, 1) + FI(2, 1) * J0I(0, 2)) + Ny(0, 9)* (FI(0, 1) * J0I(1, 0) + FI(1, 1) * J0I(1, 1) + FI(2, 1) * J0I(1, 2)) + Nz(0, 9)* (FI(0, 1) * J0I(2, 0) + FI(1, 1) * J0I(2, 1) + FI(2, 1) * J0I(2, 2));
			strainD(5, 31) = Nx(0, 10)* (FI(0, 2) * J0I(0, 0) + FI(1, 2) * J0I(0, 1) + FI(2, 2) * J0I(0, 2)) + Ny(0, 10)* (FI(0, 2) * J0I(1, 0) + FI(1, 2) * J0I(1, 1) + FI(2, 2) * J0I(1, 2)) + Nz(0, 10)* (FI(0, 2) * J0I(2, 0) + FI(1, 2) * J0I(2, 1) + FI(2, 2) * J0I(2, 2));
			strainD(5, 32) = Nx(0, 10)* (FI(0, 1) * J0I(0, 0) + FI(1, 1) * J0I(0, 1) + FI(2, 1) * J0I(0, 2)) + Ny(0, 10)* (FI(0, 1) * J0I(1, 0) + FI(1, 1) * J0I(1, 1) + FI(2, 1) * J0I(1, 2)) + Nz(0, 10)* (FI(0, 1) * J0I(2, 0) + FI(1, 1) * J0I(2, 1) + FI(2, 1) * J0I(2, 2));
		}

		double ChElementBrick_9::Calc_detJ0(double x,
			double y,
			double z,
			ChMatrixNM<double, 1, 11>& Nx,
			ChMatrixNM<double, 1, 11>& Ny,
			ChMatrixNM<double, 1, 11>& Nz,
			ChMatrixNM<double, 1, 3>& Nx_d0,
			ChMatrixNM<double, 1, 3>& Ny_d0,
			ChMatrixNM<double, 1, 3>& Nz_d0) {
			ShapeFunctionsDerivativeX(Nx, x, y, z);
			ShapeFunctionsDerivativeY(Ny, x, y, z);
			ShapeFunctionsDerivativeZ(Nz, x, y, z);

			Nx_d0 = Nx * m_d0;
			Ny_d0 = Ny * m_d0;
			Nz_d0 = Nz * m_d0;

			double detJ0 = Nx_d0(0, 0) * Ny_d0(0, 1) * Nz_d0(0, 2) + Ny_d0(0, 0) * Nz_d0(0, 1) * Nx_d0(0, 2) +
				Nz_d0(0, 0) * Nx_d0(0, 1) * Ny_d0(0, 2) - Nx_d0(0, 2) * Ny_d0(0, 1) * Nz_d0(0, 0) -
				Ny_d0(0, 2) * Nz_d0(0, 1) * Nx_d0(0, 0) - Nz_d0(0, 2) * Nx_d0(0, 1) * Ny_d0(0, 0);

			return detJ0;
		}

		double ChElementBrick_9::Calc_detJ0(double x, double y, double z) {
			ChMatrixNM<double, 1, 11> Nx;
			ChMatrixNM<double, 1, 11> Ny;
			ChMatrixNM<double, 1, 11> Nz;
			ChMatrixNM<double, 1, 3> Nx_d0;
			ChMatrixNM<double, 1, 3> Ny_d0;
			ChMatrixNM<double, 1, 3> Nz_d0;

			return Calc_detJ0(x, y, z, Nx, Ny, Nz, Nx_d0, Ny_d0, Nz_d0);
		}

		void ChElementBrick_9::CalcCoordMatrix(ChMatrixNM<double, 11, 3>& d) {
			for (int i = 0; i < 8; i++) {
				const ChVector<>& pos = m_nodes[i]->GetPos();
				d(i, 0) = pos.x;
				d(i, 1) = pos.y;
				d(i, 2) = pos.z;
			}

			const ChVector<>& rxx = m_central_node->GetCurvatureXX();
			const ChVector<>& ryy = m_central_node->GetCurvatureYY();
			const ChVector<>& rzz = m_central_node->GetCurvatureZZ();

			d(8, 0) = rxx.x;
			d(8, 1) = rxx.y;
			d(8, 2) = rxx.z;

			d(9, 0) = ryy.x;
			d(9, 1) = ryy.y;
			d(9, 2) = ryy.z;

			d(10, 0) = rzz.x;
			d(10, 1) = rzz.y;
			d(10, 2) = rzz.z;
		}

		void ChElementBrick_9::CalcCoordDerivMatrix(ChMatrixNM<double, 33, 1>& dt) {
			for (int i = 0; i < 8; i++) {
				const ChVector<>& vel = m_nodes[i]->GetPos_dt();
				dt(3 * i, 0) = vel.x;
				dt(3 * i + 1, 0) = vel.y;
				dt(3 * i + 2, 0) = vel.z;
			}

			dt(24, 0) = m_central_node->GetCurvatureXX_dt().x;
			dt(25, 0) = m_central_node->GetCurvatureXX_dt().y;
			dt(26, 0) = m_central_node->GetCurvatureXX_dt().z;
			dt(27, 0) = m_central_node->GetCurvatureYY_dt().x;
			dt(28, 0) = m_central_node->GetCurvatureYY_dt().y;
			dt(29, 0) = m_central_node->GetCurvatureYY_dt().z;
			dt(30, 0) = m_central_node->GetCurvatureZZ_dt().x;
			dt(31, 0) = m_central_node->GetCurvatureZZ_dt().y;
			dt(32, 0) = m_central_node->GetCurvatureZZ_dt().z;

		}

	}  // end namespace fea
}  // end namespace chrono
