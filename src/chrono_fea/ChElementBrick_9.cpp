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
			ChMatrixDynamic<> Fi;
			ComputeGravityForce(system->Get_G_acc());
			ComputeMassMatrix();

		}

		// -----------------------------------------------------------------------------
		// Calculation of shape functions and their derivatives
		// -----------------------------------------------------------------------------

		void ChElementBrick_9::ShapeFunctions(ChMatrix<>& N, double x, double y, double z) {
			double a = GetDimensions().x;
			double b = GetDimensions().y;
			double c = GetDimensions().z;
			N(0) = x*(-1.0 / 8.0) - y*(1.0 / 8.0) - z*(1.0 / 8.0) + x*y*(1.0 / 8.0) + x*z*(1.0 / 8.0) + y*z*(1.0 / 8.0) - x*y*z*(1.0 / 8.0) + 1.0 / 8.0;
			N(1) = x*(1.0 / 8.0) - y*(1.0 / 8.0) - z*(1.0 / 8.0) - x*y*(1.0 / 8.0) - x*z*(1.0 / 8.0) + y*z*(1.0 / 8.0) + x*y*z*(1.0 / 8.0) + 1.0 / 8.0;
			N(2) = x*(1.0 / 8.0) + y*(1.0 / 8.0) - z*(1.0 / 8.0) + x*y*(1.0 / 8.0) - x*z*(1.0 / 8.0) - y*z*(1.0 / 8.0) - x*y*z*(1.0 / 8.0) + 1.0 / 8.0;
			N(3) = x*(-1.0 / 8.0) + y*(1.0 / 8.0) - z*(1.0 / 8.0) - x*y*(1.0 / 8.0) + x*z*(1.0 / 8.0) - y*z*(1.0 / 8.0) + x*y*z*(1.0 / 8.0) + 1.0 / 8.0;
			N(4) = x*(-1.0 / 8.0) - y*(1.0 / 8.0) + z*(1.0 / 8.0) + x*y*(1.0 / 8.0) - x*z*(1.0 / 8.0) - y*z*(1.0 / 8.0) + x*y*z*(1.0 / 8.0) + 1.0 / 8.0;
			N(5) = x*(1.0 / 8.0) - y*(1.0 / 8.0) + z*(1.0 / 8.0) - x*y*(1.0 / 8.0) + x*z*(1.0 / 8.0) - y*z*(1.0 / 8.0) - x*y*z*(1.0 / 8.0) + 1.0 / 8.0;
			N(6) = x*(1.0 / 8.0) + y*(1.0 / 8.0) + z*(1.0 / 8.0) + x*y*(1.0 / 8.0) + x*z*(1.0 / 8.0) + y*z*(1.0 / 8.0) + x*y*z*(1.0 / 8.0) + 1.0 / 8.0;
			N(7) = x*(-1.0 / 8.0) + y*(1.0 / 8.0) + z*(1.0 / 8.0) - x*y*(1.0 / 8.0) - x*z*(1.0 / 8.0) + y*z*(1.0 / 8.0) - x*y*z*(1.0 / 8.0) + 1.0 / 8.0;
			//
			N(8) = (a*a)*(-1.0 / 8.0) + (a*a)*(x*x)*(1.0 / 8.0);
			N(9) = (b*b)*(-1.0 / 8.0) + (b*b)*(y*y)*(1.0 / 8.0);
			N(10) = (c*c)*(-1.0 / 8.0) + (c*c)*(z*z)*(1.0 / 8.0);
		}

		void ChElementBrick_9::ShapeFunctionsDerivativeX(ChMatrix<>& Nx, double x, double y, double z) {
			double factor = 2 / GetDimensions().x;
			double a = GetDimensions().x;
			double b = GetDimensions().y;
			double c = GetDimensions().z;
			Nx(0) = (y*(1.0 / 4.0)) / a + (z*(1.0 / 4.0)) / a - (1.0 / 4.0) / a - (y*z*(1.0 / 4.0)) / a;
			Nx(1) = (y*(-1.0 / 4.0)) / a - (z*(1.0 / 4.0)) / a + (1.0 / 4.0) / a + (y*z*(1.0 / 4.0)) / a;
			Nx(2) = (y*(1.0 / 4.0)) / a - (z*(1.0 / 4.0)) / a + (1.0 / 4.0) / a - (y*z*(1.0 / 4.0)) / a;
			Nx(3) = (y*(-1.0 / 4.0)) / a + (z*(1.0 / 4.0)) / a - (1.0 / 4.0) / a + (y*z*(1.0 / 4.0)) / a;
			Nx(4) = (y*(1.0 / 4.0)) / a - (z*(1.0 / 4.0)) / a - (1.0 / 4.0) / a + (y*z*(1.0 / 4.0)) / a;
			Nx(5) = (y*(-1.0 / 4.0)) / a + (z*(1.0 / 4.0)) / a + (1.0 / 4.0) / a - (y*z*(1.0 / 4.0)) / a;
			Nx(6) = (y*(1.0 / 4.0)) / a + (z*(1.0 / 4.0)) / a + (1.0 / 4.0) / a + (y*z*(1.0 / 4.0)) / a;
			Nx(7) = (y*(-1.0 / 4.0)) / a - (z*(1.0 / 4.0)) / a - (1.0 / 4.0) / a - (y*z*(1.0 / 4.0)) / a;
			//
			Nx(8) = a*x*(1.0 / 2.0);
			Nx(9) = 0;
			Nx(10) = 0;
		}

		void ChElementBrick_9::ShapeFunctionsDerivativeY(ChMatrix<>& Ny, double x, double y, double z) {
			double factor = 2 / GetDimensions().y;
			double a = GetDimensions().x;
			double b = GetDimensions().y;
			double c = GetDimensions().z;
			Ny(0) = (x*(1.0 / 4.0)) / b + (z*(1.0 / 4.0)) / b - (1.0 / 4.0) / b - (x*z*(1.0 / 4.0)) / b;
			Ny(1) = (x*(-1.0 / 4.0)) / b + (z*(1.0 / 4.0)) / b - (1.0 / 4.0) / b + (x*z*(1.0 / 4.0)) / b;
			Ny(2) = (x*(1.0 / 4.0)) / b - (z*(1.0 / 4.0)) / b + (1.0 / 4.0) / b - (x*z*(1.0 / 4.0)) / b;
			Ny(3) = (x*(-1.0 / 4.0)) / b - (z*(1.0 / 4.0)) / b + (1.0 / 4.0) / b + (x*z*(1.0 / 4.0)) / b;
			Ny(4) = (x*(1.0 / 4.0)) / b - (z*(1.0 / 4.0)) / b - (1.0 / 4.0) / b + (x*z*(1.0 / 4.0)) / b;
			Ny(5) = (x*(-1.0 / 4.0)) / b - (z*(1.0 / 4.0)) / b - (1.0 / 4.0) / b - (x*z*(1.0 / 4.0)) / b;
			Ny(6) = (x*(1.0 / 4.0)) / b + (z*(1.0 / 4.0)) / b + (1.0 / 4.0) / b + (x*z*(1.0 / 4.0)) / b;
			Ny(7) = (x*(-1.0 / 4.0)) / b + (z*(1.0 / 4.0)) / b + (1.0 / 4.0) / b - (x*z*(1.0 / 4.0)) / b;
			//
			Ny(8) = 0;
			Ny(9) = b*y*(1.0 / 2.0);
			Ny(10) = 0;
		}

		void ChElementBrick_9::ShapeFunctionsDerivativeZ(ChMatrix<>& Nz, double x, double y, double z) {
			double factor = 2 / GetDimensions().z;
			double a = GetDimensions().x;
			double b = GetDimensions().y;
			double c = GetDimensions().z;
			Nz(0) = (x*(1.0 / 4.0)) / c + (y*(1.0 / 4.0)) / c - (1.0 / 4.0) / c - (x*y*(1.0 / 4.0)) / c;
			Nz(1) = (x*(-1.0 / 4.0)) / c + (y*(1.0 / 4.0)) / c - (1.0 / 4.0) / c + (x*y*(1.0 / 4.0)) / c;
			Nz(2) = (x*(-1.0 / 4.0)) / c - (y*(1.0 / 4.0)) / c - (1.0 / 4.0) / c - (x*y*(1.0 / 4.0)) / c;
			Nz(3) = (x*(1.0 / 4.0)) / c - (y*(1.0 / 4.0)) / c - (1.0 / 4.0) / c + (x*y*(1.0 / 4.0)) / c;
			Nz(4) = (x*(-1.0 / 4.0)) / c - (y*(1.0 / 4.0)) / c + (1.0 / 4.0) / c + (x*y*(1.0 / 4.0)) / c;
			Nz(5) = (x*(1.0 / 4.0)) / c - (y*(1.0 / 4.0)) / c + (1.0 / 4.0) / c - (x*y*(1.0 / 4.0)) / c;
			Nz(6) = (x*(1.0 / 4.0)) / c + (y*(1.0 / 4.0)) / c + (1.0 / 4.0) / c + (x*y*(1.0 / 4.0)) / c;
			Nz(7) = (x*(-1.0 / 4.0)) / c + (y*(1.0 / 4.0)) / c + (1.0 / 4.0) / c - (x*y*(1.0 / 4.0)) / c;
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
			j0(0, 0) = Ny_d0[0][1] * Nz_d0[0][2] - Nz_d0[0][1] * Ny_d0[0][2];
			j0(0, 1) = Ny_d0[0][2] * Nz_d0[0][0] - Ny_d0[0][0] * Nz_d0[0][2];
			j0(0, 2) = Ny_d0[0][0] * Nz_d0[0][1] - Nz_d0[0][0] * Ny_d0[0][1];
			j0(1, 0) = Nz_d0[0][1] * Nx_d0[0][2] - Nx_d0[0][1] * Nz_d0[0][2];
			j0(1, 1) = Nz_d0[0][2] * Nx_d0[0][0] - Nx_d0[0][2] * Nz_d0[0][0];
			j0(1, 2) = Nz_d0[0][0] * Nx_d0[0][1] - Nz_d0[0][1] * Nx_d0[0][0];
			j0(2, 0) = Nx_d0[0][1] * Ny_d0[0][2] - Ny_d0[0][1] * Nx_d0[0][2];
			j0(2, 1) = Ny_d0[0][0] * Nx_d0[0][2] - Nx_d0[0][0] * Ny_d0[0][2];
			j0(2, 2) = Nx_d0[0][0] * Ny_d0[0][1] - Ny_d0[0][0] * Nx_d0[0][1];
			j0.MatrDivScale(detJ0);

			ChMatrixNM<double, 3, 3> DefF;
			DefF(0, 0) = Nx_d(0, 0); DefF(1, 0) = Nx_d(0, 1); DefF(2, 0) = Nx_d(0, 2);
			DefF(0, 1) = Ny_d(0, 0); DefF(1, 1) = Ny_d(0, 1); DefF(2, 1) = Ny_d(0, 2);
			DefF(0, 2) = Nz_d(0, 0); DefF(1, 2) = Nz_d(0, 1); DefF(2, 2) = Nz_d(0, 2);

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

			// Internal force calculation
			ChMatrixNM<double, 33, 6> tempC;
			tempC.MatrTMultiply(strainD, E_eps);
			ChMatrixNM<double, 33, 1> Fint = (tempC * strain) * (detJ0 * m_element->m_GaussScaling);
			result = Fint;
		}

		// Compute internal forces and load them in the Fi vector.
		void ChElementBrick_9::ComputeInternalForces(ChMatrixDynamic<>& Fi) {
			CalcCoordMatrix(m_d);
			CalcCoordDerivMatrix(m_d_dt);
			m_ddT.MatrMultiplyT(m_d, m_d);


			Fi.Reset();

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
			j0(0, 0) = Ny_d0[0][1] * Nz_d0[0][2] - Nz_d0[0][1] * Ny_d0[0][2];
			j0(0, 1) = Ny_d0[0][2] * Nz_d0[0][0] - Ny_d0[0][0] * Nz_d0[0][2];
			j0(0, 2) = Ny_d0[0][0] * Nz_d0[0][1] - Nz_d0[0][0] * Ny_d0[0][1];
			j0(1, 0) = Nz_d0[0][1] * Nx_d0[0][2] - Nx_d0[0][1] * Nz_d0[0][2];
			j0(1, 1) = Nz_d0[0][2] * Nx_d0[0][0] - Nx_d0[0][2] * Nz_d0[0][0];
			j0(1, 2) = Nz_d0[0][0] * Nx_d0[0][1] - Nz_d0[0][1] * Nx_d0[0][0];
			j0(2, 0) = Nx_d0[0][1] * Ny_d0[0][2] - Ny_d0[0][1] * Nx_d0[0][2];
			j0(2, 1) = Ny_d0[0][0] * Nx_d0[0][2] - Nx_d0[0][0] * Ny_d0[0][2];
			j0(2, 2) = Nx_d0[0][0] * Ny_d0[0][1] - Ny_d0[0][0] * Nx_d0[0][1];
			j0.MatrDivScale(detJ0);

			ChMatrixNM<double, 3, 3> DefF;
			DefF(0, 0) = Nx_d(0, 0); DefF(1, 0) = Nx_d(0, 1); DefF(2, 0) = Nx_d(0, 2);
			DefF(0, 1) = Ny_d(0, 0); DefF(1, 1) = Ny_d(0, 1); DefF(2, 1) = Ny_d(0, 2);
			DefF(0, 2) = Nz_d(0, 0); DefF(1, 2) = Nz_d(0, 1); DefF(2, 2) = Nz_d(0, 2);

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
			ChMatrixNM<double, 6, 33> temp36;
			/*temp36.MatrMultiply(E_eps, strainD);
			ChMatrixNM<double, 33, 33> KT1;
			KT1 = temp336*strainD* (m_Kfactor + m_Rfactor * m_element->m_Alpha);*/


			KTE = (temp336 * strainD) * (m_Kfactor + m_Rfactor * m_element->m_Alpha) + (temp339 * Gd) * m_Kfactor;
			KTE *= detJ0 * (m_element->m_GaussScaling);
			result = KTE;
		}

		// Compute the Jacobian of the internal forces
		void ChElementBrick_9::ComputeInternalJacobians(double Kfactor, double Rfactor) {
			m_JacobianMatrix.Reset();

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
		//   H = Mfactor * [M] + Kfactor * [K] + Rfactor * [R]
		void ChElementBrick_9::ComputeKRMmatricesGlobal(ChMatrix<>& H, double Kfactor, double Rfactor, double Mfactor) {
			assert((H.GetRows() == 33) && (H.GetColumns() == 33));

			// Calculate the linear combination Kfactor*[K] + Rfactor*[R]
			ComputeInternalJacobians(Kfactor, Rfactor);

			// Load Jac + Mfactor*[M] into H
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
		// Returns also det[J] with J=[dx/du,..], that might be useful in gauss quadrature.
		void ChElementBrick_9::ComputeNF(
			const double U,              // parametric coordinate in volume
			const double V,              // parametric coordinate in volume
			const double W,              // parametric coordinate in volume
			ChVectorDynamic<>& Qi,       // Return result of N'*F  here, maybe with offset block_offset
			double& detJ,                // Return det[J] here
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

		// -----------------------------------------------------------------------------
		// Functions for internal computations
		// -----------------------------------------------------------------------------

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
