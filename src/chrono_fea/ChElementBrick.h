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
// Authors: Bryan Peterson, Antonio Recuero
// =============================================================================
//
// Brick element with 8 nodes (with EAS)
//
// =============================================================================

#ifndef CHELEMENTBRICK_H
#define CHELEMENTBRICK_H

#include "chrono/physics/ChContinuumMaterial.h"
#include "chrono_fea/ChApiFEA.h"
#include "chrono_fea/ChNodeFEAxyz.h"
#include "core/ChQuadrature.h"
namespace chrono {
namespace fea {

class ChApiFea ChElementBrick : public ChElementGeneric, public ChLoadableUVW {
  public:
    ChElementBrick();
    ~ChElementBrick() {}

    class MyMass : public ChIntegrable3D<ChMatrixNM<double, 24, 24> > {
	public:
		MyMass();  ///< Constructor
		MyMass(ChMatrixNM<double, 8, 3>* m_d0, ChElementBrick* element_) {
			d0 = m_d0;
			element = element_;
		}
		~MyMass() {}
	private:
        ChElementBrick* element;
        ChMatrixNM<double, 8, 3>* d0;
        ChMatrixNM<double, 3, 24> S;
        ChMatrixNM<double, 3, 24> Sx;
        ChMatrixNM<double, 3, 24> Sy;
        ChMatrixNM<double, 3, 24> Sz;
        ChMatrixNM<double, 1, 8> N;
        ChMatrixNM<double, 1, 8> Nx;
        ChMatrixNM<double, 1, 8> Ny;
        ChMatrixNM<double, 1, 8> Nz;

        /// Evaluate the S'*S  at point x
        virtual void Evaluate(ChMatrixNM<double, 24, 24>& result, const double x, const double y, const double z);
    };
    /// Internal force, EAS stiffness, and analytical jacobian are calculated
    class MyForceAnalytical : public ChIntegrable3D<ChMatrixNM<double, 906, 1> > {
	public:
		MyForceAnalytical();
		// Constructor 1
		MyForceAnalytical(ChMatrixNM<double, 8, 3>* d_, ChMatrixNM<double, 8, 3>* m_d0_, ChElementBrick* element_,
			ChMatrixNM<double, 6, 6>* T0_, double* detJ0C_, ChMatrixNM<double, 9, 1>* alpha_eas_) {
			d = d_;
			d0 = m_d0_;
			element = element_;
			T0 = T0_;
			detJ0C = detJ0C_;
			alpha_eas = alpha_eas_;
		}
		//Constructor 2
		MyForceAnalytical(ChMatrixNM<double, 8, 3>* d_, ChMatrixNM<double, 8, 3>* m_d0_, ChElementBrick* element_,
			ChMatrixNM<double, 6, 6>* T0_, double* detJ0C_, ChMatrixNM<double, 9, 1>* alpha_eas_,
			double* E_, double* v_) {
			d = d_;
			d0 = m_d0_;
			element = element_;
			T0 = T0_;
			detJ0C = detJ0C_;
			alpha_eas = alpha_eas_;
			E = E_;
			v = v_;
		}
		~MyForceAnalytical() {}
      private:
        ChElementBrick* element;
        ChMatrixNM<double, 8, 3>* d;  // this is an external matrix, use pointer
        ChMatrixNM<double, 8, 3>* d0;
        ChMatrixNM<double, 6, 6>* T0;
        ChMatrixNM<double, 9, 1>* alpha_eas;
        double* detJ0C;
        double* E;
        double* v;

        ChMatrixNM<double, 24, 1> Fint;
        ChMatrixNM<double, 24, 24> JAC11;
        ChMatrixNM<double, 9, 24> Gd;
        ChMatrixNM<double, 6, 1> stress;
        ChMatrixNM<double, 9, 9> Sigm;
        ChMatrixNM<double, 24, 6> temp246;
        ChMatrixNM<double, 24, 9> temp249;
        ChMatrixNM<double, 6, 6> E_eps;
        ChMatrixNM<double, 3, 24> Sx;
        ChMatrixNM<double, 3, 24> Sy;
        ChMatrixNM<double, 3, 24> Sz;
        ChMatrixNM<double, 1, 8> Nx;
        ChMatrixNM<double, 1, 8> Ny;
        ChMatrixNM<double, 1, 8> Nz;
        ChMatrixNM<double, 6, 24> strainD;
        ChMatrixNM<double, 6, 1> strain;
        ChMatrixNM<double, 8, 8> d_d;
        ChMatrixNM<double, 8, 1> ddNx;
        ChMatrixNM<double, 8, 1> ddNy;
        ChMatrixNM<double, 8, 1> ddNz;
        ChMatrixNM<double, 1, 3> Nxd;
        ChMatrixNM<double, 1, 3> Nyd;
        ChMatrixNM<double, 1, 3> Nzd;
        ChMatrixNM<double, 1, 1> tempA;
        ChMatrixNM<double, 1, 24> tempB;
        ChMatrixNM<double, 24, 6> tempC;
        ChMatrixNM<double, 1, 1> tempA1;  // for strain incase of initial curved
        ChMatrixNM<double, 8, 8> d0_d0;   // for strain incase of initial curved
        ChMatrixNM<double, 8, 1> d0d0Nx;  // for strain incase of initial curved
        ChMatrixNM<double, 8, 1> d0d0Ny;  // for strain incase of initial curved
        ChMatrixNM<double, 8, 1> d0d0Nz;  // for strain incase of initial curved
        double detJ0;
        //	//EAS
        ChMatrixNM<double, 6, 9> M;
        ChMatrixNM<double, 6, 9> G;
        ChMatrixNM<double, 9, 6> GT;
        ChMatrixNM<double, 6, 1> strain_EAS;

        //	/// Evaluate (strainD'*strain)  at point
        virtual void Evaluate(ChMatrixNM<double, 906, 1>& result, const double x, const double y, const double z);
    };  // end of class MyForce

    class MyForceNum : public ChIntegrable3D<ChMatrixNM<double, 330, 1> > {
		public:
		MyForceNum();
		// Constructor 1
		MyForceNum(ChMatrixNM<double, 8, 3>* d_, ChMatrixNM<double, 8, 3>* m_d0_, ChElementBrick* element_, 
			ChMatrixNM<double, 6, 6>* T0_, double* detJ0C_, ChMatrixNM<double, 9, 1>* alpha_eas_) {
			d = d_;
			d0 = m_d0_;
			element = element_;
			T0 = T0_;
			detJ0C = detJ0C_;
			alpha_eas = alpha_eas_;
		}
		//Constructor 2
		MyForceNum(ChMatrixNM<double, 8, 3>* d_, ChMatrixNM<double, 8, 3>* m_d0_, ChElementBrick* element_,
			ChMatrixNM<double, 6, 6>* T0_, double* detJ0C_, ChMatrixNM<double, 9, 1>* alpha_eas_,
			double* E_, double* v_) {
			d = d_;
			d0 = m_d0_;
			element = element_;
			T0 = T0_;
			detJ0C = detJ0C_;
			alpha_eas = alpha_eas_;
			E = E_;
			v = v_;
		}
		~MyForceNum() {}
	   private:
        ChElementBrick* element;
        /// Pointers used for external values
        ChMatrixNM<double, 8, 3>* d;
        ChMatrixNM<double, 8, 3>* d0;
        ChMatrixNM<double, 6, 6>* T0;
        ChMatrixNM<double, 9, 1>* alpha_eas;
        double* detJ0C;
        double* E;
        double* v;
        ChMatrixNM<double, 24, 1> Fint;
        ChMatrixNM<double, 6, 6> E_eps;
        ChMatrixNM<double, 3, 24> Sx;
        ChMatrixNM<double, 3, 24> Sy;
        ChMatrixNM<double, 3, 24> Sz;
        ChMatrixNM<double, 1, 8> Nx;
        ChMatrixNM<double, 1, 8> Ny;
        ChMatrixNM<double, 1, 8> Nz;
        ChMatrixNM<double, 6, 24> strainD;
        ChMatrixNM<double, 6, 1> strain;
        ChMatrixNM<double, 8, 8> d_d;
        ChMatrixNM<double, 8, 1> ddNx;
        ChMatrixNM<double, 8, 1> ddNy;
        ChMatrixNM<double, 8, 1> ddNz;
        ChMatrixNM<double, 1, 3> Nxd;
        ChMatrixNM<double, 1, 3> Nyd;
        ChMatrixNM<double, 1, 3> Nzd;
        ChMatrixNM<double, 1, 1> tempA;
        ChMatrixNM<double, 1, 24> tempB;
        ChMatrixNM<double, 24, 6> tempC;
        ChMatrixNM<double, 1, 1> tempA1;  // for strain incase of initial curved
        ChMatrixNM<double, 8, 8> d0_d0;   // for strain incase of initial curved
        ChMatrixNM<double, 8, 1> d0d0Nx;  // for strain incase of initial curved
        ChMatrixNM<double, 8, 1> d0d0Ny;  // for strain incase of initial curved
        ChMatrixNM<double, 8, 1> d0d0Nz;  // for strain incase of initial curved
        double detJ0;
        // EAS
        ChMatrixNM<double, 6, 9> M;
        ChMatrixNM<double, 6, 9> G;
        ChMatrixNM<double, 9, 6> GT;
        ChMatrixNM<double, 6, 1> strain_EAS;

        /// Gaussian integration to calculate internal forces and EAS matrices
        virtual void Evaluate(ChMatrixNM<double, 330, 1>& result, const double x, const double y, const double z);
    };

	class MyGravity : public ChIntegrable3D<ChMatrixNM<double, 24, 1> > {
	public:
		MyGravity();
		MyGravity(ChMatrixNM<double, 8, 3>* m_d0, ChElementBrick* element_) {
			d0 = m_d0;
			element = element_;
		}
		~MyGravity() {}

	private:
		ChElementBrick* element;
		ChMatrixNM<double, 8, 3>* d0;
		ChMatrixNM<double, 3, 24> S;
		ChMatrixNM<double, 1, 8> N;
		ChMatrixNM<double, 1, 8> Nx;
		ChMatrixNM<double, 1, 8> Ny;
		ChMatrixNM<double, 1, 8> Nz;
		ChMatrixNM<double, 3, 1> LocalGravityForce;

		virtual void Evaluate(ChMatrixNM<double, 24, 1>& result, const double x, const double y, const double z);
	};
    virtual int GetNnodes() override { return 8; }
    virtual int GetNcoords() override { return 8 * 3; }
    virtual int GetNdofs() override { return 8 * 3; }

    virtual ChSharedPtr<ChNodeFEAbase> GetNodeN(int n) override { return m_nodes[n]; }

    void SetNodes(ChSharedPtr<ChNodeFEAxyz> nodeA,
                  ChSharedPtr<ChNodeFEAxyz> nodeB,
                  ChSharedPtr<ChNodeFEAxyz> nodeC,
                  ChSharedPtr<ChNodeFEAxyz> nodeD,
                  ChSharedPtr<ChNodeFEAxyz> nodeE,
                  ChSharedPtr<ChNodeFEAxyz> nodeF,
                  ChSharedPtr<ChNodeFEAxyz> nodeG,
                  ChSharedPtr<ChNodeFEAxyz> nodeH);

    void SetElemNum(int kb) { m_elementnumber = kb; }
    void
    SetStockAlpha(double a1, double a2, double a3, double a4, double a5, double a6, double a7, double a8, double a9);
    void SetStockJac(const ChMatrixNM<double, 24, 24>& a) { m_stock_jac_EAS = a; }
    void SetStockKTE(const ChMatrixNM<double, 24, 24>& a) { m_stock_KTE = a; }
    void SetInertFlexVec(const ChMatrixNM<double, 3, 1>& a) { m_InertFlexVec = a; }

    int GetElemNum() const { return m_elementnumber; }

    const ChMatrixNM<double, 8, 3>& GetInitialPos() const { return m_d0; }

    ChSharedPtr<ChNodeFEAxyz> GetNodeA() const { return m_nodes[0]; }
    ChSharedPtr<ChNodeFEAxyz> GetNodeB() const { return m_nodes[1]; }
    ChSharedPtr<ChNodeFEAxyz> GetNodeC() const { return m_nodes[2]; }
    ChSharedPtr<ChNodeFEAxyz> GetNodeD() const { return m_nodes[3]; }
    ChSharedPtr<ChNodeFEAxyz> GetNodeE() const { return m_nodes[4]; }
    ChSharedPtr<ChNodeFEAxyz> GetNodeF() const { return m_nodes[5]; }
    ChSharedPtr<ChNodeFEAxyz> GetNodeG() const { return m_nodes[6]; }
    ChSharedPtr<ChNodeFEAxyz> GetNodeH() const { return m_nodes[7]; }

    double GetLengthX() const { return m_InertFlexVec(0); }
    double GetLengthY() const { return m_InertFlexVec(1); }
    double GetLengthZ() const { return m_InertFlexVec(2); }

    void SetMaterial(ChSharedPtr<ChContinuumElastic> my_material) { m_Material = my_material; }
    ChSharedPtr<ChContinuumElastic> GetMaterial() const { return m_Material; }
    /// Turn gravity on/off.
    void SetGravityOn(bool val) { m_gravity_on = val; }
    void SetMooneyRivlin(bool val) { m_isMooney = val; }
    void SetMRCoefficients(double C1, double C2) {
        CCOM1 = C1;
        CCOM2 = C2;
    }

    // Functions for ChLoadable interface
    /// Gets the number of DOFs affected by this element (position part)
    virtual int LoadableGet_ndof_x() { return 8 * 3; }

    /// Gets the number of DOFs affected by this element (speed part)
    virtual int LoadableGet_ndof_w() { return 8 * 3; }

    /// Gets all the DOFs packed in a single vector (position part)
    virtual void LoadableGetStateBlock_x(int block_offset, ChVectorDynamic<>& mD) {
        mD.PasteVector(this->m_nodes[0]->GetPos(), block_offset, 0);
        mD.PasteVector(this->m_nodes[1]->GetPos(), block_offset + 3, 0);
        mD.PasteVector(this->m_nodes[2]->GetPos(), block_offset + 6, 0);
        mD.PasteVector(this->m_nodes[3]->GetPos(), block_offset + 9, 0);
        mD.PasteVector(this->m_nodes[4]->GetPos(), block_offset + 12, 0);
        mD.PasteVector(this->m_nodes[5]->GetPos(), block_offset + 15, 0);
        mD.PasteVector(this->m_nodes[6]->GetPos(), block_offset + 18, 0);
        mD.PasteVector(this->m_nodes[7]->GetPos(), block_offset + 21, 0);
    }

    /// Gets all the DOFs packed in a single vector (speed part)
    virtual void LoadableGetStateBlock_w(int block_offset, ChVectorDynamic<>& mD) {
        mD.PasteVector(this->m_nodes[0]->GetPos_dt(), block_offset, 0);
        mD.PasteVector(this->m_nodes[1]->GetPos_dt(), block_offset + 3, 0);
        mD.PasteVector(this->m_nodes[2]->GetPos_dt(), block_offset + 6, 0);
        mD.PasteVector(this->m_nodes[3]->GetPos_dt(), block_offset + 9, 0);
        mD.PasteVector(this->m_nodes[4]->GetPos_dt(), block_offset + 12, 0);
        mD.PasteVector(this->m_nodes[5]->GetPos_dt(), block_offset + 15, 0);
        mD.PasteVector(this->m_nodes[6]->GetPos_dt(), block_offset + 18, 0);
        mD.PasteVector(this->m_nodes[7]->GetPos_dt(), block_offset + 21, 0);
    }

    /// Number of coordinates in the interpolated field: here the {x,y,z} displacement
    virtual int Get_field_ncoords() { return 3; }

    /// Tell the number of DOFs blocks (ex. =1 for a body, =4 for a tetrahedron, etc.)
    virtual int GetSubBlocks() { return 8; }

    /// Get the offset of the i-th sub-block of DOFs in global vector
    virtual unsigned int GetSubBlockOffset(int nblock) { return m_nodes[nblock]->NodeGetOffset_w(); }

    /// Get the size of the i-th sub-block of DOFs in global vector
    virtual unsigned int GetSubBlockSize(int nblock) { return 3; }

    /// Get the pointers to the contained ChLcpVariables, appending to the mvars vector.
    virtual void LoadableGetVariables(std::vector<ChLcpVariables*>& mvars) {
        for (int i = 0; i < m_nodes.size(); ++i)
            mvars.push_back(&this->m_nodes[i]->Variables());
    };

    /// Evaluate N'*F , where N is some type of shape function
    /// evaluated at U,V,W coordinates of the volume, each ranging in -1..+1
    /// F is a load, N'*F is the resulting generalized load
    /// Returns also det[J] with J=[dx/du,..], that might be useful in gauss quadrature.
    virtual void ComputeNF(const double U,              ///< parametric coordinate in volume
                           const double V,              ///< parametric coordinate in volume
                           const double W,              ///< parametric coordinate in volume
                           ChVectorDynamic<>& Qi,       ///< Return result of N'*F  here, maybe with offset block_offset
                           double& detJ,                ///< Return det[J] here
                           const ChVectorDynamic<>& F,  ///< Input F vector, size is = n.field coords.
                           ChVectorDynamic<>* state_x,  ///< if != 0, update state (pos. part) to this, then evaluate Q
                           ChVectorDynamic<>* state_w   ///< if != 0, update state (speed part) to this, then evaluate Q
                           ) {
        // evaluate shape functions (in compressed vector), btw. not dependant on state
        // ChMatrixNM<double, 1, 8> N;
        // this->ShapeFunctions(N, U, V, W); // note: U,V,W in -1..1 range

        // detJ = this->GetVolume() / 8.0;

        // Qi(0) = N(0)*F(0);
        // TO DO
    }

    /// This is needed so that it can be accessed by ChLoaderVolumeGravity
    virtual double GetDensity() { return 0; }

  private:
    enum JacobianType { ANALYTICAL, NUMERICAL };

    // Private Data
    std::vector<ChSharedPtr<ChNodeFEAxyz> > m_nodes;  ///< Element nodes

    double m_thickness;
    ChSharedPtr<ChContinuumElastic> m_Material;  ///< Elastic Material

    ChMatrixNM<double, 24, 24> m_StiffnessMatrix;  ///< Stiffness matrix
    ChMatrixNM<double, 24, 24> m_MassMatrix;       ///< Mass matrix
    ChMatrixNM<double, 3, 1> m_InertFlexVec;       ///< for element size (EL,EW,EH)
    // EAS
    int m_elementnumber;                         ///< Element number, for EAS
    ChMatrixNM<double, 24, 24> m_stock_jac_EAS;  ///< EAS per elmeent
    ChMatrixNM<double, 9, 1> m_stock_alpha_EAS;  ///< EAS per element
    ChMatrixNM<double, 24, 24> m_stock_KTE;      ///< Analytical Jacobian
    ChMatrixNM<double, 8, 3> m_d0;               ///< Initial Coordinate per element
    JacobianType m_flag_HE;
    bool m_gravity_on;  ///< Flag indicating whether or not gravity is included
    bool m_isMooney;    ///< Flag indicating whether the material is Mooney Rivlin
    double CCOM1;       ///< First coefficient for Mooney-Rivlin
    double CCOM2;       ///< Second coefficient for Mooney-Rivlin
                        // Private Methods
    /// Fills the N shape function matrix
    /// as  N = [s1*eye(3) s2*eye(3) s3*eye(3) s4*eye(3)...]; ,
    void ShapeFunctions(ChMatrix<>& N, double x, double y, double z);
    void ShapeFunctionsDerivativeX(ChMatrix<>& Nx, double x, double y, double z);
    void ShapeFunctionsDerivativeY(ChMatrix<>& Ny, double x, double y, double z);
    void ShapeFunctionsDerivativeZ(ChMatrix<>& Nz, double x, double y, double z);

    virtual void Update() override;

    /// Fills the D vector (column matrix) with the current
    /// field values at the nodes of the element, with proper ordering.
    /// If the D vector has not the size of this->GetNdofs(), it will be resized.
    ///  {x_a y_a z_a Dx_a Dx_a Dx_a x_b y_b z_b Dx_b Dy_b Dz_b}
    virtual void GetStateBlock(ChMatrixDynamic<>& mD);

    /// Computes the STIFFNESS MATRIX of the element:
    /// K = integral( .... ),
    /// Note: in this 'basic' implementation, constant section and
    /// constant material are assumed
    void ComputeStiffnessMatrix();
    /// Computes the MASS MATRIX of the element
    /// Note: in this 'basic' implementation, constant section and
    /// constant material are assumed
    void ComputeMassMatrix();
    /// Setup. Precompute mass and matrices that do not change during the
    /// simulation, ex. the mass matrix in ANCF is constant

    virtual void SetupInitial() override;
    /// Sets H as the global stiffness matrix K, scaled  by Kfactor. Optionally, also
    /// superimposes global damping matrix R, scaled by Rfactor, and global mass matrix M multiplied by Mfactor.
    void ComputeKRMmatricesGlobal(ChMatrix<>& H, double Kfactor, double Rfactor = 0, double Mfactor = 0);

    /// Computes the internal forces (ex. the actual position of
    /// nodes is not in relaxed reference position) and set values
    /// in the Fi vector.
    virtual void ComputeInternalForces(ChMatrixDynamic<>& Fi) override;

    // [EAS] matrix T0 (inverse and transposed) and detJ0 at center are used for Enhanced Assumed Strains alpha
    void T0DetJElementCenterForEAS(ChMatrixNM<double, 8, 3>& d0, ChMatrixNM<double, 6, 6>& T0, double& detJ0C);
    // [EAS] Basis function of M for Enhanced Assumed Strain
    void Basis_M(ChMatrixNM<double, 6, 9>& M, double x, double y, double z);
};

}  // end namespace fea
}  // end namespace chrono

#endif
