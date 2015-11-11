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

namespace chrono {
namespace fea {

class ChApiFea ChElementBrick : public ChElementGeneric, public ChLoadableUVW {
  public:
    ChElementBrick();
    ~ChElementBrick() {}

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
	void SetMRCoefficients(double C1, double C2) { CCOM1 = C1; CCOM2 = C2; }

    //
    // Functions for ChLoadable interface
    //

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
	virtual void ComputeNF(const double U,   ///< parametric coordinate in volume
		const double V,             ///< parametric coordinate in volume
		const double W,             ///< parametric coordinate in volume 
		ChVectorDynamic<>& Qi,      ///< Return result of N'*F  here, maybe with offset block_offset
		double& detJ,               ///< Return det[J] here
		const ChVectorDynamic<>& F, ///< Input F vector, size is = n.field coords.
		ChVectorDynamic<>* state_x, ///< if != 0, update state (pos. part) to this, then evaluate Q
		ChVectorDynamic<>* state_w  ///< if != 0, update state (speed part) to this, then evaluate Q
		) {
		// evaluate shape functions (in compressed vector), btw. not dependant on state
		//ChMatrixNM<double, 1, 8> N;
		//this->ShapeFunctions(N, U, V, W); // note: U,V,W in -1..1 range

		//detJ = this->GetVolume() / 8.0;

		//Qi(0) = N(0)*F(0);
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
