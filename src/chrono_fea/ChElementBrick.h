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

class ChApiFea ChElementBrick : public ChElementGeneric {

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

    const ChMatrixNM<double, 24, 1>& GetInitialPos() const { return m_initialpos; }

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


  private:

	  // Private Data
	  std::vector<ChSharedPtr<ChNodeFEAxyz> > m_nodes; ///< Element nodes

	  double m_thickness;
	  ChSharedPtr<ChContinuumElastic> m_Material;    ///< Elastic Material

	  ChMatrixNM<double, 24, 24> m_StiffnessMatrix;  ///< Stiffness matrix
	  ChMatrixNM<double, 24, 24> m_MassMatrix;       ///< Mass matrix
	  ChMatrixNM<double, 3, 1> m_InertFlexVec;       ///< for element size (EL,EW,EH)
	  // EAS
	  int m_elementnumber;						///< Element number, for EAS
	  ChMatrixNM<double, 24, 24> m_stock_jac_EAS;  ///< EAS per elmeent
	  ChMatrixNM<double, 9, 1> m_stock_alpha_EAS;  ///< EAS per element
	  ChMatrixNM<double, 24, 24> m_stock_KTE;      ///< Analytical Jacobian
	  ChMatrixNM<double, 24, 1> m_initialpos;  ///< Initial Coordinate per element
	  int m_flag_HE;

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
