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
  protected:
    std::vector<ChSharedPtr<ChNodeFEAxyz> > nodes;

    double thickness;
    ChSharedPtr<ChContinuumElastic> Material;

    ChMatrixNM<double, 24, 24> StiffnessMatrix;  ///< stiffness matrix
    ChMatrixNM<double, 24, 24> MassMatrix;       ///< mass matrix
    ChMatrixNM<double, 3, 1> InertFlexVec;       ///< for element size (EL,EW,EH)
    // EAS
    int elementnumber;
    ChMatrixNM<double, 24, 24> stock_jac_EAS;  ///< EAS per elmeent
    ChMatrixNM<double, 9, 1> stock_alpha_EAS;  ///< EAS per element
    ChMatrixNM<double, 24, 24> stock_KTE;
    ChMatrixNM<double, 24, 1> initialpos;  ///< Initial Coordinate per element
    int flag_HE;

    // EAS
  public:
    ChElementBrick();
    ~ChElementBrick() {}

    virtual int GetNnodes() override { return 8; }
    virtual int GetNcoords() override { return 8 * 3; }
    virtual int GetNdofs() override { return 8 * 3; }

    virtual ChSharedPtr<ChNodeFEAbase> GetNodeN(int n) override { return nodes[n]; }

    void SetNodes(ChSharedPtr<ChNodeFEAxyz> nodeA,
                  ChSharedPtr<ChNodeFEAxyz> nodeB,
                  ChSharedPtr<ChNodeFEAxyz> nodeC,
                  ChSharedPtr<ChNodeFEAxyz> nodeD,
                  ChSharedPtr<ChNodeFEAxyz> nodeE,
                  ChSharedPtr<ChNodeFEAxyz> nodeF,
                  ChSharedPtr<ChNodeFEAxyz> nodeG,
                  ChSharedPtr<ChNodeFEAxyz> nodeH);

    void SetElemNum(int kb) { elementnumber = kb; }

    void
    SetStockAlpha(double a1, double a2, double a3, double a4, double a5, double a6, double a7, double a8, double a9);

    void SetStockJac(ChMatrixNM<double, 24, 24> a) { stock_jac_EAS = a; }

    void SetStockKTE(ChMatrixNM<double, 24, 24> a) { stock_KTE = a; }

    void SetInertFlexVec(ChMatrixNM<double, 3, 1> a) { InertFlexVec = a; }

    int GetElemNum() { return elementnumber; }

    ChMatrixNM<double, 9, 1> GetStockAlpha() { return stock_alpha_EAS; }

    ChMatrixNM<double, 24, 24> GetStockJac() { return stock_jac_EAS; }

    ChMatrixNM<double, 24, 24> GetStockKTE() { return stock_KTE; }

    ChMatrixNM<double, 24, 1> GetInitialPos() { return initialpos; }

    ChSharedPtr<ChNodeFEAxyz> GetNodeA() { return nodes[0]; }

    ChSharedPtr<ChNodeFEAxyz> GetNodeB() { return nodes[1]; }

    ChSharedPtr<ChNodeFEAxyz> GetNodeC() { return nodes[2]; }

    ChSharedPtr<ChNodeFEAxyz> GetNodeD() { return nodes[3]; }

    ChSharedPtr<ChNodeFEAxyz> GetNodeE() { return nodes[4]; }

    ChSharedPtr<ChNodeFEAxyz> GetNodeF() { return nodes[5]; }

    ChSharedPtr<ChNodeFEAxyz> GetNodeG() { return nodes[6]; }

    ChSharedPtr<ChNodeFEAxyz> GetNodeH() { return nodes[7]; }

    double GetLengthX() { return InertFlexVec(0); }
    double GetLengthY() { return InertFlexVec(1); }
    double GetLengthZ() { return InertFlexVec(2); }

    void SetMaterial(ChSharedPtr<ChContinuumElastic> my_material) { Material = my_material; }
    ChSharedPtr<ChContinuumElastic> GetMaterial() { return Material; }

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
