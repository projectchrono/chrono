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
// Authors: Bryan Peterson, Milad Rakhsha, Antonio Recuero
// =============================================================================
//
// ANCF laminated shell element with four nodes.
//
// =============================================================================

#ifndef CHELEMENTSHELLANCF_H
#define CHELEMENTSHELLANCF_H

#include "ChApiFEA.h"
#include "ChElementShell.h"
#include "physics/ChContinuumMaterial.h"
#include "ChNodeFEAxyzD.h"
#include "core/ChQuadrature.h"

namespace chrono {

namespace fea {

/// ANCF laminated shell element with four nodes.
class ChApiFea ChElementShellANCF : public ChElementShell {
  protected:
    enum JacobianType { ANALYTICAL, NUMERICAL };

    std::vector<ChSharedPtr<ChNodeFEAxyzD> > nodes;

    double thickness;
    int elementnumber;
    double Alpha;
    ChSharedPtr<ChContinuumElastic> Material;

    ChMatrixNM<double, 24, 24> StiffnessMatrix;  ///< stiffness matrix
    ChMatrixNM<double, 24, 24> MassMatrix;       ///< mass matrix

    ChMatrixNM<double, 24, 24> stock_jac_EAS;  ///< EAS per elmeent 24

    ChMatrixNM<double, 24, 24> stock_KTE;  ///< Analytical Jacobian

    ChMatrixNM<double, 24, 1> initialposD;  ///< Initial Coordinate per element
    ChMatrixNM<double, 24, 1> GravForce;    ///< Gravity Force

    // Material Properties for orthotropic per element (14x7) Max #layer is 7
    ChMatrixNM<double, 98, 1> InertFlexVec;    ///< for Laminate shell
    ChMatrixNM<double, 35, 1> StockAlpha_EAS;  ///< StockAlpha(5*7,1): Max #Layer is 7
    ChMatrixNM<double, 7, 2> GaussZRange;      ///< StockAlpha(7,2): Max #Layer is 7 (-1 < GaussZ < 1)

    int NumLayer;

    JacobianType flag_HE;

    double dt;

    int FlagGravity;

    int FlagAirPressure;

  public:
    ChElementShellANCF();
    ~ChElementShellANCF() {}

    virtual int GetNnodes() override { return 4; }
    virtual int GetNcoords() override { return 4 * 6; }
    virtual int GetNdofs() override { return 4 * 6; }

    virtual ChSharedPtr<ChNodeFEAbase> GetNodeN(int n) override { return nodes[n]; }

    void SetNodes(ChSharedPtr<ChNodeFEAxyzD> nodeA,
                  ChSharedPtr<ChNodeFEAxyzD> nodeB,
                  ChSharedPtr<ChNodeFEAxyzD> nodeC,
                  ChSharedPtr<ChNodeFEAxyzD> nodeD);

    //
    // FEM functions
    //

    /// Set the section & material of shell element .
    /// It is a shared property, so it can be shared between other beams.
    void SetThickness(double th) { thickness = th; }  // Total shell thickness

    void SetElemNum(int kb) { elementnumber = kb; }  //// 2015/5/23 for EAS

    void SetStockAlpha(ChMatrixNM<double, 35, 1> a) { StockAlpha_EAS = a; }

    void SetStockJac(ChMatrixNM<double, 24, 24> a) { stock_jac_EAS = a; }  //// 2015/5/23  for EAS

    void SetStockKTE(ChMatrixNM<double, 24, 24> a) { stock_KTE = a; }  //// 2015/5/23  for EAS

    void SetGravForce(ChMatrixNM<double, 24, 1> a) { GravForce = a; }

    void SetInertFlexVec(ChMatrixNM<double, 98, 1> a) { InertFlexVec = a; }  //// 2015/5/28  for Laminate shell

    void SetNumLayer(int a) { NumLayer = a; }  //// 2015/5/28  for Laminate shell

    void SetGaussZRange(ChMatrixNM<double, 7, 2> a) { GaussZRange = a; }  //// 2015/6/1  for Laminate shell

    void Setdt(double a) { dt = a; }  // To calculate structural damping coefficient

    void SetGravityZ(int a) { FlagGravity = a; }  // Gravity Flag

    void SetAirPressure(int a) { FlagAirPressure = a; }  // AirPressure Flag

    /// Get the section & material of the element
    double GetThickness() { return thickness; }  /// Total shell thickness

    int GetElemNum() { return elementnumber; }  //// 2015/5/23  for EAS

    ChMatrixNM<double, 35, 1> GetStockAlpha() { return StockAlpha_EAS; }  //// 2015/5/23  for EAS

    ChMatrixNM<double, 24, 24> GetStockJac() { return stock_jac_EAS; }  //// 2015/5/23  for EAS

    ChMatrixNM<double, 24, 24> GetStockKTE() { return stock_KTE; }  //// Retrieve ananyltical jacobian

    ChMatrixNM<double, 24, 1> GetGravForce() { return GravForce; }

    ChMatrixNM<double, 24, 1> GetInitialPosD() { return initialposD; }  //// 2015/5/23  for Initial position

    ChMatrixNM<double, 98, 1> GetInertFlexVec() { return InertFlexVec; }  //// 2015/5/28  for Laminate shell

    ChMatrixNM<double, 7, 2> GetGaussZRange() { return GaussZRange; }  //// 2015/6/1  for Laminate shell

    int GetNumLayer() { return NumLayer; }  //// 2015/5/28  for Laminate shell

    double Getdt() { return dt; }  //// To calculate structural damping coefficient

    int GetFlagGravity() { return FlagGravity; }  // Gravity Flag

    int GetAirPressure() { return FlagAirPressure; }  // AirPressure Flag

    /// 2015/6/23 Structural Damping
    void SetAlphaDamp(double a) { Alpha = a; }

    double GetAlphaDamp() { return Alpha; }

    /// Get each node
    ChSharedPtr<ChNodeFEAxyzD> GetNodeA() { return nodes[0]; }

    ChSharedPtr<ChNodeFEAxyzD> GetNodeB() { return nodes[1]; }

    ChSharedPtr<ChNodeFEAxyzD> GetNodeC() { return nodes[2]; }

    ChSharedPtr<ChNodeFEAxyzD> GetNodeD() { return nodes[3]; }

    // double GetLengthX() {return nodes[1]->GetX0().x - nodes[0]->GetX0().x;}
    double GetLengthX() { return InertFlexVec(1); }  // For laminate shell. each layer has the same elmenet length

    // double GetLengthY() {return nodes[2]->GetX0().y - nodes[0]->GetX0().y;}
    double GetLengthY() { return InertFlexVec(2); }  // For laminate shell. each layer has the same elmenet length

    void SetMaterial(ChSharedPtr<ChContinuumElastic> my_material) { Material = my_material; }
    ChSharedPtr<ChContinuumElastic> GetMaterial() { return Material; }

    /// Fills the N shape function matrix.
    /// NOTE! actually N should be a 3row, 24 column sparse matrix,
    /// as  N = [s1*eye(3) s2*eye(3) s3*eye(3) s4*eye(3)...]; ,
    /// but to avoid wasting zero and repeated elements, here
    /// it stores only the s1 through s8 values in a 1 row, 8 columns matrix!
    void ShapeFunctions(ChMatrix<>& N, double x, double y, double z);

    /// Fills the N shape function derivative matrix with respect to
    /// the x, y, and z coordinate.
    /// NOTE! to avoid wasting zero and repeated elements, here
    /// it stores only the four values in a 1 row, 8 columns matrix!
    void ShapeFunctionsDerivativeX(ChMatrix<>& Nx, double x, double y, double z);

    void ShapeFunctionsDerivativeY(ChMatrix<>& Ny, double x, double y, double z);

    void ShapeFunctionsDerivativeZ(ChMatrix<>& Nz, double x, double y, double z);

    virtual void Update() override;

    /// Fills the D vector (column matrix) with the current
    /// field values at the nodes of the element, with proper ordering.
    /// If the D vector has not the size of this->GetNdofs(), it will be resized.
    ///  {x_a y_a z_a Dx_a Dx_a Dx_a x_b y_b z_b Dx_b Dy_b Dz_b}
    virtual void GetField(ChMatrixDynamic<>& mD) override;

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
    void ComputeGravityForce();

    virtual void SetupInitial() override;

    /// Sets H as the global stiffness matrix K, scaled  by Kfactor. Optionally, also
    /// superimposes global damping matrix R, scaled by Rfactor, and global mass matrix M multiplied by Mfactor.
    virtual void ComputeKRMmatricesGlobal(ChMatrix<>& H,
                                          double Kfactor,
                                          double Rfactor = 0,
                                          double Mfactor = 0) override;

    /// Computes the internal forces (ex. the actual position of
    /// nodes is not in relaxed reference position) and set values
    /// in the Fi vector.
    virtual void ComputeInternalForces(ChMatrixDynamic<>& Fi) override;

    // [ANS] Shape function for Assumed Naturals Strain (Interpolation of strain and strainD in a thickness direction)
    void shapefunction_ANS_BilinearShell(ChMatrixNM<double, 1, 4>& S_ANS, double x, double y);

    // [ANS] Calculation of ANS strain and strainD
    void AssumedNaturalStrain_BilinearShell(ChMatrixNM<double, 8, 3>& d,
                                            ChMatrixNM<double, 8, 3>& d0,
                                            ChMatrixNM<double, 8, 1>& strain_ans,
                                            ChMatrixNM<double, 8, 24>& strainD_ans);

    // [EAS] Basis function of M for Enhanced Assumed Strain
    void Basis_M(ChMatrixNM<double, 6, 5>& M, double x, double y, double z);

    // [EAS] matrix T0 (inverse and transposed) and detJ0 at center are used for Enhanced Assumed Strains alpha
    void T0DetJElementCenterForEAS(ChMatrixNM<double, 8, 3>& d0,
                                   ChMatrixNM<double, 6, 6>& T0,
                                   double& detJ0C,
                                   double& theta);

    //
    // Functions for interface to ChElementShell base
    //

    virtual void EvaluateSectionDisplacement(const double u,
                                             const double v,
                                             const ChMatrix<>& displ,
                                             ChVector<>& u_displ,
                                             ChVector<>& u_rotaz) override;

    virtual void EvaluateSectionFrame(const double u,
                                      const double v,
                                      const ChMatrix<>& displ,
                                      ChVector<>& point,
                                      ChQuaternion<>& rot) override;

    virtual void EvaluateSectionPoint(const double u,
                                      const double v,
                                      const ChMatrix<>& displ,
                                      ChVector<>& point) override;

    //
    // Helper functions
    //

    //// Temporary function of LU decomposition 1
    static void LUBKSB55(ChMatrixNM<double, 5, 5>& A, int N, int NP, ChMatrixNM<int, 5, 1>& INDX, ChMatrixNM<double, 5, 1>& B);

    static void LUDCMP55(ChMatrixNM<double, 5, 5>& A, double N, double NP, ChMatrixNM<int, 5, 1>& INDX, double D);

    /// Numerial inverse for a 5x5 matrix
    static void Inverse55_Numerical(ChMatrixNM<double, 5, 5>& a, int n);

    /// Analytical inverse for a 5x5 matrix
    static void Inverse55_Analytical(ChMatrixNM<double, 5, 5>& A, ChMatrixNM<double, 5, 5>& B);
};

}  // end of namespace fea
}  // end of namespace chrono

#endif
