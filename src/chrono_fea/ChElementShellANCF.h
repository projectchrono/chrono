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
// Authors: Bryan Peterson, Milad Rakhsha, Antonio Recuero, Radu Serban
// =============================================================================
//
// ANCF laminated shell element with four nodes.
//
// =============================================================================

#ifndef CHELEMENTSHELLANCF_H
#define CHELEMENTSHELLANCF_H

#include "chrono/physics/ChContinuumMaterial.h"

#include "chrono_fea/ChApiFEA.h"
#include "chrono_fea/ChElementShell.h"
#include "chrono_fea/ChNodeFEAxyzD.h"

namespace chrono {

namespace fea {

///
/// ANCF laminated shell element with four nodes.
///
class ChApiFea ChElementShellANCF : public ChElementShell {
  public:
    ChElementShellANCF();
    ~ChElementShellANCF() {}

    /// Get the number of nodes used by this element.
    virtual int GetNnodes() override { return 4; }

    /// Get the number of coordinates of the node positions in space.
    /// Note this is not the coordinates of the field, use GetNdofs() instead.
    virtual int GetNcoords() override { return 4 * 6; }

    /// Get the number of coordinates in the field used by the referenced nodes.
    virtual int GetNdofs() override { return 4 * 6; }

    /// Specify the nodes of this element.
    void SetNodes(ChSharedPtr<ChNodeFEAxyzD> nodeA,
                  ChSharedPtr<ChNodeFEAxyzD> nodeB,
                  ChSharedPtr<ChNodeFEAxyzD> nodeC,
                  ChSharedPtr<ChNodeFEAxyzD> nodeD);

    /// Access the n-th node of this element.
    virtual ChSharedPtr<ChNodeFEAbase> GetNodeN(int n) override { return m_nodes[n]; }

    /// Get a handle to the first node of this element.
    ChSharedPtr<ChNodeFEAxyzD> GetNodeA() const { return m_nodes[0]; }

    /// Get a handle to the second node of this element.
    ChSharedPtr<ChNodeFEAxyzD> GetNodeB() const { return m_nodes[1]; }

    /// Get a handle to the third node of this element.
    ChSharedPtr<ChNodeFEAxyzD> GetNodeC() const { return m_nodes[2]; }

    /// Get a handle to the fourth node of this element.
    ChSharedPtr<ChNodeFEAxyzD> GetNodeD() const { return m_nodes[3]; }

    /// Set the numbber of layers (for laminate shell)
    void SetNumLayers(int numLayers) { m_numLayers = numLayers; }

    /// Set the stotal shell thickness.
    void SetThickness(double th) { m_thickness = th; }

    /// Set the element number (for EAS).
    /// Used for debugging purposes only.
    void SetElemNum(int kb) { m_element_number = kb; }

    /// Set the continuum elastic material.
    void SetMaterial(ChSharedPtr<ChContinuumElastic> my_material) { m_Material = my_material; }

    /// Get a handle to the material for this element.
    ChSharedPtr<ChContinuumElastic> GetMaterial() const { return m_Material; }

    void SetStockAlpha(const ChMatrixNM<double, 35, 1>& a) { m_StockAlpha_EAS = a; }

    const ChMatrixNM<double, 35, 1>& GetStockAlpha() const { return m_StockAlpha_EAS; }  //// for EAS

    void SetStockJac(const ChMatrixNM<double, 24, 24>& a) { m_stock_jac_EAS = a; }  //// for EAS

    void SetStockKTE(const ChMatrixNM<double, 24, 24>& a) { m_stock_KTE = a; }  //// for EAS

    void SetInertFlexVec(const ChMatrixNM<double, 98, 1>& a) { m_InertFlexVec = a; }  //// for Laminate shell

    const ChMatrixNM<double, 98, 1>& GetInertFlexVec() const { return m_InertFlexVec; }  //// for Laminate shell

    void SetGaussZRange(const ChMatrixNM<double, 7, 2>& a) { m_GaussZRange = a; }  //// for Laminate shell

    const ChMatrixNM<double, 7, 2>& GetGaussZRange() const { return m_GaussZRange; }  //// for Laminate shell

    /// Set the step size used in calculating the structural damping coefficient.
    void Setdt(double a) { m_dt = a; }

    /// Turn gravity on/off.
    void SetGravityOn(bool val) { m_gravity_on = val; }

    /// Turn air pressure on/off.
    void SetAirPressureOn(bool val) { m_air_pressure_on = val; }

    /// Set the structural damping.
    void SetAlphaDamp(double a) { m_Alpha = a; }

    /// Get the element length in the X direction.
    /// For laminate shell. each layer has the same element length
    double GetLengthX() const { return m_InertFlexVec(1); }

    /// Get the element length in the Y direction.
    /// For laminate shell. each layer has the same element length
    double GetLengthY() const { return m_InertFlexVec(2); }

  private:
    enum JacobianType { ANALYTICAL, NUMERICAL };

    std::vector<ChSharedPtr<ChNodeFEAxyzD> > m_nodes;  ///< element nodes

    double m_thickness;
    int m_element_number;                        ///< element number (for EAS)
    double m_Alpha;                              ///< structural damping
    ChSharedPtr<ChContinuumElastic> m_Material;  ///< elastic material

    ChMatrixNM<double, 24, 24> m_StiffnessMatrix;  ///< stiffness matrix
    ChMatrixNM<double, 24, 24> m_MassMatrix;       ///< mass matrix

    ChMatrixNM<double, 24, 24> m_stock_jac_EAS;  ///< EAS per elmeent 24
    ChMatrixNM<double, 24, 24> m_stock_KTE;      ///< Analytical Jacobian

    ChMatrixNM<double, 24, 1> m_initialposD;  ///< Initial Coordinate per element
    ChMatrixNM<double, 24, 1> m_GravForce;    ///< Gravity Force

    // Material Properties for orthotropic per element (14x7) Max #layer is 7
    ChMatrixNM<double, 98, 1> m_InertFlexVec;    ///< for Laminate shell
    ChMatrixNM<double, 35, 1> m_StockAlpha_EAS;  ///< StockAlpha(5*7,1): Max #Layer is 7
    ChMatrixNM<double, 7, 2> m_GaussZRange;      ///< StockAlpha(7,2): Max #Layer is 7 (-1 < GaussZ < 1)

    int m_numLayers;  ///< number of layers for this element

    JacobianType m_flag_HE;  ///< Jacobian evaluation type (analytical or numerical)

    double m_dt;  ///< time step used in calculating structural damping coefficient

    bool m_gravity_on;       ///< flag indicating whether or not gravity is included
    bool m_air_pressure_on;  ///< flag indicating whether or not air pressure is included

    // Interface to ChElementBase base class
    // -------------------------------------

    /// Fills the D vector (column matrix) with the current
    /// field values at the nodes of the element, with proper ordering.
    /// If the D vector has not the size of this->GetNdofs(), it will be resized.
    ///  {x_a y_a z_a Dx_a Dx_a Dx_a x_b y_b z_b Dx_b Dy_b Dz_b}
    virtual void GetStateBlock(ChMatrixDynamic<>& mD) override;

    /// Sets H as the global stiffness matrix K, scaled  by Kfactor.
    /// Optionally, also superimposes global damping matrix R, scaled by Rfactor, and global
    /// mass matrix M multiplied by Mfactor.
    virtual void ComputeKRMmatricesGlobal(ChMatrix<>& H,
                                          double Kfactor,
                                          double Rfactor = 0,
                                          double Mfactor = 0) override;

    /// Computes the internal forces.
    /// (E.g. the actual position of nodes is not in relaxed reference position) and set values
    /// in the Fi vector.
    virtual void ComputeInternalForces(ChMatrixDynamic<>& Fi) override;

    /// Initial setup.
    /// This is used mostly to precompute matrices that do not change during the simulation,
    /// such as the local stiffness of each element (if any), the mass, etc.
    virtual void SetupInitial() override;

    /// Update the state of this element.
    virtual void Update() override;

    // Interface to ChElementShell base class
    // --------------------------------------

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

    // Internal computations
    // ---------------------

    /// Compute the STIFFNESS MATRIX of the element.
    /// K = integral( .... ),
    /// Note: in this 'basic' implementation, constant section and
    /// constant material are assumed
    void ComputeStiffnessMatrix();

    /// Compute the MASS MATRIX of the element.
    /// Note: in this 'basic' implementation, constant section and
    /// constant material are assumed
    void ComputeMassMatrix();

    /// Compute the gravitational forces.
    void ComputeGravityForce();

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

    // Shape functions
    // ---------------

    /// Fills the N shape function matrix.
    /// NOTE! actually N should be a 3row, 24 column sparse matrix,
    /// as  N = [s1*eye(3) s2*eye(3) s3*eye(3) s4*eye(3)...]; ,
    /// but to avoid wasting zero and repeated elements, here
    /// it stores only the s1 through s8 values in a 1 row, 8 columns matrix!
    void ShapeFunctions(ChMatrix<>& N, double x, double y, double z);

    /// Fills the Nx shape function derivative matrix with respect to X.
    /// NOTE! to avoid wasting zero and repeated elements, here
    /// it stores only the four values in a 1 row, 8 columns matrix!
    void ShapeFunctionsDerivativeX(ChMatrix<>& Nx, double x, double y, double z);

    /// Fills the Ny shape function derivative matrix with respect to Y.
    /// NOTE! to avoid wasting zero and repeated elements, here
    /// it stores only the four values in a 1 row, 8 columns matrix!
    void ShapeFunctionsDerivativeY(ChMatrix<>& Ny, double x, double y, double z);

    /// Fills the Nz shape function derivative matrix with respect to Z.
    /// NOTE! to avoid wasting zero and repeated elements, here
    /// it stores only the four values in a 1 row, 8 columns matrix!
    void ShapeFunctionsDerivativeZ(ChMatrix<>& Nz, double x, double y, double z);

    // Helper functions
    // ----------------

    /// Numerial inverse for a 5x5 matrix
    static void Inverse55_Numerical(ChMatrixNM<double, 5, 5>& a, int n);

    /// Analytical inverse for a 5x5 matrix
    static void Inverse55_Analytical(ChMatrixNM<double, 5, 5>& A, ChMatrixNM<double, 5, 5>& B);
};

}  // end of namespace fea
}  // end of namespace chrono

#endif
