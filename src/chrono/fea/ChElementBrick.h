// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
// =============================================================================
// Authors: Bryan Peterson, Antonio Recuero, Radu Serban
// =============================================================================
// Brick element with 8 nodes (with EAS)
// =============================================================================

//// RADU
//// Move implementation to cpp file!
//// Move MyMass, MyForce, etc.

#ifndef CHELEMENTBRICK_H
#define CHELEMENTBRICK_H

#include "chrono/core/ChQuadrature.h"
#include "chrono/physics/ChContinuumMaterial.h"
#include "chrono/physics/ChLoadable.h"
#include "chrono/fea/ChElementGeneric.h"
#include "chrono/fea/ChNodeFEAxyz.h"

namespace chrono {
namespace fea {

/// @addtogroup fea_elements
/// @{

/// Brick element with 8 nodes (with EAS).
class ChApi ChElementBrick : public ChElementGeneric, public ChLoadableUVW {
  public:
    using ShapeVector = ChMatrixNM<double, 1, 8>;

    ChElementBrick();
    ~ChElementBrick() {}

    class MyMass : public ChIntegrable3D<ChMatrixNM<double, 24, 24> > {
      public:
        MyMass(ChMatrixNM<double, 8, 3>* m_d0, ChElementBrick* element_) {
            d0 = m_d0;
            element = element_;

            S.setZero();
        }
        ~MyMass() {}

      private:
        ChElementBrick* element;
        ChMatrixNM<double, 8, 3>* d0;  ///< Pointer to a matrix containing the element initial coordinates
        ChMatrixNM<double, 3, 24> S;   ///< Sparse shape function matrix
        ChElementBrick::ShapeVector N;    ///< Dense shape function vector
        ChElementBrick::ShapeVector Nx;   ///< Dense shape function vector, X derivative
        ChElementBrick::ShapeVector Ny;   ///< Dense shape function vector, Y derivative
        ChElementBrick::ShapeVector Nz;   ///< Dense shape function vector, Z derivative

        /// Evaluate the S'*S  at point x
        virtual void Evaluate(ChMatrixNM<double, 24, 24>& result,
                              const double x,
                              const double y,
                              const double z) override;
    };

    /// Internal force, EAS stiffness, and analytical jacobian are calculated
    class MyForceAnalytical : public ChIntegrable3D<ChVectorN<double, 906> > {
      public:
        MyForceAnalytical() {}
        MyForceAnalytical(ChMatrixNM<double, 8, 3>* d_,
                          ChMatrixNM<double, 8, 3>* m_d0_,
                          ChElementBrick* element_,
                          ChMatrixNM<double, 6, 6>* T0_,
                          double* detJ0C_,
                          ChVectorN<double, 9>* alpha_eas_) {
            d = d_;
            d0 = m_d0_;
            element = element_;
            T0 = T0_;
            detJ0C = detJ0C_;
            alpha_eas = alpha_eas_;

            E_eps.setZero();
            Gd.setZero();
            Sigm.setZero();

            Sx.setZero();
            Sy.setZero();
            Sz.setZero();
        }

        MyForceAnalytical(ChMatrixNM<double, 8, 3>* d_,
                          ChMatrixNM<double, 8, 3>* m_d0_,
                          ChElementBrick* element_,
                          ChMatrixNM<double, 6, 6>* T0_,
                          double* detJ0C_,
                          ChVectorN<double, 9>* alpha_eas_,
                          double* E_,
                          double* v_) {
            d = d_;
            d0 = m_d0_;
            element = element_;
            T0 = T0_;
            detJ0C = detJ0C_;
            alpha_eas = alpha_eas_;
            E = E_;
            v = v_;

            E_eps.setZero();
            Gd.setZero();
            Sigm.setZero();

            Sx.setZero();
            Sy.setZero();
            Sz.setZero();
        }
        ~MyForceAnalytical() {}

      private:
        ChElementBrick* element;
        ChMatrixNM<double, 8, 3>* d;      ///< Pointer to a matrix containing the element coordinates
        ChMatrixNM<double, 8, 3>* d0;     ///< Pointer to a matrix containing the element initial coordinates
        ChMatrixNM<double, 6, 6>* T0;     ///< Pointer to transformation matrix for Enhanced Assumed Strain (EAS)
        ChVectorN<double, 9>* alpha_eas;  ///< Pointer to the 9 internal parameters for EAS
        double* detJ0C;                   ///< Pointer to determinant of the initial Jacobian at the element center
        double* E;                        ///< Pointer to Young modulus
        double* v;                        ///< Pointer to Poisson ratio

        ChVectorN<double, 24> Fint;        ///< Generalized internal (elastic) force vector
        ChMatrixNM<double, 24, 24> JAC11;  ///< Jacobian of internal forces for implicit numerical integration
        ChMatrixNM<double, 9, 24> Gd;      ///< Jacobian (w.r.t. coordinates) of the initial pos. vector gradient matrix
        ChVectorN<double, 6> stress;       ///< stress tensor in vector form
        ChMatrixNM<double, 9, 9> Sigm;     ///< stress tensor in sparse form
        ChMatrixNM<double, 6, 6> E_eps;    ///< Matrix of elastic coefficients (features orthotropy)
        ChMatrixNM<double, 3, 24> Sx;       ///< Sparse shape function matrix, X derivative
        ChMatrixNM<double, 3, 24> Sy;       ///< Sparse shape function matrix, Y derivative
        ChMatrixNM<double, 3, 24> Sz;       ///< Sparse shape function matrix, Z derivative
        ChElementBrick::ShapeVector Nx;     ///< Dense shape function vector, X derivative
        ChElementBrick::ShapeVector Ny;     ///< Dense shape function vector, Y derivative
        ChElementBrick::ShapeVector Nz;     ///< Dense shape function vector, Z derivative
        ChMatrixNM<double, 6, 24> strainD;  ///< Derivative of the strains w.r.t. the coordinates. Includes orthotropy
        ChVectorN<double, 6> strain;        ///< Vector of strains
        double detJ0;                       ///< Determinant of the initial position vector gradient matrix
        // EAS
        ChMatrixNM<double, 6, 9> M;       ///< Shape function matrix for Enhanced Assumed Strain
        ChMatrixNM<double, 6, 9> G;       ///< Matrix G interpolates the internal parameters of EAS
        ChVectorN<double, 6> strain_EAS;  ///< Enhanced assumed strain vector

        /// Evaluate (strainD'*strain)  at a point
        virtual void Evaluate(ChVectorN<double, 906>& result, const double x, const double y, const double z) override;
    };

    class MyForceNum : public ChIntegrable3D<ChVectorN<double, 330> > {
      public:
        MyForceNum(ChMatrixNM<double, 8, 3>* d_,
                   ChMatrixNM<double, 8, 3>* m_d0_,
                   ChElementBrick* element_,
                   ChMatrixNM<double, 6, 6>* T0_,
                   double* detJ0C_,
                   ChVectorN<double, 9>* alpha_eas_) {
            d = d_;
            d0 = m_d0_;
            element = element_;
            T0 = T0_;
            detJ0C = detJ0C_;
            alpha_eas = alpha_eas_;

            E_eps.setZero();

            Sx.setZero();
            Sy.setZero();
            Sz.setZero();
        }

        MyForceNum(ChMatrixNM<double, 8, 3>* d_,
                   ChMatrixNM<double, 8, 3>* m_d0_,
                   ChElementBrick* element_,
                   ChMatrixNM<double, 6, 6>* T0_,
                   double* detJ0C_,
                   ChVectorN<double, 9>* alpha_eas_,
                   double* E_,
                   double* v_) {
            d = d_;
            d0 = m_d0_;
            element = element_;
            T0 = T0_;
            detJ0C = detJ0C_;
            alpha_eas = alpha_eas_;
            E = E_;
            v = v_;

            E_eps.setZero();

            Sx.setZero();
            Sy.setZero();
            Sz.setZero();
        }
        ~MyForceNum() {}

      private:
        ChElementBrick* element;
        /// Pointers used for external values
        ChMatrixNM<double, 8, 3>* d;          ///< Pointer to a matrix containing the element coordinates
        ChMatrixNM<double, 8, 3>* d0;         ///< Pointer to a matrix containing the element initial coordinates
        ChMatrixNM<double, 6, 6>* T0;         ///< Pointer to transformation matrix for Enhanced Assumed Strain (EAS)
        ChVectorN<double, 9>* alpha_eas;      ///< Pointer to the 9 internal parameters for EAS
        double* detJ0C;                       ///< Pointer to determinant of the initial Jacobian at the element center
        double* E;                            ///< Pointer to Young modulus
        double* v;                            ///< Pointer to Poisson ratio
        ChVectorN<double, 24> Fint;           ///< Generalized internal (elastic) force vector
        ChMatrixNM<double, 6, 6> E_eps;       ///< Matrix of elastic coefficients (features orthotropy)
        ChMatrixNM<double, 3, 24> Sx;         ///< Sparse shape function matrix, X derivative
        ChMatrixNM<double, 3, 24> Sy;         ///< Sparse shape function matrix, Y derivative
        ChMatrixNM<double, 3, 24> Sz;         ///< Sparse shape function matrix, Z derivative
        ChElementBrick::ShapeVector Nx;       ///< Dense shape function vector, X derivative
        ChElementBrick::ShapeVector Ny;       ///< Dense shape function vector, Y derivative
        ChElementBrick::ShapeVector Nz;       ///< Dense shape function vector, Z derivative
        ChMatrixNM<double, 6, 24> strainD;    ///< Derivative of the strains w.r.t. the coordinates. Includes orthotropy
        ChVectorN<double, 6> strain;          ///< Vector of strains
        double detJ0;                         ///< Determinant of the initial position vector gradient matrix
        // EAS
        ChMatrixNM<double, 6, 9> M;       ///< Shape function matrix for Enhanced Assumed Strain
        ChMatrixNM<double, 6, 9> G;       ///< Matrix G interpolates the internal parameters of EAS
        ChVectorN<double, 6> strain_EAS;  ///< Enhanced assumed strain vector

        /// Gaussian integration to calculate internal forces and EAS matrices
        virtual void Evaluate(ChVectorN<double, 330>& result, const double x, const double y, const double z) override;
    };

    /// Class to calculate the gravity forces of a brick element
    class MyGravity : public ChIntegrable3D<ChVectorN<double, 24> > {
      public:
        MyGravity(ChMatrixNM<double, 8, 3>* m_d0, ChElementBrick* element_, const ChVector<> g_acc)
            : d0(m_d0), element(element_), gacc(g_acc) {}
        ~MyGravity() {}

      private:
        ChElementBrick* element;
        ChMatrixNM<double, 8, 3>* d0;    ///< Pointer to a matrix containing the element initial coordinates
        ChMatrixNM<double, 3, 24> S;     ///< Sparse shape function matrix
        ChElementBrick::ShapeVector N;   ///< Dense shape function vector
        ChElementBrick::ShapeVector Nx;  ///< Dense shape function vector, X derivative
        ChElementBrick::ShapeVector Ny;  ///< Dense shape function vector, Y derivative
        ChElementBrick::ShapeVector Nz;  ///< Dense shape function vector, Z derivative
        ChVector<> gacc;                 ///< gravitational acceleration

        virtual void Evaluate(ChVectorN<double, 24>& result,
                              const double x,
                              const double y,
                              const double z) override;
    };

    /// Get number of nodes of this element
    virtual int GetNnodes() override { return 8; }

    /// Get number of degrees of freedom of this element
    virtual int GetNdofs() override { return 8 * 3; }

    /// Get the number of coordinates from the n-th node used by this element.
    virtual int GetNodeNdofs(int n) override { return 3; }

    /// Access the n-th node of this element.
    virtual std::shared_ptr<ChNodeFEAbase> GetNodeN(int n) override { return m_nodes[n]; }

    /// Specify the nodes of this element.
    void SetNodes(std::shared_ptr<ChNodeFEAxyz> nodeA,
                  std::shared_ptr<ChNodeFEAxyz> nodeB,
                  std::shared_ptr<ChNodeFEAxyz> nodeC,
                  std::shared_ptr<ChNodeFEAxyz> nodeD,
                  std::shared_ptr<ChNodeFEAxyz> nodeE,
                  std::shared_ptr<ChNodeFEAxyz> nodeF,
                  std::shared_ptr<ChNodeFEAxyz> nodeG,
                  std::shared_ptr<ChNodeFEAxyz> nodeH);

    /// Set the element number.
    void SetElemNum(int kb) { m_elementnumber = kb; }

    /// Set EAS internal parameters (stored values).
    void
    SetStockAlpha(double a1, double a2, double a3, double a4, double a5, double a6, double a7, double a8, double a9);

    /// Set EAS Jacobian matrix.
    void SetStockJac(const ChMatrixNM<double, 24, 24>& a) { m_stock_jac_EAS = a; }

    /// Set Analytical Jacobian.
    void SetStockKTE(const ChMatrixNM<double, 24, 24>& a) { m_stock_KTE = a; }

    /// Set some element parameters (dimensions).
    void SetInertFlexVec(const ChVectorN<double, 3>& a) { m_InertFlexVec = a; }

    int GetElemNum() const { return m_elementnumber; }

    /// Get initial position of the element in matrix form
    const ChMatrixNM<double, 8, 3>& GetInitialPos() const { return m_d0; }

    std::shared_ptr<ChNodeFEAxyz> GetNodeA() const { return m_nodes[0]; }
    std::shared_ptr<ChNodeFEAxyz> GetNodeB() const { return m_nodes[1]; }
    std::shared_ptr<ChNodeFEAxyz> GetNodeC() const { return m_nodes[2]; }
    std::shared_ptr<ChNodeFEAxyz> GetNodeD() const { return m_nodes[3]; }
    std::shared_ptr<ChNodeFEAxyz> GetNodeE() const { return m_nodes[4]; }
    std::shared_ptr<ChNodeFEAxyz> GetNodeF() const { return m_nodes[5]; }
    std::shared_ptr<ChNodeFEAxyz> GetNodeG() const { return m_nodes[6]; }
    std::shared_ptr<ChNodeFEAxyz> GetNodeH() const { return m_nodes[7]; }

    double GetLengthX() const { return m_InertFlexVec(0); }
    double GetLengthY() const { return m_InertFlexVec(1); }
    double GetLengthZ() const { return m_InertFlexVec(2); }

    void SetMaterial(std::shared_ptr<ChContinuumElastic> my_material) { m_Material = my_material; }
    std::shared_ptr<ChContinuumElastic> GetMaterial() const { return m_Material; }
    /// Turn gravity on/off.
    void SetGravityOn(bool val) { m_gravity_on = val; }
    /// Set whether material is Mooney-Rivlin (Otherwise linear elastic isotropic)
    void SetMooneyRivlin(bool val) { m_isMooney = val; }
    /// Set Mooney-Rivlin coefficients
    void SetMRCoefficients(double C1, double C2) {
        CCOM1 = C1;
        CCOM2 = C2;
    }
    /// Fills the N shape function matrix
    /// as  N = [s1*eye(3) s2*eye(3) s3*eye(3) s4*eye(3)...]; ,
    void ShapeFunctions(ShapeVector& N, double x, double y, double z);
    void ShapeFunctionsDerivativeX(ShapeVector& Nx, double x, double y, double z);
    void ShapeFunctionsDerivativeY(ShapeVector& Ny, double x, double y, double z);
    void ShapeFunctionsDerivativeZ(ShapeVector& Nz, double x, double y, double z);
    // Functions for ChLoadable interface
    /// Gets the number of DOFs affected by this element (position part)
    virtual int LoadableGet_ndof_x() override { return 8 * 3; }

    /// Gets the number of DOFs affected by this element (speed part)
    virtual int LoadableGet_ndof_w() override { return 8 * 3; }

    /// Gets all the DOFs packed in a single vector (position part)
    virtual void LoadableGetStateBlock_x(int block_offset, ChState& mD) override {
        mD.segment(block_offset + 0, 3) = m_nodes[0]->GetPos().eigen();
        mD.segment(block_offset + 3, 3) = m_nodes[1]->GetPos().eigen();
        mD.segment(block_offset + 6, 3) = m_nodes[2]->GetPos().eigen();
        mD.segment(block_offset + 9, 3) = m_nodes[3]->GetPos().eigen();
        mD.segment(block_offset + 12, 3) = m_nodes[4]->GetPos().eigen();
        mD.segment(block_offset + 15, 3) = m_nodes[5]->GetPos().eigen();
        mD.segment(block_offset + 18, 3) = m_nodes[6]->GetPos().eigen();
        mD.segment(block_offset + 21, 3) = m_nodes[7]->GetPos().eigen();
    }

    /// Gets all the DOFs packed in a single vector (speed part)
    virtual void LoadableGetStateBlock_w(int block_offset, ChStateDelta& mD) override {
        mD.segment(block_offset + 0, 3) = m_nodes[0]->GetPos_dt().eigen();
        mD.segment(block_offset + 3, 3) = m_nodes[1]->GetPos_dt().eigen();
        mD.segment(block_offset + 6, 3) = m_nodes[2]->GetPos_dt().eigen();
        mD.segment(block_offset + 9, 3) = m_nodes[3]->GetPos_dt().eigen();
        mD.segment(block_offset + 12, 3) = m_nodes[4]->GetPos_dt().eigen();
        mD.segment(block_offset + 15, 3) = m_nodes[5]->GetPos_dt().eigen();
        mD.segment(block_offset + 18, 3) = m_nodes[6]->GetPos_dt().eigen();
        mD.segment(block_offset + 21, 3) = m_nodes[7]->GetPos_dt().eigen();
    }

    /// Increment all DOFs using a delta.
    virtual void LoadableStateIncrement(const unsigned int off_x, ChState& x_new, const ChState& x, const unsigned int off_v, const ChStateDelta& Dv) override {
        for (int i= 0; i<8; ++i) {
            this->m_nodes[i]->NodeIntStateIncrement(off_x+3*1 , x_new, x, off_v+3*i , Dv);
        }
    }


    /// Number of coordinates in the interpolated field: here the {x,y,z} displacement
    virtual int Get_field_ncoords() override { return 3; }

    /// Tell the number of DOFs blocks (ex. =1 for a body, =4 for a tetrahedron, etc.)
    virtual int GetSubBlocks() override { return 8; }

    /// Get the offset of the i-th sub-block of DOFs in global vector
    virtual unsigned int GetSubBlockOffset(int nblock) override { return m_nodes[nblock]->NodeGetOffset_w(); }

    /// Get the size of the i-th sub-block of DOFs in global vector
    virtual unsigned int GetSubBlockSize(int nblock) override { return 3; }

    /// Get the pointers to the contained ChVariables, appending to the mvars vector.
    virtual void LoadableGetVariables(std::vector<ChVariables*>& mvars) override {
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
                           ) override {
        // this->ComputeNF(U, V, Qi, detJ, F, state_x, state_w);
        ShapeVector N;
        ShapeVector Nx;
        ShapeVector Ny;
        ShapeVector Nz;
        ShapeFunctions(N, U, V, W);  // evaluate shape functions (in compressed vector)
        ShapeFunctionsDerivativeX(Nx, U, V, W);
        ShapeFunctionsDerivativeY(Ny, U, V, W);
        ShapeFunctionsDerivativeZ(Nz, U, V, W);

        ChMatrixNM<double, 1, 3> Nx_d0 = Nx * m_d0;
        ChMatrixNM<double, 1, 3> Ny_d0 = Ny * m_d0;
        ChMatrixNM<double, 1, 3> Nz_d0 = Nz * m_d0;

        ChMatrixNM<double, 3, 3> rd0;
        rd0.col(0) = m_d0.transpose() * Nx.transpose();
        rd0.col(1) = m_d0.transpose() * Ny.transpose();
        rd0.col(2) = m_d0.transpose() * Nz.transpose();
        detJ = rd0.determinant();
        detJ *= this->GetLengthX() * this->GetLengthY() * this->GetLengthZ() / 8.0;

        for (int i = 0; i < 8; i++) {
            Qi.segment(3 * i, 3) = N(i) * F.segment(0, 3);
        }
    }

    /// This is needed so that it can be accessed by ChLoaderVolumeGravity
    virtual double GetDensity() override { return this->m_Material->Get_density(); }

  private:
    enum JacobianType { ANALYTICAL, NUMERICAL };

    // Private Data
    std::vector<std::shared_ptr<ChNodeFEAxyz> > m_nodes;  ///< Element nodes

    double m_thickness;
    std::shared_ptr<ChContinuumElastic> m_Material;  ///< Elastic Material

    ChMatrixNM<double, 24, 24> m_StiffnessMatrix;  ///< Stiffness matrix
    ChMatrixNM<double, 24, 24> m_MassMatrix;       ///< Mass matrix
    ChVectorN<double, 3> m_InertFlexVec;           ///< for element size (EL,EW,EH)
    // EAS
    int m_elementnumber;                         ///< Element number, for EAS
    ChMatrixNM<double, 24, 24> m_stock_jac_EAS;  ///< EAS Jacobian matrix
    ChVectorN<double, 9> m_stock_alpha_EAS;      ///< EAS previous step internal parameters
    ChMatrixNM<double, 24, 24> m_stock_KTE;      ///< Analytical Jacobian
    ChMatrixNM<double, 8, 3> m_d0;               ///< Initial Coordinate per element
    ChVectorN<double, 24> m_GravForce;           ///< Gravity Force
    JacobianType m_flag_HE;
    bool m_gravity_on;  ///< Flag indicating whether or not gravity is included
    bool m_isMooney;    ///< Flag indicating whether the material is Mooney Rivlin
    double CCOM1;       ///< First coefficient for Mooney-Rivlin
    double CCOM2;       ///< Second coefficient for Mooney-Rivlin
                        // Private Methods
    virtual void Update() override;

    /// Fills the D vector with the current field values at the nodes of the element, with proper ordering.
    /// If the D vector has not the size of this->GetNdofs(), it will be resized.
    ///  {x_a y_a z_a Dx_a Dx_a Dx_a x_b y_b z_b Dx_b Dy_b Dz_b}
    virtual void GetStateBlock(ChVectorDynamic<>& mD) override;

    /// Computes the STIFFNESS MATRIX of the element:
    /// K = integral( .... ),
    /// Note: in this 'basic' implementation, constant section and
    /// constant material are assumed
    void ComputeStiffnessMatrix();
    /// Computes the MASS MATRIX of the element
    /// Note: in this 'basic' implementation, constant section and
    /// constant material are assumed
    void ComputeMassMatrix();
    /// Compute the gravitational forces.
    void ComputeGravityForce(const ChVector<>& g_acc);
    /// Setup. Precompute mass and matrices that do not change during the
    /// simulation, ex. the mass matrix in ANCF is constant
    virtual void SetupInitial(ChSystem* system) override;
    /// Sets M as the global mass matrix.
    virtual void ComputeMmatrixGlobal(ChMatrixRef M) override { M = m_MassMatrix; }
    /// Sets H as the global stiffness matrix K, scaled  by Kfactor. Optionally, also
    /// superimposes global damping matrix R, scaled by Rfactor, and global mass matrix M multiplied by Mfactor.
    virtual void ComputeKRMmatricesGlobal(ChMatrixRef H,
                                          double Kfactor,
                                          double Rfactor = 0,
                                          double Mfactor = 0) override;

    /// Computes the internal forces (ex. the actual position of nodes is not in relaxed reference position) and set
    /// values in the Fi vector.
    virtual void ComputeInternalForces(ChVectorDynamic<>& Fi) override;

    // [EAS] matrix T0 (inverse and transposed) and detJ0 at center are used for Enhanced Assumed Strains alpha
    void T0DetJElementCenterForEAS(ChMatrixNM<double, 8, 3>& d0, ChMatrixNM<double, 6, 6>& T0, double& detJ0C);
    // [EAS] Basis function of M for Enhanced Assumed Strain
    void Basis_M(ChMatrixNM<double, 6, 9>& M, double x, double y, double z);

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

/// @} fea_elements

}  // end namespace fea
}  // end namespace chrono

#endif
