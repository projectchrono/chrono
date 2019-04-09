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
        ChMatrixNM<double, 8, 3>* d0;  ///< Pointer to a matrix containing the element initial coordinates
        ChMatrixNM<double, 3, 24> S;   ///< Sparse shape function matrix
        ChMatrixNM<double, 3, 24> Sx;  ///< Sparse shape function matrix, X derivative
        ChMatrixNM<double, 3, 24> Sy;  ///< Sparse shape function matrix, Y derivative
        ChMatrixNM<double, 3, 24> Sz;  ///< Sparse shape function matrix, Z derivative
        ChMatrixNM<double, 1, 8> N;    ///< Dense shape function vector
        ChMatrixNM<double, 1, 8> Nx;   ///< Dense shape function vector, X derivative
        ChMatrixNM<double, 1, 8> Ny;   ///< Dense shape function vector, Y derivative
        ChMatrixNM<double, 1, 8> Nz;   ///< Dense shape function vector, Z derivative

        /// Evaluate the S'*S  at point x
        virtual void Evaluate(ChMatrixNM<double, 24, 24>& result,
                              const double x,
                              const double y,
                              const double z) override;
    };

    /// Internal force, EAS stiffness, and analytical jacobian are calculated
    class MyForceAnalytical : public ChIntegrable3D<ChMatrixNM<double, 906, 1> > {
      public:
        MyForceAnalytical() {}
        /// Constructor 1
        MyForceAnalytical(ChMatrixNM<double, 8, 3>* d_,
                          ChMatrixNM<double, 8, 3>* m_d0_,
                          ChElementBrick* element_,
                          ChMatrixNM<double, 6, 6>* T0_,
                          double* detJ0C_,
                          ChMatrixNM<double, 9, 1>* alpha_eas_) {
            d = d_;
            d0 = m_d0_;
            element = element_;
            T0 = T0_;
            detJ0C = detJ0C_;
            alpha_eas = alpha_eas_;

            E_eps.Reset();
            Gd.Reset();
            Sigm.Reset();
        }
        /// Constructor 2
        MyForceAnalytical(ChMatrixNM<double, 8, 3>* d_,
                          ChMatrixNM<double, 8, 3>* m_d0_,
                          ChElementBrick* element_,
                          ChMatrixNM<double, 6, 6>* T0_,
                          double* detJ0C_,
                          ChMatrixNM<double, 9, 1>* alpha_eas_,
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

            E_eps.Reset();
            Gd.Reset();
            Sigm.Reset();
        }
        ~MyForceAnalytical() {}

      private:
        ChElementBrick* element;
        ChMatrixNM<double, 8, 3>* d;          ///< Pointer to a matrix containing the element coordinates
        ChMatrixNM<double, 8, 3>* d0;         ///< Pointer to a matrix containing the element initial coordinates
        ChMatrixNM<double, 6, 6>* T0;         ///< Pointer to transformation matrix for Enhanced Assumed Strain (EAS)
        ChMatrixNM<double, 9, 1>* alpha_eas;  ///< Pointer to the 9 internal parameters for EAS
        double* detJ0C;                       ///< Pointer to determinant of the initial Jacobian at the element center
        double* E;                            ///< Pointer to Young modulus
        double* v;                            ///< Pointer to Poisson ratio

        ChMatrixNM<double, 24, 1> Fint;    ///< Generalized internal (elastic) force vector
        ChMatrixNM<double, 24, 24> JAC11;  ///< Jacobian of internal forces for implicit numerical integration
        ChMatrixNM<double, 9, 24> Gd;  ///< Jacobian (w.r.t. coordinates) of the initial position vector gradient matrix
        ChMatrixNM<double, 6, 1> stress;    ///< stress tensor in vector form
        ChMatrixNM<double, 9, 9> Sigm;      ///< stress tensor in sparse form
        ChMatrixNM<double, 24, 6> temp246;  ///< Temporary matrix for Jacobian (JAC11) calculation
        ChMatrixNM<double, 24, 9> temp249;  ///< Temporary matrix for Jacobian (JAC11) calculation
        ChMatrixNM<double, 6, 6> E_eps;     ///< Matrix of elastic coefficients (features orthotropy)
        ChMatrixNM<double, 3, 24> Sx;       ///< Sparse shape function matrix, X derivative
        ChMatrixNM<double, 3, 24> Sy;       ///< Sparse shape function matrix, Y derivative
        ChMatrixNM<double, 3, 24> Sz;       ///< Sparse shape function matrix, Z derivative
        ChMatrixNM<double, 1, 8> Nx;        ///< Dense shape function vector, X derivative
        ChMatrixNM<double, 1, 8> Ny;        ///< Dense shape function vector, Y derivative
        ChMatrixNM<double, 1, 8> Nz;        ///< Dense shape function vector, Z derivative
        ChMatrixNM<double, 6, 24> strainD;  ///< Derivative of the strains w.r.t. the coordinates. Includes orthotropy
        ChMatrixNM<double, 6, 1> strain;    ///< Vector of strains
        ChMatrixNM<double, 8, 8> d_d;       ///< d*d' matrix, where d contains current coordinates in matrix form
        ChMatrixNM<double, 8, 1> ddNx;      ///< d_d*Nx' matrix
        ChMatrixNM<double, 8, 1> ddNy;      ///< d_d*Ny' matrix
        ChMatrixNM<double, 8, 1> ddNz;      ///< d_d*Nz' matrix
        ChMatrixNM<double, 1, 1> tempA;     ///< Contains temporary strains
        ChMatrixNM<double, 1, 24> tempB;    ///< Contains temporary strain derivatives
        ChMatrixNM<double, 24, 6> tempC;    ///< Used to calculate the internal forces Fint
        ChMatrixNM<double, 1, 1> tempA1;    ///< Contains temporary strains
        ChMatrixNM<double, 8, 8> d0_d0;     ///< d0*d0' matrix, where d0 contains initial coordinates in matrix form
        ChMatrixNM<double, 8, 1> d0d0Nx;    ///< d0_d0*Nx' matrix
        ChMatrixNM<double, 8, 1> d0d0Ny;    ///< d0_d0*Ny' matrix
        ChMatrixNM<double, 8, 1> d0d0Nz;    ///< d0_d0*Nz' matrix
        double detJ0;                       ///< Determinant of the initial position vector gradient matrix
        // EAS
        ChMatrixNM<double, 6, 9> M;           ///< Shape function matrix for Enhanced Assumed Strain
        ChMatrixNM<double, 6, 9> G;           ///< Matrix G interpolates the internal parameters of EAS
        ChMatrixNM<double, 9, 6> GT;          ///< Tranpose of matrix GT
        ChMatrixNM<double, 6, 1> strain_EAS;  ///< Enhanced assumed strain vector

        /// Evaluate (strainD'*strain)  at a point
        virtual void Evaluate(ChMatrixNM<double, 906, 1>& result,
                              const double x,
                              const double y,
                              const double z) override;
    };

    class MyForceNum : public ChIntegrable3D<ChMatrixNM<double, 330, 1> > {
      public:
        MyForceNum();
        /// Constructor 1
        MyForceNum(ChMatrixNM<double, 8, 3>* d_,
                   ChMatrixNM<double, 8, 3>* m_d0_,
                   ChElementBrick* element_,
                   ChMatrixNM<double, 6, 6>* T0_,
                   double* detJ0C_,
                   ChMatrixNM<double, 9, 1>* alpha_eas_) {
            d = d_;
            d0 = m_d0_;
            element = element_;
            T0 = T0_;
            detJ0C = detJ0C_;
            alpha_eas = alpha_eas_;
        }
        /// Constructor 2
        MyForceNum(ChMatrixNM<double, 8, 3>* d_,
                   ChMatrixNM<double, 8, 3>* m_d0_,
                   ChElementBrick* element_,
                   ChMatrixNM<double, 6, 6>* T0_,
                   double* detJ0C_,
                   ChMatrixNM<double, 9, 1>* alpha_eas_,
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
        }
        ~MyForceNum() {}

      private:
        ChElementBrick* element;
        /// Pointers used for external values
        ChMatrixNM<double, 8, 3>* d;          ///< Pointer to a matrix containing the element coordinates
        ChMatrixNM<double, 8, 3>* d0;         ///< Pointer to a matrix containing the element initial coordinates
        ChMatrixNM<double, 6, 6>* T0;         ///< Pointer to transformation matrix for Enhanced Assumed Strain (EAS)
        ChMatrixNM<double, 9, 1>* alpha_eas;  ///< Pointer to the 9 internal parameters for EAS
        double* detJ0C;                       ///< Pointer to determinant of the initial Jacobian at the element center
        double* E;                            ///< Pointer to Young modulus
        double* v;                            ///< Pointer to Poisson ratio
        ChMatrixNM<double, 24, 1> Fint;       ///< Generalized internal (elastic) force vector
        ChMatrixNM<double, 6, 6> E_eps;       ///< Matrix of elastic coefficients (features orthotropy)
        ChMatrixNM<double, 3, 24> Sx;         ///< Sparse shape function matrix, X derivative
        ChMatrixNM<double, 3, 24> Sy;         ///< Sparse shape function matrix, Y derivative
        ChMatrixNM<double, 3, 24> Sz;         ///< Sparse shape function matrix, Z derivative
        ChMatrixNM<double, 1, 8> Nx;          ///< Dense shape function vector, X derivative
        ChMatrixNM<double, 1, 8> Ny;          ///< Dense shape function vector, Y derivative
        ChMatrixNM<double, 1, 8> Nz;          ///< Dense shape function vector, Z derivative
        ChMatrixNM<double, 6, 24> strainD;    ///< Derivative of the strains w.r.t. the coordinates. Includes orthotropy
        ChMatrixNM<double, 6, 1> strain;      ///< Vector of strains
        ChMatrixNM<double, 8, 8> d_d;         ///< d*d' matrix, where d contains current coordinates in matrix form
        ChMatrixNM<double, 8, 1> ddNx;        ///< d_d*Nx' matrix
        ChMatrixNM<double, 8, 1> ddNy;        ///< d_d*Ny' matrix
        ChMatrixNM<double, 8, 1> ddNz;        ///< d_d*Nz' matrix
        ChMatrixNM<double, 1, 1> tempA;       ///< Contains temporary strains
        ChMatrixNM<double, 1, 24> tempB;      ///< Contains temporary strain derivatives
        ChMatrixNM<double, 24, 6> tempC;      ///< Used to calculate the internal forces Fint
        ChMatrixNM<double, 1, 1> tempA1;      ///< Contains temporary strains
        ChMatrixNM<double, 8, 8> d0_d0;       ///< d0*d0' matrix, where d0 contains initial coordinates in matrix form
        ChMatrixNM<double, 8, 1> d0d0Nx;      ///< d0_d0*Nx' matrix
        ChMatrixNM<double, 8, 1> d0d0Ny;      ///< d0_d0*Ny' matrix
        ChMatrixNM<double, 8, 1> d0d0Nz;      ///< d0_d0*Nz' matrix
        double detJ0;                         ///< Determinant of the initial position vector gradient matrix
        // EAS
        ChMatrixNM<double, 6, 9> M;           ///< Shape function matrix for Enhanced Assumed Strain
        ChMatrixNM<double, 6, 9> G;           ///< Matrix G interpolates the internal parameters of EAS
        ChMatrixNM<double, 9, 6> GT;          ///< Tranpose of matrix GT
        ChMatrixNM<double, 6, 1> strain_EAS;  ///< Enhanced assumed strain vector

        /// Gaussian integration to calculate internal forces and EAS matrices
        virtual void Evaluate(ChMatrixNM<double, 330, 1>& result,
                              const double x,
                              const double y,
                              const double z) override;
    };

    /// Class to calculate the gravity forces of a brick element
    class MyGravity : public ChIntegrable3D<ChMatrixNM<double, 24, 1> > {
      public:
        MyGravity(ChMatrixNM<double, 8, 3>* m_d0, ChElementBrick* element_, const ChVector<> g_acc)
            : d0(m_d0), element(element_), gacc(g_acc) {}
        ~MyGravity() {}

      private:
        ChElementBrick* element;
        ChMatrixNM<double, 8, 3>* d0;  ///< Pointer to a matrix containing the element initial coordinates
        ChMatrixNM<double, 3, 24> S;   ///< Sparse shape function matrix
        ChMatrixNM<double, 1, 8> N;    ///< Dense shape function vector
        ChMatrixNM<double, 1, 8> Nx;   ///< Dense shape function vector, X derivative
        ChMatrixNM<double, 1, 8> Ny;   ///< Dense shape function vector, Y derivative
        ChMatrixNM<double, 1, 8> Nz;   ///< Dense shape function vector, Z derivative
        ChVector<> gacc;               ///< gravitational acceleration

        virtual void Evaluate(ChMatrixNM<double, 24, 1>& result,
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
    void SetInertFlexVec(const ChMatrixNM<double, 3, 1>& a) { m_InertFlexVec = a; }

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
    void ShapeFunctions(ChMatrix<>& N, double x, double y, double z);
    void ShapeFunctionsDerivativeX(ChMatrix<>& Nx, double x, double y, double z);
    void ShapeFunctionsDerivativeY(ChMatrix<>& Ny, double x, double y, double z);
    void ShapeFunctionsDerivativeZ(ChMatrix<>& Nz, double x, double y, double z);
    // Functions for ChLoadable interface
    /// Gets the number of DOFs affected by this element (position part)
    virtual int LoadableGet_ndof_x() override { return 8 * 3; }

    /// Gets the number of DOFs affected by this element (speed part)
    virtual int LoadableGet_ndof_w() override { return 8 * 3; }

    /// Gets all the DOFs packed in a single vector (position part)
    virtual void LoadableGetStateBlock_x(int block_offset, ChState& mD) override {
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
    virtual void LoadableGetStateBlock_w(int block_offset, ChStateDelta& mD) override {
        mD.PasteVector(this->m_nodes[0]->GetPos_dt(), block_offset, 0);
        mD.PasteVector(this->m_nodes[1]->GetPos_dt(), block_offset + 3, 0);
        mD.PasteVector(this->m_nodes[2]->GetPos_dt(), block_offset + 6, 0);
        mD.PasteVector(this->m_nodes[3]->GetPos_dt(), block_offset + 9, 0);
        mD.PasteVector(this->m_nodes[4]->GetPos_dt(), block_offset + 12, 0);
        mD.PasteVector(this->m_nodes[5]->GetPos_dt(), block_offset + 15, 0);
        mD.PasteVector(this->m_nodes[6]->GetPos_dt(), block_offset + 18, 0);
        mD.PasteVector(this->m_nodes[7]->GetPos_dt(), block_offset + 21, 0);
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
        ChMatrixNM<double, 1, 8> N;
        ChMatrixNM<double, 1, 8> Nx;
        ChMatrixNM<double, 1, 8> Ny;
        ChMatrixNM<double, 1, 8> Nz;
        this->ShapeFunctions(N, U, V,
                             W);  // evaluate shape functions (in compressed vector), btw. not dependant on state
        this->ShapeFunctionsDerivativeX(Nx, U, V, W);
        this->ShapeFunctionsDerivativeY(Ny, U, V, W);
        this->ShapeFunctionsDerivativeZ(Nz, U, V, W);

        ChMatrixNM<double, 1, 3> Nx_d0;
        Nx_d0.MatrMultiply(Nx, m_d0);
        ChMatrixNM<double, 1, 3> Ny_d0;
        Ny_d0.MatrMultiply(Ny, m_d0);
        ChMatrixNM<double, 1, 3> Nz_d0;
        Nz_d0.MatrMultiply(Nz, m_d0);

        ChMatrixNM<double, 3, 3> rd0;
        rd0(0, 0) = Nx_d0(0, 0);
        rd0(1, 0) = Nx_d0(0, 1);
        rd0(2, 0) = Nx_d0(0, 2);
        rd0(0, 1) = Ny_d0(0, 0);
        rd0(1, 1) = Ny_d0(0, 1);
        rd0(2, 1) = Ny_d0(0, 2);
        rd0(0, 2) = Nz_d0(0, 0);
        rd0(1, 2) = Nz_d0(0, 1);
        rd0(2, 2) = Nz_d0(0, 2);
        detJ = rd0.Det();
        detJ *= this->GetLengthX() * this->GetLengthY() * this->GetLengthZ() / 8.0;
        ChVector<> tmp;
        ChVector<> Fv = F.ClipVector(0, 0);
        tmp = N(0) * Fv;
        Qi.PasteVector(tmp, 0, 0);
        tmp = N(1) * Fv;
        Qi.PasteVector(tmp, 3, 0);
        tmp = N(2) * Fv;
        Qi.PasteVector(tmp, 6, 0);
        tmp = N(3) * Fv;
        Qi.PasteVector(tmp, 9, 0);
        tmp = N(4) * Fv;
        Qi.PasteVector(tmp, 12, 0);
        tmp = N(5) * Fv;
        Qi.PasteVector(tmp, 15, 0);
        tmp = N(6) * Fv;
        Qi.PasteVector(tmp, 18, 0);
        tmp = N(7) * Fv;
        Qi.PasteVector(tmp, 21, 0);
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
    ChMatrixNM<double, 3, 1> m_InertFlexVec;       ///< for element size (EL,EW,EH)
    // EAS
    int m_elementnumber;                         ///< Element number, for EAS
    ChMatrixNM<double, 24, 24> m_stock_jac_EAS;  ///< EAS Jacobian matrix
    ChMatrixNM<double, 9, 1> m_stock_alpha_EAS;  ///< EAS previous step internal parameters
    ChMatrixNM<double, 24, 24> m_stock_KTE;      ///< Analytical Jacobian
    ChMatrixNM<double, 8, 3> m_d0;               ///< Initial Coordinate per element
    ChMatrixNM<double, 24, 1> m_GravForce;       ///< Gravity Force
    JacobianType m_flag_HE;
    bool m_gravity_on;  ///< Flag indicating whether or not gravity is included
    bool m_isMooney;    ///< Flag indicating whether the material is Mooney Rivlin
    double CCOM1;       ///< First coefficient for Mooney-Rivlin
    double CCOM2;       ///< Second coefficient for Mooney-Rivlin
                        // Private Methods
    virtual void Update() override;

    /// Fills the D vector (column matrix) with the current
    /// field values at the nodes of the element, with proper ordering.
    /// If the D vector has not the size of this->GetNdofs(), it will be resized.
    ///  {x_a y_a z_a Dx_a Dx_a Dx_a x_b y_b z_b Dx_b Dy_b Dz_b}
    virtual void GetStateBlock(ChMatrixDynamic<>& mD) override;

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
    virtual void ComputeMmatrixGlobal(ChMatrix<>& M) override { M = m_MassMatrix; }
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

    // [EAS] matrix T0 (inverse and transposed) and detJ0 at center are used for Enhanced Assumed Strains alpha
    void T0DetJElementCenterForEAS(ChMatrixNM<double, 8, 3>& d0, ChMatrixNM<double, 6, 6>& T0, double& detJ0C);
    // [EAS] Basis function of M for Enhanced Assumed Strain
    void Basis_M(ChMatrixNM<double, 6, 9>& M, double x, double y, double z);
};

/// @} fea_elements

}  // end namespace fea
}  // end namespace chrono

#endif
