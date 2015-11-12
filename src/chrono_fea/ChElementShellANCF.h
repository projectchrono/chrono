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
// ANCF laminated shell element with four nodes.
// =============================================================================

#ifndef CHELEMENTSHELLANCF_H
#define CHELEMENTSHELLANCF_H

#include "chrono/physics/ChContinuumMaterial.h"
#include "core/ChQuadrature.h"
#include "chrono_fea/ChApiFEA.h"
#include "chrono_fea/ChElementShell.h"
#include "chrono_fea/ChNodeFEAxyzD.h"
#include "chrono_fea/ChUtilsFEA.h"

namespace chrono {
namespace fea {

/// ANCF laminated shell element with four nodes.
/// This class implements composite material elastic
/// force formulations
class ChApiFea ChElementShellANCF : public ChElementShell, public ChLoadableUV, public ChLoadableUVW {
  public:
    ChElementShellANCF();
    ~ChElementShellANCF() {}

    /// This class defines the calculations for the integrand of the inertia matrix.
    /// S = [N1*eye(3) N2*eye(3) N3*eye(3) N4*eye(3) N5*eye(3) N6*eye(3) N7*eye(3) N8*eye(3)];
    class MyMass : public ChIntegrable3D<ChMatrixNM<double, 24, 24> > {
      public:
        MyMass();  ///< Constructor
        MyMass(ChMatrixNM<double, 8, 3>* m_d0, ChElementShellANCF* element_) {
            d0 = m_d0;
            element = element_;
        }
        ~MyMass() {}

      private:
        ChElementShellANCF* element;
        ChMatrixNM<double, 8, 3>* d0;  ///< Pointer to initial coordinates
        ChMatrixNM<double, 3, 24> S;   ///< Sparse matrix with shape functions evaluated
        ChMatrixNM<double, 1, 8> N;    ///< Dense shape function vector
        ChMatrixNM<double, 1, 8> Nx;   ///< Dense shape function vector X derivative
        ChMatrixNM<double, 1, 8> Ny;   ///< Dense shape function vector Y derivative
        ChMatrixNM<double, 1, 8> Nz;   ///< Dense shape function vector Z derivative
        /// The Evaluate method calculates the integrand in the matrix result
        virtual void Evaluate(ChMatrixNM<double, 24, 24>& result, const double x, const double y, const double z);
    };
    /// This class defines the calculations for the integrand of the element gravity forces
    class MyGravity : public ChIntegrable3D<ChMatrixNM<double, 24, 1> > {
      public:
        MyGravity();
        MyGravity(ChMatrixNM<double, 8, 3>* m_d0, ChElementShellANCF* passedelement) {
            d0 = m_d0;
            element = passedelement;
        }
        ~MyGravity() {}

      private:
        ChElementShellANCF* element;
        ChMatrixNM<double, 8, 3>* d0;                ///< Pointer to initial coordinates
        ChMatrixNM<double, 1, 8> N;                  ///< Dense shape function vector
        ChMatrixNM<double, 1, 8> Nx;                 ///< Dense shape function vector X derivative
        ChMatrixNM<double, 1, 8> Ny;                 ///< Dense shape function vector Y derivative
        ChMatrixNM<double, 1, 8> Nz;                 ///< Dense shape function vector Z derivative
        ChMatrixNM<double, 3, 1> LocalGravityForce;  ///< Definition of the gravity vector (along Z by default)
        /// The Evaluate method calculates the integrand  of the gravity forces and stores it in result
        virtual void Evaluate(ChMatrixNM<double, 24, 1>& result, const double x, const double y, const double z);
    };

    /// This class defines the calculations for the integrand of the air pressure.
    /// This class is specially devised for the simulation of ANCF continuum-based tires
    class MyAirPressure : public ChIntegrable2D<ChMatrixNM<double, 24, 1> > {
      public:
        MyAirPressure();
        MyAirPressure(ChMatrixNM<double, 8, 3>* m_d0, ChMatrixNM<double, 8, 3>* d_, ChElementShellANCF* element_) {
            d0 = m_d0;
            d = d_;
            element = element_;
        }
        ~MyAirPressure() {}

      private:
        ChElementShellANCF* element;
        ChMatrixNM<double, 8, 3>* d0;               ///< Pointer to initial coordinates
        ChMatrixNM<double, 8, 3>* d;                ///< Pointer to current (this iteration) coordinates
        ChMatrixNM<double, 1, 4> S_ANS;             ///< Shape functions for Assumed Natural Strain
        ChMatrixNM<double, 1, 8> N;                 ///< Dense shape function vector
        ChMatrixNM<double, 1, 8> Nx;                ///< Dense shape function vector X derivative
        ChMatrixNM<double, 1, 8> Ny;                ///< Dense shape function vector Y derivative
        ChMatrixNM<double, 1, 8> Nz;                ///< Dense shape function vector Z derivative
        ChMatrixNM<double, 3, 1> LocalAirPressure;  ///< Local vector defining pressure
        /// The Evaluate method calculates the integrand  of the air pressure generalize force and stores it in result
        virtual void Evaluate(ChMatrixNM<double, 24, 1>& result, const double x, const double y);
    };
    /// This class defines the calculations for the integrand of shell element internal forces
    /// Capabilities of this class include: Application of enhanced assumed strain and assumed natural
    /// strain formulations to avoid thickness and (tranvese and in-plane) shear locking. This implementation
    /// also features a composite material implementation that allows for selecting a number of layers over the
    /// element thickness; each of which has an independent, user-selected fiber angle (direction for orthotropic
    /// constitutive
    /// behavior)
    class MyForce : public ChIntegrable3D<ChMatrixNM<double, 750, 1> > {
      public:
        MyForce();
        // Constructor
        MyForce(
            ChMatrixNM<double, 8, 3>* d_,             ///< Pointer to current (this iteration) coordinates
            ChMatrixNM<double, 24, 1>* d_dt_,         ///< Pointer to current (this iteration) generalized velocities
            ChMatrixNM<double, 8, 1>* strain_ans_,    ///< Vector for assumed natural strain
            ChMatrixNM<double, 8, 24>* strainD_ans_,  ///< Matrix for Jacobian of assumed natural strain
            ChMatrixNM<double, 8, 3>* m_d0,           ///< Pointer to initial coordinates
            ChMatrixNM<double, 6, 6>* E_eps_,         ///< Pointer to matrix of elastic coefficients (Orthotropic style)
            ChElementShellANCF* element_,             ///< Pointer to this element
            ChMatrixNM<double, 6, 6>*
                T0_,  ///< Pointer to transformation matrix, function of fiber angle (see Yamashita et al, 2015, JCND)
            double* detJ0C_,  ///< Determinant of the position vector gradient of initial configuration evaluated at the
            /// element center
            double* theta_,  ///< Fiber angle (user input in degrees)
            ChMatrixNM<double, 5, 1>*
                alpha_eas_)  ///< Pointer to the vector of internal parameters for Enhanced Assumed Strain formulation
        {
            d = d_;
            d_dt = d_dt_;
            d0 = m_d0;
            strain_ans = strain_ans_;
            strainD_ans = strainD_ans_;
            element = element_;
            E_eps = E_eps_;
            T0 = T0_;
            detJ0C = detJ0C_;
            theta = theta_;
            alpha_eas = alpha_eas_;
        }
        ~MyForce() {}

      private:
        ChElementShellANCF* element;
        //// External values
        ChMatrixNM<double, 8, 3>* d;
        ChMatrixNM<double, 8, 1>* strain_ans;
        ChMatrixNM<double, 8, 24>* strainD_ans;
        ChMatrixNM<double, 8, 3>* d0;
        ChMatrixNM<double, 24, 1>* d_dt;
        ChMatrixNM<double, 6, 6>* T0;
        ChMatrixNM<double, 5, 1>* alpha_eas;
        ChMatrixNM<double, 6, 6>* E_eps;
        double* detJ0C;
        double* theta;

        ChMatrixNM<double, 24, 1> Fint;    ///< Internal force vector, added to the equations
        ChMatrixNM<double, 24, 24> JAC11;  ///< Jacobian of element elastic forces for implicit numerical integration
        ChMatrixNM<double, 9, 24> Gd;  ///< Jacobian (w.r.t. coordinates) of the initial position vector gradient matrix
        ChMatrixNM<double, 6, 1> stress;  ///< Stress vector: (*)E_eps*strain
        ChMatrixNM<double, 9, 9> Sigm;    ///< Rearrangement of stress vector (not always needed)
        ChMatrixNM<double, 24, 6>
            temp246;  ///< Temporary matrix for the calculation of JAC11 (Jacobian of element elastic forces)
        ChMatrixNM<double, 24, 9>
            temp249;  ///< Temporary matrix for the calculation of JAC11 (Jacobian of element elastic forces)
        ChMatrixNM<double, 1, 8> Nx;          ///< Dense shape function vector X derivative
        ChMatrixNM<double, 1, 8> Ny;          ///< Dense shape function vector Y derivative
        ChMatrixNM<double, 1, 8> Nz;          ///< Dense shape function vector Z derivative
        ChMatrixNM<double, 6, 24> strainD;    ///< Derivative of the strains w.r.t. the coordinates. Includes orthotropy
        ChMatrixNM<double, 6, 1> strain;      ///< Vector of strains
        ChMatrixNM<double, 8, 8> d_d;         ///< d*d' matrix, where d contains current coordinates in matrix form
        ChMatrixNM<double, 8, 1> ddNx;        ///< d_d*Nx' matrix
        ChMatrixNM<double, 8, 1> ddNy;        ///< d_d*Ny' matrix
        ChMatrixNM<double, 8, 1> ddNz;        ///< d_d*Nz' matrix
        ChMatrixNM<double, 8, 8> d0_d0;       ///< d0*d0' matrix, where d0 contains initial coordinates in matrix form
        ChMatrixNM<double, 8, 1> d0d0Nx;      ///< d0_d0*Nx' matrix
        ChMatrixNM<double, 8, 1> d0d0Ny;      ///< d0_d0*Ny' matrix
        ChMatrixNM<double, 8, 1> d0d0Nz;      ///< d0_d0*Nz' matrix
        ChMatrixNM<double, 1, 1> tempA;       ///< Contains temporary strains for ANS
        ChMatrixNM<double, 1, 1> tempA1;      ///< Contains temporary strains for ANS
        ChMatrixNM<double, 1, 24> tempB;      ///< Temporary matrix to calculate strainD
        ChMatrixNM<double, 24, 6> tempC;      ///< Temporary matrix to compute internal forces Fint
        double detJ0;                         ///< Determinant of the initial position vector gradient matrix
        double alphaHHT;                      ///< Hard-coded damping coefficient for structural dissipation
        double betaHHT;                       ///< HHT coefficient for structural damping
        double gammaHHT;                      ///< HHT coefficient for structural damping
        ChMatrixNM<double, 1, 8> N;           ///< Shape function vector
        ChMatrixNM<double, 1, 4> S_ANS;       ///< Shape function vector for Assumed Natural Strain
        ChMatrixNM<double, 1, 24> tempBB;     ///< Temporary matrix used to calculate strainD
        ChMatrixNM<double, 6, 5> M;           ///< Shape function vector for Enhanced Assumed Strain
        ChMatrixNM<double, 6, 5> G;           ///< Matrix G interpolates the internal parameters of EAS
        ChMatrixNM<double, 5, 6> GT;          ///< Tranpose of matrix GT
        ChMatrixNM<double, 6, 1> strain_EAS;  ///< Enhanced assumed strain vector

        /// Evaluate (strainD'*strain)  at point x, include ANS and EAS.
        virtual void Evaluate(ChMatrixNM<double, 750, 1>& result, const double x, const double y, const double z);
    };
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

    /// Calculate the first part of the Jacobian of the elastic forces of the ANCF shell element. It exploits sparsity
    /// to speed up computations
    void JacCalcUnrolled(
        const ChMatrixNM<double, 6, 1>& stress,  ///< [in] Stress vector
        const ChMatrixNM<double, 9, 24>& Gd,     ///< [in] Matrix obtained from intermediate step differentiation
        ChMatrixNM<double, 24, 24>& JAC11);      ///< [out] Partial Jacobian computed
    /// Set the storage of the five alpha parameters for EAS (max no. of layers 7)
    void SetStockAlpha(const ChMatrixNM<double, 35, 1>& a) { m_StockAlpha_EAS = a; }
    /// Set all the alpha parameters for EAS
    const ChMatrixNM<double, 35, 1>& GetStockAlpha() const { return m_StockAlpha_EAS; }
    /// Set Jacobian of EAS
    void SetStockJac(const ChMatrixNM<double, 24, 24>& a) { m_stock_jac_EAS = a; }
    /// Set Jacobian
    void SetStockKTE(const ChMatrixNM<double, 24, 24>& a) { m_stock_KTE = a; }
    /// Set element properties for all layers: Elastic parameters, dimensions, etc.
    void SetInertFlexVec(const ChMatrixNM<double, 98, 1>& a) { m_InertFlexVec = a; }
    /// Get element properties for all layers: Elastic parameters, dimensions, etc.
    const ChMatrixNM<double, 98, 1>& GetInertFlexVec() const { return m_InertFlexVec; }
    /// Set Gauss range for laminated shell based on layer thickness
    void SetGaussZRange(const ChMatrixNM<double, 7, 2>& a) { m_GaussZRange = a; }
    /// Get Gauss range for laminated shell based on layer thickness
    const ChMatrixNM<double, 7, 2>& GetGaussZRange() const { return m_GaussZRange; }
    /// Set the step size used in calculating the structural damping coefficient.
    void Setdt(double a) { m_dt = a; }
    /// Turn gravity on/off.
    void SetGravityOn(bool val) { m_gravity_on = val; }
    /// Turn air pressure on/off.
    void SetAirPressureOn(bool val) { m_air_pressure_on = val; }
    /// Set the structural damping.
    void SetAlphaDamp(double a) { m_Alpha = a; }
    /// Get the element length in the X direction.Each layer has the same element length
    double GetLengthX() const { return m_InertFlexVec(1); }
    /// Get the element length in the Y direction.Each layer has the same element length
    double GetLengthY() const { return m_InertFlexVec(2); }

  private:
    enum JacobianType { ANALYTICAL, NUMERICAL };

    std::vector<ChSharedPtr<ChNodeFEAxyzD> > m_nodes;  ///< element nodes

    double m_thickness;
    int m_element_number;                          ///< element number (for EAS)
    double m_Alpha;                                ///< structural damping
    ChSharedPtr<ChContinuumElastic> m_Material;    ///< elastic material
    ChMatrixNM<double, 24, 24> m_StiffnessMatrix;  ///< stiffness matrix
    ChMatrixNM<double, 24, 24> m_MassMatrix;       ///< mass matrix
    ChMatrixNM<double, 24, 24> m_stock_jac_EAS;    ///< EAS per element
    ChMatrixNM<double, 24, 24> m_stock_KTE;        ///< Analytical Jacobian
    ChMatrixNM<double, 8, 3> m_d0;                 ///< initial nodal coordinates
    ChMatrixNM<double, 24, 1> m_GravForce;         ///< Gravity Force
    // Material Properties for orthotropic per element (14x7) Max #layer is 7
    ChMatrixNM<double, 98, 1> m_InertFlexVec;    ///< Contains element's parameters
    ChMatrixNM<double, 35, 1> m_StockAlpha_EAS;  ///< StockAlpha(5*7,1): Max #Layer is 7
    ChMatrixNM<double, 7, 2> m_GaussZRange;      ///< StockAlpha(7,2): Max #Layer is 7 (-1 < GaussZ < 1)
    int m_numLayers;                             ///< number of layers for this element
    JacobianType m_flag_HE;                      ///< Jacobian evaluation type (analytical or numerical)
    double m_dt;                                 ///< time step used in calculating structural damping coefficient
    bool m_gravity_on;                           ///< flag indicating whether or not gravity is included
    bool m_air_pressure_on;                      ///< flag indicating whether or not air pressure is included

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

    //
    // Functions for ChLoadable interface
    //

    /// Gets the number of DOFs affected by this element (position part)
    virtual int LoadableGet_ndof_x() { return 4 * 6; }

    /// Gets the number of DOFs affected by this element (speed part)
    virtual int LoadableGet_ndof_w() { return 4 * 6; }

    /// Gets all the DOFs packed in a single vector (position part)
    virtual void LoadableGetStateBlock_x(int block_offset, ChVectorDynamic<>& mD) {
        mD.PasteVector(this->m_nodes[0]->GetPos(), block_offset, 0);
        mD.PasteVector(this->m_nodes[0]->GetD(), block_offset + 3, 0);
        mD.PasteVector(this->m_nodes[1]->GetPos(), block_offset + 6, 0);
        mD.PasteVector(this->m_nodes[1]->GetD(), block_offset + 9, 0);
        mD.PasteVector(this->m_nodes[2]->GetPos(), block_offset + 12, 0);
        mD.PasteVector(this->m_nodes[2]->GetD(), block_offset + 15, 0);
        mD.PasteVector(this->m_nodes[3]->GetPos(), block_offset + 18, 0);
        mD.PasteVector(this->m_nodes[3]->GetD(), block_offset + 21, 0);
    }

    /// Gets all the DOFs packed in a single vector (speed part)
    virtual void LoadableGetStateBlock_w(int block_offset, ChVectorDynamic<>& mD) {
        mD.PasteVector(this->m_nodes[0]->GetPos_dt(), block_offset, 0);
        mD.PasteVector(this->m_nodes[0]->GetD_dt(), block_offset + 3, 0);
        mD.PasteVector(this->m_nodes[1]->GetPos_dt(), block_offset + 6, 0);
        mD.PasteVector(this->m_nodes[1]->GetD_dt(), block_offset + 9, 0);
        mD.PasteVector(this->m_nodes[2]->GetPos_dt(), block_offset + 12, 0);
        mD.PasteVector(this->m_nodes[2]->GetD_dt(), block_offset + 15, 0);
        mD.PasteVector(this->m_nodes[3]->GetPos_dt(), block_offset + 18, 0);
        mD.PasteVector(this->m_nodes[3]->GetD_dt(), block_offset + 21, 0);
    }

    /// Number of coordinates in the interpolated field, ex=3 for a
    /// tetrahedron finite element or a cable, = 1 for a thermal problem, etc.
    virtual int Get_field_ncoords() { return 6; }

    /// Tell the number of DOFs blocks (ex. =1 for a body, =4 for a tetrahedron, etc.)
    virtual int GetSubBlocks() { return 4; }

    /// Get the offset of the i-th sub-block of DOFs in global vector
    virtual unsigned int GetSubBlockOffset(int nblock) { return m_nodes[nblock]->NodeGetOffset_w(); }

    /// Get the size of the i-th sub-block of DOFs in global vector
    virtual unsigned int GetSubBlockSize(int nblock) { return 6; }

    /// Get the pointers to the contained ChLcpVariables, appending to the mvars vector.
    virtual void LoadableGetVariables(std::vector<ChLcpVariables*>& mvars) {
        for (int i = 0; i < m_nodes.size(); ++i) {
            mvars.push_back(&this->m_nodes[i]->Variables());
            mvars.push_back(&this->m_nodes[i]->Variables_D());
        }
    };

    /// Evaluate N'*F , where N is some type of shape function
    /// evaluated at U,V coordinates of the surface, each ranging in -1..+1
    /// F is a load, N'*F is the resulting generalized load
    /// Returns also det[J] with J=[dx/du,..], that might be useful in gauss quadrature.
    virtual void ComputeNF(const double U,              ///< parametric coordinate in surface
                           const double V,              ///< parametric coordinate in surface
                           ChVectorDynamic<>& Qi,       ///< Return result of Q = N'*F  here
                           double& detJ,                ///< Return det[J] here
                           const ChVectorDynamic<>& F,  ///< Input F vector, size is =n. field coords.
                           ChVectorDynamic<>* state_x,  ///< if != 0, update state (pos. part) to this, then evaluate Q
                           ChVectorDynamic<>* state_w   ///< if != 0, update state (speed part) to this, then evaluate Q
                           ) {
        ChMatrixNM<double, 1, 8> N;
        this->ShapeFunctions(N, U, V,
                             0);  // evaluate shape functions (in compressed vector), btw. not dependant on state

        detJ = GetLengthX() * GetLengthY() /
               4.0;  // ***TODO***  compute exact determinant of jacobian at U,V; approx. is area/4..

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
        this->ComputeNF(U, V, Qi, detJ, F, state_x, state_w);
        detJ /= 2.0;  // because UV surface interpreted as volume, cut the effect of integration on -1...+1 on normal
    }

    /// This is needed so that it can be accessed by ChLoaderVolumeGravity
    /// Density is mass per unit surface.
    virtual double GetDensity() {
        //***TODO*** check if the following is correct
        //***TODO*** performance improvement: loop on layers to accumulate tot_density
        //           could be at element initialization, and tot_density as aux.data in material
        double tot_density = 0;  // to acumulate kg/surface per all layers
        for (int kl = 0; kl < m_numLayers; kl++) {
            int ij = 14 * kl;
            double rho = m_InertFlexVec(ij);
            double layerthick = m_InertFlexVec(ij + 3);
            tot_density += rho * layerthick;
        }
        return tot_density;
    }

    /// Gets the normal to the surface at the parametric coordinate U,V.
    /// Each coordinate ranging in -1..+1.
    virtual ChVector<> ComputeNormal(const double U, const double V) {
        //***TODO*** compute normal at precise U,V coordinate,
        //           ex.using shape function derivatives.
        ChVector<> mnorm(Vcross(GetNodeB()->pos - GetNodeA()->pos, GetNodeD()->pos - GetNodeA()->pos));
        return mnorm;
    }
};

}  // end of namespace fea
}  // end of namespace chrono

#endif
