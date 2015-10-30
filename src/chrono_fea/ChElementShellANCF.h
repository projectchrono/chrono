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
#include "core/ChQuadrature.h"
#include "chrono_fea/ChApiFEA.h"
#include "chrono_fea/ChElementShell.h"
#include "chrono_fea/ChNodeFEAxyzD.h"
#include "chrono_fea/ChUtilsFEA.h"

namespace chrono {

namespace fea {

///
/// ANCF laminated shell element with four nodes.
///
class ChApiFea ChElementShellANCF : public ChElementShell, public ChLoadableUV, public ChLoadableUVW {
  public:
    ChElementShellANCF();
    ~ChElementShellANCF() {}
    /// New class

    /// Integrate  rho*(S'*S)
    /// where S=[N1*eye(3) N2*eye(3) N3*eye(3) N4*eye(3) N5*eye(3) N6*eye(3) N7*eye(3) N8*eye(3)]
    class MyMass : public ChIntegrable3D<ChMatrixNM<double, 24, 24> > {
      public:
		MyMass();
		MyMass(ChMatrixNM<double, 8, 3> *m_d0 , ChElementShellANCF* passedelement){
			d0 = m_d0;
			element = passedelement;
		}
		~MyMass() {}
	private:
        ChElementShellANCF* element;
        ChMatrixNM<double, 8, 3>* d0;  //// pointer to initial coordinates
        ChMatrixNM<double, 3, 24> S;
        ChMatrixNM<double, 1, 8> N;
        ChMatrixNM<double, 1, 8> Nx;
        ChMatrixNM<double, 1, 8> Ny;
        ChMatrixNM<double, 1, 8> Nz;
        /// Evaluate the S'*S  at point x
        virtual void Evaluate(ChMatrixNM<double, 24, 24>& result, const double x, const double y, const double z);
    };
    // Add gravity force
    class MyGravity : public ChIntegrable3D<ChMatrixNM<double, 24, 1> > {
      public:
		  MyGravity();
		  MyGravity(ChMatrixNM<double, 8, 3> *m_d0, ChElementShellANCF* passedelement){
			  d0 = m_d0;
			  element = passedelement;
		  }
		  ~MyGravity() {}
	private:
        ChElementShellANCF* element;
        ChMatrixNM<double, 8, 3>* d0;
        ChMatrixNM<double, 1, 8> N;
        ChMatrixNM<double, 1, 8> Nx;
        ChMatrixNM<double, 1, 8> Ny;
        ChMatrixNM<double, 1, 8> Nz;
        ChMatrixNM<double, 3, 1> LocalGravityForce;

        virtual void Evaluate(ChMatrixNM<double, 24, 1>& result, const double x, const double y, const double z);
    };

	// Add air pressure
	class MyAirPressure : public ChIntegrable2D<ChMatrixNM<double, 24, 1> > {
	public:
		MyAirPressure();
		MyAirPressure(ChMatrixNM<double, 8, 3> *m_d0, ChMatrixNM<double, 8, 3>* d_, ChElementShellANCF* element_){
			d0 = m_d0;
			d = d_;
			element = element_;
		}
		~MyAirPressure() {}
	private:
		ChElementShellANCF* element;
		ChMatrixNM<double, 8, 3>* d0;
		ChMatrixNM<double, 8, 3>* d;
		ChMatrixNM<double, 1, 4> S_ANS;
		ChMatrixNM<double, 1, 8> N;
		ChMatrixNM<double, 1, 8> Nx;
		ChMatrixNM<double, 1, 8> Ny;
		ChMatrixNM<double, 1, 8> Nz;
		ChMatrixNM<double, 3, 1> LocalAirPressure;

		virtual void Evaluate(ChMatrixNM<double, 24, 1>& result, const double x, const double y);
	};
    ///============ Internal force, EAS stiffness, and analytical jacobian are calculated
    class MyForce : public ChIntegrable3D<ChMatrixNM<double, 750, 1> > {
      public:
		  MyForce();
		  // Constructor
		  MyForce (ChMatrixNM<double, 8, 3>* d_, ChMatrixNM<double, 24, 1>* d_dt_, ChMatrixNM<double, 8, 1>* strain_ans_, 
			  ChMatrixNM<double, 8, 24>* strainD_ans_, ChMatrixNM<double, 8, 3> *m_d0, ChMatrixNM<double, 6, 6>* E_eps_, 
			  ChElementShellANCF* element_, ChMatrixNM<double, 6, 6>* T0_, double* detJ0C_, double* theta_, 
			  ChMatrixNM<double, 5, 1>* alpha_eas_){
			  d = d_;
			  d_dt = d_dt_; // Structural damping
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
        ChMatrixNM<double, 24, 1>* d_dt;  // for structural damping
        ChMatrixNM<double, 6, 6>* T0;
        ChMatrixNM<double, 5, 1>* alpha_eas;
        ChMatrixNM<double, 6, 6>* E_eps;
        double* detJ0C;
        double* theta;

        ChMatrixNM<double, 24, 1> Fint;
        ChMatrixNM<double, 24, 24> JAC11;
        ChMatrixNM<double, 9, 24> Gd;
        ChMatrixNM<double, 6, 1> stress;
        ChMatrixNM<double, 9, 9> Sigm;
        ChMatrixNM<double, 24, 6> temp246;
        ChMatrixNM<double, 24, 9> temp249;
        ChMatrixNM<double, 1, 8> Nx;
        ChMatrixNM<double, 1, 8> Ny;
        ChMatrixNM<double, 1, 8> Nz;
        ChMatrixNM<double, 6, 24> strainD;
        ChMatrixNM<double, 6, 1> strain;
        ChMatrixNM<double, 8, 8> d_d;
        ChMatrixNM<double, 8, 1> ddNx;
        ChMatrixNM<double, 8, 1> ddNy;
        ChMatrixNM<double, 8, 1> ddNz;
        ChMatrixNM<double, 8, 8> d0_d0;
        ChMatrixNM<double, 8, 1> d0d0Nx;
        ChMatrixNM<double, 8, 1> d0d0Ny;
        ChMatrixNM<double, 8, 1> d0d0Nz;
        ChMatrixNM<double, 1, 3> Nxd;
        ChMatrixNM<double, 1, 3> Nyd;
        ChMatrixNM<double, 1, 3> Nzd;
        ChMatrixNM<double, 1, 1> tempA;
        ChMatrixNM<double, 1, 1> tempA1;
        ChMatrixNM<double, 1, 24> tempB;
        ChMatrixNM<double, 24, 6> tempC;
        double detJ0;
        double alphaHHT;
        double betaHHT;
        double gammaHHT;
        // ANS
        ChMatrixNM<double, 1, 8> N;
        ChMatrixNM<double, 1, 4> S_ANS;
        ChMatrixNM<double, 1, 24> tempBB;
        // EAS
        ChMatrixNM<double, 6, 5> M;
        ChMatrixNM<double, 6, 5> G;
        ChMatrixNM<double, 5, 6> GT;
        ChMatrixNM<double, 6, 1> strain_EAS;

        /// Evaluate (strainD'*strain)  at point x
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

    /// Calculate the first part of the Jacobian of the elastic forces of the ANCF shell element
    void JacCalcUnrolled(ChMatrixNM<double, 6, 1> stress,
                         ChMatrixNM<double, 9, 24> Gd,
                         ChMatrixNM<double, 24, 24>& JAC11);

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

    ChMatrixNM<double, 8, 3> m_d0;  ///< initial nodal coordinates

    ChMatrixNM<double, 24, 1> m_GravForce;  ///< Gravity Force

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

    //
    // Functions for ChLoadable interface
    //

    /// Gets the number of DOFs affected by this element (position part)
    virtual int LoadableGet_ndof_x() { return 4 * 6; }

    /// Gets the number of DOFs affected by this element (speed part)
    virtual int LoadableGet_ndof_w() { return 4 * 6; }

    /// Gets all the DOFs packed in a single vector (position part)
    virtual void LoadableGetStateBlock_x(int block_offset, ChMatrixDynamic<>& mD) {
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
    virtual void LoadableGetStateBlock_w(int block_offset, ChMatrixDynamic<>& mD) {
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
