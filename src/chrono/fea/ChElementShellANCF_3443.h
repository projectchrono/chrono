// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Michael Taylor, Antonio Recuero, Radu Serban
// =============================================================================
// Fully Parameterized ANCF shell element with 4 nodes (48DOF). A Description of this element can be found in: Aki M
// Mikkola and Ahmed A Shabana. A non-incremental finite element procedure for the analysis of large deformation of
// plates and shells in mechanical system applications. Multibody System Dynamics, 9(3) : 283–309, 2003.
// =============================================================================
// The "Continuous Integration" style calculation for the generalized internal force is based on modifications to
// (including a new analytical Jacobian):  Gerstmayr, J., Shabana, A.A.: Efficient integration of the elastic forces and
// thin three-dimensional beam elements in the absolute nodal coordinate formulation.In: Proceedings of the Multibody
// Dynamics Eccomas thematic Conference, Madrid(2005).
//
// The "Pre-Integration" style calculation is based on modifications
// to Liu, Cheng, Qiang Tian, and Haiyan Hu. "Dynamics of a large scale rigid–flexible multibody system composed of
// composite laminated plates." Multibody System Dynamics 26, no. 3 (2011): 283-305.
//
// A report covering the detailed mathematics and implementation both of these generalized internal force calculations
// and their Jacobians can be found in: Taylor, M.: Technical Report TR-2020-09 Efficient CPU Based Calculations of the
// Generalized Internal Forces and Jacobian of the Generalized Internal Forces for ANCF Continuum Mechanics Elements
// with Linear Viscoelastic Materials, Simulation Based Engineering Lab, University of Wisconsin-Madison; 2021.
// =============================================================================

#ifndef CHELEMENTSHELLANCF3443_H
#define CHELEMENTSHELLANCF3443_H

#include <vector>

#include "chrono/fea/ChMaterialShellANCF.h"
#include "chrono/fea/ChElementANCF.h"
#include "chrono/fea/ChElementShell.h"
#include "chrono/fea/ChNodeFEAxyzDDD.h"

namespace chrono {
namespace fea {

/// @addtogroup fea_elements
/// @{

/// ANCF shell element with four nodes.  The coordinates at each node are the position vector
/// and 3 position vector gradients.
///
/// The node numbering is in ccw fashion as in the following scheme:
/// <pre>
///         v
///         ^
///         |
/// D o-----+-----o C
///   |     |     |
/// --+-----+-----+----> u
///   |     |     |
/// A o-----+-----o B
/// </pre>

class ChApi ChElementShellANCF_3443 : public ChElementANCF,
                                      public ChElementShell,
                                      public ChLoadableUV,
                                      public ChLoadableUVW {
  public:
    // Using fewer than 3 Gauss quadrature points for each midsurface direction (NP) or 2 Gauss quadrature points
    // through the thickness (NT) will likely result in numerical issues with the element.  A slightly less stiff
    // element can achieved by using NP=3, NT=2.
    static const int NP = 4;              ///< number of Gauss quadrature points for each midsurface direction
    static const int NT = 2;              ///< number of quadrature points through the thickness
    static const int NIP = NP * NP * NT;  ///< number of Gauss quadrature points
    static const int NSF = 16;            ///< number of shape functions

    using VectorN = ChVectorN<double, NSF>;
    using Vector3N = ChVectorN<double, 3 * NSF>;
    using VectorNIP = ChVectorN<double, NIP>;
    using Matrix3xN = ChMatrixNM<double, 3, NSF>;
    using MatrixNx3 = ChMatrixNM<double, NSF, 3>;
    using MatrixNx3c = ChMatrixNM_col<double, NSF, 3>;
    using MatrixNx6 = ChMatrixNM<double, NSF, 6>;
    using MatrixNxN = ChMatrixNM<double, NSF, NSF>;

    /// Internal force calculation method
    enum class IntFrcMethod {
        ContInt,  ///< "Continuous Integration" style method - Fastest for a small number of layers
        PreInt    ///< "Pre-Integration" style method - Fastest for a large number of layers
    };

    ChElementShellANCF_3443();
    ~ChElementShellANCF_3443() {}

    /// Definition of a layer
    class Layer {
      public:
        /// Return the layer thickness.
        double Get_thickness() const { return m_thickness; }

        /// Return the fiber angle.
        double Get_theta() const { return m_theta; }

        /// Return the layer material.
        std::shared_ptr<ChMaterialShellANCF> GetMaterial() const { return m_material; }

      private:
        /// Private constructor (a layer can be created only by adding it to an element)
        Layer(double thickness,                              ///< layer thickness
              double theta,                                  ///< fiber angle
              std::shared_ptr<ChMaterialShellANCF> material  ///< layer material
        );

        std::shared_ptr<ChMaterialShellANCF> m_material;  ///< layer material
        double m_thickness;                               ///< layer thickness
        double m_theta;                                   ///< fiber angle

      public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        friend class ChElementShellANCF_3443;
    };

    /// Get the number of nodes used by this element.
    virtual int GetNnodes() override { return 4; }

    /// Get the number of coordinates in the field used by the referenced nodes.
    virtual int GetNdofs() override { return 4 * 12; }

    /// Get the number of active coordinates in the field used by the referenced nodes.
    virtual int GetNdofs_active() override { return m_element_dof; }

    /// Get the number of coordinates from the n-th node used by this element.
    virtual int GetNodeNdofs(int n) override { return m_nodes[n]->GetNdofX(); }

    /// Get the number of active coordinates from the n-th node used by this element.
    virtual int GetNodeNdofs_active(int n) override { return m_nodes[n]->GetNdofX_active(); }

    /// Specify the nodes of this element.
    void SetNodes(std::shared_ptr<ChNodeFEAxyzDDD> nodeA,
                  std::shared_ptr<ChNodeFEAxyzDDD> nodeB,
                  std::shared_ptr<ChNodeFEAxyzDDD> nodeC,
                  std::shared_ptr<ChNodeFEAxyzDDD> nodeD);

    /// Specify the element dimensions.
    void SetDimensions(double lenX, double lenY);

    /// Access the n-th node of this element.
    virtual std::shared_ptr<ChNodeFEAbase> GetNodeN(int n) override { return m_nodes[n]; }

    /// Get a handle to the first node of this element.
    std::shared_ptr<ChNodeFEAxyzDDD> GetNodeA() const { return m_nodes[0]; }

    /// Get a handle to the second node of this element.
    std::shared_ptr<ChNodeFEAxyzDDD> GetNodeB() const { return m_nodes[1]; }

    /// Get a handle to the third node of this element.
    std::shared_ptr<ChNodeFEAxyzDDD> GetNodeC() const { return m_nodes[2]; }

    /// Get a handle to the fourth node of this element.
    std::shared_ptr<ChNodeFEAxyzDDD> GetNodeD() const { return m_nodes[3]; }

    /// Add a layer.  Layers should be added prior to the start of the simulation since can be a significant amount of
    /// pre-calculation overhead required.
    void AddLayer(double thickness,                              ///< layer thickness
                  double theta,                                  ///< fiber angle (radians)
                  std::shared_ptr<ChMaterialShellANCF> material  ///< layer material
    );

    /// Get the number of layers.
    size_t GetNumLayers() const { return m_numLayers; }

    /// Get a handle to the specified layer.
    const Layer& GetLayer(size_t i) const { return m_layers[i]; }

    /// Offset the midsurface of the composite shell element.  A positive value shifts the element's midsurface upward
    /// along the elements zeta direction.  The offset should be provided in model units.
    void SetMidsurfaceOffset(const double offset);

    /// Set the structural damping.
    void SetAlphaDamp(double a);

    /// Get the element length in the X direction.
    double GetLengthX() const { return m_lenX; }

    /// Get the element length in the Y direction.
    double GetLengthY() { return m_lenY; }

    /// Get the total thickness of the shell element.
    double GetThicknessZ() { return m_thicknessZ; }

    /// Set the calculation method to use for the generalized internal force and its Jacobian calculations.  This should
    /// be set prior to the start of the simulation since can be a significant amount of pre-calculation overhead
    /// required.
    void SetIntFrcCalcMethod(IntFrcMethod method);

    /// Return the type of calculation method currently set for the generalized internal force and its Jacobian
    /// calculations.
    IntFrcMethod GetIntFrcCalcMethod() { return m_method; }

    /// Get the Green-Lagrange strain tensor at the normalized element coordinates (xi, eta, zeta) at the current state
    /// of the element.  Normalized element coordinates span from -1 to 1.
    ChMatrix33<> GetGreenLagrangeStrain(const double xi, const double eta, const double zeta);

    /// Get the 2nd Piola-Kirchoff stress tensor at the normalized **layer** coordinates (xi, eta, layer_zeta) at the
    /// current state of the element for the specified layer number (0 indexed) since the stress can be discontinuous at
    /// the layer boundary.   "layer_zeta" spans -1 to 1 from the bottom surface to the top surface
    ChMatrix33<> GetPK2Stress(const double layer, const double xi, const double eta, const double layer_zeta);

    /// Get the von Mises stress value at the normalized **layer** coordinates (xi, eta, layer_zeta) at the current
    /// state of the element for the specified layer number (0 indexed) since the stress can be discontinuous at the
    /// layer boundary.  "layer_zeta" spans -1 to 1 from the bottom surface to the top surface
    double GetVonMissesStress(const double layer, const double xi, const double eta, const double layer_zeta);

  public:
    // Interface to ChElementBase base class
    // -------------------------------------

    /// Fill the D vector (column matrix) with the current field values at the nodes of the element, with proper
    /// ordering. If the D vector has not the size of this->GetNdofs(), it will be resized.
    ///  {Pos_a Du_a Dv_a Dw_a  Pos_b Du_b Dv_b Dw_b  Pos_c Du_c Dv_c Dw_c  Pos_d Du_d Dv_d Dw_d}
    virtual void GetStateBlock(ChVectorDynamic<>& mD) override;

    /// Update the state of this element.
    virtual void Update() override;

    /// Set M equal to the global mass matrix.
    virtual void ComputeMmatrixGlobal(ChMatrixRef M) override;

    /// Add contribution of element inertia to total nodal masses
    virtual void ComputeNodalMass() override;

    /// Compute the generalized internal force vector for the current nodal coordinates as set the value in the Fi
    /// vector.
    virtual void ComputeInternalForces(ChVectorDynamic<>& Fi) override;

    /// Set H as a linear combination of M, K, and R.
    ///   H = Mfactor * [M] + Kfactor * [K] + Rfactor * [R],
    /// where [M] is the mass matrix, [K] is the stiffness matrix, and [R] is the damping matrix.
    virtual void ComputeKRMmatricesGlobal(ChMatrixRef H,
                                          double Kfactor,
                                          double Rfactor = 0,
                                          double Mfactor = 0) override;

    /// Compute the generalized force vector due to gravity using the efficient ANCF specific method
    virtual void ComputeGravityForces(ChVectorDynamic<>& Fg, const ChVector<>& G_acc) override;

    // Interface to ChElementShell base class
    // --------------------------------------

    /// Gets the xyz displacement of a point on the shell midsurface,
    /// and the rotation RxRyRz of section plane, at abscissa '(xi,eta,0)'.
    virtual void EvaluateSectionDisplacement(const double xi,
                                             const double eta,
                                             ChVector<>& u_displ,
                                             ChVector<>& u_rotaz) override {}

    /// Gets the absolute xyz position of a point on the shell midsurface,
    /// and the absolute rotation of section plane, at abscissa '(xi,eta,0)'.
    /// Note, nodeA = (xi=-1, eta=-1)
    /// Note, nodeB = (xi=1, eta=-1)
    /// Note, nodeC = (xi=1, eta=1)
    /// Note, nodeD = (xi=-1, eta=1)
    virtual void EvaluateSectionFrame(const double xi,
                                      const double eta,
                                      ChVector<>& point,
                                      ChQuaternion<>& rot) override;

    /// Gets the absolute xyz position of a point on the shell midsurface specified in normalized coordinates
    virtual void EvaluateSectionPoint(const double xi, const double eta, ChVector<>& point) override;

    /// Gets the absolute xyz velocity of a point on the shell midsurface specified in normalized coordinates
    virtual void EvaluateSectionVelNorm(const double xi, const double eta, ChVector<>& Result) override;

    // Functions for ChLoadable interface
    // ----------------------------------

    /// Gets the number of DOFs affected by this element (position part).
    virtual int LoadableGet_ndof_x() override { return 4 * 12; }

    /// Gets the number of DOFs affected by this element (velocity part).
    virtual int LoadableGet_ndof_w() override { return 4 * 12; }

    /// Gets all the DOFs packed in a single vector (position part).
    virtual void LoadableGetStateBlock_x(int block_offset, ChState& mD) override;

    /// Gets all the DOFs packed in a single vector (velocity part).
    virtual void LoadableGetStateBlock_w(int block_offset, ChStateDelta& mD) override;

    /// Increment all DOFs using a delta.
    virtual void LoadableStateIncrement(const unsigned int off_x,
                                        ChState& x_new,
                                        const ChState& x,
                                        const unsigned int off_v,
                                        const ChStateDelta& Dv) override;

    /// Number of coordinates in the interpolated field, ex=3 for a
    /// tetrahedron finite element or a cable, = 1 for a thermal problem, etc.
    virtual int Get_field_ncoords() override { return 12; }

    /// Tell the number of DOFs blocks (ex. =1 for a body, =4 for a tetrahedron, etc.)
    virtual int GetSubBlocks() override { return 4; }

    /// Get the offset of the i-th sub-block of DOFs in global vector.
    virtual unsigned int GetSubBlockOffset(int nblock) override { return m_nodes[nblock]->NodeGetOffsetW(); }

    /// Get the size of the i-th sub-block of DOFs in global vector.
    virtual unsigned int GetSubBlockSize(int nblock) override { return 12; }

    /// Check if the specified sub-block of DOFs is active.
    virtual bool IsSubBlockActive(int nblock) const override { return !m_nodes[nblock]->IsFixed(); }

    /// Get the pointers to the contained ChVariables, appending to the mvars vector.
    virtual void LoadableGetVariables(std::vector<ChVariables*>& mvars) override;

    /// Evaluate N'*F , where N is some type of shape function
    /// evaluated at xi,eta coordinates of the midsurface, each ranging in -1..+1
    /// F is a load, N'*F is the resulting generalized load
    /// Returns also det[J] with J=[dx/du,..], that might be useful in gauss quadrature.
    /// For this ANCF element, only the first 6 entries in F are used in the calculation.  The first three entries is
    /// the applied force in global coordinates and the second 3 entries is the applied moment in global space.
    virtual void ComputeNF(const double xi,             ///< parametric coordinate in surface
                           const double eta,            ///< parametric coordinate in surface
                           ChVectorDynamic<>& Qi,       ///< Return result of Q = N'*F  here
                           double& detJ,                ///< Return det[J] here
                           const ChVectorDynamic<>& F,  ///< Input F vector, size is =n. field coords.
                           ChVectorDynamic<>* state_x,  ///< if != 0, update state (pos. part) to this, then evaluate
                           ChVectorDynamic<>* state_w   ///< if != 0, update state (speed part) to this, then evaluate
                           ) override;

    /// Evaluate N'*F , where N is some type of shape function
    /// evaluated at xi,eta,zeta coordinates of the volume, each ranging in -1..+1
    /// F is a load, N'*F is the resulting generalized load
    /// Returns also det[J] with J=[dx/du,..], that might be useful in gauss quadrature.
    /// For this ANCF element, only the first 6 entries in F are used in the calculation.  The first three entries is
    /// the applied force in global coordinates and the second 3 entries is the applied moment in global space.
    virtual void ComputeNF(const double xi,             ///< parametric coordinate in volume
                           const double eta,            ///< parametric coordinate in volume
                           const double zeta,           ///< parametric coordinate in volume
                           ChVectorDynamic<>& Qi,       ///< Return result of N'*F  here, maybe with offset block_offset
                           double& detJ,                ///< Return det[J] here
                           const ChVectorDynamic<>& F,  ///< Input F vector, size is = n.field coords.
                           ChVectorDynamic<>* state_x,  ///< if != 0, update state (pos. part) to this, then evaluate Q
                           ChVectorDynamic<>* state_w   ///< if != 0, update state (speed part) to this, then evaluate Q
                           ) override;

    /// This is needed so that it can be accessed by ChLoaderVolumeGravity.
    /// Density is the average mass per unit volume for the entire shell in the reference state of the element.
    virtual double GetDensity() override;

    /// Gets the normal to the surface at the parametric coordinate xi,eta.
    /// Each coordinate ranging in -1..+1.
    virtual ChVector<> ComputeNormal(const double xi, const double eta) override;

  private:
    /// Initial setup. This is used to precompute matrices that do not change during the simulation, such as the local
    /// stiffness of each element (if any), the mass, etc.
    virtual void SetupInitial(ChSystem* system) override;

    // Internal computations
    // ---------------------

    /// Compute the mass matrix & generalized gravity force of the element.
    /// Note: in this implementation, a constant density material is assumed
    void ComputeMassMatrixAndGravityForce();

    /// Precalculate constant matrices and scalars for the internal force calculations.  This selects and calls the
    /// method for the style of internal force calculations that is currently selected.
    void PrecomputeInternalForceMatricesWeights();

    /// Precalculate constant matrices and scalars for the "Continuous Integration" style method
    void PrecomputeInternalForceMatricesWeightsContInt();

    /// Precalculate constant matrices and scalars for the "Pre-Integration" style method
    void PrecomputeInternalForceMatricesWeightsPreInt();

    /// Calculate the generalized internal force for the element at the current nodal coordinates and time derivatives
    /// of the nodal coordinates using the "Continuous Integration" style method assuming damping is included
    void ComputeInternalForcesContIntDamping(ChVectorDynamic<>& Fi);

    /// Calculate the generalized internal force for the element at the current nodal coordinates and time derivatives
    /// of the nodal coordinates using the "Continuous Integration" style method assuming no damping
    void ComputeInternalForcesContIntNoDamping(ChVectorDynamic<>& Fi);

    /// Calculate the generalized internal force for the element at the current nodal coordinates and time derivatives
    /// of the nodal coordinates using the "Pre-Integration" style method assuming damping (works well for the case of
    /// no damping as well)
    void ComputeInternalForcesContIntPreInt(ChVectorDynamic<>& Fi);

    /// Calculate the calculate the Jacobian of the internal force integrand using the "Continuous Integration" style
    /// method assuming damping is included This function calculates a linear combination of the stiffness (K) and
    /// damping (R) matrices,
    ///     J = Kfactor * K + Rfactor * R
    /// for given coefficients Kfactor and Rfactor.
    /// This Jacobian will be further combined with the global mass matrix M and included in the global
    /// stiffness matrix H in the function ComputeKRMmatricesGlobal().
    void ComputeInternalJacobianContIntDamping(ChMatrixRef& H, double Kfactor, double Rfactor);

    /// Calculate the calculate the Jacobian of the internal force integrand using the "Continuous Integration" style
    /// method assuming damping is not included This function calculates just the stiffness (K) matrix,
    ///     J = Kfactor * K
    /// for the given coefficient Kfactor
    /// This Jacobian will be further combined with the global mass matrix M and included in the global
    /// stiffness matrix H in the function ComputeKRMmatricesGlobal().
    void ComputeInternalJacobianContIntNoDamping(ChMatrixRef& H, double Kfactor);

    /// Calculate the calculate the Jacobian of the internal force integrand using the "Pre-Integration" style method
    /// assuming damping is included This function calculates a linear combination of the stiffness (K) and damping (R)
    /// matrices,
    ///     J = Kfactor * K + Rfactor * R
    /// for given coefficients Kfactor and Rfactor.
    /// This Jacobian will be further combined with the global mass matrix M and included in the global
    /// stiffness matrix H in the function ComputeKRMmatricesGlobal().
    void ComputeInternalJacobianPreInt(ChMatrixRef& H, double Kfactor, double Rfactor);

    /// Calculate the current 3Nx1 vector of nodal coordinates.
    void CalcCoordVector(Vector3N& e);

    /// Calculate the current 3xN matrix of nodal coordinates.
    void CalcCoordMatrix(Matrix3xN& ebar);

    /// Calculate the current 3Nx1 vector of nodal coordinate time derivatives.
    void CalcCoordDerivVector(Vector3N& edot);

    /// Calculate the current 3xN matrix of nodal coordinate time derivatives.
    void CalcCoordDerivMatrix(Matrix3xN& ebardot);

    /// Calculate the current Nx6 matrix of the transpose of the nodal coordinates and nodal coordinate time
    /// derivatives.
    void CalcCombinedCoordMatrix(MatrixNx6& ebar_ebardot);

    /// Calculate the Nx1 Compact Vector of the Normalized Shape Functions (just the unique values)
    void Calc_Sxi_compact(VectorN& Sxi_compact, double xi, double eta, double zeta, double thickness, double zoffset);

    /// Calculate the Nx1 Compact Vector of the Derivative of the Normalized Shape Functions with respect to xi
    void Calc_Sxi_xi_compact(VectorN& Sxi_xi_compact,
                             double xi,
                             double eta,
                             double zeta,
                             double thickness,
                             double zoffset);

    /// Calculate the Nx1 Compact Vector of the Derivative of the Normalized Shape Functions with respect to eta
    void Calc_Sxi_eta_compact(VectorN& Sxi_eta_compact,
                              double xi,
                              double eta,
                              double zeta,
                              double thickness,
                              double zoffset);

    /// Calculate the Nx1 Compact Vector of the Derivative of the Normalized Shape Functions with respect to zeta
    void Calc_Sxi_zeta_compact(VectorN& Sxi_zeta_compact,
                               double xi,
                               double eta,
                               double zeta,
                               double thickness,
                               double zoffset);

    /// Calculate the Nx3 Compact Matrix of the Derivatives of the Normalized Shape Functions with respect to xi, eta,
    /// and then zeta
    void Calc_Sxi_D(MatrixNx3c& Sxi_D, double xi, double eta, double zeta, double thickness, double zoffset);

    /// Calculate the element Jacobian of the reference configuration with respect to the normalized configuration
    void Calc_J_0xi(ChMatrix33<double>& J_0xi, double xi, double eta, double zeta, double thickness, double zoffset);

    /// Calculate the determinate of the element Jacobian of the reference configuration with respect to the normalized
    /// configuration
    double Calc_det_J_0xi(double xi, double eta, double zeta, double thickness, double zoffset);

    /// Calculate the rotated 6x6 stiffness matrix and reorder it to match the Voigt notation order used with this
    /// element
    void RotateReorderStiffnessMatrix(ChMatrixNM<double, 6, 6>& D, double theta);

    /// Access a statically-allocated set of tables, from 0 to a 10th order, with precomputed tables.
    static ChQuadratureTables* GetStaticGQTables();

    IntFrcMethod m_method;                                  ///< internal force and Jacobian calculation method
    std::vector<std::shared_ptr<ChNodeFEAxyzDDD>> m_nodes;  ///< element nodes

    std::vector<Layer, Eigen::aligned_allocator<Layer>> m_layers;  ///< element layers
    std::vector<double, Eigen::aligned_allocator<double>>
        m_layer_zoffsets;      ///< Offsets of Bottom of Layers to the Bottom of the Element
    int m_numLayers;           ///< number of layers for this element
    double m_lenX;             ///< total element length along X
    double m_lenY;             ///< total element length along Y
    double m_thicknessZ;       ///< total element thickness along Z
    double m_midsurfoffset;    ///< Offset of the midsurface along Z
    double m_Alpha;            ///< structural damping
    bool m_damping_enabled;    ///< Flag to run internal force damping calculations
    VectorN m_GravForceScale;  ///< Gravity scaling matrix used to get the generalized force due to gravity
    Matrix3xN m_ebar0;         ///< Element Position Coordinate Vector for the Reference Configuration
    ChVectorN<double, (NSF * (NSF + 1)) / 2>
        m_MassMatrix;  /// Mass Matrix in extra compact form (Upper Triangular Part only)
    ChMatrixDynamic<>
        m_SD;  ///< Precomputed corrected normalized shape function derivative matrices ordered by columns instead of by
               ///< Gauss quadrature points used for the "Continuous Integration" style method
    ChMatrixDynamic_col<> m_kGQ;  ///< Precomputed Gauss-Quadrature Weight & Element Jacobian scale factors used for the
                                  ///< "Continuous Integration" style method
    ChMatrixDynamic_col<> m_O1;   ///< Precomputed Matrix combined with the nodal coordinates used for the
                                  ///< "Pre-Integration" style method internal force calculation
    ChMatrixDynamic_col<> m_O2;   ///< Precomputed Matrix combined with the nodal coordinates used for the
                                  ///< "Pre-Integration" style method Jacobian calculation
    ChMatrixDynamic_col<> m_K3Compact;  ///< Precomputed Matrix combined with the nodal coordinates used for the
                                        ///< "Pre-Integration" style method internal force calculation
    ChMatrixDynamic_col<>
        m_K13Compact;  ///< Saved results from the generalized internal force calculation that are reused for the
                       ///< Jacobian calculations for the "Pre-Integration" style method

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

/// @} fea_elements

}  // end of namespace fea
}  // end of namespace chrono

#endif
