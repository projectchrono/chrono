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
// Authors: Alessandro Tasora
// =============================================================================
// Geometrically exact kinematics of shell, with fromulation from Masarati et.al.
// =============================================================================

#ifndef CHELEMENTSHELLEANS4_H
#define CHELEMENTSHELLEANS4_H

#include <vector>
#include <array>

#include "chrono_fea/ChApiFEA.h"
#include "chrono_fea/ChElementShell.h"
#include "chrono_fea/ChNodeFEAxyzrot.h"
#include "chrono_fea/ChUtilsFEA.h"
#include "chrono_fea/ChMaterialShellReissner.h"
#include "chrono/solver/ChVariablesGenericDiagonalMass.h"

namespace chrono {
namespace fea {

/// @addtogroup fea_elements
/// @{

/// Shell with geometrically exact kinematics, with 4 nodes.
/// Uses ANS to avoid shear locking.
/// Based on the paper:
/// "Implementation and validation of a 4-node shell finite element"
/// Marco Morandini, Pierangelo Masarati.  IDETC/CIE 2014.
///
/// The node numbering is in ccw fashion as in the following scheme:
///         v
///         ^
/// D o-----+-----o C
///   |     |     |
/// --+-----+-----+-> u
///   |     |     |
/// A o-----+-----o B
///
class ChApiFea ChElementShellReissner4 : public ChElementShell, public ChLoadableUV, public ChLoadableUVW {
  public:
    ChElementShellReissner4();
    virtual ~ChElementShellReissner4();

    /// Definition of a layer
    class Layer {
      public:
        /// Return the layer thickness.
        double Get_thickness() const { return m_thickness; }

        /// Return the fiber angle.
        double Get_theta() const { return m_theta; }

        /// Return the layer material.
        std::shared_ptr<ChMaterialShellReissner> GetMaterial() const { return m_material; }

      private:
        /// Private constructor (a layer can be created only by adding it to an element)
        Layer(ChElementShellReissner4* element,                  ///< containing element
              double thickness,                                  ///< layer thickness
              double theta,                                      ///< fiber angle
              std::shared_ptr<ChMaterialShellReissner> material  ///< layer material
              );

        /// Initial setup for this layer
        void SetupInitial();

        ChElementShellReissner4* m_element;                   ///< containing shell element
        std::shared_ptr<ChMaterialShellReissner> m_material;  ///< layer material
        double m_thickness;                                   ///< layer thickness
        double m_theta;                                       ///< fiber angle

        friend class ChElementShellReissner4;
        friend class MyForce;
        friend class MyJacobian;
    };

    /// Get the number of nodes used by this element.
    virtual int GetNnodes() override { return 4; }

    /// Get the number of coordinates in the field used by the referenced nodes.
    virtual int GetNdofs() override { return 4 * 6; }

    /// Get the number of coordinates from the n-th node used by this element.
    virtual int GetNodeNdofs(int n) override { return 6; }

    /// Specify the nodes of this element.
    /// The node numbering is in ccw fashion as in the following scheme:
    ///         v
    ///         ^
    /// D o-----+-----o C
    ///   |     |     |
    /// --+-----+-----+-> u
    ///   |     |     |
    /// A o-----+-----o B
    ///
    void SetNodes(std::shared_ptr<ChNodeFEAxyzrot> nodeA,
                  std::shared_ptr<ChNodeFEAxyzrot> nodeB,
                  std::shared_ptr<ChNodeFEAxyzrot> nodeC,
                  std::shared_ptr<ChNodeFEAxyzrot> nodeD);

    /// Access the n-th node of this element.
    virtual std::shared_ptr<ChNodeFEAbase> GetNodeN(int n) override { return m_nodes[n]; }

    /// Get a handle to the first node of this element.
    std::shared_ptr<ChNodeFEAxyzrot> GetNodeA() const { return m_nodes[0]; }

    /// Get a handle to the second node of this element.
    std::shared_ptr<ChNodeFEAxyzrot> GetNodeB() const { return m_nodes[1]; }

    /// Get a handle to the third node of this element.
    std::shared_ptr<ChNodeFEAxyzrot> GetNodeC() const { return m_nodes[2]; }

    /// Get a handle to the fourth node of this element.
    std::shared_ptr<ChNodeFEAxyzrot> GetNodeD() const { return m_nodes[3]; }

    /// Sets the neutral rotations of nodes A,B,C,D, at once,
    /// assuming the current element position is for zero strain.
    void SetAsNeutral();

    /// Add a layer.
    /// By default, when adding more than one layer, the reference z level of the
    /// shell element is centered along the total thickness.
    void AddLayer(double thickness,                                  ///< layer thickness
                  double theta,                                      ///< fiber angle (radians)
                  std::shared_ptr<ChMaterialShellReissner> material  ///< layer material
                  );

    /// Impose the reference z level of shell element as centered along the total thickness.
    /// This is the default behavior each time you call AddLayer();
    /// Note! Use after you added all layers.
    void SetLayerZreferenceCentered();

    /// Impose the reference z level of shell element respect to the lower face of bottom layer
    /// Note! Use after you added all layers.
    void SetLayerZreference(double z_from_bottom);

    /// Get the number of layers.
    size_t GetNumLayers() const { return m_layers.size(); }

    /// Get a handle to the specified layer.
    const Layer& GetLayer(size_t i) const { return m_layers[i]; }

    /// Set the structural damping: this is the Rayleigh "alpha" for the
    /// stiffness-proportional damping. This assumes damping forces as F=alpha*[Km]*v
    /// where [Km] is the stiffness matrix (material part, i.e.excluding geometric stiffness)
    /// and v is a vector of node speeds. Usually, alpha in the range 0.0 - 0.1
    /// Note that the mass-proportional term of classical Rayleigh damping is not supported.
    void SetAlphaDamp(double a) { m_Alpha = a; }

    /// Get the element length in the X direction.
    double GetLengthX() const { return m_lenX; }
    /// Get the element length in the Y direction.
    double GetLengthY() const { return m_lenY; }
    /// Get the total thickness of the shell element (might be sum of multiple layer thicknesses)
    double GetThickness() { return tot_thickness; }

    ChQuaternion<> GetAvgRot() { return T_overline.Get_A_quaternion(); }

    // Shape functions
    // ---------------

    /// Fills the N shape function matrix.
    void ShapeFunctions(ChMatrix<>& N, double x, double y);

    /// Fills the Nx shape function derivative matrix with respect to X.
    void ShapeFunctionsDerivativeX(ChMatrix<>& Nx, double x, double y);

    /// Fills the Ny shape function derivative matrix with respect to Y.
    void ShapeFunctionsDerivativeY(ChMatrix<>& Ny, double x, double y);

    ChVector<> EvaluateGP(int igp);
    ChVector<> EvaluatePT(int ipt);

    /// Inner EAS dofs
    virtual unsigned int iGetNumDof(void) const { return 7; };

  private:
    void UpdateNodalAndAveragePosAndOrientation();
    void ComputeInitialNodeOrientation();
    void InterpolateOrientation();
    void ComputeIPCurvature();

    //***TEST*** to make private
  public:
    //
    // DATA
    //
    std::vector<std::shared_ptr<ChNodeFEAxyzrot> > m_nodes;  ///< element nodes

    std::vector<Layer> m_layers;     ///< element layers
    std::vector<double> m_layers_z;  ///< layer separation z values (not scaled, default centered tot thickness)

    double tot_thickness;                         ///< total element thickness
    double m_lenX;                                ///< element length in X direction
    double m_lenY;                                ///< element length in Y direction
    double m_Alpha;                               ///< structural damping
    ChMatrixNM<double, 24, 24> m_MassMatrix;      ///< mass matrix
    ChMatrixNM<double, 24, 24> m_JacobianMatrix;  ///< Jacobian matrix (Kfactor*[K] + Rfactor*[R])

    //
    // from original MBDyn
    //
  public:
    // numbered according to
    //
    //         ^
    // 4 o-----+-----o 3
    //   | 1_2 | 2_2 |
    // --+-----+-----+->
    //   | 1_1 | 2_1 |
    // 1 o-----+-----o 2
    //
    enum IntegrationPoint {
        IP_1_1 = 0,
        IP_1_2 = 1,
        IP_1_3 = 2,
        IP_2_1 = 3,
        IP_2_2 = 4,
        IP_2_3 = 5,
        IP_3_1 = 6,
        IP_3_2 = 7,
        IP_3_3 = 8,

        NUMIP = 4
    };

    static double xi_i[NUMIP][2];
    static double w_i[NUMIP];

    // numbered according to the side they are defined on
    enum ShearStrainEvaluationPoint {
        SSEP_1 = 0,
        SSEP_2 = 1,
        SSEP_3 = 2,
        SSEP_4 = 3,

        NUMSSEP = 4
    };

    static double xi_A[NUMSSEP][2];

    enum NodeName {
        NODE1 = 0,
        NODE2 = 1,
        NODE3 = 2,
        NODE4 = 3,

        NUMNODES = 4
    };

    static double xi_n[NUMNODES][2];

    static double xi_0[2];

    enum Deformations {
        STRAIN = 0,
        CURVAT = 1,

        NUMDEFORM = 2
    };

    enum InnerEASdofs { IDOFS = 7 };
    ChVariablesGenericDiagonalMass* mvariables;

    //***TODO*** make protected
  public:
    // nodal positions (0: initial; otherwise current)
    ChVector<> xa_0[NUMNODES];
    ChVector<> xa[NUMNODES];
    // current nodal orientation
    ChMatrix33<> iTa[NUMNODES];
    ChMatrix33<> iTa_i[NUMIP];
    ChMatrix33<> iTa_A[NUMSSEP];
    // Euler vector of Ra
    ChVector<> phi_tilde_n[NUMNODES];

    // Average orientation matrix
    ChVector<> phi_tilde_i[NUMIP];
    ChVector<> phi_tilde_A[NUMSSEP];
    ChVector<> phi_tilde_0;
    // Average orientation matrix
    //    .. in reference configuration
    ChMatrix33<> T0_overline;
    //    .. in current configuration
    ChMatrix33<> T_overline;

    // Orientation matrix
    //    .. in reference configuration
    ChMatrix33<> T_0_0;
    ChMatrix33<> T_0_i[NUMIP];
    ChMatrix33<> T_0_A[NUMSSEP];
    //    .. in current configuration
    ChMatrix33<> T_0;
    ChMatrix33<> T_i[NUMIP];
    ChMatrix33<> T_A[NUMSSEP];

    ChMatrix33<> Phi_Delta_i[NUMIP][NUMNODES];
    ChMatrix33<> Phi_Delta_A[NUMIP][NUMNODES];
    ChMatrix33<> Kappa_delta_i_1[NUMIP][NUMNODES];
    ChMatrix33<> Kappa_delta_i_2[NUMIP][NUMNODES];

    // rotation tensors
    ChMatrix33<> Q_i[NUMIP];
    ChMatrix33<> Q_A[NUMSSEP];

    // Orientation tensor derivative axial vector
    ChVector<> k_1_i[NUMIP];
    ChVector<> k_2_i[NUMIP];

    // linear deformation vectors
    //    .. in reference configuration
    ChVector<> eps_tilde_1_0_i[NUMIP];
    ChVector<> eps_tilde_2_0_i[NUMIP];
    ChVector<> eps_tilde_1_0_A[NUMSSEP];
    ChVector<> eps_tilde_2_0_A[NUMSSEP];
    //    .. in current configuration
    ChVector<> eps_tilde_1_i[NUMIP];
    ChVector<> eps_tilde_2_i[NUMIP];
    ChVector<> eps_tilde_1_A[NUMSSEP];
    ChVector<> eps_tilde_2_A[NUMSSEP];

    // angular deformation vectors
    //    .. in reference configuration
    ChVector<> k_tilde_1_0_i[NUMIP];
    ChVector<> k_tilde_2_0_i[NUMIP];
    //    .. in current configuration
    ChVector<> k_tilde_1_i[NUMIP];
    ChVector<> k_tilde_2_i[NUMIP];

    ChMatrixNM<double, 2, 2> S_alpha_beta_0;
    ChMatrixNM<double, 2, 2> S_alpha_beta_i[NUMIP];
    ChMatrixNM<double, 2, 2> S_alpha_beta_A[NUMSSEP];
    double alpha_0;
    double alpha_i[NUMIP];
    ChMatrixNM<double, 4, 2> L_alpha_beta_i[NUMIP];
    ChMatrixNM<double, 4, 2> L_alpha_beta_A[NUMSSEP];

    ChMatrixNM<double, 12, 24> B_overline_i[NUMIP];
    ChMatrixNM<double, 15, 24> D_overline_i[NUMIP];
    ChMatrixNM<double, 15, 15> G_i[NUMIP];

    ChMatrixNM<double, 12, IDOFS> P_i[NUMIP];

    ChMatrixNM<double, IDOFS, IDOFS> K_beta_beta_i[NUMIP];

    ChVector<> y_i_1[NUMIP];
    ChVector<> y_i_2[NUMIP];

    ChMatrixNM<double, IDOFS, 1> beta;
    ChMatrixNM<double, 12, 1> epsilon_hat;
    ChMatrixNM<double, 12, 1> epsilon;

    // Reference constitutive law tangent matrices
    ChMatrixNM<double, 12, 12> DRef[NUMIP];

    // stress
    ChMatrixNM<double, 12, 1> stress_i[NUMIP];

    // Is first residual
    bool bFirstRes;

  public:
    // Interface to ChElementBase base class
    // -------------------------------------

    // Fill the D vector (column matrix) with the current field values at the
    // nodes of the element, with proper ordering.
    // If the D vector has not the size of this->GetNdofs_x(), it will be resized.
    //  {x_a y_a z_a Rx_a Rx_a Rx_a x_b y_b z_b Rx_b Ry_b Rz_b}
    virtual void GetStateBlock(ChMatrixDynamic<>& mD) override;

    // Set H as a linear combination of M, K, and R.
    //   H = Mfactor * [M] + Kfactor * [K] + Rfactor * [R],
    // where [M] is the mass matrix, [K] is the stiffness matrix, and [R] is the damping matrix.
    virtual void ComputeKRMmatricesGlobal(ChMatrix<>& H,
                                          double Kfactor,
                                          double Rfactor = 0,
                                          double Mfactor = 0) override;

    // Set M as the global mass matrix.
    virtual void ComputeMmatrixGlobal(ChMatrix<>& M) override;

    /// Computes the internal forces.
    /// (E.g. the actual position of nodes is not in relaxed reference position) and set values
    /// in the Fi vector.
    virtual void ComputeInternalForces(ChMatrixDynamic<>& Fi) override;

    /// Initial setup.
    /// This is used mostly to precompute matrices that do not change during the simulation,
    /// such as the local stiffness of each element (if any), the mass, etc.
    virtual void SetupInitial(ChSystem* system) override;

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

    /// Compute Jacobians of the internal forces.
    /// This function calculates a linear combination of the stiffness (K) and damping (R) matrices,
    ///     J = Kfactor * K + Rfactor * R
    /// for given coeficients Kfactor and Rfactor.
    /// This Jacobian will be further combined with the global mass matrix M and included in the global
    /// stiffness matrix H in the function ComputeKRMmatricesGlobal().
    void ComputeInternalJacobians(double Kfactor, double Rfactor);

    /// Compute the mass matrix of the element.
    /// Note: in this 'basic' implementation, constant section and
    /// constant material are assumed
    void ComputeMassMatrix();

    /// Compute the gravitational forces.
    void ComputeGravityForce(const ChVector<>& g_acc);

    // Functions for ChLoadable interface
    // ----------------------------------

    /// Gets the number of DOFs affected by this element (position part).
    virtual int LoadableGet_ndof_x() override { return 4 * 7; }

    /// Gets the number of DOFs affected by this element (velocity part).
    virtual int LoadableGet_ndof_w() override { return 4 * 6; }

    /// Gets all the DOFs packed in a single vector (position part).
    virtual void LoadableGetStateBlock_x(int block_offset, ChState& mD) override;

    /// Gets all the DOFs packed in a single vector (velocity part).
    virtual void LoadableGetStateBlock_w(int block_offset, ChStateDelta& mD) override;

    /// Increment all DOFs using a delta.
    virtual void LoadableStateIncrement(const unsigned int off_x, ChState& x_new, const ChState& x, const unsigned int off_v, const ChStateDelta& Dv) override;

    /// Number of coordinates in the interpolated field, ex=3 for a
    /// tetrahedron finite element or a cable, = 1 for a thermal problem, etc.
    virtual int Get_field_ncoords() override { return 6; }

    /// Tell the number of DOFs blocks (ex. =1 for a body, =4 for a tetrahedron, etc.)
    virtual int GetSubBlocks() override { return 4; }

    /// Get the offset of the i-th sub-block of DOFs in global vector.
    virtual unsigned int GetSubBlockOffset(int nblock) override { return m_nodes[nblock]->NodeGetOffset_w(); }

    /// Get the size of the i-th sub-block of DOFs in global vector.
    virtual unsigned int GetSubBlockSize(int nblock) override { return 6; }

    virtual void EvaluateSectionVelNorm(double U, double V, ChVector<>& Result) override;

    /// Get the pointers to the contained ChVariables, appending to the mvars vector.
    virtual void LoadableGetVariables(std::vector<ChVariables*>& mvars) override;

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
                           ) override;

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
                           ) override;

    /// This is needed so that it can be accessed by ChLoaderVolumeGravity.
    /// Density is mass per unit surface.
    virtual double GetDensity() override;

    /// Gets the normal to the surface at the parametric coordinate U,V.
    /// Each coordinate ranging in -1..+1.
    virtual ChVector<> ComputeNormal(const double U, const double V) override;

    friend class MyMassEANS;
    friend class MyGravity;
    friend class MyForceEANS;
    friend class MyJacobianEANS;
};

/// @} fea_elements

}  // end of namespace fea
}  // end of namespace chrono

#endif
