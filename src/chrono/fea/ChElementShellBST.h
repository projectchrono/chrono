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
// Authors: Alessandro Tasora
// =============================================================================
// A Kirchhoff triangle thin shell element with 6 node (3 for triangle, 3 for
// neighbouring triangles) as in the BST Basic Shell Triangle (Onate et al.)
// =============================================================================

#ifndef CHELEMENTSHELLBST_H
#define CHELEMENTSHELLBST_H

#include <vector>
#include <array>

#include "chrono/fea/ChElementShell.h"
#include "chrono/fea/ChNodeFEAxyz.h"
#include "chrono/fea/ChMaterialShellKirchhoff.h"
#include "chrono/solver/ChVariablesGenericDiagonalMass.h"

namespace chrono {
namespace fea {

/// @addtogroup fea_elements
/// @{

/// A Kirchhoff-Love thin shell element of triangular shape.
/// This is based on the BST Basic Shell Triangle idea of Onate et al., where
/// 3 purely translational nodes are used for the triangle itself, and 3 other
/// purely translational nodes are used from the neighbouring triangles to infer
/// curvature.
///
/// The node numbering is as in the following scheme, where 0-1-2 are the nodes of the
/// triangle, and 3-4-5 are the nodes from neighbouring elements in the mesh:
/// <pre>
///
///          2
///   4 o----o---o 3
///      \   |\  |
///       \  | \ |
///        \ |  \|
///        0 o---o 1
///           \  |
///            \ |
///             \|
///              o 5
/// </pre>
class ChApi ChElementShellBST : public ChElementShell, public ChLoadableUV, public ChLoadableUVW {
  public:
    using ShapeVector = ChMatrixNM<double, 1, 3>;

    ChElementShellBST();
    ~ChElementShellBST();

    /// Definition of a layer
    class Layer {
      public:
        /// Return the layer thickness.
        double GetThickness() const { return m_thickness; }

        /// Return the fiber angle.
        double GetFiberAngle() const { return m_theta; }

        /// Return the layer material.
        std::shared_ptr<ChMaterialShellKirchhoff> GetMaterial() const { return m_material; }

      private:
        /// Private constructor (a layer can be created only by adding it to an element)
        Layer(ChElementShellBST* element,                         ///< containing element
              double thickness,                                   ///< layer thickness
              double theta,                                       ///< fiber angle
              std::shared_ptr<ChMaterialShellKirchhoff> material  ///< layer material
        );

        /// Initial setup for this layer
        void SetupInitial();

        ChElementShellBST* m_element;                          ///< containing shell element
        std::shared_ptr<ChMaterialShellKirchhoff> m_material;  ///< layer material
        double m_thickness;                                    ///< layer thickness
        double m_theta;                                        ///< fiber angle

        friend class ChElementShellBST;
        friend class MyForce;
        friend class MyJacobian;
    };

    /// Specify the nodes of this element.
    /// The node numbering is as in the following scheme, where 0-1-2 are the nodes of the
    /// triangle, and 3-4-5 are the nodes from neighbouring elements in the mesh:
    /// On the boundary, one or more of nodes 3,4,5 might be "nullptr".
    /// <pre>
    ///          2
    ///   4 o----o---o 3
    ///      \   |\  |
    ///       \  | \ |
    ///        \ |  \|
    ///        0 o---o 1
    ///           \  |
    ///            \ |
    ///             \|
    ///              o 5
    /// </pre>
    void SetNodes(std::shared_ptr<ChNodeFEAxyz> node0,
                  std::shared_ptr<ChNodeFEAxyz> node1,
                  std::shared_ptr<ChNodeFEAxyz> node2,
                  std::shared_ptr<ChNodeFEAxyz> node3,
                  std::shared_ptr<ChNodeFEAxyz> node4,
                  std::shared_ptr<ChNodeFEAxyz> node5);

    /// Get the number of nodes used by this element, not considering those marked with "nullptr" ex. if at boundary
    virtual unsigned int GetNumNodes() override { return n_usednodes; }

    /// Get the number of coordinates in the field used by the referenced nodes.
    virtual unsigned int GetNumCoordsPosLevel() override { return n_usednodes * 3; }

    /// Get the number of coordinates from the n-th node used by this element.
    virtual unsigned int GetNodeNumCoordsPosLevel(unsigned int index) override { return 3; }

    /// Access the n-th node of this element, not considering those marked with "nullptr" ex. if at boundary n=0..5
    virtual std::shared_ptr<ChNodeFEAbase> GetNode(unsigned int index) override { return m_nodes[nodes_used_to_six[index]]; }

    /// Get a handle to the n node of this element, among the three of the triangle part, n=0..2
    std::shared_ptr<ChNodeFEAxyz> GetNodeMainTriangle(unsigned int index) const { return m_nodes[index]; }

    /// Get a handle to the n node from the three of the neighbouring triangles, n=0..2.  Even if nullptr.
    std::shared_ptr<ChNodeFEAxyz> GetNodeNeighbour(unsigned int index) const { return m_nodes[3 + index]; }

    /// Sets the neutral rotations of nodes at once,
    /// assuming the current element position is for zero strain.
    void SetAsNeutral();

    /// Add a layer.
    /// By default, when adding more than one layer, the reference z level of the
    /// shell element is centered along the total thickness.
    void AddLayer(double thickness,                                   ///< layer thickness
                  double theta,                                       ///< fiber angle (radians)
                  std::shared_ptr<ChMaterialShellKirchhoff> material  ///< layer material
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

    /// Get the total thickness of the shell element (might be sum of multiple layer thicknesses)
    double GetThickness() { return tot_thickness; }

    // Shape functions
    // ---------------

    /// Fills the N shape function matrix, evaluated in "natural" triangle coordinates U and V.
    void ShapeFunctions(ShapeVector& N, const double u, const double v);

    /// Fills the Nu shape function derivative matrix with respect to U. Note, may remove u,v parms cause constant
    void ShapeFunctionsDerivativeU(ShapeVector& Nu, const double u, const double v);

    /// Fills the Nv shape function derivative matrix with respect to V. Note, may remove u,v parms cause constant
    void ShapeFunctionsDerivativeV(ShapeVector& Nv, const double u, const double v);

    /// Fills the Nx shape function derivative matrix with respect to X local element direction.
    /// The jacobian Jac_ux = [d{u,v}/d{x,y}] must be provided, often from inverse of Jac_xu = [d{x,y}/d{u,v}],
    /// so that [Nx;Ny]=[Jux]*[Nu;Nv].   Note, may remove u,v parms cause constant
    void ShapeFunctionsDerivativeX(ShapeVector& Nx,
                                   const ChMatrixNM<double, 2, 2>& Jac_ux,
                                   const double u,
                                   const double v);

    /// Fills the Ny shape function derivative matrix with respect to Y local element direction.
    /// The jacobian Jac_ux = [d{u,v}/d{x,y}] must be provided, often from inverse of Jac_xu = [d{x,y}/d{u,v}],
    /// so that [Nx;Ny]=[Jux]*[Nu;Nv].   Note, may remove u,v parms cause constant
    void ShapeFunctionsDerivativeY(ShapeVector& Ny,
                                   const ChMatrixNM<double, 2, 2>& Jac_ux,
                                   const double u,
                                   const double v);

  private:
    /// Initial setup.
    /// This is used mostly to precompute matrices that do not change during the simulation,
    /// such as the local stiffness of each element (if any), the mass, etc.
    virtual void SetupInitial(ChSystem* system) override;

    //// TEST, to be made private

    int n_usednodes;           // for optimization. Number of non-nullptr nodes
    int nodes_used_to_six[6];  // for optimization. Maps [0,n_usednodes) to six nodes index [0..6). Padded with -1 after
                               // n_usednodes.

  public:
    std::vector<std::shared_ptr<ChNodeFEAxyz> > m_nodes;  ///< element nodes

    std::vector<Layer> m_layers;     ///< element layers
    std::vector<double> m_layers_z;  ///< layer separation z values (not scaled, default centered tot thickness)

    double tot_thickness;  ///< total element thickness

    ChMatrixNM<double, 2, 2> Jux;  ///< jacobian [d{u,v}/d{x,y}], as inverse of [d{x,y}/d{u,v}]

    double area;    ///< initial element triangle area
    ChVector3d l0;  ///< initial lengths

    ChVector3d cM[3];  ///< cM coefficients
    ChVector3d cI[3];  ///< cI coefficients
    ChVector3d rI;     ///< rI coefficients = RIi/(RMi/RIi) = (1/hIi)/((1/hMi)(1/hIi))
    ChVector3d phi0;   ///< initial edge bendings
    ChVector3d phi;    ///< actual edge bendings (last computed)

    ChVector3d k0;  ///< initial curvature (not needed?)
    ChVector3d e0;  ///< initial strain

    ChVector3d k;  ///< actual curvature (last computed)
    ChVector3d e;  ///< actual strain (last computed)

    ChVector3d n;  ///< actual stress, membrane (last computed)
    ChVector3d m;  ///< actual stress, bending (last computed)

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  public:
    // Interface to ChElementBase base class
    // -------------------------------------

    /// Fill the D vector with the current field values at thenodes of the element, with proper ordering.
    /// If the D vector has not the size of this->GetNumCoordsPosLevel(), it will be resized.
    ///  {x_a y_a z_a  x_b y_b z_b ....}
    virtual void GetStateBlock(ChVectorDynamic<>& mD) override;

    // Set H as a linear combination of M, K, and R.
    //   H = Mfactor * [M] + Kfactor * [K] + Rfactor * [R],
    // where [M] is the mass matrix, [K] is the stiffness matrix, and [R] is the damping matrix.
    virtual void ComputeKRMmatricesGlobal(ChMatrixRef H,
                                          double Kfactor,
                                          double Rfactor = 0,
                                          double Mfactor = 0) override;

    /// Computes the internal forces.
    /// (E.g. the actual position of nodes is not in relaxed reference position) and set values in the Fi vector.
    virtual void ComputeInternalForces(ChVectorDynamic<>& Fi) override;

    void ComputeInternalForces_impl(
        ChVectorDynamic<>& Fi,                 ///< vector of internal forces
        ChState& state_x,                      ///< state position to evaluate Fi
        ChStateDelta& state_w,                 ///< state speed to evaluate Fi
        bool used_for_differentiation = false  ///< true if called during finite-difference Jacobian approximation
    );

    /// Update the state of this element.
    virtual void Update() override;

    // The default implementation in ChElementBase is ok, but inefficient because it passes
    // through the computation of the M mass matrix via ComputeKRMmatricesGlobal(H,0,0,M).
    void EleIntLoadLumpedMass_Md(ChVectorDynamic<>& Md, double& error, const double c) override;

    // Interface to ChElementShell base class
    // --------------------------------------

    virtual void EvaluateSectionDisplacement(const double u,
                                             const double v,
                                             ChVector3d& u_displ,
                                             ChVector3d& u_rotaz) override;

    virtual void EvaluateSectionFrame(const double u, const double v, ChVector3d& point, ChQuaternion<>& rot) override;

    virtual void EvaluateSectionPoint(const double u, const double v, ChVector3d& point) override;

    virtual void EvaluateSectionVelNorm(double U, double V, ChVector3d& Result) override;

    virtual bool IsTriangleShell() override { return true; }

    // Internal computations
    // ---------------------

    /// Compute Jacobians of the internal forces.
    /// This function calculates a linear combination of the stiffness (K) and damping (R) matrices,
    ///     J = Kfactor * K + Rfactor * R
    /// for given coefficients Kfactor and Rfactor.
    /// This Jacobian will be further combined with the global mass matrix M and included in the global
    /// stiffness matrix H in the function ComputeKRMmatricesGlobal().
    void ComputeInternalJacobians(double Kfactor, double Rfactor);

    // Functions for ChLoadable interface
    // ----------------------------------

    /// Gets the number of DOFs affected by this element (position part).
    virtual unsigned int GetLoadableNumCoordsPosLevel() override { return n_usednodes * 3; }

    /// Gets the number of DOFs affected by this element (velocity part).
    virtual unsigned int GetLoadableNumCoordsVelLevel() override { return n_usednodes * 3; }

    /// Gets all the DOFs packed in a single vector (position part).
    virtual void LoadableGetStateBlockPosLevel(int block_offset, ChState& mD) override;

    /// Gets all the DOFs packed in a single vector (velocity part).
    virtual void LoadableGetStateBlockVelLevel(int block_offset, ChStateDelta& mD) override;

    /// Increment all DOFs using a delta.
    virtual void LoadableStateIncrement(const unsigned int off_x,
                                        ChState& x_new,
                                        const ChState& x,
                                        const unsigned int off_v,
                                        const ChStateDelta& Dv) override;

    /// Number of coordinates in the interpolated field, ex=3 for a
    /// tetrahedron finite element or a cable, = 1 for a thermal problem, etc.
    virtual unsigned int GetNumFieldCoords() override { return 3; }

    /// Get the number of DOFs sub-blocks.
    virtual unsigned int GetNumSubBlocks() override { return n_usednodes; }

    /// Get the offset of the specified sub-block of DOFs in global vector.
    virtual unsigned int GetSubBlockOffset(unsigned int nblock) override {
        return m_nodes[nodes_used_to_six[nblock]]->NodeGetOffsetVelLevel();
    }

    /// Get the size of the specified sub-block of DOFs in global vector.
    virtual unsigned int GetSubBlockSize(unsigned int nblock) override { return 3; }

    /// Check if the specified sub-block of DOFs is active.
    virtual bool IsSubBlockActive(unsigned int nblock) const override {
        return !m_nodes[nodes_used_to_six[nblock]]->IsFixed();
    }

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
    virtual ChVector3d ComputeNormal(const double U, const double V) override;

    virtual bool IsTriangleIntegrationNeeded() override { return true; }

    virtual bool IsTrianglePrismIntegrationNeeded() override { return true; }
};

/// @} fea_elements

}  // end of namespace fea
}  // end of namespace chrono

#endif
