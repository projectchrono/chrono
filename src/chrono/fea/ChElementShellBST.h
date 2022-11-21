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
class ChApi ChElementShellBST : public ChElementShell , public ChLoadableUV, public ChLoadableUVW 
{
  public:
    using ShapeVector = ChMatrixNM<double, 1, 3>;

    ChElementShellBST();
    ~ChElementShellBST();

    /// Definition of a layer
    class Layer {
      public:
        /// Return the layer thickness.
        double Get_thickness() const { return m_thickness; }

        /// Return the fiber angle.
        double Get_theta() const { return m_theta; }

        /// Return the layer material.
        std::shared_ptr<ChMaterialShellKirchhoff> GetMaterial() const { return m_material; }

      private:
        /// Private constructor (a layer can be created only by adding it to an element)
        Layer(ChElementShellBST* element,                        ///< containing element
              double thickness,                                  ///< layer thickness
              double theta,                                      ///< fiber angle
              std::shared_ptr<ChMaterialShellKirchhoff> material ///< layer material
              );

        /// Initial setup for this layer
        void SetupInitial();

        ChElementShellBST* m_element;                         ///< containing shell element
        std::shared_ptr<ChMaterialShellKirchhoff> m_material; ///< layer material
        double m_thickness;                                   ///< layer thickness
        double m_theta;                                       ///< fiber angle

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
    virtual int GetNnodes() override { return n_usednodes; }

    /// Get the number of coordinates in the field used by the referenced nodes.
    virtual int GetNdofs() override { return n_usednodes * 3; }

    /// Get the number of coordinates from the n-th node used by this element.
    virtual int GetNodeNdofs(int n) override { return 3; }


    /// Access the n-th node of this element, not considering those marked with "nullptr" ex. if at boundary
    virtual std::shared_ptr<ChNodeFEAbase> GetNodeN(int n) override { return m_nodes[nodes_used_to_six[n]]; }

    /// Get a handle to the n node of this element, among the three of the triangle part, n=0..2
    std::shared_ptr<ChNodeFEAxyz> GetNodeTriangleN(int n) const { return m_nodes[n]; }

	/// Get a handle to the n node from the three of the neighbouring triangles, n=0..2.  Even if nullptr.
    std::shared_ptr<ChNodeFEAxyz> GetNodeNeighbourN(int n) const { return m_nodes[3+n]; }

    /// Sets the neutral rotations of nodes at once,
    /// assuming the current element position is for zero strain.
    void SetAsNeutral();

    /// Add a layer.
    /// By default, when adding more than one layer, the reference z level of the
    /// shell element is centered along the total thickness.
    void AddLayer(double thickness,                                  ///< layer thickness
                  double theta,                                      ///< fiber angle (radians)
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
	/// The jacobian Jux = [d{u,v}/d{x,y}] must be provided, often from inverse of Jxu = [d{x,y}/d{u,v}],
	/// so that [Nx;Ny]=[Jux]*[Nu;Nv].   Note, may remove u,v parms cause constant
    void ShapeFunctionsDerivativeX(ShapeVector& Nx, const ChMatrixNM<double,2,2>& Jux, const double u, const double v);

    /// Fills the Ny shape function derivative matrix with respect to Y local element direction. 
	/// The jacobian Jux = [d{u,v}/d{x,y}] must be provided, often from inverse of Jxu = [d{x,y}/d{u,v}],
	/// so that [Nx;Ny]=[Jux]*[Nu;Nv].   Note, may remove u,v parms cause constant
	void ShapeFunctionsDerivativeY(ShapeVector& Ny, const ChMatrixNM<double,2,2>& Jux, const double u, const double v);


  private:
    /// Initial setup.
    /// This is used mostly to precompute matrices that do not change during the simulation,
    /// such as the local stiffness of each element (if any), the mass, etc.
    virtual void SetupInitial(ChSystem* system) override;


    //***TEST*** to make private
	
	int n_usednodes;  // for optimization. Number of non-nullptr nodes
	int nodes_used_to_six[6];  // for optimization. Maps [0,n_usednodes) to six nodes index [0..6). Padded with -1 after n_usednodes.

  public:
    std::vector<std::shared_ptr<ChNodeFEAxyz> > m_nodes;  ///< element nodes

    std::vector<Layer> m_layers;     ///< element layers
    std::vector<double> m_layers_z;  ///< layer separation z values (not scaled, default centered tot thickness)

    double tot_thickness;  ///< total element thickness

    ChMatrixNM<double, 2, 2> Jux;  ///< jacobian [d{u,v}/d{x,y}], as inverse of [d{x,y}/d{u,v}]

    double area;    ///< initial element triangle area
    ChVector<> l0;  ///< initial lengths

    ChVector<> cM[3];  ///< cM coefficients
    ChVector<> cI[3];  ///< cI coefficients
    ChVector<> rI;     ///< rI coefficients = RIi/(RMi/RIi) = (1/hIi)/((1/hMi)(1/hIi))
    ChVector<> phi0;   ///< initial edge bendings
    ChVector<> phi;    ///< actual edge bendings (last computed)

    ChVector<> k0;  ///< initial curvature (not needed?)
    ChVector<> e0;  ///< initial strain

    ChVector<> k;  ///< actual curvature (last computed)
    ChVector<> e;  ///< actual strain (last computed)

    ChVector<> n;  ///< actual stress, membrane (last computed)
    ChVector<> m;  ///< actual stress, bending (last computed)

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  public:
    // Interface to ChElementBase base class
    // -------------------------------------

    /// Fill the D vector with the current field values at thenodes of the element, with proper ordering.
    /// If the D vector has not the size of this->GetNdofs_x(), it will be resized.
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

    // Interface to ChElementShell base class
    // --------------------------------------

    virtual void EvaluateSectionDisplacement(const double u,
                                             const double v,
                                             ChVector<>& u_displ,
                                             ChVector<>& u_rotaz) override;

    virtual void EvaluateSectionFrame(const double u,
                                      const double v,
                                      ChVector<>& point,
                                      ChQuaternion<>& rot) override;

    virtual void EvaluateSectionPoint(const double u,
                                      const double v,
                                      ChVector<>& point) override;

	virtual void EvaluateSectionVelNorm(double U, double V, ChVector<>& Result) override;

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
    virtual int LoadableGet_ndof_x() override { return n_usednodes * 3; }

    /// Gets the number of DOFs affected by this element (velocity part).
    virtual int LoadableGet_ndof_w() override { return n_usednodes * 3; }

    /// Gets all the DOFs packed in a single vector (position part).
    virtual void LoadableGetStateBlock_x(int block_offset, ChState& mD) override;

    /// Gets all the DOFs packed in a single vector (velocity part).
    virtual void LoadableGetStateBlock_w(int block_offset, ChStateDelta& mD) override;

    /// Increment all DOFs using a delta.
    virtual void LoadableStateIncrement(const unsigned int off_x, ChState& x_new, const ChState& x, const unsigned int off_v, const ChStateDelta& Dv) override;

    /// Number of coordinates in the interpolated field, ex=3 for a
    /// tetrahedron finite element or a cable, = 1 for a thermal problem, etc.
    virtual int Get_field_ncoords() override { return 3; }

    /// Get the number of DOFs sub-blocks.
    virtual int GetSubBlocks() override { return n_usednodes; }

    /// Get the offset of the specified sub-block of DOFs in global vector.
    virtual unsigned int GetSubBlockOffset(int nblock) override {
        return m_nodes[nodes_used_to_six[nblock]]->NodeGetOffsetW();
    }

    /// Get the size of the specified sub-block of DOFs in global vector.
    virtual unsigned int GetSubBlockSize(int nblock) override { return 3; }

    /// Check if the specified sub-block of DOFs is active.
    virtual bool IsSubBlockActive(int nblock) const override { return !m_nodes[nodes_used_to_six[nblock]]->IsFixed(); }

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

	virtual bool IsTriangleIntegrationNeeded() override { return true; }

	virtual bool IsTrianglePrismIntegrationNeeded() override { return true; }

};

/// @} fea_elements

}  // end of namespace fea
}  // end of namespace chrono

#endif
