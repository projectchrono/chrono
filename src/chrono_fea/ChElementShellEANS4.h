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

#include "chrono/core/ChQuadrature.h"
#include "chrono_fea/ChApiFEA.h"
#include "chrono_fea/ChElementShell.h"
#include "chrono_fea/ChNodeFEAxyzrot.h"
#include "chrono_fea/ChUtilsFEA.h"

namespace chrono {
namespace fea {

/// @addtogroup fea_elements
/// @{

// ----------------------------------------------------------------------------
/// Material definition.
/// This class implements material properties for a layer from the Reissner theory,
/// see Morandini, Masarati "Implementation and Validation of a 4-node shell element"
class ChApiFea ChMaterialShellEANS {
  public:
    /// Construct an isotropic material.
    ChMaterialShellEANS(double thickness, ///< thickness
                        double rho,  ///< material density
                        double E,    ///< Young's modulus
                        double nu,   ///< Poisson ratio
                        double alpha = 1.0,///< shear factor
                        double beta = 0.1 ///< torque factor
                        );

    /// Return the thickness
    double Get_thickness() const { return m_thickness; }

    /// Return the material density.
    double Get_rho() const { return m_rho; }
    /// Return the elasticity moduli
    double Get_E() const { return m_E; }
    /// Return the Poisson ratio
    double Get_nu() const { return m_nu; }
    /// Return the shear factor
    double Get_alpha() const { return m_alpha; }
    /// Return the torque factor
    double Get_beta() const { return m_beta; }

    /// The FE code will evaluate this function to compute 
    /// u,v stresses/torques given the u,v strains/curvatures.
    /// You can inherit a more sophisticated material that override this (ex. for
    /// orthotropic materials, etc.)
    virtual void ComputeStress(ChVector<>& n_u, 
                               ChVector<>& n_v,
                               ChVector<>& m_u, 
                               ChVector<>& m_v,
                               const ChVector<>& eps_u, 
                               const ChVector<>& eps_v,
                               const ChVector<>& kur_u, 
                               const ChVector<>& kur_v);
  private: 
    double m_thickness;             ///< thickness
    double m_rho;                  ///< density
    double m_E;                      ///< elasticity moduli
    double m_nu;                     ///< Poisson ratio
    double m_alpha;                  ///< shear factor
    double m_beta ;                  ///< torque factor
};

// ----------------------------------------------------------------------------
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
class ChApiFea ChElementShellEANS4 : public ChElementShell, public ChLoadableUV, public ChLoadableUVW {
  public:
    ChElementShellEANS4();
    ~ChElementShellEANS4() {}

    /// Definition of a layer
    class Layer {
      public:
        /// Return the layer thickness.
        double Get_thickness() const { return m_thickness; }

        /// Return the fiber angle.
        double Get_theta() const { return m_theta; }

        /// Return the layer material.
        std::shared_ptr<ChMaterialShellEANS> GetMaterial() const { return m_material; }

      private:
        /// Private constructor (a layer can be created only by adding it to an element)
        Layer(ChElementShellEANS4* element,                  ///< containing element
              double thickness,                              ///< layer thickness
              double theta,                                  ///< fiber angle
              std::shared_ptr<ChMaterialShellEANS> material  ///< layer material
              );

        /// Initial setup for this layer
        void SetupInitial();

        ChElementShellEANS4* m_element;                   ///< containing shell element
        std::shared_ptr<ChMaterialShellEANS> m_material;  ///< layer material
        double m_thickness;                               ///< layer thickness
        double m_theta;                                   ///< fiber angle

        friend class ChElementShellEANS4;
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
    void AddLayer(double thickness,                              ///< layer thickness
                  double theta,                                  ///< fiber angle (radians)
                  std::shared_ptr<ChMaterialShellEANS> material  ///< layer material
                  );

    /// Get the number of layers.
    size_t GetNumLayers() const { return m_numLayers; }

    /// Get a handle to the specified layer.
    const Layer& GetLayer(size_t i) const { return m_layers[i]; }

    /// Set the structural damping.
    void SetAlphaDamp(double a) { m_Alpha = a; }

    /// Get the element length in the X direction.
    double GetLengthX() const { return m_lenX; }
    /// Get the element length in the Y direction.
    double GetLengthY() const { return m_lenY; }
    /// Get the total thickness of the shell element.
    double GetThickness() { return m_thickness; }

    ChQuaternion<> GetAvgRot() {return Tavg;}

    // Shape functions
    // ---------------

    /// Fills the N shape function matrix.
    void ShapeFunctions(ChMatrix<>& N, double x, double y, double z);

    /// Fills the Nx shape function derivative matrix with respect to X.
    void ShapeFunctionsDerivativeX(ChMatrix<>& Nx, double x, double y, double z);

    /// Fills the Ny shape function derivative matrix with respect to Y.
    void ShapeFunctionsDerivativeY(ChMatrix<>& Ny, double x, double y, double z);

    // [ANS] Shape function for Assumed Naturals Strain (Interpolation of strain and strainD in a thickness direction)
    void ShapeFunctionANSbilinearShell(ChMatrix<>& S_ANS, double x, double y);

    // [ANS] Calculate the ANS strain and strain derivatives, at given node states
    void CalcStrainANSbilinearShell(const ChVector<>& pA, const ChQuaternion<>& rA,
                                    const ChVector<>& pB, const ChQuaternion<>& rB,
                                    const ChVector<>& pC, const ChQuaternion<>& rC,
                                    const ChVector<>& pD, const ChQuaternion<>& rD);

  private:

    //
    // DATA
    //

    std::vector<std::shared_ptr<ChNodeFEAxyzrot> > m_nodes;///< element nodes
    std::vector<Layer> m_layers;                           ///< element layers
    size_t m_numLayers;                                    ///< number of layers for this element
    double m_thickness;                                    ///< total element thickness
    double m_lenX;                                         ///< element length in X direction
    double m_lenY;                                         ///< element length in Y direction
    double m_Alpha;                                        ///< structural damping
    std::vector<double> m_GaussZ;                          ///< layer separation z values (scaled to [-1,1])
    ChMatrixNM<double, 24, 24> m_MassMatrix;               ///< mass matrix
    ChMatrixNM<double, 24, 24> m_JacobianMatrix;           ///< Jacobian matrix (Kfactor*[K] + Rfactor*[R])

    enum constants
    {
       NUMNO=4, // number of nodes 
       NUMGP=4, // number of gauss points
       NUMSP=4, // number of shear stitching points points
    };

    ChMatrixNM<double, 6,   4> m_strainANS;                ///< ANS strains at shear stitching points
    ChMatrixNM<double, 4,  24> m_B3_ANS;                   ///< ANS B matrix at shear stitching points (shear only)
    ChMatrixNM<double, 4,  24> m_B6_ANS;                   ///< ANS B matrix at shear stitching points (shear only)

    std::array<ChQuaternion<>, NUMNO> iTa;                 ///< inverse of reference rotations at nodes
    std::array<ChQuaternion<>, NUMGP> iTa_i;               ///< inverse of reference rotations at gauss points
    std::array<ChQuaternion<>, NUMSP> iTa_S;               ///< inverse of reference rotations at shear points

    std::array<ChQuaternion<>, NUMGP> T_i0;                ///< initial rotations at gauss points
    std::array<ChQuaternion<>, NUMSP> T_S0;                ///< initial rotations at shear stitching points
    double alpha_i[NUMGP];                                 ///< determinant of jacobian at gauss points 
    std::array<ChMatrixNM<double,4,2>, NUMGP> L_alpha_beta_i; ///< precomputed matrices at gauss points
    std::array<ChMatrixNM<double,4,2>, NUMGP> L_alpha_beta_S; ///< precomputed matrices at shear stitching points

    ChQuaternion<> Tavg;                                  ///< average rot 

    // static data - not instanced per each shell :

    static double xi_i[NUMGP][2]; // gauss points coords
	static double  w_i[NUMGP];    // gauss points weights
    
    static double xi_S[NUMSP][2]; // shear stitching points coords
    static double xi_n[NUMNO][2]; // nodes coords

    
  private:

    void ComputeNodeAndAverageRotations(
        const ChQuaternion<>& mrA, const ChQuaternion<>& mrB,
        const ChQuaternion<>& mrC, const ChQuaternion<>& mrD,
        ChQuaternion<>& mTa, ChQuaternion<>& mTb, 
        ChQuaternion<>& mTc, ChQuaternion<>& mTd,
        ChVector<>& mF_relA, ChVector<>& mF_relB,
        ChVector<>& mF_relC, ChVector<>& mF_relD);

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

    void ComputeInternalForces_Impl(const ChVector<>& pA, const ChQuaternion<>& rA,
                                    const ChVector<>& pB, const ChQuaternion<>& rB,
                                    const ChVector<>& pC, const ChQuaternion<>& rC,
                                    const ChVector<>& pD, const ChQuaternion<>& rD,
                                    ChMatrixDynamic<>& Fi) ;

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
    virtual int LoadableGet_ndof_x() { return 4 * 7; }

    /// Gets the number of DOFs affected by this element (velocity part).
    virtual int LoadableGet_ndof_w() { return 4 * 6; }

    /// Gets all the DOFs packed in a single vector (position part).
    virtual void LoadableGetStateBlock_x(int block_offset, ChVectorDynamic<>& mD) override;

    /// Gets all the DOFs packed in a single vector (velocity part).
    virtual void LoadableGetStateBlock_w(int block_offset, ChVectorDynamic<>& mD) override;

    /// Number of coordinates in the interpolated field, ex=3 for a
    /// tetrahedron finite element or a cable, = 1 for a thermal problem, etc.
    virtual int Get_field_ncoords() { return 6; }

    /// Tell the number of DOFs blocks (ex. =1 for a body, =4 for a tetrahedron, etc.)
    virtual int GetSubBlocks() { return 4; }

    /// Get the offset of the i-th sub-block of DOFs in global vector.
    virtual unsigned int GetSubBlockOffset(int nblock) { return m_nodes[nblock]->NodeGetOffset_w(); }

    /// Get the size of the i-th sub-block of DOFs in global vector.
    virtual unsigned int GetSubBlockSize(int nblock) { return 6; }

    virtual void EvaluateSectionVelNorm(double U, double V, ChVector<>& Result) override;

    /// Get the pointers to the contained ChLcpVariables, appending to the mvars vector.
    virtual void LoadableGetVariables(std::vector<ChLcpVariables*>& mvars) override;

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
