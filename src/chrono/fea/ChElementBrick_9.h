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
// Authors: Radu Serban
// =============================================================================
// Brick element with 9 nodes (central node for curvature)
// =============================================================================

#ifndef CHELEMENTBRICK9_H
#define CHELEMENTBRICK9_H

#include "chrono/physics/ChContinuumMaterial.h"
#include "chrono/physics/ChSystem.h"
#include "chrono/fea/ChElementGeneric.h"
#include "chrono/fea/ChNodeFEAcurv.h"
#include "chrono/fea/ChNodeFEAxyz.h"

namespace chrono {
namespace fea {

/// @addtogroup fea_elements
/// @{

/// Brick element with 9 nodes.
class ChApi ChElementBrick_9 : public ChElementGeneric, public ChLoadableUVW {
  public:
    ChElementBrick_9();
    ~ChElementBrick_9() {}

    /// Get number of nodes of this element.
    virtual int GetNnodes() override { return 9; }

    /// Get number of degrees of freedom of this element.
    virtual int GetNdofs() override { return 8 * 3 + 9; }

    /// Get the number of coordinates from the n-th node used by this element.
    virtual int GetNodeNdofs(int n) override {
        if (n < 8)
            return 3;

        return 9;
    }

    /// Access the n-th node of this element.
    virtual std::shared_ptr<ChNodeFEAbase> GetNodeN(int n) override {
        if (n < 8)
            return m_nodes[n];

        return m_central_node;
    }

    /// Specify the nodes of this element.
    void SetNodes(std::shared_ptr<ChNodeFEAxyz> node1,
                  std::shared_ptr<ChNodeFEAxyz> node2,
                  std::shared_ptr<ChNodeFEAxyz> node3,
                  std::shared_ptr<ChNodeFEAxyz> node4,
                  std::shared_ptr<ChNodeFEAxyz> node5,
                  std::shared_ptr<ChNodeFEAxyz> node6,
                  std::shared_ptr<ChNodeFEAxyz> node7,
                  std::shared_ptr<ChNodeFEAxyz> node8,
                  std::shared_ptr<ChNodeFEAcurv> nodeC);

    /// Get access to individual nodes of this element.
    std::shared_ptr<ChNodeFEAxyz> GetNode1() const { return m_nodes[0]; }
    std::shared_ptr<ChNodeFEAxyz> GetNode2() const { return m_nodes[1]; }
    std::shared_ptr<ChNodeFEAxyz> GetNode3() const { return m_nodes[2]; }
    std::shared_ptr<ChNodeFEAxyz> GetNode4() const { return m_nodes[3]; }
    std::shared_ptr<ChNodeFEAxyz> GetNode5() const { return m_nodes[4]; }
    std::shared_ptr<ChNodeFEAxyz> GetNode6() const { return m_nodes[5]; }
    std::shared_ptr<ChNodeFEAxyz> GetNode7() const { return m_nodes[6]; }
    std::shared_ptr<ChNodeFEAxyz> GetNode8() const { return m_nodes[7]; }
    std::shared_ptr<ChNodeFEAcurv> GetCentralNode() const { return m_central_node; }

    /// Strain type calculation.
    enum StrainFormulation {
        GreenLagrange,  ///< Green-Lagrange strain formulation
        Hencky          ///< Hencky strain
    };

    /// Plasticity formulation.
    enum PlasticityFormulation {
        J2,                ///< J2 plasticity (metals)
        DruckerPrager,     ///< Drucker-Prager plasticity (soil)
        DruckerPrager_Cap  ///< Drucker-Prager-Cap plasticity (soil)
    };
    /// Maximum number of iteration for DP return mapping
    int m_DP_iterationNo;
    /// Tolerance for yield function value (Drucker-Prager)
    double m_DP_yield;
    double m_DPCapBeta;  ///  DP_Cap parameter
    /// Set element dimensions (x, y, z directions).
    void SetDimensions(const ChVector<>& dims) { m_dimensions = dims; }
    /// Get the element dimensions (x, y, z directions).
    const ChVector<>& GetDimensions() const { return m_dimensions; }

    /// Set the continuum material for this element.
    void SetMaterial(std::shared_ptr<ChContinuumElastic> material) { m_material = material; }
    /// Get a handle to the continuum material used by this element.
    std::shared_ptr<ChContinuumElastic> GetMaterial() const { return m_material; }

    /// Enable/disable internal gravity calculation.
    void SetGravityOn(bool val) { m_gravity_on = val; }
    /// Check if internal gravity calculation is enabled/disabled.
    bool IsGravityOn() const { return m_gravity_on; }
    /// Set the structural damping.
    void SetAlphaDamp(double a) { m_Alpha = a; }
    /// Set the strain formulation.
    void SetStrainFormulation(StrainFormulation model) { m_strain_form = model; }
    /// Get the strain formulation.
    StrainFormulation GetStrainFormulation() const { return m_strain_form; }
    /// Set the plasticity formulation.
    void SetPlasticityFormulation(PlasticityFormulation model) { m_plast_form = model; }
    /// Get the plasticity formulation.
    PlasticityFormulation GetPlasticityFormulation() const { return m_plast_form; }
    /// Set the DP iteration number.
    void SetDPIterationNo(int ItNo) { m_DP_iterationNo = ItNo; }
    /// Set the hardening parameter look-up table
    void SetDPVector1(ChMatrixDynamic<double> vec) { m_DPVector1 = vec; }
    void SetDPVector2(ChMatrixDynamic<double> vec) { m_DPVector2 = vec; }
    void SetDPVectorSize(int a) { m_DPVector_size = a; }
    /// Set DP_Cap parameter
    void SetDPCapBeta(double a) { m_DPCapBeta = a; }
    /// Get the DP iteration number.
    int GetDPIterationNo() const { return m_DP_iterationNo; }
    /// Set the DP yield function tolerance.
    void SetDPYieldTol(double yieldf_tol) { m_DP_yield = yieldf_tol; }
    /// Get the DP yield function tolerance.
    double GetDPYieldTol() const { return m_DP_yield; }
    /// Set plasticity.
    void SetPlasticity(bool val) { m_Plasticity = val; }
    /// Set Drucker-Prager plasticity.
    void SetDruckerPrager(bool val) { m_DP = val; }
    /// Set yield stress for yield function.
    void SetYieldStress(double a) { m_YieldStress = a; }
    /// Set linear isotropic modulus.
    void SetHardeningSlope(double a) { m_HardeningSlope = a; }
    /// Set internal friction angle.
    void SetFriction(double friction) { m_FrictionAngle = friction; }
    /// Set dilatancy angle.
    void SetDilatancy(double dilatancy) { m_DilatancyAngle = dilatancy; }
    /// Set Drucker-Prager hardening type.
    void SetDPType(int a) { m_DPHardening = a; }
    /// Set initial strain tensor per integration point of the 9-node element.
    void SetCCPInitial(ChMatrixNM<double, 9, 8> mat) { m_CCPinv_Plast = mat; }

    /// Calculate shape functions and their derivatives.
    ///   N = [N1, N2, N3, N4, ...]                               (1x11 row vector)
    ///   S = [N1*eye(3), N2*eye(3), N3*eye(3) ,N4*eye(3), ...]   (3x11 matrix)
    void ShapeFunctions(ChMatrix<>& N, double x, double y, double z);
    /// Calculate shape function derivative w.r.t. X.
    void ShapeFunctionsDerivativeX(ChMatrix<>& Nx, double x, double y, double z);
    /// Calculate shape function derivative w.r.t. Y.
    void ShapeFunctionsDerivativeY(ChMatrix<>& Ny, double x, double y, double z);
    /// Calculate shape function derivative w.r.t. Z.
    void ShapeFunctionsDerivativeZ(ChMatrix<>& Nz, double x, double y, double z);

    /// Number of coordinates in the interpolated field: here the {x,y,z} displacement.
    virtual int Get_field_ncoords() override { return 3; }

    /// Tell the number of DOFs blocks: here 9, 1 for each node.
    virtual int GetSubBlocks() override { return 9; }

    /// Get the offset of the i-th sub-block of DOFs in global vector.
    virtual unsigned int GetSubBlockOffset(int nblock) override {
        if (nblock < 8)
            return m_nodes[nblock]->NodeGetOffset_w();

        return m_central_node->NodeGetOffset_w();
    }

    /// Get the size of the i-th sub-block of DOFs in global vector.
    virtual unsigned int GetSubBlockSize(int nblock) override {
        if (nblock < 8)
            return 3;

        return 9;
    }

    /// Get the number of DOFs affected by this element (position part).
    virtual int LoadableGet_ndof_x() override { return 8 * 3 + 9; }

    /// Get the number of DOFs affected by this element (speed part).
    virtual int LoadableGet_ndof_w() override { return 8 * 3 + 9; }

    /// Get all the DOFs packed in a single vector (position part).
    virtual void LoadableGetStateBlock_x(int block_offset, ChState& mD) override;

    /// Get all the DOFs packed in a single vector (speed part).
    virtual void LoadableGetStateBlock_w(int block_offset, ChStateDelta& mD) override;

    /// Increment all DOFs using a delta.
    virtual void LoadableStateIncrement(const unsigned int off_x, ChState& x_new, const ChState& x, const unsigned int off_v, const ChStateDelta& Dv) override;

    /// Get the pointers to the contained ChLcpVariables, appending to the mvars vector.
    virtual void LoadableGetVariables(std::vector<ChVariables*>& mvars) override;

    /// Evaluate N'*F, where N is some type of shape function evaluated at (U,V,W).
    /// Here, U,V,W are coordinates of the volume, each ranging in -1..+1
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

    /// Return the material density.
    /// This is needed so that it can be accessed by ChLoaderVolumeGravity.
    virtual double GetDensity() override { return this->m_material->Get_density(); }

  private:
    // -----------------------------------
    // Data
    // -----------------------------------

    std::vector<std::shared_ptr<ChNodeFEAxyz>> m_nodes;  ///< corner element nodes
    std::shared_ptr<ChNodeFEAcurv> m_central_node;       ///< central node

    std::shared_ptr<ChContinuumElastic> m_material;  ///< elastic naterial

    ChVector<> m_dimensions;                      ///< element dimensions (x, y, z components)
    bool m_gravity_on;                            ///< enable/disable internal gravity calculation
    ChMatrixNM<double, 33, 1> m_GravForce;        ///< gravitational force
    ChMatrixNM<double, 33, 33> m_MassMatrix;      ///< mass matrix
    ChMatrixNM<double, 33, 33> m_JacobianMatrix;  ///< Jacobian matrix (Kfactor*[K] + Rfactor*[R])
    double m_GaussScaling;
    double m_Alpha;                      ///< structural damping
    ChMatrixNM<double, 11, 3> m_d0;      ///< initial nodal coordinates (in matrix form)
    ChMatrixNM<double, 11, 3> m_d;       ///< current nodal coordinates
    ChMatrixNM<double, 11, 11> m_ddT;    ///< matrix m_d * m_d^T
    ChMatrixNM<double, 11, 11> m_d0d0T;  ///< matrix m_d0 * m_d0^T
    ChMatrixNM<double, 33, 1> m_d_dt;    ///< current nodal velocities
    double m_FrictionAngle;   ///< Drucker-Prager Friction Angle Beta
    double m_DilatancyAngle;  ///< Drucker-Prager Dilatancy Angle Phi
    int m_DPHardening;        ///< Drucker-Prager Hardening Type

    StrainFormulation m_strain_form;     ///< Enum for strain formulation
    PlasticityFormulation m_plast_form;  ///< Enum for plasticity formulation
    bool m_Hencky;                       ///< flag activating Hencky strain formulation
    bool m_Plasticity;                   ///< flag activating Plastic deformation
    bool m_DP;                           ///< flag activating Drucker-Prager formulation

    double m_YieldStress;                     ///< plastic yield stress
    double m_HardeningSlope;                  ///< plastic hardening slope
    ChMatrixNM<double, 8, 1> m_Alpha_Plast;   ///< hardening alpha parameter
    ChMatrixNM<double, 9, 8> m_CCPinv_Plast;  ///< strain tensor for each integration point
    int m_InteCounter;                        ///< Integration point counter (up to 8)

    ChVectorDynamic<double> m_DPVector1;  /// xtab of hardening parameter look-up table
    ChVectorDynamic<double> m_DPVector2;  /// ytab of hardening parameter look-up table
    int m_DPVector_size;                  /// row number n of hardening parameter look-up table

    // -----------------------------------
    // Interface to base classes
    // -----------------------------------

    /// Update this element.
    virtual void Update() override;

    /// Fill the D vector (column matrix) with the current states of the element nodes.
    virtual void GetStateBlock(ChMatrixDynamic<>& mD) override;

    /// Initial element setup.
    virtual void SetupInitial(ChSystem* system) override;
    /// Set M as the global mass matrix.
    virtual void ComputeMmatrixGlobal(ChMatrix<>& M) override;
    /// Set H as the global stiffness matrix K, scaled  by Kfactor. Optionally, also
    /// superimposes global damping matrix R, scaled by Rfactor, and global mass matrix M multiplied by Mfactor.
    virtual void ComputeKRMmatricesGlobal(ChMatrix<>& H,
                                          double Kfactor,
                                          double Rfactor = 0,
                                          double Mfactor = 0) override;

    /// Compute internal forces and load them in the Fi vector.
    virtual void ComputeInternalForces(ChMatrixDynamic<>& Fi) override;

    // -----------------------------------
    // Functions for internal computations
    // -----------------------------------

    /// Compute the mass matrix of the element.
    void ComputeMassMatrix();
    /// Compute the gravitational forces.
    void ComputeGravityForce(const ChVector<>& g_acc);

    /// Compute Jacobians of the internal forces.
    /// This function calculates a linear combination of the stiffness (K) and damping (R) matrices,
    ///     J = Kfactor * K + Rfactor * R
    /// for given coefficients Kfactor and Rfactor.
    /// This Jacobian will be further combined with the global mass matrix M and included in the global
    /// stiffness matrix H in the function ComputeKRMmatricesGlobal().
    void ComputeInternalJacobians(double Kfactor, double Rfactor);

    /// Calculate the determinant of the initial configuration.
    double Calc_detJ0(double x, double y, double z);

    /// Calculate the determinant of the initial configuration.
    /// Same as above, but also return the dense shape function vector derivatives.
    double Calc_detJ0(double x,
                      double y,
                      double z,
                      ChMatrixNM<double, 1, 11>& Nx,
                      ChMatrixNM<double, 1, 11>& Ny,
                      ChMatrixNM<double, 1, 11>& Nz,
                      ChMatrixNM<double, 1, 3>& Nx_d0,
                      ChMatrixNM<double, 1, 3>& Ny_d0,
                      ChMatrixNM<double, 1, 3>& Nz_d0);

    // Calculate the current 11x3 matrix of nodal coordinates.
    void CalcCoordMatrix(ChMatrixNM<double, 11, 3>& d);

    // Calculate the current 33x1 matrix of nodal coordinate derivatives.
    void CalcCoordDerivMatrix(ChMatrixNM<double, 33, 1>& dt);

    void ComputeStrainD_Brick9(ChMatrixNM<double, 6, 33>& strainD,
                               ChMatrixNM<double, 1, 11> Nx,
                               ChMatrixNM<double, 1, 11> Ny,
                               ChMatrixNM<double, 1, 11> Nz,
                               ChMatrixNM<double, 3, 3> FI,
                               ChMatrixNM<double, 3, 3> J0I);

    void ComputeHardening_a(double& MeanEffP,
                            double& Hi,
                            double alphUp,
                            ChVectorDynamic<double> m_DPVector1,
                            ChVectorDynamic<double> m_DPVector2,
                            int m_DPVector_size);

    friend class MyMassBrick9;
    friend class MyGravityBrick9;
    friend class MyForceBrick9;
    friend class MyJacobianBrick9;
};

/// @} fea_elements

}  // end namespace fea
}  // end namespace chrono

#endif
