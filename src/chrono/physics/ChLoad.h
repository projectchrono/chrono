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
// Authors: Alessandro Tasora, Radu Serban
// =============================================================================

#ifndef CHLOAD_H
#define CHLOAD_H

#include "chrono/core/ChApiCE.h"

#include "chrono/physics/ChLoader.h"
#include "chrono/physics/ChLoaderU.h"
#include "chrono/physics/ChLoaderUV.h"
#include "chrono/physics/ChLoaderUVW.h"
#include "chrono/physics/ChObject.h"

#include "chrono/solver/ChKRMBlock.h"
#include "chrono/solver/ChSystemDescriptor.h"

#include "chrono/timestepper/ChState.h"

namespace chrono {

/// Utility class for storing Jacobian matrices.
/// This is automatically managed by the ChLoad, if needed (ie. for stiff loads)
class ChApi ChLoadJacobians {
  public:
    ChKRMBlock KRM;             ///< sum of K,R,M, with pointers to sparse variables
    ChMatrixDynamic<double> K;  ///< dQ/dx
    ChMatrixDynamic<double> R;  ///< dQ/dv
    ChMatrixDynamic<double> M;  ///< dQ/da

    /// Set references to the associated variables, automatically creating/resizing the KRM matrix as needed.
    void SetVariables(std::vector<ChVariables*> mvariables);
};

// -----------------------------------------------------------------------------

/// Base class for loads.
/// This class can be inherited to implement applied loads to bodies, finite elements, etc.
/// A load is an object that might wrap one or more ChLoader objects, whose value is dependent.
/// It implements functionalities to perform automatic differentiation of the load so it optionally can compute the
/// Jacobian (the tangent stiffness matrix of the load) that can be used in implicit integrators, statics, etc.
class ChApi ChLoadBase : public ChObj {
  public:
    ChLoadBase();
    virtual ~ChLoadBase();

    /// Gets the number of DOFs affected by this load (position part).
    virtual int LoadGetNumCoordsPosLevel() = 0;

    /// Gets the number of DOFs affected by this load (speed part).
    virtual int LoadGetNumCoordsVelLevel() = 0;

    /// Gets all the current DOFs packed in a single vector (position part).
    virtual void LoadGetStateBlock_x(ChState& mD) = 0;

    /// Gets all the current DOFs packed in a single vector (speed part).
    virtual void LoadGetStateBlock_w(ChStateDelta& mD) = 0;

    /// Increment a packed state (e.g., as obtained by LoadGetStateBlock_x()) by a given packed state-delta.
    /// Compute: x_new = x + dw. This method is used in calculating Jacobians with finite difference approximations
    /// (default implementations of ComputeJacobian).
    virtual void LoadStateIncrement(const ChState& x, const ChStateDelta& dw, ChState& x_new) = 0;

    /// Number of coordinates in the interpolated field.
    /// For example, 3 for a tetrahedron finite element or a cable, 1 for a thermal problem, etc.
    virtual int LoadGetNumFieldCoords() = 0;

    /// Compute the generalized load(s).
    /// A derived class should cache the resulting Q which is then used in LoadIntLoadResidual_F.
    virtual void ComputeQ(ChState* state_x,      ///< state position to evaluate Q
                          ChStateDelta* state_w  ///< state speed to evaluate Q
                          ) = 0;

    /// Compute the K=-dQ/dx, R=-dQ/dv, M=-dQ/da Jacobians.
    /// Implementation in a derived class should load the Jacobian matrices K, R, M in the structure 'm_jacobians'.
    /// Note the sign that is flipped because we assume equations are written with Q moved to left-hand side.
    virtual void ComputeJacobian(ChState* state_x,      ///< state position to evaluate Jacobians
                                 ChStateDelta* state_w  ///< state speed to evaluate Jacobians
                                 ) = 0;

    /// Access the Jacobians (if any, i.e. if this is a stiff load).
    ChLoadJacobians* GetJacobians() { return m_jacobians; }

    /// Create the Jacobian loads if needed and set the ChVariables referenced by the sparse KRM block.
    virtual void CreateJacobianMatrices() = 0;

    /// Update, called at least at each time step.
    /// - It recomputes the generalized load Q vector(s)
    /// - It recomputes the Jacobian matrices K,R,M in case of stiff load
    /// Q and Jacobians assumed evaluated at the current state. Jacobians are automatically allocated if needed.
    virtual void Update(double time, bool update_assets) override;

    /// Report if this is load is stiff.
    /// If so, InjectKRMMatrices will provide the Jacobians of the load.
    virtual bool IsStiff() = 0;

    // Functions for interfacing to the state bookkeeping and solver

    /// Add the internal loads Q (pasted at global offsets) into a global vector R, multiplied by a scaling factor c.
    /// In other words, perform the operation: R += forces * c
    virtual void LoadIntLoadResidual_F(ChVectorDynamic<>& R, double c) = 0;

    /// Increment a vector R with the matrix-vector product M*w, scaled by the factor c.
    /// In other words, perform the operation: R += c*M*w (i.e.,  R += c*(-dQ/da)*w).
    /// If no mass matrix M is present (i.e., no inertial effects), implement as no-op.
    virtual void LoadIntLoadResidual_Mv(ChVectorDynamic<>& R,        ///< result: the R residual, R += c*M*v
                                        const ChVectorDynamic<>& w,  ///< the w vector
                                        double c                     ///< scaling factor
                                        ) = 0;

    /// Add the lumped mass to an Md vector, representing a mass diagonal matrix.
    /// In other words, perform the operation: Md += c*diag(M).
    /// Used by lumped explicit integrators. If mass lumping is impossible or approximate, adds scalar error to "err"
    /// parameter. If no mass matrix M is present (i.e., no inertial effects), implement as no-op.
    virtual void LoadIntLoadLumpedMass_Md(
        ChVectorDynamic<>& Md,  ///< result: Md vector, diagonal of the lumped mass matrix
        double& err,            ///< result: not touched if lumping does not introduce errors
        const double c          ///< a scaling factor
        ) = 0;

    /// Register with the given system descriptor any ChKRMBlock objects associated with this item.
    virtual void InjectKRMMatrices(ChSystemDescriptor& descriptor);

    /// Compute and load current stiffnes (K), damping (R), and mass (M) matrices in encapsulated ChKRMBlock objects.
    /// The resulting KRM blocks represent linear combinations of the K, R, and M matrices, with the specified
    /// coefficients Kfactor, Rfactor,and Mfactor, respectively.
    virtual void LoadKRMMatrices(double Kfactor, double Rfactor, double Mfactor);

  protected:
    ChLoadJacobians* m_jacobians;
};

// -----------------------------------------------------------------------------

/// Load acting on a single ChLoadable item, via ChLoader objects.
/// There are various ChLoader interfaces ready to use, that can be used as 'building blocks'. These are especially
/// important for creating loads that are distributed on surfaces, lines, volumes, since some ChLoaders implement
/// quadrature.
class ChApi ChLoad : public ChLoadBase {
  public:
    std::shared_ptr<ChLoader> loader;

    ChLoad() {}
    ChLoad(std::shared_ptr<ChLoader> loader_object) : loader(loader_object) {}
    virtual ~ChLoad() {}

    /// "Virtual" copy constructor (covariant return type).
    virtual ChLoad* Clone() const override { return new ChLoad(*this); }

    /// Set the associated ChLoader object.
    void SetLoader(std::shared_ptr<ChLoader> loader_object) { loader = loader_object; }

    virtual int LoadGetNumCoordsPosLevel() override;
    virtual int LoadGetNumCoordsVelLevel() override;
    virtual void LoadGetStateBlock_x(ChState& mD) override;
    virtual void LoadGetStateBlock_w(ChStateDelta& mD) override;
    virtual void LoadStateIncrement(const ChState& x, const ChStateDelta& dw, ChState& x_new) override;
    virtual int LoadGetNumFieldCoords() override;

    /// Compute the generalized load(s).
    /// The generalized force Q is stored in the wrapped ChLoader.
    virtual void ComputeQ(ChState* state_x,      ///< state position to evaluate Q
                          ChStateDelta* state_w  ///< state speed to evaluate Q
                          ) override;

    /// Compute the K=-dQ/dx, R=-dQ/dv, M=-dQ/da Jacobians.
    /// This default implementation uses finite differences for computing the K, R, M matrices if the load is stiff.
    /// Note the sign that is flipped because we assume equations are written with Q moved to left-hand side.
    virtual void ComputeJacobian(ChState* state_x,      ///< state position to evaluate Jacobians
                                 ChStateDelta* state_w  ///< state speed to evaluate Jacobians
                                 ) override;

    /// Add the internal loads Q (pasted at global offsets) into a global vector R, multiplied by a scaling factor c.
    /// In other words, perform the operation: R += forces * c
    virtual void LoadIntLoadResidual_F(ChVectorDynamic<>& R, double c) override;

    /// Increment a vector R with the matrix-vector product M*w, scaled by the factor c.
    /// In other words, perform the operation: R += c*M*w (i.e.,  R += c*(-dQ/da)*w).
    /// If no mass matrix M is present (i.e., no inertial effects), implement as no-op.
    /// This default implementation uses the computed Jacobians (if available) to extract M = -dQ/da and then calculates
    /// R += c*M*w. A derived class should override this method if an analytical expressions for c*M*w is available.
    virtual void LoadIntLoadResidual_Mv(ChVectorDynamic<>& R,        ///< result: the R residual, R += c*M*w
                                        const ChVectorDynamic<>& w,  ///< the w vector
                                        double c                     ///< scaling factor
                                        ) override;

    /// Add the lumped mass to an Md vector, representing a mass diagonal matrix.
    /// In other words, perform the operation: Md += c*diag(M).
    /// Used by lumped explicit integrators. If mass lumping is impossible or approximate, adds scalar error to "err"
    /// parameter. If no mass matrix M is present (i.e., no inertial effects), implement as no-op.
    /// This default implementation uses the computed Jacobians (if available) to extract M = -dQ/da and then calculates
    /// Md += c*diag(M). A derived class should override this method if an analytical expressions for c*M*w is
    /// available.
    virtual void LoadIntLoadLumpedMass_Md(
        ChVectorDynamic<>& Md,  ///< result: Md vector, diagonal of the lumped mass matrix
        double& err,            ///< result: not touched if lumping does not introduce errors
        const double c          ///< a scaling factor
        ) override;

    /// Report if this is load is stiff.
    /// By default, the load is declared as stiff if the loader is stiff.
    virtual bool IsStiff() override { return loader->IsStiff(); }

    /// Create the Jacobian loads if needed and set the ChVariables referenced by the sparse KRM block.
    virtual void CreateJacobianMatrices() override;
};

// -----------------------------------------------------------------------------

/// Loads acting on a single ChLoadable item.
/// Differently form ChLoad, this does not use the ChLoader interface, so one must inherit from this and implement
/// ComputeQ() directly. The ComputeQ() must write the generalized forces Q into the "load_Q" vector of this object.
class ChApi ChLoadCustom : public ChLoadBase {
  public:
    std::shared_ptr<ChLoadable> loadable;
    ChVectorDynamic<> load_Q;

    ChLoadCustom(std::shared_ptr<ChLoadable> loadable_object);
    virtual ~ChLoadCustom() {}

    virtual int LoadGetNumCoordsPosLevel() override;
    virtual int LoadGetNumCoordsVelLevel() override;
    virtual void LoadGetStateBlock_x(ChState& mD) override;
    virtual void LoadGetStateBlock_w(ChStateDelta& mD) override;
    virtual void LoadStateIncrement(const ChState& x, const ChStateDelta& dw, ChState& x_new) override;
    virtual int LoadGetNumFieldCoords() override;

    /// Compute Jacobian matrices K=-dQ/dx, R=-dQ/dv, and M=-dQ/da.
    /// This default implementation uses finite differences for computing the K, R, M matrices if the load is stiff.
    /// If possible, a derived class should provide analytical Jacobians.
    virtual void ComputeJacobian(ChState* state_x,      ///< state position to evaluate Jacobians
                                 ChStateDelta* state_w  ///< state speed to evaluate Jacobians
                                 ) override;

    /// Add the internal loads Q (pasted at global offsets) into a global vector R, multiplied by a scaling factor c.
    /// In other words, perform the operation: R += forces * c
    virtual void LoadIntLoadResidual_F(ChVectorDynamic<>& R, double c) override;

    /// Increment a vector R with the matrix-vector product M*w, scaled by the factor c.
    /// In other words, perform the operation: R += c*M*w (i.e.,  R += c*(-dQ/da)*w).
    /// If no mass matrix M is present (i.e., no inertial effects), implement as no-op.
    /// This default implementation uses the computed Jacobians (if available) to extract M = -dQ/da and then calculates
    /// R += c*M*w. A derived class should override this method if an analytical expressions for c*M*w is available.
    virtual void LoadIntLoadResidual_Mv(ChVectorDynamic<>& R,        ///< result: the R residual, R += c*M*w
                                        const ChVectorDynamic<>& w,  ///< the w vector
                                        double c                     ///< scaling factor
                                        ) override;

    /// Add the lumped mass to an Md vector, representing a mass diagonal matrix.
    /// In other words, perform the operation: Md += c*diag(M).
    /// Used by lumped explicit integrators. If mass lumping is impossible or approximate, adds scalar error to "err"
    /// parameter. If no mass matrix M is present (i.e., no inertial effects), implement as no-op.
    /// This default implementation uses the computed Jacobians (if available) to extract M = -dQ/da and then calculates
    /// Md += c*diag(M). A derived class should override this method if an analytical expressions for c*M*w is
    /// available.
    virtual void LoadIntLoadLumpedMass_Md(
        ChVectorDynamic<>& Md,  ///< result: Md vector, diagonal of the lumped mass matrix
        double& err,            ///< result: not touched if lumping does not introduce errors
        const double c          ///< a scaling factor
        ) override;

    /// Create the Jacobian loads if needed and set the ChVariables referenced by the sparse KRM block.
    virtual void CreateJacobianMatrices() override;

    /// Access the generalized load vector Q.
    virtual ChVectorDynamic<>& GetQ() { return load_Q; }
};

// -----------------------------------------------------------------------------

/// Loads acting on multiple ChLoadable items.
/// One must inherit from this and implement ComputeQ() directly. The ComputeQ() must write the generalized forces Q
/// into the "load_Q" vector of this object. Given that multiple ChLoadable objects are referenced here, their
/// sub-forces Q are assumed appended in sequence in the "load_Q" vector, in the same order that has been used in the
/// std::vector "mloadables" for ChLoadCustomMultiple creation. The same applies for the order of the sub-matrices of
/// in the Jacobian matrices K, R, M.
class ChApi ChLoadCustomMultiple : public ChLoadBase {
  public:
    std::vector<std::shared_ptr<ChLoadable>> loadables;
    ChVectorDynamic<> load_Q;

    ChLoadCustomMultiple(std::vector<std::shared_ptr<ChLoadable>>& loadable_objects);
    ChLoadCustomMultiple(std::shared_ptr<ChLoadable> loadableA, std::shared_ptr<ChLoadable> loadableB);
    ChLoadCustomMultiple(std::shared_ptr<ChLoadable> loadableA,
                         std::shared_ptr<ChLoadable> loadableB,
                         std::shared_ptr<ChLoadable> loadableC);
    virtual ~ChLoadCustomMultiple() {}

    virtual int LoadGetNumCoordsPosLevel() override;
    virtual int LoadGetNumCoordsVelLevel() override;
    virtual void LoadGetStateBlock_x(ChState& mD) override;
    virtual void LoadGetStateBlock_w(ChStateDelta& mD) override;
    virtual void LoadStateIncrement(const ChState& x, const ChStateDelta& dw, ChState& x_new) override;
    virtual int LoadGetNumFieldCoords() override;

    /// Compute Jacobian matrices K=-dQ/dx, R=-dQ/dv, and M=-dQ/da.
    /// This default implementation uses finite differences for computing the K, R, M matrices if the load is stiff.
    /// If possible, a derived class should provide analytical Jacobians.
    /// Note: Given that multiple ChLoadable objects are referenced here, sub-matrices of K and R are pasted in (i,j)
    /// block positions that reflect the order in which loadable objects were specified.
    virtual void ComputeJacobian(ChState* state_x,      ///< state position to evaluate Jacobians
                                 ChStateDelta* state_w  ///< state speed to evaluate Jacobians
                                 ) override;

    /// Add the internal loads Q (pasted at global offsets) into a global vector R, multiplied by a scaling factor c.
    /// In other words, perform the operation: R += forces * c
    virtual void LoadIntLoadResidual_F(ChVectorDynamic<>& R, double c) override;

    /// Increment a vector R with the matrix-vector product M*w, scaled by the factor c.
    /// In other words, perform the operation: R += c*M*w (i.e.,  R += c*(-dQ/da)*w).
    /// If no mass matrix M is present (i.e., no inertial effects), implement as no-op.
    /// This default implementation uses the computed Jacobians (if available) to extract M = -dQ/da and then calculates
    /// R += c*M*w. A derived class should override this method if an analytical expressions for c*M*w is available.
    virtual void LoadIntLoadResidual_Mv(ChVectorDynamic<>& R,        ///< result: the R residual, R += c*M*w
                                        const ChVectorDynamic<>& w,  ///< the w vector
                                        double c                     ///< scaling factor
                                        ) override;

    /// Add the lumped mass to an Md vector, representing a mass diagonal matrix.
    /// In other words, perform the operation: Md += c*diag(M).
    /// Used by lumped explicit integrators. If mass lumping is impossible or approximate, adds scalar error to "err"
    /// parameter. If no mass matrix M is present (i.e., no inertial effects), implement as no-op.
    /// This default implementation uses the computed Jacobians (if available) to extract M = -dQ/da and then calculates
    /// Md += c*diag(M). A derived class should override this method if an analytical expressions for c*M*w is
    /// available.
    virtual void LoadIntLoadLumpedMass_Md(
        ChVectorDynamic<>& Md,  ///< result: Md vector, diagonal of the lumped mass matrix
        double& err,            ///< result: not touched if lumping does not introduce errors
        const double c          ///< a scaling factor
        ) override;

    /// Create the Jacobian loads if needed and set the ChVariables referenced by the sparse KRM block.
    virtual void CreateJacobianMatrices() override;

    /// Access the generalized load vector Q.
    virtual ChVectorDynamic<>& GetQ() { return load_Q; }
};

}  // end namespace chrono

#endif
