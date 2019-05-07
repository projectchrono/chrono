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

#include "chrono/physics/ChLoader.h"
#include "chrono/physics/ChLoaderU.h"
#include "chrono/physics/ChLoaderUV.h"
#include "chrono/physics/ChLoaderUVW.h"
#include "chrono/physics/ChObject.h"
#include "chrono/solver/ChKblockGeneric.h"
#include "chrono/solver/ChSystemDescriptor.h"
#include "chrono/timestepper/ChState.h"

namespace chrono {

/// Utility class for storing jacobian matrices.
/// This is automatically managed by the ChLoad, if needed
/// (ie. for stiff loads)

class ChApi ChLoadJacobians {
  public:
    ChKblockGeneric KRM;        ///< sum of K,R,M, with pointers to sparse variables
    ChMatrixDynamic<double> K;  ///< dQ/dx
    ChMatrixDynamic<double> R;  ///< dQ/dv
    ChMatrixDynamic<double> M;  ///< dQ/da

    /// Set references to the constrained objects, each of ChVariables type,
    /// automatically creating/resizing K matrix if needed.
    void SetVariables(std::vector<ChVariables*> mvariables);
};

// -----------------------------------------------------------------------------

/// Base class for loads.
/// This class can be inherited to implement applied loads to bodies,
/// finite elements, etc.
/// A load is an object that might wrap one or more ChLoader objects, whose
/// value is dependent.
/// It implements functionalities to perform automatic differentiation of
/// the load so it optionally can compute the jacobian (the tangent stiffness
/// matrix of the load) that can be used in implicit integrators, statics, etc.

class ChApi ChLoadBase : public ChObj {
  protected:
    ChLoadJacobians* jacobians;

  public:
    ChLoadBase();

    virtual ~ChLoadBase();

    /// Gets the number of DOFs affected by this load (position part)
    virtual int LoadGet_ndof_x() = 0;

    /// Gets the number of DOFs affected by this load (speed part)
    virtual int LoadGet_ndof_w() = 0;

    /// Gets all the current DOFs packed in a single vector (position part)
    virtual void LoadGetStateBlock_x(ChState& mD) = 0;

    /// Gets all the current DOFs packed in a single vector (speed part)
    virtual void LoadGetStateBlock_w(ChStateDelta& mD) = 0;

    /// Increment a packed state (ex. as obtained by LoadGetStateBlock_x()) by a given packed state-delta.
    /// Compute: x_new = x + dw. Ex. this is called by the BDF numerical differentiation routine that computes jacobian
    /// in the default ComputeJacobian() fallback, if not overriding ComputeJacobian() with an analytical form.
    virtual void LoadStateIncrement(const ChState& x, const ChStateDelta& dw, ChState& x_new) = 0;

    /// Number of coordinates in the interpolated field, ex=3 for a
    /// tetrahedron finite element or a cable, = 1 for a thermal problem, etc.
    virtual int LoadGet_field_ncoords() = 0;

    /// Compute Q, the generalized load(s).
    /// Where Q is stored depends on children classes.
    /// Called automatically at each Update().
    virtual void ComputeQ(ChState* state_x,      ///< state position to evaluate Q
                          ChStateDelta* state_w  ///< state speed to evaluate Q
                          ) = 0;

    /// Compute the K=-dQ/dx, R=-dQ/dv , M=-dQ/da jacobians.
    /// Called automatically at each Update().
    virtual void ComputeJacobian(ChState* state_x,       ///< state position to evaluate jacobians
                                 ChStateDelta* state_w,  ///< state speed to evaluate jacobians
                                 ChMatrix<>& mK,         ///< result dQ/dx
                                 ChMatrix<>& mR,         ///< result dQ/dv
                                 ChMatrix<>& mM          ///< result dQ/da
                                 ) = 0;

    /// Access the jacobians (if any, i.e. if this is a stiff load)
    ChLoadJacobians* GetJacobians() { return this->jacobians; }

    /// Create the jacobian loads if needed, and also
    /// set the ChVariables referenced by the sparse KRM block.
    virtual void CreateJacobianMatrices() = 0;

    /// Update: this is called at least at each time step.
    /// - It recomputes the generalized load Q vector(s)
    /// - It recomputes the jacobian(s) K,R,M in case of stiff load
    /// Q and jacobians assumed evaluated at the current state.
    /// Jacobian structures are automatically allocated if needed.
    virtual void Update(double time);

    /// Report if this is load is stiff. If so, InjectKRMmatrices will provide
    /// the jacobians of the load.
    virtual bool IsStiff() = 0;

    //
    // Functions for interfacing to the state bookkeeping and solver
    //

    /// Adds the internal loads Q (pasted at global nodes offsets) into
    /// a global vector R, multiplied by a scaling factor c, as
    ///   R += forces * c
    virtual void LoadIntLoadResidual_F(ChVectorDynamic<>& R, const double c) = 0;

    /// Tell to a system descriptor that there are item(s) of type
    /// ChKblock in this object (for further passing it to a solver)
    /// Basically does nothing, but inherited classes must specialize this.
    virtual void InjectKRMmatrices(ChSystemDescriptor& mdescriptor);

    /// Adds the current stiffness K and damping R and mass M matrices in encapsulated
    /// ChKblock item(s), if any. The K, R, M matrices are added with scaling
    /// values Kfactor, Rfactor, Mfactor.
    virtual void KRMmatricesLoad(double Kfactor, double Rfactor, double Mfactor);
};

// -----------------------------------------------------------------------------

/// Class for a load acting on a single ChLoadable item, via ChLoader objects.
/// There are various ChLoader interfaces ready to use, that can be used
/// as 'building blocks'. These are especially important for creating loads
/// that are distributed on surfaces, lines, volumes, since some ChLoaders implement quadrature.
/// Create them as ChLoad< ChLoaderPressure > my_load(...); for example.

template <class Tloader>
class ChLoad : public ChLoadBase {
  public:
    Tloader loader;

    ChLoad(std::shared_ptr<typename Tloader::type_loadable> mloadable) : loader(mloadable) {}

    virtual ~ChLoad() {}

    /// "Virtual" copy constructor (covariant return type).
    virtual ChLoad* Clone() const override { return new ChLoad(*this); }

    virtual int LoadGet_ndof_x() override;
    virtual int LoadGet_ndof_w() override;
    virtual void LoadGetStateBlock_x(ChState& mD) override;
    virtual void LoadGetStateBlock_w(ChStateDelta& mD) override;
    virtual void LoadStateIncrement(const ChState& x, const ChStateDelta& dw, ChState& x_new) override;
    virtual int LoadGet_field_ncoords() override;

    /// Compute Q, the generalized load.
    /// Q is stored in the wrapped ChLoader.
    /// Called automatically at each Update().
    virtual void ComputeQ(ChState* state_x,      ///< state position to evaluate Q
                          ChStateDelta* state_w  ///< state speed to evaluate Q
                          ) override;

    /// Compute jacobians (default fallback).
    /// Uses a numerical differentiation for computing K, R, M jacobians, if stiff load.
    /// If possible, override this with an analytical jacobian.
    /// Compute the K=-dQ/dx, R=-dQ/dv , M=-dQ/da jacobians.
    /// Called automatically at each Update().
    virtual void ComputeJacobian(ChState* state_x,       ///< state position to evaluate jacobians
                                 ChStateDelta* state_w,  ///< state speed to evaluate jacobians
                                 ChMatrix<>& mK,         ///< result dQ/dx
                                 ChMatrix<>& mR,         ///< result dQ/dv
                                 ChMatrix<>& mM          ///< result dQ/da
                                 ) override;

    virtual void LoadIntLoadResidual_F(ChVectorDynamic<>& R, const double c) override;

    /// Default: load is stiff if the loader is stiff. Override if needed.
    virtual bool IsStiff() override { return loader.IsStiff(); }

    /// Create the jacobian loads if needed, and also
    /// set the ChVariables referenced by the sparse KRM block.
    virtual void CreateJacobianMatrices() override;
};

// -----------------------------------------------------------------------------

/// Loads acting on a single ChLoadable item.
/// Differently form ChLoad, this does not use the ChLoader interface,
/// so one must inherit from this and implement ComputeQ() directly. 
/// The ComputeQ() must write the generalized forces Q into the "load_Q" vector of this object.

class ChApi ChLoadCustom : public ChLoadBase {
  public:
    std::shared_ptr<ChLoadable> loadable;
    ChVectorDynamic<> load_Q;

    ChLoadCustom(std::shared_ptr<ChLoadable> mloadable);

    virtual ~ChLoadCustom() {}

    virtual int LoadGet_ndof_x() override;
    virtual int LoadGet_ndof_w() override;
    virtual void LoadGetStateBlock_x(ChState& mD) override;
    virtual void LoadGetStateBlock_w(ChStateDelta& mD) override;
    virtual void LoadStateIncrement(const ChState& x, const ChStateDelta& dw, ChState& x_new) override;
    virtual int LoadGet_field_ncoords() override;

    /// Compute jacobians (default fallback).
    /// Uses a numerical differentiation for computing K, R, M jacobians, if stiff load.
    /// If possible, override this with an analytical jacobian.
    /// Compute the K=-dQ/dx, R=-dQ/dv , M=-dQ/da jacobians.
    /// Called automatically at each Update().
    virtual void ComputeJacobian(ChState* state_x,       ///< state position to evaluate jacobians
                                 ChStateDelta* state_w,  ///< state speed to evaluate jacobians
                                 ChMatrix<>& mK,         ///< result dQ/dx
                                 ChMatrix<>& mR,         ///< result dQ/dv
                                 ChMatrix<>& mM          ///< result dQ/da
                                 ) override;

    virtual void LoadIntLoadResidual_F(ChVectorDynamic<>& R, const double c) override;

    /// Create the jacobian loads if needed, and also
    /// set the ChVariables referenced by the sparse KRM block.
    virtual void CreateJacobianMatrices() override;

    /// Access the generalized load vector Q.
    virtual ChVectorDynamic<>& GetQ() { return load_Q; }
};

// -----------------------------------------------------------------------------

/// Loads acting on multiple ChLoadable items.
/// One must inherit from this and implement ComputeQ() directly. The ComputeQ() must
/// write the generalized forces Q into the "load_Q" vector of this object.
/// Given that multiple ChLoadable objects are referenced here, their sub-forces Q are
/// assumed appended in sequence in the "load_Q" vector, in the same order that has been
/// used in the std::vector "mloadables" for ChLoadCustomMultiple creation.
/// The same applies for the order of the sub-matrices of jacobians K,R etc.

class ChApi ChLoadCustomMultiple : public ChLoadBase {
  public:
    std::vector<std::shared_ptr<ChLoadable>> loadables;
    ChVectorDynamic<> load_Q;

    ChLoadCustomMultiple(std::vector<std::shared_ptr<ChLoadable>>& mloadables);
    ChLoadCustomMultiple(std::shared_ptr<ChLoadable> mloadableA, std::shared_ptr<ChLoadable> mloadableB);
    ChLoadCustomMultiple(std::shared_ptr<ChLoadable> mloadableA,
                         std::shared_ptr<ChLoadable> mloadableB,
                         std::shared_ptr<ChLoadable> mloadableC);

    virtual ~ChLoadCustomMultiple() {}

    virtual int LoadGet_ndof_x() override;
    virtual int LoadGet_ndof_w() override;
    virtual void LoadGetStateBlock_x(ChState& mD) override;
    virtual void LoadGetStateBlock_w(ChStateDelta& mD) override;
    virtual void LoadStateIncrement(const ChState& x, const ChStateDelta& dw, ChState& x_new) override;
    virtual int LoadGet_field_ncoords() override;

    /// Compute jacobians (default fallback).
    /// Compute the K=-dQ/dx, R=-dQ/dv , M=-dQ/da jacobians.
    /// Uses a numerical differentiation for computing K, R, M jacobians, if stiff load.
    /// If possible, override this with an analytical jacobian.
    /// NOTE: Given that multiple ChLoadable objects are referenced here, sub-matrices of mK,mR are
    /// assumed pasted in i,j block-positions where i,j reflect the same order that has been
    /// used in the std::vector "mloadables" at ChLoadCustomMultiple creation.
    /// Called automatically at each Update().
    virtual void ComputeJacobian(ChState* state_x,       ///< state position to evaluate jacobians
                                 ChStateDelta* state_w,  ///< state speed to evaluate jacobians
                                 ChMatrix<>& mK,         ///< result dQ/dx
                                 ChMatrix<>& mR,         ///< result dQ/dv
                                 ChMatrix<>& mM          ///< result dQ/da
                                 ) override;

    virtual void LoadIntLoadResidual_F(ChVectorDynamic<>& R, const double c) override;

    /// Create the jacobian loads if needed, and also
    /// set the ChVariables referenced by the sparse KRM block.
    virtual void CreateJacobianMatrices() override;

    /// Access the generalized load vector Q.
    virtual ChVectorDynamic<>& GetQ() { return load_Q; }
};

// =============================================================================
// IMPLEMENTATION OF ChLoad<Tloader> methods
// =============================================================================

template <class Tloader>
inline int ChLoad<Tloader>::LoadGet_ndof_x() {
    return this->loader.GetLoadable()->LoadableGet_ndof_x();
}

template <class Tloader>
inline int ChLoad<Tloader>::LoadGet_ndof_w() {
    return this->loader.GetLoadable()->LoadableGet_ndof_w();
}

template <class Tloader>
inline void ChLoad<Tloader>::LoadGetStateBlock_x(ChState& mD) {
    this->loader.GetLoadable()->LoadableGetStateBlock_x(0, mD);
}

template <class Tloader>
inline void ChLoad<Tloader>::LoadGetStateBlock_w(ChStateDelta& mD) {
    this->loader.GetLoadable()->LoadableGetStateBlock_w(0, mD);
}

template <class Tloader>
inline void ChLoad<Tloader>::LoadStateIncrement(const ChState& x, const ChStateDelta& dw, ChState& x_new) {
    this->loader.GetLoadable()->LoadableStateIncrement(0, x_new, x, 0, dw);
}

template <class Tloader>
inline int ChLoad<Tloader>::LoadGet_field_ncoords() {
    return this->loader.GetLoadable()->Get_field_ncoords();
}

template <class Tloader>
inline void ChLoad<Tloader>::ComputeQ(ChState* state_x, ChStateDelta* state_w) {
    this->loader.ComputeQ(state_x, state_w);
}

template <class Tloader>
inline void ChLoad<Tloader>::ComputeJacobian(ChState* state_x,
                                             ChStateDelta* state_w,
                                             ChMatrix<>& mK,
                                             ChMatrix<>& mR,
                                             ChMatrix<>& mM) {
    double Delta = 1e-8;

    int mrows_w = this->LoadGet_ndof_w();
    int mrows_x = this->LoadGet_ndof_x();

    // compute Q at current speed & position, x_0, v_0
    ChVectorDynamic<> Q0(mrows_w);
    this->loader.ComputeQ(state_x, state_w);  // Q0 = Q(x, v)
    Q0 = this->loader.Q;

    ChVectorDynamic<> Q1(mrows_w);
    ChVectorDynamic<> Jcolumn(mrows_w);
    ChState state_x_inc(mrows_x, nullptr);
    ChStateDelta state_delta(mrows_w, nullptr);

    // Compute K=-dQ(x,v)/dx by backward differentiation
    state_delta.Reset(mrows_w, nullptr);

    for (int i = 0; i < mrows_w; ++i) {
        state_delta(i) += Delta;
        this->LoadStateIncrement(*state_x, state_delta,
                                 state_x_inc);         // exponential, usually state_x_inc(i) = state_x(i) + Delta;
        this->loader.ComputeQ(&state_x_inc, state_w);  // Q1 = Q(x+Dx, v)
        Q1 = this->loader.Q;
        state_delta(i) -= Delta;

        Jcolumn = (Q1 - Q0) * (-1.0 / Delta);  // - sign because K=-dQ/dx
        this->jacobians->K.PasteMatrix(Jcolumn, 0, i);
    }
    // Compute R=-dQ(x,v)/dv by backward differentiation
    for (int i = 0; i < mrows_w; ++i) {
        (*state_w)(i) += Delta;
        this->loader.ComputeQ(state_x, state_w);  // Q1 = Q(x, v+Dv)
        Q1 = this->loader.Q;
        (*state_w)(i) -= Delta;

        Jcolumn = (Q1 - Q0) * (-1.0 / Delta);  // - sign because R=-dQ/dv
        this->jacobians->R.PasteMatrix(Jcolumn, 0, i);
    }
}

template <class Tloader>
inline void ChLoad<Tloader>::LoadIntLoadResidual_F(ChVectorDynamic<>& R, const double c) {
    unsigned int rowQ = 0;
    for (int i = 0; i < this->loader.GetLoadable()->GetSubBlocks(); ++i) {
        unsigned int moffset = this->loader.GetLoadable()->GetSubBlockOffset(i);
        for (unsigned int row = 0; row < this->loader.GetLoadable()->GetSubBlockSize(i); ++row) {
            R(row + moffset) += this->loader.Q(rowQ) * c;
            ++rowQ;
        }
    }
}

template <class Tloader>
inline void ChLoad<Tloader>::CreateJacobianMatrices() {
    if (!this->jacobians) {
        // create jacobian structure
        this->jacobians = new ChLoadJacobians;
        // set variables forsparse KRM block
        std::vector<ChVariables*> mvars;
        loader.GetLoadable()->LoadableGetVariables(mvars);
        this->jacobians->SetVariables(mvars);
    }
}

}  // end namespace chrono

#endif
