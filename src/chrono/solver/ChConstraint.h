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

#ifndef CHCONSTRAINT_H
#define CHCONSTRAINT_H

#include "chrono/core/ChApiCE.h"
#include "chrono/core/ChClassFactory.h"
#include "chrono/core/ChMatrix.h"

namespace chrono {

/// Base class for representing constraints (bilateral or unilateral).
/// These constraints are used with variational inequality or DAE solvers for problems including equalities,
/// inequalities, nonlinearities, etc.
///
/// See ChSystemDescriptor for more information about the overall problem and data representation.
///
/// The Jacobian matrix [Cq] is built row by row from individual constraint Jacobians [Cq_i].
/// [E] optionally includes 'cfm_i' terms on the diagonal.
///
/// In general, typical bilateral constraints must be solved to have residual c_i = 0 and unilaterals to have c_i>0,
/// where the following linearization is introduced:
/// <pre>
///      c_i= [Cq_i]*q + b_i
/// </pre>
///
/// The base class introduces just the minimum requirements for the solver, that is the basic methods that will be
/// called by the solver.
class ChApi ChConstraint {
  public:
    /// Constraint mode.
    enum class Mode {
        FREE,        ///< does not enforce anything
        LOCK,        ///< enforces c_i=0 (bilateral)
        UNILATERAL,  ///< enforces linear complementarity c_i>=0, l_i>=0, l_i*c_i=0
        FRICTION,    ///< one of three reactions in friction (cone complementarity problem)
    };

    ChConstraint();
    ChConstraint(const ChConstraint& other);
    virtual ~ChConstraint() {}

    /// "Virtual" copy constructor.
    virtual ChConstraint* Clone() const = 0;

    /// Assignment operator: copy from other object.
    ChConstraint& operator=(const ChConstraint& other);

    /// Comparison (compares only flags, not the Jacobians).
    bool operator==(const ChConstraint& other) const;

    /// Indicate if the constraint data is currently valid.
    bool IsValid() const { return valid; }

    /// Set the "valid" state of this constraint.
    void SetValid(bool mon) {
        valid = mon;
        UpdateActiveFlag();
    }

    /// Indicate if the constraint is currently turned on or off.
    bool IsDisabled() const { return disabled; }

    // Enable/disable this constraint.
    void SetDisabled(bool mon) {
        disabled = mon;
        UpdateActiveFlag();
    }

    /// Indicate if the constraint is redundant or singular.
    bool IsRedundant() const { return redundant; }

    /// Mark the constraint as redundant.
    void SetRedundant(bool mon) {
        redundant = mon;
        UpdateActiveFlag();
    }

    /// Indicate if the constraint is broken, due to excess pulling/pushing.
    bool IsBroken() const { return broken; }

    /// Set the constraint as broken.
    /// By default, constraints never break.
    void SetBroken(bool mon) {
        broken = mon;
        UpdateActiveFlag();
    }

    /// Indicate if the constraint is unilateral (typical complementarity constraint).
    virtual bool IsUnilateral() const { return mode == Mode::UNILATERAL; }

    /// Indicate if the constraint is linear.
    virtual bool IsLinear() const { return true; }

    /// Get the mode of the constraint.
    /// A typical constraint has LOCK mode (bilateral) by default.
    Mode GetMode() const { return mode; }

    /// Set the mode of the constraint.
    void SetMode(Mode mmode) {
        mode = mmode;
        UpdateActiveFlag();
    }

    /// Indicate whether the constraint is currently active.
    /// In general, this indicates if it must be included into the system solver or not.
    /// This method acumulates the effect of all flags, so a constraint may be inactive either because it is 'disabled',
    /// 'broken', 'redundant', or not 'valid'.
    inline bool IsActive() const { return active; }

    /// Set the status of the constraint to active
    void SetActive(bool isactive) { active = isactive; }

    /// Compute the residual of the constraint using the linear expression
    /// <pre>
    /// c_i= [Cq_i]*q + cfm_i*l_i + b_i
    /// </pre>
    /// For a satisfied bilateral constraint, this residual must be near zero.
    virtual double ComputeResidual() {
        c_i = ComputeJacobianTimesState() + cfm_i * l_i + b_i;
        return c_i;
    }

    /// Return the residual of this constraint.
    // (CURRENTLY NOT USED)
    double GetResidual() const { return c_i; }

    /// Sets the known term b_i in [Cq_i]*q + b_i = 0, where: c_i = [Cq_i]*q + b_i = 0.
    void SetRightHandSide(const double mb) { b_i = mb; }

    /// Return the known term b_i in [Cq_i]*q + b_i = 0, where: c_i= [Cq_i]*q + b_i = 0.
    double GetRightHandSide() const { return b_i; }

    /// Set the constraint force mixing term (default=0).
    /// This adds artificial 'elasticity' to the constraint, as c_i= [Cq_i]*q + b_i + cfm*l_i = 0.
    void SetComplianceTerm(const double mcfm) { cfm_i = mcfm; }

    /// Return the constraint force mixing term.
    double GetComplianceTerm() const { return cfm_i; }

    /// Set the value of the corresponding Lagrange multiplier (constraint reaction).
    void SetLagrangeMultiplier(double ml_i) { l_i = ml_i; }

    /// Return the corresponding Lagrange multiplier (constraint reaction).
    double GetLagrangeMultiplier() const { return l_i; }

    // -----  Functions often used by iterative solvers:

    /// This function updates the following auxiliary data:
    ///  - the Eq_a and Eq_b matrices
    ///  - the g_i product
    /// This function is often called by solvers at the beginning of the solution process.
    /// Note: This function *must* be overridden by specialized derived classes, which have some Jacobians.
    virtual void UpdateAuxiliary() {}

    /// Return the 'g_i' product, that is [Cq_i]*[invM_i]*[Cq_i]' (+cfm)
    double GetSchurComplement() const { return g_i; }

    /// Usually you should not use the SetSchurComplement function, because g_i
    /// should be automatically computed during the UpdateAuxiliary() .
    void SetSchurComplement(double m_g_i) { g_i = m_g_i; }

    /// Compute the product between the Jacobian of this constraint, [Cq_i], and the vector of variables.
    /// In other words, perform the operation:
    /// <pre>
    ///   CV = [Cq_i] * v
    /// </pre>
    virtual double ComputeJacobianTimesState() = 0;

    /// Increment the vector of variables with the quantity [invM]*[Cq_i]'*deltal.
    /// In other words, perform the operation:
    /// <pre>
    ///    v += [invM] * [Cq_i]' * deltal
    /// or else
    ///    v+=[Eq_i] * deltal
    /// </pre>
    virtual void IncrementState(double deltal) = 0;

    /// Add the product of the corresponding block in the system matrix by 'vect' and add to result.
    /// Note: 'vect' is assumed to be of proper size; the procedure uses the ChVariable offsets to index in 'vect'.
    virtual void AddJacobianTimesVectorInto(double& result, ChVectorConstRef vect) const = 0;

    /// Add the product of the corresponding transposed block in the system matrix by 'l' and add to result.
    /// Note: 'result' is assumed to be of proper size; the procedure uses the ChVariable offsets to index in 'result'.
    virtual void AddJacobianTransposedTimesScalarInto(ChVectorRef result, double l) const = 0;

    /// Project the value of a possible 'l_i' value of constraint reaction onto admissible orthant/set.
    /// Default behavior: if constraint is unilateral and l_i<0, reset l_i=0
    /// Note: This function MAY BE OVERRIDDEN by specialized inherited classes. For example,
    /// - a bilateral constraint does nothing
    /// - a unilateral constraint: l_i= std::max(0., l_i)
    /// - a 'boxed constraint': l_i= std::min(std::max(min., l_i), max)
    virtual void Project();

    /// Return the constraint violation.
    /// The function receives as input the linear map
    /// <pre>
    ///    mc_i =  [Cq]*q + b_i + cfm*l_i
    /// </pre>
    ///   For bilateral constraint,  violation = mc_i.
    ///   For unilateral constraint, violation = min(mc_i, 0),
    ///   For boxed constraints and similar, inherited class *should* override this implementation.
    virtual double Violation(double mc_i);

    /// Write the constraint Jacobian into the specified global matrix at the offsets of the associated variables.
    /// The (start_row, start_col) pair specifies the top-left corner of the system-level constraint Jacobian in the
    /// provided matrix.
    virtual void PasteJacobianInto(ChSparseMatrix& mat, unsigned int start_row, unsigned int start_col) const = 0;

    /// Write the transposed constraint Jacobian into the specified global matrix at the offsets of the associated
    /// variables. The (start_row, start_col) pair specifies the top-left corner of the system-level constraint Jacobian
    /// in the provided matrix.
    virtual void PasteJacobianTransposedInto(ChSparseMatrix& mat,
                                             unsigned int start_row,
                                             unsigned int start_col) const = 0;

    /// Set offset in global q vector (set automatically by ChSystemDescriptor)
    void SetOffset(unsigned int off) { offset = off; }

    /// Get offset in global q vector
    unsigned int GetOffset() const { return offset; }

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOut(ChArchiveOut& archive_out);

    /// Method to allow de-serialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& archive_in);

  protected:
    double c_i;  ///< constraint residual (if satisfied, c must be 0)
    double l_i;  ///< Lagrange multiplier (reaction)
    double b_i;  ///< right-hand side term in [Cq_i]*q+b_i=0 , note: c_i= [Cq_i]*q + b_i

    /// Constraint force mixing if needed to add some numerical 'compliance' in the constraint.
    /// With this, the equation becomes: c_i= [Cq_i]*q + b_i + cfm*l_i =0.
    /// For example, this additional term could be:  cfm = [k * h^2](^-1), where k is stiffness.
    /// Usually set to 0.
    double cfm_i;

    Mode mode;            ///< mode of the constraint
    double g_i;           ///< product [Cq_i]*[invM_i]*[Cq_i]' (+cfm)
    unsigned int offset;  ///< offset in global "l" state vector (needed by some solvers)

  private:
    void UpdateActiveFlag();

    bool valid;      ///< the link has no formal problems (references restored correctly, etc)
    bool disabled;   ///< the user can turn on/off the link easily
    bool redundant;  ///< the constraint is redundant or singular
    bool broken;     ///< the constraint is broken (someone pulled too much..)
    bool active;     ///< Cached active state depending on previous flags. Internal update.
};

}  // end namespace chrono

#endif
