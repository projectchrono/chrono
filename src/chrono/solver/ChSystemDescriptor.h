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

#ifndef CHSYSTEMDESCRIPTOR_H
#define CHSYSTEMDESCRIPTOR_H

#include <vector>

#include "chrono/solver/ChConstraint.h"
#include "chrono/solver/ChKRMBlock.h"
#include "chrono/solver/ChVariables.h"

namespace chrono {

/// Base class for collecting objects inherited from ChConstraint,
/// ChVariables and optionally ChKRMBlock. These objects
/// can be used to define a sparse representation of the system.
/// This collector is important because it contains all the required
/// information that is sent to a solver (usually a VI/CCP solver, or
/// as a subcase, a linear solver).\n
/// The problem is described by a variational inequality VI(Z*x-d,K):\n
/// The matrix \f$Z\f$ that represents the problem has this form:
/// <pre>
///  | H -Cq'|*|q|- | f|= |0|
///  | Cq -E | |l|  |-b|  |c|
/// </pre>
/// with \f$Y \ni \mathbb{l}  \perp \mathbb{c} \in N_y\f$ \n
/// where \f$N_y\f$ is the normal cone to \f$Y\f$ \n
/// By flipping the sign of \f$l_i\f$, the matrix \f$Z\f$ can be symmetric (but in general non positive definite)
/// <pre>
/// | H  Cq'|*| q|-| f|=|0|
/// | Cq  E | |-l| |-b| |c|
/// </pre>
///
/// * Linear Problem: \f$ \forall i \,Y_i = \mathbb{R}, N_{y_{i}} = 0\f$ (e.g. all bilateral)
/// * Linear Complementarity Problem (LCP): \f$ 0\le c \perp l\ge0 \f$ (i.e. \f$Y_i = \mathbb{R}^+\f$)
/// * Cone Complementarity Problem (CCP): \f$Y \ni \mathbb{l}  \perp \mathbb{c} \in N_y\f$ (\f$Y_i\f$ are friction
/// cones)
///
/// *Notes*
/// 1. most often you call BuildSystemMatrix() right after a dynamic simulation step,
///    in order to get the system matrices updated to the last timestep;
/// 2. when using Anitescu default stepper, the 'f' vector contains forces*timestep = F*dt
/// 3. when using Anitescu default stepper, 'q' represents the 'delta speed',
/// 4. when using Anitescu default stepper, 'b' represents the dt/phi stabilization term.
/// 5. usually, H = M, the mass matrix, but in some cases, ex. when using
///         implicit integrators, objects inherited from ChKRMBlock can be added
///         too, hence H could be H=a*M+b*K+c*R (but not all solvers handle ChKRMBlock!)
///
/// All solvers require that the description of the system
/// is passed by means of a ChSystemDescriptor object
/// that holds a list of all the constraints, variables, masses, known terms
///	(ex.forces) under the form of ChVariables, ChConstraints and ChKRMBlock.
///
/// In this default implementation, the ChSystemDescriptor
/// simply holds a vector of pointers to ChVariables
/// or to ChConstraints, but more advanced implementations (ex. for
/// supporting parallel GPU solvers) could store constraints
/// and variables structures with other, more efficient data schemes.

class ChApi ChSystemDescriptor {
  public:
    ChSystemDescriptor();
    virtual ~ChSystemDescriptor();

    /// Access the vector of constraints.
    std::vector<ChConstraint*>& GetConstraints() { return m_constraints; }

    /// Access the vector of variables.
    std::vector<ChVariables*>& GetVariables() { return m_variables; }

    /// Begin insertion of items
    virtual void BeginInsertion() {
        m_constraints.clear();
        m_variables.clear();
        m_KRMblocks.clear();
    }

    /// Insert reference to a ChConstraint object.
    virtual void InsertConstraint(ChConstraint* mc) { m_constraints.push_back(mc); }

    /// Insert reference to a ChVariables object.
    virtual void InsertVariables(ChVariables* mv) { m_variables.push_back(mv); }

    /// Insert reference to a ChKRMBlock object (a piece of matrix).
    virtual void InsertKRMBlock(ChKRMBlock* mk) { m_KRMblocks.push_back(mk); }

    /// End insertion of items.
    /// A derived class should always call UpdateCountsAndOffsets.
    virtual void EndInsertion() { UpdateCountsAndOffsets(); }

    /// Count & returns the scalar variables in the system.
    /// This excludes ChVariable object that are set as inactive.
    /// Notes:
    /// - the number of scalar variables is not necessarily the number of inserted ChVariable objects,
    ///   some could be inactive.
    /// - also updates the offsets of all variables in 'q' global vector (see GetOffset() in ChVariables).
    virtual unsigned int CountActiveVariables() const;

    /// Count & returns the scalar constraints in the system
    /// This excludes ChConstraint object that are set as inactive. Note that this also updates the offsets of all
    /// constraints in 'l' global vector (see GetOffset() in ChConstraint).
    virtual unsigned int CountActiveConstraints() const;

    /// Update counts of scalar variables and scalar constraints.
    virtual void UpdateCountsAndOffsets();

    /// Set the c_a coefficient (default=1) used for scaling the M masses of the m_variables.
    /// Used when performing SchurComplementProduct(), SystemProduct(), BuildSystemMatrix().
    virtual void SetMassFactor(const double mc_a) { c_a = mc_a; }

    /// Get the c_a coefficient (default=1) used for scaling the M masses of the m_variables.
    virtual double GetMassFactor() { return c_a; }

    /// Gather all 'fb' known terms from all variables into a column vector.
    /// The column vector will be automatically reset and resized to the proper length if necessary.
    virtual unsigned int BuildFbVector(ChVectorDynamic<>& f,       ///< system-level vector 'f'
                                       unsigned int start_row = 0  ///< offset in global 'f' vector
    ) const;

    /// Gather the constraint residuals 'bi' into a coluimn vector.
    /// The column vector will be automatically reset and resized to the proper length if necessary.
    virtual unsigned int BuildBiVector(ChVectorDynamic<>& b,       ///< system-level vector 'b'
                                       unsigned int start_row = 0  ///< offset in global 'b' vector
    ) const;

    /// Get the vector d = {f; -b} with all the 'fb' and 'bi' known terms, as in Z*y-d.
    /// This vector is the concatenation of BuildFbVector and BuildBiVector.
    /// It will be automatically reset and resized to the proper length if necessary.
    virtual unsigned int BuildDiVector(ChVectorDynamic<>& d, double scale_factor = 1) const;

    /// Get the D diagonal of the Z system matrix, as a single column vector.
    /// This includes all the diagonal masses of M, and all the diagonal E (-cfm) terms.
    /// The diagonal_vect must already have size equal to the number of unknowns, otherwise it will be resized as
    /// necessary.
    virtual unsigned int BuildDiagonalVector(ChVectorDynamic<>& diagonal_vect) const;

    /// Gather the 'q' tertms from all variables into a column vector.
    /// The column vector will be automatically reset and resized to the proper length if requested.
    /// \return  the number of scalar variables (i.e. the rows of the column vector).
    virtual unsigned int FromVariablesToVector(ChVectorDynamic<>& vector,  ///< system-level vector 'q'
                                               bool resize_vector = true   ///< if true, resize vector as necessary
    ) const;

    /// Scatter the given vector to the 'q' terms for all variables.
    /// Note that *no* check on the size and ordering of the provided vector is performed.
    /// \return  the number of scalar variables (i.e. the rows of the column vector).
    virtual unsigned int FromVectorToVariables(const ChVectorDynamic<>& vector);

    /// Gather all constraint reactions 'l' into a column vector.
    /// The column vectorwill be automatically reset and resized to the proper length if requested.
    /// \return  the number of scalar constr.multipliers (i.e. the rows of the column vector).
    virtual unsigned int FromConstraintsToVector(ChVectorDynamic<>& vector,  ///< system-level vector 'l_i'
                                                 bool resize_vector = true   ///< if true, resize vector as necessary
    ) const;

    /// Scatter the given vector to the constraint reaction terms 'l' for all constraints.
    /// Note that *no* check on the size and ordering of the provided vector is performed.
    /// \return  the number of scalar constraint multipliers (i.e. the rows of the column vector).
    virtual unsigned int FromVectorToConstraints(const ChVectorDynamic<>& vector);

    /// Gather all unknows x = {q,l} into a column vector.
    /// The column vector will be automatically reset and resized to the proper length if requested.
    /// \return  the number of scalar unknowns
    virtual unsigned int FromUnknownsToVector(ChVectorDynamic<>& vector,  ///< system-level vector x={q,l}
                                              bool resize_vector = true   ///< if true, resize vector as necessary
    ) const;

    /// Scatter the given vector to the variables 'q' and constraint multipliers 'l'.
    /// Note that *no* check on the size and ordering of the provided vector is performed.
    /// The scattered Lagrange multipliers are first multiplied by 1/scale_factor.
    /// \return  the number of scalar unknowns
    virtual unsigned int FromVectorToUnknowns(const ChVectorDynamic<>& vector, double scale_factor = 1);

    // ------------------------------------------

    /// Return true if the system descriptor contains KRM blocks.
    bool HasKRMBlocks();

    /// Return true if the system descriptor supports Schur complement-based solvers.
    /// That means that:
    /// - the system does not include KRM blocks, or
    /// - the system does not include KR blocks and an inverse mass matrix is provided.
    bool SupportsSchurComplement();

    /// Set the inverse mass matrix.
    /// If provided, this matrix is used in calculating the Schur complement (for both SchurComplementProduct and
    /// SchurComplementRHS); this allows using solver based on the Schur complement also for problems that have a
    /// non-block diagonal generalized mass matrix (e.g., when using hydrodynamic loads which include so-called
    /// "added mass" terms.
    /// NOTE: use this approach only for moderatley large problems and only if the inverse calculation is not
    /// expensive (e.g., for a constant mass matrix for which the inverse van be pre-computed).
    void SetMassInverse(ChMatrixConstRef M_inverse);

    /// Return true if an inverse mass matrix was provided.
    bool HasMassInverse() const { return m_use_Minv; }

    /// Update constraints in the system.
    /// This function calls ChConstraint::UpdateAuxiliary to calculate entries in the Schur complement matrix
    /// corresponding to each constraint and, optionally, averages these entries for friction constraints (NSC).
    void SchurComplementUpdateConstraints(bool average);

    /// Update all variables in the system.
    /// The 'q' data for the system ChVariables will be
    /// <pre>
    ///   incremented by: M^(-1) * Cq' * l,         if 'Mif' is NULL, or
    ///   set to:         Mif + M^(-1) * Cq' * l,   if 'Mif' is non-NULL,
    /// </pre>
    /// where [l] is the vector of Lagrange multipliers.
    void SchurComplementIncrementVariables(const ChVectorDynamic<>* Mif = nullptr);

    /// Update all variables in the system by M^(-1) * Cq' * lvector.
    void SchurComplementIncrementVariables(const ChVectorDynamic<>& lvector);

    /// Performs the product of N, the Schur complement of the KKT matrix, by an 'l' vector.
    /// <pre>
    ///    result = N * l = [ CqM^(-1)Cq' - E ] * l
    /// </pre>
    /// where [Cq] is the constraint Jacobian, [M] is the mass matrix, [E] is the matrix of the optional 'constraint
    /// force mixing' terms for compliant constraints. If an inverse mass matrix is not provided (default), the N matrix
    /// is never built explicitly; instead, it is described by the inserted constraints and inserted variables.
    /// If non-NULL, the vector idiag will contain the inverse diagonal of the Schur complement matrix N (automatically
    /// resized).
    /// NOTES:
    /// - the 'q' data in the ChVariables of the system descriptor is changed by this operation, so it may need to be
    ///   cached them via FromVariablesToVector().
    /// - unless the inverse of the matrix (M+K) is provided, this function does NOT support systems that include
    ///   ChKRMBlock objects.
    virtual void SchurComplementProduct(ChVectorDynamic<>& result,          ///< result of  N * l_i
                                        const ChVectorDynamic<>& lvector,   ///< vector to be multiplied
                                        ChVectorDynamic<>* idiag = nullptr  ///< (optional) inverse diagonal
    );

    /// Calculate the RHS of the Schur complement equation.
    /// <pre>
    ///    result = Cq'*(M^-1) * f + b
    /// </pre>
    /// where [Cq] is the constraint Jacobian, [M] is the mass matrix, and [b] is the RHS of the KKT equation.
    /// If non-NULL, the vector Mif will contain the intermediate values (M^-1)*f (automatically resized).
    void SchurComplementRHS(ChVectorDynamic<>& b_Schur,       ///< transformed RHS
                            ChVectorDynamic<>* Mif = nullptr  ///< (optional) intermediate quantity Minv*f
    );

    // ------------------------------------------

    /// Performs the product of the entire system matrix (KKT matrix), by a vector x={q,l}.
    /// Note that the 'q' data in the ChVariables of the system descriptor is changed by this operation, so that may
    /// need to be backed up via FromVariablesToVector().
    virtual void SystemProduct(ChVectorDynamic<>& result,  ///< result vector (multiplication of system matrix by x)
                               const ChVectorDynamic<>& x  ///< vector to be multiplied
    );

    /// Compute upper part of system descriptor product, as in `[Z]*y = d -> res = [H]*v + [CqT]*l`.
    /// Note:
    /// - 'result' is automatically resized
    /// - if negate_lambda = true, automatically flip sign to provided lambda.
    void SystemProductUpper(ChVectorDynamic<>& result,   ///< result vector
                            const ChVectorDynamic<>& v,  ///< primal variable (e.g. generalized velocity)
                            const ChVectorDynamic<>& l,  ///< dual variable (e.g. lagrange multipliers)
                            bool negate_lambda           ///< flip sign to dual variable
    );

    /// Compute lower part of system descriptor product, as in `[Z]*y = d -> res = [Cq]*v + [E]*l`.
    /// Note:
    /// - 'result' is automatically resized
    /// - if negate_lambda = true, automatically flip sign to provided lambda.
    void SystemProductLower(ChVectorDynamic<>& result,   ///< result vector
                            const ChVectorDynamic<>& v,  ///< primal variable (e.g. generalized velocity)
                            const ChVectorDynamic<>& l,  ///< dual variable (e.g. lagrange multipliers)
                            bool negate_lambda           ///< flip sign to dual variable
    );

    // ------------------------------------------

    /// Performs projection of constraint multipliers onto allowed set (in case
    /// of bilateral constraints it does not affect multipliers, but for frictional
    /// constraints, for example, it projects multipliers onto the friction cones)
    /// Note! the 'l_i' data in the ChConstraints of the system descriptor are changed
    /// by this operation (they get the value of 'multipliers' after the projection), so
    /// it may happen that you need to backup them via FromConstraintToVector().
    virtual void ConstraintsProject(
        ChVectorDynamic<>& multipliers  ///< system-level vector of 'l_i' multipliers to be projected
    );

    /// As ConstraintsProject(), but instead of passing the l vector, the entire
    /// vector of unknowns x={q,-l} is passed.
    /// Note! the 'l_i' data in the ChConstraints of the system descriptor are changed
    /// by this operation (they get the value of 'multipliers' after the projection), so
    /// it may happen that you need to backup them via FromConstraintToVector().
    virtual void UnknownsProject(
        ChVectorDynamic<>& mx  ///< system-level vector of unknowns x={q,-l} (only the l part is projected)
    );

    /// The following (obsolete) function may be called after a solver's 'Solve()'
    /// operation has been performed. This gives an estimate of 'how
    /// good' the solver had been in finding the proper solution.
    /// Resulting estimates are passed as references in member arguments.
    virtual void ComputeFeasabilityViolation(
        double& resulting_maxviolation,  ///< gets the max constraint violation (either bi- and unilateral.)
        double& resulting_feasability    ///< gets the max feasability as max |l*c| , for unilateral only
    );

    // LOGGING/OUTPUT/ETC.

    /// Paste the stiffness, damping or mass matrix of the system into a sparse matrix.
    /// Before calling this function the user needs to:
    /// - resize Z (and potentially call SetZeroValues if the case)
    /// - call LoadKRMMatrices with the desired factors
    /// - call SetMassFactor() with the appropriate value
    void PasteMassKRMMatrixInto(ChSparseMatrix& Z,
                                unsigned int start_row = 0,
                                unsigned int start_col = 0,
                                double scale_factor = 1) const;

    /// Paste the constraints jacobian of the system into a sparse matrix at a given position.
    /// Before calling this function the user needs to:
    /// - resize Z (and potentially call SetZeroValues if the case)
    /// - call LoadConstraintJacobians
    /// Returns the number of pasted constraints.
    unsigned int PasteConstraintsJacobianMatrixInto(ChSparseMatrix& Z,
                                                    unsigned int start_row = 0,
                                                    unsigned int start_col = 0,
                                                    bool only_bilateral = false) const;

    /// Paste the _transposed_ constraints jacobian of the system into a sparse matrix at a given position.
    /// Before calling this function the user needs to:
    /// - resize Z (and potentially call SetZeroValues if the case)
    /// - call LoadConstraintJacobians
    /// Returns the number of pasted constraints.
    unsigned int PasteConstraintsJacobianMatrixTransposedInto(ChSparseMatrix& Z,
                                                              unsigned int start_row = 0,
                                                              unsigned int start_col = 0,
                                                              bool only_bilateral = false) const;

    /// Paste E, the negative compliance matrix of the system into a sparse matrix at a given position.
    /// Before calling this function the user needs to:
    /// - resize Z (and potentially call SetZeroValues if the case)
    /// - call LoadKRMMatrices with the desired factors
    /// - call SetMassFactor() with the appropriate value
    void PasteComplianceMatrixInto(ChSparseMatrix& Z,
                                   unsigned int start_row = 0,
                                   unsigned int start_col = 0,
                                   double scale_factor = 1,
                                   bool only_bilateral = false) const;

    /// Create and return the assembled system matrix and/or RHS vector at a given position.
    virtual void BuildSystemMatrix(ChSparseMatrix* Z,       ///< assembled system matrix
                                   ChVectorDynamic<>* rhs,  ///< assembled RHS vector
                                   double scale_factor = 1  ///< optional scaling factor for generalized mass matrix
    ) const;

    /// Write the current system matrix blocks and right-hand side components.
    /// The system matrix is formed by calling BuildSystemMatrix() as used with direct linear solvers.
    /// The following files are written in the directory specified by [path]:
    /// - [prefix]_H.dat   masses and/or stiffness (Matlab sparse format)
    /// - [prefix]_Cq.dat  Jacobians (Matlab sparse format)
    /// - [prefix]_E.dat   constraint negative compliance (Matlab sparse format)
    /// - [prefix]_f.dat   applied loads
    /// - [prefix]_b.dat   constraint rhs
    /// By default, uses 1-based indices (as in Matlab).
    virtual void WriteMatrixBlocks(const std::string& path, const std::string& prefix, bool one_indexed = true);

    /// Write the current assembled system matrix and right-hand side vector.
    /// The system matrix is formed by calling BuildSystemMatrix() as used with direct linear solvers.
    /// The following files are written in the directory specified by [path]:
    /// - [prefix]_Z.dat    the assembled optimization matrix (COO sparse format)
    /// - [prefix]_rhs.dat  the assmbled RHS
    /// By default, uses 1-based indices (as in Matlab).
    virtual void WriteMatrix(const std::string& path, const std::string& prefix, bool one_indexed = true);

    /// Write the current assembled system matrix and right-hand side vector.
    /// The system matrix is formed by multiple calls to SystemProduct() as used with iterative linear solvers.
    /// The following files are written in the directory specified by [path]:
    /// - [prefix]_Z.dat    the assembled optimization matrix (Matlab sparse format)
    /// - [prefix]_rhs.dat  the assmbled RHS
    /// By default, uses 1-based indices (as in Matlab).
    virtual void WriteMatrixSpmv(const std::string& path, const std::string& prefix, bool one_indexed = true);

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOut(ChArchiveOut& archive_out) {
        // version number
        archive_out.VersionWrite<ChSystemDescriptor>();
        // serialize parent class
        // serialize all member data:
    }

    /// Method to allow de-serialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& archive_in) {
        // version number
        /*int version =*/archive_in.VersionRead<ChSystemDescriptor>();
        // deserialize parent class
        // stream in all member data:
    }

  protected:
    std::vector<ChConstraint*> m_constraints;  ///< list of all constraints in the current Chrono system
    std::vector<ChVariables*> m_variables;     ///< list of all variables in the current Chrono system
    std::vector<ChKRMBlock*> m_KRMblocks;      ///< list of all KRM blocks in the current Chrono system

    double c_a;  ///< coefficient form M mass matrices in m_variables

  private:
    mutable unsigned int n_q;  ///< number of active variables
    mutable unsigned int n_c;  ///< number of active constraints
    bool freeze_count;         ///< cache the number of active variables and constraints

    bool m_use_Minv;
    ChMatrixDynamic<> m_Minv;
};

CH_CLASS_VERSION(ChSystemDescriptor, 0)

}  // end namespace chrono

#endif
