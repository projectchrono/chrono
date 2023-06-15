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
#include "chrono/solver/ChKblock.h"
#include "chrono/solver/ChVariables.h"

namespace chrono {

/// Base class for collecting objects inherited from ChConstraint,
/// ChVariables and optionally ChKblock. These objects
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
/// 1. most often you call ConvertToMatrixForm() right after a dynamic simulation step,
///    in order to get the system matrices updated to the last timestep;
/// 2. when using Anitescu default stepper, the 'f' vector contains forces*timestep = F*dt
/// 3. when using Anitescu default stepper, 'q' represents the 'delta speed',
/// 4. when using Anitescu default stepper, 'b' represents the dt/phi stabilization term.
/// 5. usually, H = M, the mass matrix, but in some cases, ex. when using
///         implicit integrators, objects inherited from ChKblock can be added
///         too, hence H could be H=a*M+b*K+c*R (but not all solvers handle ChKblock!)
///
/// All solvers require that the description of the system
/// is passed by means of a ChSystemDescriptor object
/// that holds a list of all the constraints, variables, masses, known terms
///	(ex.forces) under the form of ChVariables, ChConstraints and ChKblock.
///
/// In this default implementation, the ChSystemDescriptor
/// simply holds a vector of pointers to ChVariables
/// or to ChConstraints, but more advanced implementations (ex. for
/// supporting parallel GPU solvers) could store constraints
/// and variables structures with other, more efficient data schemes.

class ChApi ChSystemDescriptor {
  protected:
    std::vector<ChConstraint*> vconstraints;  ///< list of pointers to all the ChConstraint in the current Chrono system
    std::vector<ChVariables*> vvariables;     ///< list of pointers to all the ChVariables in the current Chrono system
    std::vector<ChKblock*> vstiffness;        ///< list of pointers to all the ChKblock in the current Chrono system

    double c_a;  // coefficient form M mass matrices in vvariables

  private:
    int n_q;            ///< number of active variables
    int n_c;            ///< number of active constraints
    bool freeze_count;  ///< for optimization: avoid to re-count the number of active variables and constraints

  public:
    /// Constructor
    ChSystemDescriptor();

    /// Destructor
    virtual ~ChSystemDescriptor();

    // DATA MANAGEMENT FUNCTIONS

    /// Access the vector of constraints
    std::vector<ChConstraint*>& GetConstraintsList() { return vconstraints; }

    /// Access the vector of variables
    std::vector<ChVariables*>& GetVariablesList() { return vvariables; }

    /// Access the vector of stiffness matrix blocks
    std::vector<ChKblock*>& GetKblocksList() { return vstiffness; }

    /// Begin insertion of items
    virtual void BeginInsertion() {
        vconstraints.clear();
        vvariables.clear();
        vstiffness.clear();
    }

    /// Insert reference to a ChConstraint object
    virtual void InsertConstraint(ChConstraint* mc) { vconstraints.push_back(mc); }

    /// Insert reference to a ChVariables object
    virtual void InsertVariables(ChVariables* mv) { vvariables.push_back(mv); }

    /// Insert reference to a ChKblock object (a piece of matrix)
    virtual void InsertKblock(ChKblock* mk) { vstiffness.push_back(mk); }

    /// End insertion of items.
    /// A derived class should always call UpdateCountsAndOffsets.
    virtual void EndInsertion() { UpdateCountsAndOffsets(); }

    /// Count & returns the scalar variables in the system (excluding ChVariable objects
    /// that have  IsActive() as false). Note: the number of scalar variables is not necessarily
    /// the number of inserted ChVariable objects, some could be inactive.
    /// Note: this function also updates the offsets of all variables
    /// in 'q' global vector (see GetOffset() in ChVariables).
    virtual int CountActiveVariables();

    /// Count & returns the scalar constraints in the system (excluding ChConstraint objects
    /// that have  IsActive() as false).
    /// Note: this function also updates the offsets of all constraints
    /// in 'l' global vector (see GetOffset() in ChConstraint).
    virtual int CountActiveConstraints();

    /// Updates counts of scalar variables and scalar constraints,
    /// if you added/removed some item or if you switched some active state,
    /// otherwise CountActiveVariables() and CountActiveConstraints() might fail.
    virtual void UpdateCountsAndOffsets();

    /// Sets the c_a coefficient (default=1) used for scaling the M masses of the vvariables
    /// when performing ShurComplementProduct(), SystemProduct(), ConvertToMatrixForm(),
    virtual void SetMassFactor(const double mc_a) { c_a = mc_a; }

    /// Gets the c_a coefficient (default=1) used for scaling the M masses of the vvariables
    /// when performing ShurComplementProduct(), SystemProduct(), ConvertToMatrixForm(),
    virtual double GetMassFactor() { return c_a; }

    // DATA <-> MATH.VECTORS FUNCTIONS

    /// Get a vector with all the 'fb' known terms ('forces'etc.) associated to all variables,
    /// ordered into a column vector. The column vector must be passed as a ChMatrix<>
    /// object, which will be automatically reset and resized to the proper length if necessary.
    virtual int BuildFbVector(ChVectorDynamic<>& Fvector  ///< system-level vector 'f'
    );

    /// Get a vector with all the 'bi' known terms ('constraint residuals' etc.) associated to all constraints,
    /// ordered into a column vector. The column vector must be passed as a ChMatrix<>
    /// object, which will be automatically reset and resized to the proper length if necessary.
    virtual int BuildBiVector(ChVectorDynamic<>& Bvector  ///< system-level vector 'b'
    );

    /// Get the d vector = {f; -b} with all the 'fb' and 'bi' known terms, as in  Z*y-d
    /// (it is the concatenation of BuildFbVector and BuildBiVector) The column vector must be passed as a ChMatrix<>
    /// object, which will be automatically reset and resized to the proper length if necessary.
    virtual int BuildDiVector(ChVectorDynamic<>& Dvector  ///< system-level vector {f;-b}
    );

    /// Get the D diagonal of the Z system matrix, as a single column vector (it includes all the diagonal
    /// masses of M, and all the diagonal E (-cfm) terms).
    /// The Diagonal_vect must already have the size of n. of unknowns, otherwise it will be resized if necessary).
    virtual int BuildDiagonalVector(
        ChVectorDynamic<>& Diagonal_vect  ///< system-level vector of terms on M and E diagonal
    );

    /// Using this function, one may get a vector with all the variables 'q'
    /// ordered into a column vector. The column vector must be passed as a ChMatrix<>
    /// object, which will be automatically reset and resized to the proper length if necessary
    /// (but if you are sure that the vector has already the proper size, you can optimize
    /// the performance a bit by setting resize_vector as false).
    /// \return  the number of scalar variables (i.e. the rows of the column vector).
    virtual int FromVariablesToVector(ChVectorDynamic<>& mvector,  ///< system-level vector 'q'
                                      bool resize_vector = true    ///< if true, resize vector as necessary
    );

    /// Using this function, one may go in the opposite direction of the FromVariablesToVector()
    /// function, i.e. one gives a vector with all the variables 'q' ordered into a column vector, and
    /// the variables objects are updated according to these values.
    /// NOTE!!! differently from  FromVariablesToVector(), which always works, this
    /// function will fail if mvector does not match the amount and ordering of
    /// the variable objects!!! (it is up to the user to check this!) btw: most often,
    /// this is called after FromVariablesToVector() to do a kind of 'undo', for example.
    /// \return  the number of scalar variables (i.e. the rows of the column vector).
    virtual int FromVectorToVariables(const ChVectorDynamic<>& mvector  ///< system-level vector 'q'
    );

    /// Using this function, one may get a vector with all the constraint reactions 'l_i'
    /// ordered into a column vector. The column vector must be passed as a ChMatrix<>
    /// object, which will be automatically reset and resized to the proper length if necessary
    /// (but uf you are sure that the vector has already the proper size, you can optimize
    /// the performance a bit by setting resize_vector as false).
    /// Optionally, you can pass an 'enabled' vector of bools, that must have the same
    /// length of the l_i reactions vector; constraints with enabled=false are not handled.
    /// \return  the number of scalar constr.multipliers (i.e. the rows of the column vector).
    virtual int FromConstraintsToVector(ChVectorDynamic<>& mvector,  ///< system-level vector 'l_i'
                                        bool resize_vector = true    ///< if true, resize vector as necessary
    );

    /// Using this function, one may go in the opposite direction of the FromConstraintsToVector()
    /// function, i.e. one gives a vector with all the constr.reactions 'l_i' ordered into a column vector, and
    /// the constraint objects are updated according to these values.
    /// Optionally, you can pass an 'enabled' vector of bools, that must have the same
    /// length of the l_i reactions vector; constraints with enabled=false are not handled.
    /// NOTE!!! differently from  FromConstraintsToVector(), which always works, this
    /// function will fail if mvector does not match the amount and ordering of
    /// the variable objects!!! (it is up to the user to check this!) btw: most often,
    /// this is called after FromConstraintsToVector() to do a kind of 'undo', for example.
    /// \return  the number of scalar constraint multipliers (i.e. the rows of the column vector).
    virtual int FromVectorToConstraints(const ChVectorDynamic<>& mvector  ///< system-level vector 'l_i'
    );

    /// Using this function, one may get a vector with all the unknowns x={q,l} i.e. q variables & l_i constr.
    /// ordered into a column vector. The column vector must be passed as a ChMatrix<>
    /// object, which will be automatically reset and resized to the proper length if necessary
    /// (but if you are sure that the vector has already the proper size, you can optimize
    /// the performance a bit by setting resize_vector as false).
    /// \return  the number of scalar unknowns
    virtual int FromUnknownsToVector(ChVectorDynamic<>& mvector,  ///< system-level vector x={q,l}
                                     bool resize_vector = true    ///< if true, resize vector as necessary
    );

    /// Using this function, one may go in the opposite direction of the FromUnknownsToVector()
    /// function, i.e. one gives a vector with all the unknowns x={q,l} ordered into a column vector, and
    /// the variables q and constr.multipliers l objects are updated according to these values.
    /// NOTE!!! differently from  FromUnknownsToVector(), which always works, this
    /// function will fail if mvector does not match the amount and ordering of
    /// the variable and constraint objects!!! (it is up to the user to check this!)
    virtual int FromVectorToUnknowns(const ChVectorDynamic<>& mvector  ///< system-level vector x={q,l}
    );

    // MATHEMATICAL OPERATIONS ON DATA

    /// Performs the product of N, the Shur complement of the KKT matrix, by an 'l' vector
    /// <pre>
    ///    result = [N]*l = [ [Cq][M^(-1)][Cq'] - [E] ] * l
    /// </pre>
    /// where [Cq] are the jacobians, [M] is the mass matrix, [E] is the matrix
    /// of the optional cfm 'constraint force mixing' terms for compliant constraints.
    /// The N matrix is not built explicitly, to exploit sparsity, it is described by the
    /// inserted constraints and inserted variables.
    /// Optionally, you can pass an 'enabled' vector of bools, that must have the same
    /// length of the l_i reactions vector; constraints with enabled=false are not handled.
    /// NOTE! the 'q' data in the ChVariables of the system descriptor is changed by this
    /// operation, so it may happen that you need to backup them via FromVariablesToVector()
    /// NOTE! currently this function does NOT support the cases that use also ChKblock
    /// objects, because it would need to invert the global M+K, that is not diagonal,
    /// for doing = [N]*l = [ [Cq][(M+K)^(-1)][Cq'] - [E] ] * l
    virtual void ShurComplementProduct(
        ChVectorDynamic<>& result,            ///< result of  N * l_i
        const ChVectorDynamic<>& lvector,     ///< vector to be multiplied
        std::vector<bool>* enabled = nullptr  ///< optional: vector of "enabled" flags, one per scalar constraint.
    );

    /// Performs the product of the entire system matrix (KKT matrix), by a vector x ={q,l}.
    /// Note that the 'q' data in the ChVariables of the system descriptor is changed by this
    /// operation, so thay may need to be backed up via FromVariablesToVector()
    virtual void SystemProduct(ChVectorDynamic<>& result,  ///< result vector (multiplication of system matrix by x)
                               const ChVectorDynamic<>& x  ///< vector to be multiplied
    );

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

    /// The following function may be used to create the Jacobian and the
    /// mass matrix of the variational problem in matrix form, by assembling all
    /// the jacobians of all the constraints/contacts, all the mass matrices, all vectors,
    /// as they are _currently_ stored in the sparse data of all ChConstraint and ChVariables
    /// contained in this ChSystemDescriptor.
    ///
    /// This can be useful for debugging, data dumping, and similar purposes (most solvers avoid
    /// using these matrices, for performance), for example you will load these matrices in Matlab.
    /// Optionally, tangential (u,v) contact jacobians may be skipped, or only bilaterals can be considered
    /// The matrices and vectors are automatically resized if needed.
    virtual void ConvertToMatrixForm(
        ChSparseMatrix* Cq,            ///< fill this system jacobian matrix, if not null
        ChSparseMatrix* H,             ///< fill this system H (mass+stiffness+damp) matrix, if not null
        ChSparseMatrix* E,             ///< fill this system 'compliance' matrix , if not null
        ChVectorDynamic<>* Fvector,    ///< fill this vector as the known term 'f', if not null
        ChVectorDynamic<>* Bvector,    ///< fill this vector as the known term 'b', if not null
        ChVectorDynamic<>* Frict,      ///< fill as a vector with friction coefficients (=-1 for tangent comp.; =-2 for
                                       ///< bilaterals), if not null
        bool only_bilaterals = false,  ///< skip unilateral constraints
        bool skip_contacts_uv = false  ///< skip the tangential reaction constraints
    );

    /// Create and return the assembled system matrix and RHS vector.
    virtual void ConvertToMatrixForm(ChSparseMatrix* Z,      ///< [out] assembled system matrix
                                     ChVectorDynamic<>* rhs  ///< [out] assembled RHS vector
    );

    /// Write the current assembled system matrix and right-hand side vector.
    /// The system matrix is formed by calling ConvertToMatrixForm() as used with direct linear solvers.
    /// The following files are written in the directory specified by [path]:
    /// - [prefix]_Z.dat    the assembled optimization matrix (Matlab sparse format)
    /// - [prefix]_rhs.dat  the assmbled RHS
    virtual void WriteMatrix(const std::string& path, const std::string& prefix);

    /// Write the current system matrix blocks and right-hand side components.
    /// The system matrix is formed by calling ConvertToMatrixForm() as used with direct linear solvers.
    /// The following files are written in the directory specified by [path]:
    /// - [prefix]_H.dat   masses and/or stiffness (Matlab sparse format)
    /// - [prefix]_Cq.dat  Jacobians (Matlab sparse format)
    /// - [prefix]_E.dat   constraint compliance (Matlab sparse format)
    /// - [prefix]_f.dat   applied loads
    /// - [prefix]_b.dat   constraint rhs
    virtual void WriteMatrixBlocks(const std::string& path, const std::string& prefix);

    /// Write the current assembled system matrix and right-hand side vector.
    /// The system matrix is formed by multiple calls to SystemProduct() as used with iterative linear solvers.
    /// The following files are written in the directory specified by [path]:
    /// - [prefix]_Z.dat    the assembled optimization matrix (Matlab sparse format)
    /// - [prefix]_rhs.dat  the assmbled RHS
    virtual void WriteMatrixSpmv(const std::string& path, const std::string& prefix);

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOut(ChArchiveOut& marchive) {
        // version number
        marchive.VersionWrite<ChSystemDescriptor>();
        // serialize parent class
        // serialize all member data:
    }

    /// Method to allow de-serialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& marchive) {
        // version number
        /*int version =*/ marchive.VersionRead<ChSystemDescriptor>();
        // deserialize parent class
        // stream in all member data:
    }
};

CH_CLASS_VERSION(ChSystemDescriptor, 0)

}  // end namespace chrono

#endif
