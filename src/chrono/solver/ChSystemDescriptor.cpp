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

#include <iomanip>

#include "chrono/solver/ChSystemDescriptor.h"

namespace chrono {

// Register into the object factory, to enable run-time
// dynamic creation and persistence
CH_FACTORY_REGISTER(ChSystemDescriptor)

#define CH_SPINLOCK_HASHSIZE 203

ChSystemDescriptor::ChSystemDescriptor() : n_q(0), n_c(0), c_a(1.0), freeze_count(false), m_use_Minv(false) {
    m_constraints.clear();
    m_variables.clear();
    m_KRMblocks.clear();
}

ChSystemDescriptor::~ChSystemDescriptor() {
    m_constraints.clear();
    m_variables.clear();
    m_KRMblocks.clear();
}

bool ChSystemDescriptor::HasKRMBlocks() {
    return m_KRMblocks.size() > 0;
}

bool ChSystemDescriptor::SupportsSchurComplement() {
    // if no KRM blocks, return true
    if (m_KRMblocks.size() == 0)
        return true;

    // if any KRM block has KR components, return false
    for (const auto& KRMBlock : m_KRMblocks) {
        if (KRMBlock->HasKRComponents())
            return false;
    }

    // return true only if an inverse mass matrix was provided
    return m_use_Minv;
}

void ChSystemDescriptor::ComputeFeasabilityViolation(double& resulting_maxviolation, double& resulting_feasability) {
    resulting_maxviolation = 0;
    resulting_feasability = 0;

    for (const auto& constr : m_constraints) {
        // the the residual of the constraint..
        double mres_i = constr->ComputeResidual();

        double candidate_violation = fabs(constr->Violation(mres_i));

        if (candidate_violation > resulting_maxviolation)
            resulting_maxviolation = candidate_violation;

        if (constr->IsUnilateral()) {
            double candidate_feas = fabs(mres_i * constr->GetLagrangeMultiplier());  // =|c*l|
            if (candidate_feas > resulting_feasability)
                resulting_feasability = candidate_feas;
        }
    }
}

unsigned int ChSystemDescriptor::CountActiveVariables() const {
    if (freeze_count)  // optimization, avoid list count all times
        return n_q;

    n_q = 0;
    for (auto& var : m_variables) {
        if (var->IsActive()) {
            var->SetOffset(n_q);  // also store offsets in state and MC matrix
            n_q += var->GetDOF();
        }
    }
    return n_q;
}

unsigned int ChSystemDescriptor::CountActiveConstraints() const {
    if (freeze_count)  // optimization, avoid list count all times
        return n_c;

    n_c = 0;
    for (auto& constr : m_constraints) {
        if (constr->IsActive()) {
            constr->SetOffset(n_c);  // also store offsets in state and MC matrix
            n_c++;
        }
    }
    return n_c;
}

void ChSystemDescriptor::UpdateCountsAndOffsets() {
    freeze_count = false;
    CountActiveVariables();
    CountActiveConstraints();
    freeze_count = true;
}

void ChSystemDescriptor::PasteMassKRMMatrixInto(ChSparseMatrix& Z,
                                                unsigned int start_row,
                                                unsigned int start_col) const {
    // Contribution of mass or rigid bodies and node-concentrated masses
    for (const auto& var : m_variables) {
        if (var->IsActive()) {
            var->PasteMassInto(Z, start_row, start_col, c_a);
        }
    }

    // Contribution of stiffness, damping, and mass matrices
    for (const auto& KRMBlock : m_KRMblocks) {
        KRMBlock->PasteMatrixInto(Z, start_row, start_col, false);
    }
}

unsigned int ChSystemDescriptor::PasteConstraintsJacobianMatrixInto(ChSparseMatrix& Z,
                                                                    unsigned int start_row,
                                                                    unsigned int start_col,
                                                                    bool only_bilateral) const {
    unsigned int s_c = 0;
    for (const auto& constr : m_constraints) {
        if (constr->IsActive() && !(only_bilateral && constr->GetMode() != ChConstraint::Mode::LOCK)) {
            constr->PasteJacobianInto(Z, s_c + start_row, start_col);
            s_c++;
        }
    }

    return s_c;
}

unsigned int ChSystemDescriptor::PasteConstraintsJacobianMatrixTransposedInto(ChSparseMatrix& Z,
                                                                              unsigned int start_row,
                                                                              unsigned int start_col,
                                                                              bool only_bilateral) const {
    unsigned int s_c = 0;
    for (const auto& constr : m_constraints) {
        if (constr->IsActive() && !(only_bilateral && constr->GetMode() != ChConstraint::Mode::LOCK)) {
            constr->PasteJacobianTransposedInto(Z, start_row, s_c + start_col);
            s_c++;
        }
    }

    return s_c;
}

void ChSystemDescriptor::PasteComplianceMatrixInto(ChSparseMatrix& Z,
                                                   unsigned int start_row,
                                                   unsigned int start_col,
                                                   bool only_bilateral) const {
    int s_c = 0;
    for (const auto& constr : m_constraints) {
        if (constr->IsActive() && !(only_bilateral && constr->GetMode() != ChConstraint::Mode::LOCK)) {
            Z.SetElement(start_row + s_c, start_col + s_c, -constr->GetComplianceTerm());
            s_c++;
        }
    }
}

void ChSystemDescriptor::BuildSystemMatrix(ChSparseMatrix* Z, ChVectorDynamic<>* rhs) const {
    n_q = CountActiveVariables();

    n_c = CountActiveConstraints();

    if (Z) {
        Z->conservativeResize(n_q + n_c, n_q + n_c);

        Z->setZeroValues();

        PasteMassKRMMatrixInto(*Z, 0, 0);

        PasteConstraintsJacobianMatrixInto(*Z, n_q, 0);

        PasteConstraintsJacobianMatrixTransposedInto(*Z, 0, n_q);

        PasteComplianceMatrixInto(*Z, n_q, n_q);
    }

    if (rhs) {
        rhs->setZero(n_q + n_c, 1);

        BuildDiVector(*rhs);
    }
}

unsigned int ChSystemDescriptor::BuildFbVector(ChVectorDynamic<>& f, unsigned int start_row) const {
    n_q = CountActiveVariables();
    f.setZero(n_q);

    // Fills the 'f' vector
    for (const auto& var : m_variables) {
        if (var->IsActive()) {
            f.segment(start_row + var->GetOffset(), var->GetDOF()) = var->Force();
        }
    }
    return n_q;
}

unsigned int ChSystemDescriptor::BuildBiVector(ChVectorDynamic<>& b, unsigned int start_row) const {
    n_c = CountActiveConstraints();
    b.setZero(n_c);

    // Fill the 'b' vector
    for (const auto& constr : m_constraints) {
        if (constr->IsActive()) {
            b(start_row + constr->GetOffset()) = constr->GetRightHandSide();
        }
    }

    return n_c;
}

unsigned int ChSystemDescriptor::BuildDiVector(ChVectorDynamic<>& d) const {
    n_q = CountActiveVariables();
    n_c = CountActiveConstraints();
    d.setZero(n_q + n_c);

    // Fills the 'f' vector part
    for (const auto& var : m_variables) {
        if (var->IsActive()) {
            d.segment(var->GetOffset(), var->GetDOF()) = var->Force();
        }
    }

    // Fill the '-b' vector (with flipped sign!)
    for (const auto& constr : m_constraints) {
        if (constr->IsActive()) {
            d(constr->GetOffset() + n_q) = -constr->GetRightHandSide();
        }
    }

    return n_q + n_c;
}

unsigned int ChSystemDescriptor::BuildDiagonalVector(ChVectorDynamic<>& diagonal_vect) const {
    n_q = CountActiveVariables();
    n_c = CountActiveConstraints();
    diagonal_vect.setZero(n_q + n_c);

    // Fill the diagonal values given by ChKRMBlock objects , if any
    // (This cannot be easily parallelized because of possible write concurrency).
    for (const auto& krm_block : m_KRMblocks) {
        krm_block->DiagonalAdd(diagonal_vect);
    }

    // Get the 'M' diagonal terms given by ChVariables objects
    for (const auto& var : m_variables) {
        if (var->IsActive()) {
            var->AddMassDiagonalInto(diagonal_vect, c_a);
        }
    }

    // Get the 'E' diagonal terms (E_i = - cfm_i )
    for (const auto& constr : m_constraints) {
        if (constr->IsActive()) {
            diagonal_vect(constr->GetOffset() + n_q) = -constr->GetComplianceTerm();
        }
    }

    return n_q + n_c;
}

unsigned int ChSystemDescriptor::FromVariablesToVector(ChVectorDynamic<>& vector, bool resize_vector) const {
    // Count active variables and resize vector if necessary
    if (resize_vector) {
        n_q = CountActiveVariables();
        vector.setZero(n_q);
    }

    // Fill the vector
    for (const auto& var : m_variables) {
        if (var->IsActive()) {
            vector.segment(var->GetOffset(), var->GetDOF()) = var->State();
        }
    }

    return n_q;
}

unsigned int ChSystemDescriptor::FromVectorToVariables(const ChVectorDynamic<>& vector) {
    assert(CountActiveVariables() == vector.rows());

    // fetch from the vector
    for (const auto& var : m_variables) {
        if (var->IsActive()) {
            var->State() = vector.segment(var->GetOffset(), var->GetDOF());
        }
    }

    return n_q;
}

unsigned int ChSystemDescriptor::FromConstraintsToVector(ChVectorDynamic<>& vector, bool resize_vector) const {
    // Count active constraints and resize vector if necessary
    if (resize_vector) {
        n_c = CountActiveConstraints();
        vector.setZero(n_c);
    }

    // Fill the vector
    for (const auto& constr : m_constraints) {
        if (constr->IsActive()) {
            vector(constr->GetOffset()) = constr->GetLagrangeMultiplier();
        }
    }

    return n_c;
}

unsigned int ChSystemDescriptor::FromVectorToConstraints(const ChVectorDynamic<>& vector) {
    n_c = CountActiveConstraints();

    assert(n_c == vector.size());

    // Fill the vector
    for (const auto& constr : m_constraints) {
        if (constr->IsActive()) {
            constr->SetLagrangeMultiplier(vector(constr->GetOffset()));
        }
    }

    return n_c;
}

unsigned int ChSystemDescriptor::FromUnknownsToVector(ChVectorDynamic<>& vector, bool resize_vector) const {
    // Count active variables & constraints and resize vector if necessary
    n_q = CountActiveVariables();
    n_c = CountActiveConstraints();

    if (resize_vector)
        vector.setZero(n_q + n_c);

    // Fill the first part of vector, x.q ,with variables q
    for (const auto& var : m_variables) {
        if (var->IsActive()) {
            vector.segment(var->GetOffset(), var->GetDOF()) = var->State();
        }
    }

    // Fill the second part of vector, x.l, with constraint multipliers -l (with flipped sign!)
    for (const auto& constr : m_constraints) {
        if (constr->IsActive()) {
            vector(constr->GetOffset() + n_q) = -constr->GetLagrangeMultiplier();
        }
    }

    return n_q + n_c;
}

unsigned int ChSystemDescriptor::FromVectorToUnknowns(const ChVectorDynamic<>& vector) {
    n_q = CountActiveVariables();
    n_c = CountActiveConstraints();

    assert((n_q + n_c) == vector.size());

    // Fetch from the first part of vector (x.q = q)
    for (const auto& var : m_variables) {
        if (var->IsActive()) {
            var->State() = vector.segment(var->GetOffset(), var->GetDOF());
        }
    }

    // Fetch from the second part of vector (x.l = -l), with flipped sign!
    for (const auto& constr : m_constraints) {
        if (constr->IsActive()) {
            constr->SetLagrangeMultiplier(-vector(constr->GetOffset() + n_q));
        }
    }

    return n_q + n_c;
}

void ChSystemDescriptor::SetMassInverse(ChMatrixConstRef M_inverse) {
    m_use_Minv = true;
    m_Minv = M_inverse;
}

void ChSystemDescriptor::SchurComplementProduct(ChVectorDynamic<>& result, const ChVectorDynamic<>& lvector) {
    // nothing to do if no constraints
    if (n_c == 0)
        return;

    assert(SupportsSchurComplement());
    assert(lvector.size() == CountActiveConstraints());

    result.setZero(n_c);

    // Use the inverse mass matrix if provided
    //// TODO: take into account only active constraints
    if (m_use_Minv) {
        ChSparseMatrix Cq(n_c, n_q);
        ChSparseMatrix E(n_c, n_c);
        Cq.setZero();
        E.setZero();
        PasteConstraintsJacobianMatrixInto(Cq);
        PasteComplianceMatrixInto(E);
        result = (Cq * m_Minv * Cq.transpose() - E) * lvector;
        return;
    }

    // 1 - set the qb vector (aka speeds) to zero, for each ChVariable
    for (const auto& var : m_variables) {
        if (var->IsActive())
            var->State().setZero();
    }

    // 2 - calculate qb = [M^(-1)][Cq']*l
    //     Also, begin to add the compliance term ( -[E]*l ) to the result.
    //     ATTENTION:  this loop cannot be parallelized, as concurrent writes to some q may happen.
    for (const auto& constr : m_constraints) {
        if (constr->IsActive()) {
            int s_c = constr->GetOffset();

            double li = lvector(s_c);

            // Compute qb += [M^(-1)][Cq']*l_i
            // NOTE: concurrent update to same q data, risk of collision if parallel
            constr->IncrementState(li);

            // Add -[E] * l_i
            // NOTE: compliance term  cfm = -E, so   -[E] * l_i = cfm * l_i
            result(s_c) = constr->GetComplianceTerm() * li;
        }
    }

    // 3 - calculate result = [Cq']*qb
    for (const auto& constr : m_constraints) {
        if (constr->IsActive()) {
            result(constr->GetOffset()) += constr->ComputeJacobianTimesState();
        }
    }
}

void ChSystemDescriptor::SchurComplementRHS(ChVectorDynamic<>& result, ChVectorDynamic<>* Mik) {
    assert(SupportsSchurComplement());

    // Use the inverse mass matrix if provided
    //// TODO: take into account only active constraints
    if (m_use_Minv) {
        ChVectorDynamic<double> f(n_q);
        ChVectorDynamic<double> b(n_c);
        f.setZero();
        b.setZero();
        BuildFbVector(f);
        BuildBiVector(b);
        if (n_c == 0) {
            if (Mik)
                *Mik = m_Minv * f;
            result = b;
            return;
        }
        ChSparseMatrix Cq(n_c, n_q);
        Cq.setZero();
        PasteConstraintsJacobianMatrixInto(Cq);
        if (Mik) {
            *Mik = m_Minv * f;
            result = -Cq * (*Mik) - b;
        } else {
            result = -Cq * m_Minv * f - b;
        }
        return;
    }

    // Load (M^-1)*k in q sparse vector of each variable
    for (const auto& var : m_variables) {
        if (var->IsActive())
            var->ComputeMassInverseTimesVector(var->State(), var->Force());
    }

    // Calculate b_schur = - Cq*q = - Cq*(M^-1)*k
    result.setZero();
    int s_i = 0;
    for (const auto& constr : m_constraints) {
        if (constr->IsActive()) {
            result(s_i, 0) = -constr->ComputeJacobianTimesState();
            ++s_i;
        }
    }

    // Calculate b_schur = b_schur - c
    ChVectorDynamic<> c(n_c);
    BuildBiVector(c);  // b_i = -c = phi/h
    result -= c;

    // Optionally cache q = (M^-1)*k calculated above
    if (Mik)
        FromVariablesToVector(*Mik, true);
}

void ChSystemDescriptor::SystemProduct(ChVectorDynamic<>& result, const ChVectorDynamic<>& x) {
    n_q = CountActiveVariables();
    n_c = CountActiveConstraints();

    result.setZero(n_q + n_c);

    // 1) First row: result.q =  [M + K]*x.q + [Cq']*x.l

    // 1.1)  add M*x.q
    for (const auto& var : m_variables) {
        if (var->IsActive()) {
            var->AddMassTimesVectorInto(result, x, c_a);
        }
    }

    // 1.2)  add also K*x.q  (NOT straight parallelizable - risk of concurrency in writing)
    for (const auto& krm_block : m_KRMblocks) {
        krm_block->AddMatrixTimesVectorInto(result, x);
    }

    // 1.3)  add also [Cq]'*x.l  (NOT straight parallelizable - risk of concurrency in writing)
    for (const auto& constr : m_constraints) {
        if (constr->IsActive()) {
            constr->AddJacobianTransposedTimesScalarInto(result, x(constr->GetOffset() + n_q));
        }
    }

    // 2) Second row: result.l =  [C_q]*x.q + [E]*x.l
    for (const auto& constr : m_constraints) {
        if (constr->IsActive()) {
            int s_c = constr->GetOffset() + n_q;
            constr->AddJacobianTimesVectorInto(result(s_c), x);    // result.l_i += [C_q_i]*x.q
            result(s_c) += -constr->GetComplianceTerm() * x(s_c);  // result.l_i += [E]*x.l_i = -cfm * x.l_i
        }
    }
}

void ChSystemDescriptor::SystemProductUpper(ChVectorDynamic<>& result,
                                            const ChVectorDynamic<>& v,
                                            const ChVectorDynamic<>& l,
                                            bool negate_lambda) {
    double lambda_sign = negate_lambda ? -1 : +1;
    n_q = CountActiveVariables();
    c_a = GetMassFactor();

    result.setZero(n_q);

    // First row: result.q part =  [M + K]*x.q + [Cq']*x.l

    // 1. do  M*x.q
    for (const auto& var : m_variables) {
        if (var->IsActive()) {
            var->AddMassTimesVectorInto(result, v, c_a);
        }
    }

    // 2. add also K*x.q  (NON straight parallelizable - risk of concurrency in writing)
    for (const auto& krm_block : m_KRMblocks) {
        krm_block->AddMatrixTimesVectorInto(result, v);
    }

    // 3. add also [Cq]'*x.l  (NON straight parallelizable - risk of concurrency in writing)
    for (const auto& constr : m_constraints) {
        if (constr->IsActive()) {
            constr->AddJacobianTransposedTimesScalarInto(result, lambda_sign * l(constr->GetOffset()));
        }
    }
}

void ChSystemDescriptor::SystemProductLower(ChVectorDynamic<>& result,
                                            const ChVectorDynamic<>& v,
                                            const ChVectorDynamic<>& l,
                                            bool negate_lambda) {
    double lambda_sign = negate_lambda ? -1 : +1;
    n_c = CountActiveConstraints();
    result.setZero(n_c);

    // Second row: result.l part =  [C_q]*x.q + [E]*x.l
    for (const auto& constr : m_constraints) {
        if (constr->IsActive()) {
            int s_c = constr->GetOffset();
            constr->AddJacobianTimesVectorInto(result(s_c), v);  // result.l_i += [C_q_i]*x.q
            result(s_c) +=
                -constr->GetComplianceTerm() * lambda_sign * l(s_c);  // result.l_i += [E]*x.l_i = -cfm * x.l_i
        }
    }
}

void ChSystemDescriptor::ConstraintsProject(ChVectorDynamic<>& multipliers) {
    FromVectorToConstraints(multipliers);

    for (const auto& constr : m_constraints) {
        if (constr->IsActive())
            constr->Project();
    }

    FromConstraintsToVector(multipliers, false);
}

void ChSystemDescriptor::UnknownsProject(ChVectorDynamic<>& mx) {
    n_q = CountActiveVariables();

    // vector -> constraints
    // Fetch from the second part of vector (x.l = -l), with flipped sign!
    for (const auto& constr : m_constraints) {
        if (constr->IsActive()) {
            constr->SetLagrangeMultiplier(-mx(constr->GetOffset() + n_q));
        }
    }

    // constraint projection!
    for (const auto& constr : m_constraints) {
        if (constr->IsActive())
            constr->Project();
    }

    // constraints -> vector
    // Fill the second part of vector, x.l, with constraint multipliers -l (with flipped sign!)
    for (const auto& constr : m_constraints) {
        if (constr->IsActive()) {
            mx(constr->GetOffset() + n_q) = -constr->GetLagrangeMultiplier();
        }
    }
}

// -----------------------------------------------------------------------------

void ChSystemDescriptor::WriteMatrixBlocks(const std::string& path, const std::string& prefix, bool one_indexed) {
    ChSparseMatrix mass_matrix(CountActiveVariables(), CountActiveVariables());
    ChSparseMatrix jacob_matrix(CountActiveConstraints(), CountActiveVariables());
    ChSparseMatrix compl_matrix(CountActiveConstraints(), CountActiveConstraints());
    ChVectorDynamic<double> f;
    ChVectorDynamic<double> b;

    mass_matrix.setZeroValues();
    jacob_matrix.setZeroValues();
    f.setZero(CountActiveVariables());
    b.setZero(CountActiveConstraints());

    PasteMassKRMMatrixInto(mass_matrix);
    PasteConstraintsJacobianMatrixInto(jacob_matrix);
    PasteComplianceMatrixInto(compl_matrix);
    BuildFbVector(f);
    BuildBiVector(b);

    std::ofstream file_H(path + "/" + prefix + "_H.dat");
    file_H << std::setprecision(12) << std::scientific;
    StreamOut(mass_matrix, file_H, one_indexed);

    std::ofstream file_Cq(path + "/" + prefix + "_Cq.dat");
    file_Cq << std::setprecision(12) << std::scientific;
    StreamOut(jacob_matrix, file_Cq, one_indexed);

    std::ofstream file_E(path + "/" + prefix + "_E.dat");
    file_E << std::setprecision(12) << std::scientific;
    StreamOut(compl_matrix, file_E, one_indexed);

    std::ofstream file_f(path + "/" + prefix + "_f.dat");
    file_f << std::setprecision(12) << std::scientific;
    StreamOut(f, file_f);

    std::ofstream file_b(path + "/" + prefix + "_b.dat");
    file_b << std::setprecision(12) << std::scientific;
    StreamOut(b, file_b);
}

void ChSystemDescriptor::WriteMatrix(const std::string& path, const std::string& prefix, bool one_indexed) {
    ChSparseMatrix Z;
    ChVectorDynamic<double> rhs;
    BuildSystemMatrix(&Z, &rhs);

    std::ofstream file_Z(path + "/" + prefix + "_Z.dat");
    file_Z << std::setprecision(12) << std::scientific;
    StreamOut(Z, file_Z, one_indexed);

    std::ofstream file_rhs(path + "/" + prefix + "_rhs.dat");
    file_rhs << std::setprecision(12) << std::scientific;
    StreamOut(rhs, file_rhs);
}

void ChSystemDescriptor::WriteMatrixSpmv(const std::string& path, const std::string& prefix, bool one_indexed) {
    // Count constraints.
    int mn_c = 0;
    for (auto& cnstr : m_constraints) {
        if (cnstr->IsActive())
            mn_c++;
    }

    // Generate dense system matrix, column by column
    int size = n_q + mn_c;
    ChMatrixDynamic<> A(size, size);
    ChVectorDynamic<> v(size);
    ChVectorDynamic<> e(size);
    e.setZero();

    for (int i = 0; i < size; i++) {
        e[i] = 1;
        SystemProduct(v, e);
        A.col(i) = v;
        e[i] = 0;
    }

    // Convert to Eigen sparse matrix
    ChSparseMatrix Z = A.sparseView();

    // Write sparse matrix to file
    std::ofstream file_Z(path + "/" + prefix + "_Z.dat");
    file_Z << std::setprecision(12) << std::scientific;
    StreamOut(Z, file_Z, one_indexed);

    // Write RHS to file
    ChVectorDynamic<double> rhs;
    BuildSystemMatrix(nullptr, &rhs);
    std::ofstream file_rhs(path + "/" + prefix + "_rhs.dat");
    file_rhs << std::setprecision(12) << std::scientific;
    StreamOut(rhs, file_rhs);
}

}  // end namespace chrono
