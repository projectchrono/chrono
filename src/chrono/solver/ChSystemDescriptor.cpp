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
#include "chrono/solver/ChConstraintTwoTuplesContactN.h"
#include "chrono/solver/ChConstraintTwoTuplesFrictionT.h"
#include "chrono/core/ChMatrix.h"

namespace chrono {

// Register into the object factory, to enable run-time
// dynamic creation and persistence
CH_FACTORY_REGISTER(ChSystemDescriptor)

#define CH_SPINLOCK_HASHSIZE 203

ChSystemDescriptor::ChSystemDescriptor() : n_q(0), n_c(0), c_a(1.0), freeze_count(false) {
    m_constraints.clear();
    m_variables.clear();
    m_KRMblocks.clear();
}

ChSystemDescriptor::~ChSystemDescriptor() {
    m_constraints.clear();
    m_variables.clear();
    m_KRMblocks.clear();
}

void ChSystemDescriptor::ComputeFeasabilityViolation(double& resulting_maxviolation, double& resulting_feasability) {
    resulting_maxviolation = 0;
    resulting_feasability = 0;

    auto vc_size = m_constraints.size();

    for (size_t ic = 0; ic < vc_size; ic++) {
        // the the residual of the constraint..
        double mres_i = m_constraints[ic]->Compute_c_i();

        double candidate_violation = fabs(m_constraints[ic]->Violation(mres_i));

        if (candidate_violation > resulting_maxviolation)
            resulting_maxviolation = candidate_violation;

        if (m_constraints[ic]->IsUnilateral()) {
            double candidate_feas = fabs(mres_i * m_constraints[ic]->Get_l_i());  // =|c*l|
            if (candidate_feas > resulting_feasability)
                resulting_feasability = candidate_feas;
        }
    }
}

unsigned int ChSystemDescriptor::CountActiveVariables() {
    if (freeze_count)  // optimization, avoid list count all times
        return n_q;

    auto vv_size = m_variables.size();

    n_q = 0;
    for (size_t iv = 0; iv < vv_size; iv++) {
        if (m_variables[iv]->IsActive()) {
            m_variables[iv]->SetOffset(n_q);  // also store offsets in state and MC matrix
            n_q += m_variables[iv]->Get_ndof();
        }
    }
    return n_q;
}

unsigned int ChSystemDescriptor::CountActiveConstraints() {
    if (freeze_count)  // optimization, avoid list count all times
        return n_c;

    auto vc_size = m_constraints.size();

    n_c = 0;
    for (size_t ic = 0; ic < vc_size; ic++) {
        if (m_constraints[ic]->IsActive()) {
            m_constraints[ic]->SetOffset(n_c);  // also store offsets in state and MC matrix
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

void ChSystemDescriptor::PasteMassKRMMatrixInto(ChSparseMatrix& Z, int row_offset, int col_offset) {
    bool overwrite = true;
    // Contribution of mass or rigid bodies and node-concentrated masses
    if (c_a != 0) {
        int s_q = 0;
        for (size_t iv = 0; iv < m_variables.size(); iv++) {
            if (m_variables[iv]->IsActive()) {
                m_variables[iv]->PasteMassInto(Z, s_q + row_offset, s_q + col_offset, c_a);
                s_q += m_variables[iv]->Get_ndof();
            }
        }
        overwrite = false;
    }

    // Contribution of stiffness, damping and mass matrices
    for (auto& KRMBlock : m_KRMblocks) {
        KRMBlock->PasteInto(Z, row_offset, col_offset, overwrite);
    }
}

unsigned int ChSystemDescriptor::PasteConstraintsJacobianMatrixInto(ChSparseMatrix& Z,
                                                                    int row_offset,
                                                                    int col_offset,
                                                                    bool only_bilateral) {
    unsigned int s_c = 0;
    for (size_t ic = 0; ic < m_constraints.size(); ic++) {
        if (m_constraints[ic]->IsActive() && !(only_bilateral && !m_constraints[ic]->GetMode() == CONSTRAINT_LOCK)) {
            m_constraints[ic]->PasteJacobianInto(Z, s_c + row_offset, col_offset);
            s_c++;
        }
    }

    return s_c;
}

unsigned int ChSystemDescriptor::PasteConstraintsJacobianMatrixTransposedInto(ChSparseMatrix& Z,
                                                                              int row_offset,
                                                                              int col_offset,
                                                                              bool only_bilateral) {
    unsigned int s_c = 0;
    for (auto& constr : m_constraints) {
        if (constr->IsActive() && !(only_bilateral && !constr->GetMode() == CONSTRAINT_LOCK)) {
            constr->PasteJacobianTransposedInto(Z, row_offset, s_c + col_offset);
            s_c++;
        }
    }

    return s_c;
}

void ChSystemDescriptor::PasteComplianceMatrixInto(ChSparseMatrix& Z,
                                                   int row_offset,
                                                   int col_offset,
                                                   bool only_bilateral) {
    int s_c = 0;
    for (auto& constr : m_constraints) {
        if (constr->IsActive() && !(only_bilateral && !constr->GetMode() == CONSTRAINT_LOCK)) {
            Z.SetElement(row_offset + s_c, col_offset + s_c, constr->Get_cfm_i());
            s_c++;
        }
    }
}

void ChSystemDescriptor::ConvertToMatrixForm(ChSparseMatrix* Z, ChVectorDynamic<>* rhs) {

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

int ChSystemDescriptor::BuildFbVector(ChVectorDynamic<>& Fvector, unsigned int row_offset) {
    n_q = CountActiveVariables();
    Fvector.setZero(n_q);

    auto vv_size = m_variables.size();

    // Fills the 'f' vector
    for (size_t iv = 0; iv < vv_size; iv++) {
        if (m_variables[iv]->IsActive()) {
            Fvector.segment(row_offset + m_variables[iv]->GetOffset(), m_variables[iv]->Get_ndof()) =
                m_variables[iv]->Get_fb();
        }
    }
    return n_q;
}

int ChSystemDescriptor::BuildBiVector(ChVectorDynamic<>& Bvector, unsigned int row_offset) {
    n_c = CountActiveConstraints();
    Bvector.setZero(n_c);

    auto vc_size = m_constraints.size();

    // Fill the 'b' vector
    for (size_t ic = 0; ic < vc_size; ic++) {
        if (m_constraints[ic]->IsActive()) {
            Bvector(row_offset + m_constraints[ic]->GetOffset()) = m_constraints[ic]->Get_b_i();
        }
    }

    return n_c;
}

int ChSystemDescriptor::BuildDiVector(ChVectorDynamic<>& Dvector) {
    n_q = CountActiveVariables();
    n_c = CountActiveConstraints();
    Dvector.setZero(n_q + n_c);

    auto vv_size = m_variables.size();
    auto vc_size = m_constraints.size();

    // Fills the 'f' vector part
    for (size_t iv = 0; iv < vv_size; iv++) {
        if (m_variables[iv]->IsActive()) {
            Dvector.segment(m_variables[iv]->GetOffset(), m_variables[iv]->Get_ndof()) = m_variables[iv]->Get_fb();
        }
    }

    // Fill the '-b' vector (with flipped sign!)
    for (size_t ic = 0; ic < vc_size; ic++) {
        if (m_constraints[ic]->IsActive()) {
            Dvector(m_constraints[ic]->GetOffset() + n_q) = -m_constraints[ic]->Get_b_i();
        }
    }

    return n_q + n_c;
}

int ChSystemDescriptor::BuildDiagonalVector(ChVectorDynamic<>& Diagonal_vect) {
    n_q = CountActiveVariables();
    n_c = CountActiveConstraints();
    Diagonal_vect.setZero(n_q + n_c);

    auto vk_size = m_KRMblocks.size();
    auto vv_size = m_variables.size();
    auto vc_size = m_constraints.size();

    // Fill the diagonal values given by ChKRMBlock objects , if any
    // (This cannot be easily parallelized because of possible write concurrency).
    for (size_t is = 0; is < vk_size; is++) {
        m_KRMblocks[is]->DiagonalAdd(Diagonal_vect);
    }

    // Get the 'M' diagonal terms given by ChVariables objects
    for (size_t iv = 0; iv < vv_size; iv++) {
        if (m_variables[iv]->IsActive()) {
            m_variables[iv]->DiagonalAdd(Diagonal_vect, c_a);
        }
    }

    // Get the 'E' diagonal terms (E_i = cfm_i )
    for (size_t ic = 0; ic < vc_size; ic++) {
        if (m_constraints[ic]->IsActive()) {
            Diagonal_vect(m_constraints[ic]->GetOffset() + n_q) = m_constraints[ic]->Get_cfm_i();
        }
    }

    return n_q + n_c;
}

int ChSystemDescriptor::FromVariablesToVector(ChVectorDynamic<>& mvector, bool resize_vector) {
    // Count active variables and resize vector if necessary
    if (resize_vector) {
        n_q = CountActiveVariables();
        mvector.setZero(n_q);
    }

    // Fill the vector
    auto vv_size = m_variables.size();
    for (size_t iv = 0; iv < vv_size; iv++) {
        if (m_variables[iv]->IsActive()) {
            mvector.segment(m_variables[iv]->GetOffset(), m_variables[iv]->Get_ndof()) = m_variables[iv]->Get_qb();
        }
    }

    return n_q;
}

int ChSystemDescriptor::FromVectorToVariables(const ChVectorDynamic<>& mvector) {
    assert(CountActiveVariables() == mvector.rows());

    // fetch from the vector
    auto vv_size = m_variables.size();
    for (size_t iv = 0; iv < vv_size; iv++) {
        if (m_variables[iv]->IsActive()) {
            m_variables[iv]->Get_qb() = mvector.segment(m_variables[iv]->GetOffset(), m_variables[iv]->Get_ndof());
        }
    }

    return n_q;
}

int ChSystemDescriptor::FromConstraintsToVector(ChVectorDynamic<>& mvector, bool resize_vector) {
    // Count active constraints and resize vector if necessary
    if (resize_vector) {
        n_c = CountActiveConstraints();
        mvector.setZero(n_c);
    }

    auto vc_size = (int)m_constraints.size();

    // Fill the vector
    for (size_t ic = 0; ic < vc_size; ic++) {
        if (m_constraints[ic]->IsActive()) {
            mvector(m_constraints[ic]->GetOffset()) = m_constraints[ic]->Get_l_i();
        }
    }

    return n_c;
}

int ChSystemDescriptor::FromVectorToConstraints(const ChVectorDynamic<>& mvector) {
    n_c = CountActiveConstraints();

    assert(n_c == mvector.size());

    auto vc_size = (int)m_constraints.size();

    // Fill the vector
    for (size_t ic = 0; ic < vc_size; ic++) {
        if (m_constraints[ic]->IsActive()) {
            m_constraints[ic]->Set_l_i(mvector(m_constraints[ic]->GetOffset()));
        }
    }

    return n_c;
}

int ChSystemDescriptor::FromUnknownsToVector(ChVectorDynamic<>& mvector, bool resize_vector) {
    // Count active variables & constraints and resize vector if necessary
    n_q = CountActiveVariables();
    n_c = CountActiveConstraints();

    if (resize_vector) {
        mvector.setZero(n_q + n_c);
    }

    auto vv_size = (int)m_variables.size();
    auto vc_size = (int)m_constraints.size();

    // Fill the first part of vector, x.q ,with variables q
    for (size_t iv = 0; iv < vv_size; iv++) {
        if (m_variables[iv]->IsActive()) {
            mvector.segment(m_variables[iv]->GetOffset(), m_variables[iv]->Get_ndof()) = m_variables[iv]->Get_qb();
        }
    }

    // Fill the second part of vector, x.l, with constraint multipliers -l (with flipped sign!)
    for (size_t ic = 0; ic < vc_size; ic++) {
        if (m_constraints[ic]->IsActive()) {
            mvector(m_constraints[ic]->GetOffset() + n_q) = -m_constraints[ic]->Get_l_i();
        }
    }

    return n_q + n_c;
}

int ChSystemDescriptor::FromVectorToUnknowns(const ChVectorDynamic<>& mvector) {
    n_q = CountActiveVariables();
    n_c = CountActiveConstraints();

    assert((n_q + n_c) == mvector.size());

    auto vv_size = m_variables.size();
    auto vc_size = m_constraints.size();

    // Fetch from the first part of vector (x.q = q)
    for (size_t iv = 0; iv < vv_size; iv++) {
        if (m_variables[iv]->IsActive()) {
            m_variables[iv]->Get_qb() = mvector.segment(m_variables[iv]->GetOffset(), m_variables[iv]->Get_ndof());
        }
    }

    // Fetch from the second part of vector (x.l = -l), with flipped sign!
    for (size_t ic = 0; ic < vc_size; ic++) {
        if (m_constraints[ic]->IsActive()) {
            m_constraints[ic]->Set_l_i(-mvector(m_constraints[ic]->GetOffset() + n_q));
        }
    }

    return n_q + n_c;
}

void ChSystemDescriptor::SchurComplementProduct(ChVectorDynamic<>& result,
                                                const ChVectorDynamic<>& lvector,
                                                std::vector<bool>* enabled) {
    // currently, the case with ChKRMBlock items is not supported (only diagonal M is supported, no K)
    assert(m_KRMblocks.size() == 0);
    assert(lvector.size() == CountActiveConstraints());

    result.setZero(n_c);

    // Performs the sparse product    result = [N]*l = [ [Cq][M^(-1)][Cq'] - [E] ] *l
    // in different phases:

    auto vv_size = m_variables.size();
    auto vc_size = m_constraints.size();

    // 1 - set the qb vector (aka speeds, in each ChVariable sparse data) as zero

    for (size_t iv = 0; iv < vv_size; iv++) {
        if (m_variables[iv]->IsActive())
            m_variables[iv]->Get_qb().setZero();
    }

    // 2 - performs    qb=[M^(-1)][Cq']*l  by
    //     iterating over all constraints (when implemented in parallel this
    //     could be non-trivial because race conditions might occur -> reduction buffer etc.)
    //     Also, begin to add the cfm term ( -[E]*l ) to the result.

    // ATTENTION:  this loop cannot be parallelized! Concurrent write to some q may happen
    for (size_t ic = 0; ic < vc_size; ic++) {
        if (m_constraints[ic]->IsActive()) {
            int s_c = m_constraints[ic]->GetOffset();

            bool process = (!enabled) || (*enabled)[s_c];

            if (process) {
                double li = lvector(s_c);

                // Compute qb += [M^(-1)][Cq']*l_i
                //  NOTE! concurrent update to same q data, risk of collision if parallel.
                m_constraints[ic]->Increment_q(li);  // computationally intensive

                // Add constraint force mixing term  result = cfm * l_i = [E]*l_i
                result(s_c) = m_constraints[ic]->Get_cfm_i() * li;
            }
        }
    }

    // 3 - performs    result=[Cq']*qb    by
    //     iterating over all constraints

    for (size_t ic = 0; ic < vc_size; ic++) {
        if (m_constraints[ic]->IsActive()) {
            bool process = (!enabled) || (*enabled)[m_constraints[ic]->GetOffset()];

            if (process)
                result(m_constraints[ic]->GetOffset()) +=
                    m_constraints[ic]->Compute_Cq_q();  // computationally intensive
            else
                result(m_constraints[ic]->GetOffset()) = 0;  // not enabled constraints, just set to 0 result
        }
    }
}

void ChSystemDescriptor::SystemProduct(ChVectorDynamic<>& result, const ChVectorDynamic<>& x) {
    n_q = CountActiveVariables();
    n_c = CountActiveConstraints();

    result.setZero(n_q + n_c);

    auto vv_size = m_variables.size();
    auto vc_size = m_constraints.size();
    auto vk_size = m_KRMblocks.size();

    // 1) First row: result.q part =  [M + K]*x.q + [Cq']*x.l

    // 1.1)  do  M*x.q
    for (size_t iv = 0; iv < vv_size; iv++) {
        if (m_variables[iv]->IsActive()) {
            m_variables[iv]->MultiplyAndAdd(result, x, c_a);
        }
    }

    // 1.2)  add also K*x.q  (NON straight parallelizable - risk of concurrency in writing)
    for (size_t ik = 0; ik < vk_size; ik++) {
        m_KRMblocks[ik]->MultiplyAndAdd(result, x);
    }

    // 1.3)  add also [Cq]'*x.l  (NON straight parallelizable - risk of concurrency in writing)
    for (size_t ic = 0; ic < vc_size; ic++) {
        if (m_constraints[ic]->IsActive()) {
            m_constraints[ic]->MultiplyTandAdd(result, x(m_constraints[ic]->GetOffset() + n_q));
        }
    }

    // 2) Second row: result.l part =  [C_q]*x.q + [E]*x.l
    for (size_t ic = 0; ic < vc_size; ic++) {
        if (m_constraints[ic]->IsActive()) {
            int s_c = m_constraints[ic]->GetOffset() + n_q;
            m_constraints[ic]->MultiplyAndAdd(result(s_c), x);       // result.l_i += [C_q_i]*x.q
            result(s_c) += m_constraints[ic]->Get_cfm_i() * x(s_c);  // result.l_i += [E]*x.l_i
        }
    }
}

void ChSystemDescriptor::ConstraintsProject(ChVectorDynamic<>& multipliers) {
    FromVectorToConstraints(multipliers);

    auto vc_size = m_constraints.size();

    for (size_t ic = 0; ic < vc_size; ic++) {
        if (m_constraints[ic]->IsActive())
            m_constraints[ic]->Project();
    }

    FromConstraintsToVector(multipliers, false);
}

void ChSystemDescriptor::UnknownsProject(ChVectorDynamic<>& mx) {
    n_q = CountActiveVariables();

    auto vc_size = m_constraints.size();

    // vector -> constraints
    // Fetch from the second part of vector (x.l = -l), with flipped sign!
    for (size_t ic = 0; ic < vc_size; ic++) {
        if (m_constraints[ic]->IsActive()) {
            m_constraints[ic]->Set_l_i(-mx(m_constraints[ic]->GetOffset() + n_q));
        }
    }

    // constraint projection!
    for (size_t ic = 0; ic < vc_size; ic++) {
        if (m_constraints[ic]->IsActive())
            m_constraints[ic]->Project();
    }

    // constraints -> vector
    // Fill the second part of vector, x.l, with constraint multipliers -l (with flipped sign!)
    for (size_t ic = 0; ic < vc_size; ic++) {
        if (m_constraints[ic]->IsActive()) {
            mx(m_constraints[ic]->GetOffset() + n_q) = -m_constraints[ic]->Get_l_i();
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

    std::ofstream file_M(path + "/" + prefix + "_M.dat");
    file_M << std::setprecision(12) << std::scientific;
    StreamOut(mass_matrix, file_M, one_indexed);

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
    ConvertToMatrixForm(&Z, &rhs);

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
    ConvertToMatrixForm(nullptr, &rhs);
    std::ofstream file_rhs(path + "/" + prefix + "_rhs.dat");
    file_rhs << std::setprecision(12) << std::scientific;
    StreamOut(rhs, file_rhs);
}

}  // end namespace chrono
