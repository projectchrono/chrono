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
    vconstraints.clear();
    vvariables.clear();
    vstiffness.clear();
}

ChSystemDescriptor::~ChSystemDescriptor() {
    vconstraints.clear();
    vvariables.clear();
    vstiffness.clear();
}

void ChSystemDescriptor::ComputeFeasabilityViolation(double& resulting_maxviolation, double& resulting_feasability) {
    resulting_maxviolation = 0;
    resulting_feasability = 0;

    auto vc_size = vconstraints.size();

    for (size_t ic = 0; ic < vc_size; ic++) {
        // the the residual of the constraint..
        double mres_i = vconstraints[ic]->Compute_c_i();

        double candidate_violation = fabs(vconstraints[ic]->Violation(mres_i));

        if (candidate_violation > resulting_maxviolation)
            resulting_maxviolation = candidate_violation;

        if (vconstraints[ic]->IsUnilateral()) {
            double candidate_feas = fabs(mres_i * vconstraints[ic]->Get_l_i());  // =|c*l|
            if (candidate_feas > resulting_feasability)
                resulting_feasability = candidate_feas;
        }
    }
}

int ChSystemDescriptor::CountActiveVariables() {
    if (freeze_count)  // optimization, avoid list count all times
        return n_q;
    
    auto vv_size = vvariables.size();

    n_q = 0;
    for (size_t iv = 0; iv < vv_size; iv++) {
        if (vvariables[iv]->IsActive()) {
            vvariables[iv]->SetOffset(n_q);  // also store offsets in state and MC matrix
            n_q += vvariables[iv]->Get_ndof();
        }
    }
    return n_q;
}

int ChSystemDescriptor::CountActiveConstraints() {
    if (freeze_count)  // optimization, avoid list count all times
        return n_c;

    auto vc_size = vconstraints.size();

    n_c = 0;
    for (size_t ic = 0; ic < vc_size; ic++) {
        if (vconstraints[ic]->IsActive()) {
            vconstraints[ic]->SetOffset(n_c);  // also store offsets in state and MC matrix
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

void ChSystemDescriptor::ConvertToMatrixForm(ChSparseMatrix* Cq,
                                             ChSparseMatrix* H,
                                             ChSparseMatrix* E,
                                             ChVectorDynamic<>* Fvector,
                                             ChVectorDynamic<>* Bvector,
                                             ChVectorDynamic<>* Frict,
                                             bool only_bilaterals,
                                             bool skip_contacts_uv) {
    std::vector<ChConstraint*>& mconstraints = GetConstraintsList();
    std::vector<ChVariables*>& mvariables = GetVariablesList();

    auto mv_size = mvariables.size();
    auto mc_size = mconstraints.size();
    auto vs_size = vstiffness.size();

    // Count bilateral and other constraints.. (if wanted, bilaterals only)

    int mn_c = 0;
    for (size_t ic = 0; ic < mc_size; ic++) {
        if (mconstraints[ic]->IsActive())
            if (!((mconstraints[ic]->GetMode() == CONSTRAINT_FRIC) && only_bilaterals))
                if (!((dynamic_cast<ChConstraintTwoTuplesFrictionTall*>(mconstraints[ic])) && skip_contacts_uv)) {
                    mn_c++;
                }
    }

    // Count active variables, by scanning through all variable blocks,
    // and set offsets

    n_q = CountActiveVariables();

    // Reset and resize (if needed) auxiliary vectors

    if (Cq)
        Cq->resize(mn_c, n_q);
    if (H)
        H->resize(n_q, n_q);
    if (E)
        E->resize(mn_c, mn_c);
    if (Fvector)
        Fvector->setZero(n_q);
    if (Bvector)
        Bvector->setZero(mn_c);
    if (Frict)
        Frict->setZero(mn_c);

    // Fills H submasses and 'f' vector,
    // by looping on variables
    int s_q = 0;
    for (size_t iv = 0; iv < mv_size; iv++) {
        if (mvariables[iv]->IsActive()) {
            if (H)
                mvariables[iv]->Build_M(*H, s_q, s_q, c_a);  // .. fills  H  (often H=M , the mass)
            if (Fvector)
                Fvector->segment(s_q, vvariables[iv]->Get_ndof()) = vvariables[iv]->Get_fb();  // .. fills  'f'
            s_q += mvariables[iv]->Get_ndof();
        }
    }

    // If some stiffness / hessian matrix has been added to H ,
    // also add it to the sparse H
    if (H) {
        for (size_t ik = 0; ik < vs_size; ik++) {
            vstiffness[ik]->Build_K(*H, true);
        }
    }

    // Fills Cq jacobian, E 'compliance' matrix , the 'b' vector and friction coeff.vector,
    // by looping on constraints
    int s_c = 0;
    for (size_t ic = 0; ic < mc_size; ic++) {
        if (mconstraints[ic]->IsActive())
            if (!((mconstraints[ic]->GetMode() == CONSTRAINT_FRIC) && only_bilaterals))
                if (!((dynamic_cast<ChConstraintTwoTuplesFrictionTall*>(mconstraints[ic])) && skip_contacts_uv)) {
                    if (Cq)
                        mconstraints[ic]->Build_Cq(*Cq, s_c);  // .. fills Cq
                    if (E)
                        E->SetElement(s_c, s_c, mconstraints[ic]->Get_cfm_i());  // .. fills E ( = cfm )
                    if (Bvector)
                        (*Bvector)(s_c) = mconstraints[ic]->Get_b_i();  // .. fills 'b'
                    if (Frict)                                          // .. fills vector of friction coefficients
                    {
                        (*Frict)(s_c) = -2;  // mark with -2 flag for bilaterals (default)
                        if (auto mcon = dynamic_cast<ChConstraintTwoTuplesContactNall*>(mconstraints[ic]))
                            (*Frict)(s_c) =
                                mcon->GetFrictionCoefficient();  // friction coeff only in row of normal component
                        if (dynamic_cast<ChConstraintTwoTuplesFrictionTall*>(mconstraints[ic]))
                            (*Frict)(s_c) = -1;  // mark with -1 flag for rows of tangential components
                    }
                    s_c++;
                }
    }
}

void ChSystemDescriptor::ConvertToMatrixForm(ChSparseMatrix* Z, ChVectorDynamic<>* rhs) {
    std::vector<ChConstraint*>& mconstraints = GetConstraintsList();
    std::vector<ChVariables*>& mvariables = GetVariablesList();

    auto mv_size = mvariables.size();
    auto mc_size = mconstraints.size();
    auto vs_size = vstiffness.size();

    // Count constraints.
    int mn_c = 0;
    for (size_t ic = 0; ic < mc_size; ic++) {
        if (mconstraints[ic]->IsActive())
            mn_c++;
    }

    // Count active variables, by scanning through all variable blocks, and set offsets.
    n_q = CountActiveVariables();

    if (Z) {
        Z->conservativeResize(n_q + mn_c, n_q + mn_c);
        Z->setZeroValues();        

        // Fill Z with masses and inertias.
        int s_q = 0;
        for (size_t iv = 0; iv < mv_size; iv++) {
            if (mvariables[iv]->IsActive()) {
                // Masses and inertias in upper-left block of Z
                mvariables[iv]->Build_M(*Z, s_q, s_q, c_a);
                s_q += mvariables[iv]->Get_ndof();
            }
        }

        // If present, add stiffness matrix K to upper-left block of Z.
        for (size_t ik = 0; ik < vs_size; ik++) {
            vstiffness[ik]->Build_K(*Z, true);
        }

        // Fill Z by looping over constraints.
        int s_c = 0;
        for (size_t ic = 0; ic < mc_size; ic++) {
            if (mconstraints[ic]->IsActive()) {
                // Constraint Jacobian in lower-left block of Z
                mconstraints[ic]->Build_Cq(*Z, n_q + s_c);
                // Transposed constraint Jacobian in upper-right block of Z
                mconstraints[ic]->Build_CqT(*Z, n_q + s_c);
                // E ( = cfm ) in lower-right block of Z
                Z->SetElement(n_q + s_c, n_q + s_c, mconstraints[ic]->Get_cfm_i());
                s_c++;
            }
        }
    }

    if (rhs) {
        rhs->setZero(n_q + mn_c, 1);

        // Fill rhs with forces.
        int s_q = 0;
        for (size_t iv = 0; iv < mv_size; iv++) {
            if (mvariables[iv]->IsActive()) {
                // Forces in upper section of rhs
                rhs->segment(s_q, vvariables[iv]->Get_ndof()) = vvariables[iv]->Get_fb();
                s_q += mvariables[iv]->Get_ndof();
            }
        }

        // Fill rhs by looping over constraints.
        int s_c = 0;
        for (size_t ic = 0; ic < mc_size; ic++) {
            if (mconstraints[ic]->IsActive()) {
                // -b term in lower section of rhs
                (*rhs)(n_q + s_c) = -(mconstraints[ic]->Get_b_i());
                s_c++;
            }
        }
    }
}

int ChSystemDescriptor::BuildFbVector(ChVectorDynamic<>& Fvector) {
    n_q = CountActiveVariables();
    Fvector.setZero(n_q);

    auto vv_size = vvariables.size();

    // Fills the 'f' vector
    for (size_t iv = 0; iv < vv_size; iv++) {
        if (vvariables[iv]->IsActive()) {
            Fvector.segment(vvariables[iv]->GetOffset(), vvariables[iv]->Get_ndof()) = vvariables[iv]->Get_fb();
        }
    }
    return n_q;
}

int ChSystemDescriptor::BuildBiVector(ChVectorDynamic<>& Bvector) {
    n_c = CountActiveConstraints();
    Bvector.setZero(n_c);

    auto vc_size = vconstraints.size();

    // Fill the 'b' vector
    for (size_t ic = 0; ic < vc_size; ic++) {
        if (vconstraints[ic]->IsActive()) {
            Bvector(vconstraints[ic]->GetOffset()) = vconstraints[ic]->Get_b_i();
        }
    }

    return n_c;
}

int ChSystemDescriptor::BuildDiVector(ChVectorDynamic<>& Dvector) {
    n_q = CountActiveVariables();
    n_c = CountActiveConstraints();
    Dvector.setZero(n_q + n_c);

    auto vv_size = vvariables.size();
    auto vc_size = vconstraints.size();

    // Fills the 'f' vector part
    for (size_t iv = 0; iv < vv_size; iv++) {
        if (vvariables[iv]->IsActive()) {
            Dvector.segment(vvariables[iv]->GetOffset(), vvariables[iv]->Get_ndof()) = vvariables[iv]->Get_fb();
        }
    }

    // Fill the '-b' vector (with flipped sign!)
    for (size_t ic = 0; ic < vc_size; ic++) {
        if (vconstraints[ic]->IsActive()) {
            Dvector(vconstraints[ic]->GetOffset() + n_q) = -vconstraints[ic]->Get_b_i();
        }
    }

    return n_q + n_c;
}

int ChSystemDescriptor::BuildDiagonalVector(ChVectorDynamic<>& Diagonal_vect) {
    n_q = CountActiveVariables();
    n_c = CountActiveConstraints();
    Diagonal_vect.setZero(n_q + n_c);

    auto vs_size = vstiffness.size();
    auto vv_size = vvariables.size();
    auto vc_size = vconstraints.size();

    // Fill the diagonal values given by ChKblock objects , if any
    // (This cannot be easily parallelized because of possible write concurrency).
    for (size_t is = 0; is < vs_size; is++) {
        vstiffness[is]->DiagonalAdd(Diagonal_vect);
    }

    // Get the 'M' diagonal terms given by ChVariables objects
    for (size_t iv = 0; iv < vv_size; iv++) {
        if (vvariables[iv]->IsActive()) {
            vvariables[iv]->DiagonalAdd(Diagonal_vect, c_a);
        }
    }

    // Get the 'E' diagonal terms (E_i = cfm_i )
    for (size_t ic = 0; ic < vc_size; ic++) {
        if (vconstraints[ic]->IsActive()) {
            Diagonal_vect(vconstraints[ic]->GetOffset() + n_q) = vconstraints[ic]->Get_cfm_i();
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
    auto vv_size = vvariables.size();
    for (size_t iv = 0; iv < vv_size; iv++) {
        if (vvariables[iv]->IsActive()) {
            mvector.segment(vvariables[iv]->GetOffset(), vvariables[iv]->Get_ndof()) = vvariables[iv]->Get_qb();
        }
    }

    return n_q;
}

int ChSystemDescriptor::FromVectorToVariables(const ChVectorDynamic<>& mvector) {
    assert(CountActiveVariables() == mvector.rows());

    // fetch from the vector
    auto vv_size = vvariables.size();
    for (size_t iv = 0; iv < vv_size; iv++) {
        if (vvariables[iv]->IsActive()) {
            vvariables[iv]->Get_qb() = mvector.segment(vvariables[iv]->GetOffset(), vvariables[iv]->Get_ndof());
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

    auto vc_size = (int)vconstraints.size();

    // Fill the vector
    for (size_t ic = 0; ic < vc_size; ic++) {
        if (vconstraints[ic]->IsActive()) {
            mvector(vconstraints[ic]->GetOffset()) = vconstraints[ic]->Get_l_i();
        }
    }

    return n_c;
}

int ChSystemDescriptor::FromVectorToConstraints(const ChVectorDynamic<>& mvector) {
    n_c = CountActiveConstraints();

    assert(n_c == mvector.size());

    auto vc_size = (int)vconstraints.size();

    // Fill the vector
    for (size_t ic = 0; ic < vc_size; ic++) {
        if (vconstraints[ic]->IsActive()) {
            vconstraints[ic]->Set_l_i(mvector(vconstraints[ic]->GetOffset()));
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

    auto vv_size = (int)vvariables.size();
    auto vc_size = (int)vconstraints.size();

    // Fill the first part of vector, x.q ,with variables q
    for (size_t iv = 0; iv < vv_size; iv++) {
        if (vvariables[iv]->IsActive()) {
            mvector.segment(vvariables[iv]->GetOffset(), vvariables[iv]->Get_ndof()) = vvariables[iv]->Get_qb();
        }
    }

    // Fill the second part of vector, x.l, with constraint multipliers -l (with flipped sign!)
    for (size_t ic = 0; ic < vc_size; ic++) {
        if (vconstraints[ic]->IsActive()) {
            mvector(vconstraints[ic]->GetOffset() + n_q) = -vconstraints[ic]->Get_l_i();
        }
    }

    return n_q + n_c;
}

int ChSystemDescriptor::FromVectorToUnknowns(const ChVectorDynamic<>& mvector) {
    n_q = CountActiveVariables();
    n_c = CountActiveConstraints();

    assert((n_q + n_c) == mvector.size());

    auto vv_size = vvariables.size();
    auto vc_size = vconstraints.size();

    // Fetch from the first part of vector (x.q = q)
    for (size_t iv = 0; iv < vv_size; iv++) {
        if (vvariables[iv]->IsActive()) {
            vvariables[iv]->Get_qb() = mvector.segment(vvariables[iv]->GetOffset(), vvariables[iv]->Get_ndof());
        }
    }

    // Fetch from the second part of vector (x.l = -l), with flipped sign!
    for (size_t ic = 0; ic < vc_size; ic++) {
        if (vconstraints[ic]->IsActive()) {
            vconstraints[ic]->Set_l_i(-mvector(vconstraints[ic]->GetOffset() + n_q));
        }
    }

    return n_q + n_c;
}

void ChSystemDescriptor::ShurComplementProduct(ChVectorDynamic<>& result,
                                               const ChVectorDynamic<>& lvector,
                                               std::vector<bool>* enabled) {
    // currently, the case with ChKblock items is not supported (only diagonal M is supported, no K)
    assert(vstiffness.size() == 0);
    assert(lvector.size() == CountActiveConstraints());

    result.setZero(n_c);

    // Performs the sparse product    result = [N]*l = [ [Cq][M^(-1)][Cq'] - [E] ] *l
    // in different phases:

    auto vv_size = vvariables.size();
    auto vc_size = vconstraints.size();

    // 1 - set the qb vector (aka speeds, in each ChVariable sparse data) as zero

    for (size_t iv = 0; iv < vv_size; iv++) {
        if (vvariables[iv]->IsActive())
            vvariables[iv]->Get_qb().setZero();
    }

    // 2 - performs    qb=[M^(-1)][Cq']*l  by
    //     iterating over all constraints (when implemented in parallel this
    //     could be non-trivial because race conditions might occur -> reduction buffer etc.)
    //     Also, begin to add the cfm term ( -[E]*l ) to the result.

    // ATTENTION:  this loop cannot be parallelized! Concurrent write to some q may happen
    for (size_t ic = 0; ic < vc_size; ic++) {
        if (vconstraints[ic]->IsActive()) {
            int s_c = vconstraints[ic]->GetOffset();

            bool process = (!enabled) || (*enabled)[s_c];

            if (process) {
                double li = lvector(s_c);

                // Compute qb += [M^(-1)][Cq']*l_i
                //  NOTE! concurrent update to same q data, risk of collision if parallel.
                vconstraints[ic]->Increment_q(li);  // computationally intensive

                // Add constraint force mixing term  result = cfm * l_i = [E]*l_i
                result(s_c) = vconstraints[ic]->Get_cfm_i() * li;
            }
        }
    }

    // 3 - performs    result=[Cq']*qb    by
    //     iterating over all constraints

    for (size_t ic = 0; ic < vc_size; ic++) {
        if (vconstraints[ic]->IsActive()) {
            bool process = (!enabled) || (*enabled)[vconstraints[ic]->GetOffset()];

            if (process)
                result(vconstraints[ic]->GetOffset()) += vconstraints[ic]->Compute_Cq_q();  // computationally intensive
            else
                result(vconstraints[ic]->GetOffset()) = 0;  // not enabled constraints, just set to 0 result
        }
    }
}

void ChSystemDescriptor::SystemProduct(ChVectorDynamic<>& result, const ChVectorDynamic<>& x) {
    n_q = CountActiveVariables();
    n_c = CountActiveConstraints();

    result.setZero(n_q + n_c);

    auto vv_size = vvariables.size();
    auto vc_size = vconstraints.size();
    auto vs_size = vstiffness.size();

    // 1) First row: result.q part =  [M + K]*x.q + [Cq']*x.l

    // 1.1)  do  M*x.q
    for (size_t iv = 0; iv < vv_size; iv++) {
        if (vvariables[iv]->IsActive()) {
            vvariables[iv]->MultiplyAndAdd(result, x, c_a);
        }
    }

    // 1.2)  add also K*x.q  (NON straight parallelizable - risk of concurrency in writing)
    for (size_t ik = 0; ik < vs_size; ik++) {
        vstiffness[ik]->MultiplyAndAdd(result, x);
    }

    // 1.3)  add also [Cq]'*x.l  (NON straight parallelizable - risk of concurrency in writing)
    for (size_t ic = 0; ic < vc_size; ic++) {
        if (vconstraints[ic]->IsActive()) {
            vconstraints[ic]->MultiplyTandAdd(result, x(vconstraints[ic]->GetOffset() + n_q));
        }
    }

    // 2) Second row: result.l part =  [C_q]*x.q + [E]*x.l
    for (size_t ic = 0; ic < vc_size; ic++) {
        if (vconstraints[ic]->IsActive()) {
            int s_c = vconstraints[ic]->GetOffset() + n_q;
            vconstraints[ic]->MultiplyAndAdd(result(s_c), x);       // result.l_i += [C_q_i]*x.q
            result(s_c) += vconstraints[ic]->Get_cfm_i() * x(s_c);  // result.l_i += [E]*x.l_i
        }
    }
}

void ChSystemDescriptor::ConstraintsProject(ChVectorDynamic<>& multipliers) {
    FromVectorToConstraints(multipliers);

    auto vc_size = vconstraints.size();

    for (size_t ic = 0; ic < vc_size; ic++) {
        if (vconstraints[ic]->IsActive())
            vconstraints[ic]->Project();
    }

    FromConstraintsToVector(multipliers, false);
}

void ChSystemDescriptor::UnknownsProject(ChVectorDynamic<>& mx) {
    n_q = CountActiveVariables();

    auto vc_size = vconstraints.size();

    // vector -> constraints
    // Fetch from the second part of vector (x.l = -l), with flipped sign!
    for (size_t ic = 0; ic < vc_size; ic++) {
        if (vconstraints[ic]->IsActive()) {
            vconstraints[ic]->Set_l_i(-mx(vconstraints[ic]->GetOffset() + n_q));
        }
    }

    // constraint projection!
    for (size_t ic = 0; ic < vc_size; ic++) {
        if (vconstraints[ic]->IsActive())
            vconstraints[ic]->Project();
    }

    // constraints -> vector
    // Fill the second part of vector, x.l, with constraint multipliers -l (with flipped sign!)
    for (size_t ic = 0; ic < vc_size; ic++) {
        if (vconstraints[ic]->IsActive()) {
            mx(vconstraints[ic]->GetOffset() + n_q) = -vconstraints[ic]->Get_l_i();
        }
    }
}

// -----------------------------------------------------------------------------

void ChSystemDescriptor::WriteMatrix(const std::string& path, const std::string& prefix) {
    const char* numformat = "%.12g";
    std::string filename;

    ChSparseMatrix Z;
    ChVectorDynamic<double> rhs;
    ConvertToMatrixForm(&Z, &rhs);

    filename = path + "/" + prefix + "_Z.dat";
    ChStreamOutAsciiFile file_Z(filename.c_str());
    file_Z.SetNumFormat(numformat);
    StreamOutSparseMatlabFormat(Z, file_Z);

    filename = path + "/" + prefix + "_rhs.dat";
    ChStreamOutAsciiFile file_rhs(filename.c_str());
    file_rhs.SetNumFormat(numformat);
    StreamOutDenseMatlabFormat(rhs, file_rhs);
}

void ChSystemDescriptor::WriteMatrixBlocks(const std::string& path, const std::string& prefix) {
    const char* numformat = "%.12g";
    std::string filename;

    ChSparseMatrix mdM;
    ChSparseMatrix mdCq;
    ChSparseMatrix mdE;
    ChVectorDynamic<double> mdf;
    ChVectorDynamic<double> mdb;
    ChVectorDynamic<double> mdfric;
    ConvertToMatrixForm(&mdCq, &mdM, &mdE, &mdf, &mdb, &mdfric);

    filename = path + "/" + prefix + "_M.dat";
    ChStreamOutAsciiFile file_M(filename.c_str());
    file_M.SetNumFormat(numformat);
    StreamOutSparseMatlabFormat(mdM, file_M);

    filename = path + "/" + prefix + "_Cq.dat";
    ChStreamOutAsciiFile file_Cq(filename.c_str());
    file_Cq.SetNumFormat(numformat);
    StreamOutSparseMatlabFormat(mdCq, file_Cq);

    filename = path + "/" + prefix + "_E.dat";
    ChStreamOutAsciiFile file_E(filename.c_str());
    file_E.SetNumFormat(numformat);
    StreamOutSparseMatlabFormat(mdE, file_E);

    filename = path + "/" + prefix + "_f.dat";
    ChStreamOutAsciiFile file_f(filename.c_str());
    file_f.SetNumFormat(numformat);
    StreamOutDenseMatlabFormat(mdf, file_f);

    filename = path + "/" + prefix + "_b.dat";
    ChStreamOutAsciiFile file_b(filename.c_str());
    file_b.SetNumFormat(numformat);
    StreamOutDenseMatlabFormat(mdb, file_b);

    filename = path + "/" + prefix + "_fric.dat";
    ChStreamOutAsciiFile file_fric(filename.c_str());
    file_fric.SetNumFormat(numformat);
    StreamOutDenseMatlabFormat(mdfric, file_fric);
}

void ChSystemDescriptor::WriteMatrixSpmv(const std::string& path, const std::string& prefix) {
    const char* numformat = "%.12g";
    std::string filename;

    // Count constraints.
    int mn_c = 0;
    for (auto& cnstr : vconstraints) {
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
    filename = path + "/" + prefix + "_Z.dat";
    ChStreamOutAsciiFile file_Z(filename.c_str());
    file_Z.SetNumFormat(numformat);
    StreamOutSparseMatlabFormat(Z, file_Z);

    // Write RHS to file
    ChVectorDynamic<double> rhs;
    ConvertToMatrixForm(nullptr, &rhs);
    filename = path + "/" + prefix + "_rhs.dat";
    ChStreamOutAsciiFile file_rhs(filename.c_str());
    file_rhs.SetNumFormat(numformat);
    StreamOutDenseMatlabFormat(rhs, file_rhs);
}

}  // end namespace chrono
