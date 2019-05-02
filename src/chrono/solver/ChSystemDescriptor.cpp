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

#include "chrono/solver/ChSystemDescriptor.h"
#include "chrono/solver/ChConstraintTwoTuplesContactN.h"
#include "chrono/solver/ChConstraintTwoTuplesFrictionT.h"
#include "chrono/core/ChLinkedListMatrix.h"

namespace chrono {

// Register into the object factory, to enable run-time
// dynamic creation and persistence
CH_FACTORY_REGISTER(ChSystemDescriptor)

#define CH_SPINLOCK_HASHSIZE 203

ChSystemDescriptor::ChSystemDescriptor() {
    vconstraints.clear();
    vvariables.clear();
    vstiffness.clear();

    c_a = 1.0;

    n_q = 0;
    n_c = 0;
    freeze_count = false;

    this->num_threads = CHOMPfunctions::GetNumProcs();

    spinlocktable = new ChSpinlock[CH_SPINLOCK_HASHSIZE];
}

ChSystemDescriptor::~ChSystemDescriptor() {
    vconstraints.clear();
    vvariables.clear();
    vstiffness.clear();

    if (spinlocktable)
        delete[] spinlocktable;
    spinlocktable = 0;
}

void ChSystemDescriptor::ComputeFeasabilityViolation(
    double& resulting_maxviolation,  ///< gets the max constraint violation (either bi- and unilateral.)
    double& resulting_feasability    ///< gets the max feasability as max |l*c|, for unilateral only
    ) {
    resulting_maxviolation = 0;
    resulting_feasability = 0;

    for (unsigned int ic = 0; ic < vconstraints.size(); ic++) {
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
    if (this->freeze_count)  // optimization, avoid list count all times
        return n_q;

    n_q = 0;
    for (unsigned int iv = 0; iv < vvariables.size(); iv++) {
        if (vvariables[iv]->IsActive()) {
            vvariables[iv]->SetOffset(n_q);  // also store offsets in state and MC matrix
            n_q += vvariables[iv]->Get_ndof();
        }
    }
    return n_q;
}

int ChSystemDescriptor::CountActiveConstraints() {
    if (this->freeze_count)  // optimization, avoid list count all times
        return n_c;

    n_c = 0;
    for (unsigned int ic = 0; ic < vconstraints.size(); ic++) {
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
                                             ChMatrix<>* Fvector,
                                             ChMatrix<>* Bvector,
                                             ChMatrix<>* Frict,
                                             bool only_bilaterals,
                                             bool skip_contacts_uv) {
    std::vector<ChConstraint*>& mconstraints = this->GetConstraintsList();
    std::vector<ChVariables*>& mvariables = this->GetVariablesList();

    // Count bilateral and other constraints.. (if wanted, bilaterals only)

    int mn_c = 0;
    for (unsigned int ic = 0; ic < mconstraints.size(); ic++) {
        if (mconstraints[ic]->IsActive())
            if (!((mconstraints[ic]->GetMode() == CONSTRAINT_FRIC) && only_bilaterals))
                if (!((dynamic_cast<ChConstraintTwoTuplesFrictionTall*>(mconstraints[ic])) && skip_contacts_uv)) {
                    mn_c++;
                }
    }

    // Count active variables, by scanning through all variable blocks,
    // and set offsets

    n_q = this->CountActiveVariables();

    // Reset and resize (if needed) auxiliary vectors

    if (Cq)
        Cq->Reset(mn_c, n_q);
    if (H)
        H->Reset(n_q, n_q);
    if (E)
        E->Reset(mn_c, mn_c);
    if (Fvector)
        Fvector->Reset(n_q, 1);
    if (Bvector)
        Bvector->Reset(mn_c, 1);
    if (Frict)
        Frict->Reset(mn_c, 1);

    // Fills H submasses and 'f' vector,
    // by looping on variables
    int s_q = 0;
    for (unsigned int iv = 0; iv < mvariables.size(); iv++) {
        if (mvariables[iv]->IsActive()) {
            if (H)
                mvariables[iv]->Build_M(*H, s_q, s_q, this->c_a);  // .. fills  H  (often H=M , the mass)
            if (Fvector)
                Fvector->PasteMatrix(vvariables[iv]->Get_fb(), s_q, 0);  // .. fills  'f'
            s_q += mvariables[iv]->Get_ndof();
        }
    }

    // If some stiffness / hessian matrix has been added to H ,
    // also add it to the sparse H
    int s_k = 0;
    if (H) {
        for (unsigned int ik = 0; ik < this->vstiffness.size(); ik++) {
            this->vstiffness[ik]->Build_K(*H, true);
        }
    }

    // Fills Cq jacobian, E 'compliance' matrix , the 'b' vector and friction coeff.vector,
    // by looping on constraints
    int s_c = 0;
    for (unsigned int ic = 0; ic < mconstraints.size(); ic++) {
        if (mconstraints[ic]->IsActive())
            if (!((mconstraints[ic]->GetMode() == CONSTRAINT_FRIC) && only_bilaterals))
                if (!((dynamic_cast<ChConstraintTwoTuplesFrictionTall*>(mconstraints[ic])) && skip_contacts_uv)) {
                    if (Cq)
                        mconstraints[ic]->Build_Cq(*Cq, s_c);  // .. fills Cq
                    if (E)
                        E->SetElement(s_c, s_c, -mconstraints[ic]->Get_cfm_i());  // .. fills E ( = - cfm )
                    if (Bvector)
                        (*Bvector)(s_c) = mconstraints[ic]->Get_b_i();  // .. fills 'b'
                    if (Frict)                                          // .. fills vector of friction coefficients
                    {
                        (*Frict)(s_c) = -2;  // mark with -2 flag for bilaterals (default)
                        if (ChConstraintTwoTuplesContactNall* mcon =
                                dynamic_cast<ChConstraintTwoTuplesContactNall*>(mconstraints[ic]))
                            (*Frict)(s_c) =
                                mcon->GetFrictionCoefficient();  // friction coeff only in row of normal component
                        if (ChConstraintTwoTuplesFrictionTall* mcon =
                                dynamic_cast<ChConstraintTwoTuplesFrictionTall*>(mconstraints[ic]))
                            (*Frict)(s_c) = -1;  // mark with -1 flag for rows of tangential components
                    }
                    s_c++;
                }
    }
}

void ChSystemDescriptor::ConvertToMatrixForm(ChSparseMatrix* Z, ChMatrix<>* rhs) {

    std::vector<ChConstraint*>& mconstraints = this->GetConstraintsList();
    std::vector<ChVariables*>& mvariables = this->GetVariablesList();

    // Count constraints.
    int mn_c = 0;
    for (unsigned int ic = 0; ic < mconstraints.size(); ic++) {
        if (mconstraints[ic]->IsActive())
            mn_c++;
    }

    // Count active variables, by scanning through all variable blocks, and set offsets.
    n_q = this->CountActiveVariables();

   
	if (Z)
	{
		Z->Reset(n_q + mn_c, n_q + mn_c);

		// Fill Z with masses and inertias.
		int s_q = 0;
		for (unsigned int iv = 0; iv < mvariables.size(); iv++) {
			if (mvariables[iv]->IsActive()) {
				// Masses and inertias in upper-left block of Z
				mvariables[iv]->Build_M(*Z, s_q, s_q, this->c_a);
				s_q += mvariables[iv]->Get_ndof();
			}
		}

		// If present, add stiffness matrix K to upper-left block of Z.
		int s_k = 0;
		for (unsigned int ik = 0; ik < this->vstiffness.size(); ik++) {
			this->vstiffness[ik]->Build_K(*Z, true);
		}

		// Fill Z by looping over constraints.
		int s_c = 0;
		for (unsigned int ic = 0; ic < mconstraints.size(); ic++) {
			if (mconstraints[ic]->IsActive()) {
				// Constraint Jacobian in lower-left block of Z
				mconstraints[ic]->Build_Cq(*Z, n_q + s_c);
				// Transposed constraint Jacobian in upper-right block of Z
				mconstraints[ic]->Build_CqT(*Z, n_q + s_c);
				// -E ( = cfm ) in lower-right block of Z
				Z->SetElement(n_q + s_c, n_q + s_c, mconstraints[ic]->Get_cfm_i());
				//Z->Element(n_q + s_c, n_q + s_c) = mconstraints[ic]->Get_cfm_i();
				s_c++;
			}
		}
	}
    

	if (rhs)
	{
		rhs->Reset(n_q + mn_c, 1);

		// Fill rhs with forces.
		int s_q = 0;
		for (unsigned int iv = 0; iv < mvariables.size(); iv++) {
			if (mvariables[iv]->IsActive()) {
				// Forces in upper section of rhs
				rhs->PasteMatrix(vvariables[iv]->Get_fb(), s_q, 0);
				s_q += mvariables[iv]->Get_ndof();
			}
		}


		// Fill rhs by looping over constraints.
		int s_c = 0;
		for (unsigned int ic = 0; ic < mconstraints.size(); ic++) {
			if (mconstraints[ic]->IsActive()) {
				// -b term in lower section of rhs
				(*rhs)(n_q + s_c) = -(mconstraints[ic]->Get_b_i());
				s_c++;
			}
		}
	}
	

}


void ChSystemDescriptor::DumpLastMatrices(bool assembled, const char* path) {
    char filename[300];
    try {
        const char* numformat = "%.12g";

        if (assembled) {
            ChLinkedListMatrix Z;
            ChMatrixDynamic<double> rhs;
            ConvertToMatrixForm(&Z, &rhs);

            sprintf(filename, "%s%s", path, "Z.dat");
            ChStreamOutAsciiFile file_Z(filename);
            file_Z.SetNumFormat(numformat);
            Z.StreamOUTsparseMatlabFormat(file_Z);

            sprintf(filename, "%s%s", path, "rhs.dat");
            ChStreamOutAsciiFile file_rhs(filename);
            file_rhs.SetNumFormat(numformat);
            rhs.StreamOUTdenseMatlabFormat(file_rhs);
        } else {
            ChLinkedListMatrix mdM;
            ChLinkedListMatrix mdCq;
            ChLinkedListMatrix mdE;
            ChMatrixDynamic<double> mdf;
            ChMatrixDynamic<double> mdb;
            ChMatrixDynamic<double> mdfric;
            ConvertToMatrixForm(&mdCq, &mdM, &mdE, &mdf, &mdb, &mdfric);

            sprintf(filename, "%s%s", path, "M.dat");
            ChStreamOutAsciiFile file_M(filename);
            file_M.SetNumFormat(numformat);
            mdM.StreamOUTsparseMatlabFormat(file_M);

            sprintf(filename, "%s%s", path, "Cq.dat");
                ChStreamOutAsciiFile file_Cq(filename);
            file_Cq.SetNumFormat(numformat);
            mdCq.StreamOUTsparseMatlabFormat(file_Cq);

            sprintf(filename, "%s%s", path, "E.dat");
                ChStreamOutAsciiFile file_E(filename);
            file_E.SetNumFormat(numformat);
            mdE.StreamOUTsparseMatlabFormat(file_E);

            sprintf(filename, "%s%s", path, "f.dat");
                ChStreamOutAsciiFile file_f(filename);
            file_f.SetNumFormat(numformat);
            mdf.StreamOUTdenseMatlabFormat(file_f);

            sprintf(filename, "%s%s", path, "b.dat");
                ChStreamOutAsciiFile file_b(filename);
            file_b.SetNumFormat(numformat);
            mdb.StreamOUTdenseMatlabFormat(file_b);

            sprintf(filename, "%s%s", path, "fric.dat");
                ChStreamOutAsciiFile file_fric(filename);
            file_fric.SetNumFormat(numformat);
            mdfric.StreamOUTdenseMatlabFormat(file_fric);
        }
    } catch (chrono::ChException myexc) {
        chrono::GetLog() << myexc.what();
    }
}

int ChSystemDescriptor::BuildFbVector(ChMatrix<>& Fvector  ///< matrix which will contain the entire vector of 'f'
                                         ) {
    n_q = CountActiveVariables();
    Fvector.Reset(n_q, 1);  // fast! Reset() method does not realloc if size doesn't change

// Fills the 'f' vector
    for (int iv = 0; iv < (int)vvariables.size(); iv++) {
        if (vvariables[iv]->IsActive()) {
            Fvector.PasteMatrix(vvariables[iv]->Get_fb(), vvariables[iv]->GetOffset(), 0);
        }
    }
    return this->n_q;
}

int ChSystemDescriptor::BuildBiVector(ChMatrix<>& Bvector  ///< matrix which will contain the entire vector of 'b'
                                         ) {
    n_c = CountActiveConstraints();
    Bvector.Reset(n_c, 1);

// Fill the 'b' vector
    for (int ic = 0; ic < (int)vconstraints.size(); ic++) {
        if (vconstraints[ic]->IsActive()) {
            Bvector(vconstraints[ic]->GetOffset()) = vconstraints[ic]->Get_b_i();
        }
    }

    return n_c;
}

int ChSystemDescriptor::BuildDiVector(ChMatrix<>& Dvector) {
    n_q = CountActiveVariables();
    n_c = CountActiveConstraints();

    Dvector.Reset(n_q + n_c, 1);  // fast! Reset() method does not realloc if size doesn't change

// Fills the 'f' vector part
    for (int iv = 0; iv < (int)vvariables.size(); iv++) {
        if (vvariables[iv]->IsActive()) {
            Dvector.PasteMatrix(vvariables[iv]->Get_fb(), vvariables[iv]->GetOffset(), 0);
        }
    }
// Fill the '-b' vector (with flipped sign!)
    for (int ic = 0; ic < (int)vconstraints.size(); ic++) {
        if (vconstraints[ic]->IsActive()) {
            Dvector(vconstraints[ic]->GetOffset() + n_q) = -vconstraints[ic]->Get_b_i();
        }
    }

    return n_q + n_c;
}

int ChSystemDescriptor::BuildDiagonalVector(
    ChMatrix<>& Diagonal_vect  ///< matrix which will contain the entire vector of terms on M and E diagonal
    ) {
    n_q = CountActiveVariables();
    n_c = CountActiveConstraints();

    Diagonal_vect.Reset(n_q + n_c, 1);  // fast! Reset() method does not realloc if size doesn't change

    // Fill the diagonal values given by ChKblock objects , if any
    // (This cannot be easily parallelized because of possible write concurrency).
    for (int is = 0; is < (int)vstiffness.size(); is++) {
        vstiffness[is]->DiagonalAdd(Diagonal_vect);
    }

    // Get the 'M' diagonal terms given by ChVariables objects
    for (int iv = 0; iv < (int)vvariables.size(); iv++) {
        if (vvariables[iv]->IsActive()) {
            vvariables[iv]->DiagonalAdd(Diagonal_vect, this->c_a);
        }
    }

// Get the 'E' diagonal terms (note the sign: E_i = -cfm_i )
    for (int ic = 0; ic < (int)vconstraints.size(); ic++) {
        if (vconstraints[ic]->IsActive()) {
            Diagonal_vect(vconstraints[ic]->GetOffset() + n_q) = -vconstraints[ic]->Get_cfm_i();
        }
    }
    return n_q + n_c;
}

int ChSystemDescriptor::FromVariablesToVector(ChMatrix<>& mvector, bool resize_vector) {
    // Count active variables and resize vector if necessary
    if (resize_vector) {
        n_q = CountActiveVariables();
        mvector.Reset(n_q, 1);
    }

// Fill the vector
    for (int iv = 0; iv < (int)vvariables.size(); iv++) {
        if (vvariables[iv]->IsActive()) {
            mvector.PasteMatrix(vvariables[iv]->Get_qb(), vvariables[iv]->GetOffset(), 0);
        }
    }

    return n_q;
}

int ChSystemDescriptor::FromVectorToVariables(ChMatrix<>& mvector) {
    assert(CountActiveVariables() == mvector.GetRows());
    assert(mvector.GetColumns() == 1);

// fetch from the vector
    for (int iv = 0; iv < (int)vvariables.size(); iv++) {
        if (vvariables[iv]->IsActive()) {
            vvariables[iv]->Get_qb().PasteClippedMatrix(mvector, vvariables[iv]->GetOffset(), 0,
                                                        vvariables[iv]->Get_ndof(), 1, 0, 0);
        }
    }

    return n_q;
}

int ChSystemDescriptor::FromConstraintsToVector(ChMatrix<>& mvector, bool resize_vector) {
    // Count active constraints and resize vector if necessary
    if (resize_vector) {
        n_c = CountActiveConstraints();
        mvector.Reset(n_c, 1);
    }

// Fill the vector
    for (int ic = 0; ic < (int)vconstraints.size(); ic++) {
        if (vconstraints[ic]->IsActive()) {
            mvector(vconstraints[ic]->GetOffset()) = vconstraints[ic]->Get_l_i();
        }
    }

    return n_c;
}

int ChSystemDescriptor::FromVectorToConstraints(ChMatrix<>& mvector) {
    n_c = CountActiveConstraints();

    assert(n_c == mvector.GetRows());
    assert(mvector.GetColumns() == 1);

// Fill the vector
    for (int ic = 0; ic < (int)vconstraints.size(); ic++) {
        if (vconstraints[ic]->IsActive()) {
            vconstraints[ic]->Set_l_i(mvector(vconstraints[ic]->GetOffset()));
        }
    }

    return n_c;
}

int ChSystemDescriptor::FromUnknownsToVector(ChMatrix<>& mvector, bool resize_vector) {
    // Count active variables & constraints and resize vector if necessary
    n_q = CountActiveVariables();
    n_c = CountActiveConstraints();

    if (resize_vector) {
        mvector.Reset(n_q + n_c, 1);
    }

// Fill the first part of vector, x.q ,with variables q
    for (int iv = 0; iv < (int)vvariables.size(); iv++) {
        if (vvariables[iv]->IsActive()) {
            mvector.PasteMatrix(vvariables[iv]->Get_qb(), vvariables[iv]->GetOffset(), 0);
        }
    }
// Fill the second part of vector, x.l, with constraint multipliers -l (with flipped sign!)
    for (int ic = 0; ic < (int)vconstraints.size(); ic++) {
        if (vconstraints[ic]->IsActive()) {
            mvector(vconstraints[ic]->GetOffset() + n_q) = -vconstraints[ic]->Get_l_i();
        }
    }

    return n_q + n_c;
}

int ChSystemDescriptor::FromVectorToUnknowns(ChMatrix<>& mvector) {
    n_q = CountActiveVariables();
    n_c = CountActiveConstraints();

    assert((n_q + n_c) == mvector.GetRows());
    assert(mvector.GetColumns() == 1);

// fetch from the first part of vector (x.q = q)
    for (int iv = 0; iv < (int)vvariables.size(); iv++) {
        if (vvariables[iv]->IsActive()) {
            vvariables[iv]->Get_qb().PasteClippedMatrix(mvector, vvariables[iv]->GetOffset(), 0,
                                                        vvariables[iv]->Get_ndof(), 1, 0, 0);
        }
    }
// fetch from the second part of vector (x.l = -l), with flipped sign!
    for (int ic = 0; ic < (int)vconstraints.size(); ic++) {
        if (vconstraints[ic]->IsActive()) {
            vconstraints[ic]->Set_l_i(-mvector(vconstraints[ic]->GetOffset() + n_q));
        }
    }

    return n_q + n_c;
}

void ChSystemDescriptor::ShurComplementProduct(ChMatrix<>& result, ChMatrix<>* lvector, std::vector<bool>* enabled) {
    assert(this->vstiffness.size() == 0); // currently, the case with ChKblock items is not supported (only diagonal M is supported, no K)
    assert(lvector->GetRows() == CountActiveConstraints());
    assert(lvector->GetColumns() == 1);

    result.Reset(n_c, 1);  // fast! Reset() method does not realloc if size doesn't change

// Performs the sparse product    result = [N]*l = [ [Cq][M^(-1)][Cq'] - [E] ] *l
// in different phases:

// 1 - set the qb vector (aka speeds, in each ChVariable sparse data) as zero

    for (int iv = 0; iv < (int)vvariables.size(); iv++) {
        if (vvariables[iv]->IsActive())
            vvariables[iv]->Get_qb().FillElem(0);
    }

    // 2 - performs    qb=[M^(-1)][Cq']*l  by
    //     iterating over all constraints (when implemented in parallel this
    //     could be non-trivial because race conditions might occur -> reduction buffer etc.)
    //     Also, begin to add the cfm term ( -[E]*l ) to the result.

    // ATTENTION:  this loop cannot be parallelized! Concurrent write to some q may happen
    for (int ic = 0; ic < (int)vconstraints.size(); ic++) {
        if (vconstraints[ic]->IsActive()) {
            int s_c = vconstraints[ic]->GetOffset();

            bool process = true;
            if (enabled)
                if ((*enabled)[s_c] == false)
                    process = false;

            if (process) {
                double li;
                if (lvector)
                    li = (*lvector)(s_c, 0);
                else
                    li = vconstraints[ic]->Get_l_i();

                // Compute qb += [M^(-1)][Cq']*l_i
                //  NOTE! concurrent update to same q data, risk of collision if parallel!!
                vconstraints[ic]->Increment_q(li);  // <----!!!  fpu intensive

                // Add constraint force mixing term  result = cfm * l_i = -[E]*l_i
                result(s_c, 0) = vconstraints[ic]->Get_cfm_i() * li;
            }
        }
    }

// 3 - performs    result=[Cq']*qb    by
//     iterating over all constraints

    for (int ic = 0; ic < (int)vconstraints.size(); ic++) {
        if (vconstraints[ic]->IsActive()) {
            bool process = true;
            if (enabled)
                if ((*enabled)[vconstraints[ic]->GetOffset()] == false)
                    process = false;

            if (process)
                result(vconstraints[ic]->GetOffset(), 0) +=
                    vconstraints[ic]->Compute_Cq_q();  // <----!!!  fpu intensive
            else
                result(vconstraints[ic]->GetOffset(), 0) = 0;  // not enabled constraints, just set to 0 result
        }
    }
}

void ChSystemDescriptor::SystemProduct(
    ChMatrix<>& result,  ///< matrix which contains the result of matrix by x
    ChMatrix<>* x        ///< optional matrix with the vector to be multiplied (if null, use current l_i and q)
    // std::vector<bool>* enabled=0 ///< optional: vector of enable flags, one per scalar constraint. true=enable,
    // false=disable (skip)
    ) {
    n_q = this->CountActiveVariables();
    n_c = this->CountActiveConstraints();

    ChMatrix<>* x_ql = 0;

    ChMatrix<>* vect;

    if (x) {
        assert(x->GetRows() == n_q + n_c);
        assert(x->GetColumns() == 1);

        vect = x;
    } else {
        x_ql = new ChMatrixDynamic<double>(n_q + n_c, 1);
        vect = x_ql;
        this->FromUnknownsToVector(*vect);
    }

    result.Reset(n_q + n_c, 1);  // fast! Reset() method does not realloc if size doesn't change

// 1) First row: result.q part =  [M + K]*x.q + [Cq']*x.l

// 1.1)  do  M*x.q
    for (int iv = 0; iv < (int)vvariables.size(); iv++)
        if (vvariables[iv]->IsActive()) {
            vvariables[iv]->MultiplyAndAdd(result, *x, this->c_a);
        }

    // 1.2)  add also K*x.q  (NON straight parallelizable - risk of concurrency in writing)
    for (int ik = 0; ik < (int)vstiffness.size(); ik++) {
        vstiffness[ik]->MultiplyAndAdd(result, *x);
    }

    // 1.3)  add also [Cq]'*x.l  (NON straight parallelizable - risk of concurrency in writing)
    for (int ic = 0; ic < (int)vconstraints.size(); ic++) {
        if (vconstraints[ic]->IsActive()) {
            vconstraints[ic]->MultiplyTandAdd(result, (*x)(vconstraints[ic]->GetOffset() + n_q));
        }
    }

// 2) Second row: result.l part =  [C_q]*x.q + [E]*x.l
    for (int ic = 0; ic < (int)vconstraints.size(); ic++) {
        if (vconstraints[ic]->IsActive()) {
            int s_c = vconstraints[ic]->GetOffset() + n_q;
            vconstraints[ic]->MultiplyAndAdd(result(s_c), (*x));       // result.l_i += [C_q_i]*x.q
            result(s_c) -= vconstraints[ic]->Get_cfm_i() * (*x)(s_c);  // result.l_i += [E]*x.l_i  NOTE:  cfm = -E
        }
    }

    // if a temp vector has been created because x was not provided, then delete it
    if (x_ql)
        delete x_ql;
}

void ChSystemDescriptor::ConstraintsProject(
    ChMatrix<>& multipliers  ///< matrix which contains the entire vector of 'l_i' multipliers to be projected
    ) {
    this->FromVectorToConstraints(multipliers);

    for (int ic = 0; ic < (int)vconstraints.size(); ic++) {
        if (vconstraints[ic]->IsActive())
            vconstraints[ic]->Project();
    }

    this->FromConstraintsToVector(multipliers, false);
}

void ChSystemDescriptor::UnknownsProject(
    ChMatrix<>& mx  ///< matrix which contains the entire vector of unknowns x={q,-l} (only the l part is projected)
    ) {
    n_q = this->CountActiveVariables();

// vector -> constraints
// Fetch from the second part of vector (x.l = -l), with flipped sign!
    for (int ic = 0; ic < (int)vconstraints.size(); ic++) {
        if (vconstraints[ic]->IsActive()) {
            vconstraints[ic]->Set_l_i(-mx(vconstraints[ic]->GetOffset() + n_q));
        }
    }

// constraint projection!
    for (int ic = 0; ic < (int)vconstraints.size(); ic++) {
        if (vconstraints[ic]->IsActive())
            vconstraints[ic]->Project();
    }

// constraints -> vector
// Fill the second part of vector, x.l, with constraint multipliers -l (with flipped sign!)
    for (int ic = 0; ic < (int)vconstraints.size(); ic++) {
        if (vconstraints[ic]->IsActive()) {
            mx(vconstraints[ic]->GetOffset() + n_q) = -vconstraints[ic]->Get_l_i();
        }
    }
}

void ChSystemDescriptor::SetNumThreads(int nthreads) {
    if (nthreads == this->num_threads)
        return;

    this->num_threads = nthreads;
}

}  // end namespace chrono
