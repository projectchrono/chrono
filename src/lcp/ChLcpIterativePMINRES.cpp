///////////////////////////////////////////////////
//
//   ChLcpIterativePMINRES.cpp
//
//
//    file for CHRONO HYPEROCTANT LCP solver
//
// ------------------------------------------------
// 	 Copyright:Alessandro Tasora / DeltaKnowledge
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////
  
   
#include "ChLcpIterativePMINRES.h"
#include "ChLcpConstraintTwoFrictionT.h"

namespace chrono
{

double ChLcpIterativePMINRES::Solve(
					ChLcpSystemDescriptor& sysd,		///< system description with constraints and variables	
					bool add_Mq_to_f 
					)
{
	bool do_preconditioning = false;
	bool verbose = false;
this->grad_diffstep = 0.000000001;

/*
//***TEST***
bool do_dump = false;
int iters_to_dump=5;

chrono::ChStreamOutAsciiFile* file_alphabeta= 0;
if (do_dump) //***TEST***
{
file_alphabeta = new chrono::ChStreamOutAsciiFile("test__alpha_beta.dat"); file_alphabeta->SetNumFormat("%.9g");
chrono::ChMatrixDynamic<double> mvold;
sysd.FromVariablesToVector(mvold);
chrono::ChStreamOutAsciiFile file_vold("test_v_old.dat"); file_vold.SetNumFormat("%.9g");
mvold.StreamOUTdenseMatlabFormat(file_vold);
chrono::ChSparseMatrix mdM;
chrono::ChSparseMatrix mdCq;
chrono::ChSparseMatrix mdE;
chrono::ChMatrixDynamic<double> mdf;
chrono::ChMatrixDynamic<double> mdb;
chrono::ChMatrixDynamic<double> mdfric;
sysd.ConvertToMatrixForm(&mdCq, &mdM, &mdE, &mdf, &mdb, &mdfric);
chrono::ChStreamOutAsciiFile file_M("test_M.dat"); file_M.SetNumFormat("%.9g");
mdM.StreamOUTsparseMatlabFormat(file_M);
chrono::ChStreamOutAsciiFile file_Cq("test_Cq.dat"); file_Cq.SetNumFormat("%.9g");
mdCq.StreamOUTsparseMatlabFormat(file_Cq);
chrono::ChStreamOutAsciiFile file_E("test_E.dat"); file_E.SetNumFormat("%.9g");
mdE.StreamOUTsparseMatlabFormat(file_E);
chrono::ChStreamOutAsciiFile file_f("test_f.dat"); file_f.SetNumFormat("%.9g");
mdf.StreamOUTdenseMatlabFormat(file_f);
chrono::ChStreamOutAsciiFile file_b("test_b.dat"); file_b.SetNumFormat("%.9g");
mdb.StreamOUTdenseMatlabFormat(file_b);
chrono::ChStreamOutAsciiFile file_fric("test_fric.dat"); file_fric.SetNumFormat("%.9g");
mdfric.StreamOUTdenseMatlabFormat(file_fric);
}
*/
	std::vector<ChLcpConstraint*>& mconstraints = sysd.GetConstraintsList();
	std::vector<ChLcpVariables*>&  mvariables	= sysd.GetVariablesList();


		// Allocate auxiliary vectors;
	
	int nc = sysd.CountActiveConstraints();
	if (verbose) GetLog() <<"\n-----Projected MINRES, solving nc=" << nc << "unknowns \n";

	ChMatrixDynamic<> ml(nc,1);
	ChMatrixDynamic<> mb(nc,1);
	ChMatrixDynamic<> mp(nc,1);
	ChMatrixDynamic<> mr(nc,1);
	ChMatrixDynamic<> mz(nc,1);
	ChMatrixDynamic<> mz_old(nc,1);
	ChMatrixDynamic<> mNp(nc,1);
	ChMatrixDynamic<> mMNp(nc,1);
	ChMatrixDynamic<> mNMr(nc,1);
	ChMatrixDynamic<> mNMr_old(nc,1);
	ChMatrixDynamic<> mtmp(nc,1);
	ChMatrixDynamic<> mD (nc,1);


	double maxviolation = 0.;


	// Update auxiliary data in all constraints before starting,
	// that is: g_i=[Cq_i]*[invM_i]*[Cq_i]' and  [Eq_i]=[invM_i]*[Cq_i]'
	for (unsigned int ic = 0; ic< mconstraints.size(); ic++)
		mconstraints[ic]->Update_auxiliary();

	// Average all g_i for the triplet of contact constraints n,u,v.
	//  Can be used as diagonal preconditioner.
	int j_friction_comp = 0;
	double gi_values[3];
	for (unsigned int ic = 0; ic< mconstraints.size(); ic++)
	{
		if (mconstraints[ic]->GetMode() == CONSTRAINT_FRIC) 
		{
			gi_values[j_friction_comp] = mconstraints[ic]->Get_g_i();
			j_friction_comp++;
			if (j_friction_comp==3)
			{
				double average_g_i = (gi_values[0]+gi_values[1]+gi_values[2])/3.0;
				mconstraints[ic-2]->Set_g_i(average_g_i);
				mconstraints[ic-1]->Set_g_i(average_g_i);
				mconstraints[ic-0]->Set_g_i(average_g_i);
				j_friction_comp=0;
			}
		}	
	}
			
	// The vector with the diagonal of the N matrix
	int d_i = 0;
	for (unsigned int ic = 0; ic< mconstraints.size(); ic++)
		if (mconstraints[ic]->IsActive())
		{
			mD(d_i, 0) = mconstraints[ic]->Get_g_i();
			++d_i;
		}


	// ***TO DO*** move the following thirty lines in a short function ChLcpSystemDescriptor::ShurBvectorCompute() ?

	// Compute the b_shur vector in the Shur complement equation N*l = b_shur
	// with 
	//   N_shur  = D'* (M^-1) * D
	//   b_shur  = - c + D'*(M^-1)*k = b_i + D'*(M^-1)*k
	// but flipping the sign of lambdas,  b_shur = - b_i - D'*(M^-1)*k
	// Do this in three steps:
	
	// Put (M^-1)*k    in  q  sparse vector of each variable..
	for (unsigned int iv = 0; iv< mvariables.size(); iv++)
		if (mvariables[iv]->IsActive())
			if (add_Mq_to_f)
				mvariables[iv]->Compute_inc_invMb_v(mvariables[iv]->Get_qb(), mvariables[iv]->Get_fb()); // q = q_old + [M]'*fb 
			else
				mvariables[iv]->Compute_invMb_v(mvariables[iv]->Get_qb(), mvariables[iv]->Get_fb()); // q = [M]'*fb 

	// ...and now do  b_shur = - D' * q  ..
	int s_i = 0;
	for (unsigned int ic = 0; ic< mconstraints.size(); ic++)
		if (mconstraints[ic]->IsActive())
		{
			mb(s_i, 0) = - mconstraints[ic]->Compute_Cq_q();
			++s_i;
		}

	// ..and finally do   b_shur = b_shur - c
	sysd.BuildBiVector(mtmp);	// b_i   =   -c   = phi/h 
	mb.MatrDec(mtmp);  


		// Optimization: backup the  q  sparse data computed above, 
		// because   (M^-1)*k   will be needed at the end when computing primals.
	ChMatrixDynamic<> mq; 
	sysd.FromVariablesToVector(mq, true);	


	double rel_tol = 1e-6;
	double abs_tol = 1e-6;
	double rel_tol_b = mb.NormInf() * rel_tol;


	// Initialize lambdas
	if (warm_start)
		sysd.FromConstraintsToVector(ml);
	else
		ml.FillElem(0);

	// Initial projection of ml   ***TO DO***?
	// ...

	// r = b - N*l;
	sysd.ShurComplementProduct(mr, &ml);		// 1)  r = N*l ...        #### MATR.MULTIPLICATION!!!### can be avoided if no warm starting!
	mr.MatrNeg();								// 2)  r =-N*l
	mr.MatrInc(mb);								// 3)  r =-N*l+b

	// r = (project_orthogonal(l+diff*r, fric) - l)/diff;
	mr.MatrScale(this->grad_diffstep);
	mr.MatrInc(ml);
	sysd.ConstraintsProject(mr);				// p = P(l+diff*p) ...
	mr.MatrDec(ml);
	mr.MatrDivScale(this->grad_diffstep);		// p = (P(l+diff*p)-l)/diff

	// p = Mi * r;
	mp = mr;  
	if (do_preconditioning)
		mp.MatrDivScale(mD);
	
	// z = Mi * r;
	mz = mp;

	// NMr = N*M*r = N*z
	sysd.ShurComplementProduct(mNMr, &mz);		// NMr = N*z    #### MATR.MULTIPLICATION!!!###

	// Np = N*p  
	sysd.ShurComplementProduct(mNp, &mp);		// Np = N*p    #### MATR.MULTIPLICATION!!!###


	//
	// THE LOOP
	//

	std::vector<double> f_hist;

	for (int iter = 0; iter < max_iterations; iter++)
	{
		// MNp = Mi*Np; % = Mi*N*p                  %% -- Precond
		mMNp = mNp;
		if (do_preconditioning)
			mMNp.MatrDivScale(mD);

		// alpha = (z'*(NMr))/((MNp)'*(Np));
		double zNMr =  mz.MatrDot(&mz,&mNMr);		// 1)  zMNr = z'* NMr
		double MNpNp = mMNp.MatrDot(&mMNp,&mNp);	// 2)  MNpNp = ((MNp)'*(Np))
		/*
		 if (fabs(MNpNp)<10e-12) 
		 {
			if (verbose) GetLog() << "Rayleygh quotient alpha breakdown: " << zNMr << " / " << MNpNp << "  iter=" << iter << "\n";
			MNpNp=10e-12;
		 }
		 */
		double alpha = zNMr/MNpNp;					// 3)  alpha = (z'*(NMr))/((MNp)'*(Np));

		// l = l + alpha * p;
		mtmp = mp;
		mtmp.MatrScale(alpha);
		ml.MatrInc(mtmp);

		double maxdeltalambda = mtmp.NormTwo(); //***better NormInf() for speed reasons?

		/*
		if (maxdeltalambda < this->GetTolerance() ) 
		{
			if (verbose) GetLog() << "Converged! iter=" << iter <<  "\n";
		 	//break;
		}
		*/

		// l = Proj(l)
		sysd.ConstraintsProject(ml);				// l = P(l) 


		// r = b - N*l;
		sysd.ShurComplementProduct(mr, &ml);		// 1)  r = N*l ...        #### MATR.MULTIPLICATION!!!###
		mr.MatrNeg();								// 2)  r =-N*l
		mr.MatrInc(mb);								// 3)  r =-N*l+b
		
		// r = (project_orthogonal(l+diff*r, fric) - l)/diff; 
		mr.MatrScale(this->grad_diffstep);
		mr.MatrInc(ml);
		sysd.ConstraintsProject(mr);				// r = P(l+diff*r) ...
		mr.MatrDec(ml);
		mr.MatrDivScale(this->grad_diffstep);		// r = (P(l+diff*r)-l)/diff 
		
		// Terminate iteration when the projected r is small, if (norm(r,2) <= max(rel_tol_b,abs_tol))
		double r_proj_resid = mr.NormTwo();
		if (r_proj_resid < ChMax(rel_tol_b, abs_tol) )
		{
			//if (verbose) 
				GetLog() << "P(r)-converged! iter=" << iter <<  " |P(r)|=" << r_proj_resid << "\n";
			break;
		}

		// z_old = z;
		mz_old = mz;
    
		// z = Mi*r;                                 %% -- Precond
		mz = mr;
		if (do_preconditioning)
			mz.MatrDivScale(mD);

		// NMr_old = NMr;
		mNMr_old = mNMr;
   
		// NMr = N*z;                             
		sysd.ShurComplementProduct(mNMr, &mz);		// NMr = N*z;    #### MATR.MULTIPLICATION!!!###

		// beta = z'*(NMr-NMr_old)/(z_old'*(NMr_old));
		mtmp.MatrSub(mNMr,mNMr_old);
		double numerator = mz.MatrDot(&mz,&mtmp);
		double denominator = mz_old.MatrDot(&mz_old, &mNMr_old);
		/*
		if (fabs(denominator)<10e-12)
		 {
			if (verbose) GetLog() << "Rayleygh quotient beta breakdown: " << numerator << " / " << denominator <<  "  iter=" << iter << "\n";
			denominator=10e-12;
		 }
		 */
		double beta = numerator / denominator;
		
		beta = ChMax(0.0, beta);

		// p = z + beta * p;   
		mtmp = mp;
		mtmp.MatrScale(beta);
		mp = mz;
		mp.MatrInc(mtmp);


		// Np = NMr + beta*Np;   // Optimization!! avoid matr x vect!!! (if no 'p' projection has been done)
		mNp.MatrScale(beta);
		mNp.MatrInc(mNMr);




		// ---------------------------------------------
		// METRICS - convergence, plots, etc

		// For recording into correction/residuals/violation history, if debugging
		if (this->record_violation_history)
			AtIterationEnd(r_proj_resid, maxdeltalambda, iter);
			
	}


	// Resulting DUAL variables:
	// store ml temporary vector into ChLcpConstraint 'l_i' multipliers
	sysd.FromVectorToConstraints(ml); 


	// Resulting PRIMAL variables:
	// compute the primal variables as   v = (M^-1)(k + D*l) 

		// v = (M^-1)*k  ...    (by rewinding to the backup vector computed ad the beginning)
	sysd.FromVectorToVariables(mq);


		// ... + (M^-1)*D*l     (this increment and also stores 'qb' in the ChLcpVariable items)
	for (unsigned int ic = 0; ic < mconstraints.size(); ic++)
	{	
		if (mconstraints[ic]->IsActive())
			mconstraints[ic]->Increment_q( mconstraints[ic]->Get_l_i() );
	}

	if (verbose) GetLog() <<"-----\n";

	return maxviolation;

}







} // END_OF_NAMESPACE____


