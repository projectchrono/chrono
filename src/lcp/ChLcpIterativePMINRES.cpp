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
	bool do_project_p = false;
	bool verbose = false;
//***TEST***
bool do_dump = false;
int iters_to_dump=5;
this->grad_diffstep = 0.01;
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

	std::vector<ChLcpConstraint*>& mconstraints = sysd.GetConstraintsList();
	std::vector<ChLcpVariables*>&  mvariables	= sysd.GetVariablesList();

	double maxviolation = 0.;

	
	


	// Update auxiliary data in all constraints before starting,
	// that is: g_i=[Cq_i]*[invM_i]*[Cq_i]' and  [Eq_i]=[invM_i]*[Cq_i]'
	for (unsigned int ic = 0; ic< mconstraints.size(); ic++)
		mconstraints[ic]->Update_auxiliary();


	// Allocate auxiliary vectors;
	
	int nc = sysd.CountActiveConstraints();
	if (verbose) GetLog() <<"\n-----Projected MINRES, solving nc=" << nc << "unknowns \n";

	ChMatrixDynamic<> ml(nc,1);
	ChMatrixDynamic<> mb(nc,1);
	ChMatrixDynamic<> mp(nc,1);
	ChMatrixDynamic<> mr(nc,1);
	ChMatrixDynamic<> mr_old(nc,1);
	ChMatrixDynamic<> mNp(nc,1);
	ChMatrixDynamic<> mNr(nc,1);
	ChMatrixDynamic<> mNr_old(nc,1);
	ChMatrixDynamic<> mtmp(nc,1);


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



if (do_dump) //***TEST***
{
chrono::ChStreamOutAsciiFile file_shur("test__b_shur.dat"); file_shur.SetNumFormat("%.9g");
mb.StreamOUTdenseMatlabFormat(file_shur);
}

		// Optimization: backup the  q  sparse data computed above, 
		// because   (M^-1)*k   will be needed at the end when computing primals.
	ChMatrixDynamic<> mq; 
	sysd.FromVariablesToVector(mq, true);	




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

			//*** should project initial mp and mr ?
	// r = (project_orthogonal(l+diff*r, fric) - l)/diff;
	mr.MatrScale(this->grad_diffstep);
	mr.MatrInc(ml);
	sysd.ConstraintsProject(mr);				// p = P(l+diff*p) ...
	mr.MatrDec(ml);
	mr.MatrDivScale(this->grad_diffstep);		// p = (P(l+diff*p)-l)/diff

	// p = r;
	mp = mr;   

	// Nr = N*r
	sysd.ShurComplementProduct(mNr, &mr);		// Nr = N*r    #### MATR.MULTIPLICATION!!!###

	// Np = N*p  i.e.  Np = Nr  because starting with p=r
	mNp = mNr;

if (do_dump) //***TEST***
{
chrono::ChStreamOutAsciiFile file_mr("test__r_0.dat"); file_mr.SetNumFormat("%.9g");
mr.StreamOUTdenseMatlabFormat(file_mr);
chrono::ChStreamOutAsciiFile file_mp("test__p_0.dat"); file_mp.SetNumFormat("%.9g");
mp.StreamOUTdenseMatlabFormat(file_mp);
chrono::ChStreamOutAsciiFile file_mNr("test__Nr_0.dat"); file_mNr.SetNumFormat("%.9g");
mNr.StreamOUTdenseMatlabFormat(file_mNr);
}


	//
	// THE LOOP
	//

	std::vector<double> f_hist;

	for (int iter = 0; iter < max_iterations; iter++)
	{

		// alpha = r'*(Nr)/((Np)'*(Np));  // alpha =  u'*p / p'*N*p 
		double rNr =  mr.MatrDot(&mr,&mNr);			// 1)  rNr = r'*N*r
		double NpNp = mNp.MatrDot(&mNp,&mNp);		// 2)  NpNp = ((Np)'*(Np))
		 if (fabs(NpNp)<10e-12) 
		 {
			if (verbose) GetLog() << "Rayleygh quotient alpha breakdown: " << rNr << " / " << NpNp << "  iter=" << iter << "\n";
			NpNp=10e-12;
		 }
		double alpha = rNr/NpNp;					// 3)  alpha = r'*(Nr)/((Np)'*(Np))

		// l = l + alpha * p;
		mtmp = mp;
		mtmp.MatrScale(alpha);
		ml.MatrInc(mtmp);

if (do_dump && (iter<iters_to_dump)) //***TEST***
{
char mybuff[100];
sprintf(mybuff,"test__l_%d.dat", (iter+1));
chrono::ChStreamOutAsciiFile file_ml(mybuff); file_ml.SetNumFormat("%.9g");
ml.StreamOUTdenseMatlabFormat(file_ml);
}

		double maxdeltalambda = mtmp.NormInf();

		if (maxdeltalambda < this->GetTolerance() ) 
		{
			if (verbose) GetLog() << "Converged! iter=" << iter <<  "\n";
		 	//break;
		}

		// l = Proj(l)
		sysd.ConstraintsProject(ml);				// l = P(l) 

if (do_dump && (iter<iters_to_dump)) //***TEST***
{
char mybuff[100];
sprintf(mybuff,"test__l_proj_%d.dat", (iter+1));
chrono::ChStreamOutAsciiFile file_ml(mybuff); file_ml.SetNumFormat("%.9g");
ml.StreamOUTdenseMatlabFormat(file_ml);
}

		// r_old = r;
		mr_old = mr;

		// r = b - N*l;
		sysd.ShurComplementProduct(mr, &ml);		// 1)  r = N*l ...        #### MATR.MULTIPLICATION!!!###
		mr.MatrNeg();								// 2)  r =-N*l
		mr.MatrInc(mb);								// 3)  r =-N*l+b

if (do_dump && (iter<iters_to_dump)) //***TEST***
{
char mybuff[100];
sprintf(mybuff,"test__r_%d.dat", (iter+1));
chrono::ChStreamOutAsciiFile file_mr(mybuff); file_mr.SetNumFormat("%.9g");
mr.StreamOUTdenseMatlabFormat(file_mr);
sprintf(mybuff,"test__p_%d.dat", (iter+1));
chrono::ChStreamOutAsciiFile file_mp(mybuff); file_mp.SetNumFormat("%.9g");
mp.StreamOUTdenseMatlabFormat(file_mp);
}
		
		// r = (project_orthogonal(l+diff*r, fric) - l)/diff; 
		mr.MatrScale(this->grad_diffstep);
		mr.MatrInc(ml);
if (do_dump) //***TEST***
{
char mybuff[100];
sprintf(mybuff,"test__rA_%d.dat", (iter+1));
chrono::ChStreamOutAsciiFile file_mr(mybuff); file_mr.SetNumFormat("%.9g");
mr.StreamOUTdenseMatlabFormat(file_mr);
}
		sysd.ConstraintsProject(mr);				// r = P(l+diff*r) ...
if (do_dump) //***TEST***
{
char mybuff[100];
sprintf(mybuff,"test__rB_%d.dat", (iter+1));
chrono::ChStreamOutAsciiFile file_mr(mybuff); file_mr.SetNumFormat("%.9g");
mr.StreamOUTdenseMatlabFormat(file_mr);
}
		mr.MatrDec(ml);
if (do_dump) //***TEST***
{
char mybuff[100];
sprintf(mybuff,"test__rC_%d.dat", (iter+1));
chrono::ChStreamOutAsciiFile file_mr(mybuff); file_mr.SetNumFormat("%.9g");
mr.StreamOUTdenseMatlabFormat(file_mr);
}
		mr.MatrDivScale(this->grad_diffstep);		// r = (P(l+diff*r)-l)/diff 

		if (do_project_p)
		{
			// p = (project_orthogonal(l+diff*p, fric) - l)/diff;
			mp.MatrScale(this->grad_diffstep);
			mp.MatrInc(ml);
			sysd.ConstraintsProject(mp);				// p = P(l+diff*p) ...
			mp.MatrDec(ml);
			mp.MatrDivScale(this->grad_diffstep);		// p = (P(l+diff*p)-l)/diff 
		}

if (do_dump && (iter<iters_to_dump)) //***TEST***
{
char mybuff[100];
sprintf(mybuff,"test__r_proj_%d.dat", (iter+1));
chrono::ChStreamOutAsciiFile file_mr(mybuff); file_mr.SetNumFormat("%.9g");
mr.StreamOUTdenseMatlabFormat(file_mr);
sprintf(mybuff,"test__p_proj_%d.dat", (iter+1));
chrono::ChStreamOutAsciiFile file_mp(mybuff); file_mp.SetNumFormat("%.9g");
mp.StreamOUTdenseMatlabFormat(file_mp);
}

		// Nr_old = Nr;
		mNr_old = mNr;

		// Nr = N*r; 
		sysd.ShurComplementProduct(mNr, &mr);		// Nr = N*r    #### MATR.MULTIPLICATION!!!###

		// beta = r'*(Nr-Nr_old)/(r_old'*(Nr_old));
		mtmp.MatrSub(mNr,mNr_old);
		double numerator = mr.MatrDot(&mr,&mtmp);
		double denominator = mr_old.MatrDot(&mr_old, &mNr_old);
		if (fabs(denominator)<10e-12)
		 {
			if (verbose) GetLog() << "Rayleygh quotient beta breakdown: " << numerator << " / " << denominator <<  "  iter=" << iter << "\n";
			denominator=10e-12;
		 }
		double beta = numerator / denominator;
		
		beta = ChMax(0.0, beta);

if (do_dump)// && (iter<iters_to_dump)) //***TEST***
{
	GetLog() << " iter=" << (iter+1) << "   alpha= " << alpha << "   beta= " << beta << "\n";
	*file_alphabeta << alpha << "  " << beta << "\n";
}

		// p = r + beta * p;
		mtmp = mp;
		mtmp.MatrScale(beta);
		mp = mr;
		mp.MatrInc(mtmp);

		if (do_project_p)
		{
			// Np = N*p;
			sysd.ShurComplementProduct(mNp, &mp);		// Np = N*p    #### MATR.MULTIPLICATION!!!###
		}
		else
		{
			// Np = Nr + beta*Np;     // Optimization!! avoid matr x vect!!! (if no 'p' projection has been done)
			mNp.MatrScale(beta);
			mNp.MatrInc(mNr);
		}



		// ---------------------------------------------
		// METRICS - convergence, plots, etc

		//double maxd			  = mu.NormInf();  // ***TO DO***  should be max violation, but just for test...
		double maxd=0; // ***TO DO***

		// For recording into correction/residuals/violation history, if debugging
		if (this->record_violation_history)
			AtIterationEnd(maxd, maxdeltalambda, iter);
			
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
	
if (do_dump)
{
	delete file_alphabeta;
}

	if (verbose) GetLog() <<"-----\n";

	return maxviolation;

}







} // END_OF_NAMESPACE____


