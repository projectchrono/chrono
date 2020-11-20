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
// Authors: Alessandro Tasora
// =============================================================================

#include "chrono/solver/ChSolverADMM.h"
#include "chrono/core/ChMathematics.h"

#include "chrono/solver/ChIterativeSolverLS.h"
#include "chrono/core/ChSparsityPatternLearner.h"

namespace chrono {

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChSolverADMM)

ChSolverADMM::ChSolverADMM() : 
    r_prim (0),
    r_dual (0),
    precond (false),
    rho(0.1),
    rho_b(1e-9),
    sigma(1e-6),
    stepadjust_each(5),
    stepadjust_threshold(1.5),
    stepadjust_maxfactor(50),
    stepadjust_type(AdmmStepType::BALANCED_FAST),
    tol_prim(1e-6),
    tol_dual(1e-6)
{
    LS_solver = chrono_types::make_shared<ChSolverSparseQR>();
}

ChSolverADMM::ChSolverADMM(std::shared_ptr<ChDirectSolverLS> my_LS_engine) : 
    ChSolverADMM()
{ 
    this->LS_solver = my_LS_engine;
}


double ChSolverADMM::Solve(ChSystemDescriptor& sysd) {

    ChTimer<> m_timer_convert;
    ChTimer<> m_timer_factorize;
    ChTimer<> m_timer_solve;

    double rho_i = this->rho;

    std::vector<ChConstraint*>& mconstraints = sysd.GetConstraintsList();
    std::vector<ChVariables*>& mvariables = sysd.GetVariablesList();

    int nc = sysd.CountActiveConstraints();
    int nv = sysd.CountActiveVariables();

    ChVectorDynamic<> v(nv);

    // shortcut for the case of no constraints:
    if (nc == 0) {

        ChSparseMatrix H(nv, nv);
        ChVectorDynamic<> k(nv);


        m_timer_convert.start();

        //sysd.ConvertToMatrixForm(0, &LS_solver->A(), 0, &LS_solver->b(), 0, 0, 0);
        // much faster to fill brand new sparse matrices??!!

        ChSparsityPatternLearner sparsity_pattern(nv, nv);
        sysd.ConvertToMatrixForm(&sparsity_pattern, 0); 
        sparsity_pattern.Apply(H);
        sysd.ConvertToMatrixForm(&H,&k);  
        LS_solver->A() = H; 
        LS_solver->b() = k; 

        m_timer_convert.stop();
        if (this->verbose) GetLog() << " Time for ConvertToMatrixForm: << " << m_timer_convert.GetTimeSecondsIntermediate() << "s\n";

        // v = H\k
        LS_solver->SetupCurrent();
        LS_solver->SolveCurrent();

        //v = LS_solver->x();
        sysd.FromVectorToVariables(LS_solver->x());

        return 0;
    }
        
    ChSparseMatrix Cq(nc,nv);
    ChSparseMatrix E(nc,nc);
    ChVectorDynamic<> k(nv);
    ChVectorDynamic<> b(nc);

    ChSparseMatrix    A(nv+nc,nv+nc);
    ChVectorDynamic<> B(nv+nc);
    //ChVectorDynamic<> X(nv+nc);


    ChVectorDynamic<> l(nc);
    ChVectorDynamic<> z(nc);
    ChVectorDynamic<> y(nc);
    ChVectorDynamic<> ltemp(nc);

    ChVectorDynamic<> l_old(nc);
    ChVectorDynamic<> z_old(nc);
    ChVectorDynamic<> y_old(nc);
    
    sysd.ConvertToMatrixForm(&Cq, 0, &E, &k, &b, 0);
    Cq.makeCompressed();
    E.makeCompressed();

    if (!this->m_warm_start) {
        l.setZero();
        z.setZero();
        y.setZero();
    }
    else
    {
        // warmstarted l:
        sysd.FromConstraintsToVector(l, false);

        // warmstarted v:
        //v = H\(k + D*l); // PERFORMANCE HIT, probably better reuse last v if possible..
        sysd.FromVariablesToVector(v, false); // this works supposing that variables have been warmstarted with "v" too, otherwise:  

        // warmstarted y:
        // the following correct only if r_dual was approx.=0. //***TODO*** as parameter
        y = - (Cq*v - E*l + b); //  dual residual exploiting the kkt form instead of - (N*l+r), faster!
            
        /*
        GetLog() << "Y warmastarted:  \n";
        for (int k = 0; k < ChMin(y.rows(), 10); ++k)
            GetLog() << "  " << y(k) << "\n";
        */
        
        // warmstarted z:
        z = l; //  warm start also this - should project? //***TODO*** as parameter
        //sysd.ConstraintsProject(z);

        // y_hat  = y;   // only for spectral stepsize
    }

    ChVectorDynamic<> S(nc);

    if (this->precond == true) {
        // Compute diagonal values of N , only mass effect, neglecting stiffness for the moment, TODO
        //  g_i=[Cq_i]*[invM_i]*[Cq_i]' 
        for (unsigned int ic = 0; ic < mconstraints.size(); ic++)
            mconstraints[ic]->Update_auxiliary();

        // Average all g_i for the triplet of contact constraints n,u,v.
        int j_friction_comp = 0;
        double gi_values[3];
        for (unsigned int ic = 0; ic < mconstraints.size(); ic++) {
            if (mconstraints[ic]->GetMode() == CONSTRAINT_FRIC) {
                gi_values[j_friction_comp] = mconstraints[ic]->Get_g_i();
                j_friction_comp++;
                if (j_friction_comp == 3) {
                    double average_g_i = (gi_values[0] + gi_values[1] + gi_values[2]) / 3.0;
                    mconstraints[ic - 2]->Set_g_i(average_g_i);
                    mconstraints[ic - 1]->Set_g_i(average_g_i);
                    mconstraints[ic - 0]->Set_g_i(average_g_i);
                    j_friction_comp = 0;
                }
            }
        }
        // The vector with the diagonal of the N matrix
        S.setZero();
        int d_i = 0;
        for (unsigned int ic = 0; ic < mconstraints.size(); ic++)
            if (mconstraints[ic]->IsActive()) {
                S(d_i, 0) = sqrt(mconstraints[ic]->Get_g_i());  // square root of diagonal of N, just mass matrices considered, no stiffness matrices anyway
                ++d_i;
            }
        // TODO: scale Cq, E, b
        // Cq = Cq*diag(S);
        // E = diag(S)*E*diag(S);
        // b = diag(S)*b;
        S.setConstant(1);
    }
    else {
        S.setConstant(1);
    }  

    // vsigma = ones(nconstr,1)*sigma;
    ChVectorDynamic<> vsigma(nc); // not needed
    vsigma.setConstant(this->sigma);
    

    // vrho = ones(nconstr,1)*rho;
    ChVectorDynamic<> vrho(nc);
    vrho.setConstant(rho_i);

    // vrho(fric==-2) = rho_b;          // special step for bilateral joints
    int s_c = 0;
    for (unsigned int ic = 0; ic < mconstraints.size(); ic++) {
        if (mconstraints[ic]->IsActive()) {
            if (mconstraints[ic]->GetMode()==eChConstraintMode::CONSTRAINT_LOCK)
                vrho(s_c) = rho_b;
            s_c++;
        }
    }
 
    // FACTORIZATION
    //
    // A = [M, Cq'; Cq, -diag(vsigma+vrho) + E ];

    
    m_timer_convert.start();

    LS_solver->A().resize(nv + nc, nv + nc); // otherwise conservativeResize in ConvertToMatrixForm() causes error

    //sysd.ConvertToMatrixForm(&LS_solver->A(),&LS_solver->b());  // A = [M, Cq'; Cq, E ];
    // much faster to fill brand new sparse matrices??!!
    ChSparsityPatternLearner sparsity_pattern(nv + nc, nv + nc);
    sysd.ConvertToMatrixForm(&sparsity_pattern, nullptr);
    sparsity_pattern.Apply(A);

    sysd.ConvertToMatrixForm(&A,&B);  // A = [M, Cq'; Cq, E ]; 
    LS_solver->A() = A; 
    LS_solver->b() = B; 

    for (unsigned int i = 0; i < nc; ++i)
        LS_solver->A().coeffRef(nv + i, nv + i) += -(sigma + vrho(i));  //  A = [M, Cq'; Cq, -diag(vsigma+vrho) + E ];

    m_timer_convert.stop();
    if (this->verbose) GetLog() << " Time for ConvertToMatrixForm: << " << m_timer_convert.GetTimeSecondsIntermediate() << "s\n";

    m_timer_factorize.start();
                
    LS_solver->SetupCurrent();  // LU decomposition ++++++++++++++++++++++++++++++++++++++

    m_timer_factorize.stop();
    if (this->verbose) GetLog() << " Time for factorize : << " << m_timer_factorize.GetTimeSecondsIntermediate() << "s\n";

    /*
    res_story.r_prim=zeros(1,1);
    res_story.r_dual_Nlry=zeros(1,1);
    res_story.r_dual=zeros(1,1);
    res_story.r_combined_pre=zeros(1,1);
    res_story.r_combined=zeros(1,1);
    res_story.r_deltal=zeros(1,1);
    res_story.r_rho=zeros(1,1);
    res_story.r_violation=zeros(1,1);
    */

    for (int iter = 0; iter < m_max_iterations; iter++) {

        // diagnostic
        l_old = l;
        z_old = z;
        y_old = y;
        //y_hat_old = y_hat;

        // X   (lambda)

        // SOLVE LINEAR SYSTEM HERE
        // ckkt = -bkkt + (vsigma+vrho).*z - y;
        
        ChVectorDynamic<> ckkt = -b + (vsigma + vrho).cwiseProduct(z) - y;
        LS_solver->b() << k, ckkt;         // B = [k;ckkt];
        
        m_timer_solve.start();

        LS_solver->SolveCurrent();                                                      // LU forward/backsolve ++++++++++++++++++++++++++++++++++++++
        
        m_timer_solve.stop();
        if (this->verbose) GetLog() << " Time for solve : << " << m_timer_solve.GetTimeSecondsIntermediate() << "s\n";

        // x = dA\B;      // A* x = B  with x = [v, -l]    
        l = -LS_solver->x().block(nv, 0, nc, 1);
        v =  LS_solver->x().block(0, 0, nv, 1);
        

        // Z

        //    z = project_orthogonal(l + y. / vrho, fric); 
        z = l + y.cwiseQuotient(vrho);
        sysd.ConstraintsProject(z); 

        // Y

        y = y + vrho.asDiagonal() * (l - z);


        // y_hat = y + vrho .* (l - z_old);   


        // Compute residuals for tolerances


        r_prim = ((z - l).cwiseProduct(S)).lpNorm<Eigen::Infinity>();     //r_prim     = norm((z - l).*S, inf);   
        double r_prim_pre = (z - l).lpNorm<Eigen::Infinity>();              //r_prim_pre = norm((z - l)   , inf); 
        //r_dual_Nlry = norm((D'*v-E*l+bkkt+y)./S,inf); % dual residual exploiting the kkt form instead of (N*l+r+y)./S, faster!
        //r_dual_Nlry_pre = norm((D'*v-E*l+bkkt+y)   ,inf); 
        r_dual = (((z - z_old).cwiseProduct(vrho)).cwiseQuotient(S)).lpNorm<Eigen::Infinity>(); // r_dual = norm((vrho.*(z - z_old)). / S, inf); % even faster!See book of Boyd.But coincides only for alpha = 1 !!!
        double r_dual_pre = ((z - z_old).cwiseProduct(vrho)).lpNorm<Eigen::Infinity>();  // r_dual_pre     = norm((vrho.*(z - z_old))   ,inf); 
        //r_combined = norm((z - l).*S, 2) + norm((z - z_old). / S, 2);% combined res.in original metric
        //r_combined_pre = norm((z - l), 2) + norm((z - z_old), 2);% combined res.in precond.metric

        /*
        res_story.r_prim(j) = r_prim;
        res_story.r_dual_Nlry(j) = r_dual_Nlry;
        res_story.r_dual(j) = r_dual;
        res_story.r_combined_pre(j) = r_combined_pre;
        res_story.r_combined(j) = r_combined;
        res_story.r_rho(j) = rho_i;
        res_story.r_deltal(j) = norm((l - l_old).*S, inf);% diagnostic
        */

        // only to compare vs.BB method :
        /*
        testres = (N * l + r). / S;
        resid = norm((testres - project_residual(testres, fric)), inf);
        res_story.r_violation(j) = resid;
        */

        if (this->verbose)
            GetLog() << "ADMM iter=" << iter << " prim=" << r_prim << " dual=" << r_dual << "  rho=" << rho_i << "  tols=" << this->tol_prim << " " << this->tol_dual <<  "\n";

        
        // Termination:
        if ((r_prim < this->tol_prim) && (r_dual < this->tol_dual)) {
            if (this->verbose)
                GetLog() << "ADMM converged ok! at iter=" << iter << "\n";
            break;
        }



        // once in a while update the rho step parameter
        if ((iter % this->stepadjust_each) == 0) {

            double rhofactor = 1; // default do not shrink / enlarge

            if (this->stepadjust_type == AdmmStepType::NONE) {
                rhofactor = 1.;
            }

            if (this->stepadjust_type == AdmmStepType::BALANCED_UNSCALED) {
                rhofactor = sqrt(r_prim_pre / r_dual_pre);
            }

            if (this->stepadjust_type == AdmmStepType::BALANCED_FAST) {
                double r_prim_scaled = r_prim_pre / (ChMax(z.lpNorm<Eigen::Infinity>(), l.lpNorm<Eigen::Infinity>()) + 1e-10); // maybe norm(l, inf) very similar to norm(z, inf)
                double r_dual_scaled = r_dual_pre / (y.lpNorm<Eigen::Infinity>() + 1e-10);  //  as in "ADMM Penalty Parameter Selection by Residual Balancing", Brendt Wohlberg
                rhofactor = sqrt(r_prim_scaled / (r_dual_scaled + 1e-10));
            }

            if (this->stepadjust_type == AdmmStepType::BALANCED_RANGE) {
                double r_prim_scaled = r_prim / this->tol_prim;
                double r_dual_scaled = r_dual / this->tol_dual;
                rhofactor = sqrt(r_prim_scaled / (r_dual_scaled + 1e-10));
            }

            /*
            if (msetup.stepadjust_type == "spectral")
                % from "Adaptive ADMM with spectral penalty parameter
                % selection", Zheng et al.
                Dy_hat = y_hat - y_hat_old;
                Dy = y - y_old;
                DH = l - l_old;
                DG = z - z_old;
                alpha_hat_SD = (Dy_hat' * Dy_hat)/(DH' * Dy_hat);
                alpha_hat_MG = (DH' * Dy_hat)/(DH' * DH);
                beta_hat_SD = (Dy' * Dy)/(DG' * Dy);
                beta_hat_MG = (DG' * Dy)/(DG' * DG);
                alpha_cor = (DH' * Dy_hat)/((norm(DH,2)*norm(Dy_hat,2)));
                    beta_cor = (DG' * Dy    )/((norm(DG,2)*norm(Dy    ,2)));
                        alpha_hat = 0; beta_hat = 0;
                if (2 * alpha_hat_MG > alpha_hat_SD)
                    alpha_hat = alpha_hat_MG;
                else
                    alpha_hat = alpha_hat_SD - alpha_hat_MG / 2;
                end
                    if (2 * beta_hat_MG > beta_hat_SD)
                        beta_hat = beta_hat_MG;
                    else
                        beta_hat = beta_hat_SD - beta_hat_MG / 2;
                end
                    % alpha_hat = alpha_hat_MG;
                % beta_hat = beta_hat_MG;
                % compute optimal step
                    e_cor = 0.02;
                newrho = rho_i;
                if ((alpha_cor > e_cor) && (beta_cor > e_cor))
                    newrho = sqrt(alpha_hat * beta_hat);
                disp(['  ab spectral new rho=', num2str(newrho)]);
                end
                    if ((alpha_cor > e_cor) && (beta_cor <= e_cor))
                        newrho = alpha_hat;
                disp(['  a  spectral new rho=', num2str(newrho)]);
                end
                    if ((alpha_cor <= e_cor) && (beta_cor > e_cor))
                        newrho = beta_hat;
                disp(['  b  spectral new rho=', num2str(newrho)]);
                end
                % newrho = abs(alpha_hat);
                rhofactor = newrho / rho_i;
            end
            */

            // safeguards against extreme shrinking
            if (rhofactor < 1.0 / this->stepadjust_maxfactor) {
                rhofactor = 1.0 / this->stepadjust_maxfactor;
            }
            if (rhofactor > this->stepadjust_maxfactor) {
                rhofactor = this->stepadjust_maxfactor;
            }

            if ((rhofactor > this->stepadjust_threshold) || (rhofactor < 1.0 / this->stepadjust_threshold)) {

                ChTimer<> m_timer_factorize;
                m_timer_factorize.start();

                // Avoid rebuilding all sparse matrix: 
                // A) just remove old rho with -= :
                for (unsigned int i = 0; i < nc; ++i)
                    LS_solver->A().coeffRef(nv + i, nv + i) -= -(sigma + vrho(i));  

                // Update rho
                rho_i = rho_i * rhofactor;

                // vrho(fric == -2) = rho_b; //  special step for bilateral joints
                vrho.setConstant(rho_i);
                int s_c = 0;
                for (unsigned int ic = 0; ic < mconstraints.size(); ic++) {
                    if (mconstraints[ic]->IsActive()) {
                        if (mconstraints[ic]->GetMode()==eChConstraintMode::CONSTRAINT_LOCK)
                            vrho(s_c) = rho_b;
                        s_c++;
                    }
                }

                // UPDATE FACTORIZATION
                //
                //  A = [M, Cq'; Cq, -diag(vsigma+vrho) + E ];
                /*
                sysd.ConvertToMatrixForm(&A,&C);  // A = [M, Cq'; Cq, E ];  .... 
                for (unsigned int i = 0; i < nc; ++i)
                    A.coeffRef(nv + i, nv + i) += -(sigma + vrho(i)); // ... A = [M, Cq'; Cq, -diag(vsigma+vrho) + E ];
                m_engine.compute(A); // LU decomposition ++++++++++++++++++++++++++++++++++++++
                */
                /*
                LS_solver->A().resize(nv + nc, nv + nc); // otherwise conservativeResize in ConvertToMatrixForm() causes error
                sysd.ConvertToMatrixForm(&LS_solver->A(),&LS_solver->b());  // A = [M, Cq'; Cq, E ];
                for (unsigned int i = 0; i < nc; ++i)
                    LS_solver->A().coeffRef(nv + i, nv + i) += -(sigma + vrho(i));  //  A = [M, Cq'; Cq, -diag(vsigma+vrho) + E ];
                */
                // B) add old rho with += :
                for (unsigned int i = 0; i < nc; ++i)
                    LS_solver->A().coeffRef(nv + i, nv + i) += -(sigma + vrho(i));  

                LS_solver->SetupCurrent();  // LU decomposition ++++++++++++++++++++++++++++++++++++++

                m_timer_factorize.stop();
                if (this->verbose) GetLog() << " Time for re-factorize : << " << m_timer_factorize.GetTimeSecondsIntermediate() << "s\n";
            }

        } // end step adjust


    } // end iteration

    l = l.cwiseProduct(S);

    /*
    GetLog() << "Y resulting:  \n";
        for (int k = 0; k < ChMin(y.rows(), 10); ++k)
            GetLog() << "  " << y(k) << "\n";
    */

    sysd.FromVectorToConstraints(l);
    sysd.FromVectorToVariables(v);

    return r_dual;
}

//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////

class my_enum_mappers : public ChSolverADMM {
  public:
    CH_ENUM_MAPPER_BEGIN(AdmmStepType);
    CH_ENUM_VAL(NONE);
    CH_ENUM_VAL(BALANCED_UNSCALED);
    CH_ENUM_VAL(BALANCED_FAST);
    CH_ENUM_VAL(BALANCED_RANGE);
    CH_ENUM_MAPPER_END(AdmmStepType);
};

void ChSolverADMM::ArchiveOUT(ChArchiveOut& marchive) {
    // version number
    marchive.VersionWrite<ChSolverADMM>();
    // serialize parent class
    ChIterativeSolverVI::ArchiveOUT(marchive);
    // serialize all member data:
    marchive << CHNVP(precond);
    marchive << CHNVP(rho);
    marchive << CHNVP(rho_b);
    marchive << CHNVP(sigma);
    marchive << CHNVP(stepadjust_each);
    marchive << CHNVP(stepadjust_threshold);
    marchive << CHNVP(stepadjust_maxfactor);
    marchive << CHNVP(tol_prim);
    marchive << CHNVP(tol_dual);
    my_enum_mappers::AdmmStepType_mapper mmapper;
    marchive << CHNVP(mmapper(this->stepadjust_type), "stepadjust_type");
}

void ChSolverADMM::ArchiveIN(ChArchiveIn& marchive) {
    // version number
    int version = marchive.VersionRead<ChSolverADMM>();
    // deserialize parent class
    ChIterativeSolverVI::ArchiveIN(marchive);
    // stream in all member data:
    marchive >> CHNVP(precond);
    marchive >> CHNVP(rho);
    marchive >> CHNVP(rho_b);
    marchive >> CHNVP(sigma);
    marchive >> CHNVP(stepadjust_each);
    marchive >> CHNVP(stepadjust_threshold);
    marchive >> CHNVP(stepadjust_maxfactor);
    marchive >> CHNVP(tol_prim);
    marchive >> CHNVP(tol_dual);
    my_enum_mappers::AdmmStepType_mapper mmapper;
    marchive >> CHNVP(mmapper(this->stepadjust_type), "stepadjust_type");
}

}  // end namespace chrono
