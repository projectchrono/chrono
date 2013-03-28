#include "ChSolverGPU.h"
using namespace chrono;

uint ChSolverGPU::SolveAPGD(custom_vector<real> &x, const custom_vector<real> &b, const uint max_iter) {
    real gdiff = 0.000001;
    custom_vector<real> zvec(x.size());
    Thrust_Fill(zvec,0.0);
    
    real lastgoodres=10e30;
    real theta_k=1.0;
    real theta_k1=theta_k;
    real beta_k1=0.0;
    custom_vector<real> ms, mg_tmp2, mb_tmp(x.size()), d01(x.size());
    custom_vector<real> mg_tmp(x.size()), mg_tmp1(x.size());
    custom_vector<real> mg = ShurProduct(x) - b;

    Thrust_Fill(d01, -1.0);
    real L_k = Norm(ShurProduct(d01)) / Norm(d01);
    real t_k = 1 / L_k;
    cout << "L_k:" << L_k << " t_k:" << t_k << "\n";
    custom_vector<real>  my = x;
    custom_vector<real>  mx = x;
    custom_vector<real>  ml = x;
    custom_vector<real>  ml_candidate = x;
    
    real obj1=0.0;
    real obj2=0.0;
    
    for (current_iteration = 0; current_iteration < max_iter; current_iteration++) {
    	mg_tmp1 = ShurProduct(my);
        mg = mg_tmp1-b;
        SEAXPY(-t_k, mg, my, mx);   // mx = my + mg*(-t_k);
        Project(mx);
        
        mg_tmp = ShurProduct(mx);
        mg_tmp2 = mg_tmp-b;
        //mg_tmp = 0.5*mg_tmp-b;
        obj1 = Dot(mx, 0.5*mg_tmp-b);
        obj2 = Dot(my, 0.5*mg_tmp1-b);
        ms = mx - my;
        while (obj1 > obj2 + Dot(mg, ms) + 0.5 * L_k * pow(Norm(ms), 2.0)) {
            L_k = 2 * L_k;
            t_k = 1 / L_k;
            SEAXPY(-t_k, mg, my, mx); // mx = my + mg*(-t_k);
            Project(mx);
            
            mg_tmp = ShurProduct(mx);
        	mg_tmp2 = mg_tmp-b;
            obj1 = Dot(mx, 0.5*mg_tmp-b);
            ms = mx - my;

            cout << "APGD halving stepsize at it " << current_iteration << ", now t_k=" << t_k << "\n";
        }
        theta_k1 = (-pow(theta_k, 2) + theta_k * sqrt(pow(theta_k, 2) + 4)) / 2.0;
        beta_k1 = theta_k * (1.0 - theta_k) / (pow(theta_k, 2) + theta_k1);
        ms = mx - ml;
        my = mx + beta_k1 * ms;
        if (Dot(mg, ms) > 0) {
            my = mx;
            theta_k1 = 1.0;
            cout << "Restarting APGD at it " << current_iteration  << "\n";
        }
        L_k = 0.9 * L_k;
        t_k = 1 / L_k;
        
        ml = mx;
        theta_k = theta_k1;
        
       	//this is res1
       	real g_proj_norm=CompRes(mg_tmp2,number_of_contacts);
        
        //this is res4
        //SEAXPY(-gdiff, mg_tmp2, x, mb_tmp); //mb_tmp=x+mg_tmp2*(-gdiff)
        //Project(mb_tmp);
        //d01=mb_tmp-x;
        //mb_tmp = (-1.0/gdiff)*d01;
        //real g_proj_norm = Norm(mb_tmp);
        
        if(g_proj_norm < lastgoodres) {
        	lastgoodres = g_proj_norm;
        	ml_candidate = ml;
        }
        
        residual=lastgoodres;
        if (residual < tolerance) {
            break;
        }
    }
    x=ml_candidate;
    return current_iteration;
}
