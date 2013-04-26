#include "ChSolverGPU.cuh"
using namespace chrono;

__host__ __device__ void partA_function(const uint &i, const real* b, const real* mg_tmp1, const real* my, const real t_k, real* mg, real* mx) {
	real _mg = mg_tmp1[i] - b[i];

	mg[i] = _mg;
	mx[i] = my[i] + _mg * t_k;
}

__global__ void partA_device(const uint size, const real* b, const real* mg_tmp1, const real* my, const real t_k, real* mg, real* mx) {

	INIT_CHECK_THREAD_BOUNDED(INDEX1D, size);
	partA_function(index, b, mg_tmp1, my, t_k, mg, mx);
}

void partA_host(const uint size, const custom_vector<real> b,
const custom_vector<real> mg_tmp1,
const custom_vector<real> my,
const real t_k,
custom_vector<real> &mg,
custom_vector<real> &mx) {

#ifdef SIM_ENABLE_GPU_MODE
		partA_device CUDA_KERNEL_DIM(BLOCKS(size), THREADS)(size,
				CASTR1(b),
				CASTR1(mg_tmp1),
				CASTR1(my),
				t_k,
				CASTR1(mg),
				CASTR1(mx));

#else
#pragma omp parallel for schedule(guided)
		for (uint index = 0; index < size; index++) {
			partA_function(index,b.data(),mg_tmp1.data(),my.data(),t_k,mg.data(),mx.data());
		}

#endif

	}

__host__ __device__ void partOne_function(const uint &i, const real* b, const real* mx, const real* my, const real* mg_tmp, const real* mg_tmp1, real* mg_tmp2, real* obj1_tmp, real* obj2_tmp,
		real* ms) {
	real _mx = mx[i];
	real _my = my[i];
	ms[i] = _mx - _my;

	real _b = b[i];
	real _mg_tmp = mg_tmp[i];

	real _mg_tmp2 = _mg_tmp - _b;
	mg_tmp2[i] = _mg_tmp2;

	real _obj1_tmp = 0.5 * _mg_tmp - _b;
	obj1_tmp[i] = _obj1_tmp * _mx;

	real _obj2_tmp = 0.5 * mg_tmp1[i] - _b;
	obj2_tmp[i] = _obj2_tmp * _my;

}

__global__ void partOne_device(const uint size, const real* b, const real* mx, const real* my, const real* mg_tmp, const real* mg_tmp1, real* mg_tmp2, real* obj1_tmp, real* obj2_tmp, real* ms) {

	INIT_CHECK_THREAD_BOUNDED(INDEX1D, size);
	partOne_function(index, b, mx, my, mg_tmp, mg_tmp1, mg_tmp2, obj1_tmp, obj2_tmp, ms);
}

void partOne_host(const uint size, const custom_vector<real> b,
const custom_vector<real> mx,
const custom_vector<real> my,
const custom_vector<real> mg_tmp,
const custom_vector<real> mg_tmp1,
custom_vector<real> &mg_tmp2,
custom_vector<real> &obj1_tmp,
custom_vector<real> &obj2_tmp,
real & obj1,real& obj2,
custom_vector<real> &ms) {

#ifdef SIM_ENABLE_GPU_MODE
		partOne_device CUDA_KERNEL_DIM(BLOCKS(size), THREADS)(size,
				CASTR1(b),
				CASTR1(mx),
				CASTR1(my),
				CASTR1(mg_tmp),
				CASTR1(mg_tmp1),
				CASTR1(mg_tmp2),
				CASTR1(obj1_tmp),
				CASTR1(obj2_tmp),
				CASTR1(ms));

#else
#pragma omp parallel for
		for (uint index = 0; index < size; index++) {
			partOne_function(index,b.data(),mx.data(),my.data(),mg_tmp.data(),mg_tmp1.data(),mg_tmp2.data(),obj1_tmp.data(),obj2_tmp.data(),ms.data());
		}

#endif

		obj1=Thrust_Total(obj1_tmp);
		obj2=Thrust_Total(obj2_tmp);
	}

__host__ __device__ void partTwo_function(const uint &i, const real* ms, const real* mg, real* obj1_tmp, real* obj2_tmp) {
	real _mg = mg[i];
	real _ms = ms[i];

	obj1_tmp[i] = _mg * _ms;
	obj2_tmp[i] = _ms * _ms;
}
__global__ void partTwo_device(const uint size, const real* ms, const real* mg, real* obj1_tmp, real* obj2_tmp) {

	INIT_CHECK_THREAD_BOUNDED(INDEX1D, size);
	partTwo_function(index, ms, mg, obj1_tmp, obj2_tmp);
}

real partTwo_host(const uint size, const real &obj2, const real &L_k, const custom_vector<real> ms,
const custom_vector<real> mg,
custom_vector<real> &obj1_tmp,
custom_vector<real> &obj2_tmp) {

#ifdef SIM_ENABLE_GPU_MODE
		partOne_device CUDA_KERNEL_DIM(BLOCKS(size), THREADS)(size,
				CASTR1(ms),
				CASTR1(mg),
				CASTR1(obj1_tmp),
				CASTR1(obj2_tmp));

#else
#pragma omp parallel for schedule(guided)
		for (uint index = 0; index < size; index++) {
			partTwo_function(index,ms.data(),mg.data(),obj1_tmp.data(),obj2_tmp.data());
		}

#endif

		real temp1=Thrust_Total(obj1_tmp);
		real temp2=sqrt(Thrust_Total(obj2_tmp));
		return obj2 + temp1 + 0.5 * L_k * powf(temp2, real(2.0));

	}

__host__ __device__ void partThree_function(const uint &i, const real* mx, const real* ml, const real* mg, const real beta_k1, real* ms, real* my, real* obj1_tmp) {
	real _mx = mx[i];
	real _ms = _mx - ml[i];
	ms[i] = _ms;
	my[i] = _mx + beta_k1 * _ms;
	obj1_tmp[i] = mg[i] * _ms;

}

__global__ void partThree_device(const uint size, const real* mx, const real* ml, const real* mg, const real beta_k1, real* ms, real* my, real* obj1_tmp) {

	INIT_CHECK_THREAD_BOUNDED(INDEX1D, size);
	partThree_function(index, mx, ml, mg, beta_k1, ms, my, obj1_tmp);
}

void partThree_host(const uint size, const custom_vector<real> mx,
const custom_vector<real> ml,
const custom_vector<real> mg,
const real beta_k1,
custom_vector<real> &ms,
custom_vector<real> &my,
custom_vector<real> &obj1_tmp,
real & temp_dot_prod) {

#ifdef SIM_ENABLE_GPU_MODE
		partThree_device CUDA_KERNEL_DIM(BLOCKS(size), THREADS)(size,
				CASTR1(mx),
				CASTR1(ml),
				CASTR1(mg),
				beta_k1,
				CASTR1(ms),
				CASTR1(my),
				CASTR1(obj1_tmp));

#else
#pragma omp parallel for schedule(guided)
		for (uint index = 0; index < size; index++) {
			partThree_function(index,mx.data(),ml.data(),mg.data(),beta_k1,ms.data(),my.data(),obj1_tmp.data());
		}

#endif

		temp_dot_prod=Thrust_Total(obj1_tmp);
	}

uint ChSolverGPU::SolveAPGD(custom_vector<real> &x, const custom_vector<real> &b, const uint max_iter) {
	bool verbose = false;

	real gdiff = 0.000001;
	custom_vector<real> zvec(x.size());
	Thrust_Fill(zvec,0.0);

	custom_vector<real> ms(x.size()), mg_tmp2(x.size()), mb_tmp(x.size()), d01(x.size());
	custom_vector<real> mg_tmp(x.size()), mg_tmp1(x.size());
	custom_vector<real> obj1_tmp(x.size()), obj2_tmp(x.size()),temp(x.size());
	real lastgoodres=10e30;
	real theta_k=1.0;
	real theta_k1=theta_k;
	real beta_k1=0.0;
	custom_vector<real> ml = x;
	Project(ml);
	custom_vector<real> ml_candidate = ml;

	custom_vector<real> mg(x.size());
	ShurProduct(x,mg);
	mg=mg- b; // 1)  g = N*l // 2)  g = N*l - b_shur ...

		Thrust_Fill(d01, -1.0);
		d01+=ml;
		ShurProduct(d01,temp);
		real L_k = (Norm(d01)==0) ? 1 : Norm(temp) / Norm(d01);
		real t_k = 1.0 / L_k;
		if (verbose) cout << "L_k:" << L_k << " t_k:" << -t_k << "\n";
		custom_vector<real> my = x;
		custom_vector<real> mx = x;

		real obj1=0.0;
		real obj2=0.0;

		for (current_iteration = 0; current_iteration < max_iter; current_iteration++) {
			ShurProduct(my,mg_tmp1);
			//mg = mg_tmp1-b;
			//SEAXPY(-t_k, mg, my, mx); // mx = my + mg*(t_k);
			partA_host(x.size(),b,mg_tmp1,my,-t_k,mg,mx);

			Project(mx);
			//mg_tmp = ShurProduct(mx);
			//mg_tmp2 = mg_tmp-b;
			//mg_tmp = 0.5*mg_tmp-b;

			//obj1 = Dot(mx, 0.5*mg_tmp-b);
			//obj2 = Dot(my, 0.5*mg_tmp1-b);
			//ms = mx - my;
			partOne_host(x.size(),b,mx,my,mg_tmp,mg_tmp1,mg_tmp2,obj1_tmp,obj2_tmp,obj1,obj2,ms);

			//
			//obj2 + Dot(mg, ms) + 0.5 * L_k * pow(Norm(ms), real(2.0))
			while (obj1 > partTwo_host(x.size(),obj2,L_k,ms,mg,obj1_tmp,obj2_tmp)) {
				L_k = 2 * L_k;
				t_k = 1.0 / L_k;
				SEAXPY(-t_k, mg, my, mx); // mx = my + mg*(t_k);
				Project(mx);

				ShurProduct(mx,mg_tmp);
				mg_tmp2 = mg_tmp-b;
				obj1 = Dot(mx, 0.5*mg_tmp-b);
				ms = mx - my;

				if (verbose) cout << "APGD halving stepsize at it " << current_iteration << ", now t_k=" << -t_k << "\n";
			}
			theta_k1 = (-pow(theta_k, 2) + theta_k * sqrt(pow(theta_k, 2) + 4)) / 2.0;
			beta_k1 = theta_k * (1.0 - theta_k) / (pow(theta_k, 2) + theta_k1);
			real temp_dot_prod=0;
			partThree_host(x.size(),
					mx,
					ml,
					mg,
					beta_k1,
					ms,
					my,
					obj1_tmp,
					temp_dot_prod);

			//ms = mx - ml;
			//SEAXPY(beta_k1,ms,mx,my);//my = mx + beta_k1 * (ms);
			//temp_dot_prod = Dot(mg, ms);

			if (temp_dot_prod > 0) {
				my = mx;
				theta_k1 = 1.0;
				if (verbose) cout << "Restarting APGD at it " << current_iteration << "\n";
			}
			L_k = 0.9 * L_k;
			t_k = 1.0 / L_k;

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
