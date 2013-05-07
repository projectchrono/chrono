#include "ChSolverGPU.cuh"
using namespace chrono;

__host__ __device__ void function_Project_single(uint &index, int2 &ids, real *fric, real3 & gamma) {
	int2 body_id = ids;
	real f_tang = sqrt(gamma.y * gamma.y + gamma.z * gamma.z);
	real mu = (fric[body_id.x] == 0 || fric[body_id.y] == 0) ? 0 : (fric[body_id.x] + fric[body_id.y]) * .5;
	if (mu == 0) {
		gamma.x = gamma.x < 0 ? 0 : gamma.x;
		gamma.y = gamma.z = 0;
		return;
	}
	// inside upper cone? keep untouched!
	if (f_tang < (mu * gamma.x)) {
		return;
	}
	// inside lower cone? reset  normal,u,v to zero!
	if ((f_tang) < -(1.0 / mu) * gamma.x || (fabs(gamma.x) < 10e-15)) {
		gamma = R3(0);
		return;
	}
	// remaining case: project orthogonally to generator segment of upper cone
	gamma.x = (f_tang * mu + gamma.x) / (mu * mu + 1);
	real tproj_div_t = (gamma.x * mu) / f_tang;
	gamma.y *= tproj_div_t;
	gamma.z *= tproj_div_t;

}

struct partA_functor: public thrust::binary_function<thrust::tuple<real, real, real, real>, thrust::tuple<real, real, real, real>, thrust::tuple<real, real, real, real> > {
	__host__ __device__
	thrust::tuple<real, real, real, real> operator()(const thrust::tuple<real, real, real, real> &lhs, const thrust::tuple<real, real, real, real> &rhs) const {
		return thrust::tuple<real, real, real, real>(thrust::get < 0 > (lhs) + thrust::get < 0 > (rhs), thrust::get < 1 > (lhs) + thrust::get < 1 > (rhs),
				thrust::get < 2 > (lhs) + thrust::get < 2 > (rhs), thrust::get < 3 > (lhs) + thrust::get < 3 > (rhs));
	}
};

real ChSolverGPU::PART_A(const uint size, custom_vector<bool> & active, custom_vector<int2> & ids,
custom_vector<real> & fric,
custom_vector<real3> & QXYZ, custom_vector<real3> & QUVW,
custom_vector<real> & mx,custom_vector<real> & my,custom_vector<real> & ms,
const custom_vector<real> & b,custom_vector<real> & mg,custom_vector<real> & mg_tmp2,

custom_vector<real3> & JXYZA, custom_vector<real3> & JXYZB,
custom_vector<real3> & JUVWA, custom_vector<real3> & JUVWB,
const real & t_k,const real & L_k,

real & obj1,real& obj2,real& min_val,
custom_vector<real> &obj1_tmp,custom_vector<real> &obj2_tmp,custom_vector<real> &obj3_tmp1,custom_vector<real> &obj3_tmp2

) {

	shurA (my);

#pragma omp parallel for
		for (uint index = 0; index < number_of_contacts; index++) {
			real3 temp = R3(0);
			int2 id_=ids[index];
			uint b1 = id_.x;
			uint b2 = id_.y;

			if (active[b1] != 0) {
				real3 XYZ = QXYZ[b1];
				real3 UVW = QUVW[b1];

				temp.x += dot(XYZ, JXYZA[index+ number_of_contacts * 0]);
				temp.x += dot(UVW, JUVWA[index+ number_of_contacts * 0]);

				temp.y += dot(XYZ, JXYZA[index+ number_of_contacts * 1]);
				temp.y += dot(UVW, JUVWA[index+ number_of_contacts * 1]);

				temp.z += dot(XYZ, JXYZA[index+ number_of_contacts * 2]);
				temp.z += dot(UVW, JUVWA[index+ number_of_contacts * 2]);

			}
			if (active[b2] != 0) {
				real3 XYZ = QXYZ[b2];
				real3 UVW = QUVW[b2];

				temp.x += dot(XYZ, JXYZB[index+ number_of_contacts * 0]);
				temp.x += dot(UVW, JUVWB[index+ number_of_contacts * 0]);

				temp.y += dot(XYZ, JXYZB[index+ number_of_contacts * 1]);
				temp.y += dot(UVW, JUVWB[index+ number_of_contacts * 1]);

				temp.z += dot(XYZ, JXYZB[index+ number_of_contacts * 2]);
				temp.z += dot(UVW, JUVWB[index+ number_of_contacts * 2]);
			}

			temp.x+= my[index + number_of_contacts * 0] * inv_hhpa * compliance;
			temp.y+= my[index + number_of_contacts * 1] * inv_hhpa * compliance;
			temp.z+= my[index + number_of_contacts * 2] * inv_hhpa * compliance;

			real3 _mg_tmp1 = temp; //+ gamma[index] * inv_hhpa * compliance;

		real3 _b,_my,_mx;

		_b.x = b[index+ number_of_contacts * 0];
		_b.y = b[index+ number_of_contacts * 1];
		_b.z = b[index+ number_of_contacts * 2];

		_my.x = my[index+ number_of_contacts * 0];
		_my.y = my[index+ number_of_contacts * 1];
		_my.z = my[index+ number_of_contacts * 2];

		real3 _mg = _mg_tmp1 -_b;
		mg[index+ number_of_contacts * 0] = _mg.x;
		mg[index+ number_of_contacts * 1] = _mg.y;
		mg[index+ number_of_contacts * 2] = _mg.z;

		real3 _obj2_tmp = 0.5 * _mg_tmp1 -_b;

		obj2_tmp[index+ number_of_contacts * 0] = _obj2_tmp.x * _my.x;
		obj2_tmp[index+ number_of_contacts * 1] = _obj2_tmp.y * _my.y;
		obj2_tmp[index+ number_of_contacts * 2] = _obj2_tmp.z * _my.z;

		_mx.x = _my.x + _mg.x * t_k;
		_mx.y = _my.y + _mg.y * t_k;
		_mx.z = _my.z + _mg.z * t_k;

		function_Project_single(index, id_, fric.data(), _mx);

		mx[index+ number_of_contacts * 0] = _mx.x;
		mx[index+ number_of_contacts * 1] = _mx.y;
		mx[index+ number_of_contacts * 2] = _mx.z;

	}

	shurA(mx);
	real lm;
#pragma omp parallel private(lm)
		{
			lm=FLT_MAX;
#pragma omp for
		for (uint index = 0; index < number_of_contacts; index++) {
			real3 temp = R3(0);
			int2 id_=ids[index];
			uint b1 = id_.x;
			uint b2 = id_.y;

			if (active[b1] != 0) {
				real3 XYZ = QXYZ[b1];
				real3 UVW = QUVW[b1];

				temp.x += dot(XYZ, JXYZA[index+ number_of_contacts * 0]);
				temp.x += dot(UVW, JUVWA[index+ number_of_contacts * 0]);

				temp.y += dot(XYZ, JXYZA[index+ number_of_contacts * 1]);
				temp.y += dot(UVW, JUVWA[index+ number_of_contacts * 1]);

				temp.z += dot(XYZ, JXYZA[index+ number_of_contacts * 2]);
				temp.z += dot(UVW, JUVWA[index+ number_of_contacts * 2]);

			}
			if (active[b2] != 0) {
				real3 XYZ = QXYZ[b2];
				real3 UVW = QUVW[b2];

				temp.x += dot(XYZ, JXYZB[index+ number_of_contacts * 0]);
				temp.x += dot(UVW, JUVWB[index+ number_of_contacts * 0]);

				temp.y += dot(XYZ, JXYZB[index+ number_of_contacts * 1]);
				temp.y += dot(UVW, JUVWB[index+ number_of_contacts * 1]);

				temp.z += dot(XYZ, JXYZB[index+ number_of_contacts * 2]);
				temp.z += dot(UVW, JUVWB[index+ number_of_contacts * 2]);
			}
			temp.x+= mx[index + number_of_contacts * 0] * inv_hhpa * compliance;
			temp.y+= mx[index + number_of_contacts * 1] * inv_hhpa * compliance;
			temp.z+= mx[index + number_of_contacts * 2] * inv_hhpa * compliance;

			real3 _mx,_my,_b,_ms, _mg;
			_mx.x = mx[index+ number_of_contacts * 0];
			_mx.y = mx[index+ number_of_contacts * 1];
			_mx.z = mx[index+ number_of_contacts * 2];

			_my.x = my[index+ number_of_contacts * 0];
			_my.y = my[index+ number_of_contacts * 1];
			_my.z = my[index+ number_of_contacts * 2];

			_ms = _mx-_my;

			ms[index+ number_of_contacts * 0] = _ms.x;
			ms[index+ number_of_contacts * 1] = _ms.y;
			ms[index+ number_of_contacts * 2] = _ms.z;

			_b.x = b[index+ number_of_contacts * 0];
			_b.y = b[index+ number_of_contacts * 1];
			_b.z = b[index+ number_of_contacts * 2];

			_mg.x = mg[index+ number_of_contacts * 0];
			_mg.y = mg[index+ number_of_contacts * 1];
			_mg.z = mg[index+ number_of_contacts * 2];

			obj3_tmp1[index+ number_of_contacts * 0] = _mg.x * _ms.x;
			obj3_tmp1[index+ number_of_contacts * 1] = _mg.y * _ms.y;
			obj3_tmp1[index+ number_of_contacts * 2] = _mg.z * _ms.z;

			obj3_tmp2[index+ number_of_contacts * 0] = _ms.x * _ms.x;
			obj3_tmp2[index+ number_of_contacts * 1] = _ms.y * _ms.y;
			obj3_tmp2[index+ number_of_contacts * 2] = _ms.z * _ms.z;

			real3 _mg_tmp = temp;
			real3 _mg_tmp2 = _mg_tmp-_b;

//			mg_tmp2[index+ number_of_contacts * 0] = _mg_tmp2.x;
//			mg_tmp2[index+ number_of_contacts * 1] = _mg_tmp2.y;
//			mg_tmp2[index+ number_of_contacts * 2] = _mg_tmp2.z;

			lm = std::min(lm, _mg_tmp2.x);

			real3 _obj1_tmp = 0.5 * _mg_tmp-_b;

			obj1_tmp[index+ number_of_contacts * 0] = _obj1_tmp.x * _mx.x;
			obj1_tmp[index+ number_of_contacts * 1] = _obj1_tmp.y * _mx.y;
			obj1_tmp[index+ number_of_contacts * 2] = _obj1_tmp.z * _mx.z;

		}

		if ( lm < min_val ) {
#pragma critical
		{
			if ( lm < min_val ) min_val = lm;
		}
	}

}

real _obj1 = 0;
real _obj2 = 0;
real temp1 = 0;
real temp2 = 0;

#pragma omp parallel for  reduction(+:_obj1,_obj2,temp1,temp2)
		for (int i = 0; i < size; i++) {
			_obj1 += obj1_tmp[i];
			_obj2 += obj2_tmp[i];
			temp1+= obj3_tmp1[i];
			temp2+= obj3_tmp2[i];
		}
		obj1 = _obj1;
		obj2 = _obj2;
		temp2 = sqrt(temp2);
//
//thrust::tuple<real,real,real,real> init (0,0,0,0);
//
//thrust::tuple<real,real,real,real> result = thrust::reduce(
//
//		thrust::make_zip_iterator(
//				thrust::make_tuple(
//						obj1_tmp.begin(),
//						obj2_tmp.begin(),
//						obj3_tmp1.begin(),
//						obj3_tmp2.begin()
//				)),
//
//		thrust::make_zip_iterator(thrust::make_tuple(
//						obj1_tmp.end(),
//						obj2_tmp.end(),
//						obj3_tmp1.end(),
//						obj3_tmp2.end()
//				)), init,partA_functor());
//
//obj1 = get<0>(result);
//obj2 = get<1>(result);
//real temp1 = get<2>(result);
//real temp2 = sqrt(get<3>(result));

//		obj1=Thrust_Total(obj1_tmp);
//		obj2=Thrust_Total(obj2_tmp);
//
//		real temp1=Thrust_Total(obj3_tmp1);
//		real temp2=sqrt(Thrust_Total(obj3_tmp2));

		return obj2 + temp1 + 0.5 * L_k * powf(temp2, real(2.0));

	}
struct partB_functor: public thrust::binary_function<thrust::tuple<real, real, real>, thrust::tuple<real, real, real>, thrust::tuple<real, real, real, real> > {
	__host__ __device__
	thrust::tuple<real, real, real> operator()(const thrust::tuple<real, real, real> &lhs, const thrust::tuple<real, real, real> &rhs) const {
		return thrust::tuple<real, real, real>(thrust::get < 0 > (lhs) + thrust::get < 0 > (rhs), thrust::get < 1 > (lhs) + thrust::get < 1 > (rhs), thrust::get < 2 > (lhs) + thrust::get < 2 > (rhs));
	}
};

real ChSolverGPU::PART_B(const uint size, custom_vector<bool> & active, custom_vector<int2> & ids,
custom_vector<real> & fric,
custom_vector<real3> & QXYZ, custom_vector<real3> & QUVW,
custom_vector<real> & mx,custom_vector<real> & my,custom_vector<real> & ms,
const custom_vector<real> & b,custom_vector<real> & mg,custom_vector<real> & mg_tmp2,

custom_vector<real3> & JXYZA, custom_vector<real3> & JXYZB,
custom_vector<real3> & JUVWA, custom_vector<real3> & JUVWB,
const real & t_k,const real & L_k,

real & obj1,real& obj2,real& min_val,
custom_vector<real> &obj1_tmp,custom_vector<real> &obj2_tmp,custom_vector<real> &obj3_tmp1,custom_vector<real> &obj3_tmp2) {

#pragma omp parallel for
		for (uint index = 0; index < number_of_contacts; index++) {
			int2 id_ = ids[index];
			real3 _mx;
			_mx.x = my[index + number_of_contacts * 0] + mg[index + number_of_contacts * 0] * (t_k);
			_mx.y = my[index + number_of_contacts * 1] + mg[index + number_of_contacts * 1] * (t_k);
			_mx.z = my[index + number_of_contacts * 2] + mg[index + number_of_contacts * 2] * (t_k);

			function_Project_single(index, id_, fric.data(), _mx);

			mx[index + number_of_contacts * 0] = _mx.x;
			mx[index + number_of_contacts * 1] = _mx.y;
			mx[index + number_of_contacts * 2] = _mx.z;

		}

		shurA(mx);
		real lm;
#pragma omp parallel private(lm)
		{
			lm=FLT_MAX;
#pragma omp for
		for (uint index = 0; index < number_of_contacts; index++) {
			real3 temp = R3(0);
			int2 id_ = ids[index];
			uint b1 = id_.x;
			uint b2 = id_.y;

			if (active[b1] != 0) {
				real3 XYZ = QXYZ[b1];
				real3 UVW = QUVW[b1];

				temp.x += dot(XYZ, JXYZA[index + number_of_contacts * 0]);
				temp.x += dot(UVW, JUVWA[index + number_of_contacts * 0]);

				temp.y += dot(XYZ, JXYZA[index + number_of_contacts * 1]);
				temp.y += dot(UVW, JUVWA[index + number_of_contacts * 1]);

				temp.z += dot(XYZ, JXYZA[index + number_of_contacts * 2]);
				temp.z += dot(UVW, JUVWA[index + number_of_contacts * 2]);

			}
			if (active[b2] != 0) {
				real3 XYZ = QXYZ[b2];
				real3 UVW = QUVW[b2];

				temp.x += dot(XYZ, JXYZB[index + number_of_contacts * 0]);
				temp.x += dot(UVW, JUVWB[index + number_of_contacts * 0]);

				temp.y += dot(XYZ, JXYZB[index + number_of_contacts * 1]);
				temp.y += dot(UVW, JUVWB[index + number_of_contacts * 1]);

				temp.z += dot(XYZ, JXYZB[index + number_of_contacts * 2]);
				temp.z += dot(UVW, JUVWB[index + number_of_contacts * 2]);
			}

			temp.x+= mx[index + number_of_contacts * 0] * inv_hhpa * compliance;
			temp.y+= mx[index + number_of_contacts * 1] * inv_hhpa * compliance;
			temp.z+= mx[index + number_of_contacts * 2] * inv_hhpa * compliance;

			real3 _b,_mx,_my,_ms,_mg;
			_b.x = b[index + number_of_contacts * 0];
			_b.y = b[index + number_of_contacts * 1];
			_b.z = b[index + number_of_contacts * 2];

			real3 _mg_tmp = temp;
			real3 _mg_tmp2 = _mg_tmp - _b;

//			mg_tmp2[index + number_of_contacts * 0] = _mg_tmp2.x;
//			mg_tmp2[index + number_of_contacts * 1] = _mg_tmp2.y;
//			mg_tmp2[index + number_of_contacts * 2] = _mg_tmp2.z;

		lm = std::min(lm, _mg_tmp2.x);

		real3 _obj1_tmp = 0.5 * _mg_tmp - _b;

		_mx.x = mx[index+ number_of_contacts * 0];
		_mx.y = mx[index+ number_of_contacts * 1];
		_mx.z = mx[index+ number_of_contacts * 2];

		_my.x = my[index+ number_of_contacts * 0];
		_my.y = my[index+ number_of_contacts * 1];
		_my.z = my[index+ number_of_contacts * 2];

		obj1_tmp[index + number_of_contacts * 0] = _obj1_tmp.x * _mx.x;
		obj1_tmp[index + number_of_contacts * 1] = _obj1_tmp.y * _mx.y;
		obj1_tmp[index + number_of_contacts * 2] = _obj1_tmp.z * _mx.z;

		_ms = _mx - _my;

		ms[index + number_of_contacts * 0] = _ms.x;
		ms[index + number_of_contacts * 1] = _ms.y;
		ms[index + number_of_contacts * 2] = _ms.z;

		_mg.x = mg[index+ number_of_contacts * 0];
		_mg.y = mg[index+ number_of_contacts * 1];
		_mg.z = mg[index+ number_of_contacts * 2];

		obj3_tmp1[index+ number_of_contacts * 0] = _mg.x * _ms.x;
		obj3_tmp1[index+ number_of_contacts * 1] = _mg.y * _ms.y;
		obj3_tmp1[index+ number_of_contacts * 2] = _mg.z * _ms.z;

		obj3_tmp2[index+ number_of_contacts * 0] = _ms.x * _ms.x;
		obj3_tmp2[index+ number_of_contacts * 1] = _ms.y * _ms.y;
		obj3_tmp2[index+ number_of_contacts * 2] = _ms.z * _ms.z;

	}

	if ( lm < min_val ) {
#pragma critical
		{
			if ( lm < min_val ) min_val = lm;
		}
	}
}
real _obj1 = 0;

real temp1 = 0;
real temp2 = 0;

#pragma omp parallel for reduction(+:_obj1,temp1,temp2)
		for (int i = 0; i < size; i++) {
			_obj1 += obj1_tmp[i];
			temp1+= obj3_tmp1[i];
			temp2+= obj3_tmp2[i];
		}
		obj1 = _obj1;

		temp2=sqrt(temp2);

//thrust::tuple<real,real,real> init (0,0,0);
//
//thrust::tuple<real,real,real> result = thrust::reduce(
//
//		thrust::make_zip_iterator(
//				thrust::make_tuple(
//						obj1_tmp.begin(),
//						obj3_tmp1.begin(),
//						obj3_tmp2.begin()
//				)),
//
//		thrust::make_zip_iterator(thrust::make_tuple(
//						obj1_tmp.end(),
//						obj3_tmp1.end(),
//						obj3_tmp2.end()
//				)), init,partB_functor());
//
//obj1 = get<0>(result);
//real temp1 = get<1>(result);
//real temp2 = sqrt(get<2>(result));

//		obj1=Thrust_Total(obj1_tmp);
//
//		real temp1=Thrust_Total(obj3_tmp1);
//		real temp2=sqrt(Thrust_Total(obj3_tmp2));

		return obj2 + temp1 + 0.5 * L_k * powf(temp2, real(2.0));
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
		partTwo_device CUDA_KERNEL_DIM(BLOCKS(size), THREADS)(size,
				CASTR1(ms),
				CASTR1(mg),
				CASTR1(obj1_tmp),
				CASTR1(obj2_tmp));

#else
#pragma omp parallel for
		for (uint index = 0; index < size; index++) {
			partTwo_function(index,ms.data(),mg.data(),obj1_tmp.data(),obj2_tmp.data());
		}

#endif

		real temp1=Thrust_Total(obj1_tmp);
		real temp2=sqrt(Thrust_Total(obj2_tmp));
		return obj2 + temp1 + 0.5 * L_k * powf(temp2, real(2.0));

	}

__host__ __device__ void partThree_function(const uint &i, const real* mx, real* ml, const real* mg, const real beta_k1, real* ms, real* my, real* obj1_tmp) {
	real _mx = mx[i];
	real _ms = _mx - ml[i];
	ms[i] = _ms;
	my[i] = _mx + beta_k1 * _ms;
	obj1_tmp[i] = mg[i] * _ms;
	ml[i] = _mx;

}

__global__ void partThree_device(const uint size, const real* mx, real* ml, const real* mg, const real beta_k1, real* ms, real* my, real* obj1_tmp) {

	INIT_CHECK_THREAD_BOUNDED(INDEX1D, size);
	partThree_function(index, mx, ml, mg, beta_k1, ms, my, obj1_tmp);
}

void partThree_host(const uint size, const custom_vector<real> &mx,
custom_vector<real> &ml,
const custom_vector<real> &mg,
const real &beta_k1,
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
#pragma omp parallel for
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
	real min_val = FLT_MAX;
	custom_vector<real> ms(x.size()), mg_tmp2, mb_tmp(x.size()), d01(x.size());
	custom_vector<real> mg_tmp(x.size()), mg_tmp1(x.size());
	custom_vector<real> obj1_tmp(x.size()), obj2_tmp(x.size()),temp(x.size());
	custom_vector<real> obj3_tmp1(x.size()),obj3_tmp2(x.size());
	real lastgoodres=10e30;
	real theta_k=1.0;
	real theta_k1=theta_k;
	real beta_k1=0.0;
	//custom_vector<real> ml = x;
		Project(x);
	//custom_vector<real> ml_candidate = ml;

		custom_vector<real> mg(x.size());
		ShurProduct(x,mg);
		mg=mg- b;// 1)  g = N*l // 2)  g = N*l - b_shur ...

		Thrust_Fill(d01, -1.0);
		d01+=x;
		ShurProduct(d01,temp);
		real L_k = (Norm(d01)==0) ? 1 : Norm(temp) / Norm(d01);
		real t_k = 1.0 / L_k;
		if (verbose) cout << "L_k:" << L_k << " t_k:" << -t_k << "\n";
		custom_vector<real> my = x;
		custom_vector<real> mx = x;

		real obj1=0.0;
		real obj2=0.0;
#ifdef PRINT_DEBUG_GPU
		cout<<"solver_inner"<<endl;
#endif
		for (current_iteration = 0; current_iteration < max_iter; current_iteration++) {
#ifdef PRINT_DEBUG_GPU
		cout<<"iter_start"<<endl;
#endif

		min_val = FLT_MAX;
//		ShurProduct(my,mg_tmp1);
//		//mg = mg_tmp1-b;
//		//SEAXPY(-t_k, mg, my, mx); // mx = my + mg*(t_k);
//
//		Project(mx);
//		ShurProduct(mx,mg_tmp);
//		//mg_tmp2 = mg_tmp-b;
//		////mg_tmp = 0.5*mg_tmp-b;
//
//		//obj1 = Dot(mx, 0.5*mg_tmp-b);
//		//obj2 = Dot(my, 0.5*mg_tmp1-b);
//		//ms = mx - my;

		real obj3 = PART_A(x.size(), gpu_data->device_active_data, temp_bids,
				gpu_data->device_fric_data,
				gpu_data->device_QXYZ_data,gpu_data->device_QUVW_data,
				mx,my,ms,
				b,mg,mg_tmp2,
				gpu_data->device_JXYZA_data, gpu_data->device_JXYZB_data,
				gpu_data->device_JUVWA_data, gpu_data->device_JUVWB_data,
				-t_k,L_k,
				obj1,obj2,min_val,
				obj1_tmp,obj2_tmp,obj3_tmp1,obj3_tmp2

		);

		//
		//
		while (obj1 > obj3) {
			L_k = 2 * L_k;
			t_k = 1.0 / L_k;
//			SEAXPY(-t_k, mg, my, mx); // mx = my + mg*(t_k);
//			Project(mx);
//
//			ShurProduct(mx,mg_tmp);
//			mg_tmp2 = mg_tmp-b;
//			obj1 = Dot(mx, 0.5*mg_tmp-b);
//			ms = mx - my;
//			obj3 = partTwo_host(x.size(),obj2,L_k,ms,mg,obj1_tmp,obj2_tmp);
			min_val = FLT_MAX;
			obj3 = PART_B(x.size(), gpu_data->device_active_data, temp_bids,
					gpu_data->device_fric_data,
					gpu_data->device_QXYZ_data,gpu_data->device_QUVW_data,
					mx,my,ms,
					b,mg,mg_tmp2,
					gpu_data->device_JXYZA_data, gpu_data->device_JXYZB_data,
					gpu_data->device_JUVWA_data, gpu_data->device_JUVWB_data,
					-t_k,L_k,
					obj1,obj2,min_val,
					obj1_tmp,obj2_tmp,obj3_tmp1,obj3_tmp2

			);

#ifdef PRINT_DEBUG_GPU
		cout << "APGD halving stepsize at it " << current_iteration << ", now t_k=" << -t_k << "\n";
#endif
	}
	theta_k1 = (-pow(theta_k, 2) + theta_k * sqrt(pow(theta_k, 2) + 4)) / 2.0;
	beta_k1 = theta_k * (1.0 - theta_k) / (pow(theta_k, 2) + theta_k1);
	real temp_dot_prod=0;

	//ms = mx - ml;
	//SEAXPY(beta_k1,ms,mx,my);//my = mx + beta_k1 * (ms);
	//temp_dot_prod = Dot(mg, ms);
	partThree_host(x.size(),
			mx,
			x,
			mg,
			beta_k1,
			ms,
			my,
			obj1_tmp,
			temp_dot_prod);
	if (temp_dot_prod > 0) {
		my = mx;
		theta_k1 = 1.0;
#ifdef PRINT_DEBUG_GPU
		cout << "Restarting APGD at it " << current_iteration << "\n";
#endif
	}
	L_k = 0.9 * L_k;
	t_k = 1.0 / L_k;

	//x = mx;
	theta_k = theta_k1;

	//this is res1
	real g_proj_norm=fmax(real(0.0),-min_val);
	//CompRes(mg_tmp2,number_of_contacts);
	//cout<<"MINVAL "<<g_proj_norm<<" "<<fmax(real(0.0),-min_val)<<endl;
	//this is res4
	//SEAXPY(-gdiff, mg_tmp2, x, mb_tmp); //mb_tmp=x+mg_tmp2*(-gdiff)
	//Project(mb_tmp);
	//d01=mb_tmp-x;
	//mb_tmp = (-1.0/gdiff)*d01;
	//real g_proj_norm = Norm(mb_tmp);

	if(g_proj_norm < lastgoodres) {
		lastgoodres = g_proj_norm;
		//ml_candidate = ml;
	}

	residual=lastgoodres;

#ifdef PRINT_DEBUG_GPU
		cout<<"iter_end "<<residual<<endl;
#endif
		real maxdeltalambda = NormInf(ms);

		AtIterationEnd(residual, maxdeltalambda, current_iteration);

		if (residual < tolerance) {
			break;
		}
	}
	//x=ml_candidate;
	return current_iteration;
}
