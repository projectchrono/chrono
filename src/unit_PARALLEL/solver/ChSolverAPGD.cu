#include "ChSolverGPU.cuh"
using namespace chrono;

__host__ __device__ void function_Project_single(uint &index, int2 &ids, real *fric, real *cohesion, real3 & gamma) {
	int2 body_id = ids;
	real coh = (cohesion[body_id.x] + cohesion[body_id.y]) * .5;
	gamma.x += coh;

	real f_tang = sqrt(gamma.y * gamma.y + gamma.z * gamma.z);
	real mu = (fric[body_id.x] == 0 || fric[body_id.y] == 0) ? 0 : (fric[body_id.x] + fric[body_id.y]) * .5;
	if (mu == 0) {
		gamma.x = gamma.x < 0 ? 0 : gamma.x - coh;
		gamma.y = gamma.z = 0;
		return;
	}
	// inside upper cone? keep untouched!
	if (f_tang < (mu * gamma.x)) {
		gamma.x -= coh;
		return;
	}
	// inside lower cone? reset  normal,u,v to zero!
	if ((f_tang) < -(1.0 / mu) * gamma.x || (fabs(gamma.x) < 10e-15)) {
		gamma = R3(-coh, 0, 0);
		return;
	}
//	// remaining case: project orthogonally to generator segment of upper cone
	gamma.x = (f_tang * mu + gamma.x) / (mu * mu + 1);
	real tproj_div_t = (gamma.x * mu) / f_tang;
	gamma.y *= tproj_div_t;
	gamma.z *= tproj_div_t;
//
	gamma.x -= coh;

}

real min_custom(const custom_vector<real>& v)
{
	real shared_min;
#pragma omp parallel
		{
			real min = std::numeric_limits<real>::max();
#pragma omp for nowait
		for(size_t ii=0; ii<v.size(); ++ii)
		{
			min = std::min(v[ii], min);
		}
#pragma omp critical
		{
			shared_min = std::min(shared_min, min);
		}
	}
	return shared_min;
}

real ChSolverGPU::PART_A(const uint size, custom_vector<int2> & ids,
custom_vector<real> & mx,
custom_vector<real> & my,
custom_vector<real> & ms,
const custom_vector<real> & b,
custom_vector<real> & mg,
custom_vector<real> & mg_tmp2,
const real & t_k,
const real & L_k,
real & obj1,
real& obj2,
real& min_val) {

	shurA (my);

#pragma omp parallel for
		for (uint index = 0; index < number_of_contacts; index++) {
			real3 temp = R3(0);
			int2 id_=ids[index];
			uint b1 = id_.x;
			uint b2 = id_.y;

			if (gpu_data->device_active_data[b1] != 0) {
				real3 XYZ = gpu_data->device_QXYZ_data[b1];
				real3 UVW = gpu_data->device_QUVW_data[b1];

				temp.x += dot(XYZ, gpu_data->device_JXYZA_data[index+ number_of_contacts * 0]);
				temp.x += dot(UVW, gpu_data->device_JUVWA_data[index+ number_of_contacts * 0]);

				temp.y += dot(XYZ, gpu_data->device_JXYZA_data[index+ number_of_contacts * 1]);
				temp.y += dot(UVW, gpu_data->device_JUVWA_data[index+ number_of_contacts * 1]);

				temp.z += dot(XYZ, gpu_data->device_JXYZA_data[index+ number_of_contacts * 2]);
				temp.z += dot(UVW, gpu_data->device_JUVWA_data[index+ number_of_contacts * 2]);

			}
			if (gpu_data->device_active_data[b2] != 0) {
				real3 XYZ = gpu_data->device_QXYZ_data[b2];
				real3 UVW = gpu_data->device_QUVW_data[b2];

				temp.x += dot(XYZ, gpu_data->device_JXYZB_data[index+ number_of_contacts * 0]);
				temp.x += dot(UVW, gpu_data->device_JUVWB_data[index+ number_of_contacts * 0]);

				temp.y += dot(XYZ, gpu_data->device_JXYZB_data[index+ number_of_contacts * 1]);
				temp.y += dot(UVW, gpu_data->device_JUVWB_data[index+ number_of_contacts * 1]);

				temp.z += dot(XYZ, gpu_data->device_JXYZB_data[index+ number_of_contacts * 2]);
				temp.z += dot(UVW, gpu_data->device_JUVWB_data[index+ number_of_contacts * 2]);
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

		//_obj2 += dot(_obj2_tmp ,_my);
		obj2_temp[index+ number_of_contacts * 0]=_obj2_tmp.x*_my.x;
		obj2_temp[index+ number_of_contacts * 1]=_obj2_tmp.y*_my.y;
		obj2_temp[index+ number_of_contacts * 2]=_obj2_tmp.z*_my.z;

		_mx.x = _my.x + _mg.x * t_k;
		_mx.y = _my.y + _mg.y * t_k;
		_mx.z = _my.z + _mg.z * t_k;

		function_Project_single(index, id_, gpu_data->device_fric_data.data(), gpu_data->device_cohesion_data.data(), _mx);

		mx[index+ number_of_contacts * 0] = _mx.x;
		mx[index+ number_of_contacts * 1] = _mx.y;
		mx[index+ number_of_contacts * 2] = _mx.z;

	}
	if(number_of_bilaterals>0) {
#pragma omp parallel for
		for (uint index = 0; index < number_of_bilaterals; index++) {
			real temp =0;
			int2 id_=ids[index+ number_of_contacts * 3];
			uint b1 = id_.x;
			uint b2 = id_.y;

			if (gpu_data->device_active_data[b1] != 0) {
				real3 XYZ = gpu_data->device_QXYZ_data[b1];
				real3 UVW = gpu_data->device_QUVW_data[b1];

				temp += dot(XYZ, gpu_data->device_JXYZA_data[index + number_of_contacts * 3]);
				temp += dot(UVW, gpu_data->device_JUVWA_data[index + number_of_contacts * 3]);
			}
			if (gpu_data->device_active_data[b2] != 0) {
				real3 XYZ = gpu_data->device_QXYZ_data[b2];
				real3 UVW = gpu_data->device_QUVW_data[b2];

				temp += dot(XYZ, gpu_data->device_JXYZB_data[index + number_of_contacts * 3]);
				temp += dot(UVW, gpu_data->device_JUVWB_data[index + number_of_contacts * 3]);
			}
			temp+= my[index + number_of_contacts * 3] * inv_hhpa * compliance;
			real _mg_tmp1 = temp; //+ gamma[index] * inv_hhpa * compliance;
			real _b,_my,_mx;
			_b = b[index+ number_of_contacts * 3];
			_my = my[index+ number_of_contacts * 3];
			real _mg = _mg_tmp1 -_b;
			mg[index+ number_of_contacts * 3] = _mg;
			real _obj2_tmp = 0.5 * _mg_tmp1 -_b;
//		_obj2 += _obj2_tmp *_my;
			obj2_temp[index+ number_of_contacts * 3]=_obj2_tmp*_my;
			_mx = _my + _mg * t_k;
			mx[index+ number_of_contacts * 3] = _mx;
		}
	}
	shurA(mx);

#pragma omp parallel for
		for (uint index = 0; index < number_of_contacts; index++) {
			real3 temp = R3(0);
			int2 id_=ids[index];
			uint b1 = id_.x;
			uint b2 = id_.y;

			if (gpu_data->device_active_data[b1] != 0) {
				real3 XYZ = gpu_data->device_QXYZ_data[b1];
				real3 UVW = gpu_data->device_QUVW_data[b1];

				temp.x += dot(XYZ, gpu_data->device_JXYZA_data[index+ number_of_contacts * 0]);
				temp.x += dot(UVW, gpu_data->device_JUVWA_data[index+ number_of_contacts * 0]);

				temp.y += dot(XYZ, gpu_data->device_JXYZA_data[index+ number_of_contacts * 1]);
				temp.y += dot(UVW, gpu_data->device_JUVWA_data[index+ number_of_contacts * 1]);

				temp.z += dot(XYZ, gpu_data->device_JXYZA_data[index+ number_of_contacts * 2]);
				temp.z += dot(UVW, gpu_data->device_JUVWA_data[index+ number_of_contacts * 2]);

			}
			if (gpu_data->device_active_data[b2] != 0) {
				real3 XYZ = gpu_data->device_QXYZ_data[b2];
				real3 UVW = gpu_data->device_QUVW_data[b2];

				temp.x += dot(XYZ, gpu_data->device_JXYZB_data[index+ number_of_contacts * 0]);
				temp.x += dot(UVW, gpu_data->device_JUVWB_data[index+ number_of_contacts * 0]);

				temp.y += dot(XYZ, gpu_data->device_JXYZB_data[index+ number_of_contacts * 1]);
				temp.y += dot(UVW, gpu_data->device_JUVWB_data[index+ number_of_contacts * 1]);

				temp.z += dot(XYZ, gpu_data->device_JXYZB_data[index+ number_of_contacts * 2]);
				temp.z += dot(UVW, gpu_data->device_JUVWB_data[index+ number_of_contacts * 2]);
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

			temp1[index+ number_of_contacts * 0]= _mg.x * _ms.x;
			temp1[index+ number_of_contacts * 1]= _mg.y * _ms.y;
			temp1[index+ number_of_contacts * 2]= _mg.z * _ms.z;

			temp2[index+ number_of_contacts * 0]= _ms.x * _ms.x;
			temp2[index+ number_of_contacts * 1]= _ms.y * _ms.y;
			temp2[index+ number_of_contacts * 2]= _ms.z * _ms.z;

			real3 _mg_tmp = temp;
			real3 _mg_tmp2 = _mg_tmp-_b;

//			mg_tmp2[index+ number_of_contacts * 0] = _mg_tmp2.x;
//			mg_tmp2[index+ number_of_contacts * 1] = _mg_tmp2.y;
//			mg_tmp2[index+ number_of_contacts * 2] = _mg_tmp2.z;

			lm[index] = _mg_tmp2.x;

			real3 _obj1_tmp = 0.5 * _mg_tmp-_b;

			obj1_temp[index+ number_of_contacts * 0]= _obj1_tmp.x * _mx.x;
			obj1_temp[index+ number_of_contacts * 1]= _obj1_tmp.y * _mx.y;
			obj1_temp[index+ number_of_contacts * 2]= _obj1_tmp.z * _mx.z;

		}
		if(number_of_bilaterals>0) {
#pragma omp parallel for
		for (uint index = 0; index < number_of_bilaterals; index++) {
			real temp =0;
			int2 id_=ids[index+ number_of_contacts * 3];
			uint b1 = id_.x;
			uint b2 = id_.y;

			if (gpu_data->device_active_data[b1] != 0) {
				real3 XYZ = gpu_data->device_QXYZ_data[b1];
				real3 UVW = gpu_data->device_QUVW_data[b1];

				temp += dot(XYZ, gpu_data->device_JXYZA_data[index + number_of_contacts * 3]);
				temp += dot(UVW, gpu_data->device_JUVWA_data[index + number_of_contacts * 3]);
			}
			if (gpu_data->device_active_data[b2] != 0) {
				real3 XYZ = gpu_data->device_QXYZ_data[b2];
				real3 UVW = gpu_data->device_QUVW_data[b2];

				temp += dot(XYZ, gpu_data->device_JXYZB_data[index + number_of_contacts * 3]);
				temp += dot(UVW, gpu_data->device_JUVWB_data[index + number_of_contacts * 3]);
			}

			temp+= mx[index + number_of_contacts * 3] * inv_hhpa * compliance;

			real _mx,_my,_b,_ms, _mg;
			_mx = mx[index+ number_of_contacts * 3];
			_my = my[index+ number_of_contacts * 3];
			_ms = _mx-_my;
			ms[index+ number_of_contacts * 3] = _ms;
			_b = b[index+ number_of_contacts * 3];
			_mg = mg[index+ number_of_contacts * 3];

			temp1[index+ number_of_contacts * 3]= _mg * _ms;
			temp2[index+ number_of_contacts * 3]= _ms * _ms;
			real _mg_tmp = temp;
			real _mg_tmp2 = _mg_tmp-_b;

//			mg_tmp2[index+ number_of_contacts * 3] = _mg_tmp2;
			//lm = std::min(lm, _mg_tmp2);
			real _obj1_tmp = 0.5 * _mg_tmp-_b;
			obj1_temp[index+ number_of_contacts * 3]= _obj1_tmp * _mx;
		}
	}
	real _obj1=0, _obj2=0;
	real _temp1 = 0, _temp2=0;
	if(number_of_contacts>0) {
		min_val = min_custom(lm);
	}
#pragma omp parallel for reduction(+:_obj1,_obj2,_temp1,_temp2)
		for(int ii=0; ii< size; ++ii)
		{
			_obj1 += obj1_temp[ii];
			_obj2 += obj2_temp[ii];
			_temp1+=temp1[ii];
			_temp2+=temp2[ii];
		}
		obj1=_obj1;
		obj2=_obj2;
		_temp2 = sqrt(_temp2);

		return obj2 + _temp1 + 0.5 * L_k * powf(_temp2, real(2.0));
	}
real ChSolverGPU::PART_B(const uint size, custom_vector<int2> & ids,
custom_vector<real> & mx,
custom_vector<real> & my,
custom_vector<real> & ms,
const custom_vector<real> & b,
custom_vector<real> & mg,
custom_vector<real> & mg_tmp2,
const real & t_k,
const real & L_k,
real & obj1,
real& obj2,
real& min_val) {

#pragma omp parallel for
		for (uint index = 0; index < number_of_contacts; index++) {
			int2 id_ = ids[index];
			real3 _mx;
			_mx.x = my[index + number_of_contacts * 0] + mg[index + number_of_contacts * 0] * (t_k);
			_mx.y = my[index + number_of_contacts * 1] + mg[index + number_of_contacts * 1] * (t_k);
			_mx.z = my[index + number_of_contacts * 2] + mg[index + number_of_contacts * 2] * (t_k);

			function_Project_single(index, id_, gpu_data->device_fric_data.data(),gpu_data->device_cohesion_data.data(), _mx);

			mx[index + number_of_contacts * 0] = _mx.x;
			mx[index + number_of_contacts * 1] = _mx.y;
			mx[index + number_of_contacts * 2] = _mx.z;
		}
#pragma omp parallel for
		for (uint index = 0; index < number_of_bilaterals; index++) {
			int2 id_ = ids[index+ number_of_contacts * 3];
			real _mx;
			_mx = my[index + number_of_contacts * 3] + mg[index + number_of_contacts * 3] * (t_k);
			//function_Project_single(index, id_, gpu_data->device_fric_data.data(),gpu_data->device_cohesion_data.data(), _mx);
		mx[index + number_of_contacts * 3] = _mx;
	}
	shurA(mx);

#pragma omp parallel for
		for (uint index = 0; index < number_of_contacts; index++) {
			real3 temp = R3(0);
			int2 id_ = ids[index];
			uint b1 = id_.x;
			uint b2 = id_.y;

			if (gpu_data->device_active_data[b1] != 0) {
				real3 XYZ = gpu_data->device_QXYZ_data[b1];
				real3 UVW = gpu_data->device_QUVW_data[b1];

				temp.x += dot(XYZ, gpu_data->device_JXYZA_data[index + number_of_contacts * 0]);
				temp.x += dot(UVW, gpu_data->device_JUVWA_data[index + number_of_contacts * 0]);

				temp.y += dot(XYZ, gpu_data->device_JXYZA_data[index + number_of_contacts * 1]);
				temp.y += dot(UVW, gpu_data->device_JUVWA_data[index + number_of_contacts * 1]);

				temp.z += dot(XYZ, gpu_data->device_JXYZA_data[index + number_of_contacts * 2]);
				temp.z += dot(UVW, gpu_data->device_JUVWA_data[index + number_of_contacts * 2]);

			}
			if (gpu_data->device_active_data[b2] != 0) {
				real3 XYZ = gpu_data->device_QXYZ_data[b2];
				real3 UVW = gpu_data->device_QUVW_data[b2];

				temp.x += dot(XYZ, gpu_data->device_JXYZB_data[index + number_of_contacts * 0]);
				temp.x += dot(UVW, gpu_data->device_JUVWB_data[index + number_of_contacts * 0]);

				temp.y += dot(XYZ, gpu_data->device_JXYZB_data[index + number_of_contacts * 1]);
				temp.y += dot(UVW, gpu_data->device_JUVWB_data[index + number_of_contacts * 1]);

				temp.z += dot(XYZ, gpu_data->device_JXYZB_data[index + number_of_contacts * 2]);
				temp.z += dot(UVW, gpu_data->device_JUVWB_data[index + number_of_contacts * 2]);
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

			lm[index] = _mg_tmp2.x;

			real3 _obj1_tmp = 0.5 * _mg_tmp - _b;

			_mx.x = mx[index+ number_of_contacts * 0];
			_mx.y = mx[index+ number_of_contacts * 1];
			_mx.z = mx[index+ number_of_contacts * 2];

			_my.x = my[index+ number_of_contacts * 0];
			_my.y = my[index+ number_of_contacts * 1];
			_my.z = my[index+ number_of_contacts * 2];

			obj1_temp[index+ number_of_contacts * 0]= _obj1_tmp.x * _mx.x;
			obj1_temp[index+ number_of_contacts * 1]= _obj1_tmp.y * _mx.y;
			obj1_temp[index+ number_of_contacts * 2]= _obj1_tmp.z * _mx.z;

			_ms = _mx - _my;

			ms[index + number_of_contacts * 0] = _ms.x;
			ms[index + number_of_contacts * 1] = _ms.y;
			ms[index + number_of_contacts * 2] = _ms.z;

			_mg.x = mg[index+ number_of_contacts * 0];
			_mg.y = mg[index+ number_of_contacts * 1];
			_mg.z = mg[index+ number_of_contacts * 2];

			temp1[index+ number_of_contacts * 0]= _mg.x * _ms.x;
			temp1[index+ number_of_contacts * 1]= _mg.y * _ms.y;
			temp1[index+ number_of_contacts * 2]= _mg.z * _ms.z;

			temp2[index+ number_of_contacts * 0]= _ms.x * _ms.x;
			temp2[index+ number_of_contacts * 1]= _ms.y * _ms.y;
			temp2[index+ number_of_contacts * 2]= _ms.z * _ms.z;

		}
#pragma omp parallel for
		for (uint index = 0; index < number_of_bilaterals; index++) {
			real temp = 0;
			int2 id_ = ids[index+ number_of_contacts * 3];
			uint b1 = id_.x;
			uint b2 = id_.y;

			if (gpu_data->device_active_data[b1] != 0) {
				real3 XYZ = gpu_data->device_QXYZ_data[b1];
				real3 UVW = gpu_data->device_QUVW_data[b1];

				temp += dot(XYZ, gpu_data->device_JXYZA_data[index + number_of_contacts * 3]);
				temp += dot(UVW, gpu_data->device_JUVWA_data[index + number_of_contacts * 3]);
			}
			if (gpu_data->device_active_data[b2] != 0) {
				real3 XYZ = gpu_data->device_QXYZ_data[b2];
				real3 UVW = gpu_data->device_QUVW_data[b2];

				temp += dot(XYZ, gpu_data->device_JXYZB_data[index + number_of_contacts * 3]);
				temp += dot(UVW, gpu_data->device_JUVWB_data[index + number_of_contacts * 3]);
			}
			temp+= mx[index + number_of_contacts * 3] * inv_hhpa * compliance;

			real _b,_mx,_my,_ms,_mg;
			_b = b[index + number_of_contacts * 3];
			real _mg_tmp = temp;
			real _mg_tmp2 = _mg_tmp - _b;

//			mg_tmp2[index + number_of_contacts * 3] = _mg_tmp2;

			//lm = std::min(lm, _mg_tmp2);

			real _obj1_tmp = 0.5 * _mg_tmp - _b;

			_mx = mx[index+ number_of_contacts * 3];
			_my = my[index+ number_of_contacts * 3];

			obj1_temp[index+ number_of_contacts * 3] = _obj1_tmp * _mx;
			_ms = _mx - _my;
			ms[index + number_of_contacts * 3] = _ms;
			_mg = mg[index+ number_of_contacts * 3];
			temp1[index+ number_of_contacts * 3]= _mg * _ms;
			temp2[index+ number_of_contacts * 3]= _ms * _ms;
		}

		real _obj1=0;
		real _temp1 = 0, _temp2=0;
		if(number_of_contacts>0) {
			min_val = min_custom(lm);
		}
#pragma omp parallel for reduction(+:_obj1,_temp1,_temp2)
		for(int ii=0; ii< size; ++ii)
		{
			_obj1 += obj1_temp[ii];
			_temp1+=temp1[ii];
			_temp2+=temp2[ii];
		}

		obj1 = _obj1;
		_temp2 = sqrt(_temp2);

		return obj2 + _temp1 + 0.5 * L_k * powf(_temp2, real(2.0));
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

void ChSolverGPU::partThree_host(const uint size, const custom_vector<real> &mx,
custom_vector<real> &ml,
const custom_vector<real> &mg,
const real &beta_k1,
custom_vector<real> &ms,
custom_vector<real> &my,
real & temp_dot_prod) {

#ifdef SIM_ENABLE_GPU_MODE
		partThree_device CUDA_KERNEL_DIM(BLOCKS(size), THREADS)(size,
				CASTR1(mx),
				CASTR1(ml),
				CASTR1(mg),
				beta_k1,
				CASTR1(ms),
				CASTR1(my)
				CASTR1(obj1_temp));

#else
#pragma omp parallel for
		for (uint index = 0; index < size; index++) {
			partThree_function(index,mx.data(),ml.data(),mg.data(),beta_k1,ms.data(),my.data(),obj1_temp.data());
		}

#endif
		real _tmp_dot_prod=0;

#pragma omp parallel for reduction(+:_tmp_dot_prod)
		for(int ii=0; ii< size; ++ii) {

			_tmp_dot_prod+= obj1_temp[ii];
		}

		temp_dot_prod=_tmp_dot_prod;
	}

uint ChSolverGPU::SolveAPGD(custom_vector<real> &x, const custom_vector<real> &b, const uint max_iter) {
	bool verbose = false;

	temp1.resize(x.size());
	temp2.resize(x.size());

	obj2_temp.resize(x.size());
	obj1_temp.resize(x.size());
	lm.resize(number_of_contacts);

	real gdiff = 0.000001;
	custom_vector<real> zvec(x.size());
	Thrust_Fill(zvec,0.0);
	real min_val;
	custom_vector<real> ms(x.size()), mg_tmp2, mb_tmp(x.size()), d01(x.size());
	custom_vector<real> mg_tmp(x.size()), mg_tmp1(x.size());
	//<real> obj1_tmp(x.size());
		custom_vector<real> temp(x.size());
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

//		ShurProduct(my,mg_tmp1);
//		mg = mg_tmp1-b;
//		SEAXPY(-t_k, mg, my, mx); // mx = my + mg*(t_k);
//		Project(mx);
//		ShurProduct(mx,mg_tmp);
//		mg_tmp2 = mg_tmp-b;
//		obj1 = Dot(mx, 0.5*mg_tmp-b);
//		obj2 = Dot(my, 0.5*mg_tmp1-b);
//		ms = mx - my;

		real obj3 = PART_A(x.size(), temp_bids,mx,my,ms,b,mg,mg_tmp2,-t_k,L_k,obj1,obj2,min_val);

		while (obj1 > obj3) {
#ifdef PRINT_DEBUG_GPU
		cout <<"L_k "<<L_k<<" t_k "<<t_k<<endl;
#endif
		L_k = 2 * L_k;
		t_k = 1.0 / L_k;
//			SEAXPY(-t_k, mg, my, mx); // mx = my + mg*(t_k);
//			Project(mx);
//
//			ShurProduct(mx,mg_tmp);
//			mg_tmp2 = mg_tmp-b;
//			obj1 = Dot(mx, 0.5*mg_tmp-b);
//			ms = mx - my;
		obj3 = PART_B(x.size(), temp_bids,mx,my,ms,b,mg,mg_tmp2,-t_k,L_k,obj1,obj2,min_val);

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
	//obj1_tmp,
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
	real maxdeltalambda = 0; //NormInf(ms);

#ifdef PRINT_DEBUG_GPU
		cout<<"iter_end "<<residual<<endl;
		cout<<"maxdeltalambda "<<maxdeltalambda<<endl;
#endif

		AtIterationEnd(residual, maxdeltalambda, current_iteration);

		if (residual < tolerance) {
			break;
		}
	}
	//x=ml_candidate;
	return current_iteration;
}

uint ChSolverGPU::SolveAPGD_ALT(custom_vector<real> &x, const custom_vector<real> &b, const uint max_iter) {
	bool verbose = false;
	//cout<<b<<endl;
		real gdiff = 0.000001;

		custom_vector<real> ms(x.size()), mg_tmp2, mb_tmp(x.size());
		custom_vector<real> mg_tmp(x.size()), mg_tmp1(x.size());
	//custom_vector<real> obj1_tmp(x.size());
		real lastgoodres=10e30;
		real theta_k=1.0;
		real theta_k1=theta_k;
		real beta_k1=0.0;
		custom_vector<real> ml = x;
		Project(x);
	//custom_vector<real> ml_candidate = ml;

		custom_vector<real> mg(x.size());
		ShurProduct(x,mg);
		mg=mg- b;// 1)  g = N*l // 2)  g = N*l - b_shur ...

		Thrust_Fill(mb_tmp, -1.0);
		mb_tmp+=x;
		ShurProduct(mb_tmp,mg_tmp);
		//cout<<mg_tmp<<endl;
		real L_k;
		if (Norm(mb_tmp) == 0) {
			L_k = 1;
			std::cout<<"HERE"<<std::endl;
		} else {
			L_k =Norm(mg_tmp) / Norm(mb_tmp);
		}

		// L_k = (Norm(d01)==0) ? 1 : Norm(temp) / Norm(d01);

		real t_k = 1.0 / L_k;
		if (verbose) cout << "L_k:" << L_k << " t_k:" << -t_k << "\n";
		custom_vector<real> my = x;
		custom_vector<real> mx = x;

		real obj1=0.0;
		real obj2=0.0;
		//std::cout<<"tk "<<t_k<<std::endl;

		for (current_iteration = 0; current_iteration < max_iter; current_iteration++) {
			ShurProduct(my,mg_tmp1);
			mg = mg_tmp1-b;
			SEAXPY(-t_k, mg, my, mx); // mx = my + mg*(t_k);
			Project(mx);
			ShurProduct(mx,mg_tmp);
			mg_tmp2 = mg_tmp-b;
			obj1 = Dot(mx, 0.5*mg_tmp-b);
			obj2 = Dot(my, 0.5*mg_tmp1-b);
			ms = mx - my;

			//std::cout<<"OBJ "<<obj1<<" "<<obj2<<std::endl;

			real obj3 = obj2 + Dot(mg, ms) + 0.5 * L_k * powf(Norm(ms), 2.0);

			while (obj1 > obj3) {
				L_k = 2 * L_k;
				t_k = 1.0 / L_k;
				SEAXPY(-t_k, mg, my, mx); // mx = my + mg*(t_k);
				Project(mx);

				ShurProduct(mx,mg_tmp);
				mg_tmp2 = mg_tmp-b;
				obj1 = Dot(mx, 0.5*mg_tmp-b);
				ms = mx - my;
				obj3 = obj2 + Dot(mg, ms) + 0.5 * L_k * powf(Norm(ms), 2.0);

			}
			theta_k1 = (-pow(theta_k, 2) + theta_k * sqrt(pow(theta_k, 2) + 4)) / 2.0;
			beta_k1 = theta_k * (1.0 - theta_k) / (pow(theta_k, 2) + theta_k1);
			real temp_dot_prod=0;

			ms = mx - ml;
			SEAXPY(beta_k1,ms,mx,my); //my = mx + beta_k1 * (ms);
			temp_dot_prod = Dot(mg, ms);

			if (temp_dot_prod > 0) {
				my = mx;
				theta_k1 = 1.0;
			}
			L_k = 0.9 * L_k;
			t_k = 1.0 / L_k;

			x = mx;
			theta_k = theta_k1;

			//this is res1
			//real g_proj_norm=fmax(real(0.0),-min_val);
			real g_proj_norm=CompRes(mg_tmp2,number_of_contacts);
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
			real maxdeltalambda = 0; //NormInf(ms);

			AtIterationEnd(residual, maxdeltalambda, current_iteration);

			if (residual < tolerance) {
				break;
			}
		}
		//cout<<x<<endl;
//x=ml_candidate;
		return current_iteration;
	}

