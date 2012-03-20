#include "ChCCollisionGPU.h"

#include "ChCCollisionGPU.cuh"
__constant__ uint last_active_bin_const;
__constant__ float3 bin_size_vec_const;
__constant__ uint number_of_models_const;

template<class T>
__device__ __host__ inline uint3 Hash(const T &A) {
	return U3(A.x * bin_size_vec_const.x, A.y * bin_size_vec_const.y, A.z * bin_size_vec_const.z);
}
template<class T>
__device__ __host__ inline uint Hash_Index(const T &A) {
	return ((A.x * 73856093) ^ (A.y * 19349663) ^ (A.z * 83492791));
}
__device__ bool TestAABBAABB(const AABB &A, const AABB &B, uint Bin) {
	return (Bin == Hash_Index(Hash(((A.min + A.max) * .5 + (B.min + B.max) * .5) * .5))) && (A.min.x <= B.max.x && B.min.x <= A.max.x) && (A.min.y <= B.max.y && B.min.y <= A.max.y)
			&& (A.min.z <= B.max.z && B.min.z <= A.max.z);
}
__device__ bool PointInAABB(const AABB &A, const float3 &P) {
	if (P.x > A.min.x && P.x < A.max.x && P.y > A.min.y && P.y < A.max.y && P.z > A.min.z && P.z < A.max.z) {
		return true;
	}
	return false;
}
__device__ __host__ bool AABB_Contact_Pt(const AABB& A, const AABB& B, uint &Bin) {
	float3 Pa, Pb;

	//return (F3(A.min+A.max)*.5+F3(B.min+B.max)*.5)*.5;
//	float3 centerA=(A.min+A.max)*.5;
//	float3 centerB=(B.min+B.max)*.5;
//	float radiusA=length(A.max-A.min)*.5;
//	float radiusB=length(B.max-B.min)*.5;
//	float3 N=normalize(centerB-centerA);
//	Pa=centerA+N*radiusA;
//	Pb=centerB+-N*radiusB;

	bool minX = (A.min.x <= B.min.x && A.max.x >= B.max.x), minY = (A.min.y <= B.min.y && A.max.y >= B.max.y), minZ = (A.min.z <= B.min.z && A.max.z >= B.max.z);

	if (minX && minY && minZ) {
		Pa = (B.min);
		Pb = (B.max);
	} //B inside A

	else if (minY && minZ) {
		if ((A.max.x >= B.min.x && A.max.x <= B.max.x)) {
			Pa = F3(A.max.x, B.max.y, B.max.z);
			Pb = (B.min);
		} //right
		else if ((A.min.x >= B.min.x && A.min.x <= B.max.x)) {
			Pa = F3(A.min.x, B.min.y, B.min.z);
			Pb = (B.max);
		} //left
	} else if (minX && minZ) {
		if ((A.max.y >= B.min.y && A.max.y <= B.max.y)) {
			Pa = F3(B.max.x, A.max.y, B.max.z);
			Pb = (B.min);
		} //top
		else if ((A.min.y >= B.min.y && A.min.y <= B.max.y)) {
			Pa = F3(B.min.x, A.min.y, B.min.z);
			Pb = (B.max);
		} //bottom
	} else if (minY && minX) {
		if ((A.max.z >= B.min.z && A.max.z <= B.max.z)) {
			Pa = F3(B.max.x, B.max.y, A.max.z);
			Pb = (B.min);
		} //front
		else if ((A.min.z >= B.min.z && A.min.z <= B.max.z)) {
			Pa = F3(B.min.x, B.min.y, A.min.z);
			Pb = (B.max);
		} //back
	} else { //corners
		if (A.max.x >= B.min.x || A.max.x <= B.max.x) {
			Pa.x = A.max.x;
			Pb.x = B.min.x;
		} else if (A.min.x <= B.max.x || A.min.x >= B.min.x) {
			Pa.x = A.min.x;
			Pb.x = B.max.x;
		}
		if (A.max.y >= B.min.y || A.max.y <= B.max.y) {
			Pa.y = A.max.y;
			Pb.y = B.min.y;
		} else if (A.min.y <= B.max.y || A.min.y >= B.min.y) {
			Pa.y = A.min.y;
			Pb.y = B.max.y;
		}
		if (A.max.z >= B.min.z || A.max.z <= B.max.z) {
			Pa.z = A.max.z;
			Pb.z = B.min.z;
		} else if (A.min.z <= B.max.z || A.min.z >= B.min.z) {
			Pa.z = A.min.z;
			Pb.z = B.max.z;
		}
	}

	bool inContact = (A.min.x <= B.max.x && B.min.x <= A.max.x) && (A.min.y <= B.max.y && B.min.y <= A.max.y) && (A.min.z <= B.max.z && B.min.z <= A.max.z);
	return (Bin == Hash_Index(Hash((Pa + Pb) * .5f)) && inContact);
}

__device__ int Contact_Type(const int &A, const int &B) {
	if (A == 0 && B == 0) {
		return 0;
	} //sphere-sphere
	if (A == 0 && B == 1) {
		return 1;
	} //sphere-triangle
	if (A == 1 && B == 0) {
		return 2;
	} //triangle-sphere
	if (A == 3 && B == 3) {
		return 3;
	} //ellipsoid-ellipsoid
	return 20;
}
__global__ void AABB_Bins_Count(float3* device_aabb_data, uint* Bins_Intersected) {
	uint index = blockIdx.x * blockDim.x + threadIdx.x;//threadIdx.x + blockDim.x * threadIdx.y + (blockIdx.x * blockDim.x * blockDim.y) + (blockIdx.y * blockDim.x * blockDim.y);
	if (index >= number_of_models_const) {
		return;
	}
	uint3 gmin = Hash(device_aabb_data[index]);
	uint3 gmax = Hash(device_aabb_data[index + number_of_models_const]);
	Bins_Intersected[index] = (gmax.x - gmin.x + 1) * (gmax.y - gmin.y + 1) * (gmax.z - gmin.z + 1);
}
__global__ void AABB_Bins(float3* device_aabb_data, uint* Bins_Intersected, uint * bin_number, uint * body_number) {
	uint index = blockIdx.x * blockDim.x + threadIdx.x;//threadIdx.x + blockDim.x * threadIdx.y + (blockIdx.x * blockDim.x * blockDim.y) + (blockIdx.y * blockDim.x * blockDim.y);
	if (index >= number_of_models_const) {
		return;
	}
	uint count = 0, i, j, k;
	uint3 gmin = Hash(device_aabb_data[index]);
	uint3 gmax = Hash(device_aabb_data[index + number_of_models_const]);

	uint mInd = (index == 0) ? 0 : Bins_Intersected[index - 1];

	for (i = gmin.x; i <= gmax.x; i++) {
		for (j = gmin.y; j <= gmax.y; j++) {
			for (k = gmin.z; k <= gmax.z; k++) {
				bin_number[mInd + count] = Hash_Index(U3(i, j, k));
				body_number[mInd + count] = index;
				count++;
			}
		}
	}

}
__global__ void AABB_AABB_Count(float3* device_aabb_data, int2* device_fam_data, uint * bin_number, uint * body_number, uint * bin_start_index, uint* Num_ContactD) {
	uint index = blockIdx.x * blockDim.x + threadIdx.x;//threadIdx.x + blockDim.x * threadIdx.y + (blockIdx.x * blockDim.x * blockDim.y) + (blockIdx.y * blockDim.x * blockDim.y);
	if (index >= last_active_bin_const) {
		return;
	}
	uint end = bin_start_index[index], count = 0, i = (!index) ? 0 : bin_start_index[index - 1], Bin = bin_number[index];
	AABB A, B;
	if (end - i == 1) {
		Num_ContactD[index] = 0;
		return;
	}
	for (; i < end; i++) {
		A.min = device_aabb_data[body_number[i]];
		A.max = device_aabb_data[body_number[i] + number_of_models_const];
		int2 FA = device_fam_data[body_number[i]];
		for (int k = i + 1; k < end; k++) {
			B.min = device_aabb_data[body_number[k]];
			B.max = device_aabb_data[body_number[k] + number_of_models_const];
			int2 FB = device_fam_data[body_number[k]];
			if ((FA.x == FB.y || FB.x == FA.y) == false && AABB_Contact_Pt(A, B, Bin) == true) {
				count++;
			}
		}
	}
	Num_ContactD[index] = count;
}
__global__ void AABB_AABB(float3* device_aabb_data, int3* device_typ_data, int2* device_fam_data, uint * bin_number, uint * body_number, uint * bin_start_index, uint* Num_ContactD, long long* pair) {
	uint index = blockIdx.x * blockDim.x + threadIdx.x;//threadIdx.x + blockDim.x * threadIdx.y + (blockIdx.x * blockDim.x * blockDim.y) + (blockIdx.y * blockDim.x * blockDim.y);
	if (index >= last_active_bin_const) {
		return;
	}
	uint end = bin_start_index[index], count = 0, i = (!index) ? 0 : bin_start_index[index - 1], Bin = bin_number[index];
	uint offset = (!index) ? 0 : Num_ContactD[index - 1];
	if (end - i == 1) {
		return;
	}

	AABB A, B;
	int3 A_type, B_type;
	for (; i < end; i++) {
		A.min = device_aabb_data[body_number[i]];
		A.max = device_aabb_data[body_number[i] + number_of_models_const];
		A_type = device_typ_data[body_number[i]];
		int2 FA = device_fam_data[body_number[i]];
		for (int k = i + 1; k < end; k++) {
			B.min = device_aabb_data[body_number[k]];
			B.max = device_aabb_data[body_number[k] + number_of_models_const];
			B_type = device_typ_data[body_number[k]];
			int2 FB = device_fam_data[body_number[k]];
			if ((FA.x == FB.y || FB.x == FA.y) == false && AABB_Contact_Pt(A, B, Bin) == true) {
				//int type=Contact_Type(A_aux.x, B_aux.x);
				pair[offset + count] = ((long long) A_type.y << 32 | (long long) B_type.y); //the two indicies of the objects that make up the contact
				count++;
			}
		}
	}
}

__device__ __host__ bool operator ==(const float3 &a, const float3 &b) {
	return ((a.x == b.x) && (a.y == b.y) && (a.z == b.z));
}

void ChCCollisionGPU::Broadphase(gpu_container & gpu_data, bool tune) {
	//START_TIMING(gpu_data.start_b, gpu_data.stop_b, gpu_data.time_Broad_other);
	float3 bin_size_vec = gpu_data.bins_per_axis / gpu_data.bin_size_vec;
	int number_of_models = gpu_data.number_of_models;
	uint last_active_bin = 0, number_of_bin_intersections = 0, number_of_contacts_possible = 0;

	gpu_data.generic_counter.resize(number_of_models);

	COPY_TO_CONST_MEM(bin_size_vec);
	COPY_TO_CONST_MEM(number_of_models);
	//START_TIMING(gpu_data.start_a, gpu_data.stop_a, gpu_data.time_AABB_Bins_Count);

	AABB_Bins_Count CUDA_KERNEL_DIM(BLOCKS(number_of_models),THREADS)(CASTF3(gpu_data.device_aabb_data), CASTU1(gpu_data.generic_counter));
	//STOP_TIMING(gpu_data.start_a, gpu_data.stop_a, gpu_data.time_AABB_Bins_Count);
	Thrust_Inclusive_Scan_Sum(gpu_data.generic_counter, number_of_bin_intersections);

	gpu_data.bin_number_B.resize(number_of_bin_intersections);
	gpu_data.body_number_B.resize(number_of_bin_intersections);
	gpu_data.bin_start_index_B.resize(number_of_bin_intersections);
	//START_TIMING(gpu_data.start_a, gpu_data.stop_a, gpu_data.time_AABB_Bins);
	AABB_Bins CUDA_KERNEL_DIM(BLOCKS(number_of_models),THREADS)(CASTF3(gpu_data.device_aabb_data), CASTU1(gpu_data.generic_counter), CASTU1(gpu_data.bin_number_B), CASTU1(gpu_data.body_number_B));
	//STOP_TIMING(gpu_data.start_a, gpu_data.stop_a, gpu_data.time_AABB_Bins);

	//Thrust_Sort_By_Key(gpu_data.bin_number_B, gpu_data.body_number_B);
	//Thrust_Reduce_By_KeyA(last_active_bin, gpu_data.bin_number_B, gpu_data.bin_start_index_B);


	thrust::host_vector<uint> tempA = gpu_data.bin_number_B;
	thrust::host_vector<uint> tempB = gpu_data.bin_start_index_B;
	thrust::host_vector<uint> tempC = gpu_data.body_number_B;

	Thrust_Sort_By_Key(tempA, tempC);
	Thrust_Reduce_By_KeyA(last_active_bin, tempA, tempB);

	gpu_data.bin_number_B = tempA;
	gpu_data.bin_start_index_B = tempB;
	gpu_data.body_number_B = tempC;



	uint val = gpu_data.bin_start_index_B[thrust::max_element(gpu_data.bin_start_index_B.begin(), gpu_data.bin_start_index_B.begin() + last_active_bin) - gpu_data.bin_start_index_B.begin()];
	gpu_data.bin_start_index_B.resize(last_active_bin);

	Thrust_Inclusive_Scan(gpu_data.bin_start_index_B);

	gpu_data.generic_counter.resize(last_active_bin);

	COPY_TO_CONST_MEM(last_active_bin);
	//START_TIMING(gpu_data.start_a, gpu_data.stop_a, gpu_data.time_AABB_AABB_Count);
	AABB_AABB_Count CUDA_KERNEL_DIM(BLOCKS(last_active_bin),THREADS)(CASTF3(gpu_data.device_aabb_data), CASTI2(gpu_data.device_fam_data), CASTU1(gpu_data.bin_number_B),
			CASTU1(gpu_data.body_number_B), CASTU1(gpu_data.bin_start_index_B), CASTU1(gpu_data.generic_counter));
	//STOP_TIMING(gpu_data.start_a, gpu_data.stop_a, gpu_data.time_AABB_AABB_Count);
	Thrust_Inclusive_Scan_Sum(gpu_data.generic_counter, number_of_contacts_possible);
	gpu_data.device_pair_data.resize(number_of_contacts_possible);
	//START_TIMING(gpu_data.start_a, gpu_data.stop_a, gpu_data.time_AABB_AABB);
	AABB_AABB CUDA_KERNEL_DIM(BLOCKS(last_active_bin),THREADS)(CASTF3(gpu_data.device_aabb_data), CASTI3(gpu_data.device_typ_data), CASTI2(gpu_data.device_fam_data), CASTU1(gpu_data.bin_number_B),
			CASTU1(gpu_data.body_number_B), CASTU1(gpu_data.bin_start_index_B), CASTU1(gpu_data.generic_counter), CASTLL(gpu_data.device_pair_data));
	//STOP_TIMING(gpu_data.start_a, gpu_data.stop_a, gpu_data.time_AABB_AABB);
	gpu_data.number_of_contacts_possible = number_of_contacts_possible;

	//STOP_TIMING(gpu_data.start_b, gpu_data.stop_b, gpu_data.time_Broad_other);
	gpu_data.time_Broad_other = gpu_data.time_Broad_other - gpu_data.time_AABB_Bins_Count - gpu_data.time_AABB_Bins - gpu_data.time_AABB_AABB_Count - gpu_data.time_AABB_AABB;

	gpu_data.number_of_bin_intersections = number_of_bin_intersections;
	gpu_data.number_of_contacts_possible = number_of_contacts_possible;
	gpu_data.last_active_bin = last_active_bin;

	if (val > 100) {
		gpu_data.bins_per_axis = gpu_data.bins_per_axis * 1.1;
	} else if (val < 50&&val>20) {
		gpu_data.bins_per_axis = gpu_data.bins_per_axis * .9;

	}

//	if (gpu_data.bin_start_index_B[last_active_bin - 1] > gpu_data.maxvaltest) {
//		gpu_data.maxvaltest = gpu_data.bin_start_index_B[last_active_bin - 1];
//	}
	cout << val << " " << gpu_data.bins_per_axis.x<<" ";
}
