#include "ChCCollisionGPU.h"

#include "ChCCollisionGPU.cuh"
__constant__ uint last_active_bin_const;

__device__ bool TestAABBAABB(const AABB &A, const AABB &B, uint Bin) {
	return (Bin == Hash_Index(Hash(((A.min + A.max) * .5 + (B.min + B.max) * .5) * .5))) && (A.min.x <= B.max.x && B.min.x <= A.max.x) && (A.min.y
			<= B.max.y && B.min.y <= A.max.y) && (A.min.z <= B.max.z && B.min.z <= A.max.z);
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

	bool minX = (A.min.x <= B.min.x && A.max.x >= B.max.x), minY = (A.min.y <= B.min.y && A.max.y >= B.max.y), minZ = (A.min.z <= B.min.z && A.max.z
			>= B.max.z);

	if (minX && minY && minZ) {
		Pa = (B.min);
		Pb = (B.max);
	}//B inside A

	else if (minY && minZ) {
		if ((A.max.x >= B.min.x && A.max.x <= B.max.x)) {
			Pa = F3(A.max.x, B.max.y, B.max.z);
			Pb = (B.min);
		}//right
		else if ((A.min.x >= B.min.x && A.min.x <= B.max.x)) {
			Pa = F3(A.min.x, B.min.y, B.min.z);
			Pb = (B.max);
		}//left
	} else if (minX && minZ) {
		if ((A.max.y >= B.min.y && A.max.y <= B.max.y)) {
			Pa = F3(B.max.x, A.max.y, B.max.z);
			Pb = (B.min);
		}//top
		else if ((A.min.y >= B.min.y && A.min.y <= B.max.y)) {
			Pa = F3(B.min.x, A.min.y, B.min.z);
			Pb = (B.max);
		}//bottom
	} else if (minY && minX) {
		if ((A.max.z >= B.min.z && A.max.z <= B.max.z)) {
			Pa = F3(B.max.x, B.max.y, A.max.z);
			Pb = (B.min);
		}//front
		else if ((A.min.z >= B.min.z && A.min.z <= B.max.z)) {
			Pa = F3(B.min.x, B.min.y, A.min.z);
			Pb = (B.max);
		}//back
	} else {//corners
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
	return (Bin == Hash_Index(Hash((Pa + Pb) * .5f))) && (A.min.x <= B.max.x && B.min.x <= A.max.x) && (A.min.y <= B.max.y && B.min.y <= A.max.y)
			&& (A.min.z <= B.max.z && B.min.z <= A.max.z);
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

__global__ void AABB_Bins_Count(float3* AABBs, uint* Bins_Intersected) {
	uint index = blockIdx.x * blockDim.x + threadIdx.x;
	if (index >= number_of_models_const) {
		return;
	}
	uint3 gmin = Hash(AABBs[index]);
	uint3 gmax = Hash(AABBs[index + number_of_models_const]);
	Bins_Intersected[index] = (gmax.x - gmin.x + 1) * (gmax.y - gmin.y + 1) * (gmax.z - gmin.z + 1);//count;
}
__global__ void AABB_Bins(float3* AABBs, uint* Bins_Intersected, uint * Bin_Number, uint * Body_Number) {
	uint index = blockIdx.x * blockDim.x + threadIdx.x;
	if (index >= number_of_models_const) {
		return;
	}
	uint count = 0, i, j, k;
	uint3 gmin = Hash(AABBs[index]);
	uint3 gmax = Hash(AABBs[index + number_of_models_const]);
	uint mInd = (!index) ? count : Bins_Intersected[index - 1];

	for (i = gmin.x; i <= gmax.x; i++) {
		for (j = gmin.y; j <= gmax.y; j++) {
			for (k = gmin.z; k <= gmax.z; k++) {
				Bin_Number[mInd + count] = Hash_Index(U3(i, j, k));
				Body_Number[mInd + count] = index;
				count++;
			}
		}
	}
}
__global__ void AABB_AABB_Count(float3* AABBs, int2* family, uint * bin_number, uint * body_number, uint * Bin_Start, uint* Num_ContactD) {
	uint index = blockIdx.x * blockDim.x + threadIdx.x;
	if (index >= last_active_bin_const) {
		return;
	}
	uint end = Bin_Start[index], count = 0, i = (!index) ? 0 : Bin_Start[index - 1], Bin = bin_number[index];
	AABB A, B;

	for (; i < end; i++) {
		A.min = AABBs[body_number[i]];
		A.max = AABBs[body_number[i] + number_of_models_const];
		int2 FA = family[body_number[i]];
		for (int k = i + 1; k < end; k++) {
			B.min = AABBs[body_number[k]];
			B.max = AABBs[body_number[k] + number_of_models_const];
			int2 FB = family[body_number[k]];
			if ((FA.x == FB.y || FB.x == FA.y) == false && AABB_Contact_Pt(A, B, Bin) == true) {
				count++;
			}
		}
	}
	Num_ContactD[index] = count;
}
__global__ void AABB_AABB(
							float3* AABBs,
							int3* type,
							int2* family,
							uint * bin_number,
							uint * body_number,
							uint * Bin_Start,
							uint* Num_ContactD,
							long long* pair) {
	uint index = blockIdx.x * blockDim.x + threadIdx.x;
	if (index >= last_active_bin_const) {
		return;
	}
	uint end = Bin_Start[index], count = 0, i = (!index) ? 0 : Bin_Start[index - 1], Bin = bin_number[index];
	uint offset = (!index) ? 0 : Num_ContactD[index - 1];
	AABB A, B;
	int3 A_type, B_type;
	for (; i < end; i++) {
		A.min = AABBs[body_number[i]];
		A.max = AABBs[body_number[i] + number_of_models_const];
		A_type = type[body_number[i]];
		int2 FA = family[body_number[i]];
		for (int k = i + 1; k < end; k++) {
			B.min = AABBs[body_number[k]];
			B.max = AABBs[body_number[k] + number_of_models_const];
			B_type = type[body_number[k]];
			int2 FB = family[body_number[k]];
			if ((FA.x == FB.y || FB.x == FA.y) == false && AABB_Contact_Pt(A, B, Bin) == true) {
				//int type=Contact_Type(A_aux.x, B_aux.x);
				pair[offset + count] = ((long long) A_type.y << 32 | (long long) B_type.y);//the two indicies of the objects that make up the contact
				count++;
			}
		}
	}
}
template <class T>
__device__ __host__ inline uint3 Hash_HOST(const T &A, float3 bin_size_vec_const) {
	return U3(A.x*bin_size_vec_const.x,A.y*bin_size_vec_const.y,A.z*bin_size_vec_const.z);
}

void ChCCollisionGPU::Broadphase_HOST(float3 &bin_size_vec, ChGPUDataManager * data_container) {
	uint number_of_models = data_container->number_of_models;
	uint last_active_bin = 0, number_of_bin_intersections = 0;
	thrust::host_vector<uint> bin_number;
	thrust::host_vector<uint> body_number;
	thrust::host_vector<uint> bin_start_index;
	thrust::host_vector<uint> generic_counter;

	for (int index = 0; index < number_of_models; index++) {
		uint count = 0, i, j, k;
		uint3 gmin = Hash_HOST(data_container->host_aabb_data[index],bin_size_vec);
		uint3 gmax = Hash_HOST(data_container->host_aabb_data[index + number_of_models],bin_size_vec);
		for (i = gmin.x; i <= gmax.x; i++) {
			for (j = gmin.y; j <= gmax.y; j++) {
				for (k = gmin.z; k <= gmax.z; k++) {
					bin_number.push_back(Hash_Index(U3(i, j, k)));
					body_number.push_back(index);
					number_of_bin_intersections++;
				}
			}
		}
	}cout<<number_of_bin_intersections<<endl;
	bin_start_index.resize(number_of_bin_intersections);

	Thrust_Sort_By_Key(bin_number,body_number);
	Thrust_Reduce_By_KeyA(last_active_bin,bin_number,bin_start_index);
	Thrust_Inclusive_Scan(bin_start_index);
	data_container->number_of_contacts_possible=0;
	for (int index = 0; index < last_active_bin; index++) {
		uint end = bin_start_index[index], count = 0, i = (!index) ? 0 : bin_start_index[index - 1], Bin = bin_number[index];
		cout<<Bin<<endl;
		AABB A, B;
		int3 A_type, B_type;
		for (; i < end; i++) {
			A.min = data_container->host_aabb_data[body_number[i]];
			A.max = data_container->host_aabb_data[body_number[i] + number_of_models];
			A_type = data_container->host_typ_data[body_number[i]];
			int2 FA = data_container->host_fam_data[body_number[i]];
			for (int k = i + 1; k < end; k++) {
				B.min = data_container->host_aabb_data[body_number[k]];
				B.max = data_container->host_aabb_data[body_number[k] + number_of_models];
				B_type = data_container->host_typ_data[body_number[k]];
				int2 FB = data_container->host_fam_data[body_number[k]];
				if ((FA.x == FB.y || FB.x == FA.y) == false && AABB_Contact_Pt(A, B, Bin) == true) {
					cout<<"here"<<endl;
					data_container->contact_pair.push_back(((long long) A_type.y << 32 | (long long) B_type.y));
					data_container->number_of_contacts_possible++;
				}
			}
		}
	}
}

void ChCCollisionGPU::Broadphase(float3 &bin_size_vec, gpu_container & gpu_data) {
	uint number_of_models = gpu_data.number_of_models;
	uint last_active_bin, number_of_bin_intersections;
	thrust::device_vector<uint> bin_number;
	thrust::device_vector<uint> body_number;
	thrust::device_vector<uint> bin_start_index;
	thrust::device_vector<uint> generic_counter;

	cudaFuncSetCacheConfig(AABB_Bins, cudaFuncCachePreferL1);
	cudaFuncSetCacheConfig(AABB_AABB_Count, cudaFuncCachePreferL1);
	cudaFuncSetCacheConfig(AABB_AABB, cudaFuncCachePreferL1);

	COPY_TO_CONST_MEM(number_of_models);
	COPY_TO_CONST_MEM(bin_size_vec);
	generic_counter.resize(number_of_models);

	AABB_Bins_Count CUDA_KERNEL_DIM(BLOCKS(number_of_models),THREADS)(
			CASTF3(gpu_data.device_aabb_data),
			CASTU1(generic_counter));
	Thrust_Inclusive_Scan_Sum(generic_counter,number_of_bin_intersections);
	bin_number .resize(number_of_bin_intersections);
	body_number.resize(number_of_bin_intersections);
	bin_start_index.resize(number_of_bin_intersections);
	AABB_Bins CUDA_KERNEL_DIM(BLOCKS(number_of_models),THREADS)(
			CASTF3(gpu_data.device_aabb_data),
			CASTU1(generic_counter),
			CASTU1(bin_number),
			CASTU1(body_number));

	Thrust_Sort_By_Key(bin_number,body_number);
	Thrust_Reduce_By_KeyA(last_active_bin,bin_number,bin_start_index);
	Thrust_Inclusive_Scan(bin_start_index);

	COPY_TO_CONST_MEM(last_active_bin);
	generic_counter.resize(last_active_bin);

	AABB_AABB_Count CUDA_KERNEL_DIM(BLOCKS(last_active_bin),THREADS)(
			CASTF3(gpu_data.device_aabb_data),
			CASTI2(gpu_data.device_fam_data),
			CASTU1(bin_number),
			CASTU1(body_number),
			CASTU1(bin_start_index),
			CASTU1(generic_counter));

	Thrust_Inclusive_Scan_Sum(generic_counter,gpu_data.number_of_contacts_possible);

	gpu_data.contact_pair.resize(gpu_data.number_of_contacts_possible);

	AABB_AABB CUDA_KERNEL_DIM(BLOCKS(last_active_bin),THREADS)(
			CASTF3(gpu_data.device_aabb_data),
			CASTI3(gpu_data.device_typ_data),
			CASTI2(gpu_data.device_fam_data),
			CASTU1(bin_number),
			CASTU1(body_number),
			CASTU1(bin_start_index),
			CASTU1(generic_counter),
			CASTLL(gpu_data.contact_pair)
	);
}
