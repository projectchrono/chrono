#include "ChForceSolverGPU.h"

using namespace chrono;

__constant__ float bin_size_const;
__constant__ int num_spheres_const;
__constant__ float3 global_origin_const;
__constant__ int last_bin_const;
__constant__ int num_contacts_const;
__device__ inline uint3 Hash(const float3 &A) {
	return U3(A.x / bin_size_const, A.y / bin_size_const, A.z / bin_size_const);
}
__device__ inline uint Hash_Index2(const uint3 &A) {
	return (A.x * 73856093) ^ (A.y * 19349663) ^ (A.z * 83492791);
}

__global__ void Bins_Intersect_Sphere_Count(float4* Body_Data,
		uint* Bins_Intersected) {
	uint Index = blockIdx.x * blockDim.x + threadIdx.x;
	if (Index >= num_spheres_const) {
		return;
	}
	float4 BodyD = Body_Data[Index];
	uint count = 0, i, j, k;
	uint3 gmin = Hash(F3(BodyD) - F3(BodyD.w) * 1.001);
	uint3 gmax = Hash(F3(BodyD) + F3(BodyD.w) * 1.001);
	for (i = gmin.x; i <= gmax.x; ++i) {
		for (j = gmin.y; j <= gmax.y; ++j) {
			for (k = gmin.z; k <= gmax.z; ++k) {
				count++;
			}
		}
	}
	Bins_Intersected[Index] = count;
}

__global__ void Bins_Intersect_Sphere_Store(float4* Body_Data,
		uint* Bins_Intersected, uint * Bins_IntersectedK,
		uint * Bins_IntersectedV) {
	uint Index = blockIdx.x * blockDim.x + threadIdx.x;
	if (Index >= num_spheres_const) {
		return;
	}

	float4 BodyD = Body_Data[Index];
	uint count = 0, i, j, k;
	uint3 gmin = Hash(F3(BodyD) - F3(BodyD.w) * 1.001);
	uint3 gmax = Hash(F3(BodyD) + F3(BodyD.w) * 1.001);
	uint mInd = (!Index) ? 0 : Bins_Intersected[Index - 1];
	for (i = gmin.x; i <= gmax.x; ++i) {
		for (j = gmin.y; j <= gmax.y; ++j) {
			for (k = gmin.z; k <= gmax.z; ++k) {
				Bins_IntersectedK[mInd + count] = Hash_Index2(U3(i, j, k));
				Bins_IntersectedV[mInd + count] = Index;
				count++;
			}
		}
	}
}
__global__ void Sphere_Sphere_Count(float4* Body_Data, uint * B_I_DV,
		uint * bin_start_dDK, uint * bin_start_dDV, uint* Num_ContactD) {
	uint Index = blockIdx.x * blockDim.x + threadIdx.x;
	if (Index >= last_bin_const) {
		return;
	}
	uint count = 0;
	uint end = bin_start_dDK[Index];
	uint Bin = bin_start_dDV[Index];
	for (uint i = (!Index) ? 0 : bin_start_dDK[Index - 1]; i < end; i++) {
		uint bidI = B_I_DV[i];
		float4 A = Body_Data[bidI];
		for (uint k = i + 1; k < end; k++) {
			uint bidK = B_I_DV[k];
			float4 B = Body_Data[bidK];
			float3 p = F3(B - A);
			float centerDist = dot(p, p);
			float rAB = B.w + A.w;
			if (centerDist <= rAB * rAB) {
				float3 onB = F3(B) - B.w * p / sqrtf(centerDist);
				if (Bin == Hash_Index2(Hash(onB + global_origin_const))) {
					count++;
				}
			}
		}
	}
	Num_ContactD[Index] = count;
}
__global__ void Sphere_Sphere_Store(float4 *DataD, int3* auxD, uint * B_I_DV,
		uint * bin_start_dDK, uint * bin_start_dDV, uint* Num_ContactD,
		int3* CData) {
	uint Index = blockIdx.x * blockDim.x + threadIdx.x;
	if (Index >= last_bin_const) {
		return;
	}
	uint count = 0;
	uint end = bin_start_dDK[Index];
	uint Bin = bin_start_dDV[Index];
	uint offset = (!Index) ? 0 : Num_ContactD[Index - 1];
	for (uint i = (!Index) ? 0 : bin_start_dDK[Index - 1]; i < end; i++) {
		uint bidI = B_I_DV[i];
		float4 A = DataD[bidI];
		int3 auxI=auxD[bidI];
		for (uint k = i + 1; k < end; k++) {
			uint bidK = B_I_DV[k];
			float4 B = DataD[bidK];
			int3 auxK=auxD[bidK];
			float3 p = F3(B - A);
			float centerDist = dot(p, p);
			float rAB = B.w + A.w;
			if (centerDist <= rAB * rAB) {
				p = p / sqrtf(centerDist);
				float3 onB = F3(B) - B.w * p;
				if (Bin == Hash_Index2(Hash(onB + global_origin_const))) {
					CData[offset+count].x=auxI.y;
					CData[offset+count].y=auxK.y;
					CData[offset+count].z=0;
					count++;
				}
			}
		}
	}
}
ChForceSolverGPU:: ~ChForceSolverGPU(){
		spheres.clear();
		aux.clear();
		forces.clear();
		contacts.clear();
		props.clear();
		spheres.clear();

		spheres_d.clear();
		aux_d.clear();
		forces_d.clear();
		props_d.clear();
		contacts_d.clear();
		intersected_d.clear();
		bin_number_d.clear();
		body_number_d.clear();
		bin_start_d.clear();
	}
void ChForceSolverGPU::CD() {
	cout<<"A"<<endl;
	min_p = fabs(min_p);
	float3 temp = min_p + max_p;
	max_dim = max(max(temp.x, temp.y), temp.z);
	bin_size = max_rad * 2.0;
	global_origin = min_p;
	cout<<"B"<<endl;
	COPY_TO_CONST_MEM(bin_size);
	COPY_TO_CONST_MEM(num_spheres);
	COPY_TO_CONST_MEM(global_origin);
	cout<<"C"<<endl;
	spheres_d = spheres;
	aux_d = aux;
	intersected_d.resize(num_spheres);
	cout<<"D"<<endl;
	Bins_Intersect_Sphere_Count<<<BLOCKS(num_spheres), THREADS>>>(
			CASTF4(spheres_d),
			CASTU1(intersected_d));
	Thrust_Inclusive_Scan(intersected_d);
	uint total_bin_intersections = intersected_d[num_spheres - 1];
	cout<<"E"<<endl;
	bin_number_d.resize(total_bin_intersections);
	body_number_d.resize(total_bin_intersections);
	cout<<"F"<<endl;
	Bins_Intersect_Sphere_Store<<<BLOCKS(num_spheres), THREADS>>>(
			CASTF4(spheres_d),
			CASTU1(intersected_d),
			CASTU1(bin_number_d),
			CASTU1(body_number_d));
	bin_start_d.resize(total_bin_intersections);
	cout<<"G"<<endl;
	Thrust_Sort_By_Key(bin_number_d,body_number_d);
	Thrust_Reduce_By_KeyA(last_bin,bin_number_d,bin_start_d);
	cout<<"H"<<endl;
	COPY_TO_CONST_MEM(last_bin);
	thrust::inclusive_scan(bin_start_d.begin(), bin_start_d.end(),
			bin_start_d.begin());
	intersected_d.resize(last_bin);
	cout<<"I"<<endl;
	Sphere_Sphere_Count<<<BLOCKS2D(last_bin),THREADS>>>(
			CASTF4(spheres_d),
			CASTU1(body_number_d) ,
			CASTU1(bin_start_d),
			CASTU1(bin_number_d),
			CASTU1(intersected_d));
	Thrust_Inclusive_Scan(intersected_d);
	num_contacts = intersected_d[last_bin - 1];
	cout<<"CONTACTS: "<<num_contacts<<endl;
	contacts_d.resize(num_contacts);
	cout<<"J"<<endl;
	Sphere_Sphere_Store<<<BLOCKS2D(last_bin),THREADS>>>(
		CASTF4(spheres_d),
		CASTI3(aux_d),
		CASTU1(body_number_d),
		CASTU1(bin_start_d),
		CASTU1(bin_number_d),
		CASTU1(intersected_d),
		CASTI3(contacts_d));
	contacts=contacts_d;
	cout<<"K"<<endl;
}

__device__ __host__ float3 Force_Adhesive(float3 p1, float3 p2, float q1,
		float q2) {
	float3 N = normalize(p2 - p1);
	return N;
}

__device__ __host__ float3 Force_Cohesive(float3 p1, float3 p2, float q1, float q2) {
	float3 N = normalize(p2 - p1);
	return N;
}

__device__ __host__ float3 Force_ElectoStatic(float3 p1, float3 p2, float q1, float q2) {
	float3 N = normalize(p2 - p1);
	float dist2 = dot(p2 - p1, p2 - p1);
	float f = k_e * q1 * q2 / dist2;
	return N * f;
}

__device__ __host__ float3 Force_Gravitational(float3 p1, float3 p2, float m1,float m2) {
	float3 N = normalize(p2 - p1);
	float dist2 = dot(p2 - p1, p2 - p1);
	float f = G * m1 * m2 / dist2;
	return N ;
}
__global__ void Force_Dispatch(int3 * contacts,float3* forces, float4 * spheres, int3 * aux, float4 * props ){
	uint Index = blockIdx.x * blockDim.x + threadIdx.x;
	if(Index>num_contacts_const){return;}
	int3 contact_temp=contacts[Index];
	float4 s1=spheres[contact_temp.x];
	float4 s2=spheres[contact_temp.y];
	float4 props1=props[contact_temp.x];
	float4 props2=props[contact_temp.y];

	forces[Index]=Force_Gravitational(F3(s1),F3(s2),props1.x,props2.x);



}
void ChForceSolverGPU::ComputeForces() {
	forces_d.resize(num_contacts);
	COPY_TO_CONST_MEM(num_contacts);
	props_d=props;

	Force_Dispatch<<<BLOCKS2D(num_contacts),THREADS>>>(
	CASTI3(contacts_d),
	CASTF3(forces_d),
	CASTF4(spheres_d),
	CASTI3(aux_d),
	CASTF4(props_d)
	);
	forces=forces_d;





}

