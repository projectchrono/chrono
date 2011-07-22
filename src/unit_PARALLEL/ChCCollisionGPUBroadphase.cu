#include "ChCCollisionGPU.h"

#include "ChCCollisionGPU.cuh"
#define EPS FLT_EPSILON
template <class T>
__device__ inline uint3 Hash(const T &A){
	return U3(A.x*bin_size_const,A.y*bin_size_const,A.z*bin_size_const);
}
template <class T>
__device__ inline uint Hash_Index(const T &A){
	return ((A.x * 73856093) ^ (A.y * 19349663) ^ (A.z * 83492791));
}

__device__ bool TestAABBAABB(const AABB &A, const AABB &B){
	return 
		(A.min.x<=B.max.x&&B.min.x<=A.max.x) && (A.min.y<=B.max.y&&B.min.y<=A.max.y) &&(A.min.z<=B.max.z&&B.min.z<=A.max.z);
}
__device__ bool PointInAABB(const AABB &A, const float3 &P){
	if(P.x >A.min.x && P.x < A.max.x &&P.y > A.min.y && P.y < A.max.y &&P.z > A.min.z && P.z < A.max.z){return true;}return false;
}
__device__ float3 AABB_Contact_Pt(const AABB& A, const AABB& B){
	float3 Pa,Pb;

	bool 
		minX=(A.min.x<=B.min.x&&A.max.x>=B.max.x),
		minY=(A.min.y<=B.min.y&&A.max.y>=B.max.y),
		minZ=(A.min.z<=B.min.z&&A.max.z>=B.max.z);

	if(minX&&minY&&minZ){Pa=F3(B.min);Pb=F3(B.max);}//B inside A	

	else if(minY && minZ){
		if(		(A.max.x>=B.min.x&&A.max.x<=B.max.x)){Pa=F3(A.max.x,B.max.y,B.max.z); Pb=F3(B.min);}//right
		else if((A.min.x>=B.min.x&&A.min.x<=B.max.x)){Pa=F3(A.min.x,B.min.y,B.min.z); Pb=F3(B.max);}//left
	}
	else if(minX && minZ){
		if(		(A.max.y>=B.min.y&&A.max.y<=B.max.y)){Pa=F3(B.max.x,A.max.y,B.max.z); Pb=F3(B.min);}//top
		else if((A.min.y>=B.min.y&&A.min.y<=B.max.y)){Pa=F3(B.min.x,A.min.y,B.min.z); Pb=F3(B.max);}//bottom
	}
	else if(minY && minX){
		if(		(A.max.z>=B.min.z&&A.max.z<=B.max.z)){Pa=F3(B.max.x,B.max.y,A.max.z); Pb=F3(B.min);}//front
		else if((A.min.z>=B.min.z&&A.min.z<=B.max.z)){Pa=F3(B.min.x,B.min.y,A.min.z); Pb=F3(B.max);}//back
	}
	else{//corners
		if(A.max.x>=B.min.x||A.max.x<=B.max.x)			{Pa.x=A.max.x;	Pb.x=B.min.x;}
		else if(A.min.x<=B.max.x||A.min.x>=B.min.x)		{Pa.x=A.min.x;	Pb.x=B.max.x;}
		if(A.max.y>=B.min.y||A.max.y<=B.max.y)			{Pa.y=A.max.y;	Pb.y=B.min.y;}
		else if(A.min.y<=B.max.y||A.min.y>=B.min.y)		{Pa.y=A.min.y;	Pb.y=B.max.y;}
		if(A.max.z>=B.min.z||A.max.z<=B.max.z)			{Pa.z=A.max.z;	Pb.z=B.min.z;}
		else if(A.min.z<=B.max.z||A.min.z>=B.min.z)		{Pa.z=A.min.z;	Pb.z=B.max.z;}
	}
	return (Pa+Pb)*.5f;
}

__device__ int Contact_Type(const int &A,const int &B){
	//if(A==0&&B==0){return 0;}	//sphere-sphere
	//if(A==0&&B==1){return 1;}	//sphere-triangle
	//if(A==1&&B==0){return 2;}	//triangle-sphere
	//if(A==3&&B==3){return 3;}	//ellipsoid-ellipsoid
	return 20;
}

__global__ void Compute_AABBs(object* object_data, AABB* AABBs){
	uint index = blockIdx.x* blockDim.x + threadIdx.x;
	if(index>=number_of_objects_const){return;}
	object temp_obj=object_data[index];
	AABB temp;

	if(temp_obj.B.w==0){	
		ComputeAABBSphere (temp_obj, temp.min, temp.max);
		temp.max.w=0; 
	}
	else if(temp_obj.B.w==1){
		ComputeAABBTriangle	(temp_obj,temp.min, temp.max);
		temp.max.w=1; 
	}
	else if(temp_obj.B.w==2||temp_obj.B.w==3||temp_obj.B.w==4){
		ComputeAABBBox(temp_obj,temp.min, temp.max);
		temp.max.w=temp_obj.B.w;
	}
	else{return;}
	temp.min=F4(F3(temp.min)-F3(collision_envelope_const)+global_origin_const,index);
	temp.max=F4(F3(temp.max)+F3(collision_envelope_const)+global_origin_const,temp.max.w);
	temp.family=temp_obj.family;
	AABBs[index]=temp;
}

__global__ void AABB_Bins_Count(AABB* AABBs,uint* Bins_Intersected){
	uint index = blockIdx.x* blockDim.x + threadIdx.x;
	if(index>=number_of_objects_const){return;}
	uint count=0, i, j, k;
	uint3 gmin = Hash(AABBs[index].min);
	uint3 gmax = Hash(AABBs[index].max);
	for(i=gmin.x; i<=gmax.x; i++){
		for(j=gmin.y; j<=gmax.y; j++){
			for(k=gmin.z; k<=gmax.z; k++){
				count++;
			}
		}
	}
	Bins_Intersected[index]=count;
}
__global__ void AABB_Bins(AABB* AABBs,uint* Bins_Intersected,uint * Bin_Number, uint * Body_Number){
	uint index = blockIdx.x* blockDim.x + threadIdx.x;
	if(index>=number_of_objects_const){return;}
	uint count=0, i, j, k;
	uint3 gmin = Hash(AABBs[index].min);
	uint3 gmax = Hash(AABBs[index].max);
	for(i=gmin.x; i<=gmax.x; i++){
		for(j=gmin.y; j<=gmax.y; j++){
			for(k=gmin.z; k<=gmax.z; k++){
					uint mInd=(!index) ? count : Bins_Intersected[index-1]+count;
					Bin_Number[mInd]=Hash_Index(U3(i,j,k));
					Body_Number[mInd]=index;
				count++;
			}
		}
	}
}
__global__ void AABB_AABB_Count(
						  AABB* AABBs,
						  uint * Bin_Number,
						  uint * Body_Number ,
						  uint * Bin_Start,
						  uint* Num_ContactD)
{
	uint Index = blockIdx.x* blockDim.x + threadIdx.x;
	if(Index>=last_active_bin_const){return;}
	uint	end=Bin_Start[Index], count=0, i=(!Index) ? 0 : Bin_Start[Index-1], Bin=Bin_Number[Index];
	for(; i<end; i++){			
		for(int k=i+1; k<end; k++){
			AABB A=AABBs[Body_Number[i]];
			AABB B=AABBs[Body_Number[k]];
			if(A.family.x==B.family.y||B.family.x==A.family.y)	{continue;}
			if(TestAABBAABB(A,B)==false)						{continue;}
			if(
				Bin==Hash_Index(Hash(AABB_Contact_Pt(A,B)))||
				Bin==Hash_Index(Hash(AABB_Contact_Pt(B,A)))){
					count++;
			}
		}
	}
	Num_ContactD[Index]=count;
}
__global__ void AABB_AABB(
						  AABB* AABBs,
						  uint * Bin_Number,
						  uint * Body_Number ,
						  uint * Bin_Start,
						  uint* Num_ContactD,
						  int3* contact)
{
	uint Index = blockIdx.x* blockDim.x + threadIdx.x;
	if(Index>=last_active_bin_const){return;}
	uint	end=Bin_Start[Index], count=0, i=(!Index) ? 0 : Bin_Start[Index-1], Bin=Bin_Number[Index];
	for(; i<end; i++){			
		for(int k=i+1; k<end; k++){
			AABB A=AABBs[Body_Number[i]];
			AABB B=AABBs[Body_Number[k]];
			if(A.family.x==B.family.y||B.family.x==A.family.y)	{continue;}
			if(TestAABBAABB(A,B)==false)						{continue;}
			if(
				Bin==Hash_Index(Hash(AABB_Contact_Pt(A,B)))||
				Bin==Hash_Index(Hash(AABB_Contact_Pt(B,A)))){
						int type=Contact_Type(A.max.w,B.max.w);
						uint offset=(!Index) ? 0 : Num_ContactD[Index-1];
						contact[offset+count]=I3(A.min.w,B.min.w,type);			//the two indicies of the objects that make up the contact

					count++;
			}
		}
	}
}
void ChCCollisionGPU::Broadphase(){								//Perform Broadphase CD
	COPY_TO_CONST_MEM(collision_envelope);						//Contact Envelope
	COPY_TO_CONST_MEM(global_origin);							//Origin for Physical Space
	COPY_TO_CONST_MEM(number_of_objects);						//Total Number of objects
	bin_size=1.0/bin_size;										//precompute inverse of bin size so that it can be multiplied
	COPY_TO_CONST_MEM(bin_size);								//Size of each Bin
	generic_counter.resize(number_of_objects);					//Create one counter for each object
	aabb_data.resize(number_of_objects);						//Storage for AABB data						
	Compute_AABBs<<<BLOCKS(number_of_objects),THREADS>>>(		//Compute the AABB for each object
		OBJCAST(object_data),									//object data
		AABBCAST(aabb_data));									//AABB data
	AABB_Bins_Count<<<BLOCKS(number_of_objects),THREADS>>>(			//Count the number of AABB bin intersections
		AABBCAST(aabb_data),									//AABB Data
		CASTU1(generic_counter));								//Number of intersections per AABB
	Thrust_Inclusive_Scan_Sum(generic_counter,number_of_bin_intersections);//Run Scan on generic_counter to get offsets, determine total intersections
	bin_number .resize(number_of_bin_intersections);			//Allocate memory for intersection bin number																
	body_number.resize(number_of_bin_intersections);			//Allocate memory for intersection body number
	bin_start_index.resize(number_of_bin_intersections);		//The Starting index of each bin, assume that each intersection has a different bin
	AABB_Bins<<<BLOCKS(number_of_objects),THREADS>>>(			//Count the number of AABB bin intersections
		AABBCAST(aabb_data),									//AABB Data
		CASTU1(generic_counter),								//Number of intersections per AABB
		CASTU1(bin_number),										//Bin Number for intersections
		CASTU1(body_number));									//Body Number for intersection	
	Thrust_Sort_By_Key(bin_number,body_number);					//Sort the intersection by bin number, bring the Body number along for the ride
	Thrust_Reduce_By_KeyA(last_active_bin,bin_number,bin_start_index);//Determine how many objects are in each bin, store output in BinStart
	Thrust_Inclusive_Scan(bin_start_index);						//Determine Bin Start offsets using inclusive scan
	COPY_TO_CONST_MEM(last_active_bin);							//Copy the Number of the last active bin to constant memory
	generic_counter.resize(last_active_bin);					//Resize the counter, will be reused to count the number of AABB contacts
	AABB_AABB_Count<<<BLOCKS(last_active_bin),THREADS>>>(				//Count the number of AABB-AABB contacts
		AABBCAST(aabb_data),									//AABB Data
		CASTU1(bin_number),										//Bin Number for intersections
		CASTU1(body_number),									//Body Number for intersection
		CASTU1(bin_start_index),								//The Starting index of each bin
		CASTU1(generic_counter));								//Number of AABB intersections									//Indices of bodies that make up AABB contact
																//Count Contacts
	Thrust_Inclusive_Scan_Sum(generic_counter,number_of_contacts);//Run Scan on generic_counter to get offsets, determine total contacts
	contact_pair.resize(number_of_contacts);					//Allocate memory for contact pair
	AABB_AABB<<<BLOCKS(last_active_bin),THREADS>>>(				//Count the number of AABB-AABB contacts
		AABBCAST(aabb_data),									//AABB Data
		CASTU1(bin_number),										//Bin Number for intersections
		CASTU1(body_number),	                         		//Body Number for intersection
		CASTU1(bin_start_index),								//The Starting index of each bin
		CASTU1(generic_counter),								//Number of AABB intersections
		CASTI3(contact_pair)                         			//Indices of bodies that make up AABB contact
		);                                                      //Store Contact 
}
