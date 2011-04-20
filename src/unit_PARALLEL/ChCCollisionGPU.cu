#include "ChCCollisionGPU.h"
#include "ChCCollisionGPU.cuh"
using namespace chrono::collision;

ChCCollisionGPU::ChCCollisionGPU(){								//Constructor
	number_of_objects=0;										//Initialize values
	contact_data_gpu=new thrust::device_vector<contactGPU>;			//Create pointer for Contact Data
};
ChCCollisionGPU::~ChCCollisionGPU(){							//Destructor
	Clear();													//Clear all GPU data
	object_data_host.clear();										//Clear CPU Sphere data
}

void ChCCollisionGPU::Clear(){									
	object_data.clear();
	contact_pair.clear();
	generic_counter.clear();
	bin_start_index.clear();
	bin_number.clear();
	aabb_data.clear();
	body_number.clear();
}

void ChCCollisionGPU::TuneCD(int ss, int ee){					//Tune Collision Detection
	
	printf("Start Tuning\n");
	uint max_time=1000000;										//Initialize Running time
	optimal_bin_size=0;											//Optimal Bin size
	for(int i=ss; i<ee; i++){									//Loop through several sizes
		bin_size=floor(max_dimension/float(i+1)*100)/100.0;						//Size based on Max point and iterator
		START_TIMING(start,stop,running_time);					//Start timer
		//COPY_TO_CONST_MEM(bin_size);							//Size of each Bin
		Broadphase();											//Run Broadphase
		STOP_TIMING(start,stop,running_time);					//Stop Timer
		if(max_time>running_time){								//Update running time is smaller
			max_time=running_time;								//Store new time
			optimal_bin_size=bin_size;							//Store optimal bins ize
		}
		printf(" %d %d %f %f\n",number_of_contacts,i,optimal_bin_size,running_time);	//Output
		Clear();												//Clear data
	}
	bin_size=optimal_bin_size;									//Store new bin size
	doTuning=false;													//Tuning complete
}


void ChCCollisionGPU::GetBounds(){
	max_bounding_point=make_float3(-FLT_MAX  ,-FLT_MAX  ,-FLT_MAX  );
	min_bounding_point=make_float3(FLT_MAX  ,FLT_MAX  ,FLT_MAX  );
	for(int i=0; i<number_of_objects; i++){
		object obj=object_data_host[i];

		float4 min,max;
		if(obj.B.w==0){
			ComputeAABBSphere(obj,min,max);
		}
		else if(obj.B.w==1){
			ComputeAABBTriangle(obj,min,max);
		}
		else if(obj.B.w==2){
			ComputeAABBBox(obj,min,max);
		}
		else if(obj.B.w==3){
			ComputeAABBBox(obj,min,max);
		}
		max_bounding_point.x=fmaxf(max_bounding_point.x,max.x);
		max_bounding_point.y=fmaxf(max_bounding_point.y,max.y);
		max_bounding_point.z=fmaxf(max_bounding_point.z,max.z);
		min_bounding_point.x=fminf(min_bounding_point.x,min.x);
		min_bounding_point.y=fminf(min_bounding_point.y,min.y);
		min_bounding_point.z=fminf(min_bounding_point.z,min.z);
	}
}

void ChCCollisionGPU::Run(){						
	GetBounds();
	object_data		=	object_data_host;						//Copy object data
	global_origin=fabs(min_bounding_point);						//Determine Global Origin
	max_dimension=max3(global_origin+fabs(max_bounding_point));	//Determine Max point in space
	
	//if(doTuning){TuneCD(1,100);}								//Tune CD
	bin_size=max_dimension/ceil(sqrtf(number_of_objects/8.0));
	

	Broadphase();												//Run Broadphase
	int ncb=number_of_contacts;
	Narrowphase();												//Run Narrowpahse
	int ncn=number_of_contacts;
	//printf("Broadphase: %d\t Narrowphase: %d\t BinSize:%f \n",ncb,ncn,bin_size);
	Clear();													//Clear Arrays
}