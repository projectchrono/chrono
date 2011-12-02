#include "ChCCollisionGPU.h"
class ChApiGPU ChSubdomainGPU {
	public:
		ChSubdomainGPU();

		void Collide();	/// given array of AABBs determine which ones are in this BB
		void Copy();	///based on the models that are in contact 



		float3 min,max;
		vector<int> bod_list, mod_list;

		ChCCollisionGPU *collision_gpu;
};
