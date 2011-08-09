#include "ChForceSystemGPU.h"


namespace chrono{

ChForceSystemGPU::ChForceSystemGPU(){}



void ChForceSystemGPU::Init(){}



void ChForceSystemGPU::Solve(ChLcpSystemDescriptor& sysd){
	std::vector<ChLcpVariables*>&  mvariables	= sysd.GetVariablesList();
	uint number_of_bodies = mvariables.size();

	mGPUSolver.maxP=F3(-FLT_MAX,-FLT_MAX,-FLT_MAX);
	mGPUSolver.minP=F3(FLT_MAX,FLT_MAX,FLT_MAX);

	for (uint i = 0; i< number_of_bodies; i++){
		ChLcpVariablesBody* mbodyvar = (ChLcpVariablesBody*) mvariables[i];
		ChBodyGPU* mbody = (ChBodyGPU*)mbodyvar->GetUserData();
		if(mbody->interDist.size()>0){
			for(uint j=0; j<mbody->interDist.size(); j++){
				float4 temp=F4(mbody->GetPos().x,mbody->GetPos().y,mbody->GetPos().z,mbody->interDist[j].x);
				int2 temp2=I2(i,mbody->interDist[j].y);
				mGPUSolver.spheres.push_back(temp);
				mGPUSolver.aux.push_back(temp2);
				mGPUSolver.maxP.x=max(temp.x,mGPUSolver.maxP.x);
				mGPUSolver.maxP.y=max(temp.y,mGPUSolver.maxP.y);
				mGPUSolver.maxP.z=max(temp.z,mGPUSolver.maxP.z);

				mGPUSolver.minP.x=min(temp.x,mGPUSolver.minP.x);
				mGPUSolver.minP.y=min(temp.y,mGPUSolver.minP.y);
				mGPUSolver.minP.z=min(temp.z,mGPUSolver.minP.z);
			}
		}
	}
	mGPUSolver.Init();
	mGPUSolver.CD();
	mGPUSolver.ComputeForces();
}
}
