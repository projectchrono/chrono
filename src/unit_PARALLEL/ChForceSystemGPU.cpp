#include "ChForceSystemGPU.h"

using namespace chrono;

ChForceSystemGPU::ChForceSystemGPU() {
}

void ChForceSystemGPU::Init() {
}

void ChForceSystemGPU::Solve(ChLcpSystemDescriptor& sysd) {
	ChForceSolverGPU mGPUSolver;
	std::vector<ChLcpVariables*>& mvariables = sysd.GetVariablesList();
	uint number_of_bodies = mvariables.size();
	ChLcpVariablesBody* mbodyvar;
	ChBodyGPU* mbody;
	cout << "START" << endl;
	mGPUSolver.spheres.resize(0);
	mGPUSolver.aux.resize(0);
	mGPUSolver.props.resize(0);

	mGPUSolver.num_spheres = 0;
	for (uint i = 0; i < number_of_bodies; i++) {
		mbodyvar = (ChLcpVariablesBody*) mvariables[i];
		mbody = (ChBodyGPU*) mbodyvar->GetUserData();
		if (mbody->interDist.size() > 0) {
			for (uint j = 0; j < mbody->interDist.size(); j++) {
				float4 temp = F4(mbody->GetPos().x, mbody->GetPos().y,
						mbody->GetPos().z, mbody->interDist[j].x);
				int3 temp2 = I3(i, mGPUSolver.num_spheres,
						mbody->interDist[j].y);
				mGPUSolver.spheres.push_back(temp);
				mGPUSolver.aux.push_back(temp2);

				mGPUSolver.max_p.x = max(temp.x, mGPUSolver.max_p.x);
				mGPUSolver.max_p.y = max(temp.y, mGPUSolver.max_p.y);
				mGPUSolver.max_p.z = max(temp.z, mGPUSolver.max_p.z);

				mGPUSolver.min_p.x = min(temp.x, mGPUSolver.min_p.x);
				mGPUSolver.min_p.y = min(temp.y, mGPUSolver.min_p.y);
				mGPUSolver.min_p.z = min(temp.z, mGPUSolver.min_p.z);

				mGPUSolver.max_rad = max(temp.w, mGPUSolver.max_rad);
				temp = F4(mbody->GetMass(), 0, 0, 0);
				mGPUSolver.props.push_back(temp);
				mGPUSolver.num_spheres++;
			}
		}

	}
	if (mGPUSolver.num_spheres != 0) {
		//mGPUSolver.Init();
		mGPUSolver.CD();
		mGPUSolver.ComputeForces();
		for (uint i = 0; i < mGPUSolver.num_contacts; i++) {
			mbodyvar
					= (ChLcpVariablesBody*) mvariables[mGPUSolver.aux[mGPUSolver.contacts[i].x].x];
			mbody = (ChBodyGPU*) mbodyvar->GetUserData();
			float3 force = mGPUSolver.forces[i];
			//mbody->Accumulate_force(ChVector<>(force.x,force.y,force.z),ChVector<>(0,0,0),0);


			mbodyvar
					= (ChLcpVariablesBody*) mvariables[mGPUSolver.aux[mGPUSolver.contacts[i].y].x];
			mbody = (ChBodyGPU*) mbodyvar->GetUserData();
			//mbody->Accumulate_force(ChVector<>(-force.x,-force.y,-force.z),ChVector<>(0,0,0),0);
		}
	}
	cout << "DONE" << endl;
}

