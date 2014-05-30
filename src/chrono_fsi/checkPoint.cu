///////////////////////////////////////////////////////////////////////////////
//	checkPoint.cu
//	Reads the initializes the particles, either from file or inside the code
//	
//	Related Files:
//	Input File:		initializer.txt (optional: if initialize from file)
//					This file contains the sph particles specifications. The description 
//					reads the number of particles first. The each line provides the 
//					properties of one SPH particl: 
//					position(x,y,z), radius, velocity(x,y,z), mass, \rho, pressure, mu, particle_type(rigid or fluid)
//
//	Created by Arman Pazouki
///////////////////////////////////////////////////////////////////////////////

#include <iostream>
#include <fstream>
using namespace std;

//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
void WriteEverythingToFile(
		thrust::host_vector<real3> & mPosRad,
		thrust::host_vector<real4> & mVelMas,
		thrust::host_vector<real4> & mRhoPresMu,
		const thrust::host_vector<uint> & bodyIndex,
		const thrust::host_vector<int3> & referenceArray,

		thrust::host_vector<real3> & posRigidH,
		thrust::host_vector<real4> & mQuatRot,
		thrust::host_vector<real4> & velMassRigidH,
		thrust::host_vector<real3> omegaLRF_H,
		thrust::host_vector<real3> jH1,
		thrust::host_vector<real3> jH2,
		thrust::host_vector<real3> jInvH1,
		thrust::host_vector<real3> jInvH2,

		const thrust::host_vector<real3> & ANCF_Nodes,
		const thrust::host_vector<real3> & ANCF_Slopes,
		const thrust::host_vector<real3> & ANCF_NodesVel,
		const thrust::host_vector<real3> & ANCF_SlopesVel,

		const thrust::host_vector<real_> & ANCF_Beam_Length,
		const thrust::host_vector<bool> & ANCF_IsCantilever,
		const thrust::host_vector<int2> & ANCF_ReferenceArrayNodesOnBeams,
		const thrust::host_vector<real_> & flexParametricDist,

		real_ channelRadius,
		real2 channelCenterYZ,
		SimParams paramsH,
		const ANCF_Params & flexParams,
		const NumberOfObjects & numObjects) {

	//****
	ofstream outMarker;
	outMarker.open("checkPointMarkersData.txt");
	outMarker << "x, y, z, vx, vy, vz, m, rho, p, mu, type, index,\n # \n";
	for (int i=0; i < mPosRad.size(); i++) {
		real3 p = R3(mPosRad[i]);
		real4 vM = mVelMas[i];
		real4 rPMtype = mRhoPresMu[i];
		uint index = bodyIndex[i];
		outMarker << p.x << ", " << p.y << ", " << p.z << ", " << vM.x << ", " << vM.y << ", " << vM.z << ", "
				<< vM.w << ", " << rPMtype.x << ", " << rPMtype.y << ", " << rPMtype.z << ", " << rPMtype.w << ", " << index << ",\n ";

	}
	outMarker.close();
	//****
	ofstream refArray;
	refArray.open("checkPointRefrenceArray.txt");
	refArray << " # \n";
	for (int i=0; i < referenceArray.size(); i++) {
		int3 ref3 = referenceArray[i];
		refArray << ref3.x << ", " << ref3.y << ", " << ref3.z << endl;
	}
	refArray.close();
	//****
	ofstream rigidData;
	rigidData.open("checkPointRigidData.txt");
	rigidData << "x, y, z, q0, q1, q2, q3, Vx, Vy, Vz, mass, om0, om1, om2, j00, j01, j02, j11, j12, j22, invj00, invj01, invj02, invj11, invj12, invj22, \n"
	rigidData << " # \n";
	for (int i=0; i < posRigidH.size(); i++) {
		real3 p = posRigidH[i];
		real4 q = mQuatRot[i];
		real4 vM = velMassRigidH[i];
		real3 om = omegaLRF_H[i];
		real3 j1 = jH1[i];
		real3 j2 = jH2[i];
		real3 invj1 = jInvH1[i];
		real3 invj2 = jInvH2[i];
		rigidData << p.x << ", " << p.y << ", " << p.z << ", " << q.x << ", " << q.y << ", " << q.z << ", " << q.w << ", " <<
				vM.x << ", " << vM.y << ", " << vM.z << ", " << vM.w << ", " << om.x << ", " << om.y << ", " << om.z << ", " <<
				j1.x << ", " << j1.y << ", " << j1.z << ", " << j2.x << ", " << j2.y << ", " << j2.z << ", " <<
				invj1.x << ", " << invj1.y << ", " << invj1.z << ", " << invj2.x << ", " << invj2.y << ", " << invj2.z << ", " <<endl;
	}
	rigidData.close();
	//****
	ofstream flexData;
	flexData.open("checkPointFlexData1.txt");
	flexData << "nx, ny, nz, sx, sy, sz, nVx, nVy, nVz, sVx, sVy, sVz,\n"
	flexData << " # \n";
	for (int i=0; i < ANCF_Nodes.size(); i++) {
		real3 n = ANCF_Nodes[i];
		real3 s = ANCF_Slopes[i];
		real3 nV = ANCF_NodesVel[i];
		real3 sV = ANCF_SlopesVel[i];
		flexData << n.x << ", " << n.y << ", " << n.z << ", " << s.x << ", " << s.y << ", " << s.z << ", " <<
				nV.x << ", " << nV.y << ", " << nV.z << ", " << sV.x << ", " << sV.y << ", " << sV.z << ", " << endl;
	}
	flexData.close();

	ofstream flexData2;
	flexData2.open("checkPointFlexData2.txt");
	flexData2 << "length, isCantilever,\n"
	flexData2 << " # \n";
	for (int i=0; i < ANCF_Beam_Length.size(); i++) {
		flexData2 << ANCF_Beam_Length[i] << ", " << ANCF_IsCantilever[i] << ", " << endl;
	}
	flexData2.close();
	//****
	ofstream flexData;
	flexData.open("checkPointFlexData.txt");
	flexData << "nx, ny, nz, sx, sy, sz, nVx, nVy, nVz, sVx, sVy, sVz,\n"
	flexData << " # \n";
	for (int i=0; i < ANCF_Nodes.size(); i++) {
		real3 n = ANCF_Nodes[i];
		real3 s = ANCF_Slopes[i];
		real3 nV = ANCF_NodesVel[i];
		real3 sV = ANCF_SlopesVel[i];
		flexData << n.x << ", " << n.y << ", " << n.z << ", " << s.x << ", " << s.y << ", " << s.z << ", " <<
				nV.x << ", " << nV.y << ", " << nV.z << ", " << sV.x << ", " << sV.y << ", " << sV.z << ", " << endl;
	}
	flexData.close();
	//****


}
