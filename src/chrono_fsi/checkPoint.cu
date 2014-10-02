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
#include <string>
#include <sstream>
#include <cstdlib> //for atof
#include "custom_cutil_math.h"
#include "SPHCudaUtils.h"
#include "collideSphereSphere.cuh"
#include <thrust/host_vector.h>
#include "checkPoint.cuh"

using namespace std;

//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
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
		const NumberOfObjects & numObjects,
		real_ time) {

	//*******************************************************************
	ofstream outMarker;
	outMarker.open("checkPointMarkersData.txt");
	outMarker << time << endl;
	outMarker << "x, y, z, vx, vy, vz, m, rho, p, mu, type, index,\n # \n";
	for (int i=0; i < mPosRad.size(); i++) {
		real3 p = mPosRad[i];
		real4 vM = mVelMas[i];
		real4 rPMtype = mRhoPresMu[i];
		uint index = bodyIndex[i];
		outMarker << p.x << ", " << p.y << ", " << p.z << ", " << vM.x << ", " << vM.y << ", " << vM.z << ", "
				<< vM.w << ", " << rPMtype.x << ", " << rPMtype.y << ", " << rPMtype.z << ", " << rPMtype.w << ", " << index << ",\n ";

	}
	outMarker.close();

	//*******************************************************************
	ofstream outRefArray;
	outRefArray.open("checkPointRefrenceArrays.txt");
	outRefArray << time << endl;
	outRefArray << " referenceArray \n #\n";
	for (int i=0; i < referenceArray.size(); i++) {
		int3 ref3 = referenceArray[i];
		outRefArray << ref3.x << "," << ref3.y << "," << ref3.z << endl;
	}

	outRefArray <<"@"<<endl;
	outRefArray << " ANCF_ReferenceArrayNodesOnBeams \n #\n";
	for (int i=0; i < ANCF_ReferenceArrayNodesOnBeams.size(); i ++) {
		int2 ref2 = ANCF_ReferenceArrayNodesOnBeams[i];
		outRefArray << ref2.x << "," << ref2.y << endl;
	}
	outRefArray <<"@"<<endl;
	outRefArray.close();

	//*******************************************************************
	ofstream outRigidData;
	outRigidData.open("checkPointRigidData.txt");
	outRigidData << time << endl;
	outRigidData << "x, y, z, q0, q1, q2, q3, Vx, Vy, Vz, mass, om0, om1, om2, j00, j01, j02, j11, j12, j22, invj00, invj01, invj02, invj11, invj12, invj22, \n";
	outRigidData << " #\n";
	for (int i=0; i < posRigidH.size(); i++) {
		real3 pR = posRigidH[i];
		real4 qR = mQuatRot[i];
		real4 vMR = velMassRigidH[i];
		real3 om = omegaLRF_H[i];
		real3 j1 = jH1[i];
		real3 j2 = jH2[i];
		real3 invj1 = jInvH1[i];
		real3 invj2 = jInvH2[i];
		outRigidData << pR.x << ", " << pR.y << ", " << pR.z << ", " << qR.x << ", " << qR.y << ", " << qR.z << ", " << qR.w << ", " <<
				vMR.x << ", " << vMR.y << ", " << vMR.z << ", " << vMR.w << ", " << om.x << ", " << om.y << ", " << om.z << ", " <<
				j1.x << ", " << j1.y << ", " << j1.z << ", " << j2.x << ", " << j2.y << ", " << j2.z << ", " <<
				invj1.x << ", " << invj1.y << ", " << invj1.z << ", " << invj2.x << ", " << invj2.y << ", " << invj2.z << ", " <<endl;
	}
	outRigidData.close();

	//*******************************************************************
	ofstream outFlexData;
	outFlexData.open("checkPointFlexData.txt");
	outFlexData << time << endl;
	outFlexData << "nx, ny, nz, sx, sy, sz, nVx, nVy, nVz, sVx, sVy, sVz,\n";
	outFlexData << " #\n";
	for (int i=0; i < ANCF_Nodes.size(); i++) {
		real3 n = ANCF_Nodes[i];
		real3 s = ANCF_Slopes[i];
		real3 nV = ANCF_NodesVel[i];
		real3 sV = ANCF_SlopesVel[i];
		outFlexData << n.x << "," << n.y << "," << n.z << "," << s.x << "," << s.y << "," << s.z << "," <<
				nV.x << "," << nV.y << "," << nV.z << "," << sV.x << "," << sV.y << "," << sV.z << "," << endl;
	}

	outFlexData << "@"<< endl;

	outFlexData << "length, isCantilever,\n";
	outFlexData << " #\n";
	for (int i=0; i < ANCF_Beam_Length.size(); i++) {
		outFlexData << ANCF_Beam_Length[i] << "," << ANCF_IsCantilever[i] << "," << endl;
	}

	outFlexData << "@"<< endl;

	outFlexData << "parametric distance,\n";
	outFlexData << " #\n";
	for (int i=0; i < flexParametricDist.size(); i++) {
		outFlexData << flexParametricDist[i] << endl;
	}

	outFlexData << "@"<< endl;
	outFlexData.close();

	//*******************************************************************
	ofstream outProbParams;
	outProbParams.open("checkPointParameters.txt");
	outProbParams << time << endl;
	outProbParams << "All parameters required for the problems,\n";
	outProbParams << " #\n";

	outProbParams << paramsH.gridSize.x << ", " << paramsH.gridSize.y << ", " << paramsH.gridSize.z << endl;
	outProbParams << paramsH.worldOrigin.x << ", " << paramsH.worldOrigin.y << ", " << paramsH.worldOrigin.z << endl ;
	outProbParams << paramsH.cellSize.x << ", " << paramsH.cellSize.y << ", " << paramsH.cellSize.z << endl;
	outProbParams << paramsH.numBodies << endl;
	outProbParams << paramsH.boxDims.x << ", " << paramsH.boxDims.y << ", " << paramsH.boxDims.z << endl;
	outProbParams << paramsH.sizeScale << endl;
	outProbParams << paramsH.HSML << endl;
	outProbParams << paramsH.MULT_INITSPACE << endl;
	outProbParams << paramsH.NUM_BOUNDARY_LAYERS << endl;
	outProbParams << paramsH.toleranceZone << endl;
	outProbParams << paramsH.NUM_BCE_LAYERS << endl;
	outProbParams << paramsH.solidSurfaceAdjust << endl;
	outProbParams << paramsH.BASEPRES << endl;
	outProbParams << paramsH.LARGE_PRES << endl;
	outProbParams << paramsH.nPeriod << endl;
	outProbParams << paramsH.gravity.x << ", " << paramsH.gravity.y << ", " << paramsH.gravity.z << endl;
	outProbParams << paramsH.bodyForce4.x << ", " << paramsH.bodyForce4.y << ", " << paramsH.bodyForce4.z << ", " << paramsH.bodyForce4.w << endl;
	outProbParams << paramsH.rho0 << endl;
	outProbParams << paramsH.mu0 << endl;
	outProbParams << paramsH.v_Max << endl;
	outProbParams << paramsH.EPS_XSPH << endl;
	outProbParams << paramsH.dT << endl;
	outProbParams << paramsH.tFinal << endl;
	outProbParams << paramsH.kdT << endl;
	outProbParams << paramsH.gammaBB << endl;
	outProbParams << paramsH.cMin.x << ", "  << paramsH.cMin.y << ", " << paramsH.cMin.z << endl;
	outProbParams << paramsH.cMax.x << ", " << paramsH.cMax.y << ", " << paramsH.cMax.z << endl;
	outProbParams << paramsH.straightChannelBoundaryMin.x << ", "  << paramsH.straightChannelBoundaryMin.y << ", " << paramsH.straightChannelBoundaryMin.z << endl;
	outProbParams << paramsH.straightChannelBoundaryMax.x << ", " << paramsH.straightChannelBoundaryMax.y << ", " << paramsH.straightChannelBoundaryMax.z << endl;
	outProbParams << paramsH.binSize0 << endl;
	outProbParams << paramsH.rigidRadius.x << ", "  << paramsH.rigidRadius.y << ", " << paramsH.rigidRadius.z << endl;
	outProbParams << paramsH.densityReinit << endl;
	outProbParams << paramsH.contactBoundary << endl;


	outProbParams << "#" <<endl;

	outProbParams << flexParams.r << endl;
	outProbParams << flexParams.E << endl;
	outProbParams << flexParams.I << endl;
	outProbParams << flexParams.rho << endl;
	outProbParams << flexParams.A << endl;
	outProbParams << flexParams.ne << endl;
	outProbParams << flexParams.gravity.x << ", " << flexParams.gravity.y << ", " << flexParams.gravity.z << endl;
	outProbParams << flexParams.bobRad << endl;

	outProbParams << "#" <<endl;

	outProbParams << channelRadius << endl;
	outProbParams << channelCenterYZ.x << ", " << channelCenterYZ.y << endl;

	outProbParams << "#" <<endl;

	outProbParams << numObjects.numRigidBodies         << endl;
	outProbParams << numObjects.numFlexBodies          << endl;
	outProbParams << numObjects.numFlBcRigid           << endl;
	outProbParams << numObjects.numFluidMarkers        << endl;
	outProbParams << numObjects.numBoundaryMarkers     << endl;
	outProbParams << numObjects.startRigidMarkers      << endl;
	outProbParams << numObjects.startFlexMarkers       << endl;
	outProbParams << numObjects.numRigid_SphMarkers    << endl;
	outProbParams << numObjects.numFlex_SphMarkers     << endl;
	outProbParams << numObjects.numAllMarkers          << endl;


	outProbParams.close();
	//****
}


//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&

void ReadEverythingFromFile(
		bool shouldIRead,
		thrust::host_vector<real3> & mPosRad,
		thrust::host_vector<real4> & mVelMas,
		thrust::host_vector<real4> & mRhoPresMu,
		thrust::host_vector<uint> & bodyIndex,
		thrust::host_vector<int3> & referenceArray,

		thrust::host_vector<real3> & posRigidH,
		thrust::host_vector<real4> & mQuatRot,
		thrust::host_vector<real4> & velMassRigidH,
		thrust::host_vector<real3> & omegaLRF_H,
		thrust::host_vector<real3> & jH1,
		thrust::host_vector<real3> & jH2,
		thrust::host_vector<real3> & jInvH1,
		thrust::host_vector<real3> & jInvH2,

		thrust::host_vector<real3> & ANCF_Nodes,
		thrust::host_vector<real3> & ANCF_Slopes,
		thrust::host_vector<real3> & ANCF_NodesVel,
		thrust::host_vector<real3> & ANCF_SlopesVel,

		thrust::host_vector<real_> & ANCF_Beam_Length,
		thrust::host_vector<bool> & ANCF_IsCantilever,

		thrust::host_vector<int2> & ANCF_ReferenceArrayNodesOnBeams,
		thrust::host_vector<real_> & flexParametricDist,

		real_ & channelRadius,
		real2 & channelCenterYZ,
		SimParams & paramsH,
		ANCF_Params & flexParams,
		NumberOfObjects & numObjects) {

	if (!shouldIRead) return;
	//*******************************************************************
	mPosRad.clear();
	mVelMas.clear();
	mRhoPresMu.clear();
	bodyIndex.clear();
	referenceArray.clear();

	posRigidH.clear();
	mQuatRot.clear();
	velMassRigidH.clear();
	omegaLRF_H.clear();
	jH1.clear();
	jH2.clear();
	jInvH1.clear();
	jInvH2.clear();

	ANCF_Nodes.clear();
	ANCF_Slopes.clear();
	ANCF_NodesVel.clear();
	ANCF_SlopesVel.clear();

	ANCF_Beam_Length.clear();
	ANCF_IsCantilever.clear();

	ANCF_ReferenceArrayNodesOnBeams.clear();
	flexParametricDist.clear();
	char ddCh;
	string ddSt;
	//*******************************************************************
	printf("reading marker data\n");
	ifstream inMarker;
	inMarker.open("checkPointMarkersData.txt");
	if (!inMarker) {
		cout << "Error! Unable to open file: " << "checkPointMarkersData.txt" << endl;
	}
	char ch = '!';
	while (ch != '#') {
		inMarker >> ch;
	}
//	inMarker.ignore(numeric_limits<streamsize>::max(), '\n');
	getline(inMarker, ddSt);

	real3 p ;
	real4 vM ;
	real4 rPMtype ;
	uint index;
	inMarker >> p.x >> ddCh >> p.y >> ddCh >> p.z >> ddCh >> vM.x >> ddCh >> vM.y >> ddCh >> vM.z >> ddCh
			>> vM.w >> ddCh >> rPMtype.x >> ddCh >> rPMtype.y >> ddCh >> rPMtype.z >> ddCh >> rPMtype.w >> ddCh >> index >> ddCh;
	while (inMarker.good()) {
		mPosRad.push_back(p);
		mVelMas.push_back(vM) ;
		mRhoPresMu.push_back(rPMtype);
		bodyIndex.push_back(index);

		inMarker >> p.x >> ddCh >> p.y >> ddCh >> p.z >> ddCh >> vM.x >> ddCh >> vM.y >> ddCh >> vM.z >> ddCh
				>> vM.w >> ddCh >> rPMtype.x >> ddCh >> rPMtype.y >> ddCh >> rPMtype.z >> ddCh >> rPMtype.w >> ddCh >> index >> ddCh;
	}
	inMarker.close();

	//*******************************************************************
	printf("reading reference array data\n");
	ifstream inRefArray;
	inRefArray.open("checkPointRefrenceArrays.txt");
	if (!inRefArray) {
		cout << "Error! Unable to open file: " << "checkPointRefrenceArrays.txt" << endl;
	}
	ch = '!';
	while (ch != '#') {
		inRefArray >> ch;
	}
//	inRefArray.ignore(numeric_limits<streamsize>::max(), '\n');
	getline(inRefArray, ddSt);
	while (inRefArray.good()) {
		string s;
		getline(inRefArray, s);
		if (s.find("@") != string::npos) {
			break;
		}
		istringstream ss(s);
		string s1,s2,s3;
		getline(ss, s1, ',');
		getline(ss, s2, ',');
		getline(ss, s3, ',');
//		ss >> ref3.x;
//		if (ss.peek() == ',') ss.ignore();
//		ss >> ref3.y;
//		if (ss.peek() == ',') ss.ignore();
//		ss >> ref3.z;
		referenceArray.push_back(I3(atoi(s1.c_str()), atoi(s2.c_str()), atoi(s3.c_str())));
	}

	ch = '!';
	while (ch != '#') {
		inRefArray >> ch;
	}
//	inRefArray.ignore(numeric_limits<streamsize>::max(), '\n');
	getline(inRefArray, ddSt);
	while (inRefArray.good()) {
		string s;
		getline(inRefArray, s);
		if (s.find("@") != string::npos) {
			break;
		}
		istringstream ss(s);
		string s1,s2;
		getline(ss, s1, ',');
		getline(ss, s2, ',');
//		ss >> ref2.x;
//		if (ss.peek() == ',') ss.ignore();
//		ss >> ref2.y;
		ANCF_ReferenceArrayNodesOnBeams.push_back(I2(atoi(s1.c_str()), atoi(s2.c_str())));
	}
	inRefArray.close();

	//*******************************************************************
	printf("reading rigid data\n");
	ifstream inRigidData;
	inRigidData.open("checkPointRigidData.txt");
	if (!inRigidData) {
		cout << "Error! Unable to open file: " << "checkPointRigidData.txt" << endl;
	}
	ch = '!';
	while (ch != '#') {
		inRigidData >> ch;
	}
	getline(inRigidData, ddSt);
//	inRigidData.ignore(numeric_limits<streamsize>::max(), '\n');
	real3 pR 	;
	real4 qR 	;
	real4 vMR 	;
	real3 om 	;
	real3 j1 	;
	real3 j2 	;
	real3 invj1 ;
	real3 invj2 ;
	inRigidData >> pR.x >> ddCh >> pR.y >> ddCh >> pR.z >> ddCh >> qR.x >> ddCh >> qR.y >> ddCh >> qR.z >> ddCh >> qR.w >> ddCh >>
					vMR.x >> ddCh >> vMR.y >> ddCh >> vMR.z >> ddCh >> vMR.w >> ddCh >> om.x >> ddCh >> om.y >> ddCh >> om.z >> ddCh >>
					j1.x >> ddCh >> j1.y >> ddCh >> j1.z >> ddCh >> j2.x >> ddCh >> j2.y >> ddCh >> j2.z >> ddCh >>
					invj1.x >> ddCh >> invj1.y >> ddCh >> invj1.z >> ddCh >> invj2.x >> ddCh >> invj2.y >> ddCh >> invj2.z >> ddCh;
	while (inRigidData.good()) {
		posRigidH.push_back(pR) 	;
		mQuatRot.push_back(qR )	;
		velMassRigidH.push_back(vMR )	;
		omegaLRF_H.push_back(om )	;
		jH1.push_back(j1 )	;
		jH2.push_back(j2 )	;
		jInvH1.push_back(invj1) ;
		jInvH2.push_back(invj2 );
		inRigidData >> pR.x >> ddCh >> pR.y >> ddCh >> pR.z >> ddCh >> qR.x >> ddCh >> qR.y >> ddCh >> qR.z >> ddCh >> qR.w >> ddCh >>
						vMR.x >> ddCh >> vMR.y >> ddCh >> vMR.z >> ddCh >> vMR.w >> ddCh >> om.x >> ddCh >> om.y >> ddCh >> om.z >> ddCh >>
						j1.x >> ddCh >> j1.y >> ddCh >> j1.z >> ddCh >> j2.x >> ddCh >> j2.y >> ddCh >> j2.z >> ddCh >>
						invj1.x >> ddCh >> invj1.y >> ddCh >> invj1.z >> ddCh >> invj2.x >> ddCh >> invj2.y >> ddCh >> invj2.z >> ddCh;
	}
	inRigidData.close();

	//*******************************************************************
	printf("reading flex data\n");
	ifstream inFlexData;
	inFlexData.open("checkPointFlexData.txt");
	if (!inFlexData) {
		cout << "Error! Unable to open file: " << "checkPointFlexData.txt" << endl;
	}
	ch = '!';
	while (ch != '#') {
		inFlexData >> ch;
	}
//	inFlexData.ignore(numeric_limits<streamsize>::max(), '\n');
	getline(inFlexData, ddSt);
	while (inFlexData.good()) {
		string st;
		getline(inFlexData, st);
		if (st.find("@") != string::npos) {
			break;
		}

		istringstream ss(st);

		string snx,sny,snz, ssx,ssy,ssz, sNVx,sNVy,sNVz, sVVx,sVVy,sVVz;
		getline(ss, snx, ',');
		getline(ss, sny, ',');
		getline(ss, snz, ',');

		getline(ss, ssx, ',');
		getline(ss, ssy, ',');
		getline(ss, ssz, ',');

		getline(ss, sNVx, ',');
		getline(ss, sNVy, ',');
		getline(ss, sNVz, ',');

		getline(ss, sVVx, ',');
		getline(ss, sVVy, ',');
		getline(ss, sVVz, ',');

		ANCF_Nodes.push_back(R3(atof(snx.c_str()), atof(sny.c_str()), atof(snz.c_str())));
		ANCF_Slopes.push_back(R3(atof(ssx.c_str()), atof(ssy.c_str()), atof(ssz.c_str())));
		ANCF_NodesVel.push_back(R3(atof(sNVx.c_str()), atof(sNVy.c_str()), atof(sNVz.c_str())));
		ANCF_SlopesVel.push_back(R3(atof(sVVx.c_str()), atof(sVVy.c_str()), atof(sVVz.c_str())));
	}

	ch = '!';
	while (ch != '#') {
		inFlexData >> ch;
	}
//	inFlexData.ignore(numeric_limits<streamsize>::max(), '\n');
	getline(inFlexData, ddSt);
	while (inFlexData.good()) {
		string s;
		getline(inFlexData, s);
		if (s.find("@") != string::npos) {
			break;
		}
		istringstream ss(s);

		string sl,sa1;
		getline(ss, sl, ',');
		getline(ss, sa1, ',');

		ANCF_Beam_Length.push_back(atof(sl.c_str()));
		if (atoi(sa1.c_str()) == 0) {
			ANCF_IsCantilever.push_back(false);
		} else {
			ANCF_IsCantilever.push_back(true);
		}
	}

	ch = '!';
	while (ch != '#') {
		inFlexData >> ch;
	}
//	inFlexData.ignore(numeric_limits<streamsize>::max(), '\n');
	getline(inFlexData, ddSt);
	while (inFlexData.good()) {
		string s;
		getline(inFlexData, s);
		if (s.find("@") != string::npos) {
			break;
		}
		istringstream ss(s);

		string sdist;
		ss >> sdist;

		flexParametricDist.push_back(real_(atof(sdist.c_str())));
	}
	inFlexData.close();

	//*******************************************************************
	printf("reading parameters\n");
	ifstream inProbParams;
	inProbParams.open("checkPointParameters.txt");
	if (!inProbParams) {
		cout << "Error! Unable to open file: " << "checkPointParameters.txt" << endl;
	}
	ch = '!';
	while (ch != '#') {
		inProbParams >> ch;
	}
	getline(inProbParams, ddSt);
//	inProbParams.ignore(numeric_limits<streamsize>::max(), '\n');

	inProbParams >> paramsH.gridSize.x >> ddCh >> paramsH.gridSize.y >> ddCh >> paramsH.gridSize.z ;
	inProbParams >> paramsH.worldOrigin.x >> ddCh >> paramsH.worldOrigin.y >> ddCh >> paramsH.worldOrigin.z  ;
	inProbParams >> paramsH.cellSize.x >> ddCh >> paramsH.cellSize.y >> ddCh >> paramsH.cellSize.z ;
	inProbParams >> paramsH.numBodies ;
	inProbParams >> paramsH.boxDims.x >> ddCh >> paramsH.boxDims.y >> ddCh >> paramsH.boxDims.z ;
	inProbParams >> paramsH.sizeScale ;
	inProbParams >> paramsH.HSML ;
	inProbParams >> paramsH.MULT_INITSPACE ;
	inProbParams >> paramsH.NUM_BOUNDARY_LAYERS;
	inProbParams >> paramsH.toleranceZone;
	inProbParams >> paramsH.NUM_BCE_LAYERS;
	inProbParams >> paramsH.solidSurfaceAdjust;
	inProbParams >> paramsH.BASEPRES ;
	inProbParams >> paramsH.LARGE_PRES ;
	inProbParams >> paramsH.nPeriod ;
	inProbParams >> paramsH.gravity.x >> ddCh >> paramsH.gravity.y >> ddCh >> paramsH.gravity.z ;
	inProbParams >> paramsH.bodyForce4.x >> ddCh >> paramsH.bodyForce4.y >> ddCh >> paramsH.bodyForce4.z >> ddCh >> paramsH.bodyForce4.w ;
	inProbParams >> paramsH.rho0 ;
	inProbParams >> paramsH.mu0 ;
	inProbParams >> paramsH.v_Max ;
	inProbParams >> paramsH.EPS_XSPH;
	inProbParams >> paramsH.dT;
	inProbParams >> paramsH.tFinal ;
	inProbParams >> paramsH.kdT ;
	inProbParams >> paramsH.gammaBB ;
	inProbParams >> paramsH.cMin.x >> ddCh  >> paramsH.cMin.y >> ddCh >> paramsH.cMin.z ;
	inProbParams >> paramsH.cMax.x >> ddCh >> paramsH.cMax.y >> ddCh >> paramsH.cMax.z ;
	inProbParams >> paramsH.straightChannelBoundaryMin.x >> ddCh  >> paramsH.straightChannelBoundaryMin.y >> ddCh >> paramsH.straightChannelBoundaryMin.z ;
	inProbParams >> paramsH.straightChannelBoundaryMax.x >> ddCh >> paramsH.straightChannelBoundaryMax.y >> ddCh >> paramsH.straightChannelBoundaryMax.z ;
	inProbParams >> paramsH.binSize0 ;
	inProbParams >> paramsH.rigidRadius.x >> ddCh  >> paramsH.rigidRadius.y >> ddCh >> paramsH.rigidRadius.z;
	inProbParams >> paramsH.densityReinit;
	inProbParams >> paramsH.contactBoundary;


	inProbParams >> ddCh;

	inProbParams >> flexParams.r ;
	inProbParams >> flexParams.E ;
	inProbParams >> flexParams.I ;
	inProbParams >> flexParams.rho ;
	inProbParams >> flexParams.A ;
	inProbParams >> flexParams.ne ;
	inProbParams >> flexParams.gravity.x >> ddCh >> flexParams.gravity.y >> ddCh >> flexParams.gravity.z ;
	inProbParams >> flexParams.bobRad ;

	inProbParams >> ddCh ;

	inProbParams >> channelRadius ;
	inProbParams >> channelCenterYZ.x >> ddCh >> channelCenterYZ.y ;

	inProbParams >> ddCh;

	inProbParams >> numObjects.numRigidBodies         ;
	inProbParams >> numObjects.numFlexBodies          ;
	inProbParams >> numObjects.numFlBcRigid           ;
	inProbParams >> numObjects.numFluidMarkers        ;
	inProbParams >> numObjects.numBoundaryMarkers     ;
	inProbParams >> numObjects.startRigidMarkers      ;
	inProbParams >> numObjects.startFlexMarkers       ;
	inProbParams >> numObjects.numRigid_SphMarkers    ;
	inProbParams >> numObjects.numFlex_SphMarkers     ;
	inProbParams >> numObjects.numAllMarkers          ;


	inProbParams.close();
	//****
}

