///////////////////////////////////////////////////////////////////////////////
//	checkPoint.cpp
//	Reads the initializes the particles, either from file or inside the code
//
//	Related Files:
//	Input File:		initializer.txt (optional: if initialize from file)
//					This file contains the sph particles specifications. The description
//					reads the number of particles first. The each line provides the
//					properties of one SPH particl:
//					position(x,y,z), radius, velocity(x,y,z), mass, \rho, pressure, mu,
//particle_type(rigid
// or fluid)
//
//	Created by Arman Pazouki
///////////////////////////////////////////////////////////////////////////////

#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <cstdlib>  //for atof
#include "chrono_fsi/custom_cutil_math.h"
#include "chrono_fsi/SPHCudaUtils.h"
#include "chrono_fsi/MyStructs.cuh"
#include "chrono_fsi/checkPointReduced.h"

using namespace std;

//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
void CheckPointMarkers_Write(const thrust::host_vector<Real3>& mPosRad,
                             const thrust::host_vector<Real4>& mVelMas,
                             const thrust::host_vector<Real4>& mRhoPresMu,
                             const thrust::host_vector<uint>& bodyIndex,
                             const thrust::host_vector<int4>& referenceArray,

                             SimParams paramsH,
                             NumberOfObjects numObjects,
                             int tStep,
                             int tStepsCheckPoint) {
  //*******************************************************************
  if (tStep % tStepsCheckPoint != 0)
    return;

  time_t rawtime;
  struct tm* timeinfo;
  time(&rawtime);
  timeinfo = localtime(&rawtime);
//	printf("Job was submittet at date/time: %s\n", asctime(timeinfo));

#ifdef _WIN32
  system("mkdir checkPoint");
#else
  system("mkdir -p checkPoint");
#endif
  if (tStep / tStepsCheckPoint == 0) {
    // linux. In windows, it is System instead of system (to invoke a command in the command line)
    system("rm checkPoint/*.txt");
  }

  char fileCounter[5];
  int dumNumChar = sprintf(fileCounter, "%d", tStep);

  char nameMarkerData[255];

  sprintf(nameMarkerData, "checkPoint/");
  strcat(nameMarkerData, fileCounter);
  strcat(nameMarkerData, "_");
  strcat(nameMarkerData, "checkPointMarkersData.txt");

  ofstream outMarker;
  outMarker.open(nameMarkerData);
  outMarker << asctime(timeinfo);
  outMarker << "x, y, z, vx, vy, vz, m, rho, p, mu, type, index,\n # \n";
  for (int i = 0; i < mPosRad.size(); i++) {
    Real3 p = mPosRad[i];
    Real4 vM = mVelMas[i];
    Real4 rPMtype = mRhoPresMu[i];
    uint index = bodyIndex[i];
    outMarker << p.x << ", " << p.y << ", " << p.z << ", " << vM.x << ", " << vM.y << ", " << vM.z << ", " << vM.w
              << ", " << rPMtype.x << ", " << rPMtype.y << ", " << rPMtype.z << ", " << rPMtype.w << ", " << index
              << ",\n ";
  }
  outMarker.close();

  //*******************************************************************
  char nameRefArray[255];

  sprintf(nameRefArray, "checkPoint/");
  strcat(nameRefArray, fileCounter);
  strcat(nameRefArray, "_");
  strcat(nameRefArray, "checkPointRefrenceArrays.txt");

  ofstream outRefArray;
  outRefArray.open(nameRefArray);
  outRefArray << asctime(timeinfo);
  outRefArray << " referenceArray \n #\n";
  for (int i = 0; i < referenceArray.size(); i++) {
    int4 ref4 = referenceArray[i];
    outRefArray << ref4.x << "," << ref4.y << "," << ref4.z << ref4.w << "," << endl;
  }

  outRefArray.close();

  //*******************************************************************
  char nameProbParams[255];

  sprintf(nameProbParams, "checkPoint/");
  strcat(nameProbParams, fileCounter);
  strcat(nameProbParams, "_");
  strcat(nameProbParams, "checkPointParameters.txt");

  ofstream outProbParams;
  outProbParams.open(nameProbParams);
  outProbParams << asctime(timeinfo);
  outProbParams << "All parameters required for the problems,\n";
  outProbParams << " #\n";

  outProbParams << paramsH.gridSize.x << ", " << paramsH.gridSize.y << ", " << paramsH.gridSize.z << endl;
  outProbParams << paramsH.worldOrigin.x << ", " << paramsH.worldOrigin.y << ", " << paramsH.worldOrigin.z << endl;
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
  outProbParams << paramsH.deltaPress.x << ", " << paramsH.deltaPress.y << ", " << paramsH.deltaPress.z << endl;
  outProbParams << paramsH.nPeriod << endl;
  outProbParams << paramsH.gravity.x << ", " << paramsH.gravity.y << ", " << paramsH.gravity.z << endl;
  outProbParams << paramsH.bodyForce3.x << ", " << paramsH.bodyForce3.y << ", " << paramsH.bodyForce3.z << endl;
  outProbParams << paramsH.rho0 << endl;
  outProbParams << paramsH.mu0 << endl;
  outProbParams << paramsH.v_Max << endl;
  outProbParams << paramsH.EPS_XSPH << endl;
  outProbParams << paramsH.multViscosity_FSI << endl;
  outProbParams << paramsH.dT << endl;
  outProbParams << paramsH.tFinal << endl;
  outProbParams << paramsH.timePause << endl;
  outProbParams << paramsH.timePauseRigidFlex << endl;
  outProbParams << paramsH.kdT << endl;
  outProbParams << paramsH.gammaBB << endl;
  outProbParams << paramsH.cMin.x << ", " << paramsH.cMin.y << ", " << paramsH.cMin.z << endl;
  outProbParams << paramsH.cMax.x << ", " << paramsH.cMax.y << ", " << paramsH.cMax.z << endl;
  outProbParams << paramsH.straightChannelBoundaryMin.x << ", " << paramsH.straightChannelBoundaryMin.y << ", "
                << paramsH.straightChannelBoundaryMin.z << endl;
  outProbParams << paramsH.straightChannelBoundaryMax.x << ", " << paramsH.straightChannelBoundaryMax.y << ", "
                << paramsH.straightChannelBoundaryMax.z << endl;
  outProbParams << paramsH.binSize0 << endl;
  outProbParams << paramsH.rigidRadius.x << ", " << paramsH.rigidRadius.y << ", " << paramsH.rigidRadius.z << endl;
  outProbParams << paramsH.densityReinit << endl;
  outProbParams << paramsH.contactBoundary << endl;
  outProbParams << paramsH.enableTweak << endl;
  outProbParams << paramsH.enableAggressiveTweak << endl;
  outProbParams << paramsH.tweakMultV << endl;
  outProbParams << paramsH.tweakMultRho << endl;

  outProbParams << "#" << endl;

  outProbParams << numObjects.numRigidBodies << endl;
  outProbParams << numObjects.numFlexBodies << endl;
  outProbParams << numObjects.numFlBcRigid << endl;
  outProbParams << numObjects.numFluidMarkers << endl;
  outProbParams << numObjects.numBoundaryMarkers << endl;
  outProbParams << numObjects.startRigidMarkers << endl;
  outProbParams << numObjects.startFlexMarkers << endl;
  outProbParams << numObjects.numRigid_SphMarkers << endl;
  outProbParams << numObjects.numFlex_SphMarkers << endl;
  outProbParams << numObjects.numAllMarkers << endl;

  outProbParams.close();
  //****
}

//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&

void CheckPointMarkers_Read(bool shouldIRead,
                            thrust::host_vector<Real3>& mPosRad,
                            thrust::host_vector<Real4>& mVelMas,
                            thrust::host_vector<Real4>& mRhoPresMu,
                            thrust::host_vector<uint>& bodyIndex,
                            thrust::host_vector<int4>& referenceArray,

                            SimParams& paramsH,
                            NumberOfObjects& numObjects) {
  if (!shouldIRead)
    return;
  //*******************************************************************
  mPosRad.clear();
  mVelMas.clear();
  mRhoPresMu.clear();
  bodyIndex.clear();
  referenceArray.clear();

  char ddCh;
  string ddSt;
  //*******************************************************************
  printf("reading marker data\n");
  ifstream inMarker;
  inMarker.open("checkPointMarkersData.txt");
  if (!inMarker) {
    cout << "Error! Unable to open file: "
         << "checkPointMarkersData.txt" << endl;
  }
  char ch = '!';
  while (ch != '#') {
    inMarker >> ch;
  }
  //	inMarker.ignore(numeric_limits<streamsize>::max(), '\n');
  getline(inMarker, ddSt);

  Real3 p;
  Real4 vM;
  Real4 rPMtype;
  uint index;
  inMarker >> p.x >> ddCh >> p.y >> ddCh >> p.z >> ddCh >> vM.x >> ddCh >> vM.y >> ddCh >> vM.z >> ddCh >> vM.w >>
      ddCh >> rPMtype.x >> ddCh >> rPMtype.y >> ddCh >> rPMtype.z >> ddCh >> rPMtype.w >> ddCh >> index >> ddCh;
  while (inMarker.good()) {
    mPosRad.push_back(p);
    mVelMas.push_back(vM);
    mRhoPresMu.push_back(rPMtype);
    bodyIndex.push_back(index);

    inMarker >> p.x >> ddCh >> p.y >> ddCh >> p.z >> ddCh >> vM.x >> ddCh >> vM.y >> ddCh >> vM.z >> ddCh >> vM.w >>
        ddCh >> rPMtype.x >> ddCh >> rPMtype.y >> ddCh >> rPMtype.z >> ddCh >> rPMtype.w >> ddCh >> index >> ddCh;
  }
  inMarker.close();

  //*******************************************************************
  printf("reading reference array data\n");
  ifstream inRefArray;
  inRefArray.open("checkPointRefrenceArrays.txt");
  if (!inRefArray) {
    cout << "Error! Unable to open file: "
         << "checkPointRefrenceArrays.txt" << endl;
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
    string s1, s2, s3, s4;
    getline(ss, s1, ',');
    getline(ss, s2, ',');
    getline(ss, s3, ',');
    getline(ss, s4, ',');
    referenceArray.push_back(mI4(atoi(s1.c_str()), atoi(s2.c_str()), atoi(s3.c_str()), atoi(s4.c_str())));
  }
  inRefArray.close();

  //*******************************************************************

  printf("reading parameters\n");
  ifstream inProbParams;
  inProbParams.open("checkPointParameters.txt");
  if (!inProbParams) {
    cout << "Error! Unable to open file: "
         << "checkPointParameters.txt" << endl;
  }
  ch = '!';
  while (ch != '#') {
    inProbParams >> ch;
  }
  getline(inProbParams, ddSt);
  //	inProbParams.ignore(numeric_limits<streamsize>::max(), '\n');

  inProbParams >> paramsH.gridSize.x >> ddCh >> paramsH.gridSize.y >> ddCh >> paramsH.gridSize.z;
  inProbParams >> paramsH.worldOrigin.x >> ddCh >> paramsH.worldOrigin.y >> ddCh >> paramsH.worldOrigin.z;
  inProbParams >> paramsH.cellSize.x >> ddCh >> paramsH.cellSize.y >> ddCh >> paramsH.cellSize.z;
  inProbParams >> paramsH.numBodies;
  inProbParams >> paramsH.boxDims.x >> ddCh >> paramsH.boxDims.y >> ddCh >> paramsH.boxDims.z;
  inProbParams >> paramsH.sizeScale;
  inProbParams >> paramsH.HSML;
  inProbParams >> paramsH.MULT_INITSPACE;
  inProbParams >> paramsH.NUM_BOUNDARY_LAYERS;
  inProbParams >> paramsH.toleranceZone;
  inProbParams >> paramsH.NUM_BCE_LAYERS;
  inProbParams >> paramsH.solidSurfaceAdjust;
  inProbParams >> paramsH.BASEPRES;
  inProbParams >> paramsH.LARGE_PRES;
  inProbParams >> paramsH.deltaPress.x >> ddCh >> paramsH.deltaPress.y >> ddCh >> paramsH.deltaPress.z;
  inProbParams >> paramsH.nPeriod;
  inProbParams >> paramsH.gravity.x >> ddCh >> paramsH.gravity.y >> ddCh >> paramsH.gravity.z;
  inProbParams >> paramsH.bodyForce3.x >> ddCh >> paramsH.bodyForce3.y >> ddCh >> paramsH.bodyForce3.z;
  inProbParams >> paramsH.rho0;
  inProbParams >> paramsH.mu0;
  inProbParams >> paramsH.v_Max;
  inProbParams >> paramsH.EPS_XSPH;
  inProbParams >> paramsH.multViscosity_FSI;
  inProbParams >> paramsH.dT;
  inProbParams >> paramsH.tFinal;
  inProbParams >> paramsH.timePause;
  inProbParams >> paramsH.timePauseRigidFlex;
  inProbParams >> paramsH.kdT;
  inProbParams >> paramsH.gammaBB;
  inProbParams >> paramsH.cMin.x >> ddCh >> paramsH.cMin.y >> ddCh >> paramsH.cMin.z;
  inProbParams >> paramsH.cMax.x >> ddCh >> paramsH.cMax.y >> ddCh >> paramsH.cMax.z;
  inProbParams >> paramsH.straightChannelBoundaryMin.x >> ddCh >> paramsH.straightChannelBoundaryMin.y >> ddCh >>
      paramsH.straightChannelBoundaryMin.z;
  inProbParams >> paramsH.straightChannelBoundaryMax.x >> ddCh >> paramsH.straightChannelBoundaryMax.y >> ddCh >>
      paramsH.straightChannelBoundaryMax.z;
  inProbParams >> paramsH.binSize0;
  inProbParams >> paramsH.rigidRadius.x >> ddCh >> paramsH.rigidRadius.y >> ddCh >> paramsH.rigidRadius.z;
  inProbParams >> paramsH.densityReinit;
  inProbParams >> paramsH.contactBoundary;
  inProbParams >> paramsH.enableTweak;
  inProbParams >> paramsH.enableAggressiveTweak;
  inProbParams >> paramsH.tweakMultV;
  inProbParams >> paramsH.tweakMultRho;

  inProbParams >> ddCh;

  inProbParams >> numObjects.numRigidBodies;
  inProbParams >> numObjects.numFlexBodies;
  inProbParams >> numObjects.numFlBcRigid;
  inProbParams >> numObjects.numFluidMarkers;
  inProbParams >> numObjects.numBoundaryMarkers;
  inProbParams >> numObjects.startRigidMarkers;
  inProbParams >> numObjects.startFlexMarkers;
  inProbParams >> numObjects.numRigid_SphMarkers;
  inProbParams >> numObjects.numFlex_SphMarkers;
  inProbParams >> numObjects.numAllMarkers;

  inProbParams.close();
  //****
}
