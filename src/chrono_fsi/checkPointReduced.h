///////////////////////////////////////////////////////////////////////////////
//	checkPoint.h
//	Reads the initializes the particles, either from file or inside the code
//
//	Related Files:
//	Input File:		initializer.txt (optional: if initialize from file)
//					This file contains the sph particles specifications. The description
//					reads the number of particles first. The each line provides the
//					properties of one SPH particl:
//					position(x,y,z), radius, velocity(x,y,z), mass, \rho, pressure, mu,
// particle_type(rigid
// or fluid)
//
//	Created by Arman Pazouki

#include <thrust/host_vector.h>
//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
void CheckPointMarkers_Write(const thrust::host_vector<Real3>& mPosRad,
		const thrust::host_vector<Real4>& mVelMas,
		const thrust::host_vector<Real4>& mRhoPresMu,
		const thrust::host_vector<uint>& bodyIndex,
		const thrust::host_vector<int4>& referenceArray,

		SimParams paramsH, NumberOfObjects numObjects, int tStep,
		int tStepsCheckPoint);

//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
void CheckPointMarkers_Read(bool shouldIRead,
		thrust::host_vector<Real3>& mPosRad,
		thrust::host_vector<Real4>& mVelMas,
		thrust::host_vector<Real4>& mRhoPresMu,
		thrust::host_vector<uint>& bodyIndex,
		thrust::host_vector<int4>& referenceArray,

		SimParams& paramsH, NumberOfObjects& numObjects);
