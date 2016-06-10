/*
 * printToFile.cu
 *
 *  Created on: Mar 2, 2015
 *      Author: Arman Pazouki
 */
#include <string.h>
#include <stdio.h>
#include <sstream>
#include <fstream>
#include <thrust/reduce.h>
#include "chrono_fsi/UtilsFsi/ChUtilsPrintSph.h"
#include "chrono_fsi/ChDeviceUtils.cuh"
#include "chrono_fsi/ChParams.cuh"

namespace chrono {
namespace fsi {
namespace utils {
//*******************************************************************************************************************************
	void PrintToFile(const thrust::device_vector<Real3>& posRadD,
		const thrust::device_vector<Real3>& velMasD,
		const thrust::device_vector<Real4>& rhoPresMuD,
		const thrust::host_vector<int4>& referenceArray,
		const std::string& out_dir) {

		thrust::host_vector<Real3> posRadH = posRadD;
		thrust::host_vector<Real3> velMasH = velMasD;
		thrust::host_vector<Real4> rhoPresMuH = rhoPresMuD;

		char fileCounter[5];
		static int dumNumChar = -1;
		dumNumChar ++;
		sprintf(fileCounter, "%d", dumNumChar);

		//*****************************************************
		const std::string nameFluid = out_dir + std::string("/fluid")
				+ std::string(fileCounter) + std::string(".csv");

		std::ofstream fileNameFluidParticles;
		fileNameFluidParticles.open(nameFluid);
		std::stringstream ssFluidParticles;
		for (int i = referenceArray[0].x; i < referenceArray[0].y; i++) {
			Real3 pos = posRadH[i];
			Real3 vel = velMasH[i];
			Real4 rP = rhoPresMuH[i];
			Real velMag = length(vel);
			ssFluidParticles << pos.x << ", " << pos.y << ", " << pos.z << ", "
					<< vel.x << ", " << vel.y << ", " << vel.z << ", " << velMag
					<< ", " << rP.x << ", " << rP.y << ", " << rP.w << ", "
					<< std::endl;
		}
		fileNameFluidParticles << ssFluidParticles.str();
		fileNameFluidParticles.close();
		//*****************************************************
		const std::string nameFluidBoundaries = out_dir
				+ std::string("/fluid_boundary") + std::string(fileCounter)
				+ std::string(".csv");

		std::ofstream fileNameFluidBoundaries;
		fileNameFluidBoundaries.open(nameFluidBoundaries);
		std::stringstream ssFluidBoundaryParticles;
		//		ssFluidBoundaryParticles.precision(20);
		for (int i = referenceArray[0].x; i < referenceArray[1].y; i++) {
			Real3 pos = posRadH[i];
			Real3 vel = velMasH[i];
			Real4 rP = rhoPresMuH[i];
			Real velMag = length(vel);
			ssFluidBoundaryParticles << pos.x << ", " << pos.y << ", " << pos.z
					<< ", " << vel.x << ", " << vel.y << ", " << vel.z << ", "
					<< velMag << ", " << rP.x << ", " << rP.y << ", " << rP.z
					<< ", " << rP.w << ", " << std::endl;
		}
		fileNameFluidBoundaries << ssFluidBoundaryParticles.str();
		fileNameFluidBoundaries.close();
		//*****************************************************
		const std::string nameBCE = out_dir + std::string("/BCE")
				+ std::string(fileCounter) + std::string(".csv");

		std::ofstream fileNameBCE;
		fileNameBCE.open(nameBCE);
		std::stringstream ssBCE;
		//		ssFluidBoundaryParticles.precision(20);

		int refSize = referenceArray.size();
		if (refSize > 2) {
			for (int i = referenceArray[2].x; i < referenceArray[refSize - 1].y;
					i++) {
				Real3 pos = posRadH[i];
				Real3 vel = velMasH[i];
				Real4 rP = rhoPresMuH[i];
				Real velMag = length(vel);
				
				ssBCE << pos.x << ", " << pos.y << ", " << pos.z << ", "
						<< vel.x << ", " << vel.y << ", " << vel.z << ", "
						<< velMag << ", " << rP.x << ", " << rP.y << ", "
						<< rP.z << ", " << rP.w << ", " << std::endl;
			}
		}
		fileNameBCE << ssBCE.str();
		fileNameBCE.close();
		//*****************************************************
		posRadH.clear();
		velMasH.clear();
		rhoPresMuH.clear();
}


}  // end namespace utils
}  // end namespace fsi
}  // end namespace chrono
