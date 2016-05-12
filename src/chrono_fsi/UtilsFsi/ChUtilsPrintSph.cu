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
#include "chrono_fsi/UtilsFsi/ChUtilsPrintSph.cuh"
#include "chrono_fsi/custom_math.h"
#include "chrono_fsi/ChDeviceUtils.cuh"
#include "chrono_fsi/ChParams.cuh"

namespace chrono {
namespace fsi {
namespace utils {
//*******************************************************************************************************************************
void PrintToFile_SPH(const thrust::device_vector<Real3>& posRadD,
		const thrust::device_vector<Real3>& velMasD,
		const thrust::device_vector<Real4>& rhoPresMuD,
		const thrust::host_vector<int4>& referenceArray,

		const SimParams paramsH, const Real realTime, int tStep, int stepSave,
		const std::string& out_dir) {
	thrust::host_vector<Real3> posRadH = posRadD;
	thrust::host_vector<Real3> velMasH = velMasD;
	thrust::host_vector<Real4> rhoPresMuH = rhoPresMuD;

	int tStepsPovFiles = stepSave;  // 25;//1000;//2000;
	if (tStep % tStepsPovFiles == 0) {
		//#ifdef _WIN32
		//			system("mkdir povFiles");
		//#else
		//			system("mkdir -p povFiles");
		//#endif
		if (tStep / tStepsPovFiles == 0) {
			const std::string rmCmd = std::string("rm ") + out_dir + std::string("/*.csv");
			system(rmCmd.c_str());
		}
		char fileCounter[5];
		int dumNumChar = sprintf(fileCounter, "%d",
				int(tStep / tStepsPovFiles));

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
		const std::string nameBoundary = out_dir + std::string("/boundary")
				+ std::string(fileCounter) + std::string(".csv");

		//    std::ofstream fileNameBoundaries;
		//    fileNameBoundaries.open(nameBoundary);
		//    std::stringstream ssBoundary;
		//    for (int i = referenceArray[1].x; i < referenceArray[1].y; i++) {
		//      Real3 pos = posRadH[i];
		//      Real3 vel = velMasH[i];
		//      Real4 rP = rhoPresMuH[i];
		//      Real velMag = length(vel);
		//      ssBoundary << pos.x << ", " << pos.y << ", " << pos.z << ", " << vel.x << ", " << vel.y << ", " << vel.z <<
		//      ", "
		//                 << velMag << ",
		//                              "<< rP.x<<",
		//          "<< rP.y<<", "<< rP.w<<", "<<std::endl;
		//    }
		//    fileNameBoundaries << ssBoundary.str();
		//    fileNameBoundaries.close();
		//*****************************************************
		const std::string nameFluidBoundaries = out_dir + std::string("/fluid_boundary")
				+ std::string(fileCounter) + std::string(".csv");

		std::ofstream fileNameFluidBoundaries;
		fileNameFluidBoundaries.open(nameFluidBoundaries);
		std::stringstream ssFluidBoundaryParticles;
		//		ssFluidBoundaryParticles.precision(20);
		for (int i = referenceArray[0].x; i < referenceArray[1].y; i++) {
			Real3 pos = posRadH[i];
			Real3 vel = velMasH[i];
			Real4 rP = rhoPresMuH[i];
			Real velMag = length(vel);
			// if (pos.y > .0002 && pos.y < .0008)
			ssFluidBoundaryParticles << pos.x << ", " << pos.y << ", " << pos.z
					<< ", " << vel.x << ", " << vel.y << ", " << vel.z << ", "
					<< velMag << ", " << rP.x << ", " << rP.y << ", " << rP.z
					<< ", " << rP.w << ", " << std::endl;
		}
		fileNameFluidBoundaries << ssFluidBoundaryParticles.str();
		fileNameFluidBoundaries.close();
		//*****************************************************
		const std::string nameBCE = out_dir + std::string("/BCE") + std::string(fileCounter)
				+ std::string(".csv");

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
				// if (pos.y > .0002 && pos.y < .0008)
				ssBCE << pos.x << ", " << pos.y << ", " << pos.z << ", "
						<< vel.x << ", " << vel.y << ", " << vel.z << ", "
						<< velMag << ", " << rP.x << ", " << rP.y << ", "
						<< rP.z << ", " << rP.w << ", " << std::endl;
			}
		}
		fileNameBCE << ssBCE.str();
		fileNameBCE.close();
		//*****************************************************
	}
	posRadH.clear();
	velMasH.clear();
	rhoPresMuH.clear();
}

//*******************************************************************************************************************************

void PrintToFile(const thrust::device_vector<Real3>& posRadD,
		const thrust::device_vector<Real3>& velMasD,
		const thrust::device_vector<Real4>& rhoPresMuD,
		const thrust::host_vector<int4>& referenceArray,
		const SimParams paramsH, Real realTime, int tStep, int stepSave,
		const std::string& out_dir) {
	// print fluid stuff
	PrintToFile_SPH(posRadD, velMasD, rhoPresMuD, referenceArray, paramsH,
			realTime, tStep, stepSave, out_dir);
}
//*******************************************************************************************************************************
// to be implemented
void PrintToFileCartesian() {
	// ######## the commented sections need to be fixed. you need cartesian data by calling SphSystemGpu.MapSPH_ToGrid
	////////-+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++//comcom
	//	std::ofstream fileNameCartesianTotal;
	//	thrust::host_vector<Real4> rho_Pres_CartH(1);
	//	thrust::host_vector<Real4> vel_VelMag_CartH(1);
	//	Real resolution = 2 * paramsH.HSML;
	//	int3 cartesianGridDims;
	//	int tStepCartesianTotal = 1000000;
	//	int tStepCartesianSlice = 100000;
	//	int tStepPoiseuilleProf = 1000; //tStepCartesianSlice;
	//
	//	int stepCalcCartesian = min(tStepCartesianTotal, tStepCartesianSlice);
	//	stepCalcCartesian = min(stepCalcCartesian, tStepPoiseuilleProf);
	//
	//	if (tStep % stepCalcCartesian == 0) {
	//		MapSPH_ToGrid(resolution, cartesianGridDims, rho_Pres_CartH, vel_VelMag_CartH, posRadD, velMasD,
	// rhoPresMuD,
	//				referenceArray[referenceArray.size() - 1].y, paramsH);
	//	}
	//	if (tStep % tStepCartesianTotal == 0) {
	//		if (tStep / tStepCartesianTotal == 0) {
	//			fileNameCartesianTotal.open("dataCartesianTotal.txt");
	//			fileNameCartesianTotal<<"variables = \"x\", \"y\", \"z\", \"Vx\", \"Vy\", \"Vz\", \"Velocity
	// Magnitude\", \"Rho\", \"Pressure\"\n";
	//		} else {
	//			fileNameCartesianTotal .open("dataCartesianTotal.txt", std::ios::app);
	//		}
	//		fileNameCartesianTotal<<"zone I = "<<cartesianGridDims.x<<", J = "<<cartesianGridDims.y<<", K =
	//"<<cartesianGridDims.z<<std::endl;
	//		std::stringstream ssCartesianTotal;
	//		for (int k = 0; k < cartesianGridDims.z; k++) {
	//			for (int j = 0; j < cartesianGridDims.y; j++) {
	//				for (int i = 0; i < cartesianGridDims.x; i++) {
	//					int index = i + j * cartesianGridDims.x + k * cartesianGridDims.x *
	// cartesianGridDims.y;
	//					Real3 gridNodeLoc = resolution * mR3(i, j, k) + paramsH.worldOrigin;
	//					ssCartesianTotal<<gridNodeLoc.x<<", "<< gridNodeLoc.y<<", "<< gridNodeLoc.z<<",
	//"<<
	//							vel_VelMag_CartH[index].x<<", "<< vel_VelMag_CartH[index].y<<",
	//"<<
	// vel_VelMag_CartH[index].z<<", "<< vel_VelMag_CartH[index].w<<", "<<
	//							rho_Pres_CartH[index].x<<", "<< rho_Pres_CartH[index].y<<std::endl;
	//				}
	//			}
	//		}
	//		fileNameCartesianTotal<<ssCartesianTotal.str();
	//		fileNameCartesianTotal.close();
	//	}
	//////////-+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ //comcom
	//	std::ofstream fileNameCartesianMidplane;
	//	if (tStep % tStepCartesianSlice == 0) {
	//		if (tStep / tStepCartesianSlice == 0) {
	//			fileNameCartesianMidplane.open("dataCartesianMidplane.txt");
	//			fileNameCartesianMidplane<<"variables = \"x\", \"z\", \"Vx\", \"Vy\", \"Vz\", \"Velocity
	// Magnitude\",
	//\"Rho\", \"Pressure\"\n";
	//		} else {
	//			fileNameCartesianMidplane .open("dataCartesianMidplane.txt", std::ios::app);
	//		}
	//		fileNameCartesianMidplane<< "zone I = "<<cartesianGridDims.x<<", J = "<<cartesianGridDims.z<<"\n";
	//		int j = cartesianGridDims.y / 2;
	//		std::stringstream ssCartesianMidplane;
	//		for (int k = 0; k < cartesianGridDims.z; k++) {
	//			for (int i = 0; i < cartesianGridDims.x; i++) {
	//				int index = i + j * cartesianGridDims.x + k * cartesianGridDims.x * cartesianGridDims.y;
	//				Real3 gridNodeLoc = resolution * mR3(i, j, k) + paramsH.worldOrigin;
	//				ssCartesianMidplane<<gridNodeLoc.x<<", "<< gridNodeLoc.z<<", "<<
	// vel_VelMag_CartH[index].x<<",
	//"<<
	//						vel_VelMag_CartH[index].y<<", "<< vel_VelMag_CartH[index].z<<", "<<
	// vel_VelMag_CartH[index].w<<", "<< rho_Pres_CartH[index].x<<", "<<
	//						rho_Pres_CartH[index].y<<std::endl;
	//			}
	//		}
	//		fileNameCartesianMidplane<<ssCartesianMidplane.str();
	//		fileNameCartesianMidplane.close();
	//	}
	//	rho_Pres_CartH.clear();
	//////////-+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++comcom
	//	std::ofstream fileVelocityProfPoiseuille;
	//	if (tStep % tStepPoiseuilleProf == 0) {
	//		if (tStep / tStepPoiseuilleProf == 0) {
	//			fileVelocityProfPoiseuille.open("dataVelProfile.txt");
	//			fileVelocityProfPoiseuille<< "variables = \"Z(m)\", \"Vx(m/s)\"\n";
	//
	//		} else {
	//			fileVelocityProfPoiseuille.open("dataVelProfile.txt", std::ios::app);
	//		}
	//		fileVelocityProfPoiseuille<<"zone T=\"t = "<< realTime <<"\""std::endl;
	//		std::stringstream ssVelocityProfPoiseuille;
	//		int j = cartesianGridDims.y / 2;
	//		int i = cartesianGridDims.x / 2;
	//		for (int k = 0; k < cartesianGridDims.z; k++) {
	//			int index = i + j * cartesianGridDims.x + k * cartesianGridDims.x * cartesianGridDims.y;
	//			Real3 gridNodeLoc = resolution * mR3(i, j, k) + paramsH.worldOrigin;
	//			if (gridNodeLoc.z > 1 * paramsH.sizeScale && gridNodeLoc.z < 2 * paramsH.sizeScale) {
	//				ssVelocityProfPoiseuille<<gridNodeLoc.z<<", "<< vel_VelMag_CartH[index].x<<std::endl;
	//			}
	//		}
	//		fileVelocityProfPoiseuille<<ssVelocityProfPoiseuille.str();
	//		fileVelocityProfPoiseuille.close();
	//	}
	//	vel_VelMag_CartH.clear();
	//////////-+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++comcom
}


}  // end namespace utils
}  // end namespace fsi
}  // end namespace chrono
