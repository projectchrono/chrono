//// =============================================================================
//// PROJECT CHRONO - http://projectchrono.org
//
////
//// Copyright (c) 2014 projectchrono.org
//// All right reserved.
////
//// Use of this source code is governed by a BSD-style license that can be found
//// in the LICENSE file at the top level of the distribution and at
//// http://projectchrono.org/license-chrono.txt.
////
//// =============================================================================
//// Author: Milad Rakhsha
//// =============================================================================
////
//// Utility function to print the save fluid, bce, and boundary data into file
//// =============================================================================
//#include <thrust/reduce.h>
//#include <iostream>
//#include <sstream>
//#include <fstream>
//#include <string>
//#include <algorithm>
//#include <functional>
//#include <thrust/reduce.h>
//
//#include "chrono_fsi/utils/ChUtilsReadSph.h"
//#include "chrono_fsi/ChDeviceUtils.cuh"
//#include "chrono_fsi/ChParams.cuh"
//#include "chrono/core/ChException.h"
//#include "chrono/core/ChMatrixDynamic.h"
//
// namespace chrono {
// namespace fsi {
// namespace utils {
////*******************************************************************************************************************************
//
// void ReadFromParaViewFile(thrust::host_vector<Real3>& posRadH,
//                          thrust::host_vector<Real3>& velMasH,
//                          thrust::host_vector<Real4>& rhoPresMuH,
//                          thrust::host_vector<int4>& referenceArray,
//                          thrust::host_vector<int4>& referenceArrayFEA,
//                          const std::string& out_dir,
//                          const int output) {
//  //*****************************************************
//  const std::string nameFluidBoundaries =
//      out_dir + std::string("/fluid_boundary") + std::to_string(output) + std::string(".csv");
//  std::fstream fin(nameFluidBoundaries);
//  if (!fin.good())
//    throw ChException("ERROR opening SPH data file file: " + nameFluidBoundaries + "\n");
//
//  ChMatrixDynamic<double> SPH_Values(1, 11);
//  std::string line;
//  getline(fin, line);
//  while (getline(fin, line)) {
//    // trims white space from the beginning of the string
//    for (int isph = 0; isph < referenceArray[1].y; isph++) {
//      int ntoken = 0;
//      std::string token;
//      std::istringstream ss(line);
//      while (std::getline(ss, token, ' ') && ntoken < 12) {
//        std::istringstream stoken(token);
//        stoken >> SPH_Values(0, ntoken);
//        ++ntoken;
//      }
//      posRadH[isph].x = SPH_Values(0, 0);
//      posRadH[isph].y = SPH_Values(0, 1);
//      posRadH[isph].z = SPH_Values(0, 2);
//
//      velMasH[isph].x = SPH_Values(0, 3);
//      velMasH[isph].y = SPH_Values(0, 4);
//      velMasH[isph].z = SPH_Values(0, 5);
//
//      rhoPresMuH[isph].x = SPH_Values(0, 7);
//      rhoPresMuH[isph].y = SPH_Values(0, 8);
//      rhoPresMuH[isph].z = SPH_Values(0, 9);
//      rhoPresMuH[isph].w = SPH_Values(0, 10);
//    }
//  }
//
//  fin.close();
//  //*****************************************************
//  const std::string nameBCE = out_dir + std::string("/BCE") + std::to_string(output) + std::string(".csv");
//  std::fstream fin2(nameBCE);
//  if (!fin2.good())
//    throw ChException("ERROR opening SPH data file file: " + std::string(nameBCE) + "\n");
//
//  int numSHell = referenceArrayFEA.size();
//  getline(fin2, line);
//  while (getline(fin2, line)) {
//    // trims white space from the beginning of the string
//
//    for (int isph = referenceArray[1].y; isph < referenceArray[numSHell].z; isph++) {
//      int ntoken = 0;
//      std::string token;
//      std::istringstream ss(line);
//      while (std::getline(ss, token, ' ') && ntoken < 12) {
//        std::istringstream stoken(token);
//        stoken >> SPH_Values(0, ntoken);
//        ++ntoken;
//      }
//      posRadH[isph].x = SPH_Values(0, 0);
//      posRadH[isph].y = SPH_Values(0, 1);
//      posRadH[isph].z = SPH_Values(0, 2);
//
//      velMasH[isph].x = SPH_Values(0, 3);
//      velMasH[isph].y = SPH_Values(0, 4);
//      velMasH[isph].z = SPH_Values(0, 5);
//
//      rhoPresMuH[isph].x = SPH_Values(0, 7);
//      rhoPresMuH[isph].y = SPH_Values(0, 8);
//      rhoPresMuH[isph].z = SPH_Values(0, 9);
//      rhoPresMuH[isph].w = SPH_Values(0, 10);
//    }
//  }
//
//  fin2.close();
//  //*****************************************************
//
//  posRadH.clear();
//  velMasH.clear();
//  rhoPresMuH.clear();
//}
//}  // end namespace utils
//}  // end namespace fsi
//}  // end namespace chrono
