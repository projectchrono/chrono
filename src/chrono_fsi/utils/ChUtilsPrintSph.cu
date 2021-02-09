// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Author: Arman Pazouki, Milad Rakhsha, Wei Hu
// =============================================================================
//
// Utility function to print the save fluid, bce, and boundary data to files
// =============================================================================
#include <thrust/reduce.h>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <sstream>
#include "chrono_fsi/utils/ChUtilsDevice.cuh"
#include "chrono_fsi/physics/ChParams.cuh"
#include "chrono_fsi/utils/ChUtilsPrintSph.cuh"

namespace chrono {
namespace fsi {
namespace utils {

void PrintToFile(const thrust::device_vector<Real4>& posRadD,
                 const thrust::device_vector<Real3>& velMasD,
                 const thrust::device_vector<Real4>& rhoPresMuD,
                 const thrust::device_vector<Real4>& sr_tau_I_mu_i,
                 const thrust::host_vector<int4>& referenceArray,
                 const thrust::host_vector<int4>& referenceArrayFEA,
                 const std::string& out_dir,
                 bool printToParaview) {
    thrust::host_vector<Real4> posRadH = posRadD;
    thrust::host_vector<Real3> velMasH = velMasD;
    thrust::host_vector<Real4> rhoPresMuH = rhoPresMuD;
    thrust::host_vector<Real4> h_sr_tau_I_mu_i = sr_tau_I_mu_i;

    bool short_out = true; //if output with less information, set to true

    bool haveHelper = (referenceArray[0].z == -3) ? true : false;
    bool haveGhost = (referenceArray[0].z == -2 || referenceArray[1].z == -2) ? true : false;
    char fileCounter[5];
    static int dumNumChar = -1;
    dumNumChar++;
    double eps = 1e-20;
    sprintf(fileCounter, "%d", dumNumChar);

    if (haveHelper || haveGhost) {
        const std::string nameOthers =
            out_dir + std::string("/others") + std::string(fileCounter) + std::string(".csv");

        std::ofstream fileNameOtherParticles;
        fileNameOtherParticles.open(nameOthers);
        std::stringstream ssotherParticles;
        if (printToParaview){
            if(short_out){
                ssotherParticles << "x,y,z,v_x,v_y,v_z,|U|,rho,pressure\n";
            }
            else{
                ssotherParticles << "x,y,z,h,v_x,v_y,v_z,|U|,rho(rpx),p(rpy),mu(rpz),sr,tau,I,mu_i,type(rpw)\n";
            }
        }
        for (size_t i = referenceArray[0].x; i < referenceArray[haveHelper + haveGhost].y; i++) {
            Real4 rP = rhoPresMuH[i];
            if (rP.w > -2)
                continue;
            Real4 pos = posRadH[i];
            Real3 vel = velMasH[i];
            Real4 stIm = h_sr_tau_I_mu_i[i] + mR4(1e-20);

            Real velMag = length(vel);
            if(short_out){
                ssotherParticles << pos.x << ", " << pos.y << ", " << pos.z << ", " 
                                 << vel.x + eps << ", "<< vel.y + eps << ", " << vel.z + eps << ", " 
                                 << velMag + eps << ", " << rP.x << ", " << rP.y + eps << std::endl;
            }
            else{
                ssotherParticles << pos.x << ", " << pos.y << ", " << pos.z << ", " << pos.w << ", " << vel.x + eps << ", "
                                << vel.y + eps << ", " << vel.z + eps << ", " << velMag + eps << ", " << rP.x << ", "
                                << rP.y + eps << ", " << rP.z << ", " << stIm.x << ", " << stIm.y << ", " << stIm.z << ", "
                                << stIm.w << ", " << rP.w << std::endl;
            }
        }

        fileNameOtherParticles << ssotherParticles.str();
        fileNameOtherParticles.close();
    }
    //*****************************************************
    const std::string nameFluid = out_dir + std::string("/fluid") + std::string(fileCounter) + std::string(".csv");

    std::ofstream fileNameFluidParticles;
    fileNameFluidParticles.open(nameFluid);
    std::stringstream ssFluidParticles;
    if (printToParaview){
        if(short_out){
            ssFluidParticles << "x,y,z,v_x,v_y,v_z,|U|,rho,pressure\n";
        }
        else{
            ssFluidParticles << "x,y,z,h,v_x,v_y,v_z,|U|,rho(rpx),p(rpy),mu(rpz),sr,tau,I,mu_i,type(rpw)\n";
        }
    }

    //    int startFluid = haveHelper + haveGhost;
    for (size_t i = referenceArray[haveHelper + haveGhost].x; i < referenceArray[haveHelper + haveGhost].y; i++) {
        Real4 rP = rhoPresMuH[i];
        if (rP.w != -1)
            continue;
        Real4 pos = posRadH[i];
        Real3 vel = velMasH[i] + mR3(1e-20);
        Real4 stIm = h_sr_tau_I_mu_i[i] + mR4(1e-20);

        Real velMag = length(vel);
        if(short_out){
            ssFluidParticles << pos.x << ", " << pos.y << ", " << pos.z << ", " 
                             << vel.x + eps << ", "<< vel.y + eps << ", " << vel.z + eps << ", " 
                             << velMag + eps << ", " << rP.x << ", " << rP.y + eps << std::endl;
        }
        else{
            ssFluidParticles << pos.x << ", " << pos.y << ", " << pos.z << ", " << pos.w << ", " << vel.x + eps << ", "
                            << vel.y + eps << ", " << vel.z + eps << ", " << velMag + eps << ", " << rP.x << ", "
                            << rP.y + eps << ", " << rP.z << ", " << stIm.x << ", " << stIm.y + eps << ", " << stIm.z
                            << ", " << stIm.w << ", " << rP.w << std::endl;
        }
    }
    fileNameFluidParticles << ssFluidParticles.str();
    fileNameFluidParticles.close();

    //*****************************************************
    const std::string nameFluidBoundaries =
        out_dir + std::string("/boundary") + std::string(fileCounter) + std::string(".csv");

    std::ofstream fileNameFluidBoundaries;
    fileNameFluidBoundaries.open(nameFluidBoundaries);
    std::stringstream ssFluidBoundaryParticles;
    if (printToParaview){
        if(short_out){
            ssFluidBoundaryParticles << "x,y,z,v_x,v_y,v_z,|U|,rho,pressure\n";
        }
        else{
            ssFluidBoundaryParticles << "x,y,z,h,v_x,v_y,v_z,|U|,rho(rpx),p(rpy),mu(rpz),sr,tau,I,mu_i,type(rpw)\n";
        }
    }

    //		ssFluidBoundaryParticles.precision(20);
    for (size_t i = referenceArray[haveHelper + haveGhost + 1].x; i < referenceArray[haveHelper + haveGhost + 1].y;
         i++) {
        Real4 rP = rhoPresMuH[i];
        if (rP.w != 0)
            continue;
        Real4 pos = posRadH[i];
        Real3 vel = velMasH[i] + mR3(1e-20);
        Real4 stIm = h_sr_tau_I_mu_i[i] + mR4(1e-20);

        Real velMag = length(vel);
        if(short_out){
            ssFluidBoundaryParticles << pos.x << ", " << pos.y << ", " << pos.z << ", " 
                             << vel.x + eps << ", "<< vel.y + eps << ", " << vel.z + eps << ", " 
                             << velMag + eps << ", " << rP.x << ", " << rP.y + eps << std::endl;
        }
        else{
            ssFluidBoundaryParticles << pos.x << ", " << pos.y << ", " << pos.z << ", " << pos.w << ", " << vel.x + eps
                                    << ", " << vel.y + eps << ", " << vel.z + eps << ", " << velMag + eps << ", " << rP.x
                                    << ", " << rP.y + eps << ", " << rP.z << ", " << stIm.x << ", " << stIm.y + eps << ", "
                                    << stIm.z << ", " << stIm.w << ", " << rP.w << std::endl;
        }
    }
    fileNameFluidBoundaries << ssFluidBoundaryParticles.str();
    fileNameFluidBoundaries.close();

    //*****************************************************
    int refSize = (int)referenceArray.size();
    if (refSize > haveHelper + haveGhost + 2) {
        const std::string nameBCE = out_dir + std::string("/BCE_Rigid") + std::string(fileCounter) + std::string(".csv");

        std::ofstream fileNameBCE;
        fileNameBCE.open(nameBCE);
        std::stringstream ssBCE;
        //		ssFluidBoundaryParticles.precision(20);
        if (printToParaview){
            if(short_out){
                ssBCE << "x,y,z,v_x,v_y,v_z,|U|,rho,pressure\n";
            }
            else{
                ssBCE << "x,y,z,h,v_x,v_y,v_z,|U|,rho(rpx),p(rpy),mu(rpz),sr,tau,I,mu_i,type(rpw)\n";
            }
        }

        for (size_t i = referenceArray[haveHelper + haveGhost + 2].x; i < referenceArray[refSize - 1].y; i++) {
            //            if (referenceArray[haveHelper + haveGhost + 2].w)
            Real4 pos = posRadH[i];
            Real3 vel = velMasH[i] + mR3(1e-20);
            Real4 rP = rhoPresMuH[i];
            Real velMag = length(vel);
            Real4 stIm = h_sr_tau_I_mu_i[i] + mR4(1e-20);

            if (rP.w == 1.0){
                if(short_out){
                    ssBCE << pos.x << ", " << pos.y << ", " << pos.z << ", " 
                                     << vel.x + eps << ", "<< vel.y + eps << ", " << vel.z + eps << ", " 
                                     << velMag + eps << ", " << rP.x << ", " << rP.y + eps << std::endl;
                }
                else{
                    ssBCE << pos.x << ", " << pos.y << ", " << pos.z << ", " << pos.w << ", " << vel.x + eps << ", "
                        << vel.y + eps << ", " << vel.z + eps << ", " << velMag + eps << ", " << rP.x << ", "
                        << rP.y + eps << ", " << rP.z << ", " << stIm.x << ", " << stIm.y + eps << ", " << stIm.z << ", "
                        << stIm.w << ", " << rP.w << std::endl;
                }
            }
        }
    fileNameBCE << ssBCE.str();
    fileNameBCE.close();        
    }

    //*****************************************************
    int refSize_Flex = (int)referenceArrayFEA.size();
    if (refSize_Flex > 0) {
        const std::string nameBCE_Flex =
            out_dir + std::string("/BCE_Flex") + std::string(fileCounter) + std::string(".csv");

        std::ofstream fileNameBCE_Flex;
        fileNameBCE_Flex.open(nameBCE_Flex);
        std::stringstream ssBCE_Flex;
        //		ssFluidBoundaryParticles.precision(20);

        if (printToParaview){
            if(short_out){
                ssBCE_Flex << "x,y,z,v_x,v_y,v_z,|U|,rho,pressure\n";
            }
            else{
                ssBCE_Flex << "x,y,z,h,v_x,v_y,v_z,|U|,rho(rpx),p(rpy),mu(rpz),sr,tau,I,mu_i,type(rpw)\n";
            }
        }
        for (size_t i = referenceArrayFEA[0].x; i < referenceArrayFEA[refSize_Flex - 1].y; i++) {
            Real4 pos = posRadH[i];
            Real3 vel = velMasH[i] + mR3(1e-20);
            Real4 rP = rhoPresMuH[i];
            Real4 stIm = h_sr_tau_I_mu_i[i] + mR4(1e-20);

            Real velMag = length(vel);
            if(short_out){
                ssBCE_Flex << pos.x << ", " << pos.y << ", " << pos.z << ", " 
                                 << vel.x + eps << ", "<< vel.y + eps << ", " << vel.z + eps << ", " 
                                 << velMag + eps << ", " << rP.x << ", " << rP.y + eps << std::endl;
            }
            else{
                ssBCE_Flex << pos.x << ", " << pos.y << ", " << pos.z << ", " << pos.w << ", " << vel.x + eps << ", "
                        << vel.y + eps << ", " << vel.z + eps << ", " << velMag + eps << ", " << rP.x << ", "
                        << rP.y + 1e-20 << ", " << rP.z << ", " << stIm.x << ", " << stIm.y + eps << ", " << stIm.z
                        << ", " << stIm.w << ", " << rP.w << std::endl;
            }
        }
        fileNameBCE_Flex << ssBCE_Flex.str();
        fileNameBCE_Flex.close();
    }

    //*****************************************************
    posRadH.clear();
    velMasH.clear();
    rhoPresMuH.clear();
}

}  // end namespace utils
}  // end namespace fsi
}  // end namespace chrono
