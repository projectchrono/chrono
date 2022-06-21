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
                 const std::string& dir,
                 const std::shared_ptr<SimParams>& paramsH,
                 bool printToParaview) {
    thrust::host_vector<Real4> posRadH = posRadD;
    thrust::host_vector<Real3> velMasH = velMasD;
    thrust::host_vector<Real4> rhoPresMuH = rhoPresMuD;
    thrust::host_vector<Real4> h_sr_tau_I_mu_i = sr_tau_I_mu_i;

    // Current frame number
    static int frame_num = -1;
    frame_num++;

    // Set the data output length
    int out_length = paramsH->output_length;

    bool haveHelper = (referenceArray[0].z == -3) ? true : false;
    bool haveGhost = (referenceArray[0].z == -2 || referenceArray[1].z == -2) ? true : false;
    double eps = 1e-20;

    // Save helper and ghost particles to files 
    if (haveHelper || haveGhost) {
        std::string nameOthers = dir + "/others" + std::to_string(frame_num) + ".csv";

        std::ofstream fileNameOtherParticles;
        fileNameOtherParticles.open(nameOthers);
        std::stringstream ssotherParticles;
        if (printToParaview) {
            if (out_length == 0) {
                ssotherParticles << "x,y,z,|U|\n";
            } else if (out_length == 1) {
                ssotherParticles << "x,y,z,v_x,v_y,v_z,|U|,rho,pressure\n";
            } else if (out_length == 2) {
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
            if (out_length == 0) {
                ssotherParticles << pos.x << ", " << pos.y << ", " << pos.z << ", " << velMag + eps << std::endl;
            } else if (out_length == 1) {
                ssotherParticles << pos.x << ", " << pos.y << ", " << pos.z << ", " << vel.x + eps << ", "
                                 << vel.y + eps << ", " << vel.z + eps << ", " << velMag + eps << ", " << rP.x << ", "
                                 << rP.y + eps << std::endl;
            } else if (out_length == 2) {
                ssotherParticles << pos.x << ", " << pos.y << ", " << pos.z << ", " << pos.w << ", " << vel.x + eps
                                 << ", " << vel.y + eps << ", " << vel.z + eps << ", " << velMag + eps << ", " << rP.x
                                 << ", " << rP.y + eps << ", " << rP.z << ", " << stIm.x << ", " << stIm.y << ", "
                                 << stIm.z << ", " << stIm.w << ", " << rP.w << std::endl;
            }
        }

        fileNameOtherParticles << ssotherParticles.str();
        fileNameOtherParticles.close();
    }
    
    // Save fluid/granular SPH particles to files
    std::string nameFluid = dir + "/fluid" + std::to_string(frame_num) + ".csv";

    std::ofstream fileNameFluidParticles;
    fileNameFluidParticles.open(nameFluid);
    std::stringstream ssFluidParticles;
    if (printToParaview) {
        if (out_length == 0) {
            ssFluidParticles << "x,y,z,|U|\n";
        } else if (out_length == 1) {
            ssFluidParticles << "x,y,z,v_x,v_y,v_z,|U|,rho,pressure\n";
        } else if (out_length == 2) {
            ssFluidParticles << "x,y,z,h,v_x,v_y,v_z,|U|,rho(rpx),p(rpy),mu(rpz),sr,tau,I,mu_i,type(rpw)\n";
        }
    }

    for (size_t i = referenceArray[haveHelper + haveGhost].x; i < referenceArray[haveHelper + haveGhost].y; i++) {
        Real4 rP = rhoPresMuH[i];
        if (rP.w != -1)
            continue;
        Real4 pos = posRadH[i];
        Real3 vel = velMasH[i] + mR3(1e-20);
        Real4 stIm = h_sr_tau_I_mu_i[i] + mR4(1e-20);

        Real velMag = length(vel);
        if (out_length == 0) {
            ssFluidParticles << pos.x << ", " << pos.y << ", " << pos.z << ", " << velMag + eps << std::endl;
        } else if (out_length == 1) {
            ssFluidParticles << pos.x << ", " << pos.y << ", " << pos.z << ", " << vel.x + eps << ", "
                             << vel.y + eps << ", " << vel.z + eps << ", " << velMag + eps << ", " << rP.x << ", "
                             << rP.y + eps << std::endl;
        } else if (out_length == 2) {
            ssFluidParticles << pos.x << ", " << pos.y << ", " << pos.z << ", " << pos.w << ", " << vel.x + eps << ", "
                             << vel.y + eps << ", " << vel.z + eps << ", " << velMag + eps << ", " << rP.x << ", "
                             << rP.y + eps << ", " << rP.z << ", " << stIm.x << ", " << stIm.y + eps << ", " << stIm.z
                             << ", " << stIm.w << ", " << rP.w << std::endl;
        }
    }
    fileNameFluidParticles << ssFluidParticles.str();
    fileNameFluidParticles.close();

    // Save boundary BCE particles to files
    if (frame_num == 0) {
        std::string nameFluidBoundaries = dir + "/boundary" + std::to_string(frame_num) + ".csv";

        std::ofstream fileNameFluidBoundaries;
        fileNameFluidBoundaries.open(nameFluidBoundaries);
        std::stringstream ssFluidBoundaryParticles;
        if (printToParaview) {
            if (out_length == 0) {
                ssFluidBoundaryParticles << "x,y,z,|U|\n";
            } else if (out_length == 1) {
                ssFluidBoundaryParticles << "x,y,z,v_x,v_y,v_z,|U|,rho,pressure\n";
            } else if (out_length == 2) {
                ssFluidBoundaryParticles << "x,y,z,h,v_x,v_y,v_z,|U|,rho(rpx),p(rpy),mu(rpz),sr,tau,I,mu_i,type(rpw)\n";
            }
        }

        for (size_t i = referenceArray[haveHelper + haveGhost + 1].x; i < referenceArray[haveHelper + haveGhost + 1].y;
             i++) {
            Real4 rP = rhoPresMuH[i];
            if (rP.w != 0)
                continue;
            Real4 pos = posRadH[i];
            Real3 vel = velMasH[i] + mR3(1e-20);
            Real4 stIm = h_sr_tau_I_mu_i[i] + mR4(1e-20);

            Real velMag = length(vel);
            if (out_length == 0) {
                ssFluidBoundaryParticles << pos.x << ", " << pos.y << ", " << pos.z << ", " << velMag + eps << std::endl;
            } else if (out_length == 1) {
                ssFluidBoundaryParticles << pos.x << ", " << pos.y << ", " << pos.z << ", " << vel.x + eps << ", "
                                         << vel.y + eps << ", " << vel.z + eps << ", " << velMag + eps << ", " << rP.x << ", "
                                         << rP.y + eps << std::endl;
            } else if (out_length == 2) {
                ssFluidBoundaryParticles << pos.x << ", " << pos.y << ", " << pos.z << ", " << pos.w << ", "
                                         << vel.x + eps << ", " << vel.y + eps << ", " << vel.z + eps << ", "
                                         << velMag + eps << ", " << rP.x << ", " << rP.y + eps << ", " << rP.z << ", "
                                         << stIm.x << ", " << stIm.y + eps << ", " << stIm.z << ", " << stIm.w << ", "
                                         << rP.w << std::endl;
            }
        }
        fileNameFluidBoundaries << ssFluidBoundaryParticles.str();
        fileNameFluidBoundaries.close();
    }

    // Save rigid BCE particles to files
    int refSize = (int)referenceArray.size();
    if (refSize > haveHelper + haveGhost + 2) {
        std::string nameBCE = dir + "/BCE_Rigid" + std::to_string(frame_num) + ".csv";

        std::ofstream fileNameBCE;
        fileNameBCE.open(nameBCE);
        std::stringstream ssBCE;
        if (printToParaview) {
            if (out_length == 0) {
                ssBCE << "x,y,z,|U|\n";
            } else if (out_length == 1) {
                ssBCE << "x,y,z,v_x,v_y,v_z,|U|,rho,pressure\n";
            } else if (out_length == 2) {
                ssBCE << "x,y,z,h,v_x,v_y,v_z,|U|,rho(rpx),p(rpy),mu(rpz),sr,tau,I,mu_i,type(rpw)\n";
            }
        }

        for (size_t i = referenceArray[haveHelper + haveGhost + 2].x; i < referenceArray[refSize - 1].y; i++) {
            Real4 pos = posRadH[i];
            Real3 vel = velMasH[i] + mR3(1e-20);
            Real4 rP = rhoPresMuH[i];
            Real velMag = length(vel);
            Real4 stIm = h_sr_tau_I_mu_i[i] + mR4(1e-20);

            if (rP.w == 1.0) {
                if (out_length == 0) {
                    ssBCE << pos.x << ", " << pos.y << ", " << pos.z << ", " << velMag + eps << std::endl;
                } else if (out_length == 1) {
                    ssBCE << pos.x << ", " << pos.y << ", " << pos.z << ", " << vel.x + eps << ", "
                          << vel.y + eps << ", " << vel.z + eps << ", " << velMag + eps << ", " << rP.x << ", "
                          << rP.y + eps << std::endl;
                } else if (out_length == 2) {
                    ssBCE << pos.x << ", " << pos.y << ", " << pos.z << ", " << pos.w << ", " << vel.x + eps << ", "
                          << vel.y + eps << ", " << vel.z + eps << ", " << velMag + eps << ", " << rP.x << ", "
                          << rP.y + eps << ", " << rP.z << ", " << stIm.x << ", " << stIm.y + eps << ", " << stIm.z
                          << ", " << stIm.w << ", " << rP.w << std::endl;
                }
            }
        }
        fileNameBCE << ssBCE.str();
        fileNameBCE.close();
    }

    // Save flexible BCE particles to files
    int refSize_Flex = (int)referenceArrayFEA.size();
    if (refSize_Flex > 0) {
        std::string nameBCE_Flex = dir + "/BCE_Flex" + std::to_string(frame_num) + ".csv";

        std::ofstream fileNameBCE_Flex;
        fileNameBCE_Flex.open(nameBCE_Flex);
        std::stringstream ssBCE_Flex;

        if (printToParaview) {
            if (out_length == 0) {
                ssBCE_Flex << "x,y,z,|U|\n";
            } else if (out_length == 1) {
                ssBCE_Flex << "x,y,z,v_x,v_y,v_z,|U|,rho,pressure\n";
            } else if (out_length == 2) {
                ssBCE_Flex << "x,y,z,h,v_x,v_y,v_z,|U|,rho(rpx),p(rpy),mu(rpz),sr,tau,I,mu_i,type(rpw)\n";
            }
        }
        for (size_t i = referenceArrayFEA[0].x; i < referenceArrayFEA[refSize_Flex - 1].y; i++) {
            Real4 pos = posRadH[i];
            Real3 vel = velMasH[i] + mR3(1e-20);
            Real4 rP = rhoPresMuH[i];
            Real4 stIm = h_sr_tau_I_mu_i[i] + mR4(1e-20);

            Real velMag = length(vel);
            if (out_length == 0) {
                ssBCE_Flex << pos.x << ", " << pos.y << ", " << pos.z << ", " << velMag + eps << std::endl;
            } else if (out_length == 1) {
                ssBCE_Flex << pos.x << ", " << pos.y << ", " << pos.z << ", " << vel.x + eps << ", "
                           << vel.y + eps << ", " << vel.z + eps << ", " << velMag + eps << ", " << rP.x << ", "
                           << rP.y + eps << std::endl;
            } else if (out_length == 2) {
                ssBCE_Flex << pos.x << ", " << pos.y << ", " << pos.z << ", " << pos.w << ", " << vel.x + eps << ", "
                           << vel.y + eps << ", " << vel.z + eps << ", " << velMag + eps << ", " << rP.x << ", "
                           << rP.y + 1e-20 << ", " << rP.z << ", " << stIm.x << ", " << stIm.y + eps << ", " << stIm.z
                           << ", " << stIm.w << ", " << rP.w << std::endl;
            }
        }
        fileNameBCE_Flex << ssBCE_Flex.str();
        fileNameBCE_Flex.close();
    }

    posRadH.clear();
    velMasH.clear();
    rhoPresMuH.clear();
    h_sr_tau_I_mu_i.clear();
}

void WriteCsvParticlesToFile(thrust::device_vector<Real4>& posRadD,
                             thrust::device_vector<Real3>& velMasD,
                             thrust::device_vector<Real4>& rhoPresMuD,
                             thrust::host_vector<int4>& referenceArray,
                             const std::string& outfilename) {
    thrust::host_vector<Real4> posRadH = posRadD;
    thrust::host_vector<Real3> velMasH = velMasD;
    thrust::host_vector<Real4> rhoPresMuH = rhoPresMuD;
    double eps = 1e-20;

    // ======================================================

    bool haveHelper = (referenceArray[0].z == -3) ? true : false;
    bool haveGhost = (referenceArray[0].z == -2 || referenceArray[1].z == -2) ? true : false;

    // ======================================================

    std::ofstream fileNameFluidParticles;
    fileNameFluidParticles.open(outfilename);
    std::stringstream ssFluidParticles;
    ssFluidParticles << "x,y,z,v_x,v_y,v_z,|U|,rho,pressure\n";

    for (size_t i = referenceArray[haveHelper + haveGhost].x; i < referenceArray[haveHelper + haveGhost].y; i++) {
        Real4 rP = rhoPresMuH[i];
        if (rP.w != -1)
            continue;
        Real4 pos = posRadH[i];
        Real3 vel = velMasH[i] + mR3(1e-20);

        Real velMag = length(vel);
        ssFluidParticles << pos.x << ", " << pos.y << ", " << pos.z << ", " << vel.x + eps << ", " << vel.y + eps
                         << ", " << vel.z + eps << ", " << velMag + eps << ", " << rP.x << ", " << rP.y + eps
                         << std::endl;
    }
    fileNameFluidParticles << ssFluidParticles.str();
    fileNameFluidParticles.close();
}

void WriteChPFParticlesToFile(thrust::device_vector<Real4>& posRadD,
                              thrust::host_vector<int4>& referenceArray,
                              const std::string& outfilename) {
    std::ofstream ptFile(outfilename, std::ios::out | std::ios::binary);

    ParticleFormatWriter pw;

    thrust::host_vector<Real4> posRadH = posRadD;
    std::vector<float> pos_x(posRadH.size());
    std::vector<float> pos_y(posRadH.size());
    std::vector<float> pos_z(posRadH.size());

    // ======================================================
    bool haveHelper = (referenceArray[0].z == -3) ? true : false;
    bool haveGhost = (referenceArray[0].z == -2 || referenceArray[1].z == -2) ? true : false;
    // ======================================================

    for (size_t i = referenceArray[haveHelper + haveGhost].x; i < referenceArray[haveHelper + haveGhost].y; i++) {
        pos_x[i] = (float)posRadH[i].x;
        pos_y[i] = (float)posRadH[i].y;
        pos_z[i] = (float)posRadH[i].z;
    }

    pw.write(ptFile, ParticleFormatWriter::CompressionType::NONE, pos_x, pos_y, pos_z);
}

}  // end namespace utils
}  // end namespace fsi
}  // end namespace chrono
