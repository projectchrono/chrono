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
// Author: Arman Pazouki, Milad Rakhsha, Wei Hu, Radu Serban
// =============================================================================
//
// Utility function to print the save fluid, bce, and boundary data to files
// =============================================================================

#include <cstdio>
#include <cstring>
#include <fstream>
#include <sstream>
#include <thread>

#include <thrust/reduce.h>

#include "chrono_fsi/sph/utils/UtilsDevice.cuh"
#include "chrono_fsi/sph/utils/UtilsPrintSph.cuh"

namespace chrono {
namespace fsi {
namespace sph {

// -----------------------------------------------------------------------------
// CFD particle data

// Worker function to output CFD information for markers in specified range.
void SaveFileCFD(const std::string& filename,
                 OutputLevel level,
                 thrust::host_vector<Real4> pos,
                 thrust::host_vector<Real3> vel,
                 thrust::host_vector<Real4> acc,
                 thrust::host_vector<Real4> rhoPresMu,
                 thrust::host_vector<Real4> srTauMu,
                 int start_index,
                 int end_index) {
    std::ofstream file(filename);
    std::stringstream sstream;

    switch (level) {
        case OutputLevel::STATE:
            sstream << "x,y,z,|U|,acc\n";
            break;
        case OutputLevel::STATE_PRESSURE:
            sstream << "x,y,z,v_x,v_y,v_z,|U|,acc,rho,pressure\n";
            break;
        case OutputLevel::CRM_FULL:
            sstream << "x,y,z,h,v_x,v_y,v_z,|U|,acc,rho(rpx),p11(tauXxYyZz_11),p22(tauXxYyZz_22),p33("
                       "tauXxYyZz_33),shear12(tauXyXzYz_12),shear13(tauXyXzYz_13),shear23(tauXyXzYz_23)\n";
            break;
    }

    for (size_t i = start_index; i < end_index; i++) {
        Real4 p = pos[i];
        Real3 v = vel[i];
        Real3 a = mR3(acc[i]);
        Real4 rp = rhoPresMu[i];
        Real4 st = srTauMu[i];

        Real v_len = length(v);
        Real a_len = length(a);

        switch (level) {
            case OutputLevel::STATE:
                sstream << p.x << ", " << p.y << ", " << p.z << ", " << v_len << ", " << a_len << std::endl;
                break;
            case OutputLevel::STATE_PRESSURE:
                sstream << p.x << ", " << p.y << ", " << p.z << ", " << v.x << ", " << v.y << ", " << v.z << ", "
                        << v_len << ", " << a_len << ", " << rp.x << ", " << rp.y << std::endl;
                break;
            case OutputLevel::CRM_FULL:
                sstream << p.x << ", " << p.y << ", " << p.z << ", " << p.w << ", " << v.x << ", " << v.y << ", " << v.z
                        << ", " << v_len << ", " << a_len << ", " << rp.x << ", " << rp.y << ", " << rp.z << ", "
                        << st.x << ", " << st.y << ", " << st.z << ", " << st.w << ", " << rp.w << std::endl;
                break;
        }
    }

    file << sstream.str();
    file.close();
}

// Worker function to output CFD files at current frame.
void SaveAllCFD(const std::string& dir,
                int frame,
                OutputLevel level,
                thrust::host_vector<Real4> pos,
                thrust::host_vector<Real3> vel,
                thrust::host_vector<Real4> acc,
                thrust::host_vector<Real4> rhoPresMu,
                thrust::host_vector<Real4> srTauMu,
                const thrust::host_vector<int4>& referenceArray,
                const thrust::host_vector<int4>& referenceArrayFEA) {
    bool haveHelper = (referenceArray[0].z == -3) ? true : false;
    bool haveGhost = (referenceArray[0].z == -2 || referenceArray[1].z == -2) ? true : false;

    // Save helper and ghost particles to files
    if (haveHelper || haveGhost) {
        std::string filename = dir + "/others" + std::to_string(frame) + ".csv";
        SaveFileCFD(filename, level,                                                 //
                    pos, vel, acc, rhoPresMu, srTauMu,                               //
                    referenceArray[0].x, referenceArray[haveHelper + haveGhost].y);  //
    }

    // Save fluid/granular SPH particles to files
    {
        std::string filename = dir + "/fluid" + std::to_string(frame) + ".csv";
        SaveFileCFD(filename, level,                                                                    //
                    pos, vel, acc, rhoPresMu, srTauMu,                                                  //
                    referenceArray[haveHelper + haveGhost].x, referenceArray[haveHelper + haveGhost].y  //
        );
    }

    // Save boundary BCE particles to files
    if (frame == 0) {
        std::string filename = dir + "/boundary" + std::to_string(frame) + ".csv";
        SaveFileCFD(filename, level,                                                                              //
                    pos, vel, acc, rhoPresMu, srTauMu,                                                            //
                    referenceArray[haveHelper + haveGhost + 1].x, referenceArray[haveHelper + haveGhost + 1].y);  //
    }

    // Save rigid BCE particles to files
    int refSize = (int)referenceArray.size();
    if (refSize > haveHelper + haveGhost + 2 && referenceArray[2].z == 1) {
        std::string filename = dir + "/rigidBCE" + std::to_string(frame) + ".csv";
        SaveFileCFD(filename, level,                                                               //
                    pos, vel, acc, rhoPresMu, srTauMu,                                             //
                    referenceArray[haveHelper + haveGhost + 2].x, referenceArray[refSize - 1].y);  //
    }

    // Save flexible BCE particles to files
    int refSize_Flex = (int)referenceArrayFEA.size();
    if (refSize_Flex > 0) {
        std::string filename = dir + "/flexBCE" + std::to_string(frame) + ".csv";
        SaveFileCFD(filename, level,                                                 //
                    pos, vel, acc, rhoPresMu, srTauMu,                               //
                    referenceArrayFEA[0].x, referenceArrayFEA[refSize_Flex - 1].y);  //
    }
}

void saveParticleDataCFD(const std::string& dir, OutputLevel level, FsiDataManager& data_mgr) {
    saveParticleDataCFD(dir, level,                                                        //
                        data_mgr.sphMarkers_D->posRadD, data_mgr.sphMarkers_D->velMasD,    //
                        data_mgr.derivVelRhoOriginalD, data_mgr.sphMarkers_D->rhoPresMuD,  //
                        data_mgr.sr_tau_I_mu_i_Original,                                   //
                        data_mgr.referenceArray, data_mgr.referenceArray_FEA);             //
}

void saveParticleDataCFD(const std::string& dir,
                         OutputLevel level,
                         const thrust::device_vector<Real4>& posRadD,
                         const thrust::device_vector<Real3>& velMasD,
                         const thrust::device_vector<Real4>& derivVelRhoD,
                         const thrust::device_vector<Real4>& rhoPresMuD,
                         const thrust::device_vector<Real4>& srTauMuD,
                         const thrust::host_vector<int4>& referenceArray,
                         const thrust::host_vector<int4>& referenceArrayFEA) {
    thrust::host_vector<Real4> pos = posRadD;
    thrust::host_vector<Real3> vel = velMasD;
    thrust::host_vector<Real4> acc = derivVelRhoD;
    thrust::host_vector<Real4> rhoPresMu = rhoPresMuD;
    thrust::host_vector<Real4> srTauMu = srTauMuD;

    // Current frame number
    static int frame = -1;
    frame++;

    // Start printing in a separate thread and detach the thread to allow independent execution
    std::thread th(SaveAllCFD,                         //
                   dir, frame, level,                  //
                   pos, vel, acc, rhoPresMu, srTauMu,  //
                   referenceArray, referenceArrayFEA   //
    );
    th.detach();
}

// -----------------------------------------------------------------------------
// CRM particle data

// Worker function to output CRM information for markers in specified range.
void SaveFileCRM(const std::string& filename,
                 OutputLevel level,
                 thrust::host_vector<Real4> pos,
                 thrust::host_vector<Real3> vel,
                 thrust::host_vector<Real4> acc,
                 thrust::host_vector<Real4> rhoPresMu,
                 thrust::host_vector<Real3> tau_normal,
                 thrust::host_vector<Real3> tau_shear,
                 int start_index,
                 int end_index) {
    std::ofstream file(filename);
    std::stringstream sstream;

    switch (level) {
        case OutputLevel::STATE:
            sstream << "x,y,z,|U|,acc\n";
            break;
        case OutputLevel::STATE_PRESSURE:
            sstream << "x,y,z,v_x,v_y,v_z,|U|,acc,rho,pressure\n";
            break;
        case OutputLevel::CRM_FULL:
            sstream << "x,y,z,h,v_x,v_y,v_z,|U|,acc,rho(rpx),p11(tauXxYyZz_11),p22(tauXxYyZz_22),p33("
                       "tauXxYyZz_33),shear12(tauXyXzYz_12),shear13(tauXyXzYz_13),shear23(tauXyXzYz_23)\n";
            break;
    }

    for (size_t i = start_index; i < end_index; i++) {
        Real4 p = pos[i];
        Real3 v = vel[i];
        Real3 a = mR3(acc[i]);
        Real4 rp = rhoPresMu[i];
        Real3 tn = tau_normal[i];
        Real3 ts = tau_shear[i];

        Real v_len = length(v);
        Real a_len = length(a);

        switch (level) {
            case OutputLevel::STATE:
                sstream << p.x << ", " << p.y << ", " << p.z << ", " << v_len << ", " << a_len << std::endl;
                break;
            case OutputLevel::STATE_PRESSURE:
                sstream << p.x << ", " << p.y << ", " << p.z << ", " << v.x << ", " << v.y << ", " << v.z << ", "
                        << v_len << ", " << a_len << ", " << rp.x << ", " << rp.y << std::endl;
                break;
            case OutputLevel::CRM_FULL:
                sstream << p.x << ", " << p.y << ", " << p.z << ", " << p.w << ", " << v.x << ", " << v.y << ", " << v.z
                        << ", " << v_len << ", " << a_len << ", " << rp.x << ", " << tn.x << ", " << tn.y << ", "
                        << tn.z << ", " << ts.x << ", " << ts.y << ", " << ts.z << std::endl;
                break;
        }
    }

    file << sstream.str();
    file.close();
}

// Worker function to output CRM files at current frame.
void SaveAllCRM(const std::string& dir,
                int frame,
                OutputLevel level,
                thrust::host_vector<Real4> pos,
                thrust::host_vector<Real3> vel,
                thrust::host_vector<Real4> acc,
                thrust::host_vector<Real4> rhoPresMu,
                thrust::host_vector<Real3> tau_normal,
                thrust::host_vector<Real3> tau_shear,
                const thrust::host_vector<int4>& referenceArray,
                const thrust::host_vector<int4>& referenceArrayFEA) {
    bool haveHelper = (referenceArray[0].z == -3) ? true : false;
    bool haveGhost = (referenceArray[0].z == -2 || referenceArray[1].z == -2) ? true : false;

    // Save helper and ghost particles to files
    if (haveHelper || haveGhost) {
        std::string filename = dir + "/others" + std::to_string(frame) + ".csv";
        SaveFileCRM(filename, level,                                                 //
                    pos, vel, acc, rhoPresMu, tau_normal, tau_shear,                 //
                    referenceArray[0].x, referenceArray[haveHelper + haveGhost].y);  //
    }

    // Save fluid/granular SPH particles to files
    {
        std::string filename = dir + "/fluid" + std::to_string(frame) + ".csv";
        SaveFileCRM(filename, level,                                                                    //
                    pos, vel, acc, rhoPresMu, tau_normal, tau_shear,                                    //
                    referenceArray[haveHelper + haveGhost].x, referenceArray[haveHelper + haveGhost].y  //
        );
    }

    // Save boundary BCE particles to files
    if (frame == 0) {
        std::string filename = dir + "/boundary" + std::to_string(frame) + ".csv";
        SaveFileCRM(filename, level,                                                                              //
                    pos, vel, acc, rhoPresMu, tau_normal, tau_shear,                                              //
                    referenceArray[haveHelper + haveGhost + 1].x, referenceArray[haveHelper + haveGhost + 1].y);  //
    }

    // Save rigid BCE particles to files
    int refSize = (int)referenceArray.size();
    if (refSize > haveHelper + haveGhost + 2 && referenceArray[2].z == 1) {
        std::string filename = dir + "/rigidBCE" + std::to_string(frame) + ".csv";
        SaveFileCRM(filename, level,                                                               //
                    pos, vel, acc, rhoPresMu, tau_normal, tau_shear,                               //
                    referenceArray[haveHelper + haveGhost + 2].x, referenceArray[refSize - 1].y);  //
    }

    // Save flexible BCE particles to files
    int refSize_Flex = (int)referenceArrayFEA.size();
    if (refSize_Flex > 0) {
        std::string filename = dir + "/flexBCE" + std::to_string(frame) + ".csv";
        SaveFileCRM(filename, level,                                                 //
                    pos, vel, acc, rhoPresMu, tau_normal, tau_shear,                 //
                    referenceArrayFEA[0].x, referenceArrayFEA[refSize_Flex - 1].y);  //
    }
}

void saveParticleDataCRM(const std::string& dir, OutputLevel level, FsiDataManager& data_mgr) {
    saveParticleDataCRM(dir, level,                                                            //
                        data_mgr.sphMarkers_D->posRadD, data_mgr.sphMarkers_D->velMasD,        //
                        data_mgr.derivVelRhoOriginalD, data_mgr.sphMarkers_D->rhoPresMuD,      //
                        data_mgr.sphMarkers_D->tauXxYyZzD, data_mgr.sphMarkers_D->tauXyXzYzD,  //
                        data_mgr.referenceArray, data_mgr.referenceArray_FEA);                 //
}

void saveParticleDataCRM(const std::string& dir,
                         OutputLevel level,
                         const thrust::device_vector<Real4>& posRadD,
                         const thrust::device_vector<Real3>& velMasD,
                         const thrust::device_vector<Real4>& derivVelRhoD,
                         const thrust::device_vector<Real4>& rhoPresMuD,
                         const thrust::device_vector<Real3>& tauXxYyZzD,
                         const thrust::device_vector<Real3>& tauXyXzYzD,
                         const thrust::host_vector<int4>& referenceArray,
                         const thrust::host_vector<int4>& referenceArrayFEA) {
    thrust::host_vector<Real4> pos = posRadD;
    thrust::host_vector<Real3> vel = velMasD;
    thrust::host_vector<Real4> acc = derivVelRhoD;
    thrust::host_vector<Real4> rhoPresMu = rhoPresMuD;
    thrust::host_vector<Real3> tau_normal = tauXxYyZzD;
    thrust::host_vector<Real3> tau_shear = tauXyXzYzD;

    // Current frame number
    static int frame = -1;
    frame++;

    // Start printing in a separate thread and detach the thread to allow independent execution
    std::thread th(SaveAllCRM,                                       //
                   dir, frame, level,                                //
                   pos, vel, acc, rhoPresMu, tau_normal, tau_shear,  //
                   referenceArray, referenceArrayFEA);               //
    th.detach();
}

// -----------------------------------------------------------------------------
// FSI solids data

// Worker function to write current data for all FSI solids.
void SaveAllSolid(const std::string& dir,
                  const std::string& delim,
                  double time,
                  thrust::host_vector<Real3> posRigid,
                  thrust::host_vector<Real4> rotRigid,
                  thrust::host_vector<Real3> velRigid,
                  thrust::host_vector<Real3> forceRigid,
                  thrust::host_vector<Real3> torqueRigid,
                  thrust::host_vector<Real3> pos1DNode,
                  thrust::host_vector<Real3> vel1DNode,
                  thrust::host_vector<Real3> force1DNode,
                  thrust::host_vector<Real3> pos2DNode,
                  thrust::host_vector<Real3> vel2DNode,
                  thrust::host_vector<Real3> force2DNode) {
    // Number of rigids, 1D nodes, and 2D nodes
    size_t numRigids = posRigid.size();
    size_t numNodes1D = pos1DNode.size();
    size_t numNodes2D = pos2DNode.size();

    // Write information for FSI rigid bodies
    for (size_t i = 0; i < numRigids; i++) {
        Real3 pos = posRigid[i];
        Real4 rot = rotRigid[i];
        Real3 vel = velRigid[i];
        Real3 force = forceRigid[i];
        Real3 torque = torqueRigid[i];

        std::string filename = dir + "/FSI_body" + std::to_string(i) + ".csv";
        std::ofstream file(filename, std::fstream::app);
        file << time << delim << pos.x << delim << pos.y << delim << pos.z << delim << rot.x << delim << rot.y << delim
             << rot.z << delim << rot.w << delim << vel.x << delim << vel.y << delim << vel.z << delim << force.x
             << delim << force.y << delim << force.z << delim << torque.x << delim << torque.y << delim << torque.z
             << std::endl;
        file.close();
    }

    // Write information for FSI flexible 1-D nodes
    for (size_t i = 0; i < numNodes1D; i++) {
        Real3 pos = pos1DNode[i];
        Real3 vel = vel1DNode[i];
        Real3 force = force1DNode[i];

        std::string filename = dir + "/FSI_1Dnode" + std::to_string(i) + ".csv";
        std::ofstream file(filename, std::fstream::app);
        file << time << delim << pos.x << delim << pos.y << delim << pos.z << delim << vel.x << delim << vel.y << delim
             << vel.z << delim << force.x << delim << force.y << delim << force.z << std::endl;
        file.close();
    }

    // Write information for FSI flexible 2-D nodes
    for (size_t i = 0; i < numNodes2D; i++) {
        Real3 pos = pos2DNode[i];
        Real3 vel = vel2DNode[i];
        Real3 force = force2DNode[i];

        std::string filename = dir + "/FSI_2Dnode" + std::to_string(i) + ".csv";
        std::ofstream file(filename, std::fstream::app);
        file << time << delim << pos.x << delim << pos.y << delim << pos.z << delim << vel.x << delim << vel.y << delim
             << vel.z << delim << force.x << delim << force.y << delim << force.z << std::endl;
        file.close();
    }
}

void saveSolidData(const std::string& dir, double time, FsiDataManager& data_mgr) {
    saveSolidData(dir, time,                                                       //
                  data_mgr.fsiBodyState_D->pos, data_mgr.fsiBodyState_D->rot,      //
                  data_mgr.fsiBodyState_D->lin_vel,                                //
                  data_mgr.rigid_FSI_ForcesD, data_mgr.rigid_FSI_TorquesD,         //
                  data_mgr.fsiMesh1DState_D->pos, data_mgr.fsiMesh1DState_D->vel,  //
                  data_mgr.flex1D_FSIforces_D,                                     //
                  data_mgr.fsiMesh2DState_D->pos, data_mgr.fsiMesh2DState_D->vel,  //
                  data_mgr.flex2D_FSIforces_D);                                    //
}

void saveSolidData(const std::string& dir,
                   double time,
                   const thrust::device_vector<Real3>& posRigidD,
                   const thrust::device_vector<Real4>& rotRigidD,
                   const thrust::device_vector<Real3>& velRigidD,
                   const thrust::device_vector<Real3>& forceRigidD,
                   const thrust::device_vector<Real3>& torqueRigidD,
                   const thrust::device_vector<Real3>& pos1DNodeD,
                   const thrust::device_vector<Real3>& vel1DNodeD,
                   const thrust::device_vector<Real3>& force1DNodeD,
                   const thrust::device_vector<Real3>& pos2DNodeD,
                   const thrust::device_vector<Real3>& vel2DNodeD,
                   const thrust::device_vector<Real3>& force2DNodeD) {
    const std::string delim = ",";

    // Copy data arrays to host
    thrust::host_vector<Real3> posRigidH = posRigidD;
    thrust::host_vector<Real3> velRigidH = velRigidD;
    thrust::host_vector<Real4> rotRigidH = rotRigidD;
    thrust::host_vector<Real3> forceRigidH = forceRigidD;
    thrust::host_vector<Real3> torqueRigidH = torqueRigidD;

    thrust::host_vector<Real3> pos1DNodeH = pos1DNodeD;
    thrust::host_vector<Real3> vel1DNodeH = vel1DNodeD;
    thrust::host_vector<Real3> force1DNodeH = force1DNodeD;

    thrust::host_vector<Real3> pos2DNodeH = pos2DNodeD;
    thrust::host_vector<Real3> vel2DNodeH = vel2DNodeD;
    thrust::host_vector<Real3> force2DNodeH = force2DNodeD;

    // Number of rigids, 1D nodes, and 2D nodes
    size_t numRigids = posRigidH.size();
    size_t numNodes1D = pos1DNodeH.size();
    size_t numNodes2D = pos2DNodeH.size();

    // Create files if needed
    static bool create_files = true;
    if (create_files) {
        for (size_t i = 0; i < numRigids; i++) {
            std::string filename = dir + "/FSI_body" + std::to_string(i) + ".csv";
            std::ofstream file(filename);
            file << "Time" << delim << "x" << delim << "y" << delim << "z" << delim << "q0" << delim << "q1" << delim
                 << "q2" << delim << "q3" << delim << "Vx" << delim << "Vy" << delim << "Vz" << delim << "Fx" << delim
                 << "Fy" << delim << "Fz" << delim << "Tx" << delim << "Ty" << delim << "Tz" << std::endl;
            file.close();
        }
        for (size_t i = 0; i < numNodes1D; i++) {
            std::string filename = dir + "/FSI_1Dnode" + std::to_string(i) + ".csv";
            std::ofstream file(filename);
            file << "Time" << delim << "x" << delim << "y" << delim << "z" << delim << "Vx" << delim << "Vy" << delim
                 << "Vz" << delim << "Fx" << delim << "Fy" << delim << "Fz" << std::endl;
            file.close();
        }
        for (size_t i = 0; i < numNodes2D; i++) {
            std::string filename = dir + "/FSI_2Dnode" + std::to_string(i) + ".csv";
            std::ofstream file(filename);
            file << "Time" << delim << "x" << delim << "y" << delim << "z" << delim << "Vx" << delim << "Vy" << delim
                 << "Vz" << delim << "Fx" << delim << "Fy" << delim << "Fz" << std::endl;
            file.close();
        }
    }
    create_files = false;

    // Start printing in a separate thread and detach the thread to allow independent execution
    std::thread th(SaveAllSolid,                                                //
                   dir, delim, time,                                            //
                   posRigidH, rotRigidH, velRigidH, forceRigidH, torqueRigidH,  //
                   pos1DNodeH, vel1DNodeH, force1DNodeH,                        //
                   pos2DNodeH, vel2DNodeH, force2DNodeH);                       //
    th.detach();
}

// -----------------------------------------------------------------------------

void writeParticleFileCSV(const std::string& filename, FsiDataManager& data_mgr) {
    writeParticleFileCSV(filename, data_mgr.sphMarkers_D->posRadD, data_mgr.sphMarkers_D->velMasD,
                         data_mgr.sphMarkers_D->rhoPresMuD, data_mgr.referenceArray);
}

void writeParticleFileCSV(const std::string& outfilename,
                          thrust::device_vector<Real4>& posRadD,
                          thrust::device_vector<Real3>& velMasD,
                          thrust::device_vector<Real4>& rhoPresMuD,
                          thrust::host_vector<int4>& referenceArray) {
    thrust::host_vector<Real4> posRadH = posRadD;
    thrust::host_vector<Real3> velMasH = velMasD;
    thrust::host_vector<Real4> rhoPresMuH = rhoPresMuD;
    double eps = 1e-20;

    bool haveHelper = (referenceArray[0].z == -3) ? true : false;
    bool haveGhost = (referenceArray[0].z == -2 || referenceArray[1].z == -2) ? true : false;

    std::ofstream fileNameFluidParticles;
    fileNameFluidParticles.open(outfilename);
    std::stringstream ssFluidParticles;
    ssFluidParticles << "x,y,z,v_x,v_y,v_z,|U|,rho,pressure\n";

    for (size_t i = referenceArray[haveHelper + haveGhost].x; i < referenceArray[haveHelper + haveGhost].y; i++) {
        Real4 rP = rhoPresMuH[i];
        if (rP.w != -1)
            continue;
        Real4 pos = posRadH[i];
        Real3 vel = velMasH[i] + mR3(Real(1e-20));

        Real velMag = length(vel);
        ssFluidParticles << pos.x << ", " << pos.y << ", " << pos.z << ", " << vel.x + eps << ", " << vel.y + eps
                         << ", " << vel.z + eps << ", " << velMag + eps << ", " << rP.x << ", " << rP.y + eps
                         << std::endl;
    }
    fileNameFluidParticles << ssFluidParticles.str();
    fileNameFluidParticles.close();
}

}  // end namespace sph
}  // end namespace fsi
}  // end namespace chrono
