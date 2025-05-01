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
// Author: Milad Rakhsha, Arman Pazouki, Wei Hu, Radu Serban
// =============================================================================
//
// Custom FSI interface for coupling the SPH-based fluid system with a Chrono MBS
// =============================================================================

////#define DEBUG_LOG

#include "chrono/utils/ChUtils.h"
#include "chrono_fsi/sph/utils/UtilsPrintSph.cuh"

#include "chrono_fsi/sph/ChFsiInterfaceSPH.h"
#include "chrono_fsi/sph/physics/FsiDataManager.cuh"
#include "chrono_fsi/sph/physics/FsiInterface.cuh"
#include "chrono_fsi/sph/utils/UtilsDevice.cuh"
#include "chrono_fsi/sph/utils/UtilsTypeConvert.cuh"

namespace chrono {
namespace fsi {
namespace sph {

ChFsiInterfaceSPH::ChFsiInterfaceSPH(ChSystem& sysMBS, ChFsiFluidSystemSPH& sysSPH)
    : ChFsiInterface(sysMBS, sysSPH), m_data_mgr(sysSPH.m_data_mgr.get()) {}

ChFsiInterfaceSPH::~ChFsiInterfaceSPH() {}

void ChFsiInterfaceSPH::ExchangeSolidStates() {
    {
        // Load from rigid bodies on host
        int index = 0;
        for (const auto& fsi_body : m_fsi_bodies) {
            m_data_mgr->fsiBodyState_H->pos[index] = ToReal3(fsi_body.body->GetPos());
            m_data_mgr->fsiBodyState_H->lin_vel[index] = ToReal3(fsi_body.body->GetPosDt());
            m_data_mgr->fsiBodyState_H->lin_acc[index] = ToReal3(fsi_body.body->GetPosDt2());
            m_data_mgr->fsiBodyState_H->rot[index] = ToReal4(fsi_body.body->GetRot());
            m_data_mgr->fsiBodyState_H->ang_vel[index] = ToReal3(fsi_body.body->GetAngVelLocal());
            m_data_mgr->fsiBodyState_H->ang_acc[index] = ToReal3(fsi_body.body->GetAngAccLocal());
            index++;
        }

        // Transfer to device
        m_data_mgr->fsiBodyState_D->CopyFromH(*m_data_mgr->fsiBodyState_H);
    }

    {
        // Load from FEA nodes on host
        int index = 0;
        for (const auto& fsi_mesh : m_fsi_meshes1D) {
            ////int num_nodes = (int)fsi_mesh.ind2ptr_map.size();
            for (const auto& node : fsi_mesh.ind2ptr_map) {
                m_data_mgr->fsiMesh1DState_H->pos[index] = ToReal3(node.second->GetPos());
                m_data_mgr->fsiMesh1DState_H->vel[index] = ToReal3(node.second->GetPosDt());
                m_data_mgr->fsiMesh1DState_H->acc[index] = ToReal3(node.second->GetPosDt2());
                index++;
            }
        }

        // Transfer to device
        m_data_mgr->fsiMesh1DState_D->CopyFromH(*m_data_mgr->fsiMesh1DState_H);
        if (m_use_node_directions) {
            calculateDirectionsMesh1D(*m_data_mgr);
            ChDebugLog("Calculate directions");
#ifdef DEBUG_LOG
            printDirectionsMesh1D(*m_data_mgr);
#endif
        }
    }

    {
        // Load from FEA nodes on host
        int index = 0;
        for (const auto& fsi_mesh : m_fsi_meshes2D) {
            ////int num_nodes = (int)fsi_mesh.ind2ptr_map.size();
            for (const auto& node : fsi_mesh.ind2ptr_map) {
                m_data_mgr->fsiMesh2DState_H->pos[index] = ToReal3(node.second->GetPos());
                m_data_mgr->fsiMesh2DState_H->vel[index] = ToReal3(node.second->GetPosDt());
                m_data_mgr->fsiMesh2DState_H->acc[index] = ToReal3(node.second->GetPosDt2());
                index++;
            }
        }

        // Transfer to device
        m_data_mgr->fsiMesh2DState_D->CopyFromH(*m_data_mgr->fsiMesh2DState_H);
        if (m_use_node_directions)
            calculateDirectionsMesh2D(*m_data_mgr);
    }
}

void ChFsiInterfaceSPH::ExchangeSolidForces() {
    {
        // Transfer to host
        auto forcesH = m_data_mgr->GetRigidForces();
        auto torquesH = m_data_mgr->GetRigidTorques();

        // Apply to rigid bodies
        int index = 0;
        for (const auto& fsi_body : m_fsi_bodies) {
            m_fsi_bodies[index].fsi_force = ToChVector(forcesH[index]);
            m_fsi_bodies[index].fsi_torque = ToChVector(torquesH[index]);
            fsi_body.body->EmptyAccumulator(fsi_body.fsi_accumulator);
            fsi_body.body->AccumulateForce(fsi_body.fsi_accumulator, m_fsi_bodies[index].fsi_force,
                                           fsi_body.body->GetPos(), false);
            fsi_body.body->AccumulateTorque(fsi_body.fsi_accumulator, m_fsi_bodies[index].fsi_torque, false);
            index++;
        }
    }

    {
        // Transfer to host
        auto forces_H = m_data_mgr->GetFlex1dForces();

        // Apply to FEA 1-D mesh nodes
        int index = 0;
        for (const auto& fsi_mesh : m_fsi_meshes1D) {
            ////int num_nodes = (int)fsi_mesh.ind2ptr_map.size();
            for (auto& node : fsi_mesh.ind2ptr_map) {
                node.second->SetForce(ToChVector(forces_H[index]));
                index++;
            }
        }
    }

    {
        // Transfer to host
        auto forces_H = m_data_mgr->GetFlex2dForces();

        // Apply to FEA 2-D mesh nodes
        int index = 0;
        for (const auto& fsi_mesh : m_fsi_meshes2D) {
            ////int num_nodes = (int)fsi_mesh.ind2ptr_map.size();
            for (auto& node : fsi_mesh.ind2ptr_map) {
                node.second->SetForce(ToChVector(forces_H[index]));
                index++;
            }
        }
    }
}

}  // namespace sph
}  // end namespace fsi
}  // end namespace chrono
