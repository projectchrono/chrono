// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2020 projectchrono.org
// All right reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban
// =============================================================================
//
// Definition of the OpenMP granular TERRAIN NODE (using Chrono::Parallel).
//
// The global reference frame has Z up, X towards the front of the vehicle, and
// Y pointing to the left.
//
// =============================================================================

//// TODO:
////    better approximation of mass / inertia? (CreateProxies)
////    angular velocity (UpdateProxies)
////    implement (PrintProxiesContactData)

#ifndef TESTRIG_TERRAINNODE_GRANULAR_OMP_H
#define TESTRIG_TERRAINNODE_GRANULAR_OMP_H

#include <vector>

#include "chrono/utils/ChUtilsCreators.h"
#include "chrono/utils/ChUtilsGenerators.h"
#include "chrono/utils/ChUtilsInputOutput.h"

#include "chrono_parallel/physics/ChSystemParallel.h"

#include "chrono_vehicle/cosim/ChVehicleCosimTerrainNode.h"

namespace chrono {
namespace vehicle {

class CH_VEHICLE_API ChVehicleCosimTerrainNodeGranularOMP : public ChVehicleCosimTerrainNode {
  public:
    /// Create a Chrono::Parallel granular terrain subsystem.
    ChVehicleCosimTerrainNodeGranularOMP(ChContactMethod method,  ///< contact method (penalty or complementatiry)
                                         bool use_checkpoint,     ///< initialize granular terrain from checkpoint
                                         bool render,             ///< use OpenGL rendering
                                         int num_threads          ///< number of OpenMP threads
    );
    ~ChVehicleCosimTerrainNodeGranularOMP();

    virtual ChSystem* GetSystem() override { return m_system; }

    /// Set properties of granular material.
    void SetGranularMaterial(double radius,   ///< particle radius (default: 0.01)
                             double density,  ///< particle material density (default: 2000)
                             int num_layers   ///< number of generated particle layers (default: 5)
    );

    /// Set the material properties for terrain.
    /// The type of material must be consistent with the contact method (penalty or complementarity)
    /// specified at construction. These parameters characterize the material for the container and
    /// (if applicable) the granular material.  Tire contact material is received from the rig node.
    virtual void SetMaterialSurface(const std::shared_ptr<ChMaterialSurface>& mat) override;

    /// Specify whether contact coefficients are based on material properties (default: true).
    /// Note that this setting is only relevant when using the penalty method.
    virtual void UseMaterialProperties(bool flag) override;

    /// Set the normal contact force model (default: Hertz)
    /// Note that this setting is only relevant when using the penalty method.
    virtual void SetContactForceModel(ChSystemSMC::ContactForceModel model) override;

    /// Set simulation length for settling of granular material (default: 0.4).
    void SetSettlingTime(double time) { m_time_settling = time; }

    /// Enable/disable output during settling (default: false).
    /// If enabled, output files are generated with a frequency of 100 FPS.
    void EnableSettlingOutput(bool val) { m_settling_output = val; }

    /// Obtain settled terrain configuration.
    /// This is an optional operation that a terrain subsystem may perform before initiating
    /// communictation with the rig node. For granular terrain, a settled configuration can
    /// be obtained either through simulation or by initializing particles from a previously
    /// generated checkpointing file.
    virtual void Settle() override;

    /// Write checkpointing file.
    void WriteCheckpoint();

  private:
    ChSystemParallel* m_system;  ///< containing system
    bool m_constructed;          ///< system construction completed?

    bool m_use_checkpoint;         ///< initialize granular terrain from checkpoint file
    int m_Id_g;                    ///< first identifier for granular material bodies
    int m_num_layers;              ///< number of generated particle layers
    unsigned int m_num_particles;  ///< number of granular material bodies
    double m_radius_g;             ///< radius of one particle of granular material
    double m_rho_g;                ///< particle material density

    double m_time_settling;  ///< simulation length for settling of granular material
    bool m_settling_output;  ///< output files during settling?

    int m_particles_start_index;  ///< start index for granular material bodies in system body list

    static const std::string m_checkpoint_filename;  ///< name of checkpointing file

    // Private methods

    virtual void Construct() override;
    virtual void CreateProxies() override;
    virtual void UpdateProxies() override;
    virtual void ForcesProxies(std::vector<double>& vert_forces, std::vector<int>& vert_indices) override;
    virtual void PrintProxiesUpdateData() override;
    virtual void PrintProxiesContactData() override;
    virtual void OutputTerrainData(int frame) override;
    virtual void OnSynchronize(int step_number, double time) override;
    virtual void OnAdvance(double step_size) override;

    void WriteParticleInformation(utils::CSV_writer& csv);

    static ChVector<> CalcBarycentricCoords(const ChVector<>& v1,
                                            const ChVector<>& v2,
                                            const ChVector<>& v3,
                                            const ChVector<>& vP);
};

}  // end namespace vehicle
}  // end namespace chrono

#endif
