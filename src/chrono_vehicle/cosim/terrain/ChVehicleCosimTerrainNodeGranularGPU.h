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
// Definition of the GPU granular TERRAIN NODE (using Chrono::Gpu).
//
// The global reference frame has Z up, X towards the front of the vehicle, and
// Y pointing to the left.
//
// =============================================================================

#ifndef TESTRIG_TERRAIN_NODE_GRANULAR_GPU_H
#define TESTRIG_TERRAIN_NODE_GRANULAR_GPU_H

#include "chrono/ChConfig.h"
#include "chrono/physics/ChSystemSMC.h"
#include "chrono/utils/ChUtilsSamplers.h"
#include "chrono/assets/ChVisualSystem.h"
#include "chrono_gpu/physics/ChSystemGpu.h"

#include "chrono_vehicle/cosim/terrain/ChVehicleCosimTerrainNodeChrono.h"

#include "chrono_thirdparty/rapidjson/document.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle_cosim_chrono
/// @{

/// Definition of the GPU granular terrain node (using Chrono::Gpu).
class CH_VEHICLE_API ChVehicleCosimTerrainNodeGranularGPU : public ChVehicleCosimTerrainNodeChrono {
  public:
    /// Create a Chrono::Granular terrain node.
    ChVehicleCosimTerrainNodeGranularGPU(double length, double width);

    /// Create a Chrono::Granular terrain node and set parameters from the provided JSON specfile.
    ChVehicleCosimTerrainNodeGranularGPU(const std::string& specfile);

    ~ChVehicleCosimTerrainNodeGranularGPU();

    virtual ChSystem* GetSystem() override { return m_system; }

    /// Set full terrain specification from JSON specfile.
    void SetFromSpecfile(const std::string& specfile);

    /// Set properties of granular material.
    void SetGranularMaterial(double radius,  ///< particle radius (default: 0.01)
                             double density  ///< particle material density (default: 2000)
    );

    /// Set the material properties for terrain.
    /// These parameters characterize the material for the container and the granular material.
    /// Object contact material is received from the rig node.
    void SetMaterialSurface(const std::shared_ptr<ChMaterialSurfaceSMC>& mat);

    /// Set the normal contact force model (default: Hertz).
    ////void SetContactForceModel(ChSystemSMC::ContactForceModel model);

    /// Set the tangential contact displacement model (default: SINGLE_STEP).
    void SetTangentialDisplacementModel(gpu::CHGPU_FRICTION_MODE model);

    /// Set sampling method for generation of granular material.
    /// The granular material is created in the volume defined by the x-y dimensions of the terrain patch and the
    /// specified initial height, using the specified sampling type, layer by layer or all at once.
    /// Note: for correct HCP, do not initialize in layers!
    void SetSamplingMethod(utils::SamplingType type,   ///< volume sampling type (default POISSON_DISK)
                           double init_height,         ///< height of granular material at initialization (default 0.2)
                           double sep_factor = 1.001,  ///< radius inflation factor for initial separation
                           bool in_layers = false      ///< initialize material in layers
    );

    /// Set the integrator type (default: CENTERED_DIFFERENCE)
    void SetIntegratorType(gpu::CHGPU_TIME_INTEGRATOR type);

    /// Initialize granular terrain from the specified checkpoint file (which must exist in the output directory).
    /// By default, particles are created uniformly distributed in the specified domain such that they are initially not
    /// in contact.
    void SetInputFromCheckpoint(const std::string& filename);

    /// Set simulation length for settling of granular material (default: 0.4).
    void SetSettlingTime(double time);

    /// Set total kinetic energy threshold as stopping criteria for settling (default: 1e-3).
    void SetSettlingKineticEneryThreshold(double threshold);

    /// Enable/disable output during settling (default: false).
    /// If enabled, output files are generated with the specified frequency.
    void EnableSettlingOutput(bool output, double output_fps = 100);

    /// Obtain settled terrain configuration.
    /// This is an optional operation that can be performed for granular terrain before initiating
    /// communictation with the rig node. For granular terrain, a settled configuration can
    /// be obtained either through simulation or by initializing particles from a previously
    /// generated checkpointing file.
    void Settle();

    /// Initialize this Chrono terrain node.
    /// Construct the terrain system and the proxy bodies, then finalize the underlying system.
    virtual void OnInitialize(unsigned int num_objects) override;

    /// Write checkpoint to the specified file (which will be created in the output directory).
    virtual void WriteCheckpoint(const std::string& filename) const override;

    /// Estimate packing density (eta) of granular material in current configuration.
    /// Note that porosity is phi=1-eta and void ratio is e=(1-eta)/eta=phi/(1-phi).
    /// The function also returns the current depth of granular material.
    double CalculatePackingDensity(double& depth);

    /// Output post-processing visualization data.
    virtual void OutputVisualizationData(int frame) override final;

  private:
    ChSystemSMC* m_system;              ///< system for proxy bodies
    gpu::ChSystemGpuMesh* m_systemGPU;  ///< Chrono::Gpu system
    bool m_constructed;                 ///< system construction completed?

    std::shared_ptr<ChVisualSystem> m_vsys;  ///< run-time visualization system

    gpu::CHGPU_TIME_INTEGRATOR m_integrator_type;
    gpu::CHGPU_FRICTION_MODE m_tangential_model;

    utils::SamplingType m_sampling_type;  ///< sampling method for generation of particles
    double m_init_depth;                  ///< height of granular maerial initialization volume
    double m_separation_factor;           ///< radius inflation factor for initial particle separation
    bool m_in_layers;                     ///< initialize material layer-by-layer (true) or all at once (false)

    bool m_use_checkpoint;              ///< initialize granular terrain from checkpoint file
    std::string m_checkpoint_filename;  ///< name of input checkpoint file

    unsigned int m_num_particles;  ///< number of granular material bodies
    double m_radius_g;             ///< radius of one particle of granular material
    double m_rho_g;                ///< particle material density

    bool m_fixed_settling_duration;  ///< flag controlling settling stop criteria
    double m_time_settling;          ///< simulation length for settling of granular material
    double m_KE_settling;            ///< threshold total kinetic energy for stopping settling
    bool m_settling_output;          ///< output files during settling?
    double m_settling_fps;           ///< frequency of output during settling phase

    virtual ChSystem* GetSystemPostprocess() const override { return m_system; }

    virtual bool SupportsMeshInterface() const override { return false; }

    /// Construct granular terrain
    virtual void Construct() override;

    /// Return current total number of contacts.
    virtual int GetNumContacts() const override { return m_systemGPU->GetNumContacts(); }

    virtual void CreateRigidProxy(unsigned int i) override;
    virtual void UpdateRigidProxy(unsigned int i, BodyState& rigid_state) override;
    virtual void GetForceRigidProxy(unsigned int i, TerrainForce& rigid_contact) override;

    virtual void OnOutputData(int frame) override;
    virtual void OnRender() override;

    /// Advance simulation.
    /// This function is called after a synchronization to allow the node to advance
    /// its state by the specified time step.  A node is allowed to take as many internal
    /// integration steps as required, but no inter-node communication should occur.
    virtual void OnAdvance(double step_size) override;

    /// Set composite material properties for internal granular system contacts.
    void SetMatPropertiesInternal();

    /// Set composite material properties for granular-object contacts
    /// (can be invoked only once object material was received).
    void SetMatPropertiesExternal(unsigned int i_shape);

    /// Update position of visualization shapes for granular material.
    /// Note that this requires memory transfer from GPU.
    void UpdateVisualizationParticles();
};

/// @} vehicle_cosim_chrono

}  // end namespace vehicle
}  // end namespace chrono

#endif