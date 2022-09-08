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
// Definition of the OpenMP granular TERRAIN NODE (using Chrono::Multicore).
//
// The global reference frame has Z up, X towards the front of the vehicle, and
// Y pointing to the left.
//
// =============================================================================

#ifndef TESTRIG_TERRAIN_NODE_GRANULAR_OMP_H
#define TESTRIG_TERRAIN_NODE_GRANULAR_OMP_H

#include "chrono/ChConfig.h"
#include "chrono/utils/ChUtilsSamplers.h"
#include "chrono_multicore/physics/ChSystemMulticore.h"

#include "chrono_vehicle/cosim/terrain/ChVehicleCosimTerrainNodeChrono.h"

#ifdef CHRONO_OPENGL
    #include "chrono_opengl/ChVisualSystemOpenGL.h"
#endif

#include "chrono_thirdparty/rapidjson/document.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle_cosim_chrono
/// @{

/// Definition of the OpenMP granular terrain node (using Chrono::Multicore).
class CH_VEHICLE_API ChVehicleCosimTerrainNodeGranularOMP : public ChVehicleCosimTerrainNodeChrono {
  public:
    /// Create a Chrono::Multicore granular terrain node using the specified contact method (SMC or NSC).
    ChVehicleCosimTerrainNodeGranularOMP(double length, double width, ChContactMethod method);

    /// Create a Chrono::Multicore granular terrain node using the specified contact method (SMC or NSC) and set
    /// parameters from the provided JSON specfile.
    ChVehicleCosimTerrainNodeGranularOMP(ChContactMethod method, const std::string& specfile);

    ~ChVehicleCosimTerrainNodeGranularOMP();

    virtual ChSystem* GetSystem() override { return m_system; }

    /// Set the number of OpenMP threads Chrono::multicore terrain simulation.
    void SetNumThreads(int num_threads);

    /// Set full terrain specification from JSON specfile.
    void SetFromSpecfile(const std::string& specfile);

    /// Set container wall thickness (default: 0.2)
    void SetWallThickness(double thickness);

    /// Set properties of granular material.
    void SetGranularMaterial(double radius,  ///< particle radius (default: 0.01)
                             double density  ///< particle material density (default: 2000)
    );

    /// Set the material properties for terrain.
    /// The type of material must be consistent with the contact method (SMC or NSC)
    /// specified at construction. These parameters characterize the material for the container and
    /// (if applicable) the granular material.  Object contact material is received from the rig node.
    void SetMaterialSurface(const std::shared_ptr<ChMaterialSurface>& mat);

    /// Specify whether contact coefficients are based on material properties (default: true).
    /// Note that this setting is only relevant when using the SMC method.
    void UseMaterialProperties(bool flag);

    /// Set the normal contact force model (default: Hertz).
    /// Note that this setting is only relevant when using the SMC method.
    void SetContactForceModel(ChSystemSMC::ContactForceModel model);

    /// Set the tangential contact displacement model (default: OneStep).
    /// Note that this setting is only relevant when using the SMC method.
    void SetTangentialDisplacementModel(ChSystemSMC::TangentialDisplacementModel model);

    /// Set sampling method for generation of granular material.
    /// The granular material is created in the volume defined by the x-y dimensions of the terrain patch and the
    /// specified initial height, using the specified sampling type, layer by layer or all at once.
    /// Note: for correct HCP, do not initialize in layers!
    void SetSamplingMethod(utils::SamplingType type,   ///< volume sampling type (default POISSON_DISK)
                           double init_height,         ///< height of granular material at initialization (default 0.2)
                           double sep_factor = 1.001,  ///< radius inflation factor for initial separation
                           bool in_layers = false      ///< initialize material in layers
    );

    /// Set sweeping sphere radius for proxy bodies (default 5e-3).
    /// This value is used as a "thickness" for collision meshes (a non-zero value can improve robustness of the
    /// collision detection algorithm).
    void SetProxyContactRadius(double radius) { m_radius_p = radius; }

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

    /// Write checkpoint to the specified file (which will be created in the output directory).
    virtual void WriteCheckpoint(const std::string& filename) const override;

    /// Estimate packing density (eta) of granular material in current configuration.
    /// Note that porosity is phi=1-eta and void ratio is e=(1-eta)/eta=phi/(1-phi).
    /// The function also returns the current depth of granular material.
    double CalculatePackingDensity(double& depth);

    /// Output post-processing visualization data.
    virtual void OutputVisualizationData(int frame) override final;

  private:
    ChSystemMulticore* m_system;  ///< containing system
    bool m_constructed;           ///< system construction completed?

#ifdef CHRONO_OPENGL
    opengl::ChVisualSystemOpenGL* m_vsys;  ///< OpenGL visualization system
#endif

    double m_hthick;  ///< container wall half-thickness

    double m_radius_p;  ///< radius for a proxy body

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

    virtual bool SupportsMeshInterface() const override { return true; }

    virtual void Construct() override;

    /// Return current total number of contacts.
    virtual int GetNumContacts() const override { return m_system->GetNcontacts(); }

    virtual void CreateMeshProxy(unsigned int i) override;
    virtual void UpdateMeshProxy(unsigned int i, MeshState& mesh_state) override;
    virtual void GetForceMeshProxy(unsigned int i, MeshContact& mesh_contact) override;
    void PrintMeshProxiesUpdateData(unsigned int i, const MeshState& mesh_state);

    virtual void CreateRigidProxy(unsigned int i) override;
    virtual void UpdateRigidProxy(unsigned int i, BodyState& rigid_state) override;
    virtual void GetForceRigidProxy(unsigned int i, TerrainForce& rigid_contact) override;

    virtual void OnAdvance(double step_size) override;
    virtual void OnOutputData(int frame) override;
    virtual void Render(double time) override;

    /// Calculate current height of granular terrain.
    double CalcCurrentHeight();

    /// Calculate total kinetic energy of granular material.
    double CalcTotalKineticEnergy();

    void WriteParticleInformation(utils::CSV_writer& csv);

    static ChVector<> CalcBarycentricCoords(const ChVector<>& v1,
                                            const ChVector<>& v2,
                                            const ChVector<>& v3,
                                            const ChVector<>& vP);
};

/// @} vehicle_cosim_chrono

}  // end namespace vehicle
}  // end namespace chrono

#endif
