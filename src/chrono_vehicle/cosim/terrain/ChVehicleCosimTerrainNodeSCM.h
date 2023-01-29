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
// Definition of the SCM deformable TERRAIN NODE.
//
// The global reference frame has Z up, X towards the front of the vehicle, and
// Y pointing to the left.
//
// =============================================================================

#ifndef CH_VEHCOSIM_TERRAIN_NODE_SCM_H
#define CH_VEHCOSIM_TERRAIN_NODE_SCM_H

#include "chrono/physics/ChSystem.h"
#include "chrono_vehicle/terrain/SCMTerrain.h"

#include "chrono_vehicle/cosim/terrain/ChVehicleCosimTerrainNodeChrono.h"

namespace chrono {

#ifdef CHRONO_IRRLICHT
namespace irrlicht {
class ChVisualSystemIrrlicht;
}
#endif

namespace vehicle {

/// @addtogroup vehicle_cosim_chrono
/// @{

/// Definition of the SCM deformable terrain node.
class CH_VEHICLE_API ChVehicleCosimTerrainNodeSCM : public ChVehicleCosimTerrainNodeChrono {
  public:
    /// Create an SCM terrain node. Note that no SCM parameters are set.
    ChVehicleCosimTerrainNodeSCM(double length, double width);

    /// Create an SCM terrain node using parameters from the provided JSON specfile.
    ChVehicleCosimTerrainNodeSCM(const std::string& specfile);

    ~ChVehicleCosimTerrainNodeSCM();

    virtual ChSystem* GetSystem() override { return m_system; }

    /// Set full terrain specification from JSON specfile.
    void SetFromSpecfile(const std::string& specfile);

    /// Set the SCM material properties for terrain.
    void SetPropertiesSCM(
        double spacing,        ///< SCM grid spacing
        double Bekker_Kphi,    ///< Kphi, frictional modulus in Bekker model
        double Bekker_Kc,      ///< Kc, cohesive modulus in Bekker model
        double Bekker_n,       ///< n, exponent of sinkage in Bekker model (usually 0.6...1.8)
        double Mohr_cohesion,  ///< cohesion [Pa], for shear failure
        double Mohr_friction,  ///< Friction angle [degrees], for shear failure
        double Janosi_shear,   ///< shear parameter J [m], (usually a few mm or cm)
        double elastic_K,      ///< elastic stiffness K per unit area, [Pa/m] (must be larger than Kphi)
        double damping_R       ///< vertical damping R per unit area [Pa.s/m] (proportional to vertical speed)
    );

    /// Set the number of OpenMP threads for SCM ray-casting (default: 1).
    void SetNumThreads(int num_threads);

    /// Set sweeping sphere radius for proxy bodies (default 5e-3).
    /// This value is used as a "thickness" for collision meshes (a non-zero value can improve robustness of the
    /// collision detection algorithm).
    void SetProxyContactRadius(double radius) { m_radius_p = radius; }

    /// Initialize SCM terrain from the specified checkpoint file (which must exist in the output directory).
    /// By default, a flat rectangular SCM terrain patch is used.
    void SetInputFromCheckpoint(const std::string& filename);

    /// Write checkpoint to the specified file (which will be created in the output directory).
    virtual void WriteCheckpoint(const std::string& filename) const override;

  private:
    ChSystem* m_system;               ///< containing system
    SCMTerrain* m_terrain;  ///< SCM terrain
#ifdef CHRONO_IRRLICHT
    std::shared_ptr<irrlicht::ChVisualSystemIrrlicht> m_vsys;  ///< Irrlicht run-time visualization
#endif

    double m_spacing;        ///< SCM grid spacing
    double m_Bekker_Kphi;    ///< Kphi, frictional modulus in Bekker model
    double m_Bekker_Kc;      ///< Kc, cohesive modulus in Bekker model
    double m_Bekker_n;       ///< n, exponent of sinkage in Bekker model (usually 0.6...1.8)
    double m_Mohr_cohesion;  ///< Cohesion in, Pa, for shear failure
    double m_Mohr_friction;  ///< Friction angle (in degrees!), for shear failure
    double m_Janosi_shear;   ///< J , shear parameter, in meters, in Janosi-Hanamoto formula (usually few mm or cm)
    double m_elastic_K;      ///< elastic stiffness K per unit area [Pa/m]
    double m_damping_R;      ///< vetical damping R per unit area [Pa.s/m]

    double m_radius_p;  ///< radius for a proxy body

    bool m_use_checkpoint;              ///< if true, initialize height from checkpoint file
    std::string m_checkpoint_filename;  ///< name of input checkpoint file

    virtual bool SupportsMeshInterface() const override { return false; }  //// TODO

    virtual void Construct() override;

    virtual void CreateMeshProxy(unsigned int i) override;
    virtual void UpdateMeshProxy(unsigned int i, MeshState& mesh_state) override;
    virtual void GetForceMeshProxy(unsigned int i, MeshContact& mesh_contact) override;

    virtual void CreateRigidProxy(unsigned int i) override;
    virtual void UpdateRigidProxy(unsigned int i, BodyState& rigid_state) override;
    virtual void GetForceRigidProxy(unsigned int i, TerrainForce& rigid_contact) override;

    virtual void OnOutputData(int frame) override;
    virtual void Render(double time) override;
};

/// @} vehicle_cosim_chrono

}  // end namespace vehicle
}  // end namespace chrono

#endif
