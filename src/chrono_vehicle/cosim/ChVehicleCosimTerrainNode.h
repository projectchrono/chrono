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
// Definition of the base class TERRAIN NODE.
//
// The global reference frame has Z up, X towards the front of the vehicle, and
// Y pointing to the left.
//
// =============================================================================

//// TODO:
////    mesh connectivity doesn't need to be communicated every time (modify Chrono?)

#ifndef CH_VEHCOSIM__TERRAINNODE_H
#define CH_VEHCOSIM__TERRAINNODE_H

#include <vector>

#include "chrono/ChConfig.h"

#include "chrono/utils/ChUtilsCreators.h"
#include "chrono/utils/ChUtilsGenerators.h"
#include "chrono/utils/ChUtilsInputOutput.h"

#include "chrono/physics/ChSystem.h"

#include "chrono_vehicle/cosim/ChVehicleCosimBaseNode.h"

namespace chrono {
namespace vehicle {

/// Base class for all terrain nodes.
class CH_VEHICLE_API ChVehicleCosimTerrainNode : public ChVehicleCosimBaseNode {
  public:
    enum class Type { RIGID, SCM, GRANULAR_OMP, GRANULAR_GPU, GRANULAR_MPI, GRANULAR_SPH };

    virtual ~ChVehicleCosimTerrainNode() {}

    Type GetType() const { return m_type; }

    /// Set container dimensions.
    void SetContainerDimensions(double length,    ///< length in direction X (default: 2)
                                double width,     ///< width in Y direction (default: 0.5)
                                double height,    ///< height in Z direction (default: 1)
                                double thickness  ///< wall thickness (default: 0.2)
    );

    /// Set properties of proxy bodies.
    /// A concrete terrain class may create spherical proxy bodies or triangle proxy bodies.
    void SetProxyProperties(double mass,    ///< mass of a proxy body (default: 1)
                            double radius,  ///< contact radius of a proxy body (default: 0.01)
                            bool fixed      ///< proxies fixed to ground? (default: false)
    );

    /// Set the material properties for terrain.
    /// The type of material must be consistent with the contact method (penalty or complementarity)
    /// specified at construction. These parameters characterize the material for the container and
    /// (if applicable) the granular material.  Tire contact material is received from the rig node.
    virtual void SetMaterialSurface(const std::shared_ptr<ChMaterialSurface>& mat) = 0;

    /// Specify whether contact coefficients are based on material properties (default: true).
    /// Note that this setting is only relevant when using the penalty method.
    virtual void UseMaterialProperties(bool flag) = 0;

    /// Set the normal contact force model (default: Hertz)
    /// Note that this setting is only relevant when using the penalty method.
    virtual void SetContactForceModel(ChSystemSMC::ContactForceModel model) = 0;

    /// Obtain settled terrain configuration.
    /// This is an optional operation that a terrain subsystem may perform before initiating
    /// communictation with the rig node.
    virtual void Settle() = 0;

    /// Initialize this node.
    /// This function allows the node to initialize itself and, optionally, perform an
    /// initial data exchange with any other node.
    virtual void Initialize() override;

    /// Synchronize this node.
    /// This function is called at every co-simulation synchronization time to
    /// allow the node to exchange information with any other node.
    virtual void Synchronize(int step_number, double time) override;

    /// Advance simulation.
    /// This function is called after a synchronization to allow the node to advance
    /// its state by the specified time step.  A node is allowed to take as many internal
    /// integration steps as required, but no inter-node communication should occur.
    virtual void Advance(double step_size) override;

    /// Output logging and debugging data.
    virtual void OutputData(int frame) override;

  protected:
    /// Triangle vertex indices.
    struct Triangle {
        int v1;
        int v2;
        int v3;
    };

    /// Mesh vertex state.
    struct VertexState {
        ChVector<> pos;
        ChVector<> vel;
    };

    /// Association between a proxy body and a mesh index.
    /// The body can be associated with either a mesh vertex or a mesh triangle.
    struct ProxyBody {
        ProxyBody(std::shared_ptr<ChBody> body, int index) : m_body(body), m_index(index) {}
        std::shared_ptr<ChBody> m_body;
        int m_index;
    };

    Type m_type;  ///< terrain type (RIGID or GRANULAR)

    ChContactMethod m_method;                               ///< contact method (SMC or NSC)
    std::shared_ptr<ChMaterialSurface> m_material_terrain;  ///< material properties for terrain bodies
    std::shared_ptr<ChMaterialSurface> m_material_tire;     ///< material properties for proxy bodies

    std::vector<ProxyBody> m_proxies;  ///< list of proxy bodies with associated mesh index
    bool m_fixed_proxies;              ///< flag indicating whether or not proxy bodies are fixed to ground
    double m_mass_p;                   ///< mass of a proxy body
    double m_radius_p;                 ///< radius for a proxy body

    double m_hdimX;   ///< container half-length (X direction)
    double m_hdimY;   ///< container half-width (Y direction)
    double m_hdimZ;   ///< container half-height (Z direction)
    double m_hthick;  ///< container wall half-thickness

    double m_init_height;  ///< initial terrain height (after optional settling)

    unsigned int m_num_vert;  ///< number of tire mesh vertices
    unsigned int m_num_tri;   ///< number of tire mesh triangles

    std::vector<VertexState> m_vertex_states;  ///< mesh vertex states
    std::vector<Triangle> m_triangles;         ///< tire mesh connectivity

    bool m_render;  ///< if true, use OpenGL rendering

    /// Construct a base class terrain node.
    ChVehicleCosimTerrainNode(Type type,               ///< terrain type (RIGID or GRANULAR)
                              ChContactMethod method,  ///< contact method (penalty or complementatiry)
                              bool render              ///< use OpenGL rendering
    );

    /// Print vertex and face connectivity data, as received from the rig node at synchronization.
    void PrintMeshUpdateData();

    // Private virtual methods

    virtual ChSystem* GetSystem() = 0;

    virtual void Construct() = 0;

    virtual void CreateProxies() = 0;
    virtual void UpdateProxies() = 0;

    virtual void ForcesProxies(std::vector<double>& vert_forces, std::vector<int>& vert_indices) = 0;

    virtual void PrintProxiesUpdateData() = 0;
    virtual void PrintProxiesContactData() = 0;

    virtual void OutputTerrainData(int frame) = 0;

    virtual void OnSynchronize(int step_number, double time) {}
    virtual void OnAdvance(double step_size) {}
};

}  // end namespace vehicle
}  // end namespace chrono

#endif
