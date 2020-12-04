// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2020 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Jay Taves
// =============================================================================
//
// Class that wraps and synchronizes deformable terrain between Chrono Systems
// See chrono_vehicle/terrain/SCMDeformableTerrain for the physics
//
// We have a square grid of points that get depressed as an object contacts
// them. SCMDeformableTerrain computes what their deformation should be (and
// what forces get applied) at each time step. Every time step this class saves
// the changes to each node, then at the SynChrono heartbeat sends those changes
// (which span several physics timesteps) to all other ranks.
//
// =============================================================================

#ifndef SYN_SCM_TERRAIN_H
#define SYN_SCM_TERRAIN_H

#include "chrono_vehicle/terrain/SCMDeformableTerrain.h"

#include "chrono_synchrono/flatbuffer/message/SynMessage.h"
#include "chrono_synchrono/terrain/SynTerrain.h"

namespace chrono {
namespace synchrono {

/// @addtogroup synchrono_terrain
/// @{

struct SCMParameters;

/// Class that wraps and synchronizes deformable terrain between Chrono Systems
class SYN_API SynSCMTerrain : public SynTerrain {
  public:
    /// @brief Create SynSCMTerrain by optionally adding an underlying SCMDeformableTerrain
    ///
    /// @param terrain Shared pointer to the terrain object this class will use and sync
    SynSCMTerrain(std::shared_ptr<vehicle::SCMDeformableTerrain> terrain, ChSystem* system) {
        SetTerrain(terrain);
        m_system = system;
    }

    /// @brief NOT YET SUPPORTED - Construct deformable terrain on a particular system from a json file
    SynSCMTerrain(ChSystem* system, const std::string& filename);

    /// @brief Empty destructor
    ~SynSCMTerrain() {}

    /// @brief Set terrain's soil parameters from the passed struct
    ///
    /// @param params Physical parameters for terrain, see SCMParameters struct
    void SetSoilParametersFromStruct(SCMParameters* params);

    /// @brief Call terrain.Advance(step) and save any new nodes that were changed into our m_scm_terrain record of them
    ///
    /// @param step duration to advance the state of the terrain
    void Advance(double step) override;

    /// @brief Apply a list of received node changes to our underlying terrain object. This happens via
    /// SCMDeformableTerrain::SetModifiedNodes(changed_nodes)
    ///
    /// @param message SynMessage* with terrain data on nodes changed by a different rank
    virtual void ProcessMessage(SynMessage* message) override;

    /// @brief Pack all changes we have made to our SCMDeformableTerrain in the preceding heartbeat into a message.
    /// Clear out our record of changes to start recording fresh for the next heartbeat
    ///
    /// @param messages Where to add this data to
    /// @param rank Which rank we are attached to (only our agent knows)
    virtual void GenerateMessagesToSend(std::vector<SynMessage*>& messages, int rank) override;

    void SetTerrain(std::shared_ptr<vehicle::SCMDeformableTerrain> terrain) { m_scm_terrain = terrain; }

    virtual std::shared_ptr<vehicle::ChTerrain> GetTerrain() override { return m_scm_terrain; }
    std::shared_ptr<vehicle::SCMDeformableTerrain> GetSCMTerrain() { return m_scm_terrain; }

  private:
    /// There is no STL default for hashing a pair of ints, but the SCM grid is indexed with integers, so we store diffs
    /// using a map of that format.
    /// This hash function has in no way been tested for performance (it should not matter), 31 is just a decently-sized
    /// prime number to reduce bucket collisions
    struct CoordHash {
      public:
        std::size_t operator()(const ChVector2<int>& p) const { return p.x() * 31 + p.y(); }
    };

    std::shared_ptr<vehicle::SCMDeformableTerrain> m_scm_terrain;  ///< Underlying terrain we manage
    std::unordered_map<ChVector2<int>, double, CoordHash>
        m_modified_nodes;  ///< Where we store changes to our terrain between synchronization heartbeats
    ChSystem* m_system;    ///< Cache the ChSystem we're attached to so that we can know what time it is
};

/// Groups SCM parameters into a struct, defines some useful defaults
/// See SCMDeformableTerrain::SetSoilParameters and SoilParametersCallback for more details on these
struct SCMParameters {
    double m_Bekker_Kphi;    ///< Kphi, frictional modulus in Bekker model
    double m_Bekker_Kc;      ///< Kc, cohesive modulus in Bekker model
    double m_Bekker_n;       ///< n, exponent of sinkage in Bekker model (usually 0.6...1.8)
    double m_Mohr_cohesion;  ///< Cohesion for shear failure [Pa]
    double m_Mohr_friction;  ///< Friction angle for shear failure [degree]
    double m_Janosi_shear;   ///< Shear parameter in Janosi-Hanamoto formula [m]
    double m_elastic_K;      ///< elastic stiffness K per unit area, [Pa/m] (must be larger than Kphi)
    double m_damping_R;      ///< vertical damping R per unit area [Pa.s/m] (proportional to vertical speed)

    void SetParameters(double Bekker_Kphi,
                       double Bekker_Kc,
                       double Bekker_n,
                       double Mohr_cohesion,
                       double Mohr_friction,
                       double Janosi_shear,
                       double elastic_K,
                       double damping_R) {
        m_Bekker_Kphi = Bekker_Kphi;
        m_Bekker_Kc = Bekker_Kc;
        m_Bekker_n = Bekker_n;
        m_Mohr_cohesion = Mohr_cohesion;
        m_Mohr_friction = Mohr_friction;
        m_Janosi_shear = Janosi_shear;
        m_elastic_K = ChMax(elastic_K, Bekker_Kphi);
        m_damping_R = damping_R;
    }

    void InitializeParametersAsSoft() {
        m_Bekker_Kphi = 0.2e6;
        m_Bekker_Kc = 0;
        m_Bekker_n = 1.1;
        m_Mohr_cohesion = 0;
        m_Mohr_friction = 30;
        m_Janosi_shear = 0.01;
        m_elastic_K = 4e7;
        m_damping_R = 3e4;
    }

    void InitializeParametersAsMid() {
        m_Bekker_Kphi = 2e6;
        m_Bekker_Kc = 0;
        m_Bekker_n = 1.1;
        m_Mohr_cohesion = 0;
        m_Mohr_friction = 30;
        m_Janosi_shear = 0.01;
        m_elastic_K = 2e8;
        m_damping_R = 3e4;
    }

    void InitializeParametersAsHard() {
        m_Bekker_Kphi = 5301e3;
        m_Bekker_Kc = 102e3;
        m_Bekker_n = 0.793;
        m_Mohr_cohesion = 1.3e3;
        m_Mohr_friction = 31.1;
        m_Janosi_shear = 1.2e-2;
        m_elastic_K = 4e8;
        m_damping_R = 3e4;
    }
};

/// @} synchrono_terrain

}  // namespace synchrono
}  // namespace chrono

#endif
