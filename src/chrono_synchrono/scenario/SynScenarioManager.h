#ifndef SYN_SCENARIO_MANAGER_H
#define SYN_SCENARIO_MANAGER_H

#include "chrono_synchrono/communication/mpi/SynMPIManager.h"
#include "chrono_synchrono/agent/SynAgent.h"

#include "chrono_synchrono/utils/SynUtilsJSON.h"

#include "chrono_synchrono/simulation/SynSimulationConfig.h"

using namespace chrono;

namespace chrono {
namespace synchrono {

/// Manager of one SynChrono simulation
class SYN_API SynScenarioManager {
  public:
    // Create the simulation environment from the specified simulation file
    SynScenarioManager(SynMPIManager& mpi_manager);
    SynScenarioManager(SynMPIManager& mpi_manager, ChSystem* system);

    void LoadScenario(const std::string& filename);

    void LoadRank(const rapidjson::Value& d);

  private:
    unsigned int m_rank;
    unsigned int m_num_ranks;

    SynMPIManager& m_mpi_manager;

    ChSystem* m_system;

    std::shared_ptr<SynAgent> m_agent;

    SynAgentList m_zombie_list;
};

}  // namespace synchrono
}  // namespace chrono

#endif
