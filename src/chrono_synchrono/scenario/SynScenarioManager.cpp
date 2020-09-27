#include "chrono_synchrono/scenario/SynScenarioManager.h"

#include "chrono_synchrono/agent/SynWheeledVehicleAgent.h"
#include "chrono_synchrono/agent/SynTrackedVehicleAgent.h"
#include "chrono_synchrono/utils/SynDataLoader.h"

#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChSystemSMC.h"

using namespace chrono;

using namespace rapidjson;

namespace chrono {
namespace synchrono {

SynScenarioManager::SynScenarioManager(SynMPIManager& mpi_manager, ChSystem* system)
    : m_mpi_manager(mpi_manager),
      m_rank(mpi_manager.GetRank()),
      m_num_ranks(mpi_manager.GetNumRanks()),
      m_system(system) {}

SynScenarioManager::SynScenarioManager(SynMPIManager& mpi_manager)
    : m_mpi_manager(mpi_manager), m_rank(mpi_manager.GetRank()), m_num_ranks(mpi_manager.GetNumRanks()) {
    m_system = (CONTACT_METHOD == ChContactMethod::NSC) ? static_cast<ChSystem*>(new ChSystemNSC)
                                                        : static_cast<ChSystem*>(new ChSystemSMC);

    m_system->Set_G_acc(-9.81 * VECT_Z);

    // Integration and Solver settings
    switch (CONTACT_METHOD) {
        case ChContactMethod::NSC:
            m_system->SetSolverType(ChSolver::Type::BARZILAIBORWEIN);
            break;
        default:
            break;
    }

    m_system->SetSolverMaxIterations(150);
    m_system->SetMaxPenetrationRecoverySpeed(4.0);
}

void SynScenarioManager::LoadScenario(const std::string& filename) {
    // Open and parse the input file
    Document d = ReadFileJSON(filename);
    if (d.IsNull())
        throw ChException("Scenario file not read properly in ParseScenarioFileJSON");

    // Read top-level data
    assert(d.HasMember("Name"));
    assert(d.HasMember("Type"));

    std::string name = d["Name"].GetString();
    std::string type = d["Type"].GetString();
    assert(type.compare("Scenario") == 0);

    assert(d.HasMember("Ranks"));
    assert(d["Ranks"].IsArray());

    if (d["Ranks"].Size() > m_num_ranks) {
        std::string out = "Number of ranks specified in the JSON file (" + std::to_string(d["Ranks"].Size()) +
                          ") is more than SynChrono was told to launch at runtime (" + std::to_string(m_num_ranks) +
                          ").";
        std::cout << out << std::endl;
        m_mpi_manager.Exit();
    }

    // Create ranks
    for (int i = 0; i < m_num_ranks; i++) {
        LoadRank(d["Ranks"][i]);
    }
}

void SynScenarioManager::LoadRank(const rapidjson::Value& d) {
    assert(d.IsObject());
    assert(d.HasMember("Rank"));

    int rank = d["Rank"].GetInt();

    if (d.HasMember("Agent")) {
        assert(d["Agent"].HasMember("Type"));
        assert(d["Agent"].HasMember("Input File"));

        std::string type = d["Agent"]["Type"].GetString();
        std::string filename = GetSynDataFile(d["Agent"]["Input File"].GetString());

        std::shared_ptr<SynAgent> agent;

        if (type.compare("WheeledVehicleAgent") == 0) {
            assert(d["Agent"].HasMember("Location"));
            assert(d["Agent"].HasMember("Orientation"));

            auto vehicle_agent = chrono_types::make_shared<SynWheeledVehicleAgent>(rank, filename, m_system);
            vehicle_agent->Initialize(ReadCoordsysJSON(d["Agent"]["Location"], d["Agent"]["Orientation"]));

            agent = vehicle_agent;
        } else if (type.compare("TrackedVehicleAgent") == 0) {
            assert(d["Agent"].HasMember("Location"));
            assert(d["Agent"].HasMember("Orientation"));

            auto vehicle_agent = chrono_types::make_shared<SynTrackedVehicleAgent>(rank, filename, m_system);
            vehicle_agent->Initialize(ReadCoordsysJSON(d["Agent"]["Location"], d["Agent"]["Orientation"]));

            agent = vehicle_agent;
        }

        m_mpi_manager.AddAgent(agent, rank);
    }
}

}  // namespace synchrono
}  // namespace chrono
