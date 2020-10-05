#ifndef SYN_AGENT_FACTORY_H
#define SYN_AGENT_FACTORY_H

#include "chrono_synchrono/SynApi.h"

#include "chrono_synchrono/agent/SynAgent.h"

namespace chrono {
namespace synchrono {

/// Generates SynTerrain's from JSON files
/// Used to improve generality in Agent classes
class SYN_API SynAgentFactory {
  public:
    /// Generate the corresponding SynAgent from a description message
    static std::shared_ptr<SynAgent> CreateAgent(SynAgentMessage* message);

    /// Generate the corresponding SynAgent from a JSON specification file
    static std::shared_ptr<SynAgent> CreateAgent(unsigned int rank,
                                                 ChCoordsys<> coord_sys,
                                                 const std::string& filename,
                                                 ChSystem* system);

    /// Generate the corresponding SynAgent from a JSON specification file
    static std::shared_ptr<SynAgent> CreateAgent(unsigned int rank,
                                                 ChCoordsys<> coord_sys,
                                                 const std::string& filename,
                                                 ChContactMethod contact_method = ChContactMethod::NSC);
};

}  // namespace synchrono
}  // namespace chrono

#endif
