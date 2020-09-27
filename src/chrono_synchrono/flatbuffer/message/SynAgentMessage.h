#ifndef SYN_AGENT_MESSAGE_H
#define SYN_AGENT_MESSAGE_H

#include "chrono_synchrono/flatbuffer/message/SynMessage.h"

namespace chrono {
namespace synchrono {

///@brief The agent description struct
/// Should be inherited and stored with additional description information relavent to the new agent types
/// Should hold infrequently passed data
/// For messages or data that needs to be passed frequently, see SynMessageState
struct SynAgentDescription {
    std::string json = "";  ///< the json specification file that is used to create an agent

    ///@brief Construct a new SynAgentDescription object
    ///
    ///@param json the json specification file used to create an agent
    SynAgentDescription(std::string json = "") : json(json) {}
};

class SYN_API SynAgentMessage : public SynMessage {
  public:
    ///@brief Construct a new SynAgentMessage object
    ///
    ///@param rank the rank of this message
    SynAgentMessage(int rank, SynMessageType type, std::string json = "");

    // ---------------------------------------------------------------------------------------------------------------

    ///@brief Generates and sets the state of this message from flatbuffer message
    ///
    ///@param message the flatbuffer message to convert to a MessageState object
    virtual void StateFromMessage(const SynFlatBuffers::Message* message) override;

    // ---------------------------------------------------------------------------------------------------------------

    ///@brief Generates and sets the description of the agent from flatbuffer message
    ///
    ///@param message the flatbuffer message to convert to a SynAgentDescription object
    virtual void DescriptionFromMessage(const SynFlatBuffers::Message* message);

    ///@brief Generates a SynFlatBuffers::Message from the agent description
    ///
    ///@param builder the flatbuffer builder used to construct messages
    ///@return flatbuffers::Offset<SynFlatBuffers::Message> the generated message
    virtual FlatBufferMessage MessageFromDescription(flatbuffers::FlatBufferBuilder& builder) = 0;

    ///@brief Get the SynAgentDescription object
    ///
    ///@return std::shared_ptr<SynAgentDescription> the description associated with this message
    std::shared_ptr<SynAgentDescription> GetDescription() { return m_description; }

    // Set the json file that was used to generate the agent
    void SetJSON(const std::string& json) { m_description->json = json; }

  protected:
    std::shared_ptr<SynAgentDescription> m_description;  ///< handle to the agent description
};

}  // namespace synchrono
}  // namespace chrono

#endif
