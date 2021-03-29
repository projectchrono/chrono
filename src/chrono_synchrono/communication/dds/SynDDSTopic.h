#ifndef SYN_DDS_TOPIC_H
#define SYN_DDS_TOPIC_H

#include <string>
#include <map>
#include <vector>
#include <memory>

namespace eprosima {
namespace fastdds {
namespace dds {

class Topic;

class TypeSupport;
class TopicDataType;

class DomainParticipant;

}  // namespace dds
}  // namespace fastdds
}  // namespace eprosima

namespace chrono {
namespace synchrono {

class SynDDSCommunicator;

/// @addtogroup synchrono_communication_dds
/// @{

/// Describes information that's being distributed over an abstract "wire"
/// Includes data type and meta information (i.e. topic name)
class SynDDSTopic {
  public:
    ///@brief Constructor
    /// Will call Initialize
    ///
    ///@param topic_name The topic name
    ///@param data_type The topic data type
    ///@param participant the participant used to create the topic and register the type, if necessary
    ///@param topic_prefix The prefix used.
    SynDDSTopic(const std::string& topic_name,
				const std::string& topic_prefix,
                eprosima::fastdds::dds::TopicDataType* data_type,
                eprosima::fastdds::dds::DomainParticipant* participant = nullptr);

    ///@brief Destructor
    ///
    ~SynDDSTopic();

    ///@brief Creates the TypeSupport and the DDS topic. The type support will be registered.
    ///
    ///@param participant the participant used to create the topic and register the type, if necessary
    void Initialize(eprosima::fastdds::dds::DomainParticipant* participant);

    // -----------------------------------------------------------------------------------------------

    ///@brief Get the topic name
    ///
    const std::string& GetTopicName() const { return m_topic_name; }

    ///@brief Get the full topic name with the prefix attached
    ///
    std::string GetFullTopicName();

    ///@brief Get the DDS topic object
    ///
    eprosima::fastdds::dds::Topic* GetDDSTopic() { return m_dds_topic; }

    ///@brief Set the static prefix for this SynDDSTopic in particular
    ///
    void SetPrefix(const std::string& prefix);

    ///@brief Remove the prefix on a topic
    /// If prefix not found, the original string is returned
    ///
    ///@param topic string representation of the topic with a prefix
    ///@param prefix the used prefix
    ///@return std::string the topic with the prefix removed
    static std::string RemovePrefix(const std::string& topic, const std::string& prefix);

    // -----------------------------------------------------------------------------------------------

    ///@brief Delete the DDS topic created
    ///
    ///@param participant The participant used to delete the topic
    void DeleteDDSTopic(eprosima::fastdds::dds::DomainParticipant* participant);

    // -----------------------------------------------------------------------------------------------

  private:
    ///@brief Register the DDS topic type
    ///
    ///@param participant the dds participant used to register the type
    bool RegisterType(eprosima::fastdds::dds::DomainParticipant* participant);

    // -----------------------------------------------------------------------------------------------

    std::string m_topic_name;                    ///< The topic name
    std::string m_topic_prefix;                  ///< The topic prefix
    eprosima::fastdds::dds::Topic* m_dds_topic;  ///< ptr to the DDS topic object

    std::string m_type_name;                          ///< The type support name
    eprosima::fastdds::dds::TypeSupport* m_dds_type;  ///< ptr to the DDS type support object
};

typedef std::map<std::string, std::shared_ptr<SynDDSTopic>> TopicMap;
typedef std::vector<std::shared_ptr<SynDDSTopic>> TopicList;

/// @} synchrono_communication

}  // namespace synchrono
}  // namespace chrono

#endif