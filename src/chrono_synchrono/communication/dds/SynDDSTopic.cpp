#include "chrono_synchrono/communication/dds/SynDDSTopic.h"

#include "chrono_synchrono/utils/SynLog.h"

#include "chrono/core/ChException.h"

#include <fastdds/dds/topic/Topic.hpp>
#include <fastdds/dds/topic/TypeSupport.hpp>
#include <fastdds/dds/domain/DomainParticipant.hpp>

using namespace eprosima::fastdds::dds;


namespace chrono {
namespace synchrono {

SynDDSTopic::SynDDSTopic(const std::string& topic_name,
                         const std::string& topic_prefix,
                         TopicDataType* data_type,
                         DomainParticipant* participant
)
    : m_topic_name(topic_name), m_dds_topic(nullptr), m_dds_type(new TypeSupport(data_type)) {
    m_topic_prefix = topic_prefix;
    m_type_name = m_dds_type->get_type_name();

    if (participant)
        Initialize(participant);
}

void SynDDSTopic::Initialize(DomainParticipant* participant) {
    if (!participant) {
        std::string str = "SynDDSTopic: Participant is NULL";
        throw ChException(str);
    }

    if (!RegisterType(participant)) {
        std::string str = "Failed to register topic " + m_topic_name;
        throw ChException(str);
    }

    m_dds_topic = participant->create_topic(GetFullTopicName(), m_type_name, TOPIC_QOS_DEFAULT);
    if (!m_dds_topic) {
        std::string str = "Failed to create topic " + m_topic_name;
        throw ChException(str);
    }
}

SynDDSTopic::~SynDDSTopic() {}

// -----------------------------------------------------------------------------------------------

std::string SynDDSTopic::GetFullTopicName() {
    return m_topic_prefix + m_topic_name;
}

void SynDDSTopic::SetPrefix(const std::string& prefix) {
    m_topic_prefix = prefix;
}

std::string SynDDSTopic::RemovePrefix(const std::string& topic, const std::string& prefix) {
    std::size_t pos = topic.find(prefix);
    if (pos == std::string::npos)
        return topic;  // prefix not found

    return topic.substr(prefix.size());
}

// -----------------------------------------------------------------------------------------------

void SynDDSTopic::DeleteDDSTopic(DomainParticipant* participant) {
    if (m_dds_topic)
        participant->delete_topic(m_dds_topic);
}

// -----------------------------------------------------------------------------------------------

bool SynDDSTopic::RegisterType(DomainParticipant* participant) {
    if (!m_dds_type)
        return false;

    return m_dds_type->register_type(participant) == ReturnCode_t::RETCODE_OK;
}

}  // namespace synchrono
}  // namespace chrono