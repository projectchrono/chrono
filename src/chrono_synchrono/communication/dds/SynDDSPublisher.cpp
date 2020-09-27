#include "chrono_synchrono/communication/dds/SynDDSPublisher.h"

#include <fastdds/dds/publisher/qos/DataWriterQos.hpp>

namespace chrono {
namespace synchrono {

SynDDSPublisher::SynDDSPublisher(DomainParticipant* participant, std::string topic_name, std::string topic_type)
    : m_participant(participant),
      m_topic_name(topic_name),
      m_topic_type(topic_type),
      m_publisher(nullptr),
      m_writer(nullptr),
      m_topic(nullptr),
      m_listener(this) {}

SynDDSPublisher::~SynDDSPublisher() {
    // Clean the Publisher
    if (m_writer && m_publisher)
        m_publisher->delete_datawriter(m_writer);

    if (m_topic)
        m_participant->delete_topic(m_topic);

    if (m_publisher)
        m_participant->delete_publisher(m_publisher);
}

bool SynDDSPublisher::Initialize() {
    // Create the publication topic
    std::cout << "Publishing to " << m_topic_name << std::endl;
    m_topic = m_participant->create_topic(m_topic_name, m_topic_type, TOPIC_QOS_DEFAULT);
    if (!m_topic)
        return false;

    // Create the Publisher
    m_publisher = m_participant->create_publisher(PUBLISHER_QOS_DEFAULT, &m_listener);
    if (!m_publisher)
        return false;

    // Create the DataWriterQos, of which allows for reallocation on data sending so that an unsized sequence could be
    // used in the idl file
    DataWriterQos qos;
    qos.endpoint().history_memory_policy = PREALLOCATED_WITH_REALLOC_MEMORY_MODE;

    // Create the DataWriter
    m_writer = m_publisher->create_datawriter(m_topic, qos, nullptr);
    if (!m_writer)
        return false;

    return true;
}

bool SynDDSPublisher::Publish(SynDDSMessage& msg) {
    if (m_listener.IsMatched()) {
        m_writer->write(&msg);

        if (m_writer->wait_for_acknowledgments({5, 0}) != ReturnCode_t::RETCODE_OK) {
            std::cout << "SynDDSPublisher::Publisher: Writer failed to receive acknowledgments." << std::endl;
            return false;
        }

        return true;
    }
    std::cout << "Failed to publish." << std::endl;
    return false;
}

SynDDSPublisher::SynDDSPublisherListener::SynDDSPublisherListener(SynDDSPublisher* publisher)
    : m_publisher(publisher) {}

SynDDSPublisher::SynDDSPublisherListener::~SynDDSPublisherListener() {}

void SynDDSPublisher::SynDDSPublisherListener::on_publication_matched(DataWriter* writer,
                                                                      const PublicationMatchedStatus& info) {
    if (info.current_count_change == 1) {
        m_matched = info.total_count;
        std::cout << "Publisher matched." << std::endl;
    } else if (info.current_count_change == -1) {
        m_matched = info.total_count;
        m_publisher->m_ok = false;
        std::cout << "Publisher unmatched." << std::endl;
    } else {
        m_publisher->m_ok = false;
        std::cout << info.current_count_change
                  << " is not a valid value for PublicationMatchedStatus "
                     "current count change."
                  << std::endl;
    }
}

}  // namespace synchrono
}  // namespace chrono