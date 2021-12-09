#include "chrono_synchrono/communication/dds/SynDDSSubscriber.h"

#include "chrono_synchrono/communication/dds/SynDDSTopic.h"
#include "chrono_synchrono/communication/dds/SynDDSListener.h"

#include "chrono_synchrono/communication/dds/idl/SynDDSMessage.h"

#include <fastdds/dds/domain/DomainParticipant.hpp>
#include <fastdds/dds/subscriber/DataReader.hpp>
#include <fastdds/dds/subscriber/SampleInfo.hpp>
#include <fastdds/dds/subscriber/Subscriber.hpp>
#include <fastdds/dds/subscriber/qos/DataReaderQos.hpp>

#include "chrono/core/ChTypes.h"

using namespace eprosima::fastrtps;
using namespace eprosima::fastdds::dds;
using namespace eprosima::fastrtps::rtps;
using namespace eprosima::fastrtps::types;

namespace chrono {
namespace synchrono {

// -------------------------------------------------------------------------------------------------

SynDDSSubscriber::SynDDSSubscriber(Subscriber* subscriber,
                                   DataReader* reader,
                                   SynDDSDataReaderListener* listener,
                                   std::shared_ptr<SynDDSTopic> topic,
                                   std::function<void(void*)> callback,
                                   void* message,
                                   bool is_synchronous)
    : m_is_synchronous(is_synchronous),
      m_subscriber(subscriber),
      m_reader(reader),
      m_listener(listener),
      m_topic(topic),
      m_message(message),
      m_callback(callback) {}

SynDDSSubscriber::~SynDDSSubscriber() {}

void SynDDSSubscriber::DeleteDDSEntities(DomainParticipant* participant) {
    m_topic->DeleteDDSTopic(participant);

    if (m_subscriber)
        participant->delete_subscriber(m_subscriber);
}

void SynDDSSubscriber::Receive(long double wait_time) {
    if (!m_callback) {
        std::cerr << "WARNING: Subscriber callback has not been defined! Synchronous read is ignored." << std::endl;
        return;
    }

    if (!m_message) {
        std::cerr << "WARNING: Subscriber message has not been defined! Synchronous read is ignored." << std::endl;
        return;
    }

    eprosima::fastrtps::Duration_t timeout(wait_time);
    if (m_reader->wait_for_unread_message(timeout)) {
        SampleInfo info;
        if (m_reader->take_next_sample(m_message, &info) == ReturnCode_t::RETCODE_OK) {
            if (info.instance_state == ALIVE_INSTANCE_STATE) {
                m_callback(m_message);
            } else {
                std::cout << "Remote writer for topic " << m_topic->GetFullTopicName() << " is dead" << std::endl;
            }
        }
    } else {
        std::cerr << "WARNING: SynDDSSubscriber timed out while waiting for incoming message on topic "
                  << m_topic->GetFullTopicName() << std::endl;
    }
}

void SynDDSSubscriber::AsyncReceive() {
    if (!m_callback) {
        std::cerr << "WARNING: Subscriber callback has not been defined! Asynchronous read is ignored." << std::endl;
        return;
    }

    if (!m_message) {
        std::cerr << "WARNING: Subscriber message has not been defined! Asynchronous read is ignored." << std::endl;
        return;
    }

    m_reader->set_listener(m_listener, StatusMask::data_available());
}

void SynDDSSubscriber::WaitForMatches(unsigned int matches) {
    m_listener->BlockUntilMatches(matches);
}

}  // namespace synchrono
}  // namespace chrono