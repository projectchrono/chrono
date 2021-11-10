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
// =============================================================================
//
// Class that handles communication across ranks or external entities. A
// communicator is something that passes messages over some protocol and
// interfaecs either a rank with another rank, a rank with an external process
// or really anything that relies on communication over some network interface.
//
// This class is implemented as a very generic abstract handler that holds and
// defines common functions and variables used by all communicators.
//
// =============================================================================

#include "chrono_synchrono/communication/dds/SynDDSCommunicator.h"

#include "chrono_synchrono/utils/SynLog.h"
#include "chrono_synchrono/communication/dds/SynDDSTopic.h"
#include "chrono_synchrono/communication/dds/SynDDSListener.h"

#undef ALIVE

#include "chrono_synchrono/communication/dds/idl/SynDDSMessage.h"
#include "chrono_synchrono/communication/dds/idl/SynDDSMessagePubSubTypes.h"

#include <fastdds/dds/domain/DomainParticipant.hpp>
#include <fastdds/dds/domain/DomainParticipantFactory.hpp>
#include <fastdds/dds/domain/qos/DomainParticipantQos.hpp>

#include <fastdds/dds/publisher/Publisher.hpp>
#include <fastdds/dds/publisher/DataWriter.hpp>
#include <fastdds/dds/publisher/qos/DataWriterQos.hpp>

#include <fastdds/dds/subscriber/Subscriber.hpp>
#include <fastdds/dds/subscriber/DataReader.hpp>
#include <fastdds/dds/subscriber/qos/DataReaderQos.hpp>

#include <fastdds/rtps/transport/UDPv4TransportDescriptor.h>

using namespace eprosima::fastdds::dds;
using namespace eprosima::fastdds::rtps;
using namespace eprosima::fastrtps::rtps;

namespace chrono {
namespace synchrono {

SynDDSCommunicator::SynDDSCommunicator(int node_id, const std::string& prefix) : m_prefix(prefix) {
    AgentKey agent_key = AgentKey(node_id, 0);
    InitQoS(m_prefix + agent_key.GetKeyString());
}

SynDDSCommunicator::SynDDSCommunicator(const std::string& name, const std::string& prefix) : m_prefix(prefix) {
    InitQoS(name);
}

SynDDSCommunicator::SynDDSCommunicator(eprosima::fastdds::dds::DomainParticipantQos& qos, const std::string& prefix)
    : m_prefix(prefix) {
    CreateParticipant(qos);
}

void SynDDSCommunicator::InitQoS(const std::string& name) {
    // Create domain participant qos and set its name
    DomainParticipantQos qos;
    qos.name(name);

    // Use UDP by default
    qos.transport().user_transports.push_back(chrono_types::make_shared<UDPv4TransportDescriptor>());
    qos.transport().use_builtin_transports = false;

    CreateParticipant(qos);
}

void SynDDSCommunicator::CreateParticipant(DomainParticipantQos& qos) {
    // Create the listener and mask
    m_listener = new SynDDSParticipantListener(m_prefix);
    auto mask = StatusMask::all();
    mask.set(9, false);

    // Create the domain participant
    m_participant = DomainParticipantFactory::get_instance()->create_participant(0, qos, m_listener, mask);
    if (!m_participant)
        SynLog() << "ERROR: Failed to create participant\n";
}

SynDDSCommunicator::~SynDDSCommunicator() {
    for (auto& publisher : m_publishers) {
        publisher->GetPublisher()->delete_datawriter(publisher->GetDataWriter());
    }

    for (auto& subscriber : m_subscribers) {
        subscriber->GetSubscriber()->delete_datareader(subscriber->GetDataReader());
    }

    for (auto& pair : m_topics) {
        auto topic = pair.second;
        m_participant->delete_topic(topic->GetDDSTopic());
    }

    for (auto& publisher : m_publishers) {
        m_participant->delete_publisher(publisher->GetPublisher());
    }

    for (auto& subscriber : m_subscribers) {
        m_participant->delete_subscriber(subscriber->GetSubscriber());
    }

    DomainParticipantFactory::get_instance()->delete_participant(m_participant);
}

void SynDDSCommunicator::Initialize() {
    // Wait for all participants to be available
    Barrier();
    SynCommunicator::Initialize();
}

void SynDDSCommunicator::Synchronize() {
    // Complete the buffer
    m_flatbuffers_manager.Finish();

    // Publish data
    Publish();

    // Blocking wait for a message to be received
    Listen();
}

void SynDDSCommunicator::Barrier() {
    for (auto subscriber : m_subscribers)
        subscriber->WaitForMatches(1);
}
// -----------------------------------------------------------------------------------------------

void SynDDSCommunicator::Barrier(unsigned int num_participants) {
    m_listener->BlockUntilMatches(num_participants);
}

std::vector<std::string> SynDDSCommunicator::GetMatchedParticipantNames() {
    return m_listener->GetParticipantNames();
}

// -----------------------------------------------------------------------------------------------

std::shared_ptr<SynDDSTopic> SynDDSCommunicator::CreateTopic(const std::string& topic_name,
                                                             TopicDataType* data_type,
                                                             const std::string& topic_prefix) {
    if (m_topics.find(topic_name) != m_topics.end()) {
        SynLog() << "Topic with name " << topic_name << "is already registered.\n";
        return m_topics[topic_name];
    }

    auto topic = chrono_types::make_shared<SynDDSTopic>(topic_name, topic_prefix, data_type, m_participant);
    m_topics[topic_name] = topic;

    return topic;
}

std::shared_ptr<SynDDSTopic> SynDDSCommunicator::CreateTopic(const std::string& topic_name,
                                                             TopicDataType* data_type) {
    auto topic = this->CreateTopic(topic_name, data_type, m_prefix);
    return topic;
}

std::shared_ptr<SynDDSSubscriber> SynDDSCommunicator::CreateSubscriber(std::shared_ptr<SynDDSTopic> topic,
                                                                       std::function<void(void*)> callback,
                                                                       void* message,
                                                                       bool is_synchronous,
                                                                       bool is_managed,
                                                                       DataReaderQos* read_qos) {
    SynLog() << "Creating Subscriber " << topic->GetFullTopicName() << "\n";

    if (!m_participant) {
        SynLog() << "CreateSubscriber: Participant is NULL\n";
        return nullptr;
    }

    // Create the subscriber with no listener and a default qos
    auto subscriber = m_participant->create_subscriber(SUBSCRIBER_QOS_DEFAULT);
    if (!subscriber) {
        SynLog() << "CreateSubscriber: Subscriber instantiation FAILED\n";
        return nullptr;
    }
    std::shared_ptr<DataReaderQos> qos = nullptr;
    if (read_qos != nullptr) {
        qos = std::make_shared<DataReaderQos>(*read_qos);
    }
    // Create data reader qos and allow for automatic reallocation on data reception
    else {
        qos = std::make_shared<DataReaderQos>();
        qos->reliability().kind = RELIABLE_RELIABILITY_QOS;
        qos->durability().kind = TRANSIENT_LOCAL_DURABILITY_QOS;
        qos->endpoint().history_memory_policy = PREALLOCATED_WITH_REALLOC_MEMORY_MODE;
        qos->history().kind = KEEP_LAST_HISTORY_QOS;
    }
    // Create the listener
    auto listener = new SynDDSDataReaderListener(callback, message);

    // Create the data reader
    StatusMask mask = StatusMask::all();
    if (is_synchronous)
        mask >> StatusMask::data_available();
    auto reader = subscriber->create_datareader(topic->GetDDSTopic(), *qos, listener, mask);
    if (!reader) {
        SynLog() << "CreateSubscriber: Reader instantiation FAILED\n";
        return nullptr;
    }

    auto syn_subscriber =
        std::make_shared<SynDDSSubscriber>(subscriber, reader, listener, topic, callback, message, is_synchronous);
    if (is_managed)
        m_subscribers.push_back(syn_subscriber);

    return syn_subscriber;
}

std::shared_ptr<SynDDSSubscriber> SynDDSCommunicator::CreateSubscriber(const std::string& topic_name,
                                                                       TopicDataType* data_type,
                                                                       std::function<void(void*)> callback,
                                                                       void* message,
                                                                       bool is_synchronous,
                                                                       bool is_managed) {
    if (!m_participant) {
        SynLog() << "CreateSubscriber: Participant is NULL\n";
        return nullptr;
    }

    // Create the topic
    auto topic = CreateTopic(topic_name, data_type);
    if (!topic->GetDDSTopic()) {
        SynLog() << "CreateSubscriber: Topic (" << topic_name << ") instantiation FAILED\n";
        return nullptr;
    }
	for(auto sub : m_subscribers) {
		if (topic->GetTopicName().find(sub->m_topic->GetTopicName()) != std::string::npos) {
			return nullptr;
		}
	}

    return CreateSubscriber(topic, callback, message, is_synchronous, is_managed);
}

std::shared_ptr<SynDDSPublisher> SynDDSCommunicator::CreatePublisher(std::shared_ptr<SynDDSTopic> topic,
                                                                     bool is_managed,
                                                                     eprosima::fastdds::dds::DataWriterQos* write_qos) {
    SynLog() << "Creating Publisher " << topic->GetFullTopicName() << "\n";

    if (!m_participant) {
        SynLog() << "CreatePublisher: Participant is NULL\n";
        return nullptr;
    }

    // Create the publisher
    auto publisher = m_participant->create_publisher(PUBLISHER_QOS_DEFAULT);
    if (!publisher) {
        SynLog() << "CreatePublisher: Publisher Instantiation FAILED\n";
        return nullptr;
    }

    std::shared_ptr<DataWriterQos> qos = nullptr;
    if (write_qos != nullptr) {
        qos = std::make_shared<DataWriterQos>(*write_qos);
    }
    // Create data reader qos and allow for automatic reallocation on data reception
    else {
        qos = std::make_shared<DataWriterQos>();
        qos->reliability().kind = RELIABLE_RELIABILITY_QOS;
        qos->endpoint().history_memory_policy = PREALLOCATED_WITH_REALLOC_MEMORY_MODE;
        qos->history().kind = KEEP_LAST_HISTORY_QOS;
    }

    // Create the listener
    auto listener = new SynDDSDataWriterListener();

    // Create the DataWriter
    auto writer = publisher->create_datawriter(topic->GetDDSTopic(), *qos, listener);
    if (!writer) {
        SynLog() << "CreatePublisher: Writer Instantiation FAILED\n";
        return nullptr;
    }

    auto syn_publisher = std::make_shared<SynDDSPublisher>(publisher, writer, listener, topic);
    if (is_managed)
        m_publishers.push_back(syn_publisher);

    return syn_publisher;
}

std::shared_ptr<SynDDSPublisher> SynDDSCommunicator::CreatePublisher(const std::string& topic_name,
                                                                     TopicDataType* data_type,
                                                                     bool is_managed) {
    if (!m_participant) {
        SynLog() << "CreatePublisher: Participant is NULL\n";
        return nullptr;
    }

    // Create the topic
    auto topic = this->CreateTopic(topic_name, data_type);
    if (!topic->GetDDSTopic()) {
        SynLog() << "CreateSubscriber: Topic (" << topic_name << ") instantiation FAILED\n";
        return nullptr;
    }

    return CreatePublisher(topic, is_managed);
}

// -----------------------------------------------------------------------------------------------

void SynDDSCommunicator::Publish() {
    SynDDSMessage msg;
    msg.data(m_flatbuffers_manager.ToMessageBuffer());

    for (auto publisher : m_publishers)
        publisher->Publish(&msg);

    m_flatbuffers_manager.Reset();
}

void SynDDSCommunicator::Listen() {
    for (auto subscriber : m_subscribers)
        if (subscriber->IsSynchronous())
            subscriber->Receive();
}

}  // namespace synchrono
}  // namespace chrono