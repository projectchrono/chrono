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
// Authors: Aaron Young
// =============================================================================
//
// File description
//
// =============================================================================

#include "chrono_synchrono/communication/dds/SynDDSParticipant.h"

#include "chrono_synchrono/communication/dds/idl/SynDDSMessagePubSubTypes.h"

namespace chrono {
namespace synchrono {

SynDDSParticipant::SynDDSParticipant(std::string name)
    : m_name(name), m_participant(nullptr), m_listener(this), m_type(new SynDDSMessagePubSubType()) {}

SynDDSParticipant::~SynDDSParticipant() {
    // Clean the Publishers
    for (auto publisher : m_publishers) {
        if (publisher) {
            m_participant->delete_publisher(publisher->GetPublisher());
            delete publisher;
        }
    }

    // Clean the Subscribers
    for (auto subscriber : m_subscribers) {
        if (subscriber) {
            m_participant->delete_subscriber(subscriber->GetSubscriber());
            delete subscriber;
        }
    }

    // Clean the Participant
    DomainParticipantFactory::get_instance()->delete_participant(m_participant);
}

bool SynDDSParticipant::Initialize(std::vector<uint8_t>* user_data) {
    // Assigns a name to the participant through the QoS of the DomainParticipant
    DomainParticipantQos qos;
    qos.name(m_name);

    if (user_data) {
        // UserDataQosPolicy is used to pass information about the agent between ranks
        UserDataQosPolicy user_data_qos;
        user_data_qos.data_vec(*user_data);
        qos.user_data(user_data_qos);
    }

    // TODO: Add a locator
    // qos.transport().use_builtin_transports = false;
    // auto transport = chrono_types::make_shared<UDPv4TransportDescriptor>();
    // qos.transport().user_transports.push_back(transport);

    // eprosima::fastrtps::rtps::Locator_t locator;
    // IPLocator::setIPv4(locator, "128.104.190.1");
    // locator.port = 0;
    // qos.wire_protocol().builtin.initialPeersList.push_back(locator);

    // Create the participant
    // DomainId, qos, listener
    StatusMask mask = StatusMask::data_available();
    m_participant = DomainParticipantFactory::get_instance()->create_participant(0, qos, &m_listener, mask);
    if (!m_participant)
        return false;

    // Register the type
    m_type.register_type(m_participant);

    return true;
}

void SynDDSParticipant::RegisterParticipant(DomainParticipant* participant,
                                            eprosima::fastrtps::rtps::ParticipantDiscoveryInfo& info) {
    auto subscriber = new SynDDSSubscriber(participant, info.info.m_participantName.to_string());
    subscriber->Initialize();

    // Store the data passed by the new participant
    // PushData(info.info.m_userData);

    m_subscribers.push_back(subscriber);
}

void SynDDSParticipant::ProcessReceivedMessage(SynDDSMessage& msg) {
    // Store the received data
    PushData(msg.data());
}

void SynDDSParticipant::WaitForMatches(unsigned int num_matches, unsigned int rate) {
    m_listener.WaitForMatches(num_matches - 1, rate);

    for (auto publisher : m_publishers)
        publisher->WaitForMatches();

    for (auto subscriber : m_subscribers)
        subscriber->WaitForMatches();
}

SynDDSParticipant::SynDDSParticipantListener::SynDDSParticipantListener(SynDDSParticipant* participant)
    : m_participant(participant) {}

void SynDDSParticipant::SynDDSParticipantListener::on_participant_discovery(
    DomainParticipant* participant,
    eprosima::fastrtps::rtps::ParticipantDiscoveryInfo&& info) {
    if (info.status == eprosima::fastrtps::rtps::ParticipantDiscoveryInfo::DISCOVERED_PARTICIPANT) {
        std::cout << "[" << participant->get_qos().name() << "]: Participant with name " << info.info.m_participantName
                  << " discovered" << std::endl;

        m_participant->RegisterParticipant(participant, info);

        m_matched++;

    } else if (info.status == eprosima::fastrtps::rtps::ParticipantDiscoveryInfo::REMOVED_PARTICIPANT ||
               info.status == eprosima::fastrtps::rtps::ParticipantDiscoveryInfo::DROPPED_PARTICIPANT) {
        m_participant->m_ok = false;

        std::cout << "[" << participant->get_qos().name() << "]: Participant with name " << info.info.m_participantName
                  << " lost" << std::endl;

        m_participant->DeregisterParticipant(participant, info);

        m_matched--;
    }
}

}  // namespace synchrono
}  // namespace chrono