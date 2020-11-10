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

#include "chrono_synchrono/communication/dds/SynDDSSubscriber.h"

#include "chrono_synchrono/communication/dds/SynDDSManager.h"

#include <fastdds/dds/subscriber/qos/DataReaderQos.hpp>

namespace chrono {
namespace synchrono {

SynDDSSubscriber::SynDDSSubscriber(DomainParticipant* participant, std::string topic_name, std::string topic_type)
    : m_participant(participant),
      m_topic_name(topic_name),
      m_topic_type(topic_type),
      m_subscriber(nullptr),
      m_reader(nullptr),
      m_topic(nullptr),
      m_listener(this) {}

SynDDSSubscriber::~SynDDSSubscriber() {
    // Clean the Subscriber
    if (m_reader && m_subscriber)
        m_subscriber->delete_datareader(m_reader);

    if (m_topic)
        m_participant->delete_topic(m_topic);

    if (m_subscriber)
        m_participant->delete_subscriber(m_subscriber);
}

bool SynDDSSubscriber::Initialize() {
    // Create the subscription topic
    m_topic = m_participant->create_topic(m_topic_name, m_topic_type, TOPIC_QOS_DEFAULT);
    if (!m_topic)
        return false;

    // Create the Subscriber
    m_subscriber = m_participant->create_subscriber(SUBSCRIBER_QOS_DEFAULT, nullptr);
    if (!m_subscriber)
        return false;

    // Create the DataReaderQos, of which allows for reallocation on data reception so that an unsized sequence could be
    // used in the idl file
    DataReaderQos qos;
    qos.endpoint().history_memory_policy = PREALLOCATED_WITH_REALLOC_MEMORY_MODE;

    // Create the DataReader
    m_reader = m_subscriber->create_datareader(m_topic, qos, &m_listener);
    if (!m_reader)
        return false;

    return true;
}

void SynDDSSubscriber::ReadMessage(SynDDSMessage& msg) {
    PushData(msg.data());
}

SynDDSSubscriber::SynDDSSubscriberListener::SynDDSSubscriberListener(SynDDSSubscriber* subscriber)
    : m_subscriber(subscriber) {}

void SynDDSSubscriber::SynDDSSubscriberListener::on_subscription_matched(DataReader* reader,
                                                                         const SubscriptionMatchedStatus& info) {
    if (info.current_count_change == 1) {
        m_matched = info.total_count;
        std::cout << "Subscriber matched." << std::endl;
    } else if (info.current_count_change == -1) {
        m_matched = info.total_count;
        m_subscriber->m_ok = false;
        std::cout << "Subscriber unmatched." << std::endl;
    } else {
        m_subscriber->m_ok = false;
        std::cout << info.current_count_change
                  << " is not a valid value for SubscriptionMatchedStatus "
                     "current count change"
                  << std::endl;
    }
}

void SynDDSSubscriber::SynDDSSubscriberListener::on_data_available(DataReader* reader) {
    SampleInfo info;
    if (reader->take_next_sample(&m_message, &info) != ReturnCode_t::RETCODE_OK) {
        std::cout << "SynDDSSubscriberListener::on_data_available: DataReader failed to read message." << std::endl;
        exit(-1);
    }

    m_subscriber->ReadMessage(m_message);
}

}  // namespace synchrono
}  // namespace chrono