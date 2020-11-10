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

#include "chrono_synchrono/communication/dds/SynDDSPublisher.h"

#include <fastdds/dds/publisher/qos/DataWriterQos.hpp>

#include "chrono/core/ChLog.h"

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
    GetLog() << "Publishing to " << m_topic_name << "\n";
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
            GetLog() << "SynDDSPublisher::Publisher: Writer failed to receive acknowledgments."
                     << "\n";
            return false;
        }

        return true;
    }
    GetLog() << "Failed to publish."
             << "\n";
    return false;
}

SynDDSPublisher::SynDDSPublisherListener::SynDDSPublisherListener(SynDDSPublisher* publisher)
    : m_publisher(publisher) {}

SynDDSPublisher::SynDDSPublisherListener::~SynDDSPublisherListener() {}

void SynDDSPublisher::SynDDSPublisherListener::on_publication_matched(DataWriter* writer,
                                                                      const PublicationMatchedStatus& info) {
    if (info.current_count_change == 1) {
        m_matched = info.total_count;
        GetLog() << "Publisher matched."
                 << "\n";
    } else if (info.current_count_change == -1) {
        m_matched = info.total_count;
        m_publisher->m_ok = false;
        GetLog() << "Publisher unmatched."
                 << "\n";
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