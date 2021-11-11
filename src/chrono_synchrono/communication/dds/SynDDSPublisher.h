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
// Class that handles publishing on a specified topic. "Publishing" is
// essentially synonomous with "Sending". A topic is basically an abstract
// connection between a publisher and a receiver (known as a subscriber in DDS
// terms).
//
// Currently, a SynDDSPublisher can only be responsible for one topic. In the
// future, a SynDDSPublisher should be able to send the same message on more
// than one topic.
//
// Simply, a publisher is responsible for managing data writers. Data writers
// actually write data to the topics desired by the user.
//
// =============================================================================

#ifndef SYN_DDS_PUBLISHER_H
#define SYN_DDS_PUBLISHER_H

#include "chrono_synchrono/SynApi.h"

#include <vector>
#include <memory>

// Forward declare classes so we don't have to include them in the header
namespace eprosima {
namespace fastdds {
namespace dds {

class DomainParticipant;
class TypeSupport;
class Publisher;
class DataWriter;
class DataWriterListener;

}  // namespace dds
}  // namespace fastdds
}  // namespace eprosima

namespace chrono {
namespace synchrono {

class SynDDSTopic;
class SynDDSDataWriterListener;

/// @addtogroup synchrono_communication_dds
/// @{

/// DDS publisher wrapper. Sends information on a topic.
class SYN_API SynDDSPublisher {
  public:
    ///@brief Construct a new SynDDSPublisher object
    ///
    SynDDSPublisher(eprosima::fastdds::dds::Publisher* publisher,
                    eprosima::fastdds::dds::DataWriter* writer,
                    SynDDSDataWriterListener* listener,
                    std::shared_ptr<SynDDSTopic> topic);

    ///@brief Destroy the SynDDSPublisher object
    ~SynDDSPublisher();

    ///@brief Delete underlying DDS objects using the passed participant
    ///
    void DeleteDDSEntities(eprosima::fastdds::dds::DomainParticipant* participant);

    ///@brief Publish a message on the publishers topic
    /// Currently only can publish to one topic
    ///
    ///@param message the type supported message that will be distributed on the topic
    bool Publish(void* message);

    ///@brief Wait for the specified number of matches
    /// Each subscriber listener has a callback that will be called when a subscriber is matched with
    /// a DataWriter. This function blocks until that the matches are achieved. By default,
    /// a subscriber will just wait for a single listener.
    ///
    void WaitForMatches(unsigned int matches);

    eprosima::fastdds::dds::Publisher* GetPublisher() { return m_publisher; }
    eprosima::fastdds::dds::DataWriter* GetDataWriter() { return m_writer; }

  private:
    eprosima::fastdds::dds::Publisher* m_publisher;  ///< FastDDS publisher that handles data writers
    eprosima::fastdds::dds::DataWriter* m_writer;    ///< FastDDS Data writer which handles data writing (sending)
    SynDDSDataWriterListener* m_listener;            ///< Listener used for listening for subscribers

    std::shared_ptr<SynDDSTopic> m_topic;  ///< the topic this publisher will send data on
};

typedef std::vector<std::shared_ptr<SynDDSPublisher>> PublisherList;

/// @} synchrono_communication

}  // namespace synchrono
}  // namespace chrono

#endif