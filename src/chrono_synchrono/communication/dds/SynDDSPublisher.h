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

#ifndef SYN_DDS_PUBLISHER_H
#define SYN_DDS_PUBLISHER_H

#include "chrono_synchrono/SynApi.h"

#include "chrono_synchrono/communication/dds/SynDDSEntity.h"

#undef ALIVE  // Defined in Eigen and in FastDDS

#include "chrono_synchrono/communication/dds/idl/SynDDSMessage.h"

#include <fastdds/dds/domain/DomainParticipant.hpp>  // Acts as a container for all Entity objects and as a factory for Publisher, Subscriber and Topic objects
#include <fastdds/dds/publisher/DataWriter.hpp>  // Allows the application to set the value of the data to be published under a given Topic
#include <fastdds/dds/publisher/Publisher.hpp>  // The object responsible for the creation of DataReaders
#include <fastdds/dds/topic/TypeSupport.hpp>  // Provides the participant with the functions to serialize, deserialize and get the key of a specific data type
#include <fastdds/dds/publisher/PublisherListener.hpp>

using namespace eprosima::fastdds::dds;
using namespace eprosima::fastrtps::rtps;

namespace chrono {
namespace synchrono {

/// @addtogroup synchrono_communication_dds
/// @{

/// TODO: Class description here
class SYN_API SynDDSPublisher : public SynDDSEntity {
  public:
    ///@brief Construct a new SynDDSPublisher object
    SynDDSPublisher(DomainParticipant* participant, std::string topic_name, std::string topic_type = "SynDDSMessage");

    ///@brief Destroy the SynDDSPublisher object
    ~SynDDSPublisher();

    ///@brief Initialize the Publisher
    ///
    ///@return true successfully initialized the Publisher
    ///@return false failed to initialze the Publisher
    bool Initialize();

    ///@brief Get the underlying SynDDSListener object
    ///
    ///@return SynDDSListener&
    virtual SynDDSListener& GetListener() override { return m_listener; }

    ///@brief Publish the message
    bool Publish(SynDDSMessage& msg);

    ///@brief Get the Publisher object
    ///
    ///@return Publisher*
    Publisher* GetPublisher() { return m_publisher; }

    ///@brief Get the DataWriter object
    ///
    ///@return DataWriter*
    DataWriter* GetWriter() { return m_writer; }

  private:
    DomainParticipant* m_participant;  ///< FastDDS DomainParticipant responsible for handling all entities

    Publisher* m_publisher;         ///< FastDDS Subscriber that handles data readers
    DataWriter* m_writer;           ///< FastDDS Data reader which handles data reading
    Topic* m_topic;                 ///< FastDDS Topic associated with this subscriber
    std::string m_topic_name = "";  ///< Topic name
    std::string m_topic_type = "";  ///< Topic type

    class SYN_API SynDDSPublisherListener : public PublisherListener, public SynDDSListener {
      public:
        SynDDSPublisherListener(SynDDSPublisher* publisher);

        virtual ~SynDDSPublisherListener();

        void on_publication_matched(DataWriter* reader, const PublicationMatchedStatus& info) override;

      private:
        SynDDSPublisher* m_publisher;

    } m_listener;

    friend class SynDDSPublisherListener;
};

/// @} synchrono_communication_dds

}  // namespace synchrono
}  // namespace chrono

#endif