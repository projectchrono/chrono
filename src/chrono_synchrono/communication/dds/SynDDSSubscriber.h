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

#ifndef SYN_DDS_SUBSCRIBER_H
#define SYN_DDS_SUBSCRIBER_H

#include "chrono_synchrono/SynApi.h"

#include "chrono_synchrono/communication/dds/SynDDSEntity.h"

#undef ALIVE  // Defined in Eigen and in FastDDS

#include "chrono_synchrono/communication/dds/idl/SynDDSMessage.h"

#include <fastdds/dds/domain/DomainParticipant.hpp>  // Acts as a container for all Entity objects and as a factory for Publisher, Subscriber and Topic objects
#include <fastdds/dds/subscriber/DataReader.hpp>  // The object responsible for the actual reception of data. Registers in the application the topic that identifies the data to be read and accesses the data received by the subscriber
#include <fastdds/dds/subscriber/SampleInfo.hpp>  // The information that accompanies each sample that is 'read' or 'taken'
#include <fastdds/dds/subscriber/Subscriber.hpp>  // The object responsible for the creation and configuration of the DataReaders
#include <fastdds/dds/subscriber/SubscriberListener.hpp>  // This is the listener assigned to the data reader
#include <fastdds/dds/topic/TypeSupport.hpp>  // Provides the participant with the functions to serialize, deserialize and get the key of a specific data type
#include <fastdds/dds/topic/Topic.hpp>
#include <fastdds/dds/subscriber/DataReaderListener.hpp>

using namespace eprosima::fastdds::dds;
using namespace eprosima::fastrtps::rtps;

namespace chrono {
namespace synchrono {

/// @addtogroup synchrono_communication_dds
/// @{

/// TODO: Class description here
class SYN_API SynDDSSubscriber : public SynDDSEntity {
  public:
    ///@brief Construct a new SynDDSSubscriber object
    SynDDSSubscriber(DomainParticipant* participant, std::string topic_name, std::string topic_type = "SynDDSMessage");

    ///@brief Destroy the SynDDSSubscriber object
    virtual ~SynDDSSubscriber();

    ///@brief Initialize the Subscriber
    ///
    ///@return true successfully initialized the Subscriber
    ///@return false failed to initialze the Subscriber
    bool Initialize();

    ///@brief Get the underlying SynDDSListener object
    ///
    ///@return SynDDSListener&
    virtual SynDDSListener& GetListener() override { return m_listener; }

    ///@brief Get the Subscriber object
    ///
    ///@return Subscriber*
    Subscriber* GetSubscriber() { return m_subscriber; }

    ///@brief Get the DataReader object
    ///
    ///@return DataReader*
    DataReader* GetReader() { return m_reader; }

  private:
    ///@brief Read the incoming message
    /// Just stores the data buffer
    ///
    ///@param msg the received FastDDS message
    void ReadMessage(SynDDSMessage& msg);

  private:
    DomainParticipant* m_participant;  ///< FastDDS DomainParticipant responsible for handling all entities

    Subscriber* m_subscriber;       ///< FastDDS Subscriber that handles data readers
    DataReader* m_reader;           ///< FastDDS Data reader which handles data reading
    Topic* m_topic;                 ///< FastDDS Topic associated with this subscriber
    std::string m_topic_name = "";  ///< Topic name
    std::string m_topic_type = "";  ///< Topic type

    class SYN_API SynDDSSubscriberListener : public DataReaderListener, public SynDDSListener {
      public:
        SynDDSSubscriberListener(SynDDSSubscriber* subscriber);

        void on_subscription_matched(DataReader* reader, const SubscriptionMatchedStatus& info) override;
        void on_data_available(DataReader* reader) override;

      private:
        SynDDSSubscriber* m_subscriber;  ///< Pointer to the SynChrono DDS Subscriber
        SynDDSMessage m_message;         ///< Message data type passed between participants

    } m_listener;

    friend class SynDDSSubscriberListener;
};

/// @} synchrono_communication_dds

}  // namespace synchrono
}  // namespace chrono

#endif