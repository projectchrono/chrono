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
// Class that handles subscribing on a specified topic. "Subscribing" is
// essentially synonomous with "Receiving". A topic is basically an abstract
// connection between a sender (known as a publisher in DDS terms) and a
// subscriber.
//
// Currently, a SynDDSSubscriber can only be responsible for one topic. In the
// future, a SynDDSSubscriber should be able to receive the data on more
// than one topic.
//
// Simply, a subscriber is responsible for managing data readers. Data readers
// actually read the data from the topics and save them for the user to use.
//
// =============================================================================

#ifndef SYN_DDS_SUBSCRIBER_H
#define SYN_DDS_SUBSCRIBER_H

#include "chrono_synchrono/SynApi.h"

#include <vector>
#include <memory>
#include <functional>

// Forward declare classes so we don't have to include them in the header
namespace eprosima {
namespace fastdds {
namespace dds {

class DomainParticipant;
class TypeSupport;
class Subscriber;
class DataReader;

}  // namespace dds
}  // namespace fastdds
}  // namespace eprosima

class SynDDSMessage;

namespace chrono {
namespace synchrono {

class SynDDSTopic;
class SynDDSDataReaderListener;

/// @addtogroup synchrono_communication_dds
/// @{

/// DDS subscriber wrapper. Listens for information on a topic.
class SYN_API SynDDSSubscriber {
  public:
    ///@brief Construct a new SynDDSSubscriber object
    ///
    SynDDSSubscriber(eprosima::fastdds::dds::Subscriber* subscriber,
                     eprosima::fastdds::dds::DataReader* reader,
                     SynDDSDataReaderListener* listener,
                     std::shared_ptr<SynDDSTopic> topic,
                     std::function<void(void*)> callback,
                     void* message,
                     bool is_synchronous = true);

    ///@brief Destroy the SynDDSSubscriber object
    ///
    virtual ~SynDDSSubscriber();

    ///@brief Delete underlying DDS objects using the passed participant
    ///
    void DeleteDDSEntities(eprosima::fastdds::dds::DomainParticipant* participant);

    ///@brief This function is responsible for synchronous receiving
    /// This will block until a message has been received. When a message is received,
    /// the passed function is called and the received data is passed as a parameter
    /// to the callback.
    ///
    ///@param wait_time timeout of the synchronous waiting
    void Receive(long double wait_time = 20.0);

    ////@brief This function is responsible for asynchronous receiving
    /// This function will return immediately. Underlying calls are asynchronous/non-blocking.
    /// When a message is received, the passed function is called and the received data
    /// is passed as a parameter to the callback.
    ///
    void AsyncReceive();

    ///@brief Wait for the specified number of matches
    /// Each subscriber listener has a callback that will be called when a subscriber is matched with
    /// a DataWriter. This function blocks until that the matches are achieved. By default,
    /// a subscriber will just wait for a single listener.
    ///
    void WaitForMatches(unsigned int matches);

    // -------------------------------------------------------------------------------------

    ///@brief Set the callback for each receive of a message
    ///
    ///@param callback the callback function called when a message is receive occurs
    void SetCallback(std::function<void(void*)> callback) { m_callback = callback; }

    ///@brief Set the message type for the SubscriberListener
    /// The message is a DDS generated message class that will be used to store
    /// the data in an easy to access way
    ///
    void SetMessage(void* message) { m_message = message; }

    // -------------------------------------------------------------------------------------

    ///@brief Get the DDS data reader
    ///
    eprosima::fastdds::dds::DataReader* GetDataReader() { return m_reader; }

    // -------------------------------------------------------------------------------------

    ///@brief Is the Subscriber set to a synchronous subscriber?
    ///
    bool IsSynchronous() { return m_is_synchronous; }
    eprosima::fastdds::dds::Subscriber* GetSubscriber() { return m_subscriber; }

  private:
    bool m_is_synchronous;  ///< Is the Subscriber set to a synchronous subscriber?

    eprosima::fastdds::dds::Subscriber* m_subscriber;  ///< FastDDS Subscriber that handles data readers
    eprosima::fastdds::dds::DataReader* m_reader;      ///< FastDDS Data reader which handles data reading
    SynDDSDataReaderListener* m_listener;  ///< Listener used for listening for publishers and receiving data

    std::shared_ptr<SynDDSTopic> m_topic;  ///< the topic this subscriber will receive data on
    void* m_message;                       ///< Pointer to the message type that will be used when receiving data

    std::function<void(void*)> m_callback;  ///< callback for each receive

	friend class SynDDSCommunicator;
};

typedef std::vector<std::shared_ptr<SynDDSSubscriber>> SubscriberList;

/// @} synchrono_communication

}  // namespace synchrono
}  // namespace chrono

#endif