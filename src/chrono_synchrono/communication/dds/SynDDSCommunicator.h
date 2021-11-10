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

#ifndef SYN_DDS_COMMUNICATOR_H
#define SYN_DDS_COMMUNICATOR_H

#include "chrono_synchrono/SynApi.h"
#include "chrono_synchrono/communication/SynCommunicator.h"

#include "chrono_synchrono/communication/dds/SynDDSSubscriber.h"
#include "chrono_synchrono/communication/dds/SynDDSPublisher.h"
#include "chrono_synchrono/communication/dds/SynDDSTopic.h"

// Forward declare classes so we don't have to include them in the header
namespace eprosima {
namespace fastdds {
namespace dds {

class DomainParticipant;
class DomainParticipantQos;
class TopicDataType;
class DataReaderQos;
class DataWriterQos;

}  // namespace dds
}  // namespace fastdds
}  // namespace eprosima

namespace chrono {
namespace synchrono {

class SynDDSParticipantListener;

/// @addtogroup synchrono_communication_dds
/// @{

const std::string default_prefix = std::string("/syn/node/");

/// Derived communicator used to establish and facilitate communication between nodes.
/// Uses the Data Distribution Service (DDS) standard
class SYN_API SynDDSCommunicator : public SynCommunicator {
  public:
    SynDDSCommunicator(int node_id, const std::string& prefix = default_prefix);

    ///@brief Default constructor
    ///
    ///@param name The name to set to the qos
    SynDDSCommunicator(const std::string& name, const std::string& prefix = default_prefix);

    ///@brief Set the QoS directly from the constructor
    ///
    ///@param qos the Quality of Service to set for the participant
    SynDDSCommunicator(eprosima::fastdds::dds::DomainParticipantQos& qos, const std::string& prefix = default_prefix);

    ///@brief Destructor
    ///
    virtual ~SynDDSCommunicator();

    ///@brief Initialization function typically responsible for establishing a connection.
    /// Although not mandatory, this function should handle initial peer-to-peer communication.
    /// This could mean a simple handshake or an actual exchange of information used during the simulation.
    ///
    virtual void Initialize() override;

    ///@brief This function is responsible for continuous synchronization steps
    /// This function, depending on it's implementation, could be blocking or non-blocking.
    /// As in, depending on the derived class implementation, this function could use synchronous
    /// or asynchronous function calls to implement a communication interface.
    ///
    /// Synchronize will serialize the underlying message buffer if it hasn't yet
    ///
    virtual void Synchronize() override;

    ///@brief This function is responsible for blocking until an action is received or done.
    /// For example, a process may call Barrier to wait until another process has established
    /// certain classes and initialized certain quantities. This functionality should be implemented
    /// in this function.
    ///
    virtual void Barrier() override;

    // -----------------------------------------------------------------------------------------------

    ///@brief This function is responsible for blocking until the passed number of participants
    /// has been matched.
    ///
    ///@param num_participants number of participants to block until matched
    void Barrier(unsigned int num_participants);

    ///@brief Get the matched participant's names
    /// A matched participant is defined as a DDSParticipant Entity that has matched
    /// with the participant created on this node
    ///
    std::vector<std::string> GetMatchedParticipantNames();

    // -----------------------------------------------------------------------------------------------

    ///@brief Create a topic
    /// The topic will be registered with the participant, if desired
    ///
    /// The SynDDSCommunicator maintains a list of topics instantiated in the simulation.
    /// A topic can only by constructed through this function or indirectly through the
    /// CreateSubscriber/CreatePublisher functions
    ///
    ///@param topic_name The name associated with the topic
    ///@param data_type The topic data type
    ///@param topic_prefix The prefix to use
    std::shared_ptr<SynDDSTopic> CreateTopic(const std::string& topic_name,
                                             eprosima::fastdds::dds::TopicDataType* data_type,
                                             const std::string& topic_prefix);
	
	///@brief Create a topic using the communicator's prefix
    /// The topic will be registered with the participant, if desired
    ///
    ///@param topic_name The name associated with the topic
    ///@param data_type The topic data type
    std::shared_ptr<SynDDSTopic> CreateTopic(const std::string& topic_name,
                                             eprosima::fastdds::dds::TopicDataType* data_type);

    ///@brief Create a subscription to the specified topic
    /// Will call the callback when a subscription is received
    /// Takes a SynDDSTopic object (see CreateTopic)
    ///
    ///@param topic Topic object describing the DDS topic
    ///@param callback The callback called when data is received
    ///@param message The message that is used to parse the returned message
    ///@param is_synchronous Whether the subscriber synchronously receives data (default: true)
    ///@param is_managed Whether the SynDDSCommunicator is responsible for using the sending/receiving function calls
    ///@param read_qos Data Reader Quality of Service. Falls back to default if nullptr
    std::shared_ptr<SynDDSSubscriber> CreateSubscriber(std::shared_ptr<SynDDSTopic> topic,
                                                       std::function<void(void*)> callback,
                                                       void* message,
                                                       bool is_synchronous = true,
                                                       bool is_managed = false,
                                                       eprosima::fastdds::dds::DataReaderQos* read_qos = nullptr);

    ///@brief Create a subscription to the specified topic
    /// Will call the callback when a subscription is received
    /// Takes a string and TopicDataType and creates a topic (see CreateTopic)
    ///
    ///@param topic_name The name associated with the topic
    ///@param data_type The topic data type
    ///@param callback The callback called when data is received
    ///@param message The message that is used to parse the returned message
    ///@param is_synchronous Whether the subscriber synchronously receives data (default: true)
    ///@param is_managed Whether the SynDDSCommunicator is responsible for using the sending/receiving function calls
    std::shared_ptr<SynDDSSubscriber> CreateSubscriber(const std::string& topic_name,
                                                       eprosima::fastdds::dds::TopicDataType* data_type,
                                                       std::function<void(void*)> callback,
                                                       void* message,
                                                       bool is_synchronous = true,
                                                       bool is_managed = false);

    ///@brief Create a Publisher
    /// Returns a publisher handle to be used to publish information
    /// Takes a SynDDSTopic object (see CreateTopic)
    ///
    ///@param topic Topic object describing the DDS topic
    ///@param is_managed Whether the SynDDSCommunicator is responsible for using the sending/receiving function calls
    ///@param read_qos Data Writer Quality of Service. Falls back to default if nullptr
    std::shared_ptr<SynDDSPublisher> CreatePublisher(std::shared_ptr<SynDDSTopic> topic, 
                                                     bool is_managed = false,
                                                     eprosima::fastdds::dds::DataWriterQos* write_qos = nullptr);

    ///@brief Create a Publisher
    /// Returns a publisher handle to be used to publish information
    /// Takes a string and TopicDataType and creates a topic (see CreateTopic)
    ///
    ///@param topic_name The name associated with the topic
    ///@param data_type The topic data type
    ///@param is_managed Whether the SynDDSCommunicator is responsible for using the sending/receiving function calls
    std::shared_ptr<SynDDSPublisher> CreatePublisher(const std::string& topic_name,
                                                     eprosima::fastdds::dds::TopicDataType* data_type,
                                                     bool is_managed = false);

    // -----------------------------------------------------------------------------------------------

    std::string m_prefix;

  private:
    void InitQoS(const std::string& name);

    ///@brief Creates the underyling participant
    ///
    ///@param qos the Quality of Service to set to the participant
    void CreateParticipant(eprosima::fastdds::dds::DomainParticipantQos& qos);

    ///@brief Publish the outgoing messagse
    ///
    void Publish();

    ///@brief Listen for incoming messages
    /// Will only block if the underlying subscribers are synchronous
    ///
    void Listen();

    // -----------------------------------------------------------------------------------------------

    eprosima::fastdds::dds::DomainParticipant* m_participant;  ///< Domain Participant which maintains the pubs/subs

    TopicMap m_topics;
    PublisherList m_publishers;
    SubscriberList m_subscribers;

    SynDDSParticipantListener* m_listener;
};

/// @} synchrono_communication

}  // namespace synchrono
}  // namespace chrono

#endif