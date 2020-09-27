#ifndef SYN_DDS_PARTICIPANT_H
#define SYN_DDS_PARTICIPANT_H

#include "chrono_synchrono/SynApi.h"

#include "chrono_synchrono/communication/dds/SynDDSEntity.h"
#include "chrono_synchrono/communication/dds/SynDDSPublisher.h"
#include "chrono_synchrono/communication/dds/SynDDSSubscriber.h"

#undef ALIVE  // Defined in Eigen and in FastDDS

// Allows us to use the FastDDS API
// Common includes
#include <fastdds/dds/domain/DomainParticipant.hpp>
#include <fastdds/dds/domain/DomainParticipantFactory.hpp>
#include <fastdds/dds/domain/DomainParticipantListener.hpp>
#include <fastdds/rtps/participant/ParticipantDiscoveryInfo.h>

#include <fastdds/dds/topic/TypeSupport.hpp>

using namespace eprosima::fastdds::dds;
using namespace eprosima::fastrtps::rtps;

namespace chrono {
namespace synchrono {

class SYN_API SynDDSParticipant : public SynDDSEntity {
  public:
    ///@brief Construct a new SynDDSParticipant object
    ///
    ///@param name the name of the underlying FastDDS DomainParticipant
    SynDDSParticipant(std::string name);

    ///@brief Destroy the SynDDSParticipant object
    ~SynDDSParticipant();

    ///@brief Initialize the underlying DomainParticipant
    ///
    ///@param user_data information passed between DomainParticipants on handshake that describes each ranks simulation
    ///@return true successfully initialized the DomainParticipant
    ///@return false failed to initialze the DomainParticipant
    bool Initialize(std::vector<uint8_t>* user_data = nullptr);

    ///@brief Get the underlying SynDDSListener object
    ///
    ///@return SynDDSListener&
    virtual SynDDSListener& GetListener() override { return m_listener; }

    ///@brief Checks and waits for the passed number of matched FastDDS Entity at the specified rate
    ///
    ///@param num_matches number of entity matches to wait for. Defaults to 1.
    ///@param rate how often to check for a match in milliseconds. Defaults to 1000ms (1Hz).
    virtual void WaitForMatches(unsigned int num_matches = 1, unsigned int rate = 1000) override;

    ///@brief Add a new publisher to this participant
    ///
    ///@param publisher the new publisher to push to the member variable publishers
    void PushPublisher(SynDDSPublisher* publisher) { m_publishers.push_back(publisher); }

    ///@brief Get the SynDDSPublishers from this object
    ///
    ///@return std::vector<SynDDSPublisher*>
    std::vector<SynDDSPublisher*> GetPublishers() { return m_publishers; }

    ///@brief Get the SynDDSSubscribers from this object
    ///
    ///@return std::vector<SynDDSSubscriber*>
    std::vector<SynDDSSubscriber*> GetSubscribers() { return m_subscribers; }

    ///@brief Get the underlying DomainParticipant object
    ///
    ///@return DomainParticipant*
    DomainParticipant* GetParticipant() { return m_participant; }

  private:
    ///@brief Process the incoming message
    /// Just stores the data buffer
    ///
    ///@param msg the received FastDDS message
    void ProcessReceivedMessage(SynDDSMessage& msg);

    /// Registers to a newly discovered participant
    /// Is called by the SynDDSParticipantListener when a new participant is discovered
    void RegisterParticipant(DomainParticipant* participant, ParticipantDiscoveryInfo& info);

    // TODO
    /// Deregisters to a newly discovered participant
    /// Is called by the SynDDSParticipantListener when a new participant is dropped
    void DeregisterParticipant(DomainParticipant* participant, ParticipantDiscoveryInfo& info) {}

  private:
    std::string m_name = "";           ///< Name of the underlying DomainParticipant
    DomainParticipant* m_participant;  ///< FastDDS Domain participant which maintains the publisher and subscribers

    TypeSupport
        m_type;  ///< Data type sent between participants. SynDDSMessagePubSubType is the only current supported type.

    std::vector<SynDDSPublisher*> m_publishers;    ///< Stores each publisher for this rank
    std::vector<SynDDSSubscriber*> m_subscribers;  ///< Stores each subscriber for this rank

    class SYN_API SynDDSParticipantListener : public DomainParticipantListener, public SynDDSListener {
      public:
        SynDDSParticipantListener(SynDDSParticipant* participant);

        virtual ~SynDDSParticipantListener() {}

        virtual void on_participant_discovery(DomainParticipant* participant, ParticipantDiscoveryInfo&& info) override;

      private:
        SynDDSParticipant* m_participant;  ///< Pointer to the SynChrono DDS Participant

    } m_listener;

    friend class SynDDSParticipantListener;
};

}  // namespace synchrono
}  // namespace chrono

#endif