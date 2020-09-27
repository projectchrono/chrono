#ifndef SYN_DDS_MANAGER_H
#define SYN_DDS_MANAGER_H

#include "chrono_synchrono/SynApi.h"

#include "chrono_synchrono/communication/SynCommunicationManager.h"

namespace chrono {
namespace synchrono {

// Forward declaration
class SynDDSParticipant;
class SynDDSPublisher;

///@brief Communication mode for the DDSManager
///
/// SYNCHRONOUS: Data listening and sending blocks
/// ASYNCHRONOUS: Data listening and sending does not block
enum class SynDDSMode { SYNCHRONOUS, ASYNCHRONOUS };

struct SYN_API SynDDSConfig {
    SynDDSMode mode = SynDDSMode::ASYNCHRONOUS;
};

SYN_API extern const SynDDSConfig DDS_CONFIG_DEFAULT;

/// TODO Allow for more than 1 subscriber and 1 publisher
class SYN_API SynDDSManager : public SynCommunicationManager {
  public:
    /// Construct a dds manager object
    SynDDSManager(int rank, int num_ranks, SynDDSConfig config = DDS_CONFIG_DEFAULT);

    /// Destructor
    ~SynDDSManager();

    /// @brief Initialize all agents.
    virtual bool Initialize() override;

    /// @brief Synchronize all zombie agents within each ranks environment
    virtual void Synchronize() override;

    /// @brief Blocks simulation from proceeding until all ranks have reached this method call
    virtual void Barrier() override;

  private:
    /// @brief Publish relevant data
    void Publish();

    /// @brief Listen for data
    bool Listen(bool synchronous = false);

  private:
    SynDDSConfig m_config;

    SynDDSParticipant* m_participant;  ///< FastDDS DomainParticipant wrapped for use in SynChrono
    SynDDSPublisher* m_publisher;      ///< FastDDS Publisher responsible for sending rank state

    std::chrono::high_resolution_clock::time_point m_last;
};

}  // namespace synchrono
}  // namespace chrono

#endif