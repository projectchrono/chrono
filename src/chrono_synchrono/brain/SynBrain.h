#ifndef SYN_BRAIN_H
#define SYN_BRAIN_H

#include "chrono_synchrono/SynApi.h"
#include "chrono_synchrono/flatbuffer/message/SynMessage.h"

namespace chrono {
namespace synchrono {

class SYN_API SynBrain {
  public:
    /// Constructor
    SynBrain(int rank) : m_rank(rank) {}

    /// Destructor
    virtual ~SynBrain() {}

    /// Advance the state of this brain until brain time syncs with passed time
    virtual void Advance(double step) = 0;

    /// Synchronize this brain to the specified time
    virtual void Synchronize(double time) = 0;

    /// Process an incoming message
    virtual void ProcessMessage(SynMessage* msg) = 0;

    /// Generate vector of SynMessage's to send
    virtual void GenerateMessagesToSend(std::vector<SynMessage*>& messages) = 0;

    /// Get this brain's rank
    int GetRank() { return m_rank; }

  protected:
    int m_rank;  ///< rank of this brain
};

}  // namespace synchrono
}  // namespace chrono

#endif  // SYN_BRAIN_H
