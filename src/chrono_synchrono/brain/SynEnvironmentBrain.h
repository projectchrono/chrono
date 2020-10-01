#ifndef SYN_ENVIRONMENTBRAIN_H
#define SYN_ENVIRONMENTBRAIN_H

#include "chrono_synchrono/brain/SynBrain.h"

namespace chrono {
namespace synchrono {

class SynEnvironmentBrain : public SynBrain {
  public:
    SynEnvironmentBrain(int rank) : SynBrain(rank){};

    virtual void Synchronize(double time) override{};
    virtual void Advance(double step) override{};
    virtual void ProcessMessage(SynMessage* msg) override{};
    virtual void GenerateMessagesToSend(std::vector<SynMessage*>& messages) override{};
};

}  // namespace synchrono
}  // namespace chrono

#endif  // SYN_ENVIRONMENTBRAIN_H