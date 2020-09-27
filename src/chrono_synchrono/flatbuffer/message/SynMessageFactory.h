#ifndef SYN_MESSAGE_FACTORY_H
#define SYN_MESSAGE_FACTORY_H

#include "chrono_synchrono/SynApi.h"

#include "chrono_synchrono/flatbuffer/message/SynMessage.h"
#include "chrono_synchrono/flatbuffer/message/SynSensorMessage.h"
#include "chrono_synchrono/flatbuffer/message/SynMAPMessage.h"
#include "chrono_synchrono/flatbuffer/message/SynSCMMessage.h"
#include "chrono_synchrono/flatbuffer/message/SynSPATMessage.h"
#include "chrono_synchrono/flatbuffer/message/SynWheeledVehicleMessage.h"
#include "chrono_synchrono/flatbuffer/message/SynTrackedVehicleMessage.h"
#include "chrono_synchrono/flatbuffer/message/SynEnvironmentMessage.h"

namespace chrono {
namespace synchrono {

/// Generates SynMessage in various ways
/// Used to improve generality in Agent classes
class SYN_API SynMessageFactory {
  public:
    /// Generate the corresponding SynMessage from a SynFlatBuffers::Message*
    static SynMessage* GenerateMessage(const SynFlatBuffers::Message* message);
};

}  // namespace synchrono
}  // namespace chrono

#endif
