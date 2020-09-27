#include "chrono_synchrono/flatbuffer/message/SynMessage.h"

namespace chrono {
namespace synchrono {

SynMessage::SynMessage(int rank, SynMessageType type) : m_rank(rank), m_type(type) {}

}  // namespace synchrono
}  // namespace chrono
