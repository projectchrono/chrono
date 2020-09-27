#include "chrono_synchrono/communication/dds/SynDDSListener.h"

#include <thread>
#include <chrono>

namespace chrono {
namespace synchrono {

SynDDSListener::SynDDSListener() : m_matched(0) {}

void SynDDSListener::WaitForMatches(unsigned int num_matches, unsigned int rate) {
    while (num_matches > m_matched)
        std::this_thread::sleep_for(std::chrono::milliseconds(rate));
}

}  // namespace synchrono
}  // namespace chrono