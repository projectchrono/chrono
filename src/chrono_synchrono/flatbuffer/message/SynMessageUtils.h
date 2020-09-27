#ifndef SYN_MESSAGE_UTILS_H
#define SYN_MESSAGE_UTILS_H

#include "chrono_synchrono/flatbuffer/message/SynFlatBuffers_generated.h"

#include "chrono_vehicle/wheeled_vehicle/ChTire.h"
//#include "chrono/core/ChFrameMoving.h"

#include "chrono_synchrono/SynApi.h"

namespace chrono {
namespace synchrono {

class SYN_API SynPose {
  public:
    ///@brief Construct a new Pose object
    SynPose(const ChVector<>& mv = ChVector<>(0, 0, 0), const ChQuaternion<>& mq = ChQuaternion<>(1, 0, 0, 0));

    ///@brief Construct a new Pose object
    SynPose(const ChFrameMoving<>& frame);

    ///@brief Construct a new Pose object from a FlatBuffers pose object
    ///
    ///@param pose the FlatBuffers pose object
    SynPose(const SynFlatBuffers::Pose* pose);

    ///@brief Convert this pose object to a flatbuffers pose type
    ///
    ///@param builder the FlatBuffer builder used to construct messages
    ///@return flatbuffers::Offset<SynFlatBuffers::Pose> the flatbuffer pose
    flatbuffers::Offset<SynFlatBuffers::Pose> ToFlatBuffers(flatbuffers::FlatBufferBuilder& builder);

    ///@brief Perform a dead reckoning step
    ///
    ///@param dt time between most recent step
    void Step(double dt);

    ChFrameMoving<>& GetFrame() { return m_frame; }

  private:
    ChFrameMoving<> m_frame;
};

}  // namespace synchrono
}  // namespace chrono

#endif