#ifndef SYN_ACC_BRAIN_H
#define SYN_ACC_BRAIN_H

#include "chrono_vehicle/driver/ChPathFollowerACCDriver.h"

#include "chrono_synchrono/SynApi.h"
#include "chrono_synchrono/brain/SynVehicleBrain.h"
#include "chrono_synchrono/brain/driver/SynMultipathDriver.h"

#include "chrono_synchrono/flatbuffer/message/SynEnvironmentMessage.h"

#include "chrono/core/ChVector.h"
#include "chrono_thirdparty/filesystem/path.h"

#ifdef SENSOR
#include "chrono_sensor/ChSensorManager.h"
#include "chrono_sensor/ChLidarSensor.h"
#include "chrono_sensor/ChCameraSensor.h"
#include "chrono_sensor/filters/ChFilterAccess.h"
#include "chrono_sensor/filters/ChFilterVisualize.h"
#include "chrono_sensor/filters/ChFilterVisualizePointCloud.h"
#include "chrono_sensor/filters/ChFilterPCfromDepth.h"
#include "chrono_sensor/filters/ChFilterSave.h"
using namespace chrono::sensor;
#endif

using namespace chrono;

namespace chrono {
namespace synchrono {

class SYN_API SynACCBrain : public SynVehicleBrain {
  public:
    SynACCBrain(int rank, std::shared_ptr<ChDriver> driver, ChVehicle& vehicle);
#ifdef SENSOR
    SynACCBrain(int rank, std::shared_ptr<ChDriver> driver, ChVehicle& vehicle, std::shared_ptr<ChLidarSensor> lidar);
#endif
    ~SynACCBrain();

    virtual void Synchronize(double time);
    virtual void Advance(double step);
    virtual void ProcessMessage(SynMessage* msg);

    void setMultipath(bool Mul) { m_is_multi_path = Mul; }
    bool m_sen_init_called;

#ifdef SENSOR
    void SetLidar(std::shared_ptr<ChLidarSensor> lidar) { m_lidar = lidar; }
#endif

    void SetNearestVehicleDistance(float dist) { m_nearest_vehicle = dist; }

    // virtual void updateMyLoc(chrono::Vector){};

  private:
    LaneColor m_light_color = LaneColor::RED;
    bool m_inside_box = false;  // Is vehicle inside stopping box area
    double m_dist = 1000;       // Distance to point to stop at light

    /// The intersection, approach and lane the vehicle is in, for traffic light checking. Only meaningful when
    /// m_inside_box is true
    int m_current_intersection = 0;
    int m_current_approach = 0;
    int m_current_lane = 0;

    double DistanceToLine(ChVector<> p, ChVector<> l1, ChVector<> l2);
    bool IsInsideBox(ChVector<> pos, ChVector<> sp, ChVector<> op, double w);
    bool IsInsideQuad(ChVector<> pos, ChVector<> sp1, ChVector<> sp2, ChVector<> cp3, ChVector<> cp4);
    void Barycentric(ChVector<> p, ChVector<> a, ChVector<> b, ChVector<> c, float& u, float& v, float& w);
    std::string getColorFromEnum(LaneColor c);
    int m_nearest_vehicle;

    bool m_is_multi_path;  ///< the agent is using a multiPathacc driver or not.

#ifdef SENSOR
    std::shared_ptr<ChSensorManager> m_manager;
    std::shared_ptr<ChLidarSensor> m_lidar;
    std::shared_ptr<ChCameraSensor> m_camera;

    bool m_save_data;
    std::string m_cam_data_path;
    std::string m_intersection_cam_data_path;

    double m_lidar_intensity_epsilon;
    UserDIBufferPtr m_recent_lidar_data;
#endif
};

}  // namespace synchrono
}  // namespace chrono

#endif
