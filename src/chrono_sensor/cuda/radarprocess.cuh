namespace chrono {
namespace sensor {

void cuda_radar_angles(void* bufIn,
                       void* bufOut,
                       int width,
                       int height,
                       float hfov,
                       float max_v_angle,
                       float min_v_angle,
                       CUstream& stream);


void cuda_radar_pointcloud_from_depth(void* bufIn,
                                      void* bufOut,
                                      int width,
                                      int height,
                                      float hfov,
                                      float max_v_angle,
                                      float min_v_angle,
                                      CUstream& stream);

}
}