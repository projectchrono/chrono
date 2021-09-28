namespace chrono {
namespace sensor {

void cuda_radar_angles(void* bufIn,
                       void* bufOut,
                       int width,
                       int height,
                       float hfov,
                       float vfov,
                       CUstream& stream);


void cuda_radar_pointcloud_from_angles(void* bufIn,
                                      void* bufOut,
                                      int width,
                                      int height,
                                      float hfov,
                                      float vfov,
                                      CUstream& stream);


void cuda_radar_pointcloud_from_depth(void* bufDI,
    void* bufOut,
    int width,
    int height,
    float hfov,
    float vfov,
    CUstream& stream);

}
}