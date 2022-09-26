#ifndef CHCAMERASENSOR_DATAGENERATORFUNTOR_H
#define CHCAMERASENSOR_DATAGENERATORFUNTOR_H

#include "chrono_vehicle/driver/ChExternalDriver.h"

#include "chrono_sensor/sensors/ChCameraSensor.h"
#include "chrono_sensor/filters/ChFilterAccess.h"

using namespace chrono::vehicle;
using namespace chrono::sensor;

// Create a data generator to add to the external driver
// This will send the camera image the external control stack
class ChCameraSensor_DataGeneratorFunctor : public ChExternalDriver::DataGeneratorFunctor {
  public:
    ChCameraSensor_DataGeneratorFunctor(const std::string& id, std::shared_ptr<ChCameraSensor> camera)
        : DataGeneratorFunctor("ChCameraSensor", id), m_camera(camera) {}

    virtual void Serialize(ChJSONWriter& writer) override {
        auto rgba8_ptr = m_camera->GetMostRecentBuffer<UserRGBA8BufferPtr>();
        if (rgba8_ptr->Buffer) {
            std::string image(reinterpret_cast<char*>(rgba8_ptr->Buffer.get()),
                              rgba8_ptr->Width * rgba8_ptr->Height * sizeof(PixelRGBA8));

            writer.Key("width") << rgba8_ptr->Width;
            writer.Key("height") << rgba8_ptr->Height;
            writer.Key("size") << sizeof(PixelRGBA8);
            writer.Key("encoding") << "rgba8";
            writer.Key("image") << image;
        }
    }

    virtual bool HasData() override {
        auto rgba8_ptr = m_camera->GetMostRecentBuffer<UserRGBA8BufferPtr>();
        return rgba8_ptr->Buffer != nullptr;
    }

  private:
    std::shared_ptr<ChCameraSensor> m_camera;
};

#endif

