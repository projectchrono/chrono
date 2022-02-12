#ifndef CHVEHICLE_DATAGENERATORFUNCTOR_H
#define CHVEHICLE_DATAGENERATORFUNCTOR_H

#include "chrono_vehicle/driver/ChExternalDriver.h"

using namespace chrono::vehicle;

class ChVehicle_DataGeneratorFunctor : public ChExternalDriver::DataGeneratorFunctor {
  public:
    ChVehicle_DataGeneratorFunctor(ChVehicle& vehicle, const std::string& id)
        : DataGeneratorFunctor("ChVehicle", id), m_vehicle(vehicle) {}

    virtual void Serialize(ChJSONWriter& writer) override {
        auto body = m_vehicle.GetChassisBody();

        writer.Key("pos") << body->GetPos();
        writer.Key("rot") << body->GetRot();
        writer.Key("lin_vel") << body->GetPos_dt();
        writer.Key("ang_vel") << body->GetWvel_loc();
        writer.Key("lin_acc") << body->GetPos_dtdt();
        writer.Key("ang_acc") << body->GetWacc_loc();
    }

  private:
    ChVehicle& m_vehicle;
};

#endif
