#ifndef CHVEHICLE_DATAGENERATORFUNCTOR_H
#define CHVEHICLE_DATAGENERATORFUNCTOR_H

#include "chrono_vehicle/driver/ChExternalDriver.h"

using namespace chrono::vehicle;

class ChVehicle_DataGeneratorFunctor : public ChExternalDriver::DataGeneratorFunctor {
  public:
    ChVehicle_DataGeneratorFunctor(ChVehicle& vehicle, const std::string& name)
        : DataGeneratorFunctor("ChVehicle", name), m_vehicle(vehicle) {}

    virtual void Serialize(rapidjson::Writer<rapidjson::StringBuffer>& writer) override {
        auto body = m_vehicle.GetChassisBody();

        writer.String("pos");
        this->Serialize_ChVector(writer, body->GetPos());
        writer.String("rot");
        this->Serialize_ChQuaternion(writer, body->GetRot());
        writer.String("lin_vel");
        this->Serialize_ChVector(writer, body->GetPos_dt());
        writer.String("ang_vel");
        this->Serialize_ChVector(writer, body->GetWvel_loc());
        writer.String("lin_acc");
        this->Serialize_ChVector(writer, body->GetPos_dtdt());
        writer.String("ang_acc");
        this->Serialize_ChVector(writer, body->GetWacc_loc());
    }

  private:
    ChVehicle& m_vehicle;
};

#endif
