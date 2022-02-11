#ifndef CHSYSTEM_DATAGENERATORFUNTOR_H
#define CHSYSTEM_DATAGENERATORFUNTOR_H

#include "chrono_vehicle/driver/ChExternalDriver.h"

using namespace chrono;
using namespace chrono::vehicle;

class ChSystem_DataGeneratorFunctor : public ChExternalDriver::DataGeneratorFunctor {
  public:
    ChSystem_DataGeneratorFunctor(ChSystem* system, const std::string& name)
        : DataGeneratorFunctor("ChSystem", name), m_system(system) {}

    virtual void Serialize(rapidjson::Writer<rapidjson::StringBuffer>& writer) override {
        writer.String("time");
        writer.Double(m_system->GetChTime());
    }

  private:
    ChSystem* m_system;
};

#endif
