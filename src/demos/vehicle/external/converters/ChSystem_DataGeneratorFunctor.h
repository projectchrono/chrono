#ifndef CHSYSTEM_DATAGENERATORFUNTOR_H
#define CHSYSTEM_DATAGENERATORFUNTOR_H

#include "chrono_vehicle/driver/ChExternalDriver.h"

using namespace chrono;
using namespace chrono::vehicle;

class ChSystem_DataGeneratorFunctor : public ChExternalDriver::DataGeneratorFunctor {
  public:
    ChSystem_DataGeneratorFunctor(ChSystem* system, const std::string& id)
        : DataGeneratorFunctor("ChSystem", id), m_system(system) {}

    virtual void Serialize(ChJSONWriter& writer) override { writer.Key("time") << m_system->GetChTime(); }

  private:
    ChSystem* m_system;
};

#endif
