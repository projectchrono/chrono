#ifndef CHDRIVERINPUTS_DATAPARSERFUNCTOR_H
#define CHDRIVERINPUTS_DATAPARSERFUNCTOR_H

#include "chrono_vehicle/driver/ChExternalDriver.h"

using namespace chrono::vehicle;

class ChDriverInputs_DataParserFunctor : public ChExternalDriver::DataParserFunctor {
  public:
    ChDriverInputs_DataParserFunctor(ChDriver& driver) : DataParserFunctor("ChDriverInputs"), m_driver(driver) {}

    virtual void Deserialize(rapidjson::Value& v) override {
        m_driver.SetThrottle(v["throttle"].GetDouble());
        m_driver.SetSteering(v["steering"].GetDouble());
        m_driver.SetBraking(v["braking"].GetDouble());
    }

  private:
    ChDriver& m_driver;
};

#endif
