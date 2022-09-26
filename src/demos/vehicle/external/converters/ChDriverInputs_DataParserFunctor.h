#ifndef CHDRIVERINPUTS_DATAPARSERFUNCTOR_H
#define CHDRIVERINPUTS_DATAPARSERFUNCTOR_H

#include "chrono_vehicle/driver/ChExternalDriver.h"

using namespace chrono::vehicle;

class ChDriverInputs_DataParserFunctor : public ChExternalDriver::DataParserFunctor {
  public:
    ChDriverInputs_DataParserFunctor(ChDriver& driver) : DataParserFunctor("ChDriverInputs"), m_driver(driver) {}

    virtual void Deserialize(ChJSONReader& reader) override {
        double steering, throttle, braking;
        reader >> steering >> throttle >> braking;

        m_driver.SetThrottle(throttle);
        m_driver.SetSteering(steering);
        m_driver.SetBraking(braking);
    }

  private:
    ChDriver& m_driver;
};

#endif
