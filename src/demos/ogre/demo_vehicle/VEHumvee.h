#pragma once

#include "VEVehicle.h"
#include "VESuspensionDemo.h"

namespace VehicleEnvironment {

	class VEHumvee : public VESuspensionDemo {

	public:

		VEHumvee();
		VEHumvee(EnvironmentCore::EnvironmentCoreApplication* App);
		~VEHumvee();

		virtual void build(chrono::ChVector<>& Pos);
		virtual void update();
		virtual void reset(chrono::ChVector<>& Pos);

		virtual void shift(uint8_t gear);
		virtual void brake();

        virtual chrono::ChSharedPtr<ChBody> getChassis();

	protected:

		enum PointId {
			SPINDLE,    // spindle location
			UPRIGHT,    // upright location
			UCA_F,      // upper control arm, chassis front
			UCA_B,      // upper control arm, chassis back
			UCA_U,      // upper control arm, upright
			LCA_F,      // lower control arm, chassis front
			LCA_B,      // lower control arm, chassis back
			LCA_U,      // lower control arm, upright
			SHOCK_C,    // shock, chassis
			SHOCK_U,    // shock, upright
			TIEROD_C,   // tierod, chassis
			TIEROD_U,   // tierod, upright
			NUM_POINTS
		};

		chrono::ChVector<> getFrontLocation(PointId which);
		chrono::ChVector<> getRearLocation(PointId which);

	private:



	};

}