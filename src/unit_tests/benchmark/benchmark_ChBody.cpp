#include "../ChTestConfig.h"
#include "physics/ChSystem.h"
#include <iostream>
using namespace chrono;
using namespace std;
#define TIME(X, Y)                                \
    timer.start();                                \
    for (int i = 0; i < body_list->size(); i++) { \
        X;                                        \
    }                                             \
    timer.stop();                                 \
    cout << Y << timer() << endl;

#define TIMEBODY(X, Y) TIME(body_list->at(i)->X, Y)

int main() {
    ChTimer<double> timer, full;
    const int num_bodies = 1000000;
    const double current_time = 1;
    const double time_step = .1;
    ChSystem dynamics_system;

    for (int i = 0; i < num_bodies; i++) {
        ChSharedBodyPtr body(new ChBody());
        body->SetPos(ChVector<>(rand() % 1000 / 1000.0, rand() % 1000 / 1000.0, rand() % 1000 / 1000.0));
        dynamics_system.AddBody(body);
    }

    std::vector<ChBody*>* body_list = dynamics_system.Get_bodylist();

    full.start();

    TIMEBODY(UpdateTime(current_time), "UpdateTime ");
    TIMEBODY(UpdateForces(current_time), "UpdateForces ");
    TIMEBODY(UpdateMarkers(current_time), "UpdateMarkers ");

    TIMEBODY(ClampSpeed(), "ClampSpeed ");
    TIMEBODY(ComputeGyro(), "ComputeGyro ");

    TIMEBODY(VariablesFbReset(), "VariablesFbReset ");
    TIMEBODY(VariablesFbLoadForces(time_step), "VariablesFbLoadForces ");
    TIMEBODY(VariablesQbLoadSpeed(), "VariablesQbLoadSpeed ");
    TIMEBODY(VariablesQbIncrementPosition(time_step), "VariablesQbIncrementPosition ");
    TIMEBODY(VariablesQbSetSpeed(time_step), "VariablesQbSetSpeed ");
    // TIMEBODY(VariablesBody().GetBodyInvInertia(), "GetBodyInvInertia ");

    full.stop();
    cout << "Total: " << full() << endl;
    timer.start();
    for (int i = 0; i < body_list->size(); i++) {
        body_list->at(i)->UpdateTime(current_time);
        body_list->at(i)->UpdateForces(current_time);
        body_list->at(i)->UpdateMarkers(current_time);
        body_list->at(i)->ClampSpeed();
        body_list->at(i)->ComputeGyro();
        body_list->at(i)->VariablesFbReset();
        body_list->at(i)->VariablesFbLoadForces(time_step);
        body_list->at(i)->VariablesQbLoadSpeed();
        body_list->at(i)->VariablesQbIncrementPosition(time_step);
        body_list->at(i)->VariablesQbSetSpeed(time_step);
    }
    timer.stop();
    cout << "SIngle Loop " << timer() << endl;

    return 0;
}
