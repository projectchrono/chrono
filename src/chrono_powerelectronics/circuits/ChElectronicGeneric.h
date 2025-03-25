// ==================================================================================================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2024 Federico Maria Reato, Matteo Santelia, Filippo Morlacchi, Claudio Ricci, Maurizio Zama, Iseo Serrature S.p.a, projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// ==================================================================================================================================================
// Authors: Federico Maria Reato, Matteo Santelia, Filippo Morlacchi, Claudio Ricci
// ==================================================================================================================================================

#ifndef CHELECTRONICGENERIC_H
#define CHELECTRONICGENERIC_H

#include "ChElectronicCircuit.h"


namespace chrono {
namespace powerelectronics {

class ChElectronicGeneric : public ChElectronicCircuit {
public:
    // Constructors
    ChElectronicGeneric(std::string Netlist_location, double t_step);
    void InputDefinition(std::map<std::string, double> PWLIn, std::map<std::string, double> FlowIn);

    // Overriden methods from ChElectronicCircuit
    void PreInitialize() override;   
    void PostInitialize() override;
    void PreAdvance(double dt_mbs) override;
    void PostAdvance(double dt_mbs) override;

private:
    std::map<std::string, double> PWLInVar;
    std::map<std::string, double> FlowInVar;
};


}
}
#endif // CHELECTRONICMOTOR_H
