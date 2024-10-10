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

// =========================
// ======== Headers ========
// =========================
#include "ChElectronicsCosimulation.h"
#include "ChNgSpice.h"

namespace chrono {
namespace powerelectronics {

// ======== Method: allows to run a Spice Netlist simulation and solve the circuit ========
// CosimResults
std::map<std::string,std::vector<double>> ChElectronicsCosimulation::RunSpice(Netlist_V netlist, double t_step, double t_end)
{
    std::map<std::string, std::vector<double>> circuit_map;

    try {
        // Create an instance of the ChNgSpice class
        ChNgSpice ngspice;

        ngspice.runTransientAnalysis(netlist,t_step,t_end);

        // Extract and display the simulation results
        auto [nodeNames, nodeValues, branchNames, branchValues] = ngspice.extractResults();

        for (size_t i = 0; i < branchNames.size(); ++i) {
            circuit_map[branchNames[i]] = branchValues[i];
        }
        for (size_t i = 0; i < nodeNames.size(); ++i) {
            circuit_map[nodeNames[i]] = nodeValues[i];
        }

        std::vector<double> sim_time;
        

        this->results = CosimResults {
            sim_time, nodeValues, nodeNames, branchValues, branchNames
        };

        // // Display node names and values
        // std::cout << "\nNode Names and Values:\n";
        // for (size_t i = 0; i < nodeNames.size(); ++i) {
        //     std::cout << nodeNames[i] << ": " << nodeValues[i] << std::endl;
        // }

        // // Display branch names and values
        // std::cout << "\nBranch Names and Values:\n";
        // for (size_t i = 0; i < branchNames.size(); ++i) {
        //     std::cout << branchNames[i] << ": " << branchValues[i] << std::endl;
        // }

    } catch (const std::exception& ex) {
        std::cerr << "Error: " << ex.what() << std::endl;
        return circuit_map;
    }


    IncrementStep();

    return circuit_map;
}

// void Cosimulate(CosimResults results, double t_step, double t_end)
void ChElectronicsCosimulation::Cosimulate(CosimResults results, FlowInMap flow_in, PWLInMap pwl_in, double t_step, double t_end) 
{
    this->netlist.UpdateNetlist(results,flow_in,pwl_in,t_step,t_end);

    // this->netlist.WriteNetlist("../data/Circuit/MotorControl/Circuit_Netlist.cir.run");
}  

CosimResults ChElectronicsCosimulation::GetResult_V() {
    return this->results;
}

}
}