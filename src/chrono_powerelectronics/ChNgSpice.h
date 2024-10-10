#ifndef CHNGSPICE_HPP
#define CHNGSPICE_HPP

#include <iostream>
#include <ngspice/sharedspice.h>
#include <string>
#include <vector>
#include <fstream>
#include <cmath>   // For log10 and floor
#include <cstdlib> // For rand and RAND_MAX

class ChNgSpice {
public:
    // Constructor and Destructor
    ChNgSpice();
    ~ChNgSpice();

    // Load circuit from file
    bool loadCircuitFromFile(const std::string& filename, double dt_mbs);

    // Run the NGSpice simulation
    bool runSimulation();

    // Get the results from the simulation
    std::tuple<std::vector<std::string>, std::vector<std::vector<double>>, std::vector<std::string>, std::vector<std::vector<double>>> extractResults();

    void runTransientAnalysis(std::vector<std::string> netlist, double t_step, double dt_mbs, bool initial);

private:
    // NGSpice callback functions
    static int ngGetChar(char* output, int id, void* userdata);
    static int ngThreadRuns(char* status, int id, void* userdata);

    // Circuit data in NGSpice format
    std::vector<char*> ngspiceCircuit;

    // Helper to convert circuit lines to NGSpice format
    std::vector<std::string> readCircuitFromFile(const std::string& filename);

    // Node and branch information
    std::vector<std::string> nodeNames;
    std::vector<double> nodeValues;
    std::vector<std::string> branchNames;
    std::vector<double> branchValues;

    // Poll NGSpice until simulation completes
    bool waitForCompletion();
};

#endif // CHNGSPICE_HPP
