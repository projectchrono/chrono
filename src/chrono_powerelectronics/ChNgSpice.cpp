#include "ChNgSpice.h"
#include <tuple>

// Static callback implementations
int ChNgSpice::ngGetChar(char* output, int id, void* userdata) {
    std::cout << output << std::endl;
    return 0;
}

int ChNgSpice::ngThreadRuns(char* status, int id, void* userdata) {
    bool isRunning = status[0] == '1';
    if (isRunning) {
        // std::cout << "NGSpice simulation is running." << std::endl;
    } else {
        // std::cout << "NGSpice simulation has finished." << std::endl;
    }
    return 0;
}

// Constructor
ChNgSpice::ChNgSpice() {
    if (ngSpice_Init(&ngGetChar, &ngThreadRuns, nullptr, nullptr, nullptr, nullptr, nullptr) != 0) {
        std::cerr << "Failed to initialize NGSpice." << std::endl;
        throw std::runtime_error("NGSpice initialization failed.");
    }
}

// Destructor
ChNgSpice::~ChNgSpice() {
    // Cleanup NGSpice resources if needed
}

// Load circuit from file
bool ChNgSpice::loadCircuitFromFile(const std::string& filename, double dt_mbs) {
    std::vector<std::string> circuit = readCircuitFromFile(filename);
    if (circuit.empty()) {
        std::cerr << "No circuit data found." << std::endl;
        return false;
    }

    // Convert circuit lines to NGSpice format
    for (const std::string& line : circuit) {
        ngspiceCircuit.push_back(const_cast<char*>(line.c_str()));
    }

    if (ngSpice_Circ(ngspiceCircuit.data()) != 0) {
        std::cerr << "Failed to load the circuit into NGSpice." << std::endl;
        return false;
    }

    return true;
}

void ChNgSpice::runTransientAnalysis(std::vector<std::string> netlist, double t_step, double dt_mbs) {
    std::string optCommand = ".options reltol=1e-6 abstol=1e-12";


    int t_step_exponent = static_cast<int>(std::floor(std::log10(std::abs(t_step))));
    double t_step_mantissa = t_step / std::pow(10, t_step_exponent);
    double t_step_rand = static_cast<double>(rand()) / static_cast<double>(RAND_MAX);;
    double t_step_new = (t_step_mantissa + t_step_rand) * std::pow(10, t_step_exponent);

    std::string tranCommand = ".tran " + std::to_string(t_step_new)  + " " + std::to_string(dt_mbs) + " uic";
    std::string uicCommand = ".uic";

    std::vector<char*> ngspiceCircuit;


    for (const std::string& line : netlist) {
        ngspiceCircuit.push_back(const_cast<char*>(line.c_str()));
    }

    // ngspiceCircuit.push_back(const_cast<char*>(optCommand.c_str()));
    // ngspiceCircuit.push_back(const_cast<char*>(uicCommand.c_str()));
    ngspiceCircuit.push_back(const_cast<char*>(tranCommand.c_str()));
    ngspiceCircuit.push_back(".end");
    ngspiceCircuit.push_back(nullptr); // Null-terminate the array

    if (ngSpice_Circ(ngspiceCircuit.data()) != 0) {
        std::cerr << "Failed to load the circuit into NGSpice." << std::endl;
        return;
    }

    if(runSimulation() == 0) {
        std::cerr << "Failed to run simulation." << std::endl;
        return;
    }


}


// Helper function to read circuit from a file
std::vector<std::string> ChNgSpice::readCircuitFromFile(const std::string& filename) {
    std::vector<std::string> circuit;
    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Failed to open file: " << filename << std::endl;
        return circuit;
    }

    std::string line;
    while (std::getline(file, line)) {
        circuit.push_back(line);
    }
    // circuit.push_back(".end"); // Ensure .end statement is included
    return circuit;
}

// Run simulation
bool ChNgSpice::runSimulation() {
    if (ngSpice_Command(const_cast<char*>("run")) != 0) {
        std::cerr << "Failed to run the NGSpice simulation." << std::endl;
        return false;
    }

    return waitForCompletion();
}

// Poll NGSpice to wait for simulation completion
bool ChNgSpice::waitForCompletion() {
    while (ngSpice_CurPlot() == nullptr) {
        // Polling or waiting for NGSpice to complete
    }
    return true;
}

// Extract results from the simulation
std::tuple<std::vector<std::string>, std::vector<std::vector<double>>, std::vector<std::string>, std::vector<std::vector<double>>> ChNgSpice::extractResults() {
    const char* currentPlot = ngSpice_CurPlot();
    
    // Containers for node and branch names/values
    std::vector<std::string> nodeNames;
    std::vector<std::vector<double>> nodeValues; // A vector of vectors to store all values
    std::vector<std::string> branchNames;
    std::vector<std::vector<double>> branchValues; // A vector of vectors to store all values

    if (currentPlot) {
        char** allVectors = ngSpice_AllVecs(const_cast<char*>(currentPlot));
        if (allVectors) {
            // std::cout << "Available vectors: " << std::endl;
            for (int i = 0; allVectors[i] != nullptr; i++) {
                std::string vecName = allVectors[i];
                // std::cout << vecName << std::endl;

                // Get vector info
                pvector_info vecInfo = ngGet_Vec_Info(const_cast<char*>(vecName.c_str()));
                if (vecInfo && vecInfo->v_name) {
                    std::vector<double> values(vecInfo->v_realdata, vecInfo->v_realdata + vecInfo->v_length); // Extract all values

                    if (vecName.find("#branch") != std::string::npos) {
                        std::string cleanName = vecName.substr(0, vecName.find("#branch"));
                        branchNames.push_back(cleanName);
                        branchValues.push_back(values); // Store all values for the branch
                    } else {
                        nodeNames.push_back(vecName);
                        nodeValues.push_back(values); // Store all values for the node
                    }
                }
            }
        }

        // Return the node and branch information with all values
        return std::make_tuple(nodeNames, nodeValues, branchNames, branchValues);
    } else {
        std::cerr << "No plot data available." << std::endl;
        return std::make_tuple(std::vector<std::string>{}, std::vector<std::vector<double>>{}, std::vector<std::string>{}, std::vector<std::vector<double>>{});
    }
}
