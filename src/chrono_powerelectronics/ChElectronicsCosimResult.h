#ifndef COSIM_RESULT_H
#define COSIM_RESULT_H

#include <vector>
#include <string>
#include <stdlib.h>
#include <unordered_map>


struct CosimResults {
    std::vector<double> sim_time;                                // Contains the SPICE simulation time returned by the Python module, it is update at every call of the class
    std::vector<std::vector<double>> node_val;                   // Contains the voltage values at every SPICE circuit nodes returned by the Python module, it is update at every call of the class
    std::vector<std::string> node_name;                         // Contains the names of every SPICE circuit nodes returned by the Python module, it is update at every call of the class
    std::vector<std::vector<double>> branch_val;                 // Contains the current values at every SPICE circuit branches returned by the Python module, it is update at every call of the class
    std::vector<std::string> branch_name;                       // Contains the names of every SPICE circuit branches returned by the Python module, it is update at every call of the class
};

#endif