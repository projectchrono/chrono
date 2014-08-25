#ifndef STRUTILS_H
#define STRUTILS_H

#include <vector>
#include <string>
#include <sstream>

#include "utils/ChApiUtils.h"

namespace utils {

// @brief utility class for reading input, e.g. pac tire parameter files
class CH_UTILS_API StrUtils{
public:
// some auxiliary functions
std::vector<std::string>& splitStr(const std::string &s, char delim, std::vector<std::string> &elems); 

std::vector<std::string> splitStr(const std::string &s, char delim); 


// convert a string to another type
template<class T> T fromString(const std::string& s);

// read an input line from tm.config, return the number that matters
template<class T> T fromTline(const std::string& tline);

};

}	// end namespace utils

#endif

