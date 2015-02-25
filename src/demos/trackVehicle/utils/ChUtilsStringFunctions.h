// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All right reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Justin Madsen
// =============================================================================
//
// Some string manipulation utility functions.
//
// =============================================================================

#ifndef CH_UTILS_STRFUNCTIONS_H
#define CH_UTILS_STRFUNCTIONS_H

#include <vector>
#include <string>
#include <sstream>

namespace chrono{
namespace utils {


// some auxiliary functions
std::vector<std::string>& splitStr(const std::string &s, char delim, std::vector<std::string> &elems){
  std::stringstream ss(s);
  std::string item;
  while (getline(ss, item, delim)) {
    elems.push_back(item);
  }
  return elems;
}

std::vector<std::string> splitStr(const std::string &s, char delim) {
  std::vector<std::string> elems;
  return splitStr(s, delim, elems);
}


// convert a string to another type
template<class T> T fromString(const std::string& s){
  std::istringstream stream(s);
  T t;
  stream >> t;
  return t;
}

// read an input line from tm.config, return the number that matters
template<class T> T fromTline(const std::string& tline){
  T t = fromString<T>(splitStr(splitStr(tline, '/')[0], '=')[1]);
  return t;
}



} // end namespace utils
} // end namespace chrono


#endif
