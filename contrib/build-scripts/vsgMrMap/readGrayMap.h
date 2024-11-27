//
// Created by Rainer Gericke on 25.11.24.
//

#ifndef METALROUGHNESS_READGRAYMAP_H
#define METALROUGHNESS_READGRAYMAP_H

#include "stb_image.h"
#include <string>

unsigned char* readGrayMap(std::string &inputFileName, int &width, int &height);

#endif//METALROUGHNESS_READGRAYMAP_H
