//
// Created by Rainer Gericke on 25.11.24.
//

#include "readGrayMap.h"

unsigned char *readGrayMap(std::string &inputFileName, int &width, int &height) {
    int numChannel = 1;
    int tmpHeight = 0, tmpWidth = 0;
    unsigned char *tmpPtr = stbi_load(inputFileName.c_str(), &tmpWidth, &tmpHeight, &numChannel, 0);
    width = tmpWidth;
    height = tmpHeight;
    if (tmpPtr == NULL) {
        return tmpPtr;
    }
    switch (numChannel) {
        case 1:
            // input image is already a one-byte graymap
            return tmpPtr;
        case 2:
            // input image is a one-byte graymap with alpha
            {
                int numByteOut = width * height;
                unsigned char *outPtr = (unsigned char *) malloc(numByteOut);
                unsigned char *inPtr = tmpPtr;
                for (size_t iPixel = 0; iPixel < numByteOut; iPixel++) {
                    unsigned char gray = *inPtr;
                    outPtr[iPixel] = gray;
                    inPtr += 2;
                }
                // release unneeded memory
                stbi_image_free(tmpPtr);
                return outPtr;
            }
            break;
        case 3:
            // input image is a graymap in rgb format
            {
                int numByteOut = width * height;
                unsigned char *outPtr = (unsigned char *) malloc(numByteOut);
                unsigned char *inPtr = tmpPtr;
                for (size_t iPixel = 0; iPixel < numByteOut; iPixel++) {
                    double r = *inPtr;
                    double g = *(inPtr + 1);
                    double b = *(inPtr + 2);
                    unsigned char gray = (unsigned char) ((r + g + b) / 3.0);
                    outPtr[iPixel] = gray;
                    inPtr += 3;
                }
                // release unneeded memory
                stbi_image_free(tmpPtr);
                return outPtr;
            }
            break;
        case 4:
            // input image is a graymap in rgba format
            {
                int numByteOut = width * height;
                unsigned char *outPtr = (unsigned char *) malloc(numByteOut);
                unsigned char *inPtr = tmpPtr;
                for (size_t iPixel = 0; iPixel < numByteOut; iPixel++) {
                    double r = *inPtr;
                    double g = *(inPtr + 1);
                    double b = *(inPtr + 2);
                    unsigned char gray = (unsigned char) ((r + g + b) / 3.0);
                    outPtr[iPixel] = gray;
                    inPtr += 4;
                }
                // release unneeded memory
                stbi_image_free(tmpPtr);
                return outPtr;
            }
            break;
        default:
            // something has been loaded but cannot be processed
            if (tmpPtr) stbi_image_free(tmpPtr);
            return NULL;
    }
    return tmpPtr;
}
