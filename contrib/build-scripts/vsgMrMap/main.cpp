#include "cxxopts.hpp"
#include "readGrayMap.h"
#include "stb_image.h"
#include "stb_image_write.h"
#include <iostream>
#include <string>

using namespace std;

int main(int argc, char *argv[]) {
    string roughnessFileName, metalnessFileName, outputFileName = "MrMap.jpg";
    double metalFactor = 0.0;
    unsigned char *rough_data = NULL;
    unsigned char *metal_data = NULL;
    int width = 0, height = 0, numChannel = 0;
    size_t numBytes = 0;
    cxxopts::Options options("MetalRoughness", "Generate VSG compatible Metalness/Roughness file.\n"
                                               "Allowed image formats: png jpg tga bmp\n"
                                               "Output file = MetalRoughness.jpg\n"
                                               "At least roughness file must be present!\n");
    options.add_options()("m,metalnessFile",
                          "Metalness graymap file name",
                          cxxopts::value<std::string>())("M,MetalnessFactor",
                                                         "Metalness Factor [0..1] instead of metalness map",
                                                         cxxopts::value<double>()->default_value("0.0"))("r,roughnessFile", "Roughness graymap file name", cxxopts::value<std::string>())("o,outputFile", "MrMap file name (*.jpg | *.png | *.bmp | *.tga)", cxxopts::value<std::string>()->default_value("MrMap.jpg"))("h,help", "Print usage");
    auto result = options.parse(argc, argv);
    if (result.count("help") || argc == 1) {
        std::cout << options.help() << std::endl;
        exit(0);
    }
    outputFileName = result["outputFile"].as<std::string>();
    if (result.count("roughnessFile")) {
        roughnessFileName = result["roughnessFile"].as<std::string>();
        rough_data = readGrayMap(roughnessFileName, width, height);
        if (rough_data == NULL) {
            cout << "Could not read roughness file!\n"
                 << endl;
            exit(1);
        }
        cout << "Roughness file <" << roughnessFileName << "> successfully loaded." << endl;
        numBytes = width * height;
    } else {
        std::cout << options.help() << std::endl;
        exit(1);        
    }
    if (result.count("metalnessFile")) {
        int tmpWidth = 0, tmpHeight = 0, tmpNumChannel = 0;
        metalnessFileName = result["metalnessFile"].as<std::string>();
        metal_data = readGrayMap(roughnessFileName, tmpWidth, tmpHeight);
        if (metal_data == NULL) {
            cout << "Could not read metalness file!\n"
                 << endl;
            exit(2);
        }
        cout << "Metalness file <" << metalnessFileName << "> successfully loaded." << endl;
        if ((width != tmpWidth) || (height != tmpHeight)) {
            cout << "Metalness and Roughness must be of the same size - bailing out." << endl;
            exit(22);
        }
    }
    if (metal_data == NULL) {
        if (result.count("MetalnessFactor")) {
            metalFactor = result["MetalnessFactor"].as<double>();
            metalFactor = clamp(metalFactor, 0.0, 1.0);
            cout << "Setting metalness factor to " << metalFactor << endl;
        } else {
            cout << "Setting default metalness factor to " << metalFactor << endl;
        }
    }
    size_t numOutBytes = 3 * numBytes;
    unsigned char *out_data = (unsigned char *) malloc(numOutBytes);
    if (out_data == NULL) {
        cout << "Could not allocate memory for output - bail out." << endl;
        exit(3);
    }
    // compose output image:
    //  red = zero
    //  green = roughness
    //  blue = metalness
    size_t numOutPixels = numOutBytes / 3;
    size_t iPosGray = 0;
    for (size_t iPixel = 0, iPos = 0; iPixel < numOutPixels; iPixel++, iPos += 3) {
        out_data[iPos] = 0;                       // red
        out_data[iPos + 1] = rough_data[iPosGray];// green
        if (metal_data)
            out_data[iPos + 2] = metal_data[iPosGray];// blue
        else
            out_data[iPos + 2] = (unsigned char) (metalFactor * 255.0);
        iPosGray++;
    }
    int ok = -99;
    if (outputFileName.ends_with(".jpg") || outputFileName.ends_with(".jpeg"))
        ok = stbi_write_jpg(outputFileName.c_str(), width, height, 3, out_data, 90);
    if (outputFileName.ends_with(".png"))
        ok = stbi_write_png(outputFileName.c_str(), width, height, 3, out_data, 0);
    if (outputFileName.ends_with(".tga") || outputFileName.ends_with(".targa"))
        ok = stbi_write_tga(outputFileName.c_str(), width, height, 3, out_data);
    if (outputFileName.ends_with(".bmp"))
        ok = stbi_write_bmp(outputFileName.c_str(), width, height, 3, out_data);
    if (ok > 0) {
        cout << "Output file <" << outputFileName << "> successfully written." << endl;
    }
    free(out_data);
    stbi_image_free(rough_data);
    if (metal_data) stbi_image_free(metal_data);
    return 0;
}
