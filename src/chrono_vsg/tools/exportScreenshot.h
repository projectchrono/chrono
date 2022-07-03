#include <vsg/all.h>

#ifdef vsgXchange_FOUND
#    include <vsgXchange/all.h>
#endif

#include <algorithm>
#include <chrono>
#include <iostream>
#include <thread>
#include <string>

void exportScreenshot(vsg::ref_ptr<vsg::Window> window, std::string &imageFilename);

