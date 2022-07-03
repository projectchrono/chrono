#ifndef CH_EXPORT_SCREENSHOT_NOBLIT_H
#define CH_EXPORT_SCREENSHOT_NOBLIT_H

#include <vsg/all.h>

#include <vsgXchange/all.h>

#include <algorithm>
#include <chrono>
#include <iostream>
#include <thread>
#include <string>

void exportScreenshotNoBlit(vsg::ref_ptr<vsg::Window> window, std::string &imageFilename);

#endif