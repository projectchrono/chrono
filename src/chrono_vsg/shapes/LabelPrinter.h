
#ifndef LABELPRINTER_H
#define LABELPRINTER_H

#include <vector>
#include <iostream>
#include "chrono_thirdparty/Easy_BMP/EasyBMP.h"
#include "chrono_thirdparty/Easy_BMP/EasyBMP_Geometry.h"

namespace chrono {
namespace vsg3d {

class LabelPrinter
{
public:
    LabelPrinter();
    void printToBmp(BMP &bmp, char* text);
    int getTextWidth(char * text);

private:
    struct segment {
        float x1;
        float y1;
        float x2;
        float y2;
    };

    struct glyph {
        float width=0.0;
        std::vector<segment> sg;
    };

    std::vector<glyph> glyphVector{
#include "init_rowmans.h"
    };

};

}
}
#endif
