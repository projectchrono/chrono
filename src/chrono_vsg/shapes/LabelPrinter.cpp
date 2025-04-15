#include "LabelPrinter.h"

namespace chrono {
namespace vsg3d {

LabelPrinter::LabelPrinter() {}

void LabelPrinter::printToBmp(BMP &bmp, char* text) {
    RGBApixel Red;
    Red.Red = 255;
    Red.Green = 0;
    Red.Blue = 0;
    Red.Alpha= 255;
    int ofs = 2;
    int twidth = getTextWidth(text)+2*ofs;
    int theight = 30+2*ofs;

    bmp.SetSize(twidth,theight);
    bmp.SetBitDepth(32);

    int w = 0;
    const char *p;
    for ( p=text; *p; p++ ) {
        // get the character c to be rendered
        int ci = *((unsigned char *)p)-32;
        for(int j=0; j<glyphVector.size(); j++) {
            if(j == ci) {
                for(int k=0; k<glyphVector[j].sg.size(); k++) {
                    int x1 = glyphVector[j].sg[k].x1 + ofs + w;
                    int y1 = glyphVector[j].sg[k].y1 + ofs;
                    int x2 = glyphVector[j].sg[k].x2 + ofs + w;
                    int y2 = glyphVector[j].sg[k].y2 + ofs;
                    DrawFastLine(bmp,x1,y1,x2,y2,Red);
                }
                w += glyphVector[j].width;
            }
        }
    }
}

int LabelPrinter::getTextWidth(char* text) {
    int w = 0;
    const char *p;
    for ( p=text; *p; p++ ) {
        // get the character c to be rendered
        int ci = *((unsigned char *)p)-32;
        for(int j=0; j<glyphVector.size(); j++) {
            if(j == ci) {
                w += glyphVector[j].width;
            }
        }
    }
    return w;
}

}
}

