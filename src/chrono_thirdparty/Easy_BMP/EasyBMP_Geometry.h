/*************************************************
*                                                *
*  EasyBMP Cross-Platform Windows Bitmap Library * 
*                                                *
*  Author: Paul Macklin                          *
*   email: pmacklin@math.uci.edu                 *
*                                                *
*    file: EasyBMP_Geometry.h                    *
*    date: 2-21-2005                             *
* version: 1.05.00                               *
*                                                *
*   License: BSD (revised)                       *
* Copyright: 2005-2006 by the EasyBMP Project    * 
*                                                *
* description: draw simple geometric objects     *
*                                                *
*************************************************/

#include "EasyBMP.h"

int ebmpRound( double input );
double InverseAngle( double Xdir, double Ydir );
double LineFunction( double SlopeX , double SlopeY, 
                     int StartX, int StartY, double TestX, double TestY );
void DrawAALine( BMP &Image , int FromX, int FromY, 
                 int ToX, int ToY , RGBApixel Color );
void DrawFastLine( BMP &Image , int FromX, int FromY, 
                   int ToX, int ToY , RGBApixel Color );
void DrawArc( BMP &Image , double CenterX, double CenterY , double Radius, 
              double FromTheta, double ToTheta , RGBApixel Color );
void DrawLine( BMP &Image , int FromX , int FromY, 
               int ToX, int ToY, RGBApixel Color );

