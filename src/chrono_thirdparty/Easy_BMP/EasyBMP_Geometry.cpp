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

#include "EasyBMP_Geometry.h"

int ebmpRound( double input )
{
 double output = floor( input );
 if( output - input >= 0.50 )
 { return (int) ( output+1 ); }
 return (int) output;
}

double InverseAngle( double Xdir, double Ydir )
{
 double pi = 3.14159265358979;
 double Norm = sqrt( Xdir*Xdir + Ydir*Ydir );
 if( Norm <= 0.25 )
 { return 0.0; }
 Xdir /= Norm; 
 Ydir /= Norm;

 double Xabs = fabs( Xdir );
 double Yabs = fabs( Ydir );
 double theta = 0.5*( acos( Xabs ) + asin( Yabs ) );

 if( Xdir >= 0.0 && Ydir >= 0.0 )
 { return theta; }
 if( Xdir < 0.0 && Ydir >= 0.0 )
 { return pi - theta; }
 if( Xdir < 0.0 && Ydir < 0.0 )
 { return pi + theta; }
 return 2*pi - theta; 
}

double LineFunction( double SlopeX , double SlopeY, int StartX, int StartY, double TestX, double TestY )
{
 return fabs( SlopeX*(TestY-StartY) - SlopeY*(TestX-StartX) );
}

void DrawAALine( BMP &Image , int FromX, int FromY, int ToX, int ToY , RGBApixel Color )
{
 double SlopeX = ToX-FromX;
 double SlopeY = ToY-FromY;

 if( fabs( SlopeX ) <= 0.8 && fabs( SlopeY ) <= 0.8 ) // nothing to do; don't bother
 {
  return;
 }
 double Norm = sqrt( Square( SlopeX ) + Square( SlopeY ) );
 SlopeX /= Norm; 
 SlopeY /= Norm;

 // bounds checking

 if( FromX >= Image.TellWidth() )
 { FromX = Image.TellWidth()-1; }
 if( FromX < 0 )
 { FromX = 0; }
 if( ToX >= Image.TellWidth() )
 { ToX = Image.TellWidth()-1; }
 if( ToX < 0 )
 { ToX = 0; }

 if( FromY >= Image.TellHeight() )
 { FromY = Image.TellHeight()-1; }
 if( FromY < 0 )
 { FromY = 0; }
 if( ToY >= Image.TellHeight() )
 { ToY = Image.TellHeight()-1; }
 if( ToY < 0 )
 { ToY = 0; }

 int LeftI = FromX;
 int RightI = ToX;
 if( RightI < LeftI ){ int temp = LeftI; LeftI = RightI; RightI = temp; }
 int LeftJ = FromY;
 int RightJ = ToY;
 if( RightJ < LeftJ ){ int temp = LeftJ; LeftJ = RightJ; RightJ = temp; }
 
 int i,j;
 for( i=LeftI ; i <= RightI ; i++ )
 {
  for( j=LeftJ ; j <= RightJ ; j++ )
  {
   double ii=0;
   double jj=0;
   double dx = 0.25;
   double dy = 0.25;
   double x = i-1.5*dx;
   double y = j-1.5*dx;
   
   double Temp = 0.0;
   
   for( ii=-2; ii<=1 ; ii++)
   {
    for( jj=-2 ; jj<=1 ; jj++)
    {
     x = i+ii*dx+0.5*dx;
     y = j+jj*dy+0.5*dy;
     double Temp1 = LineFunction( SlopeX, SlopeY , FromX, FromY, x,y ); 
     if( Temp1 <= 0.5 ){ Temp1 = 1.0; }else{ Temp1 = 0.0; }
     Temp+=Temp1;
    }   
   }
   
   Temp /= 16.0;
   double MinValue = 0.03125; // 1.0/32.0
   
   if( Temp > MinValue )
   {
    Image(i,j)->Red = (ebmpBYTE) (unsigned int) (  (1.0-Temp)*( (double) Image(i,j)->Red )
                                                        +Temp*( (double) Color.Red ) );
    Image(i,j)->Green = (ebmpBYTE) (unsigned int) (  (1.0-Temp)*( (double) Image(i,j)->Green )
                                                        +Temp*( (double) Color.Green ) );
    Image(i,j)->Blue = (ebmpBYTE) (unsigned int) (  (1.0-Temp)*( (double) Image(i,j)->Blue )
                                                          +Temp*( (double) Color.Blue ) );
   }
   
  }
 }
 
 return; 
}

void DrawFastLine( BMP &Image , int FromX, int FromY, int ToX, int ToY , RGBApixel Color )
{
 // bounds checking

 if( FromX >= Image.TellWidth() )
 { FromX = Image.TellWidth()-1; }
 if( FromX < 0 )
 { FromX = 0; }
 if( ToX >= Image.TellWidth() )
 { ToX = Image.TellWidth()-1; }
 if( ToX < 0 )
 { ToX = 0; }

 if( FromY >= Image.TellHeight() )
 { FromY = Image.TellHeight()-1; }
 if( FromY < 0 )
 { FromY = 0; }
 if( ToY >= Image.TellHeight() )
 { ToY = Image.TellHeight()-1; }
 if( ToY < 0 )
 { ToY = 0; }

 // source: http://www.gamedev.net/reference/articles/article1275.asp

 int dX = ToX - FromX;
 int dY = ToY - FromY;
 
 if( dX == 0 && dY == 0 ) // nothing to do; don't bother
 {
  return;
 }
 
 int Xinc1 = 1;
 if( dX < 0 )
 { Xinc1 = -1; dX = -dX; } 
 int Yinc1 = 1;
 if( dY < 0 )
 { Yinc1 = -1; dY = -dY; }
 
 int x = FromX;               // Start x off at the first pixel
 int y = FromY;               // Start y off at the first pixel
 
 int Xinc2 = Xinc1;
 int Yinc2 = Yinc1;
 
 double Denominator;
 double Numerator;
 int NumberToAdd;
 int NumberOfPixels;

 if ( dX >= dY )         // There is at least one x-value for every y-value
 {
  Xinc1 = 0;                  // Don't change the x when numerator >= denominator
  Yinc2 = 0;                  // Don't change the y for every iteration
  Denominator = dX+0.0;
  Numerator = 0.5*dX;
  NumberToAdd = dY;
  NumberOfPixels = dX;         // There are more x-values than y-values
 }
 else                          // There is at least one y-value for every x-value
 {
  Xinc2 = 0;                  // Don't change the x for every iteration
  Yinc1 = 0;                  // Don't change the y when numerator >= denominator
  Denominator = dY+0.0;
  Numerator = 0.5*dY;
  NumberToAdd = dX;
  NumberOfPixels = dY;         // There are more y-values than x-values
 }
 
 int CurrentPixel;

 for (CurrentPixel = 0; CurrentPixel <= NumberOfPixels; CurrentPixel++ )
 {
  Image(x,y)->Red = Color.Red;
  Image(x,y)->Green = Color.Green;
  Image(x,y)->Blue = Color.Blue;
  
  Numerator += NumberToAdd;   // Increase the numerator by the top of the fraction
  if( Numerator >= Denominator ) // Check if numerator >= denominator
  {
    Numerator -= Denominator; // Calculate the new numerator value
    x += Xinc1;               // Change the x as appropriate
    y += Yinc1;               // Change the y as appropriate
  }
  x += Xinc2;                 // Change the x as appropriate
  y += Yinc2;                 // Change the y as appropriate
 }
 
 return; 
}

void DrawArc( BMP &Image , double CenterX, double CenterY , double Radius, 
              double FromTheta, double ToTheta , RGBApixel Color )
{
 double pi = 3.14159265358979;

 while( ToTheta < FromTheta )
 { ToTheta += 2*pi; }

 double Arclength = (ToTheta-FromTheta)*Radius;
 if( fabs( Arclength ) <= 1e-5 ) // if it's short, don't bother
 { return; }
 
 // set up the final circle first
 
 BMP Downsampled;
 int FinalWidth = (int) floor( 2.0*Radius + 1.0); // was ceil ,also tried Round
 
 // new for testing
 Radius = 0.5*(FinalWidth-1.0);
 // end new for testing 
 
 int FinalHeight = FinalWidth;
 Downsampled.SetSize( FinalWidth , FinalHeight );
 
 // make a temporary circle of double resolution

 BMP Temp;
 double TempRadius = 2.0*Radius; 
 
 Temp.SetSize( 2*FinalWidth, 2*FinalHeight );
 double dTheta = 1.0/TempRadius;
 
 double CenterXtemp = FinalWidth - 0.5; // was TempRadius + .5
 double CenterYtemp = FinalHeight - 0.5; // was TempRadius + 0.5;

 int x,y;
 
 double OuterRadiusSquared = TempRadius+1.0;
 OuterRadiusSquared *= OuterRadiusSquared;
 double InnerRadiusSquared = TempRadius-1.0;
 InnerRadiusSquared *= InnerRadiusSquared;
 
 for( x=0 ; x < Temp.TellWidth() ;x++)
 {
  for( y=0 ; y < Temp.TellHeight() ; y++)
  {
   double X = x-CenterXtemp;
   double Y = y-CenterYtemp;
   double TempRadiusSquared = X*X + Y*Y;
   
   if( InnerRadiusSquared <= TempRadiusSquared && 
       TempRadiusSquared <= OuterRadiusSquared )
   {
    double Angle = InverseAngle( X, Y );
    bool PlotIt = false;
    if( FromTheta <= Angle && Angle <= ToTheta )
    { PlotIt = true; }
    Angle += 2*pi;
    if( FromTheta <= Angle && Angle <= ToTheta )
    { PlotIt = true; }
    Angle -= 4*pi;
    if( FromTheta <= Angle && Angle <= ToTheta )
    { PlotIt = true; }
    
    if( PlotIt )
    { Temp(x,y)->Red = 0; }
   }
  }
 }
 
 // downsample to anti-alias
 
 int i,j;
 for( i=0 ; i < Downsampled.TellWidth() ; i++ )
 {
  for( j=0 ; j < Downsampled.TellHeight() ; j++ )
  {
   double TempRed = 0.0;
  
   int k,ell;
   for( k=0 ; k <= 1 ; k++ )
   {
    for( ell=0 ; ell <= 1 ; ell++ )
    {
     int I = 2*i+k;
     int J = 2*j+ell;
     TempRed += (double) Temp(I,J)->Red;
    }
   }
   TempRed /= 4.0;
   
   Downsampled(i,j)->Red = (ebmpBYTE) TempRed;
  }
 }
 
 // paste with alpha blending 
 
 int DestinationTopLeft = ebmpRound( CenterX - Radius );
 int DestinationTopRight = ebmpRound( CenterY - Radius );
 
 for( i=0 ; i < Downsampled.TellWidth() ; i++) 
 {
  for( j=0 ; j < Downsampled.TellHeight() ; j++)
  {
   int DestinationI = DestinationTopLeft+i;
   int DestinationJ = DestinationTopRight+j;
   if( DestinationI >= 0 && DestinationI < Image.TellWidth() &&
       DestinationJ >= 0 && DestinationJ < Image.TellHeight() )
   {
    double alpha = (255.0 - Downsampled(i,j)->Red )/255.0;
    Image(DestinationI,DestinationJ)->Red = (ebmpBYTE) ( 
      (1.0-alpha)*Image(DestinationI,DestinationJ)->Red 
    + (alpha)*Color.Red );
    Image(DestinationI,DestinationJ)->Green = (ebmpBYTE) ( 
      (1.0-alpha)*Image(DestinationI,DestinationJ)->Green 
    + (alpha)*Color.Green );
    Image(DestinationI,DestinationJ)->Blue = (ebmpBYTE) ( 
      (1.0-alpha)*Image(DestinationI,DestinationJ)->Blue 
    + (alpha)*Color.Blue );
   }
  
  }
 }
 
 return;
}

void DrawLine( BMP &Image , int FromX , int FromY, int ToX, int ToY, RGBApixel Color )
{
 if( FromX != ToX && FromY != ToY )
 {
  DrawAALine( Image, FromX, FromY, ToX, ToY, Color);
 } 
 DrawFastLine( Image, FromX, FromY, ToX, ToY, Color); 
 return;
}
