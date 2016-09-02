/*************************************************
*                                                *
*  EasyBMP Cross-Platform Windows Bitmap Library * 
*                                                *
*  Author: Paul Macklin                          *
*   email: macklin01@users.sourceforge.net       *
* support: http://easybmp.sourceforge.net        *
*                                                *
*          file: EasyBMP_VariousBMPutilities.h   *
*    date added: 05-02-2005                      *
* date modified: 12-01-2006                      *
*       version: 1.06                            *
*                                                *
*   License: BSD (revised/modified)              *
* Copyright: 2005-6 by the EasyBMP Project       * 
*                                                *
* description: Defines BMP class                 *
*                                                *
*************************************************/

#ifndef _EasyBMP_BMP_h_
#define _EasyBMP_BMP_h_

bool SafeFread( char* buffer, int size, int number, FILE* fp );
bool EasyBMPcheckDataSize( void );

class BMP
{private:

 int BitDepth;
 int Width;
 int Height;
 RGBApixel** Pixels;
 RGBApixel* Colors;
 int XPelsPerMeter;
 int YPelsPerMeter;

 ebmpBYTE* MetaData1;
 int SizeOfMetaData1;
 ebmpBYTE* MetaData2;
 int SizeOfMetaData2;
   
 bool Read32bitRow( ebmpBYTE* Buffer, int BufferSize, int Row );   
 bool Read24bitRow( ebmpBYTE* Buffer, int BufferSize, int Row );   
 bool Read8bitRow(  ebmpBYTE* Buffer, int BufferSize, int Row );  
 bool Read4bitRow(  ebmpBYTE* Buffer, int BufferSize, int Row );  
 bool Read1bitRow(  ebmpBYTE* Buffer, int BufferSize, int Row );
   
 bool Write32bitRow( ebmpBYTE* Buffer, int BufferSize, int Row );   
 bool Write24bitRow( ebmpBYTE* Buffer, int BufferSize, int Row );   
 bool Write8bitRow(  ebmpBYTE* Buffer, int BufferSize, int Row );  
 bool Write4bitRow(  ebmpBYTE* Buffer, int BufferSize, int Row );  
 bool Write1bitRow(  ebmpBYTE* Buffer, int BufferSize, int Row );
 
 ebmpBYTE FindClosestColor( RGBApixel& input );

 public: 

 int TellBitDepth( void );
 int TellWidth( void );
 int TellHeight( void );
 int TellNumberOfColors( void );
 void SetDPI( int HorizontalDPI, int VerticalDPI );
 int TellVerticalDPI( void );
 int TellHorizontalDPI( void );
  
 BMP();
 BMP( BMP& Input );
 ~BMP();
 RGBApixel* operator()(int i,int j);
 
 RGBApixel GetPixel( int i, int j ) const;
 bool SetPixel( int i, int j, RGBApixel NewPixel );
 
 bool CreateStandardColorTable( void );
 
 bool SetSize( int NewWidth, int NewHeight );
 bool SetBitDepth( int NewDepth );
 bool WriteToFile( const char* FileName );
 bool ReadFromFile( const char* FileName );
 
 RGBApixel GetColor( int ColorNumber );
 bool SetColor( int ColorNumber, RGBApixel NewColor ); 
};

#endif
