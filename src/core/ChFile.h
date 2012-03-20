#ifndef CHFILE_H
#define CHFILE_H

//////////////////////////////////////////////////
//  
//   ChFile.h   ***OBSOLETE**** , use ChStream.h !!
//
//   Class for file input-output of Chrono objects.
//   Defines some functions for ASCII parsing of the
//   textual file format of Chrono.
//   Defines binary in/out
//
//   HEADER file for CHRONO,
//	 Multibody dynamics engine
//
// ------------------------------------------------
// 	 Copyright:Alessandro Tasora / DeltaKnowledge
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////


#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <string.h>
#include <math.h>

#include "core/ChMath.h"
#include "physics/ChFormule.h"
#include "core/ChSpmatrix.h"



namespace chrono 
{



//  ***OBSOLETE**** use ChStream.h instead !!
//
// This is a base class for wrapping file handlers. You can
// use object of this class to write/read from disk.
// There are some inherited classes which specialize for some 
// particular type of files, like the postscript output and such.
//
//	***OBSOLETE**** use ChStream.h instead !!

	
class ChFile {

public:
					/// Modes for chrono files (the ch-modes)
	enum eChMode {
		CHFILE_NORMAL = 0,	///< writes/reads as a normal file
		CHFILE_NOWRITE,		///< force do not write even if in w or wb (write mode)
		CHFILE_SAFEWRITE,	///< safe writing (obsolete)
		CHFILE_OPENLATER,	///< creates the file only as soon as first writing is attempted
		CHFILE_REAL3D,		///< the file is a Realsoft3D file wrapper
	};	

protected:
				//
	  			// DATA
				//

	FILE* handler;			// this is the standard file handler
	char name[180];			// the filename
	char mode[5];			// the mode (w,r, wb, rb, etc.)

	eChMode ch_mode;			
	int cursor;				
	char buffer[60];
	char numformat[10];
			

public:

				//
	  			// CONSTRUCTORS  
				//
					/// Create a file object but does nothing. Lather you may use Reset() to do something..
	ChFile() {handler=NULL; Reset((char*)"",(char*)"",CHFILE_OPENLATER);};

					/// Create the file object, given filename on disk, mode (r,w,rb,wb, etc) and 
					/// chrono file mode (CHFILE_NORMAL, CHFILE_NOWRITE, etc.)
	ChFile(char m_name[], char m_mode[], eChMode my_ch_mode = CHFILE_NORMAL);	


	~ChFile();

				//
	  			// FUNCTIONS
				//

					/// Close (if previously open) and reopen handler with given settings. 
					/// \return true if success.
	int Reset(char m_name[], char m_mode[], eChMode my_mode);

	FILE* GetHandler () {return handler;};
	char* GetFilename () {return name;}; 

	eChMode GetChMode () {return ch_mode;}
	void	SetChMode (eChMode my_mod) {ch_mode = my_mod;}

					/// Gets the string for floating point -> string  formatting (ex "%f", see sprintf() syntax);
	char* GetNumFormat () {return numformat;}
					/// Sets the string for floating point -> string  formatting (ex "%f", see sprintf() syntax);
	void SetNumFormat (char* mf);

					/// Return TRUE if file can be written, (mostly for internal use)
	int CanWrite();			


	//----- ASCII READ-WRITE

	int ParseNext (int* m_int);				// advanced reading (parsing)
	int ParseNext (double* m_double);	
	int ParseNext (char m_string[]);
	int ParseNext (Vector* m_vector);
	int ParseNext (Quaternion* m_quaternion);
	int ParseNext (ChMatrix33<>* m_matrix);		// only 3x3 matrices

	int Write (int m_int);					// advanced writing
	int Write (double m_double);

	int Write (char* m_string);
	int Write (Vector m_vector);
	int Write (Quaternion m_quaternion);
	int Write (Coordsys m_coordsys);
	int Write (ChVar m_var);
	int Write (ChMatrix<>* m_matrix, int transpose);
	int Write (ChMatrix<>* m_matrix);
	int Write (ChSparseMatrix* m_matrix);
	int WriteAngle (double m_double);  // auto converts into degrees if set in CHGLOBALS() 
	
					/// Output carriage return in Ascii file.
	void CR() { if (CanWrite()) fprintf (handler, "\n");};	
					/// Output tab in Ascii file.
	void TAB() { if (CanWrite()) fprintf (handler, "\t");};	
					/// Output comment with preceding # char in Ascii file
	void Comment (char m_string[]);	
					/// Output a separation bar in Ascii file
	void Bar() { if (CanWrite()) fprintf (handler, "#------------------------------------------------------------------ \n");};
					/// Output a time and step //***TO REMOVE***
	void WriteStepInfo (double time, double step);


	//-----  BINARY READ-WRITE
					
					/// Write an integer number in the binary file
	int BinWrite (int m_int);
					/// Read an integer number from the binary file
	int BinRead (int* am_int);

					/// Write a 'long' number in the binary file
	int BinWrite (long m_long);
					/// Read  a 'long' number from the binary file
	int BinRead (long* am_long);

					/// Write a double precision floating point number to the binary file
	int BinWrite (double m_double); 
					/// Read a double precision floating point number from the binary file
	int BinRead (double* am_double);

					/// Write a string to the binary file (string must be null-terminated)
	int BinWrite (char* m_str);
					/// Read a string from the binary file (buffer must be already allocated with anough space).
	int BinRead (char* m_buf);			


	int BinWrite (Vector m_vector);
	int BinRead (Vector* am_vector);

	int BinWrite (Quaternion m_quaternion);
	int BinRead (Quaternion* am_quat);

	int BinWrite (Coordsys m_coordsys);
	int BinRead (Coordsys* am_coordsys);

	int BinWrite (ChMatrix<>* m_matrix);
	int BinRead (ChMatrixDynamic<>** m_matrix);	/// Matrix will be automatically intantiated.	
};







} // END_OF_NAMESPACE____


#endif
