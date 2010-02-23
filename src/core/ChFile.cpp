///////////////////////////////////////////////////
//
//   ChFile.cpp 
//
// 
//   Class for file input-output of Chrono objects.
//   Defines some functions for ASCII parsing of the
//   textual file format of Chrono.
// 
// ------------------------------------------------
// 	 Copyright:Alessandro Tasora / DeltaKnowledge
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////
 
//******OBSOLETE*****!!!!  see ChStream.h instead !!
   

#include "core/ChFile.h"
#include "physics/ChGlobal.h"


namespace chrono 
{



//////////////////////////////////////
//////////////////////////////////////

// BASE CLASS FOR HANDLING FILES

int ChFile::Reset(char m_name[], char m_mode[], eChMode my_ch_mode)
{
	int mok = TRUE;

	if (handler) 
	{ 
		fclose (handler); 
		handler = NULL;
	};

	ch_mode = my_ch_mode;
	strcpy (name, m_name);
	strcpy (mode, m_mode);
	strcpy (numformat, "%g"); 

	switch (ch_mode)
	{
	case CHFILE_NORMAL:
		handler = fopen( m_name, m_mode);
		if (!handler) mok = FALSE;
		break;
	case CHFILE_SAFEWRITE:
		handler = fopen( m_name, m_mode);
		if (!handler) mok = FALSE;
		break;
	case CHFILE_NOWRITE:
		handler = NULL;
		break;
	case CHFILE_OPENLATER:
		handler = NULL;
		break;
	}
		
	return mok;
}

ChFile::ChFile(char m_name[], char m_mode[], eChMode my_ch_mode)
{
	handler = NULL;

	ChFile::Reset(m_name, m_mode, my_ch_mode);
}



ChFile::~ChFile()
{
	if (handler) 
	{ 
		fclose (handler); 
	};
}

///////////

int ChFile::CanWrite()
{
	switch (ch_mode)
	{
	case CHFILE_NORMAL:
		if (handler)  return TRUE;

	case CHFILE_SAFEWRITE:
		if (handler) return TRUE;

	case CHFILE_NOWRITE:
		return FALSE;

	case CHFILE_OPENLATER:
		handler = fopen( name, mode);
		if (handler) return TRUE;

	case CHFILE_REAL3D:
		return TRUE;	// the file is handled by real3d

	default:
		return FALSE;
	}
}

void ChFile::SetNumFormat (char* mf)
{
	if (strlen(mf) < 10) 
		strcpy(numformat,mf);
}


/////////// READ FUNCTIONS, with automatic formatting and comment skipping 

int ChFile::ParseNext (int* m_int)
{
	while (fscanf  (handler, "%s", &buffer) != EOF) {
		if (memcmp(buffer, "#",1) == 0)  
		{
			fscanf (handler, "%*[^\n]");	// skip comments 'till EOF
		}
		else
		{
			*m_int= atoi (buffer);
			return 1;					// parse command
		}
	}
	return EOF;    // Eof
}
	

int ChFile::ParseNext (double* m_double) 
{
	while (fscanf  (handler, "%s", &buffer) != EOF) {
		if (memcmp(buffer, "#",1) == 0) 
		{
			fscanf (handler, "%*[^\n]");	// skip comments 'till EOF
		}
		else
		{
			*m_double= atof (buffer); 	// parse command
			return 1;
		}
	}
	return EOF;  // Eof
}
	
int ChFile::ParseNext (char m_string[])
{
	int success = -1;

	while (fscanf  (handler, "%s", &buffer) != EOF) {
		if (memcmp(buffer, "#",1) == 0) 
		{
			fscanf (handler, "%*[^\n]");	// skip comments 'till EOF
		}
		else
		{
			strcpy (m_string, buffer);		// parse command
			return 1;
		}
	}
	return EOF;  // Eof
}
	
int ChFile::ParseNext (Vector* m_vector) 
{
	double x,y,z;

	if (ParseNext(&x) != -1) {
		if (ParseNext(&y) != -1) {
			if (ParseNext(&z) != -1) {
				m_vector->x=x;
				m_vector->y=y;
				m_vector->z=z;
				return 1; }}}
	return EOF;
}
	
int ChFile::ParseNext (Quaternion* m_quaternion)
{
	double e0,e1,e2,e3;

	if (ParseNext(&e0) != -1) {
		if (ParseNext(&e1) != -1) {
			if (ParseNext(&e2) != -1) {
				if (ParseNext(&e3) != -1) {
				m_quaternion->e0=e0;
				m_quaternion->e1=e1;
				m_quaternion->e2=e2;
				m_quaternion->e3=e3;
				return 1; }}}}
	return EOF;
}
	
int ChFile::ParseNext (ChMatrix33<>* m_matrix) 
{
	double el;				// only for 3x3 matrices!!

	if (ParseNext(&el) == -1) return -1;
	m_matrix->SetElement(0,0,el);
	if (ParseNext(&el) == -1) return -1;
	m_matrix->SetElement(0,1,el);
	if (ParseNext(&el) == -1) return -1;
	m_matrix->SetElement(0,2,el);
	if (ParseNext(&el) == -1) return -1;
	m_matrix->SetElement(1,0,el);
	if (ParseNext(&el) == -1) return -1;
	m_matrix->SetElement(1,1,el);
	if (ParseNext(&el) == -1) return -1;
	m_matrix->SetElement(1,2,el);
	if (ParseNext(&el) == -1) return -1;
	m_matrix->SetElement(2,0,el);
	if (ParseNext(&el) == -1) return -1;
	m_matrix->SetElement(2,1,el);
	if (ParseNext(&el) == -1) return -1;
	m_matrix->SetElement(2,2,el);

	return 1;  // all ok
}

///////////////////////////////////
/////////// ASCII-WRITE FUNCTIONS

int ChFile::Write (int m_int) 
{
	if (CanWrite()) 
		return fprintf (handler, "%d", m_int);
	return 0;
} 

int ChFile::Write (double m_double) 
{
	if (CanWrite()) 
		return fprintf(handler, numformat, m_double);
	return 0;
}
 
int ChFile::WriteAngle (double m_double) 
{
	double mangle;
		mangle = m_double;

	if (CanWrite()) {  
			return fprintf (handler, numformat, mangle);
	}
	return 0;
}

int ChFile::Write (char* m_string) 
{
	if (CanWrite()) 
		return fprintf (handler, m_string);
	return 0;
}

int ChFile::Write (Vector m_vector)
{
	if (CanWrite())
	{
		Write (m_vector.x); Write ("  ");
		Write (m_vector.y); Write ("  ");
		Write (m_vector.z); Write ("  ");
		return 1;
	}
	else return 0;
}

int ChFile::Write (Quaternion m_quaternion) 
{
	if (CanWrite()) 
	{
		Write (m_quaternion.e0); Write ("  ");
		Write (m_quaternion.e1); Write ("  ");
		Write (m_quaternion.e2); Write ("  ");
		Write (m_quaternion.e3); Write ("  ");
		return 1;
	}
	else return 0;
}

int ChFile::Write (Coordsys m_coordsys)
{
	if (CanWrite()) 
	{
		Write (m_coordsys.pos); Write ("  ");
		Write (m_coordsys.rot); Write ("  ");
		return 1;
	}
	else return 0;
}

// Print matrices of all dimensions:


int ChFile::Write (ChMatrix<>* m_matrix, int transpose) 
{
	int r, c; 
	double val;
 	r = m_matrix->GetRows();
	c = m_matrix->GetColumns();

	if (CanWrite()) 
	{
		if (transpose) {	// print transposed vectors to save rows space
			int temp= r;
			r=c;
			c=temp;
		}

		for (int mr = 0; mr < r; mr++)
		{
			for (int mc = 0; mc < c; mc++)
			{
				if (!transpose) val= m_matrix->GetElement(mr,mc);
				if (transpose)  val= m_matrix->GetElement(mc,mr);
				Write (val); Write ("  ");
			}
			CR();
		}
		return 1;
	}
	else return 0;
}

int ChFile::Write (ChMatrix<>* m_matrix) 
{
	return Write (m_matrix, 0);
}

int ChFile::Write (ChSparseMatrix* m_matrix)
{
	int r, c; 
	double val;
 	r = m_matrix->GetRows();
	c = m_matrix->GetColumns();

	if (CanWrite())
	{
		for (int mr = 0; mr < r; mr++)
		{
			for (int mc = 0; mc < c; mc++)
			{
				val= m_matrix->GetElement(mr,mc);
				Write (val); Write ("  ");
			}
			CR();
		}
		return 1;
	}
	else return 0;
}

void ChFile::Comment (char m_string[])
{
	if (CanWrite())
	{
		if (GLOBAL_Vars->WriteComments)
		{
			TAB(); TAB(); TAB();
			Write ("#");
			Write (m_string);
		}
		CR();
	}
}

void ChFile::WriteStepInfo (double time, double step)
{
	if (CanWrite())
	{
		CR();
		Write(" current time: "); Write(time); 
		Write("    time step: "); Write(step); 
		CR();
	}
}

int ChFile::Write (ChVar m_var)
{
	if (CanWrite()) 
	{
		switch (m_var.classtype)
		{
		case CHCLASS_INTEGER: 
			Write (*(int*)m_var.varp); break;
		case CHCLASS_FLOAT: 
			Write (*(double*)m_var.varp); break;
		case CHCLASS_VECTOR: 
			Write (*(Vector*)m_var.varp); break;
		case CHCLASS_STRINGP: 
			Write ((char*)m_var.varp); break;
		default:
			Write ("Ch_var of class type: "); Write (m_var.classtype);
		}
		return 1;
	}
	else return 0;
}


////////////////////////////////////
/////////// BINARY-WRITE  FUNCTIONS



			// integers  -2 bytes
int ChFile::BinWrite (int m_int) 
{
	if (CanWrite()) 
		return fwrite (&m_int, sizeof (int), 1, handler);
	return 0;
}
int ChFile::BinRead (int* am_int)
{
	return fread (am_int, sizeof (int), 1, handler);
}
			// long  -4 bytes
int ChFile::BinWrite (long m_long) 
{
	if (CanWrite()) 
		return fwrite (&m_long, sizeof (long), 1, handler);
	return 0;
}
int ChFile::BinRead (long* am_long)
{
	return fread (am_long, sizeof (long), 1, handler);
}
			// floats
int ChFile::BinWrite (double m_double) 
{
	if (CanWrite()) 
		return fwrite (&m_double, sizeof (double), 1, handler);
	return 0;
}
int ChFile::BinRead (double* am_double)
{
	return 
		fread (am_double, sizeof (double), 1, handler);
}
			// strings
int ChFile::BinWrite (char* m_str)
{
	if (CanWrite()) 
	{ 
		if (!BinWrite((int)strlen(m_str)+1)) return 0; // write string lenght
		if (!fwrite (m_str, strlen(m_str)+1, 1, handler)) return 0;
		return 1;
	}
	else return 0;
}
int ChFile::BinRead (char* m_buff)
{
	int msize = 0;
	if (!BinRead(&msize)) return 0;

	return fread (m_buff, msize, 1, handler);
}


int ChFile::BinWrite (Vector m_vector)
{
	if (CanWrite())
	{
		BinWrite (m_vector.x);
		BinWrite (m_vector.y);
		BinWrite (m_vector.z);
		return 1;
	}
	else return 0;
}
int ChFile::BinRead (Vector* am_vector)
{
	double x,y,z;
	if (BinRead(&x)) {
		if (BinRead(&y)) {
			if (BinRead(&z)) {
				am_vector->x=x;
				am_vector->y=y;
				am_vector->z=z;
				return 1; }}}
	return 0; 	// if fails
}

int ChFile::BinWrite (Quaternion m_quaternion) 
{
	if (CanWrite()) 
	{
		BinWrite (m_quaternion.e0);
		BinWrite (m_quaternion.e1);
		BinWrite (m_quaternion.e2);
		BinWrite (m_quaternion.e3);
		return 1;
	}
	else return 0;
}
int ChFile::BinRead (Quaternion* am_quat)
{
	double e0,e1,e2,e3;
	if (BinRead(&e0)) {
		if (BinRead(&e1)) {
			if (BinRead(&e2)) {
				if (BinRead(&e3)) {
					am_quat->e0=e0;
					am_quat->e1=e1;
					am_quat->e2=e2;
					am_quat->e3=e3;
				return 1; }}}}
	return 0; 	// if fails
}


int ChFile::BinWrite (Coordsys m_coordsys)
{
	if (CanWrite()) 
	{
		BinWrite (m_coordsys.pos);
		BinWrite (m_coordsys.rot);
		return 1;
	}
	else return 0;
}
int ChFile::BinRead (Coordsys* am_coordsys)
{
	if (BinRead(&am_coordsys->pos)) {
		if (BinRead(&am_coordsys->rot)) {
			return 1; }}
	return 0;	// if fails
}


int ChFile::BinWrite (ChMatrix<>* m_matrix)
{
	int tot_elements;
	if (CanWrite())
	{
		BinWrite (m_matrix->GetRows());
		BinWrite (m_matrix->GetColumns());
		tot_elements = m_matrix->GetRows() * m_matrix->GetColumns();
		for (int i=0; i< tot_elements; i++)
		{
			BinWrite (m_matrix->GetElementN(i));
		}
		return 1;
	}
	else return 0;

}

int ChFile::BinRead (ChMatrixDynamic<>** m_matrix) // matrix auto instanced
{
	int tot_elements;
	int r, c;
	double elem;
	ChMatrixDynamic<>* mymatr;

	if (BinRead(&r)) {
		if (BinRead(&c)) {
				mymatr = new ChMatrixDynamic<>(r,c);
				tot_elements = r*c;
				for (int i=0; i< tot_elements; i++)
				{
					if (BinRead(&elem))
					{
						mymatr->SetElementN(i,elem);
					}
					else return 0;
				}
				*m_matrix = mymatr;
				return 1;
	}}
	return 0;
}









} // END_OF_NAMESPACE____

////// end
