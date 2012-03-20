///////////////////////////////////////////////////
//
//   ChMocap.cpp
//
// ------------------------------------------------
// 	 Copyright:Alessandro Tasora / DeltaKnowledge
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////



#include "ChMocap.h"
#include "core/ChFile.h"

namespace chrono
{



ChMocap::ChMocap()
{
	vstream = NULL;
	samples = 0;
	markers = 0;
	Hertz = 0;
	ioscale = 1;
	undersample = 1;
	format = MOCF_ELITE;
}

ChMocap::~ChMocap()
{
	Reset();
}

int ChMocap::Alloc_buffers(int msamples, int mmarkers)
{
	vstream = (Vector**) calloc (mmarkers, sizeof(Vector*));
	if (!vstream) return 0;

	for (int i=0; i<mmarkers; i++)
	{
		*(vstream+i) = (Vector*) calloc (msamples, sizeof(Vector));
		if (!(*(vstream+i))) return 0;
	}

	markers = mmarkers;
	samples = msamples;

	return TRUE; // ok, success. Buffers allocated
}

void ChMocap::Reset()
{
	for (int i = 0; i<markers; i++)
	{
		if (*(vstream + i))
			free (*(vstream + i));
	}

	if (vstream)
		free (vstream);

	vstream = NULL;
}

Vector ChMocap::Get_mpos (int mark, int samp)
{
	if ((mark < markers)&&(samp < samples))
	{
		return *(*(vstream+mark) + samp);
	}
	else
		return (VNULL);
}

void ChMocap::Set_mpos (int mark, int samp, Vector vect)
{
	if ((mark < markers)&&(samp < samples))
	{
		Vector* vp = *(vstream+mark);
		*(vp + samp) = vect;
	}
}


////////////////////////////////////
// DATA PROCESSING

void ChMocap::Lin_interpolate (int from, int to,
				 Vector va, Vector vb, int nmark)
{
	Vector vinterp;
	double weight;

	for (int samp = from+1; samp < to; samp++)
	{
		weight = ((double)(samp - from))/((double)(to - from));
		vinterp = Vadd (Vmul(va,(1-weight)), Vmul(vb,weight));
		Set_mpos (nmark, samp, vinterp);
	}
}

void ChMocap::Rescale()
{
	Vector nval;
	for (int nmark = 0; nmark < markers; nmark++)
	{
		for (int samp = 0; samp < samples; samp++)
		{
			nval = Get_mpos(nmark, samp);
			nval = Vmul (nval, ioscale);
			Set_mpos(nmark, samp, nval);
		}
	}
}

void ChMocap::Gaps_fill (int nmark)
{
	int gap;
	int ifrom, ito;
	Vector va, vb, vcurr;

	ifrom = -1;
	va = VNULL;
	vb = VNULL;
	gap = FALSE;

	for (int samp = 0; samp < samples; samp++)
	{
		vcurr = Get_mpos (nmark, samp);
		if ((vcurr.x == VGAP_MOCAP) ||
		    (vcurr.y == VGAP_MOCAP) ||
			(vcurr.z == VGAP_MOCAP))
		{
			gap = TRUE;
		}
		else
		{
			if (gap == FALSE)
			{
				va = vcurr;
				ifrom = samp;
			}
			if (gap == TRUE)	// a gap has just ended: fill it!
			{
				gap = FALSE;
				vb = vcurr; ito= samp;
				if (ifrom ==-1) va = vb; // if gap from the beginning
				// Now interpolate the missing points!
				Lin_interpolate (ifrom, ito, va, vb, nmark);
			}
		}
	}

	if (gap ==TRUE)	// open end gap
	{
		vb = va; ito= samples;
		Lin_interpolate (ifrom, ito, va, vb, nmark);
	}

}

void ChMocap::Gaps_fill_all()
{
	for (int nmark = 0; nmark < markers; nmark++)
	{
		Gaps_fill(nmark);
	}
}

////////////////////////////////////
// MULTIFORMAT FILE I/O PROCEDURES


/////////////MASTER DISPATChER

int ChMocap::Parse(char* mfile)
{
	switch (format)
	{
	case MOCF_ELITE:
		return Parse_Elite(mfile);
	case MOCF_AOA:
		return Parse_AOA(mfile);
	case MOCF_R3CURVE:
		//*** TO DO ***
		break;
	case MOCF_CHSTREAM:
		//*** TO DO ***
		break;
	case MOCF_ANNO:
		//*** TO DO ***
		break;
	}

	return 0;
}

int ChMocap::Write(char* mfile)
{
	switch (format)
	{
	case MOCF_ELITE:
		return Write_Elite(mfile);
	case MOCF_AOA:
		return Write_AOA(mfile);
	case MOCF_R3CURVE:
		//*** TO DO ***
		break;
	case MOCF_CHSTREAM:
		//*** TO DO ***
		break;
	case MOCF_ANNO:
		//*** TO DO ***
		break;
	}

	return 0;
}


///////////// AOA Adaptive Optics format

int ChMocap::Parse_AOA(char* mfile)
{
	ChFile* streamMocap;
	char buffer[100];
	int proceed = TRUE;
	int frames, frame, mmark;
	Vector mvect;
/*
	try 
	{
		ChStreamInAsciiFile stream(pathbuff);
		
		while(true)
		{
			try { stream >> buffer; } catch (ChException merr) 
				{ break; }

			if (!strcmp(buffer, "frames"))
			{
				try { stream >> buffer; } catch (ChException merr) 
					{ break; }	// ## jump "="
				try { stream >> frames; } catch (ChException merr) 
					{ break; }	// ## parse nframes (samples)
				proceed = EOF;
			}
		}

	}			
	catch (ChException merr) 
	{
		GetLog() - ChLog::ChERROR << "Cannot load file " << pathbuff << ", " << merr.what() <<"\n";
	}
*/
	
	// open the file
	streamMocap  = new ChFile( mfile, (char*)"r", ChFile::CHFILE_NORMAL );
	if ( streamMocap->GetHandler() == NULL ) {delete streamMocap; return 1;}

	// parse the header
	proceed = 1;
	while (proceed != EOF)
	{
		proceed = streamMocap->ParseNext(buffer);
		if (!strcmp(buffer, "frames"))
		{
			proceed = streamMocap->ParseNext(buffer);	// ## jump "="
			proceed = streamMocap->ParseNext(&frames);	// ## parse nframes (samples)
			proceed = EOF;
		}
	}

	proceed = 1;
	while (proceed != EOF)
	{
		proceed = streamMocap->ParseNext(buffer);
		if (!strcmp(buffer, "markers"))
		{
			proceed = streamMocap->ParseNext(buffer);	// ## jump "="
			proceed = streamMocap->ParseNext(&markers);	// ## parse nframes
			proceed = EOF;		// >>>>>>>>>> MARKERS
		}
	}

	proceed = 1;
	while (proceed != EOF)
	{
		proceed = streamMocap->ParseNext(buffer);
		if (!strcmp(buffer, "Hz"))
		{
			proceed = streamMocap->ParseNext(buffer);	// ## jump "="
			proceed = streamMocap->ParseNext(&Hertz);	// ## parse nframes
			proceed = EOF;
		}
	}

	// compute number of samples, if undersample
	double mfnumpoints;
	modf ((frames / undersample), &mfnumpoints);
	samples = (int)mfnumpoints;	// >>>>>>>>>> SAMPLES
	Hertz = Hertz/undersample;

	// Alloc data to contain the point streams
	if (!Alloc_buffers (samples, markers)) return 2;

	// parse all the XYZ coordinates of the markers
	int jumpcount = (undersample - 1);
	int npoint = 0;

	for (frame = 0; frame < frames; frame ++)
	{
		jumpcount ++;

		if (jumpcount == undersample)
		{
			for (mmark = 0; mmark < markers; mmark ++)
			{
				// ---- READ THE (xyz) VECTOR HERE
				proceed = streamMocap->ParseNext(&mvect);
				// ---- rescale
				mvect = Vmul(mvect, ioscale);
				// ---- store
				if (npoint < samples)
					Set_mpos (mmark, npoint, mvect);
			}
			npoint++;
			jumpcount = 0;
		}
		else
		{
			// skip unneeded frames
			for (mmark = 0; mmark < markers; mmark ++)
			{
				proceed = streamMocap->ParseNext(&mvect);
			}
		}
	}


	// close the file
	delete streamMocap;

	return 0; // ok, success;
}

int ChMocap::Write_AOA(char* mfile)
{
	ChFile* streamMocap;
	Vector mvect;

	streamMocap  = new ChFile( mfile, (char*)"w", ChFile::CHFILE_NORMAL );
	if ( streamMocap->GetHandler() == NULL ) {delete streamMocap; return 1;}

	// write header
	streamMocap->Write((char*)"frames = ");
	streamMocap->Write(samples);
	streamMocap->Write((char*)" markers = ");
	streamMocap->Write(markers);
	streamMocap->Write((char*)" Hz = ");
	streamMocap->Write(Hertz);
	streamMocap->CR();

	// write streams
	for (int nsamp = 0; nsamp< samples; nsamp++)
	{
		for (int nmark = 0; nmark <markers; nmark++)
		{
			mvect = Get_mpos(nmark,nsamp);
			mvect = Vmul(mvect, (1/ioscale));
			streamMocap->Write(mvect);
		}
	}

	delete streamMocap;

	return 0; // ok, success;
}


/////////// ELITE format

int ChMocap::Parse_Elite(char* mfile)
{
	ChFile* streamMocap;
	char buffer[50];
	int proceed = TRUE;
	int frames = 0;
	int frame, nmark;
	Vector mvect;

	// open the file
	streamMocap  = new ChFile( mfile, (char*)"r", ChFile::CHFILE_NORMAL );
	if ( streamMocap->GetHandler() == NULL ) {delete streamMocap; return 1;}

	// parse the header
		// (no header)
	Hertz = 50;		// that's by default;

	// get the number of frames
	proceed = 1;
	frames =0;
	while (proceed != EOF)
	{
		proceed = streamMocap->ParseNext(buffer); // x
		proceed = streamMocap->ParseNext(buffer); // y
		proceed = streamMocap->ParseNext(buffer); // z
		if (proceed != EOF) frames ++;
		if (frames > 5000) proceed=EOF;
	}
	markers = 1;					// >>>>>>> MARKERS
	frames = frames;

		// reopen file
	delete streamMocap;
	streamMocap  = new ChFile( mfile, (char*)"r", ChFile::CHFILE_NORMAL );

	// compute number of samples, if undersample
	double mfnumpoints;
	modf ((frames / undersample), &mfnumpoints);
	samples = (int)mfnumpoints;		// >>>>>>>>SAMPLES
	Hertz = Hertz/undersample;

	// Alloc data to contain the point streams
	if (!Alloc_buffers (samples, markers)) return 2;

	// parse all the XYZ coordinates of the markers
	int jumpcount = (undersample - 1);
	int npoint = 0;

	nmark = 0;	// set only the 1st mark

	for (frame = 0; frame < frames; frame ++)
	{

		jumpcount ++;

		if (jumpcount == undersample)
		{
			// ---- READ THE (xyz) VECTOR HERE
			proceed = streamMocap->ParseNext(buffer);
			if ((!strcmp(buffer, "********"))||
				(!strcmp(buffer, "-9999"))||
				(!strcmp(buffer, "-9999.00"))) mvect.x = VGAP_MOCAP;
			else mvect.x = atof (buffer);
			proceed = streamMocap->ParseNext(buffer);
			if ((!strcmp(buffer, "********"))||
				(!strcmp(buffer, "-9999"))||
				(!strcmp(buffer, "-9999.00"))) mvect.y = VGAP_MOCAP;
			else mvect.y = atof (buffer);
			proceed = streamMocap->ParseNext(buffer);
			if ((!strcmp(buffer, "********"))||
				(!strcmp(buffer, "-9999"))||
				(!strcmp(buffer, "-9999.00"))) mvect.z = VGAP_MOCAP;
			else mvect.z = atof (buffer);

			// ---- store
			if (npoint < samples)
				Set_mpos (nmark, npoint, mvect);

			npoint++;
			jumpcount = 0;
		}
		else
		{
			// skip unneeded frames
			proceed = streamMocap->ParseNext(buffer);
			proceed = streamMocap->ParseNext(buffer);
			proceed = streamMocap->ParseNext(buffer);
		}
	}

	// Fill gaps caused by missing marker data
	Gaps_fill_all();

	// Rescale with correct measures
	Rescale();

	// close the file
	delete streamMocap;

	return 0; // ok, success;
}

int ChMocap::Write_Elite(char* mfile)
{
	ChFile* streamMocap;
	Vector mvect;

	streamMocap  = new ChFile( mfile, (char*)"w", ChFile::CHFILE_NORMAL );
	if ( streamMocap->GetHandler() == NULL ) {delete streamMocap; return 1;}

	for (int nsamp = 0; nsamp< samples; nsamp++)
	{
		mvect = Get_mpos(0,nsamp);
		mvect = Vmul(mvect, (1/ioscale));
		streamMocap->Write(mvect);
		streamMocap->CR();
	}

	delete streamMocap;

	return 0; // ok, success;
}


} // END_OF_NAMESPACE____

