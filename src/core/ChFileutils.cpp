//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2010 Alessandro Tasora
// Copyright (c) 2013 Project Chrono
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be 
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//

//////////////////////////////////////////////////
//
//   ChFileutils.cpp
//
// ------------------------------------------------
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////


#include <math.h>
#include <string.h>
#include <stdio.h>
#include "core/ChFileutils.h"

#if defined(_WIN32) || defined(__WIN32__) || defined(__CYGWIN__)
	#include <direct.h>
#else
	#include <sys/types.h>
	#include <sys/stat.h>
#endif


namespace chrono
{


void ChFileutils::Change_file_ext (char fid1[], char fid[], char ext[], int force)
{
  int i,l;
  char *p,*q;

  strcpy (fid1, fid);
  l=strlen(fid1);
  p=fid1;
  for (i=0;i<l;i++)
    if (fid1[i]=='/') p=fid1+i;

  if (!force) {
    q=strchr(p,'.');
    if (q && (q!=fid1+strlen(fid1)-1)) return;
  }
  if (!strchr(p,'.')) strcat (fid1,".");
  q=strchr(p,'.');
  if (strlen(ext)>0) q++;
  *q = 0;
  strcat(fid1,ext);

}

/* ----- cut off extension on a file identifier ----- */
void ChFileutils::Cut_file_ext (char fid[])
{
  int i,l;

  l=strlen(fid);
  for (i=0;i<l;i++)
    if (fid[i]=='.') fid[i]='\0';
}

/* ----- getext: get extension on a file identifier ----- */
void ChFileutils::Get_file_ext (char fid[], char ext[])
{
  int i,l,k;

  l=strlen(fid);
  k=l-1;
  for (i=0;i<l;i++)
    if (fid[i]=='.') k=i;

  for (i=k+1;i<l;i++)
    ext[i-k-1]=fid[i];
  ext[l-k-1]='\0';
}


/* ----- get_file_size ------- */

int ChFileutils::Get_file_size (char fname[])
{
  int m,i;
  FILE *fp;

  if ((fp = fopen (fname,"r")) == NULL) {
    //printf ("Cannot open file to determine size: %s", fname);
    return -1;
  }

  m=0;
  i=getc(fp);
  while (i != EOF) {
    m++;
    i=getc(fp);
  }
  fclose (fp);
  return m;
}


bool ChFileutils::MakeDirectory (char dirname[])
{
	#if defined(_WIN32) || defined(__WIN32__) || defined(__CYGWIN__)
		int res = _mkdir(dirname);   // Windows
	#else 
		int res = mkdir(dirname, 0777); // Linux
	#endif
	
		if (res ==0)
			return true;
		else
			return false;
}


} // END_OF_NAMESPACE____
