//////////////////////////////////////////////////
//  
//   ChFileutils.h
//
// ------------------------------------------------
// 	 Copyright:Alessandro Tasora / DeltaKnowledge
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////


#include <math.h>
#include <string.h>
#include <stdio.h>
#include "core/ChFileutils.h"

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


} // END_OF_NAMESPACE____