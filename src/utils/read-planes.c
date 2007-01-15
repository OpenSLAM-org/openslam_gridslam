#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <sys/types.h>
#include <sys/time.h>
#include <signal.h>
#include <unistd.h>
#include <math.h>
#include <string.h>
#include <ctype.h>
#include <values.h>

#include <navigation/utils.h>

int
read_beam_planes( char *filename, PLANE_BEAM_DATA *planes, int verbose )
{

  char   str1[80], str2[80], str3[80], str4[80], str5[80];
    
  int i, FEnd;

  char command[256];

  FILE * iop;

  int numPts, numPlanes = 0, linectr = 0, planectr = 0;

  fprintf( stderr, "reading file %s ...\n", filename );
  if ((iop = fopen( filename, "r")) == 0){
    fprintf(stderr, " WARNING no rec file %s\n", filename );
    return(-1);
  }

  FEnd = 0;
  do{
    linectr++;
    if (fscanf(iop, "%s", command) == EOF)
      FEnd=1;
    else{
      if (!strcmp( command, "PLANE-REC")) {
	planectr++;
      }
      fgets(command,sizeof(command),iop);
    }
  } while (!FEnd);
  
  if (verbose) {
    fprintf( stderr, "*******************************\n" );
    fprintf( stderr, "num lines      = %d\n", linectr );
    fprintf( stderr, "num planes     = %d\n", planectr );
    fprintf( stderr, "*******************************\n" );
  }
  
  rewind(iop);

  planes->plane =
    (PLANE_BEAMREF *) malloc( planectr * sizeof(PLANE_BEAMREF) ); 

  numPlanes  = 0;

  FEnd = 0;
  do{
    if (fscanf(iop, "%s", command) == EOF)
      FEnd=1;
    else{
      if (!strcmp( command, "PLANE-REC")) {
	if (fscanf(iop, "%s %s %s %s %s%%",
		   str1, str2, str3, str4, str5 ) == EOF) {
	  FEnd=1;
	} else {
	  
	  planes->plane[numPlanes].param.a = atof( str1 );
	  planes->plane[numPlanes].param.b = atof( str2 );
	  planes->plane[numPlanes].param.c = atof( str3 );
	  planes->plane[numPlanes].param.d = atof( str4 );
	  numPts = atoi( str5 );
	  planes->plane[numPlanes].planebeam =
	    (BEAMREF *) malloc( numPts * sizeof(BEAMREF) );

	  for (i=0;i<numPts;i++) {
	    if (fscanf( iop, "%s", str1 ) == EOF) {
	      numPts = 0;
	      break;
	    } else {
	      planes->plane[numPlanes].planebeam[i].scan = 
		atoi( strtok( str1, "/") );
	      planes->plane[numPlanes].planebeam[i].beam =
		atoi( strtok( NULL, "/") );
	    }
	  }
	  planes->plane[numPlanes].numpoints = numPts;

	  numPlanes++;
	}
      } else {
	if (!(command[0]=='#')){
	  fprintf( stderr, "ERROR: unknown keyword %s\n", command );
	  fclose(iop);
	  return(-1);
	} else {
	  fgets(command,sizeof(command),iop);
	}
      }
    }
  } while (!FEnd);
  
  planes->numplanes = numPlanes;
  fprintf( stderr, "read %d planes\n", numPlanes );

  fclose(iop);

  return(0);
}


