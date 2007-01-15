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

#include "defines.h"


void
init_spf_point_data( SPF_POINTS_DATA *spf )
{
  spf->numpoints    = 0;
  spf->point        = NULL;
  spf->numpolygons  = 0;
  spf->polygon      = NULL;
  spf->numplanes    = 0;
  spf->plane        = NULL;
}

int
insert_spf_point_data( char *filename, SPF_POINTS_DATA *spf  )
{
  int i, check;
  int FEnd       = 0;
  int PEnd       = 0;
  int lineCtr    = 0;
  int pointCtr   = 0;
  int polygonCtr = 0;
  int planeCtr   = 0;

  int numE, numPt;
  int polygonE[MAX_POLYGON_SIZE];

  char line[MAX_LINE_LENGTH];
  char command[256];

  char strF1[256];
  char strF2[256];
  char strF3[256];
  char strF4[256];
  char strF5[256];
  
  FILE *iop;
  
  if ((iop = fopen( filename, "r")) == 0){
    fprintf( stderr, "ERROR: could not open spf-file file %s\n",
	     filename);
    return(FALSE);
  } else {
    fprintf( stderr, "INFO: open spf-file file %s\n",
	     filename);
  }
  
  do{
    if (fgets(line,MAX_LINE_LENGTH,iop) == NULL) {
      FEnd=1;
    } else {
      lineCtr++;
      sscanf(line, "%s", command);
      if (!strcmp( command, "POINT3D")) {
	pointCtr++;
      } else if (!strcmp( command, "POLYGON")) {
	polygonCtr++;
      } else if (!strcmp( command, "PLANE")) {
	planeCtr++;
      }
    }
  } while (!FEnd);

  if (pointCtr>0)
    fprintf( stderr, "INFO: insert %d points\n", pointCtr );
  if (polygonCtr>0)
    fprintf( stderr, "INFO: insert %d polygons\n", polygonCtr );
  if (planeCtr>0)
    fprintf( stderr, "INFO: insert %d planes\n", planeCtr );
  
  
  rewind(iop);

  if ( pointCtr>0 ) {
    if ( spf->numpoints == 0 ) {
      spf->point = (POINT3 *) malloc( pointCtr * sizeof(POINT3) );
    } else {
      spf->point = (POINT3 *) 
	realloc( spf->point, ( pointCtr + spf->numpoints ) * sizeof(POINT3) );
    }
  }
  
  if ( polygonCtr>0 ) {
    if ( spf->numpolygons == 0 ) {
      spf->polygon = (POLYGON_REF *)
	malloc(polygonCtr*sizeof(POLYGON_REF));
    } else {
      spf->polygon = (POLYGON_REF *)
	realloc( spf->polygon,
		 ( polygonCtr + spf->numpolygons ) * sizeof(POLYGON_REF) );
    }
  }

  if ( planeCtr>0 ) {
    if ( spf->numplanes == 0 ) {
      spf->plane = (PLANE_POINTREF *) 
	malloc( planeCtr * sizeof(PLANE_POINTREF) );
    } else {
      spf->plane = (PLANE_POINTREF *) 
	realloc( spf->plane, 
		 ( planeCtr + spf->numplanes ) *  sizeof(PLANE_POINTREF) );
    }
  }
  
  FEnd = 0;
  
  do{
    
    if (fscanf(iop, "%s", command) == EOF)
      FEnd=1;
    else{
      if (!strcmp( command, "POINT3D") ){
	if (fscanf(iop, "%s %s %s", strF1, strF2, strF3 ) == EOF) {
	  FEnd=1;
	} else {
	  spf->point[spf->numpoints].x = atof( strF1 );
	  spf->point[spf->numpoints].y = atof( strF2 );
	  spf->point[spf->numpoints].z = atof( strF3 );
	  spf->numpoints++;
	}
      } else if (!strcmp( command, "POLYGON")) {
	numE = 0;
	PEnd = 0;
	do{
	  if (fscanf( iop, "%s", strF1 ) == EOF) {
	    FEnd=1;
	  } else {
	    polygonE[numE] = atoi( strF1 );
	    numE++;
	    if (polygonE[numE-1] == -1 || numE>=MAX_POLYGON_SIZE)
	      PEnd = 1;
	  }
	} while (!FEnd && !PEnd);
	numE--;
	check = TRUE;
	for (i=0;i<numE;i++){
	  if (polygonE[i]<0 || polygonE[i]>=spf->numpoints )
	    check=FALSE;
	}
	if (check && numE>0) {
	  spf->polygon[spf->numpolygons].facept =
	    (int *) malloc( numE * sizeof(int) );
	  for (i=0;i<numE;i++) {
	    spf->polygon[spf->numpolygons].facept[i] = polygonE[i];
	  }
	  spf->polygon[spf->numpolygons].numfaces = numE;
	  spf->numpolygons++;
	}
      } else if (!strcmp( command, "PLANE")) {
	if (fscanf( iop, "%s %s %s %s %s:",
		    strF1, strF2, strF3, strF4, strF5 ) == EOF) {
	  FEnd=1;
	} else {
	  spf->plane[spf->numplanes].param.a = atof( strF1 );
	  spf->plane[spf->numplanes].param.b = atof( strF2 );
	  spf->plane[spf->numplanes].param.c = atof( strF3 );
	  spf->plane[spf->numplanes].param.d = atof( strF4 );
	  numPt = atoi( strF5 );
	  spf->plane[spf->numplanes].planept =
	    (int *) malloc( numPt * sizeof(int) );
	  for (i=0;i<numPt;i++) {
	    if (fscanf( iop, "%s", strF1 ) == EOF) {
	      numPt = 0;
	      break;
	    } else {
	      spf->plane[spf->numplanes].planept[i] = atoi( strF1 );
	    }
	  }
	  spf->plane[spf->numplanes].numpoints = numPt;
	  spf->numplanes++;
	}
      }
    }
  } while (!FEnd);

  fclose(iop);
  return(TRUE);
  
}

void
insert_smf_data( char *filename, SPF_POINTS_DATA *spf  )
{
  int i, check;
  int FEnd       = 0;
  int lineCtr    = 0;
  int pointCtr   = 0;
  int polygonCtr = 0;

  int numE;
  int polygonE[MAX_POLYGON_SIZE];

  char line[MAX_LINE_LENGTH];
  char command[256];

  char strF1[256];
  char strF2[256];
  char strF3[256];
  
  FILE *iop;
  
  if ((iop = fopen( filename, "r")) == 0){
    fprintf( stderr, "ERROR: could not open smf-file file %s\n",
	     filename);
    return;
  } else {
    fprintf( stderr, "INFO: open smf-file file %s\n",
	     filename);
  }
  
  do{
    if (fgets(line,MAX_LINE_LENGTH,iop) == NULL) {
      FEnd=1;
    } else {
      lineCtr++;
      sscanf(line, "%s", command);
      if (!strcmp( command, "v")) {
	pointCtr++;
      } else if (!strcmp( command, "f")) {
	polygonCtr++;
      }
    }
  } while (!FEnd);

  if (pointCtr>0)
    fprintf( stderr, "INFO: insert %d points\n", pointCtr );
  if (polygonCtr>0)
    fprintf( stderr, "INFO: insert %d polygons\n", polygonCtr );
  
  rewind(iop);

  if ( pointCtr>0 ) {
    if ( spf->numpoints == 0 ) {
      spf->point = (POINT3 *) malloc( pointCtr * sizeof(POINT3) );
    } else {
      spf->point = (POINT3 *) 
	realloc( spf->point, ( pointCtr + spf->numpoints ) * sizeof(POINT3) );
    }
  }
  
  if ( polygonCtr>0 ) {
    if ( spf->numpolygons == 0 ) {
      spf->polygon = (POLYGON_REF *)
	malloc(polygonCtr*sizeof(POLYGON_REF));
    } else {
      spf->polygon = (POLYGON_REF *)
	realloc( spf->polygon,
		 ( polygonCtr + spf->numpolygons ) * sizeof(POLYGON_REF) );
    }
  }

  FEnd = 0;
  
  do{
    
    if (fscanf(iop, "%s", command) == EOF)
      FEnd=1;
    else{
      if (!strcmp( command, "v") ){
	if (fscanf(iop, "%s %s %s", strF1, strF2, strF3 ) == EOF) {
	  FEnd=1;
	} else {
	  spf->point[spf->numpoints].x = atof( strF1 );
	  spf->point[spf->numpoints].y = atof( strF2 );
	  spf->point[spf->numpoints].z = atof( strF3 );
	  spf->numpoints++;
	}
      } else if (!strcmp( command, "f") ){
	if (fscanf(iop, "%s %s %s", strF1, strF2, strF3 ) == EOF) {
	  FEnd=1;
	} else {
	  polygonE[0] = atoi( strF1 )-1;
	  polygonE[1] = atoi( strF2 )-1;
	  polygonE[2] = atoi( strF3 )-1;
	  numE=3;
	  check = TRUE;
	  for (i=0;i<numE;i++){
	    if (polygonE[i]<0 || polygonE[i]>=spf->numpoints )
	      check=FALSE;
	  }
	  if (check && numE>0) {
	    spf->polygon[spf->numpolygons].facept =
	      (int *) malloc( numE * sizeof(int) );
	    for (i=0;i<numE;i++) {
	      spf->polygon[spf->numpolygons].facept[i] = polygonE[i];
	    }
	    spf->polygon[spf->numpolygons].numfaces = numE;
	    spf->numpolygons++;
	  }
	}
      }
    }
  } while (!FEnd);

  fclose(iop);
  
}

void
spf_write_data( SPF_POINTS_DATA spf, char * filename )
{
  FILE             * fp;
  int                i,j;
  
  if ((fp = fopen( filename, "w")) == 0){
    fprintf(stderr, "ERROR: can't write spf file %s\n", filename );
    return;
  } else {
    fprintf(stderr, "INFO: write spf file %s\n", filename );
  }

  for (i=0; i<spf.numpoints; i++) {
    fprintf( fp, "POINT3D %.6f %.6f %.6f\n",
	     spf.point[i].x,
	     spf.point[i].y,
	     spf.point[i].z );
  }
  
  for (i=0; i<spf.numpolygons; i++) {
    fprintf( fp, "POLYGON" );
    for (j=0; j<spf.polygon[i].numfaces; j++) {
      fprintf( fp, " %d", spf.polygon[i].facept[j] );
    }
    fprintf( fp, " -1\n" );
  }
  
  for (i=0; i<spf.numplanes; i++) {
    fprintf( fp, "PLANE %.5f %.5f %.5f %.5f %d:",
	     spf.plane[i].param.a,
	     spf.plane[i].param.b,
	     spf.plane[i].param.c,
	     spf.plane[i].param.d,
	     spf.plane[i].numpoints );
    for (j=0; j<spf.plane[i].numpoints; j++) {
      fprintf( fp, " %d", spf.plane[i].planept[j] );
    }
    fprintf( fp, "\n" );
  }
  fclose(fp);
}


