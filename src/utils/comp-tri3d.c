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

int
test_triangle_size( TRIANGLE3 tri, double min, double max )
{
  if ( vector3_distance( tri.point0, tri.point1 ) < max &&
       vector3_distance( tri.point0, tri.point1 ) > min &&
       vector3_distance( tri.point0, tri.point2 ) < max &&
       vector3_distance( tri.point0, tri.point2 ) > min &&
       vector3_distance( tri.point1, tri.point2 ) < max &&
       vector3_distance( tri.point1, tri.point2 ) > min ) {
    return(TRUE);
  }
  return(FALSE);
}

void
compute_rec3_triangles( REC3_DATA rec, TRIANGLE3_BEAMREF_SET *tri,
			double minsize, double maxsize )
{
  TRIANGLE3 ttri;
  int i, j, k, ntri;
  ntri = 0;
  for (i=0;i<rec.lsens.numsweeps;i++) {
    for (j=1;j<rec.lsens.sweep[i].numswscans;j++) {
      for (k=1;k<rec.lsens.sweep[i].swscan[j].laser.numvalues;k++) {
	if ( !rec.lsens.sweep[i].swscan[j-1].laser.maxrange[k-1] &&
	     !rec.lsens.sweep[i].swscan[j-1].laser.maxrange[k] &&
	     !rec.lsens.sweep[i].swscan[j].laser.maxrange[k-1] ) {
	  ttri.point0 = rec.lsens.sweep[i].swscan[j-1].coord[k-1].abspt;
	  ttri.point1 = rec.lsens.sweep[i].swscan[j-1].coord[k].abspt;
	  ttri.point2 = rec.lsens.sweep[i].swscan[j].coord[k-1].abspt;
	  if (test_triangle_size( ttri, minsize, maxsize ))
	      ntri++;
	}
	if ( !rec.lsens.sweep[i].swscan[j-1].laser.maxrange[k] &&
	     !rec.lsens.sweep[i].swscan[j].laser.maxrange[k-1] &&
	     !rec.lsens.sweep[i].swscan[j].laser.maxrange[k] ) {
	  ttri.point0 = rec.lsens.sweep[i].swscan[j-1].coord[k].abspt;
	  ttri.point1 = rec.lsens.sweep[i].swscan[j].coord[k-1].abspt;
	  ttri.point2 = rec.lsens.sweep[i].swscan[j].coord[k].abspt;
	  if (test_triangle_size( ttri, minsize, maxsize ))
	      ntri++;
	}
      }
    }
  }
  tri->numtriangles = ntri;
  tri->triangle = 
    (TRIANGLE3_BEAMREF *) malloc( ntri * sizeof(TRIANGLE3_BEAMREF) );
  ntri = 0;
  for (i=0;i<rec.lsens.numsweeps;i++) {
    for (j=1;j<rec.lsens.sweep[i].numswscans;j++) {
      for (k=1;k<rec.lsens.sweep[i].swscan[j].laser.numvalues;k++) {
	if ( !rec.lsens.sweep[i].swscan[j-1].laser.maxrange[k-1] &&
	     !rec.lsens.sweep[i].swscan[j-1].laser.maxrange[k] &&
	     !rec.lsens.sweep[i].swscan[j].laser.maxrange[k-1] ) {
	  ttri.point0 = rec.lsens.sweep[i].swscan[j-1].coord[k-1].abspt;
	  ttri.point1 = rec.lsens.sweep[i].swscan[j-1].coord[k].abspt;
	  ttri.point2 = rec.lsens.sweep[i].swscan[j].coord[k-1].abspt;
	  if (test_triangle_size( ttri, minsize, maxsize )) {
	    tri->triangle[ntri].ref0.sweep = i;
	    tri->triangle[ntri].ref0.scan  = j-1;
	    tri->triangle[ntri].ref0.beam  = k-1;
	    tri->triangle[ntri].ref1.sweep = i;
	    tri->triangle[ntri].ref1.scan  = j-1;
	    tri->triangle[ntri].ref1.beam  = k;
	    tri->triangle[ntri].ref2.sweep = i;
	    tri->triangle[ntri].ref2.scan  = j;
	    tri->triangle[ntri].ref2.beam  = k-1;
	    ntri++;
	  }
	}
	if ( !rec.lsens.sweep[i].swscan[j-1].laser.maxrange[k] &&
	     !rec.lsens.sweep[i].swscan[j].laser.maxrange[k-1] &&
	     !rec.lsens.sweep[i].swscan[j].laser.maxrange[k] ) {
	  ttri.point0 = rec.lsens.sweep[i].swscan[j-1].coord[k].abspt;
	  ttri.point1 = rec.lsens.sweep[i].swscan[j].coord[k-1].abspt;
	  ttri.point2 = rec.lsens.sweep[i].swscan[j].coord[k].abspt;
	  if (test_triangle_size( ttri, minsize, maxsize )) {
	    tri->triangle[ntri].ref0.sweep = i;
	    tri->triangle[ntri].ref0.scan  = j-1;
	    tri->triangle[ntri].ref0.beam  = k;
	    tri->triangle[ntri].ref1.sweep = i;
	    tri->triangle[ntri].ref1.scan  = j;
	    tri->triangle[ntri].ref1.beam  = k-1;
	    tri->triangle[ntri].ref2.sweep = i;
	    tri->triangle[ntri].ref2.scan  = j;
	    tri->triangle[ntri].ref2.beam  = k;
	    ntri++;
	  }
	}
      }
    }
  }
  fprintf( stderr, "num triangles = %d\n", ntri );
}



