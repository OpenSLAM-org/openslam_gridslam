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
compute_abs_scan_coordpts( LASERSENS3_DATA *data )
{
  VECTOR3 p, n, nR, nT, off;
  int i;

  nR.x = data->estpos.rot.x + data->declination.tilt;
  nR.y = M_PI + data->estpos.rot.y;
  nR.z = data->estpos.rot.z + data->declination.pan;

  nT.x = data->estpos.pos.x;
  nT.y = data->estpos.pos.y;
  nT.z = data->estpos.pos.z;

  // data->origin = rotate_and_translate_vector3( data->laseroffset, nR, nT );
  data->origin = translate_and_rotate_vector3( data->laseroffset, nR, nT );

  off = rotate_vector3( data->laseroffset, nR );
  data->origin = vector3_add( nT, off );
  
  nT.x = 0.0;
  nT.y = 0.0;
  nT.z = 0.0;

  for (i=0; i<data->laser.numvalues; i++) {

    p.x = data->laser.val[i] * sin(data->laser.angle[i]);
    p.y = data->laser.val[i] * cos(data->laser.angle[i]);
    p.z = 0.0;

    //    n = rotate_and_translate_vector3( p, nR, nT );
    n = translate_and_rotate_vector3( nT, nR, p );

    data->coord[i].abspt = vector3_add( data->origin, n );

  }
}

void
compute_rel_scan_coordpts( LASERSENS3_DATA *data )
{
  VECTOR3 p, n, nR, nT;
  VECTOR2 l;
  int i;

  nR.x = 0.0;
  nR.y = 0.0;
  nR.z = data->declination.pan;
  
  nT.x = 0.0;
  nT.y = 0.0;
  nT.z = 0.0;

  //  data->origin = rotate_and_translate_vector3( data->laseroffset, nR, nT );
  data->origin = translate_and_rotate_vector3( data->laseroffset, nR, nT );

  for (i=0; i<data->laser.numvalues; i++) {

    l.x = data->laser.val[i] * sin(data->laser.angle[i]);
    l.y = data->laser.val[i] * cos(data->laser.angle[i]);
    
    p.x = l.y;
    p.y = l.x * cos( -data->declination.tilt );
    p.z = l.x * sin( -data->declination.tilt );

    n = vector3_add( data->laseroffset, p );
    data->coord[i].abspt = rotate_and_translate_vector3( n, nR, nT );

  }
}

void
compute_abs_sweep_coordpts( LASERSWEEP3_DATA *data )
{
  int i;
  for (i=0;i<data->numswscans;i++) {
    compute_abs_scan_coordpts( &data->swscan[i] );
  }
}

void
compute_abs_rec3_coordpts( REC3_DATA *rec )
{
  int i;
  for (i=0;i<rec->lsens.numsweeps;i++) {
    compute_abs_sweep_coordpts( &rec->lsens.sweep[i] );
  }
}

void
compute_rel_sweep_coordpts( LASERSWEEP3_DATA *data )
{
  int i;
  for (i=0;i<data->numswscans;i++) {
    compute_rel_scan_coordpts( &data->swscan[i] );
  }
}

void
compute_rel_rec3_coordpts( REC3_DATA *rec )
{
  int i;
  for (i=0;i<rec->lsens.numsweeps;i++) {
    compute_rel_sweep_coordpts( &rec->lsens.sweep[i] );
  }
}

void
compute_alloc_rec3_coordpts( REC3_DATA *rec )
{
  int i, j;
  for (i=0;i<rec->lsens.numsweeps;i++) {
    for (j=0;j<rec->lsens.sweep[i].numswscans;j++) {
      rec->lsens.sweep[i].swscan[j].coord =
	(LASER_COORD3 *) malloc( rec->lsens.sweep[i].swscan[j].laser.numvalues*sizeof(LASER_COORD3) );
    }
  }
}

void
compute_estpos( REC3_DATA *rec )
{
  int i, j, skip, posPtr = 0;
  for (i=0;i<rec->lsens.numsweeps;i++) {
    for (j=0;j<rec->lsens.sweep[i].numswscans;j++) {
      skip = TRUE;
      do {
	if ( posPtr+1 < rec->numpositions &&
	     timeCompare( rec->psens[posPtr+1].time,
			  rec->lsens.sweep[i].swscan[i].laser.time ) <= 0 ) {
	  posPtr++;
	}
	else {
	  skip = FALSE;
	}
      } while (skip);
      rec->lsens.sweep[i].swscan[j].estpos = rec->psens[posPtr].rpos;
    }
  }
}

void
mark_maxrange( REC3_DATA *rec, double MAX_RANGE )
{
  int i, j, k;
  for (i=0;i<rec->lsens.numsweeps;i++) {
    for (j=0;j<rec->lsens.sweep[i].numswscans;j++) {
      for (k=0;k<rec->lsens.sweep[i].swscan[j].laser.numvalues;k++) {
	if (rec->lsens.sweep[i].swscan[j].laser.val[k] >= MAX_RANGE)
	  rec->lsens.sweep[i].swscan[j].laser.maxrange[k] = TRUE;
	else
	  rec->lsens.sweep[i].swscan[j].laser.maxrange[k] = FALSE;
      }
    }
  }
}


VECTOR2
compute_laser_abs_point( RPOS2 rpos, double val, RMOVE2 offset, double angle )
{
  VECTOR2 abspt;
  abspt.x =
    rpos.x + 
    cos( angle+offset.rotation+rpos.o ) * val;
  abspt.y =
    rpos.y + 
    sin( angle+offset.rotation+rpos.o ) * val;
  return(abspt);
}


LASER_COORD2
compute_laser2d_coord( LASERSENS2_DATA lsens, int i )
{
  double        val;
  LASER_COORD2  coord;
  RPOS2         rpos, npos;

  rpos  = compute_rpos2_with_movement2( lsens.estpos,
					lsens.laser.offset[i] );
  npos.x = 0.0;
  npos.y = 0.0;
  npos.o = 0.0;

  val = lsens.laser.val[i];
  coord.relpt = compute_laser_abs_point( npos, val,
					 lsens.laser.offset[i],
					 lsens.laser.angle[i] );
  coord.abspt = compute_laser_abs_point( rpos, val,
					 lsens.laser.offset[i],
					 lsens.laser.angle[i] );
  return(coord);
}

void
compute_rec2d_coordpts( REC2_DATA *rec )
{
  int i, j;
  for (j=0;j<rec->numlaserscans;j++) {
    if (rec->lsens[j].coord==NULL) {
      rec->lsens[j].coord =
	(LASER_COORD2 *) malloc( rec->lsens[j].laser.numvalues *
				 sizeof(LASER_COORD2) );
      for (i=0;i<rec->lsens[j].laser.numvalues;i++) {
	rec->lsens[j].coord[i] = compute_laser2d_coord(rec->lsens[j], i);
      }
    }
  }
}

