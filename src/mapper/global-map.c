#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <sys/types.h>
#include <sys/time.h>
#include <time.h>
#include <unistd.h>
#include <string.h>
#include <ctype.h>
#include <values.h>

#include <navigation/utils.h>
#include "map2d.h"

MAP2D_SETTINGS          settings;

extern MAP2             global_map;

typedef struct {
  double          minx;
  double          maxx;
  double          miny;
  double          maxy;
} MINMAX2_VALUES;


int
map_pos_from_rpos( RPOS2 rpos, MAP2 *map, iVECTOR2 *v )
{
  v->x = map->center.x + (int) ((rpos.x-map->offset.x)/
				 (double)map->resolution);
  v->y = map->center.y + (int) ((rpos.y-map->offset.y)/
				(double)map->resolution);
  if (v->x<0) {
    return(FALSE);
  } else if (v->x>map->mapsize.x-1) {
    return(FALSE);
  }
  if (v->y<0) {
    return(FALSE);
  } else if (v->y>map->mapsize.y-1) {
    return(FALSE);
  }
  return(TRUE);
}

void
rpos_pos_from_map_pos( iVECTOR2 v, MAP2 *map, RPOS2 *rpos )
{
  rpos->x = ((v.x-map->center.x)*(double) map->resolution) + map->offset.x;
  rpos->y = ((v.y-map->center.y)*(double) map->resolution) + map->offset.y;
}

void
vec2_from_map_pos( iVECTOR2 v, MAP2 *map, VECTOR2 *pos )
{
  pos->x = ((v.x-map->center.x)*(double) map->resolution) + map->offset.x;
  pos->y = ((v.y-map->center.y)*(double) map->resolution) + map->offset.y;
}

int
map_pos_from_vec2( VECTOR2 pos, MAP2 *map, iVECTOR2 *v )
{
  v->x = map->center.x + (int) ((pos.x-map->offset.x)/
				(double)map->resolution);
  v->y = map->center.y + (int) ((pos.y-map->offset.y)/
				(double)map->resolution);
  if (v->x<0) {
    return(FALSE);
  } else if (v->x>map->mapsize.x-1) {
    return(FALSE);
  }
  if (v->y<0) {
    return(FALSE);
  } else if (v->y>map->mapsize.y-1) {
    return(FALSE);
  }
  return(TRUE);
}

MINMAX2_VALUES
find_min_max_values( REC2_DATA rec, int laserID )
{
  MINMAX2_VALUES   minmax;
  int              i, j;
  minmax.minx   = MAXDOUBLE;
  minmax.maxx   = -MAXDOUBLE; 
  minmax.miny   = MAXDOUBLE;
  minmax.maxy   = -MAXDOUBLE;
  for (i=0;i<rec.numlaserscans;i++) {
    if ( rec.lsens[i].id == laserID ) {
      for (j=0;j<rec.lsens[i].laser.numvalues;j++) {
	if (rec.lsens[i].coord[j].abspt.x<minmax.minx)
	  minmax.minx = rec.lsens[i].coord[j].abspt.x;
	if (rec.lsens[i].coord[j].abspt.x>minmax.maxx)
	  minmax.maxx = rec.lsens[i].coord[j].abspt.x;
	if (rec.lsens[i].coord[j].abspt.y<minmax.miny)
	  minmax.miny = rec.lsens[i].coord[j].abspt.y;
	if (rec.lsens[i].coord[j].abspt.y>minmax.maxy)
	  minmax.maxy = rec.lsens[i].coord[j].abspt.y;
      }
    }
  }
  return(minmax);
}

void
initialize_global_map( MAP2 *map,  int size_x,  int size_y,
		       int start_x, int start_y, double resolution,
		       RPOS2 start, int convolve )
{
  int x, y;

  map->resolution = resolution;

  map->mapsize.x = size_x;
  map->mapsize.y = size_y;

  map->offset = start;

  map->maphit   = mdalloc( 2, sizeof(float),  size_x, size_y );
  map->mapsum   = mdalloc( 2, sizeof(short),  size_x, size_y );
  map->mapprob  = mdalloc( 2, sizeof(double), size_x, size_y );

  if(convolve)
    map->calc  = mdalloc( 2, sizeof(double), size_x, size_y );

  map->center.x = start_x;
  map->center.y = start_y;

  fprintf( stderr, "* INFO: global map:   size:   %7d - %7d\n",
	   map->mapsize.x, map->mapsize.y );
  fprintf( stderr, "* INFO: global map:   center: %7.1f - %7.1f\n",
	   map->center.x, map->center.y );
  for (x=0;x<size_x;x++) {
    for (y=0;y<size_y;y++) {
      map->mapsum[x][y] = 0;
      map->maphit[x][y] = 0;
    }
  }
}

void
clear_global_map( MAP2 *map )
{
  int x, y;
  for (x=0;x<map->mapsize.x;x++) {
    for (y=0;y<map->mapsize.y;y++) {
      map->mapsum[x][y] = 0;
      map->maphit[x][y] = 0;
      map->mapprob[x][y] = 0.0;
    }
  }
}

void
update_global_map( LASERSENS2_DATA lsens, MAP2 * map )
{
  static int            first_time = TRUE; 
  static GRID_LINE      line;
  int                   i, j, x, y;
  int                   max_num_linepoints;
  iVECTOR2              start, end;
  VECTOR2               abspt;
  RMOVE2                nomove = {0.0, 0.0, 0.0};
  
  if (first_time) {
    max_num_linepoints =
      4 * ( settings.global_map_max_range /
	    settings.global_map_resolution );
    line.grid = (iVECTOR2 *) malloc( max_num_linepoints * sizeof(iVECTOR2) );
    first_time = FALSE;
  }
  
  for (j=0;j<lsens.laser.numvalues;j++) {
    if (lsens.laser.val[j] < settings.max_usable_laser_range ) {
      if (lsens.laser.val[j] > settings.global_map_max_range ) {
	abspt = compute_laser_abs_point( lsens.estpos,
					 settings.global_map_max_range,
					 nomove, // lsens.laser.offset[j],
					 lsens.laser.angle[j] );
	map_pos_from_vec2( abspt, map, &end );
      } else {
	if (1) {
	  abspt = compute_laser_abs_point( lsens.estpos,
					   lsens.laser.val[j],
					   nomove, // lsens.laser.offset[j],
					   lsens.laser.angle[j] );
	  map_pos_from_vec2( abspt, map, &end );
	} else {
	  map_pos_from_vec2( lsens.coord[j].abspt, map, &end );
	}
	if ( end.x>=0 && end.x<map->mapsize.x &&
	     end.y>=0 && end.y<map->mapsize.y ) {
	  if (settings.use_people_prob) {
	    map->maphit[end.x][end.y] +=
	      (1.0-lsens.ptracking.hprob[j]);
	  } else if (settings.use_global_ray_map) {
	    map->maphit[end.x][end.y] +=
	      (lsens.ptracking.hprob[j]);
	  } else {
	    //map->maphit[end.x][end.y]++;
	    map->maphit[end.x][end.y]+=1;
	  }
	  if (!settings.global_map_ray_model) {
	    map->mapsum[end.x][end.y]++;
	  }
	}
      }
      if (settings.global_map_ray_model) {
	map_pos_from_rpos( lsens.estpos, map, &start );
	grid_line( start, end, &line );
	for (i=0;i<line.numgrids;i++) {
	  x = line.grid[i].x;
	  y = line.grid[i].y;
	  if ( x>=0 && x<map->mapsize.x &&
	       y>=0 && y<map->mapsize.y ) {
	    if (lsens.laser.val[j]<=settings.global_map_max_range ) {
	      if (i>=line.numgrids-2) {
		map->maphit[x][y]++;
	      }
	      map->mapsum[x][y]++;
	    } else {
	      if (i<line.numgrids-1) {
		map->mapsum[x][y]++;
	      }
	    }
	  }
	}
      }
    }
  }
}

void
update_global_ray_map( LASERSENS2_DATA lsens, MAP2 * map,
		       int loop __attribute__ ((unused)) )
{
  static int            first_time = TRUE; 
  static GRID_LINE      line;
  int                   i, j, x, y;
  int                   max_num_linepoints;
  iVECTOR2              start, end;
  VECTOR2               abspt;
  RMOVE2                nomove = {0.0, 0.0, 0.0};
  
  if (first_time) {
    max_num_linepoints =
      2 * ( settings.global_ray_map_max_range /
	    settings.global_ray_map_resolution );
    line.grid = (iVECTOR2 *) malloc( max_num_linepoints * sizeof(iVECTOR2) );
    first_time = FALSE;
  }
  
  for (j=0;j<lsens.laser.numvalues;j++) {
    if (lsens.laser.val[j] < settings.max_usable_laser_range ) {
      if (lsens.laser.val[j] > settings.global_ray_map_max_range ) {
	abspt = compute_laser_abs_point( lsens.estpos,
					 settings.global_ray_map_max_range,
					 nomove, // lsens.laser.offset[j],
					 lsens.laser.angle[j] );
	map_pos_from_vec2( abspt, map, &end );
      } else {
	map_pos_from_vec2( lsens.coord[j].abspt, map, &end );
	if ( end.x>=0 && end.x<map->mapsize.x &&
	     end.y>=0 && end.y<map->mapsize.y ) {
	  map->mapsum[end.x][end.y] += (1.0-lsens.ptracking.hprob[j]);
	  map->maphit[end.x][end.y] += (lsens.ptracking.hprob[j]);
	}
      }
      map_pos_from_rpos( lsens.estpos, map, &start );
      grid_line( start, end, &line );
      for (i=0;i<line.numgrids-settings.global_ray_map_shorten_ray;i++) {
	x = line.grid[i].x;
	y = line.grid[i].y;
	if ( x>=0 && x<map->mapsize.x &&
	     y>=0 && y<map->mapsize.y ) {
	  map->mapsum[x][y]++;
	}
      }
    }
  }
}
