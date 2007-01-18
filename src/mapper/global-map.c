#include "map2d.h"

extern MAP2D_SETTINGS   settings;
extern MAP2             global_map;

typedef struct {
  double          minx;
  double          maxx;
  double          miny;
  double          maxy;
} MINMAX2_VALUES;


int
map_pos_from_rpos( logtools_rpos2_t rpos, MAP2 *map, logtools_ivector2_t *v )
{
  v->x = (int) (map->center.x + ((rpos.x-map->offset.x)/
				 (double)map->resolution));
  v->y = (int) (map->center.y + ((rpos.y-map->offset.y)/
				(double)map->resolution));
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
rpos_pos_from_map_pos( logtools_ivector2_t v, MAP2 *map, logtools_rpos2_t *rpos )
{
  rpos->x = ((v.x-map->center.x)*(double) map->resolution) + map->offset.x;
  rpos->y = ((v.y-map->center.y)*(double) map->resolution) + map->offset.y;
}

void
vec2_from_map_pos( logtools_ivector2_t v, MAP2 *map, logtools_vector2_t *pos )
{
  pos->x = ((v.x-map->center.x)*(double) map->resolution) + map->offset.x;
  pos->y = ((v.y-map->center.y)*(double) map->resolution) + map->offset.y;
}

int
map_pos_from_vec2( logtools_vector2_t pos, MAP2 *map, logtools_ivector2_t *v )
{
  v->x = (int) (map->center.x + 
		((pos.x-map->offset.x)/(double)map->resolution));
  v->y = (int) (map->center.y + ((pos.y-map->offset.y)/
				 (double)map->resolution));
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
initialize_global_map( MAP2 *map,  int size_x,  int size_y,
		       int start_x, int start_y, double resolution,
		       logtools_rpos2_t start, int convolve )
{
  int x, y;

  map->resolution = resolution;

  map->mapsize.x = size_x;
  map->mapsize.y = size_y;

  map->offset = start;

  map->maphit   = (float **) mdalloc( 2, sizeof(float),  size_x, size_y );
  map->mapsum   = (short **) mdalloc( 2, sizeof(short),  size_x, size_y );
  map->mapprob  = (float **) mdalloc( 2, sizeof(float), size_x, size_y );

  if(convolve)
    map->calc  = (float **) mdalloc( 2, sizeof(float), size_x, size_y );

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
update_global_map( logtools_lasersens2_data_t lsens, MAP2 * map )
{
  static int                     first_time = TRUE; 
  static logtools_grid_line_t    line;
  int                            i, j, x, y;
  int                            max_num_linepoints;
  logtools_ivector2_t            start, end;
  logtools_vector2_t             abspt;
  logtools_rmove2_t              nomove = {0.0, 0.0, 0.0};
  
  if (first_time) {
    max_num_linepoints =
      (int) (4 * ( settings.global_map_max_range /
		   settings.global_map_resolution ));
    line.grid = (logtools_ivector2_t *) malloc( max_num_linepoints *
						sizeof(logtools_ivector2_t) );
    first_time = FALSE;
  }
  
  for (j=0;j<lsens.laser.numvalues;j++) {
    if (lsens.laser.val[j] < settings.max_usable_laser_range ) {
      if (lsens.laser.val[j] > settings.global_map_max_range ) {
	abspt = logtools_compute_laser_points( lsens.estpos,
					       settings.global_map_max_range,
					       nomove, 
					       lsens.laser.angle[j] );
	map_pos_from_vec2( abspt, map, &end );
      } else {
	abspt = logtools_compute_laser_points( lsens.estpos,
					       lsens.laser.val[j],
					       nomove, 
					       lsens.laser.angle[j] );
	map_pos_from_vec2( abspt, map, &end );
	if ( end.x>=0 && end.x<map->mapsize.x &&
	     end.y>=0 && end.y<map->mapsize.y ) {
	  map->maphit[end.x][end.y]+=1;
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

