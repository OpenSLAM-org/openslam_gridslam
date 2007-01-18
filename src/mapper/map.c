#include "map2d.h"

double
get_mapval( int pos_x, int pos_y, MAP2 map )
{
#ifdef SLOW
  fprintf( settings.devNULL, "%d %d -> %d %d\n", pos_x, pos_y,
	   map.mapsize.x,
	   map.mapsize.y );
#endif
  if ( pos_x>=0 && pos_x<map.mapsize.x &&
       pos_y>=0 && pos_y<map.mapsize.y ) {
    return( EPSILON + map.mapprob[pos_x][pos_y] ); 
  } else {
    return( EPSILON + settings.local_map_std_val );
  }
  return(0);
}

int
get_mapsum( int pos_x, int pos_y, MAP2 map )
{
#ifdef SLOW
  fprintf( settings.devNULL, "%d %d -> %d %d\n", pos_x, pos_y,
	   map.mapsize.x,
	   map.mapsize.y );
#endif
  if ( pos_x>=0 && pos_x<map.mapsize.x &&
       pos_y>=0 && pos_y<map.mapsize.y ) {
    return( map.mapsum[pos_x][pos_y] ); 
  } else {
    return(0);
  }
  return(0);
}

float
get_maphit( int pos_x, int pos_y, MAP2 map )
{
#ifdef SLOW
  fprintf( settings.devNULL, "%d %d -> %d %d\n", pos_x, pos_y,
	   map.mapsize.x,
	   map.mapsize.y );
#endif
  if ( pos_x>=0 && pos_x<map.mapsize.x &&
       pos_y>=0 && pos_y<map.mapsize.y ) {
    return( map.maphit[pos_x][pos_y] ); 
  } else {
    return(0);
  }
  return(0);
}

int
max( int a, int b )
{
  return( b>a?b:a );
}

int
find_quadrant( logtools_svector2_t center, int x, int y )
{
  if (x>(center.x)) {
    if (y>(center.y)) {
      return(0);
    } else {
      return(3);
    }
  } else {
    if (y>(center.y)) {
      return(1);
    } else {
      return(2);
    }
  }
}

logtools_svector2_t
newcenter( logtools_svector2_t center, int i, short stepsize )
{
  logtools_svector2_t ncenter = center;
  switch(i) {
  case 0:
    ncenter.x += stepsize;
    ncenter.y += stepsize;
    break;
  case 1:
    ncenter.x -= stepsize;
    ncenter.y += stepsize;
    break;
  case 2:
    ncenter.x -= stepsize;
    ncenter.y -= stepsize;
    break;
  case 3:
    ncenter.x += stepsize;
    ncenter.y -= stepsize;
    break;
  }
  return(ncenter);
}

void
alloc_tree( QUAD_TREE * tree, int level, logtools_svector2_t center, short stepsize )
{
  int i;
  short nstepsize = stepsize/2;
  tree->center = center;
  tree->level  = level;
  tree->inuse  = FALSE;
  if (level>0) {
    for( i=0; i<4; i++) {
      tree->elem[i] = (QUAD_TREE *) malloc( sizeof(QUAD_TREE) );
      alloc_tree( tree->elem[i], level-1, 
		  newcenter( center, i, nstepsize ), nstepsize );
    }
  }
}

void
initialize_qtree( QUAD_TREE * tree, int size_x, int size_y)
{
  int i,v,nlevel = max( (int) ceil(log10(size_x)/log10(2)),
			(int) ceil(log10(size_y)/log10(2)) );
  logtools_svector2_t center;
  fprintf( stderr, "* INFO: num levels       = %d\n", nlevel );
  v = 1;
  for (i=0;i<nlevel;i++) v=v*2;
  fprintf( stderr, "* INFO: size             = %d/%d\n", size_x, size_y );
  fprintf( stderr, "* INFO: poss. max size   = %d/%d\n", v, v );
  center.x = v-1;
  center.y = v-1;
  fprintf( stderr, "* INFO: tree center:       %5.1f %5.1f\n",
	   center.x/2.0, center.y/2.0 );
  fprintf( stderr, "* INFO: tree step:         %5.1f %5.1f\n",
	   0.5*(v/2), 0.5*(v/2) );
  
  fprintf( stderr, "* INFO: allocate tree: ... " );
  alloc_tree( tree, nlevel, center, v );
  fprintf( stderr, "done\n" );
}


void
initialize_map( MAP2 *map, int sx, int sy, int center_x, int center_y,
		double resolution, logtools_rpos2_t start )
{
  int x, y, j;

  map->mapsize.x  = sx;
  map->mapsize.y  = sy;
  map->resolution = resolution;
  map->offset     = start;

  fprintf( stderr, "* INFO: allocating memory ... " );

  if (1) {
    map->updated  =
      (unsigned char **) mdalloc( 2, sizeof(unsigned char),  sx, sy );
    map->maphit   = (float **) mdalloc( 2, sizeof(float),  sx, sy );
    map->mapsum   = (short **) mdalloc( 2, sizeof(short),  sx, sy );
    map->mapprob  = (float **) mdalloc( 2, sizeof(float), sx, sy );
    map->calc     = (float **) mdalloc( 2, sizeof(float), sx, sy );
  } else {
    map->updated  =
      (unsigned char **) malloc( sx * sizeof(unsigned char *) );
    map->maphit  =
      (float **) malloc( sx * sizeof(float *) );
    map->mapsum  =
      (short **) malloc( sx * sizeof(short *) );
    map->mapprob  =
    (float **) malloc( sx * sizeof(float *) );
    map->calc  =
      (float **) malloc( sx * sizeof(float *) );
    for (j=0; j<sx; j++) {
      map->updated[j]  =
	(unsigned char *) malloc( sy * sizeof(unsigned char) );
      map->maphit[j]  =
	(float *) malloc( sy * sizeof(float) );
      map->mapsum[j]  =
	(short *) malloc( sy * sizeof(short) );
      map->mapprob[j]  =
	(float *) malloc( sy * sizeof(float) );
      map->calc[j]  =
	(float *) malloc( sy * sizeof(float) );
    }
  }
  fprintf( stderr, "done\n" );
  map->center.x = center_x;
  map->center.y = center_y;

  fprintf( stderr, "* INFO: map:            %d %d\n",
	   map->mapsize.x, map->mapsize.y );
  fprintf( stderr, "* INFO: center:         %.1f %.1f\n",
	   map->center.x, map->center.y );
  fprintf( stderr, "* INFO: resolution:      %.2f\n",
	   map->resolution );
  fprintf( stderr, "* INFO: real-size:      [%.1f %.1f] [%.1f %.1f]\n",
	   -sx*map->resolution, sx*map->resolution,
	   -sy*map->resolution, sy*map->resolution );
  fprintf( stderr, "***************************************\n" );
  fprintf( stderr, "* INFO: clearing map ... " );

  for (x=0;x<sx;x++) {
    for (y=0;y<sy;y++) {
      map->mapprob[x][y] = settings.local_map_std_val;
      map->calc[x][y]    = settings.local_map_std_val;
      map->maphit[x][y]  = 0.0;
      map->mapsum[x][y]  = 0;
      map->updated[x][y]  = UPDT_NOT;
    }
  }
  initialize_qtree( &(map->qtree), sx, sy );
  fprintf( stderr, "***************************************\n" );
}

void
initialize_ray_map( MAP2 *map, int sx, int sy,
		    double resolution, logtools_rpos2_t start )
{
  int x, y;

  map->mapsize.x  = sx;
  map->mapsize.y  = sy;
  map->resolution = resolution;
  map->offset     = start;

  fprintf( stderr, "allocating memory ..." );
  map->maphit   = (float **) mdalloc( 2, sizeof(float),  sx, sy );
  map->mapsum   = (short **) mdalloc( 2, sizeof(short),  sx, sy );
  fprintf( stderr, "done\n" );
  map->center.x = 10; /* map->mapsize.x / 2.0; */
  map->center.y = map->mapsize.y / 2.0;

  fprintf( stderr, "map:       %5d %5d\n", map->mapsize.x, map->mapsize.y );
  fprintf( stderr, "center:    %5.1f %5.1f\n", map->center.x, map->center.y );
  fprintf( stderr, "resolutio: %5.2f\n", map->resolution );
  fprintf( stderr, "real-size: [%.2f %.2f] [%.2f %.2f]\n",
	   -sx*map->resolution, sx*map->resolution,
	   -sy*map->resolution, sy*map->resolution );

  for (x=0;x<sx;x++) {
    for (y=0;y<sy;y++) {
      map->maphit[x][y]  = 0.0;
      map->mapsum[x][y]  = 0;
    }
  }
}

void
simple_compute_map( MAP2 *map )
{
  int x, y;
  double odds, logodds;
  double dynamic_prob = (0.51);
  double static_prob  = (0.49);
  
  dynamic_prob = log(dynamic_prob/(1.0-dynamic_prob));
  static_prob  = log(static_prob/(1.0-static_prob));
  for (x=0;x<map->mapsize.x;x++) {
    for (y=0;y<map->mapsize.y;y++) {
      if (map->mapsum[x][y]>0) {
	logodds =
	  dynamic_prob * map->mapsum[x][y] +
	  static_prob * map->maphit[x][y];
	odds = exp(logodds);
	map->mapprob[x][y]     = (odds / (1+odds));
      } else {
	map->mapprob[x][y]     = 0.0;
      }
    }
  }
}
 
void
simple_convolve_map( MAP2 *map, logtools_gauss_kernel_t kernel )
{
  int x, y, k, hk;
  double ksum;
  
  hk = ( kernel.len - 1 ) / 2;
  for (x=hk;x<map->mapsize.x-hk;x++) {
    for (y=0;y<map->mapsize.y;y++) {
      ksum = 0.0;
      for (k=0;k<kernel.len;k++) {
	ksum += ( kernel.val[k] * map->mapprob[x+k-hk][y] );
      }
      map->calc[x][y] = ksum;
      if (map->calc[x][y]>1.0)
	map->calc[x][y]=1.0;
    }
  }
  for (x=0;x<map->mapsize.x;x++) {
    for (y=hk;y<map->mapsize.y-hk;y++) {
      ksum = 0.0;
      for (k=0;k<kernel.len;k++) {
	ksum += ( kernel.val[k] * map->calc[x][y+k-hk] );
      }
      map->mapprob[x][y] = ksum;
      if (map->mapprob[x][y]>1.0)
	map->mapprob[x][y]=1.0;
    }
  }
}

 
void
simple_inverse_convolve_map( MAP2 *map, logtools_gauss_kernel_t kernel )
{
  int x, y, k, hk;
  double ksum;
  
  hk = ( kernel.len - 1 ) / 2;
  for (x=hk;x<map->mapsize.x-hk;x++) {
    for (y=0;y<map->mapsize.y;y++) {
      ksum = 0.0;
      for (k=0;k<kernel.len;k++) {
	ksum += ( kernel.val[k] * (1.0-map->mapprob[x+k-hk][y]) );
      }
      map->calc[x][y] = ksum;
      if (map->calc[x][y]>1.0)
	map->calc[x][y]=1.0;
    }
  }
  for (x=0;x<map->mapsize.x;x++) {
    for (y=hk;y<map->mapsize.y-hk;y++) {
      ksum = 0.0;
      for (k=0;k<kernel.len;k++) {
	ksum += ( kernel.val[k] * map->calc[x][y+k-hk] );
      }
      if (map->mapprob[x][y]>1.0)
	map->mapprob[x][y]=1.0;
      map->mapprob[x][y] = 1.0-ksum;
    }
  }
}

 
void
simple_clear_map( MAP2 *map )
{
  int x, y;
  for (x=0;x<map->mapsize.x;x++) {
    for (y=0;y<map->mapsize.y;y++) {
	map->mapprob[x][y]     = 0.0;
	map->maphit[x][y]      = 0.0;
	map->mapsum[x][y]      = 0;
    }
  }
}
 
void
compute_prob_point( MAP2 *map, int x, int y )
{

  double          pobj;

  pobj = log(settings.local_map_object_prob/
	     (1.0-settings.local_map_object_prob));
  
  if ( x>=0 && x<map->mapsize.x &&
       y>=0 && y<map->mapsize.y ) {
    if (get_mapsum(x,y,*map)>0) {
      if (0) {
	map->mapprob[x][y]     = map->maphit[x][y] / map->mapsum[x][y];
      } else {
	map->mapprob[x][y]     = 1.0;
      }
    } else {
      map->mapprob[x][y]     = settings.local_map_std_val;
    }
  }
  
}

void
convolve_calc_point( MAP2 *map, logtools_gauss_kernel_t kernel, int hk,
		     int x, int y, double std_val )
{
  int                     k;
  double                  ksum;
  
  ksum = 0.0;

  if (x-hk>=0 && x+hk<map->mapsize.x) {
    for (k=0;k<2*hk+1;k++) {
      if (get_mapsum(x+k-hk,y,*map)>0) {
	ksum +=  ( kernel.val[k] * map->mapprob[x+k-hk][y] );
      } else {
	ksum +=  ( kernel.val[k] * std_val );
      }
    }
    map->calc[x][y]     = ksum;
    map->updated[x][y]  = UPDT_X;
  }
}

void
convolve_prob_point( MAP2 *map, logtools_gauss_kernel_t kernel,
		     int hk, int x, int y )
{
  int                     k;
  double                  ksum;
  
  ksum = 0.0;

  for (k=0;k<2*hk+1;k++) {
    if ( x>=0 && x<map->mapsize.x &&
	 y>=0 && y<map->mapsize.y)
      ksum +=  ( kernel.val[k] * map->calc[x][y+k-hk] );
  }
  map->mapprob[x][y]  = ksum;
  map->updated[x][y]  = UPDT_Y;
  
}

void
compute_prob_treemap( QUAD_TREE *tree, MAP2 *map )
{
  if ((tree->level)>0 ) {
    if (tree->elem[0]->inuse)
      compute_prob_treemap( tree->elem[0], map );
    if (tree->elem[1]->inuse)
      compute_prob_treemap( tree->elem[1], map );
    if (tree->elem[2]->inuse)
      compute_prob_treemap( tree->elem[2], map );
    if (tree->elem[3]->inuse)
      compute_prob_treemap( tree->elem[3], map );
  } else {
    compute_prob_point( map, (tree->center.x/2), (tree->center.y/2) );
  }
}

/*
void
compute_ray_prob_treemap( QUAD_TREE *tree, MAP2 *map )
{
  if ((tree->level)>0 ) {
    if (tree->elem[0]->inuse)
      compute_ray_prob_treemap( tree->elem[0], map );
    if (tree->elem[1]->inuse)
      compute_ray_prob_treemap( tree->elem[1], map );
    if (tree->elem[2]->inuse)
      compute_ray_prob_treemap( tree->elem[2], map );
    if (tree->elem[3]->inuse)
      compute_ray_prob_treemap( tree->elem[3], map );
  } else {
    compute_ray_prob_point( map, (tree->center.x/2), (tree->center.y/2) );
  }
}
*/

void
convolve_calc_treemap( QUAD_TREE *tree, MAP2 *map,
		       logtools_gauss_kernel_t kernel, int hk, double std )
{
  int i, j;
  if ((tree->level)>0 ) {
    if (tree->elem[0]->inuse)
      convolve_calc_treemap( tree->elem[0], map, kernel, hk, std );
    if (tree->elem[1]->inuse)
      convolve_calc_treemap( tree->elem[1], map, kernel, hk, std );
    if (tree->elem[2]->inuse)
      convolve_calc_treemap( tree->elem[2], map, kernel, hk, std );
    if (tree->elem[3]->inuse)
      convolve_calc_treemap( tree->elem[3], map, kernel, hk, std );
  } else {
    if ( (tree->center.x/2)>hk+1 && (tree->center.x/2)<map->mapsize.x-hk-1 &&
	 (tree->center.y/2)>hk+1 && (tree->center.y/2)<map->mapsize.y-hk-1 ) {
      for (i=(tree->center.x/2)-hk;i<(tree->center.x/2)+hk;i++) {
	for (j=(tree->center.y/2)-hk;j<(tree->center.y/2)+hk;j++) {
	  if (map->updated[i][j] != UPDT_X)
	    convolve_calc_point( map, kernel, hk, i, j, std );
	}
      }
    }

  }
}

void
convolve_prob_treemap( QUAD_TREE *tree, MAP2 *map,
		       logtools_gauss_kernel_t kernel, int hk )
{
  int i, j;
  if ((tree->level)>0 ) {
    if (tree->elem[0]->inuse)
      convolve_prob_treemap( tree->elem[0], map, kernel, hk );
    if (tree->elem[1]->inuse)
      convolve_prob_treemap( tree->elem[1], map, kernel, hk );
    if (tree->elem[2]->inuse)
      convolve_prob_treemap( tree->elem[2], map, kernel, hk );
    if (tree->elem[3]->inuse)
      convolve_prob_treemap( tree->elem[3], map, kernel, hk );
  } else {
    if ( (tree->center.x/2)>hk+1 && (tree->center.x/2)<map->mapsize.x-hk-1 &&
	 (tree->center.y/2)>hk+1 && (tree->center.y/2)<map->mapsize.y-hk-1 ) {
      for (i=(tree->center.x/2)-hk;i<(tree->center.x/2)+hk;i++) {
	for (j=(tree->center.y/2)-hk;j<(tree->center.y/2)+hk;j++) {
	  if (map->updated[i][j] != UPDT_Y)
	    convolve_prob_point( map, kernel, hk, i, j );
	}
      }
    }
  }
}

void
convolve_map( MAP2 *map )
{
  int                              i;
  static int                       hk, first_time = TRUE; 
  static logtools_gauss_kernel_t   kernel;

  if (first_time) {
    hk = (settings.local_map_kernel_len-1)/2;
    kernel = logtools_compute_gauss_kernel( settings.local_map_kernel_len );
    first_time = FALSE;
  }

  compute_prob_treemap(  &(map->qtree), map );
  for (i=0;i<settings.local_map_num_convolve;i++) {
    convolve_calc_treemap( &(map->qtree), map, kernel, hk,
			   settings.local_map_std_val );
    convolve_prob_treemap( &(map->qtree), map, kernel, hk );
  }
  
}

int
compute_map_pos_from_rpos( logtools_rpos2_t rpos, MAP2 *map,
			   logtools_ivector2_t *v )
{
  v->x = (int) (map->center.x + (rpos.x/(double)map->resolution));
  v->y = (int) (map->center.y + (rpos.y/(double)map->resolution));
  if (v->x<0 || v->x>map->mapsize.x-1) {
    return(FALSE);
  }
  if (v->y<0 || v->y>map->mapsize.y-1) {
    return(FALSE);
  }
  return(TRUE);
}

int
compute_map_pos_from_vec2( logtools_vector2_t pos, MAP2 *map,
			   logtools_ivector2_t *v )
{
  v->x = (int) (map->center.x + (pos.x/(double)(map->resolution)));
  v->y = (int) (map->center.y + (pos.y/(double)(map->resolution)));
  if (v->x<0 || v->x>map->mapsize.x-1) {
    return(FALSE);
  }
  if (v->y<0 || v->y>map->mapsize.y-1) {
    return(FALSE);
  }
  return(TRUE);
}

void
mark_maphitpoint( QUAD_TREE *tree, MAP2 *map, int x, int y, float value )
{
  tree->inuse=TRUE;
  if ((tree->level)>0) {
    mark_maphitpoint( tree->elem[find_quadrant( tree->center, x, y )], 
		      map, x, y, value  );
  } else {
    map->maphit[tree->center.x/2][tree->center.y/2] += value;
  }
}

void
mark_mapsumpoint( QUAD_TREE *tree, MAP2 *map, int x, int y )
{
  tree->inuse=TRUE;
  if ((tree->level)>0) {
    mark_mapsumpoint( tree->elem[find_quadrant( tree->center, x, y )], 
		      map, x, y );
  } else {
    map->mapsum[tree->center.x/2][tree->center.y/2]++;
  }
}

void
set_maphitpoint( MAP2 *map, int x, int y, float value  )
{
  mark_maphitpoint( &(map->qtree), map, 2*x, 2*y, value );
}

void
set_mapsumpoint( MAP2 *map, int x, int y )
{
  mark_mapsumpoint( &(map->qtree), map, 2*x, 2*y );
}

void
tree_list( QUAD_TREE *tree , int *ct )
{
  if ((tree->level)>0 ) {
    if (tree->elem[0]->inuse)
      tree_list( tree->elem[0], ct );
    if (tree->elem[1]->inuse)
      tree_list( tree->elem[1], ct );
    if (tree->elem[2]->inuse)
      tree_list( tree->elem[2], ct );
    if (tree->elem[3]->inuse)
      tree_list( tree->elem[3], ct );
  } else {
    //    fprintf( stderr, "%d %d\n", tree->center.x, tree->center.y );
    (*ct)++;
  }
}

void
clear_local_treemap( QUAD_TREE *tree, MAP2 *map, int hk )
{
  int i,j;
  if ((tree->level)>0 ) {
    if (tree->elem[0]->inuse)
      clear_local_treemap( tree->elem[0], map, hk );
    if (tree->elem[1]->inuse)
      clear_local_treemap( tree->elem[1], map, hk );
    if (tree->elem[2]->inuse)
      clear_local_treemap( tree->elem[2], map, hk );
    if (tree->elem[3]->inuse)
      clear_local_treemap( tree->elem[3], map, hk );
  } else {
    if ( (tree->center.x/2)>hk-1 && (tree->center.x/2)<map->mapsize.x-hk &&
	 (tree->center.y/2)>hk-1 && (tree->center.y/2)<map->mapsize.y-hk ) {
      for (i=(tree->center.x/2)-hk;i<=(tree->center.x/2)+hk;i++) {
	for (j=(tree->center.y/2)-hk;j<=(tree->center.y/2)+hk;j++) {
	  map->maphit[i][j]  = 0;
	  map->mapsum[i][j]  = 0;
	  map->mapprob[i][j] = settings.local_map_std_val;
	  map->calc[i][j]    = settings.local_map_std_val;
	  map->updated[i][j] = UPDT_NOT;
	}
      }
    }
  }
  tree->inuse = FALSE;
}

void
create_local_map( MAP2 *map, logtools_lasersens2_data_t data,
		  logtools_rmove2_t movement )
{
  int                            i, max_num_linepoints;
  static logtools_rpos2_t        npos = {0.0, 0.0, 0.0 };
  static logtools_rmove2_t       nmove = {0.0, 0.0, 0.0 };
  logtools_rpos2_t               rpos;
  logtools_ivector2_t            start, end;
  logtools_vector2_t             lpos;
  static int          first_time = TRUE;
  static logtools_grid_line_t    line;
  logtools_rmove2_t   offset;

  if (first_time) {
    max_num_linepoints = (int)
      (2 * ( settings.local_map_max_range / settings.local_map_resolution ));
    line.grid = (logtools_ivector2_t *) malloc( max_num_linepoints * sizeof(logtools_ivector2_t) );
    first_time = FALSE;
  }

  offset.forward  = movement.forward; //  + data.laser.offset[i].forward;
  offset.sideward = movement.sideward; // + data.laser.offset[i].sideward;
  offset.rotation = movement.rotation; // + data.laser.offset[i].rotation;

  rpos  = logtools_rpos2_backwards_with_movement2( npos, offset );

  compute_map_pos_from_rpos( rpos, map, &start );

  for (i=0;i<data.laser.numvalues;i++) {
    
    lpos = logtools_compute_laser_points( rpos,
					  data.laser.val[i],
					  nmove, // data.laser.offset[i],
					  data.laser.angle[i] );
    if (compute_map_pos_from_vec2( lpos, map, &end )) {
      if ( data.laser.val[i]<settings.local_map_max_range &&
	   end.x>=0 && end.x<map->mapsize.x &&
	   end.y>=0 && end.y<map->mapsize.y ) {
	set_mapsumpoint( map, end.x, end.y );
	set_maphitpoint( map, end.x, end.y, 1.0 );
      }
    }
  }
}

void
compute_probs_of_global_map( MAP2 * global_map )
{
  int        x, y;
  
  for (x=0;x<global_map->mapsize.x;x++) {
    for (y=0;y<global_map->mapsize.y;y++) {
      if (global_map->mapsum[x][y]>0) {
	global_map->mapprob[x][y] =
	  ( global_map->maphit[x][y] /
	    (double) ( global_map->maphit[x][y] + global_map->mapsum[x][y] ) );
      } else {
	global_map->mapprob[x][y] =
	  settings.global_map_std_val;
      }
    }
  }
  
}

void
map_change_settings( int set )
{
  fprintf( stderr, "xxx= %d\n", set );
  settings.change_map = set;
  settings.view_path = FALSE;
}
