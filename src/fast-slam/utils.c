#include "fast-slam.h"
#include <magick/api.h>

#define UNKNOWN  0
#define IN       1
#define OUT      2

#define EPSILON       0.000000001
#define MAP_STD_VAL   0.00000

double round(double x);

int
find_best_particle_logsum( SAMPLE_SET pset )
{
  int i, idx = 0;
  double val = -MAXDOUBLE;
  for (i=0;i<pset.numparticles;i++) {
    if (pset.particle[i].logsum>val) {
      idx = i;
      val = pset.particle[i].logsum;
    }
  }
  return(idx);
}

int
find_best_particle_value( SAMPLE_SET pset )
{
  int i, idx = 0;
  double val = -MAXDOUBLE;
  for (i=0;i<pset.numparticles;i++) {
    if (pset.particle[i].val>val) {
      idx = i;
      val = pset.particle[i].val;
    }
  }
  return(idx);
}

double
prob_unknown_space( double length, int endpoint )
{
  int idx;
  double frac;
  /* probs computed with seattle (intel research lab) map
     every 10 cm from 0.1 to 9m */ 
  static double table1[] = { 0.027407, 0.023016, 0.021626, 0.020518, 0.019673, 0.018884, 0.017912, 0.017986, 0.016391, 0.016052, 0.015289, 0.014647, 0.013758, 0.013561, 0.012637, 0.012228, 0.011669, 0.010961, 0.010693, 0.010087, 0.009296, 0.008914, 0.008086, 0.007877, 0.007612, 0.007329, 0.006913, 0.006440, 0.006158, 0.006033, 0.005551, 0.005274, 0.004999, 0.004867, 0.004521, 0.004226, 0.004029, 0.003899, 0.003651, 0.003372, 0.003321, 0.002983, 0.002919, 0.002815, 0.002763, 0.002553, 0.002391, 0.002405, 0.002267, 0.002037, 0.001937, 0.001808, 0.001618, 0.001627, 0.001474, 0.001443, 0.001334, 0.001302, 0.001196, 0.001141, 0.001215, 0.001155, 0.001154, 0.001046, 0.001003, 0.000949, 0.000965, 0.000931, 0.000856, 0.000814, 0.000811, 0.000727, 0.000710, 0.000743, 0.000717, 0.000646, 0.000616, 0.000658, 0.000645, 0.000644, 0.000651, 0.000553, 0.000553, 0.000485, 0.000561, 0.000605, 0.000557, 0.000503, 0.000467, 0.000521 };
  static double table2[] = { 0.840712, 0.798510, 0.759555, 0.722885, 0.686197, 0.653546, 0.620038, 0.590059, 0.562270, 0.534048, 0.507807, 0.482703, 0.459018, 0.434243, 0.415033, 0.394681, 0.373622, 0.355107, 0.339661, 0.323134, 0.309463, 0.294318, 0.281096, 0.266992, 0.254206, 0.243801, 0.232858, 0.221439, 0.212163, 0.202476, 0.190680, 0.183310, 0.176945, 0.168610, 0.160928, 0.153825, 0.147223, 0.140670, 0.135154, 0.129761, 0.123861, 0.118102, 0.113574, 0.110250, 0.104038, 0.099963, 0.095617, 0.092629, 0.088913, 0.085092, 0.082401, 0.078720, 0.077465, 0.073431, 0.072046, 0.068684, 0.067358, 0.063863, 0.062165, 0.061448, 0.058601, 0.057354, 0.055785, 0.053340, 0.052561, 0.050669, 0.049378, 0.047809, 0.045767, 0.045169, 0.043498, 0.042803, 0.041253, 0.041163, 0.039276, 0.038413, 0.037851, 0.036292, 0.035119, 0.034019, 0.032988, 0.032626, 0.031228, 0.030708, 0.030152, 0.028964, 0.028233, 0.027682, 0.026667, 0.026046 };
  if (length>900.0) {
    idx = 89;
  } else {
    idx = (int) (length / 10.0);
    if (idx<=0) {
      idx=0;
    } else if (idx>0) {
      frac = (length/10.0)-idx;
      if (endpoint)
	return( table1[idx-1]+frac*(table1[idx]-table1[idx-1]) );
      else
	return( table2[idx-1]+frac*(table2[idx]-table2[idx-1]) );
    }
  }
  if (endpoint)
    return(table1[idx]);
  else
    return(table2[idx]);
}

void
copy_particle( PARTICLE src, PARTICLE *dest )
{
  int        i;
  HISTORY  * ptr = dest->hist;
  *dest = src;
  dest->hist = ptr;
  for (i=0; i<src.histlen; i++) {
    dest->hist[i] = src.hist[i];
  }
}

void
resample( SAMPLE_SET sample1, SAMPLE_SET * sample2 )
{
  double  u, c, delta, rezi = 1 / (double) sample1.numparticles;
  int      i, j;
  delta = ( rand()/(double) RAND_MAX ) / (double) (sample1.numparticles+1);

  c = sample1.particle[0].val;
  for (i=0,j=0; j<sample1.numparticles; j++) {
    u = delta + j * rezi;
    while(u>c) {
      i++;
      c += sample1.particle[i].val;
    }
    copy_particle( sample1.particle[i], &(sample2->particle[j]) );
  }
}

void
simple_convolve_map2( MAP2 *map)
/* for fixed convolve size of 3 */
{
  int x, y;
  int maxX = map->mapsize.x-1;
  int maxY = map->mapsize.y-1;
  float **prob = map->mapprob;
  float **calc = map->calc;
  
  for (x=1;x<maxX;x++) {
    for (y=0;y<map->mapsize.y;y++) {
      calc[x][y] = 0.25 *  (prob[x-1][y]+prob[x+1][y])
	+ 0.5 * prob[x][y];
    }
  }
  
  for (x=0;x<map->mapsize.x;x++) {
    float *tmp = calc[x];
    for (y=1;y<maxY;y++) {
      prob[x][y] = 0.25 *  (tmp[y-1]+tmp[y+1])
	+ 0.5 * tmp[y];
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


int
map_pos_from_rpos( logtools_rpos2_t rpos, MAP2 *map, logtools_ivector2_t *v )
{
  v->x = map->center.x + (int) round((rpos.x-map->offset.x)/
				     (double)map->resolution);
  v->y = map->center.y + (int) round((rpos.y-map->offset.y)/
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

int
map_pos_from_vec2( logtools_vector2_t pos, MAP2 *map, logtools_ivector2_t *v )
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

void
update_map( MAP2 * map, int numvalues, float  * val, float * angle,
	    logtools_rpos2_t estpos, double max_range, double max_usable  )
{
  static int                    first_time = TRUE; 
  static logtools_grid_line_t   line;
  static int                    max_num_linepoints = 0;
  int                           i, j, x, y;
  logtools_ivector2_t           start, end;
  logtools_vector2_t            abspt;
  logtools_rmove2_t             nomove = {0.0, 0.0, 0.0};
  
  if (first_time) {
    max_num_linepoints =
      10 * ( max_range / map->resolution );
    line.grid = (logtools_ivector2_t *) malloc( max_num_linepoints * sizeof(logtools_ivector2_t) );
    first_time = FALSE;
  }

  for (j=0;j<numvalues;j++) {
    if (val[j] <= max_usable ) {
      if (val[j] > max_range ) {
	abspt = logtools_compute_laser_points( estpos, max_range, nomove, 
					       angle[j] );
	map_pos_from_vec2( abspt, map, &end );
      } else {
	abspt = logtools_compute_laser_points( estpos, val[j]+(map->resolution),
					       nomove, angle[j] );
	map_pos_from_vec2( abspt, map, &end );
      }
      map_pos_from_rpos( estpos, map, &start );
      grid_line( start, end, &line );
      for (i=0;i<line.numgrids;i++) {
	x = line.grid[i].x;
	y = line.grid[i].y;
	if ( x>=0 && x<map->mapsize.x &&
	     y>=0 && y<map->mapsize.y ) {
	  if (val[j]<=max_range ) {
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

void
map_initialize( MAP2 *map, int sx, int sy, int center_x,
		int center_y, double resolution, logtools_rpos2_t start )
{
  int x, y;

  map->mapsize.x  = sx;
  map->mapsize.y  = sy;
  map->resolution = resolution;
  map->offset     = start;

  fprintf( stderr, "* INFO: allocating memory ... " );
  map->maphit   = mdalloc( 2, sizeof(float),  sx, sy );
  map->mapsum   = mdalloc( 2, sizeof(short),  sx, sy );
  map->mapprob  = mdalloc( 2, sizeof(float),  sx, sy );
  map->calc     = mdalloc( 2, sizeof(float),  sx, sy );
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

  for (x=0;x<sx;x++) {
    for (y=0;y<sy;y++) {
      map->mapprob[x][y] = 0.0;
      map->calc[x][y]    = 0.0;
      map->maphit[x][y]  = 0.0;
      map->mapsum[x][y]  = 0;
    }
  }
}

void
map_clear( MAP2 *map )
{
  int x, y;
  for (x=0;x<map->mapsize.x;x++) {
    for (y=0;y<map->mapsize.y;y++) {
      map->mapprob[x][y] = 0.0;
      map->calc[x][y]    = 0.0;
      map->maphit[x][y]  = 0.0;
      map->mapsum[x][y]  = 0;
    }
  }
}

void
compute_probs_of_map( MAP2 * map )
{
  int        x, y;
  for (x=0;x<map->mapsize.x;x++) {
    for (y=0;y<map->mapsize.y;y++) {
      if (map->mapsum[x][y]>0) {
	map->mapprob[x][y] =
	  3.0 * (map->maphit[x][y] / (double) ( map->mapsum[x][y] ));
	if (map->mapprob[x][y]>1.0)
	  map->mapprob[x][y] = 0.999;
      } else {
	map->mapprob[x][y] = EPSILON;
      }
    }
  }
  
}

#define ADDB        2

void
compute_calc_of_map( MAP2 * map )
{
  int        x, y, vx, vy, abort;
  for (x=0;x<map->mapsize.x;x++) {
    for (y=0;y<map->mapsize.y;y++) {
      if (map->mapprob[x][y]>0.7) {
	map->calc[x][y] = 0.0;
	abort = FALSE;
	for (vx=x-ADDB;vx<x+ADDB && !abort;vx++) {
	  for (vy=y-ADDB;vy<y+ADDB && !abort;vy++) {
	    if ( vx>=0 && vx<map->mapsize.x &&
		 vy>=0 && vy<map->mapsize.y ) {
	      if (map->mapsum[vx][vy]==0) {
		map->calc[x][y] = 1.0;
		abort = TRUE;
	      }
	    }
	  }
	}
      } else {
	map->calc[x][y] = 0.0;
      }
    }
  }
  
}

logtools_bounding_box_t
compute_laser_bounding_box( logtools_rpos2_t pos, 
			    logtools_lasersens2_data_t lsens,
			    double laser_max_range )
{
  int                   i;
  logtools_vector2_t    abspt, min,max;
  logtools_rmove2_t     nomove = {0.0, 0.0, 0.0};
  logtools_bounding_box_t         bbox;
  
  min.x =  MAXDOUBLE;    min.y =  MAXDOUBLE;
  max.x = -MAXDOUBLE;    max.y = -MAXDOUBLE;
  
  for (i=0;i<lsens.laser.numvalues;i++) {
    if (lsens.laser.val[i]<=laser_max_range) {
      abspt = logtools_compute_laser_points( pos, lsens.laser.val[i],
					     nomove, lsens.laser.angle[i] );
      if (abspt.x<min.x) min.x = abspt.x;
      if (abspt.y<min.y) min.y = abspt.y;
      if (abspt.x>max.x) max.x = abspt.x;
      if (abspt.y>max.y) max.y = abspt.y;
    }
  }
  bbox.min = min;
  bbox.max = max;
  return(bbox);
}


void
mark_map_cell( logtools_ivector2_t pos, MAP2 * map )
{
  int x, y;
  for (x=pos.x-2;x<pos.x+3;x++) {
    for (y=pos.y-2;y<pos.y+3;y++) {
      if ( x>=0 && x<map->mapsize.x &&
	   y>=0 && y<map->mapsize.y ) {
	map->calc[x][y] = -1.0;
      }
    }
  }
}

double
get_map_val( logtools_ivector2_t pos, MAP2 map )
{
  double val;
  if ( pos.x>=0 && pos.x<map.mapsize.x &&
       pos.y>=0 && pos.y<map.mapsize.y ) {
    if (map.mapsum[pos.x][pos.y]>0) {
      val = (double) (map.mapprob[pos.x][pos.y]); 
    } else {
      val = EPSILON;
    }
  } else {
    val = EPSILON;
  }
  if (val>=1.0) 
    return(1.0-EPSILON);
  else
    return(val);
}

double
get_map_val3( logtools_ivector2_t pos, MAP2 map )
{
  double val, value;
  if ( pos.x>=0 && pos.x<map.mapsize.x &&
       pos.y>=0 && pos.y<map.mapsize.y ) {
    if (map.mapsum[pos.x][pos.y]>0) {
      value = (double) (map.mapprob[pos.x][pos.y]); 
      if (value>0.2)
	val = 0.99;
      else
	val = 0.70;
    } else {
      val = 0.80;
    }
  } else {
    val = 0.80;
  }
  return(val);
}

double
get_map_val2( logtools_ivector2_t pos, MAP2 map )
{
  double val;
  if ( pos.x>=0 && pos.x<map.mapsize.x &&
       pos.y>=0 && pos.y<map.mapsize.y ) {
    if (map.mapsum[pos.x][pos.y]>0) {
      val = EPSILON + (double) (map.mapprob[pos.x][pos.y]); 
    } else {
      val = 1.0-EPSILON;
    }
  } else {
    val = 1.0-EPSILON;
  }
  if (val>=1.0) 
    return(1.0-EPSILON);
  else
    return(val);
}

double
stretch_function( double x )
{
  return( (settings.max_likelihood-settings.min_likelihood) * x +
	  settings.min_likelihood );
}

double
get_beam_prob( logtools_rpos2_t pos, double val, double angle,
	       double max_range, MAP2 * map )
{
  logtools_ivector2_t              end;
  logtools_vector2_t               abspt;
  logtools_rmove2_t                nomove = {0.0, 0.0, 0.0};
  
  if (val>=max_range) {
    abspt = logtools_compute_laser_points( pos, max_range-20.0, nomove, angle );
    map_pos_from_vec2( abspt, map, &end ); 
    mark_map_cell( end, map );
    if ( end.x>=0 && end.x<map->mapsize.x &&
	 end.y>=0 && end.y<map->mapsize.y ) {
      /* maxrange and in map and previous seen */
      if (map->mapsum[end.x][end.y]>0) {
	return(stretch_function(1.0-map->mapprob[end.x][end.y]));
      } else {
	return(settings.unknown_likelihood); 
      }
    } else {
      /* maxrange and point out of map */
      return(settings.unknown_likelihood); 
    }
  } else {
    /* no maxrange */
    abspt = logtools_compute_laser_points( pos, val,  nomove, angle );
    map_pos_from_vec2( abspt, map, &end );
    mark_map_cell( end, map );
    if ( end.x>=0 && end.x<map->mapsize.x &&
	 end.y>=0 && end.y<map->mapsize.y ) {
      /* no maxrange and in map and previous seen */
      if (map->mapsum[end.x][end.y]>0) {
	return(stretch_function(map->mapprob[end.x][end.y]));
      } else {
	return(settings.unknown_likelihood); 
      }
    } else {
      /* maxrange and point out of map */
      return(settings.unknown_likelihood); 
    }
  }
}

double
compute_scan_probability( MAP2 * map, logtools_rpos2_t pos, 
			  logtools_lasersens2_data_t lsens,
			  double max_range, double max_usable )
{
  static int                   first_time = TRUE; 
  static logtools_grid_line_t  line;
  static int                   max_num_linepoints = 0;
  int                          i, j, unknown;
  logtools_ivector2_t          start, end, uknw;
  logtools_vector2_t           abspt, startv, endv;
  logtools_rmove2_t            nomove = {0.0, 0.0, 0.0};
  double                       v, val = 0.0, prob, multprob;
  
  if (first_time) {
    max_num_linepoints =
      10 * ( max_range / map->resolution );
    line.grid = (logtools_ivector2_t *) malloc( max_num_linepoints * sizeof(logtools_ivector2_t) );
    first_time = FALSE;
  }

  prob = 1.0; 
  for (j=0;j<lsens.laser.numvalues;j+=2) {
    if (lsens.laser.val[j] <= max_usable ) {
      if (0) {
	if (lsens.laser.val[j] >= max_range ) {
	    abspt = logtools_compute_laser_points( pos, max_range,
						   nomove, lsens.laser.angle[j] );
	    map_pos_from_vec2( abspt, map, &end );
	} else {
	  abspt = logtools_compute_laser_points( pos, lsens.laser.val[j],
						 nomove, lsens.laser.angle[j] );
	  map_pos_from_vec2( abspt, map, &end );
	}
	map_pos_from_rpos( pos, map, &start );
	grid_line( start, end, &line );
	multprob = 1.0;
	if (line.numgrids>0) {
	  unknown = FALSE;
	  for (i=0;i<line.numgrids;i++) {
	    
	    if ( line.grid[i].x>=0 && line.grid[i].x<map->mapsize.x &&
		 line.grid[i].y>=0 && line.grid[i].y<map->mapsize.y ) {
	      
	      if (map->mapsum[line.grid[i].x][line.grid[i].y]==0) {
		/* unknown */
		if (!unknown) {
		  uknw = line.grid[i];
		  unknown = TRUE;
		}
		if (i==line.numgrids-1) {
		  startv.x = uknw.x * map->resolution;
		  startv.y = uknw.y * map->resolution;
		  endv.x   = line.grid[i].x * map->resolution;
		  endv.y   = line.grid[i].y * map->resolution;
		  /* logprob +=
		     log10(prob_unknown_space(vector2_distance(startv, endv),1));*/
		  multprob *=
		    prob_unknown_space(logtools_vector2_distance(startv, endv),1);
		}
	      } else {
		/* known */
		if (unknown) {
		  unknown = FALSE;
		  startv.x = uknw.x * map->resolution;
		  startv.y = uknw.y * map->resolution;
		  endv.x   = line.grid[i].x * map->resolution;
		  endv.y   = line.grid[i].y * map->resolution;
		  /* logprob +=
		     log10(prob_unknown_space(vector2_distance(startv,endv),0)); */
		  multprob *=
		    prob_unknown_space(logtools_vector2_distance(startv, endv),0);
		}
		if (i<line.numgrids-1) {
		  multprob *= (1.0-get_map_val( line.grid[i], *map ));
		  /* logprob += log10((1.0-get_map_val( line.grid[i], *map ))); */
		} else {
		  if (1) {
		    if (lsens.laser.val[j] >= max_range ) {
		      multprob *= (1.0-get_map_val( line.grid[i], *map ));
		      /* logprob +=
			 log10((1.0-get_map_val( line.grid[i], *map ))); */
		    } else {
		    v = get_map_val2( line.grid[i], *map  );
		    multprob *= v;
		    /* logprob += log10(v); */
		    }
		  }
		}
	      }
	    }
	  }
	}
	//      logprob += log10(multprob);
	val = (1.0-settings.min_likelihood)*multprob+settings.min_likelihood;
      //      printf( "(beamprob:%d-%f-%f)", j, multprob, val );
      } else {
	val = log(get_beam_prob( pos, lsens.laser.val[j],
				 lsens.laser.angle[j], max_range, map ));
	prob += val;
      }
    }
  }
  return(exp(prob));
}


double
compute_beam_prob( MAP2 * map, logtools_rpos2_t pos, 
		   double length, double max_range,
		   double * prob1, double * prob2 )
{
  static int                    first_time = TRUE; 
  static logtools_grid_line_t   line;
  static int                    max_num_linepoints = 0;
  int                           i;
  logtools_ivector2_t           start, end;
  logtools_vector2_t            abspt;
  logtools_rmove2_t             nomove = {0.0, 0.0, 0.0};
  double                        lprob = 0.0;
  
  if (first_time) {
    max_num_linepoints =
      10 * ( max_range / map->resolution );
    line.grid = (logtools_ivector2_t *) malloc( max_num_linepoints * 
						sizeof(logtools_ivector2_t) );
    first_time = FALSE;
  }

  abspt = logtools_compute_laser_points( pos, length, nomove, 0.0 );
  map_pos_from_vec2( abspt, map, &end );
  map_pos_from_rpos( pos, map, &start );

  grid_line( start, end, &line );
  if (line.numgrids>0) {
    for (i=0;i<line.numgrids;i++) {
      if ( line.grid[i].x<0 || line.grid[i].x>=map->mapsize.x ||
	   line.grid[i].y<0 || line.grid[i].y>=map->mapsize.y) {
	   //	   map->mapsum[line.grid[i].x][line.grid[i].y]==0 ) {
      	return(FALSE);
      }
      if (i==line.numgrids-1) {
	*prob1 = lprob + log(get_map_val( line.grid[i], *map ));
	*prob2 = lprob + log(1.0-get_map_val( line.grid[i], *map ));
	map->calc[line.grid[i].x][line.grid[i].y] = 1;
      } else {
	lprob += log(1.0-get_map_val( line.grid[i], *map ));
	map->calc[line.grid[i].x][line.grid[i].y] = 1;
      }
    }
  } else {
    return( FALSE );
  }
  return(TRUE);
}

int
intersect_bboxes( logtools_bounding_box_t box1, logtools_bounding_box_t box2 )
{
  if (box1.min.x<=box2.min.x) {
    /* box1.min.x is smaller that box2 */
    if (box1.max.x>box2.min.x) {
      /* intersection in x */
      if (box1.min.y<=box2.min.y) {
	/* box1.min.y is smaller that box2 */
	if (box1.max.y>box2.min.y) {
	  /* intersection in y */
	  return(1);
	} else {
	  return(0);
	}
      } else {
	/* box2.min.y is smaller that box1 */
	if (box2.max.y>=box1.min.y) {
	  /* intersection in y */
	  return(1);
	} else {
	  return(0);
	}
      }
    } else {
      return(0);
    }
  } else {
    /* box2.min.x is smaller that box1 */
    if (box2.max.x>=box1.min.x) {
      /* intersection in x */
      if (box1.min.y<=box2.min.y) {
	/* box1.min.y is smaller that box2 */
	if (box1.max.y>box2.min.y) {
	  /* intersection in y */
	  return(1);
	} else {
	  return(0);
	}
      } else {
	/* box2.min.y is smaller that box1 */
	if (box2.max.y>=box1.min.y) {
	  /* intersection in y */
	  return(1);
	} else {
	  return(0);
	}
      }
    } else {
      return(0);
    }
  }
  return(0);
}

void
write_map( MAP2 map, char *filename )
{
  int                 ok = TRUE;
  int                 i, j, idx;

  Image             * image;
  ImageInfo           image_info;
  double              c = 0.0;

#if defined MagickLibVersion && MagickLibVersion >= 0x500
  ExceptionInfo       exception;
  double            * pixel;
#else
  float             * red, * green, * blue, * opacity;
#endif

#if defined MagickLibVersion && MagickLibVersion >= 0x500
  fprintf( stderr, "alloc memory of pixel map (%d bytes) ... ",
	   map.mapsize.x * map.mapsize.y * 3 * sizeof(double) );
  if ( (pixel = (double *) malloc(map.mapsize.x * map.mapsize.y *
				  3 * sizeof(double)))==NULL )
    ok = FALSE;
  fprintf( stderr, "%s\n", ok?"yes":"no" );
#else
  fprintf( stderr, "alloc memory of pixel map (%d bytes) ... ",
	   map.mapsize.x * map.mapsize.y * 3 * sizeof(float) );
  if ( (red = (float *) malloc(map.mapsize.x * map.mapsize.y *
			       sizeof(float)))==NULL )
    ok = FALSE;
  fprintf( stderr, "red: %s,", ok?"yes":"no" );
  if ( (green = (float *) malloc(map.mapsize.x * map.mapsize.y *
				 sizeof(float)))==NULL )
    ok = FALSE;
  fprintf( stderr, "green: %s,", ok?"yes":"no" );
  if ( (blue = (float *) malloc(map.mapsize.x * map.mapsize.y *
				sizeof(float)))==NULL )
    ok = FALSE;
  fprintf( stderr, "blue: %s,", ok?"yes":"no" );
  if ( (opacity = (float *) malloc(map.mapsize.x * map.mapsize.y *
				   sizeof(float)))==NULL )
    ok = FALSE;
  fprintf( stderr, "opacity: %s,", ok?"yes":"no" );
#endif
  for (i=0;i<map.mapsize.x;i++) {
    for (j=0;j<map.mapsize.y;j++) {
      idx = j*map.mapsize.x+i;
      c = 1.0-map.mapprob[i][map.mapsize.y-j-1];
      if (map.mapsum[i][map.mapsize.y-j-1]>0) {
	if (c<0.0)
	  c = 0.0;
#if defined MagickLibVersion && MagickLibVersion >= 0x500
	pixel[idx*3]     = c;
	pixel[idx*3+1]   = c;
	pixel[idx*3+2]   = c;
#else
	red[idx]     = c;
	green[idx]   = c;
	blue[idx]    = c;
	opacity[idx] = 1.0;
#endif
      } else {
#if defined MagickLibVersion && MagickLibVersion >= 0x500
	pixel[idx*3]     = 200.0/255.0;
	pixel[idx*3+1]   = 200.0/255.0;
	pixel[idx*3+2]   = 255.0/255.0;
#else
	red[idx]     = 200.0/255.0;
	green[idx]   = 200.0/255.0;
	blue[idx]    = 255.0/255.0;
	opacity[idx] = 1.0;
#endif
      }
    }
  }
#if defined MagickLibVersion && MagickLibVersion >= 0x430
  GetExceptionInfo(&exception);
#endif
  GetImageInfo(&image_info);
  fprintf( stderr, "create image of map ... " );
#if defined MagickLibVersion && MagickLibVersion >= 0x430
  image = ConstituteImage ( map.mapsize.x, map.mapsize.y, "RGB",
			    DoublePixel, pixel, &exception );
  if (image == (Image *) NULL) {
    fprintf( stderr, "ERROR: no memory!!!\n" );
    exit(1);
  }
#else
#if defined MagickLibVersion && MagickLibVersion >= 0x420
  image = CreateImage( map.mapsize.x, map.mapsize.y,
		       red, green, blue, opacity );
#else
  image = CreateImage( map.mapsize.x, map.mapsize.y, "RGB",
		       red, green, blue, NULL );
  if (image == (Image *) NULL) {
    fprintf( stderr, "ERROR: no memory!!!\n" );
    exit(1);
  }
#endif
#endif
  strcpy( image->filename, filename );
  fprintf( stderr, "done\n" );
  fprintf( stderr, "write image in file %s ... ", filename );
  WriteImage( &image_info, image );
  fprintf( stderr, "done\n" );
  DestroyImage(image);
#if defined MagickLibVersion && MagickLibVersion >= 0x430
  DestroyConstitute();
  free(pixel);
#else
  free(red);
  free(green);
  free(blue);
#endif
  

}
