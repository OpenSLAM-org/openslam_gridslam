#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <sys/types.h>
#include <sys/time.h>
#include <unistd.h>
#include <math.h>
#include <string.h>
#include <ctype.h>
#include <values.h>

#include <navigation/utils.h>
#include "map2d.h"

double
get_map_val( iVECTOR2 pos, MAP2 map )
{
#ifdef SLOW
  fprintf( settings.devNULL, "%d %d -> %d %d\n", pos.x, pos.y,
	  map.mapsize.x,
	  map.mapsize.y );
#endif
  if ( pos.x>=0 && pos.x<map.mapsize.x &&
       pos.y>=0 && pos.y<map.mapsize.y ) {
      return( EPSILON + (double) (map.mapprob[pos.x][pos.y]) ); 
  } else {
      return( EPSILON + settings.local_map_std_val );
  }
  return(0.0);
}

double
get_ray_map_val( iVECTOR2 pos, MAP2 map )
{
  if ( pos.x>=0 && pos.x<map.mapsize.x &&
       pos.y>=0 && pos.y<map.mapsize.y &&
       map.mapsum[pos.x][pos.y]>0 ) {
    return( map.mapprob[pos.x][pos.y] );
  } else {
    return( EPSILON );
  }
  return(EPSILON);
}

int
get_map_sum( iVECTOR2 pos, MAP2 map )
{
  if ( pos.x>=0 && pos.x<map.mapsize.x &&
       pos.y>=0 && pos.y<map.mapsize.y ) {
      return( map.mapsum[pos.x][pos.y] ); 
  } else {
    return( 0 );
  }
  return(0);
}

float
get_map_hit( iVECTOR2 pos, MAP2 map )
{
  if ( pos.x>=0 && pos.x<map.mapsize.x &&
       pos.y>=0 && pos.y<map.mapsize.y ) {
      return( map.maphit[pos.x][pos.y] ); 
  } else {
    return( 0 );
  }
  return(0);
}

double
probability_between_move_olds( RMOVE2 move1, RMOVE2 move2 )
{
  VECTOR2 v1, v2;
  double mval1 = 50.0;
  double mval2 = deg2rad(20.0);
  
  double val1, val2, ret;

  v1.x = move1.forward;
  v1.y = move1.sideward;
  v2.x = move2.forward;
  v2.y = move2.sideward;
  val1   = vector2_distance( v1, v2 );

  val2   = fabs( compute_orientation_diff( move1.rotation,
					   move2.rotation ) );
  if (val1>mval1) {
    val1 = mval1;
  }
  
  if (val2>mval2) {
    val2 = mval2;
  }

  ret = ( ((mval1-val1)/mval1) + ((mval2-val2)/mval2) )/2.0;
  
  return( ret );
}

double
probability_between_moves( RMOVE2 move1, RMOVE2 move2 )
{
  double sum = 0.0;
  sum += log( EPSILON +
	      gauss_function( fabs( move1.forward-move2.forward ),
			      0, settings.motion_model.forward ));
  sum += log( EPSILON +
	      gauss_function( fabs( move1.sideward-move2.sideward ),
			      0, settings.motion_model.sideward));
  sum += log( EPSILON +
	      gauss_function( fabs( compute_orientation_diff( move1.rotation,
							      move2.rotation )),
			      0, settings.motion_model.rotation ));
  return( sum );
}

int
compute_rmap_pos_from_vec2( VECTOR2 vec, MAP2 map, iVECTOR2 *v )
{
  v->x = map.center.x + (int) (vec.x/(double)map.resolution);
  v->y = map.center.y + (int) (vec.y/(double)map.resolution);
  if (v->x<0) {
    return(FALSE);
  } else if (v->x>map.mapsize.x-1) {
    return(FALSE);
  }
  if (v->y<0) {
    return(FALSE);
  } else if (v->y>map.mapsize.y-1) {
    return(FALSE);
  }
  return(TRUE);
}

void
update_dyn_prob_new( LASERSENS2_DATA * data, RMOVE2 move, MAP2 map ) {
  int       i;
  double    m, c;
  VECTOR2   pt;
  iVECTOR2  mvec;
  RPOS2     rpos;
  RPOS2     npos = {0.0, 0.0, 0.0};
  static RMOVE2 nomove = {0.0, 0.0, 0.0};

  rpos = compute_rpos2_with_movement2( npos, move );
	
  for (i=0;i<data->laser.numvalues;i++) {
    if (data->laser.val[i]< settings.local_ray_map_max_range) {
      pt = compute_laser_abs_point( rpos,
				    data->laser.val[i],
				    nomove,
				    data->laser.angle[i] );
      if (compute_rmap_pos_from_vec2( pt, map, &mvec )) {
	m = map.mapprob[mvec.x][mvec.y];
	c = settings.dynprob_prior;
	data->ptracking.hprob[i] = c*(m/(m+((1/c)-1)*(1-m)));
      } else {
	data->ptracking.hprob[i] = settings.dynprob_prior;
      }
    } else {
      data->ptracking.hprob[i] = settings.dynprob_prior;
    }

  }
    
}

void
update_dyn_prob( LASERSENS2_DATA * data, RMOVE2 move, MAP2 map ) {
  int       i;
  VECTOR2   pt;
  iVECTOR2  mvec;
  RPOS2     rpos;
  RPOS2     npos = {0.0, 0.0, 0.0};
  static RMOVE2 nomove = {0.0, 0.0, 0.0};

  rpos = compute_rpos2_with_movement2( npos, move );
	
  for (i=0;i<data->laser.numvalues;i++) {
    pt = compute_laser_abs_point( rpos,
				  data->laser.val[i],
				  nomove,
				  data->laser.angle[i] );
    if (compute_rmap_pos_from_vec2( pt, map, &mvec )) {
      data->ptracking.hprob[i] = 2.0*map.mapprob[mvec.x][mvec.y];
    } else {
      data->ptracking.hprob[i] = settings.local_map_std_val;
    }

  }
  if (data->ptracking.hprob[i]>1.0)
    data->ptracking.hprob[i]=1.0;
    
  /*
  norm = 0.0;
  for (i=0;i<data->laser.numvalues;i++) {
    norm += data->ptracking.hprob[i];
  }
  if (norm > settings.max_dynprob_change) {
    normf = settings.max_dynprob_change / norm;
    for (i=0;i<data->laser.numvalues;i++) {
      data->ptracking.hprob[i] *= normf;
    }
  }
  */
}

double
error( double val, double expect )
{
  double sigma1=40.0;
  double sigma2=80.0;
  if ( fabs(val-expect)<sigma2 &&
       (val<expect || fabs(val-expect)<sigma1) ) {
    return( 1.0- (sigma2-fabs(val-expect))/sigma2);
  } else {
    return( 1.0 );
  }
}

#define APRX 3

double
gauss_approx( double val, double expect, double sigma )
{
  double approx = 2*(1/sqrt(2*M_PI*sigma*sigma))/(APRX*sigma);
  double start  = expect-APRX*sigma;
  if (val<start) {
    return( 2*EPSILON );
  } else  if (val>expect) {
    return( EPSILON );
  } else {
    return( EPSILON+(val-start)*approx );
  }
}

void
dynamic_prob_from_global_map( LASERSENS2_DATA * data, MAP2 map )
{
  int             j;
  
  iVECTOR2        pt;
  double          mv;
  
  for (j=0;j<data->laser.numvalues;j++) {
    data->ptracking.hprob[j] = settings.dynprob_prior;
    if ( data->laser.val[j] < settings.max_usable_laser_range ) {
      if (map_pos_from_vec2( data->coord[j].abspt, &map, &pt ) &&
	  pt.x>=0 && pt.x<map.mapsize.x &&
	  pt.y>=0 && pt.y<map.mapsize.y ) {
	mv = get_map_val( pt, map );
	data->ptracking.hprob[j] = settings.dynprob_prior*(mv/(mv+((1.0/settings.dynprob_prior)-1)*(1.0-mv)));
      }
    } 
  }
}

double
ivector2_distance( iVECTOR2 p1, iVECTOR2 p2 )
{
  return sqrt( (p1.x-p2.x)*(p1.x-p2.x) +
	       (p1.y-p2.y)*(p1.y-p2.y) );
}

double
compute_beam_log_prob( double expected, double measured )
{
  double val, d = fabs(expected-measured); /* dist in cm */

  if (measured>0.95*settings.local_map_max_range) 
    return(log(0.01));
  if (d>settings.local_map_max_range)
    d = settings.local_map_max_range;
  if (d<200.0) {
    val = (220.0-d)/220.0;
  } else {
    val = 0.01;
  }
  return(log(val));
    
}


double
probability_with_rays_and_move( MAP2 map, LASERSENS2_DATA data,
				RMOVE2 move, RMOVE2 odo_move )
{
  static int       first_time = TRUE;
  static GRID_LINE line;
  int              i, j;
  int              max_num_linepoints;
  VECTOR2          pt1, pt2;
  iVECTOR2         start, end;
  RPOS2            rpos;
  RPOS2            npos = {0.0, 0.0, 0.0};
  static RMOVE2    nomove = {0.0, 0.0, 0.0};
  double           expected, prob = 0.0, bprob = 0;
  
  if (first_time) {
    max_num_linepoints =
      4 * ( settings.local_map_max_range /
	    settings.local_map_resolution );
    line.grid = (iVECTOR2 *) malloc( max_num_linepoints * sizeof(iVECTOR2) );
    first_time = FALSE;
  }
  
  rpos = compute_rpos2_with_movement2( npos, move );
  pt1.x = rpos.x;
  pt1.y = rpos.y;
  compute_rmap_pos_from_vec2( pt1, map, &start );
  
  for (i=0;i<data.laser.numvalues;i++) {

    pt2 = compute_laser_abs_point( rpos,
				   settings.local_map_max_range,
				   nomove,
				   data.laser.angle[i] );
    compute_rmap_pos_from_vec2( pt2, map, &end );
    grid_line( start, end, &line );
    expected = settings.local_map_max_range;
    for (j=0;j<line.numgrids;j++) {
      if (get_map_val( line.grid[j], map ) > LOCAL_MAP_OBSTACLE_PROB) {
	expected = ivector2_distance( start, line.grid[j] ) * map.resolution;
	break;
      }
    }
    if (data.laser.val[i]<settings.local_map_max_range) {
      bprob = compute_beam_log_prob( expected, data.laser.val[i] );
    } else {
      bprob = compute_beam_log_prob( expected,
				     settings.local_map_max_range );
    }
    prob += bprob;
  }
  
  if (settings.local_map_use_odometry != ODO_NOTHING)
    prob += probability_between_moves( move, odo_move );
  
  return( prob );
}

double
probability_with_move( MAP2 map, LASERSENS2_DATA data,
		       RMOVE2 move, RMOVE2 odo_move, double *laserprob )
{
  int       i;
  VECTOR2   pt;
  iVECTOR2  mvec;
  RPOS2     rpos;
  RPOS2     npos = {0.0, 0.0, 0.0};
  static RMOVE2 nomove = {0.0, 0.0, 0.0};

  double  bprob, prob = 0.0;
  
  rpos = compute_rpos2_with_movement2( npos, move );

  for (i=0;i<data.laser.numvalues;i++) {
    if (data.laser.val[i]<settings.local_map_max_range) {
      pt = compute_laser_abs_point( rpos,
				    data.laser.val[i],
				    nomove,
				    data.laser.angle[i] );
      compute_rmap_pos_from_vec2( pt, map, &mvec );
      if (settings.use_local_ray_map || settings.use_global_ray_map) {
	bprob =
	  (data.ptracking.hprob[i] * log(get_map_val( mvec, map )));
      } else {
	bprob =
	  log(get_map_val( mvec, map ));
      }
    } else {
      bprob = log(settings.local_map_std_val);
    }
    prob += bprob;
  }

  *laserprob = prob;
  if (settings.local_map_use_odometry != ODO_NOTHING)
    prob += probability_between_moves( move, odo_move );
  
  return( prob );
}

double
probability_with_pos( MAP2 map, LASERSENS2_DATA data, RPOS2 rpos,
		      RMOVE2 move, RMOVE2 odo_move )
{
  static RMOVE2    nomove = {0.0, 0.0, 0.0};
  int              i;
  VECTOR2          pt;
  iVECTOR2         mvec;
  double           bprob, prob = 0.0;
  
  for (i=0;i<data.laser.numvalues;i++) {
    if (data.laser.val[i]<settings.local_map_max_range) {
      pt = compute_laser_abs_point( rpos,
				    data.laser.val[i],
				    nomove,
				    data.laser.angle[i] );
      compute_rmap_pos_from_vec2( pt, map, &mvec );
      
      if (settings.use_local_ray_map || settings.use_global_ray_map) {
	bprob =
	  data.ptracking.hprob[i] * log(get_map_val( mvec, map ));
      } else {
	bprob =
	  log(get_map_val( mvec, map ));
      }
    } else {
      bprob = log(settings.local_map_std_val);
    }
    prob += bprob;
  }
  prob += probability_between_moves( move, odo_move );
  
  return( prob );
}

RMOVE2
compute_test_move( RMOVE2 smove, int nummove, int stepsize )
{
  RMOVE2 move = smove;
  double div  = pow( 2, stepsize);
  switch( nummove ) {
  case 0:
    move.rotation += ( settings.pos_corr_step_size_rotation / div );
    break;
  case 1:
    move.rotation -= ( settings.pos_corr_step_size_rotation / div );
    break;
  case 2:
    move.sideward += ( settings.pos_corr_step_size_sideward / div );
    break;
  case 3:
    move.sideward -= ( settings.pos_corr_step_size_sideward / div );
    break;
  case 4:
    move.forward  += ( settings.pos_corr_step_size_forward / div) ;
    break;
  case 5:
    move.forward  -= ( settings.pos_corr_step_size_forward / div );
    break;
  default:
    break;
  }
  return(move);
}

RMOVE2
fit_data_in_local_map( MAP2 map, MAP2 ray_map, LASERSENS2_DATA *data,
		       RMOVE2 movement,
		       int num_used_scans __attribute__ ((unused)),
		       int scannr, int loopnr )
{
  int i, l;
  int fitting = TRUE;
  RMOVE2 bmove, pmove, tmove;
  double bprob, pprob, prob, laserprob;

  static double odynprob[MAX_NUM_LASER_VALUES];
  
  int loop = 0, adjusting = TRUE;

  bprob = probability_with_move( map, *data, movement, movement, &laserprob );

  pmove = bmove = movement;
  bprob = -MAXFLOAT;

  if (settings.use_local_ray_map) {
    for (i=0; i<data->laser.numvalues; i++) {
      odynprob[i] = data->ptracking.hprob[i] = 1.0;
    }
  }

  l = 0;


  while( adjusting ) {

    loop    = 0;
    fitting = TRUE;
    
    while( fitting ) {
      
      pprob = bprob;
      for (i=0; i<6; i++) {
	tmove = compute_test_move( bmove, i, loop );
	if (settings.use_local_ray_map) {
	  update_dyn_prob( data, tmove, ray_map );
	}
	prob = probability_with_move( map, *data, tmove,
				      movement, &laserprob  );
	if ( prob>pprob) {
	  pmove  = tmove;
	  pprob = prob;
	}
      }

      if (pprob-bprob>EPSILON) {
	bmove  = pmove;
	bprob = pprob;
      } else if (loop<settings.pos_corr_step_size_loop) {
	loop++;
      } else {
	fitting = FALSE;
      }
      
      
    } /* end while( fitting ) */

    l++;
    
    if (settings.output_statistics) {
      if (0) {
	fprintf( settings.statisticsF,
		 "    %.20f # %f %f %f scan %d loop %s%s%s%d\n",
		 bprob, pmove.forward, pmove.sideward,
		 rad2deg(pmove.rotation), scannr,
		 loopnr<1000? "0":"",
		 loopnr<100? "0":"",
		 loopnr<10? "0":"",
		 loopnr );
      } else {
	fprintf( settings.statisticsF,
		 "    %.20f # laser\n", laserprob );
      }
    }
    
    if (settings.use_local_ray_map) {
      update_dyn_prob( data, bmove, ray_map );
    }
    if (l>0)
      adjusting = FALSE;
    
  } /* end while( adjust ) */
  
  if (settings.dump_probabilities) {
    fprintf( settings.probF, "%.20f\n",
	     bprob );
    fflush( settings.probF );
  }
  return(bmove);
}
