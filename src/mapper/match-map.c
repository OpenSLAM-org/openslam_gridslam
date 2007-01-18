#include "map2d.h"

double compute_orientation_diff( double start, double end );

double
get_map_val( logtools_ivector2_t pos, MAP2 map )
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

int
get_map_sum( logtools_ivector2_t pos, MAP2 map )
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
get_map_hit( logtools_ivector2_t pos, MAP2 map )
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
probability_between_move_olds( logtools_rmove2_t move1, logtools_rmove2_t move2 )
{
  logtools_vector2_t v1, v2;
  double mval1 = 50.0;
  double mval2 = deg2rad(20.0);
  
  double val1, val2, ret;

  v1.x = move1.forward;
  v1.y = move1.sideward;
  v2.x = move2.forward;
  v2.y = move2.sideward;

  val1   = logtools_vector2_distance( v1, v2 );
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
probability_between_moves( logtools_rmove2_t move1, logtools_rmove2_t move2 )
{
  double sum = 0.0;
  sum += log( EPSILON +
	      logtools_gauss_function( fabs( move1.forward-move2.forward ),
				       0, settings.motion_model.forward ));
  sum += log( EPSILON +
	      logtools_gauss_function( fabs( move1.sideward-move2.sideward ),
				       0, settings.motion_model.sideward));
  sum += log( EPSILON +
	      logtools_gauss_function( fabs( compute_orientation_diff( move1.rotation,
								       move2.rotation )),
			      0, settings.motion_model.rotation ));
  return( sum );
}

int
compute_rmap_pos_from_vec2( logtools_vector2_t vec, MAP2 map, logtools_ivector2_t *v )
{
  v->x = (int) (map.center.x + (vec.x/(double)map.resolution));
  v->y = (int) (map.center.y + (vec.y/(double)map.resolution));
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

double
ivector2_distance( logtools_ivector2_t p1, logtools_ivector2_t p2 )
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
probability_with_move( MAP2 map,
		       logtools_lasersens2_data_t data,
		       logtools_rmove2_t move,
		       logtools_rmove2_t odo_move,
		       double *laserprob )
{
  int       i;
  logtools_vector2_t   pt;
  logtools_ivector2_t  mvec;
  logtools_rpos2_t     rpos;
  logtools_rpos2_t     npos = {0.0, 0.0, 0.0};
  static logtools_rmove2_t nomove = {0.0, 0.0, 0.0};

  double  bprob, prob = 0.0;
  
  rpos = logtools_rpos2_with_movement2( npos, move );

  for (i=0;i<data.laser.numvalues;i++) {
    if (data.laser.val[i]<settings.local_map_max_range) {
      pt = logtools_compute_laser_points( rpos,
					  data.laser.val[i],
					  nomove,
					  data.laser.angle[i] );
      compute_rmap_pos_from_vec2( pt, map, &mvec );
      bprob = log(get_map_val( mvec, map ));
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
probability_with_pos( MAP2 map, logtools_lasersens2_data_t data,
		      logtools_rpos2_t rpos,
		      logtools_rmove2_t move, logtools_rmove2_t odo_move )
{
  static logtools_rmove2_t    nomove = {0.0, 0.0, 0.0};
  int                         i;
  logtools_vector2_t          pt;
  logtools_ivector2_t         mvec;
  double                      bprob, prob = 0.0;
  
  for (i=0;i<data.laser.numvalues;i++) {
    if (data.laser.val[i]<settings.local_map_max_range) {
      pt = logtools_compute_laser_points( rpos,
					  data.laser.val[i],
					  nomove,
					  data.laser.angle[i] );
      compute_rmap_pos_from_vec2( pt, map, &mvec );
      
      bprob = log(get_map_val( mvec, map ));
    } else {
      bprob = log(settings.local_map_std_val);
    }
    prob += bprob;
  }
  prob += probability_between_moves( move, odo_move );
  
  return( prob );
}

logtools_rmove2_t
compute_test_move( logtools_rmove2_t smove, int nummove, int stepsize )
{
  logtools_rmove2_t move = smove;
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

logtools_rmove2_t
fit_data_in_local_map( MAP2 map, logtools_lasersens2_data_t *data,
		       logtools_rmove2_t movement )
{
  int i, l;
  int fitting = TRUE;
  logtools_rmove2_t bmove, pmove, tmove;
  double bprob, pprob, prob, laserprob;

  int loop = 0, adjusting = TRUE;

  bprob = probability_with_move( map, *data, movement, movement, &laserprob );

  pmove = bmove = movement;
  bprob = -MAXFLOAT;

  l = 0;


  while( adjusting ) {

    loop    = 0;
    fitting = TRUE;
    
    while( fitting ) {
      
      pprob = bprob;
      for (i=0; i<6; i++) {
	tmove = compute_test_move( bmove, i, loop );
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
    
    if (l>0)
      adjusting = FALSE;
    
  } /* end while( adjust ) */
  
  return(bmove);
}
