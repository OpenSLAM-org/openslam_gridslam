#include "map2d.h"

logtools_vector2_t
map2d_compute_laser_abs_point( logtools_rpos2_t rpos, double val,
			       logtools_rmove2_t offset, double angle )
{
  logtools_vector2_t abspt;
  abspt.x =
    rpos.x + 
    cos( angle+offset.rotation+rpos.o ) * val;
  abspt.y =
    rpos.y + 
    sin( angle+offset.rotation+rpos.o ) * val;
  return(abspt);
}

logtools_laser_coord2_t
map2d_compute_laser2d_coord_with_offset( logtools_lasersens2_data_t lsens,
					 int i )
{
  double        val;
  logtools_laser_coord2_t  coord = { {0.0,0.0},{0.0,0.0},0,0 };
  logtools_rpos2_t         rpos, npos;

  rpos  = logtools_rpos2_with_movement2( lsens.estpos,
					 lsens.laser.offset );
  npos.x = 0.0;
  npos.y = 0.0;
  npos.o = 0.0;

  val = lsens.laser.val[i];
  coord.relpt = logtools_compute_laser_points( npos, val,
					       lsens.laser.offset,
					       lsens.laser.angle[i] );
  coord.abspt = logtools_compute_laser_points( rpos, val,
					       lsens.laser.offset,
					       lsens.laser.angle[i] );
  return(coord);
}

logtools_laser_coord2_t
map2d_compute_laser2d_coord( logtools_lasersens2_data_t lsens, int i )
{
  static double        val;
  static logtools_laser_coord2_t  coord;
  static logtools_rpos2_t         npos;
  static logtools_rmove2_t        nomove = {0.0, 0.0, 0.0};
  
  npos.x = 0.0;
  npos.y = 0.0;
  npos.o = 0.0;

  val = lsens.laser.val[i];
  coord.relpt = logtools_compute_laser_points( npos, val,
					       nomove,
					       lsens.laser.angle[i] );
  coord.abspt = logtools_compute_laser_points( lsens.estpos, val,
					       nomove,
					       lsens.laser.angle[i] );
  return(coord);
}

void
compute_laser2d_points( logtools_log_data_t *rec, int laserID )
{
  int i, j;
  fprintf( stderr, "compute points ...\n" );
  for (j=0;j<rec->numlaserscans;j++) {
    if (rec->lsens[j].id == laserID) {
      if (rec->lsens[j].coord==NULL) {
	rec->lsens[j].coord =
	  (logtools_laser_coord2_t *) malloc( rec->lsens[j].laser.numvalues *
					      sizeof(logtools_laser_coord2_t) );
	for (i=0;i<rec->lsens[j].laser.numvalues;i++) {
	  rec->lsens[j].coord[i] = map2d_compute_laser2d_coord(rec->lsens[j], i);
	}
      }
    }
  }
}

logtools_vector2_t
compute_rel_coord2_with_offset( logtools_lasersens2_data_t lsens, int i,
				logtools_rmove2_t offset )
{
  double angle, val, rot;
  logtools_vector2_t origin;
  logtools_vector2_t relpt;
  angle = lsens.laser.angle[i];
  val   = lsens.laser.val[i];
  origin.x = lsens.laser.offset.forward  + offset.forward;
  origin.y = lsens.laser.offset.sideward + offset.sideward;
  rot      = lsens.laser.offset.rotation + offset.rotation;
  if (val>settings.local_map_max_range)
    val = settings.local_map_max_range;
  relpt.x =
    origin.x + cos( (angle+rot) ) * val;
  relpt.y =
    origin.y + sin( (angle+rot) ) * val;
  return(relpt);
}

int
minimal_rpos_diff( logtools_rpos2_t pos1, logtools_rpos2_t pos2, 
		   double pos_diff_min_dist, 
		   double pos_diff_min_rot )
{
  logtools_vector2_t v1, v2;
  v1.x = pos1.x;    v1.y = pos1.y;
  v2.x = pos2.x;    v2.y = pos2.y;
  if ( logtools_vector2_distance(v1,v2) > pos_diff_min_dist )
    return(TRUE);
  if ( compute_orientation_diff(pos1.o,pos2.o) > pos_diff_min_rot ) 
    return(TRUE);
  return(FALSE);
}

int
minimal_rmove_diff( logtools_rmove2_t move,
		    double pos_diff_min_dist, 
		    double pos_diff_min_rot )
{
  logtools_vector2_t v1;
  v1.x = move.forward;    v1.y = move.sideward;
  if ( logtools_vector2_length(v1) > pos_diff_min_dist )
    return(TRUE);
  if ( move.rotation > pos_diff_min_rot ) 
    return(TRUE);
  return(FALSE);
}

#define BOX_MAX_SIZE   600.0
#define BOX_ADD_SIZE   0.0
#define TIME_HIST      200
#define MIN_VEL        20.0

int
isInBox( logtools_vector2_t p, logtools_vector2_t ll, logtools_vector2_t ur )
{
  double w, h, cx, cy;
  w = ur.x - ll.x;
  h = ur.y - ll.y;
  if (w>BOX_MAX_SIZE || h>BOX_MAX_SIZE) {
    cx = ll.x + w/2.0;
    cy = ll.y + h/2.0;
    if ( p.x > cx-(BOX_MAX_SIZE/2.0) &&
	 p.x < cx+(BOX_MAX_SIZE/2.0) &&
	 p.y > cy-(BOX_MAX_SIZE/2.0) &&
	 p.y < cy+(BOX_MAX_SIZE/2.0) )
      return(1);
    else
      return(0);
  } else {
    if ( p.x > ll.x-BOX_ADD_SIZE &&
	 p.x < ur.x+BOX_ADD_SIZE &&
	 p.y > ll.y-BOX_ADD_SIZE &&
	 p.y < ur.y+BOX_ADD_SIZE )
      return(1);
    else
      return(0);
  }
}

void
remove_rear_scans( logtools_log_data_t * rec, int laserID )
{
  int    i, fstart = 0, remove = 0;
  
  fprintf( stderr, "remove all rear scans\n" );
  for (i=0; i<rec->numlaserscans;i++) {
    if (rec->lsens[i].id == laserID) {
      if (i!=fstart) {
	rec->lsens[fstart] = rec->lsens[i];
      }
      fstart++;
    } else {
      remove++;
    }
  }
  fprintf( stderr, "found    %d front scans\n", fstart );
  fprintf( stderr, "removed  %d rear  scans\n", remove );
  rec->numlaserscans = fstart;
}

void
save_rec2_movements( logtools_log_data_t rec, REC2_MOVEMENTS * save, int laserID )
{
  int i, cnt;
  logtools_rmove2_t nomove;
  save->nummovements = rec.numlaserscans;
  save->pos   = (logtools_rpos2_t *) malloc( rec.numlaserscans * sizeof(logtools_rpos2_t) );
  save->move  = (logtools_rmove2_t *) malloc( rec.numlaserscans * sizeof(logtools_rmove2_t) );
  cnt = -1;
  for (i=0; i<rec.numlaserscans;i++) {
    save->pos[i] = rec.lsens[i].estpos;
  }
  nomove.forward   = 0.0;
  nomove.sideward  = 0.0;
  nomove.rotation  = 0.0;
  for (i=0; i<rec.numlaserscans;i++) {
    if (rec.lsens[i].id == laserID) {
      if (cnt != -1) {
	save->move[i] = logtools_movement2_between_rpos2( save->pos[cnt],
							  save->pos[i] );
      } else {
	save->move[i] = nomove;
      }
      cnt = i;
    }
  }
}

void
add_noise( logtools_log_data_t * rec, REC2_MOVEMENTS orig, int laserID )
{
  int        i;
  logtools_rmove2_t     move, noise, nmove;
  double     trans_factor, rot_factor;
  
  for (i=1; i<rec->numlaserscans;i++) {
    if (rec->lsens[i].id == laserID) {
      srand(settings.random_number);
      
      move = orig.move[i];
      
      if (settings.noise_type == GAUSS_NOISE) {
	noise.forward  = random_gauss();
	noise.sideward = random_gauss();
	noise.rotation = deg2rad(random_gauss());
	trans_factor   = settings.add_noise_val;
	rot_factor     = settings.add_noise_val;
      } else {
	noise.forward = rand()/(double) RAND_MAX;
	if (rand()%2==1)
	  noise.forward *= -1.0;
	noise.sideward = rand()/(double) RAND_MAX;
	if (rand()%2==1)
	  noise.sideward *= -1.0;
	noise.rotation = rand()/(double) RAND_MAX;
	if (rand()%2==1)
	  noise.rotation *= -1.0;
	trans_factor =
	  ( settings.add_noise_val *
	    sqrt( move.forward*move.forward + move.sideward*move.sideward ) ) /
	  sqrt( noise.forward*noise.forward + noise.sideward*noise.sideward );
	rot_factor =
	  ( settings.add_noise_val * fabs( move.rotation ) ) /
	  fabs( noise.rotation );
      }
      
      nmove.forward   = move.forward  + trans_factor * noise.forward;
      nmove.sideward  = move.sideward + trans_factor * noise.sideward;
      nmove.rotation  = move.rotation + rot_factor   * noise.rotation;
      
      rec->lsens[i].estpos =
	logtools_rpos2_with_movement2( rec->lsens[i-1].estpos, nmove );
    }
  }
}

void
compute_laser2d_bbox( logtools_lasersens2_data_t *lsens )
{ 
  int i;
  
  logtools_vector2_t min,max;
  min.x = MAXDOUBLE;
  min.y = MAXDOUBLE;
  max.x = -MAXDOUBLE;
  max.y = -MAXDOUBLE;
  for (i=0;i<lsens->laser.numvalues;i++) {
    if (lsens->laser.val[i]<settings.local_map_max_range) {
      if (lsens->coord[i].abspt.x<min.x)
	min.x = lsens->coord[i].abspt.x;
      if (lsens->coord[i].abspt.y<min.y)
	min.y = lsens->coord[i].abspt.y;
      if (lsens->coord[i].abspt.x>max.x)
	max.x = lsens->coord[i].abspt.x;
      if (lsens->coord[i].abspt.y>max.y)
	max.y = lsens->coord[i].abspt.y;
    }
  }
  lsens->bbox.min = min;
  lsens->bbox.max = max;
}

logtools_bounding_box_t
laser2d_bbox( int numvalues, float *val, logtools_laser_coord2_t *coord )
{ 
  static logtools_bounding_box_t   bbox;
  static logtools_vector2_t        min, max;
  int i;
  
  min.x = MAXDOUBLE;  min.y = MAXDOUBLE;
  max.x = -MAXDOUBLE; max.y = -MAXDOUBLE;
  
  for (i=0;i<numvalues;i++) {
    if (val[i]<settings.local_map_max_range) {
      if (coord[i].abspt.x<min.x)
	min.x = coord[i].abspt.x;
      if (coord[i].abspt.y<min.y)
	min.y = coord[i].abspt.y;
      if (coord[i].abspt.x>max.x)
	max.x = coord[i].abspt.x;
      if (coord[i].abspt.y>max.y)
	max.y = coord[i].abspt.y;
    }
  }
  bbox.min = min;
  bbox.max = max;
  return(bbox);
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

