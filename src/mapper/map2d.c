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

VECTOR2
map2d_compute_laser_abs_point( RPOS2 rpos, double val,
			       RMOVE2 offset, double angle )
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
map2d_compute_laser2d_coord_with_offset( LASERSENS2_DATA lsens, int i )
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

LASER_COORD2
map2d_compute_laser2d_coord( LASERSENS2_DATA lsens, int i )
{
  static double        val;
  static LASER_COORD2  coord;
  static RPOS2         npos;
  static RMOVE2        nomove = {0.0, 0.0, 0.0};
  
  npos.x = 0.0;
  npos.y = 0.0;
  npos.o = 0.0;

  val = lsens.laser.val[i];
  coord.relpt = compute_laser_abs_point( npos, val,
					 nomove,
					 lsens.laser.angle[i] );
  coord.abspt = compute_laser_abs_point( lsens.estpos, val,
					 nomove,
					 lsens.laser.angle[i] );
  return(coord);
}

void
compute_laser2d_points( REC2_DATA *rec, int laserID )
{
  int i, j;
  fprintf( stderr, "compute points ...\n" );
  for (j=0;j<rec->numlaserscans;j++) {
    if (rec->lsens[j].id == laserID) {
      if (rec->lsens[j].coord==NULL) {
	rec->lsens[j].coord =
	  (LASER_COORD2 *) malloc( rec->lsens[j].laser.numvalues *
				   sizeof(LASER_COORD2) );
	rec->lsens[j].poly.pt =
	  (VECTOR2 *) malloc( rec->lsens[j].laser.numvalues *
			      sizeof(VECTOR2) );
	rec->lsens[j].poly.numpoints = rec->lsens[j].laser.numvalues;
	for (i=0;i<rec->lsens[j].laser.numvalues;i++) {
	  rec->lsens[j].coord[i] = map2d_compute_laser2d_coord(rec->lsens[j], i);
	  //	  rec->lsens[j].poly.pt[i] = rec->lsens[j].coord[i].abspt;
	}
      }
    }
  }
}

VECTOR2
compute_rel_coord2_with_offset( LASERSENS2_DATA lsens, int i,
				RMOVE2 offset )
{
  double angle, val, rot;
  VECTOR2 origin;
  VECTOR2 relpt;
  angle = lsens.laser.angle[i];
  val   = lsens.laser.val[i];
  origin.x = lsens.laser.offset[i].forward  + offset.forward;
  origin.y = lsens.laser.offset[i].sideward + offset.sideward;
  rot      = lsens.laser.offset[i].rotation + offset.rotation;
  if (val>settings.local_map_max_range)
    val = settings.local_map_max_range;
  relpt.x =
    origin.x + cos( (angle+rot) ) * val;
  relpt.y =
    origin.y + sin( (angle+rot) ) * val;
  return(relpt);
}

int
minimal_rpos_diff( RPOS2 pos1, RPOS2 pos2, 
		   double pos_diff_min_dist, 
		   double pos_diff_min_rot )
{
  VECTOR2 v1, v2;
  v1.x = pos1.x;    v1.y = pos1.y;
  v2.x = pos2.x;    v2.y = pos2.y;
  if ( vector2_distance(v1,v2) > pos_diff_min_dist )
    return(TRUE);
  if ( compute_orientation_diff(pos1.o,pos2.o) > pos_diff_min_rot ) 
    return(TRUE);
  return(FALSE);
}

int
minimal_rmove_diff( RMOVE2 move,
		    double pos_diff_min_dist, 
		    double pos_diff_min_rot )
{
  VECTOR2 v1;
  v1.x = move.forward;    v1.y = move.sideward;
  if ( vector2_length(v1) > pos_diff_min_dist )
    return(TRUE);
  if ( move.rotation > pos_diff_min_rot ) 
    return(TRUE);
  return(FALSE);
}

void
write_sens( FILE  *fp, struct timeval time )
{
  struct tm *actual_date;
  long secs = time.tv_sec;
  actual_date = localtime( &secs );
  fprintf( fp, "@SENS %s%d-%s%d-%d %s%d:%s%d:%s%.6f\n",
	   (actual_date->tm_mday<10)?"0":"",
	   actual_date->tm_mday,
	   (actual_date->tm_mon<10)?"0":"",
	   actual_date->tm_mon,
	   1900+actual_date->tm_year,
	   (actual_date->tm_hour<10)?"0":"",
	   actual_date->tm_hour,
	   (actual_date->tm_min<10)?"0":"",
	   actual_date->tm_min,
	   (actual_date->tm_sec<10)?"0":"",
	   actual_date->tm_sec+( time.tv_usec / 1000000.0 ) );
}

void
check_hprob( REC2_DATA *rec, int laserID )
{
  int                i, j, cnt = 0;
  fprintf( stderr, "check all human probs ... " );
  for (i=0; i<rec->numlaserscans;i++) {
    if (rec->lsens[i].id == laserID &&
	rec->lsens[i].ptracking.hprob==NULL) {
      rec->lsens[i].ptracking.hprob =
	(double *) malloc( rec->lsens[i].laser.numvalues * sizeof(double) );
      for (j=0; j<rec->lsens[i].laser.numvalues; j++) {
	rec->lsens[i].ptracking.hprob[j] = 1.0;
      }
      cnt++;
    }
  }
  fprintf( stderr, "%d uncorrect probsd found\n", cnt );
}

void
convolve_hprob( REC2_DATA *rec, int laserID )
{
  int                i, j, k, kk, hk, nv; 
  GAUSS_KERNEL       kernel;
  double             ksum, kstore[MAX_NUM_LASER_VALUES];

  kernel = compute_gauss_kernel( settings.people_prob_kernel_len );
  for (i=0; i<rec->numlaserscans;i++) {
    if (rec->lsens[i].id == laserID) {
      nv = rec->lsens[i].laser.numvalues;
      for (k=0; k<settings.people_prob_num_convolve; k++) {
	for (j=0;j<nv;j++) {
	  ksum = 0.0;
	  hk   = (settings.people_prob_kernel_len-1)/2;
	  for (kk=0;kk<settings.people_prob_kernel_len;kk++) {
	    ksum += kernel.val[kk] * rec->lsens[i].ptracking.hprob[(nv+j+kk-hk)%nv];
	  }
	  kstore[j] = ksum;
	}
	for (j=0;j<nv;j++) {
	  rec->lsens[i].ptracking.hprob[j] = kstore[j];
	}
      }
    }
  }
}

void
write_script_marker( FILE * fp, int type, int data )
{
  switch(type) {
  case LOOP_START:
    fprintf( fp, "#LOOP-START %d\n\n", data );
    break;
  case LOOP_END:
    fprintf( fp, "#LOOP-END %d\n\n", data );
    break;
  }    
}

void
write_script_entry( FILE *fp, LASERSENS2_DATA lsens, int laserID )
{
  int j;
  if (settings.script_type==SCRIPT) {
    write_sens( fp, lsens.laser.time );
    fprintf( fp, "#ROBOT %.6f %.6f %.6f\n\n",
	     lsens.estpos.x,
	     lsens.estpos.y,
	     rad2deg(lsens.estpos.o) );
    write_sens( fp, lsens.laser.time );
    if (laserID==0) {
      fprintf( fp, "#LASER %d 0:",
	       lsens.laser.numvalues );
    } else {
      fprintf( fp, "#LASER 0 %d:",
	       lsens.laser.numvalues );
    }
    for (j=0;j<lsens.laser.numvalues;j++) {
      fprintf( fp, " %d", (int) lsens.laser.val[j] );
    }
    fprintf( fp, "\n\n" );
    if (settings.use_local_ray_map || settings.use_global_ray_map) {
      if (lsens.ptracking.hprob !=NULL) {
	write_sens( fp, lsens.laser.time );
	fprintf( fp, "#DYNAMIC-PROB %d 0:",
		 lsens.laser.numvalues );
	for (j=0;j<lsens.laser.numvalues;j++) {
	  fprintf( fp, " %.7f", lsens.ptracking.hprob[j] );
	}
	fprintf( fp, "\n\n" );
      }
    }
  } else {
    fprintf( fp, "POS %ld %ld: %.6f %.6f %.6f %.6f %.6f\n",
	     lsens.laser.time.tv_sec,
	     lsens.laser.time.tv_usec,
	     lsens.estpos.x,
	     lsens.estpos.y,
	     rad2deg(lsens.estpos.o),
	     0.0, 0.0 );
    fprintf( fp, "LASER-RANGE %ld %ld %d %d %.1f:",
	     lsens.laser.time.tv_sec,
	     lsens.laser.time.tv_usec,
	     lsens.id,
	     lsens.laser.numvalues,
	     rad2deg(lsens.laser.anglerange) );
    fflush(fp);
    for (j=0;j<lsens.laser.numvalues;j++) {
      fprintf( fp, " %.1f", lsens.laser.val[j] );
    }
    fprintf( fp, "\n" );
    if (settings.use_local_ray_map || settings.use_global_ray_map) {
      if (lsens.ptracking.hprob !=NULL) {
	fprintf( fp, "DYNAMIC-PROB %ld %ld %d %d %.1f:",
		 lsens.laser.time.tv_sec,
		 lsens.laser.time.tv_usec,
		 lsens.id,
		 lsens.laser.numvalues,
		 lsens.laser.anglerange );
	for (j=0;j<lsens.laser.numvalues;j++) {
	  fprintf( fp, " %.7f", lsens.ptracking.hprob[j] );
	}
	fprintf( fp, "\n" );
      }
    }
  }
  fflush(fp);
}

/*
#define BOX_MAX_SIZE   240.0
#define BOX_ADD_SIZE   80.0
#define TIME_HIST      600
#define MIN_VEL        10.0
*/

#define BOX_MAX_SIZE   600.0
#define BOX_ADD_SIZE   0.0
#define TIME_HIST      200
#define MIN_VEL        20.0

int
isInBox( VECTOR2 p, VECTOR2 ll, VECTOR2 ur )
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

int
isPeople( VECTOR2 p, LASERSENS2_DATA lsens,
	  int laserID __attribute__ ((unused)) )
{
  int i;
  for (i=0; i<lsens.ptracking.numpstates; i++) {
    if ( (lsens.ptracking.pstate[i].vel>MIN_VEL) &&
	 isInBox( p,
		  lsens.ptracking.pstate[i].ll,
		  lsens.ptracking.pstate[i].ur ))
      return(1);
  }
  return(0);
}

void
write_data_entry( FILE *fp, LASERSENS2_DATA lsens, int laserID )
{
  int j;
  LASER_COORD2 coord;
  for (j=0;j<lsens.laser.numvalues;j++) {
    if (lsens.id == laserID) { 
      if ( lsens.laser.val[j]<settings.global_map_max_range ) {
	if ( settings.use_people_prob ) {
	  coord = map2d_compute_laser2d_coord(lsens, j);
	  if ( !isPeople(coord.abspt, lsens, laserID) )
	    fprintf( fp, "%.6f %.6f\n", coord.abspt.x, coord.abspt.y ); 
	} else {
	  if ( lsens.ptracking.hprob==NULL ||
	       lsens.ptracking.hprob[j] <
	       settings.max_dynamic_prob ) {
	    coord = map2d_compute_laser2d_coord(lsens, j);
	    fprintf( fp, "%.6f %.6f\n",
		     coord.abspt.x,
		     coord.abspt.y );
	  }
	}
      }
    }
  }
  fflush(settings.outputF);
}

void
remove_rear_scans( REC2_DATA * rec, int laserID )
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
save_rec2_movements( REC2_DATA rec, REC2_MOVEMENTS * save, int laserID )
{
  int i, cnt;
  RMOVE2 nomove;
  save->nummovements = rec.numlaserscans;
  save->pos   = (RPOS2 *) malloc( rec.numlaserscans * sizeof(RPOS2) );
  save->move  = (RMOVE2 *) malloc( rec.numlaserscans * sizeof(RMOVE2) );
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
	      save->move[i] = compute_movement2_between_rpos2( save->pos[cnt],
							       save->pos[i] );
	  } else {
	      save->move[i] = nomove;
	  }
	  cnt = i;
      }
  }
}

void
add_noise( REC2_DATA * rec, REC2_MOVEMENTS orig, int laserID )
{
  int        i;
  RMOVE2     move, noise, nmove;
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
	compute_rpos2_with_movement2( rec->lsens[i-1].estpos, nmove );
    }
  }
}

void
write_experiment_error( int index, RMOVE2 cmove, RMOVE2 lmove, RMOVE2 emove,
			RPOS2 estpos, RPOS2 origpos )
{
  RMOVE2     dmove;
  RPOS2      dpos;
  double     ldiff;
  
  dmove.forward   = cmove.forward  - lmove.forward;
  dmove.sideward  = cmove.sideward - lmove.sideward;
  dmove.rotation  = cmove.rotation - lmove.rotation;

  ldiff  =
    sqrt( dmove.forward  * dmove.forward +
	  dmove.sideward * dmove.sideward );

  fprintf( settings.experimentF,
	   "%d %.8f # error rel trans\n", index, ldiff );
 
  ldiff  =
    fabs(dmove.rotation);
  
  fprintf( settings.experimentF,
	   "%d %.8f # error rel rot\n", index, rad2deg(ldiff) );
   
  dmove.forward   = cmove.forward  - emove.forward;
  dmove.sideward  = cmove.sideward - emove.sideward;
  dmove.rotation  = cmove.rotation - emove.rotation;

  ldiff  =
    sqrt( dmove.forward  * dmove.forward +
	  dmove.sideward * dmove.sideward );

  fprintf( settings.experimentF,
	   "%d %.8f # error est trans\n", index, ldiff );
 
  ldiff  =
    fabs(dmove.rotation);
  
  fprintf( settings.experimentF,
	   "%d %.8f # error est rot\n", index, rad2deg(ldiff) );
   
  dpos.x = estpos.x - origpos.x;
  dpos.y = estpos.y - origpos.y;
  dpos.o = fabs( compute_orientation_diff( estpos.o, origpos.o ) );
  
  ldiff  =
    sqrt( dpos.x * dpos.x +
	  dpos.y * dpos.y );
  
  fprintf( settings.experimentF,
	   "%d %.8f # error abs trans\n", index, ldiff );
 
  ldiff  = dpos.o;
  
  fprintf( settings.experimentF,
	   "%d %.8f # error abs rot\n", index, rad2deg(ldiff) );
  
  fflush(settings.experimentF);
}

void
write_total_error( REC2_DATA rec, REC2_MOVEMENTS orig, int laserID )
{
  int        i, lcnt, lpos;
  RPOS2      dpos;
  RMOVE2     lmove, cmove, dmove;
  double     ldiff1, ldiff2;
  double     lsum1, lsum2;
  
  lsum1     = 0.0;
  lsum2     = 0.0;
  lcnt      = 0;
  lpos      = 0;

  for (i=1; i<rec.numlaserscans;i++) {
    if ( rec.lsens[i].id == laserID ) {
	  
      lmove = compute_movement2_between_rpos2( rec.lsens[lpos].estpos,
					       rec.lsens[i].estpos );
      cmove = compute_movement2_between_rpos2( orig.pos[lpos],
					       orig.pos[i] );
      dmove.forward   = cmove.forward  - lmove.forward;
      dmove.sideward  = cmove.sideward - lmove.sideward;
      dmove.rotation  = cmove.rotation - lmove.rotation;

      ldiff1  =
	sqrt( dmove.forward  * dmove.forward +
	      dmove.sideward * dmove.sideward );
      lsum1  += ldiff1;
      
      ldiff2  = fabs(dmove.rotation);
      lsum2  += ldiff2;
      
      lpos = i;
      lcnt++;
      
    }
  }
  
  fprintf( settings.experimentF,
	   "##################################################\n" );
  fprintf( settings.experimentF,
	   "# total error of %d used scans:\n", lcnt );
  fprintf( settings.experimentF,
	   "%.8f # total error rel diff trans\n", lsum1 );
  fprintf( settings.experimentF,
	   "%.8f # total error rel diff rot\n", rad2deg(lsum2) );
  
  lsum1     = 0.0;
  lsum2     = 0.0;
  lcnt      = 0;
  lpos      = 0;

  for (i=1; i<rec.numlaserscans;i++) {
    if ( rec.lsens[i].id == laserID ) {
	  
      dpos.x = rec.lsens[i].estpos.x - orig.pos[i].x;
      dpos.y = rec.lsens[i].estpos.y - orig.pos[i].y;
      dpos.o = fabs( compute_orientation_diff( rec.lsens[i].estpos.o,
					       orig.pos[i].o ) );

      ldiff1  =
	sqrt( dpos.x * dpos.x +
	      dpos.y * dpos.y );
      lsum1  += ldiff1;
      
      ldiff2  = fabs(dpos.o);
      lsum2  += ldiff2;
      
      lcnt++;
      
    }
  }
  
  fprintf( settings.experimentF,
	   "##################################################\n" );
  fprintf( settings.experimentF,
	   "# total error of %d used scans:\n", lcnt );
  fprintf( settings.experimentF,
	   "%.8f # total error abs diff trans\n", lsum1 );
  fprintf( settings.experimentF,
	   "%.8f # total error abs diff rot\n", rad2deg(lsum2) );
  
  fflush(settings.experimentF);
}

void
compute_statistics( REC2_DATA rec, int laserID )
{
  int        i, j;
  int        scans_total    = 0;
  int        scans_skipped  = 0;
  int        scans_used     = 0;
  int        beams_total    = 0;
  int        beams_maxrange = 0;
  int        beams_used     = 0;
  int        beams_hprob    = 0;
  
  for (i=1; i<rec.numlaserscans;i++) {
    if ( rec.lsens[i].id == laserID ) {
      for (j=0;j<rec.lsens[i].laser.numvalues;j++) {
	if ( rec.lsens[i].laser.val[j]<settings.global_map_max_range ) {
	  if ( rec.lsens[i].ptracking.hprob[j]>=EPSILON ) {
	    beams_hprob++;
	  }
	  beams_used++;
	} else {
	  beams_maxrange++;
	}
      }
      scans_used++;
    } else {
      scans_skipped++;
    }
  }

  scans_total = scans_skipped + scans_used;
  beams_total = beams_maxrange + beams_used;
  
  fprintf( stderr, "##################################################\n" );
  fprintf( stderr, "###########       STATISTICS          ############\n" );
  fprintf( stderr, "##################################################\n" );
  fprintf( stderr, "#\n" );
  if ( scans_total != 0 &&
       beams_total != 0 &&
       beams_used != 0 ) {
    fprintf( stderr, "#   scans total:      %d\n", scans_total );
    fprintf( stderr, "#   scans used:       %d (%.2f%%)\n",
	     scans_used,
	     100.0*(scans_used/(double)scans_total) );
    fprintf( stderr, "#   scans skiped:     %d (%.2f%%)\n",
	     scans_skipped,
	     100.0*(scans_skipped/(double)scans_total) );
    fprintf( stderr, "#\n" );
    fprintf( stderr, "#   beams total:      %d\n", beams_total );
    fprintf( stderr, "#   beams used:       %d (%.2f%%)\n",
	     beams_used,
	     100.0*(beams_used/(double)beams_total) );
    fprintf( stderr, "#   beams max-range:  %d (%.2f%%) of all, " \
	     "(%.2f%%) of used\n",
	     beams_maxrange,
	     100.0*(beams_maxrange/(double)beams_total),
	     100.0*(beams_maxrange/(double)beams_used) );
    fprintf( stderr, "#   beams humanprob:  %d (%.2f%%) of all, " \
	     "(%.2f%%) of used\n",
	     beams_hprob,
	     100.0*(beams_hprob/(double)beams_total),
	     100.0*(beams_hprob/(double)beams_used) );

  } else {
    fprintf( stderr, "#   ERROR: no statistics possible\n" );
  }
  fprintf( stderr, "#\n" );
  fprintf( stderr, "##################################################\n" );
}

void // BOUNDING_BOX2
compute_laser2d_bbox( LASERSENS2_DATA *lsens )
{ 
  int i;
  
  VECTOR2 min,max;
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

BOUNDING_BOX2
laser2d_bbox( int numvalues, double *val, LASER_COORD2 *coord )
{ 
  static BOUNDING_BOX2  bbox;
  static VECTOR2        min, max;
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
intersect_bboxes( BOUNDING_BOX2 box1, BOUNDING_BOX2 box2 )
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

