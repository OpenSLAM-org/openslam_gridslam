#include "map2d.h"


extern logtools_lasersens2_data_t  * current_scan;
extern logtools_rmove2_t           * current_movement;

logtools_log_data_t                 rec;

int                              histcnt;
int                            * history;

double
rpos2_dist( logtools_rpos2_t pos1, logtools_rpos2_t pos2 )
{
  return(
	 sqrt( (pos1.x - pos2.x) * (pos1.x - pos2.x) +
	       (pos1.y - pos2.y) * (pos1.y - pos2.y) )
	 );
}


double
rmove2_length( logtools_rmove2_t move )
{
  return(
	 sqrt( (move.forward * move.forward) +
	       (move.sideward * move.sideward) )
	 );
}

void
plot_robot_path( void )
{
  int i;
  for (i=0; i<histcnt; i++) {
    paint_robot( rec.lsens[history[i]].estpos );
  }
}

void
run_data_file( MAP2D_SETTINGS settings,
	       MAP2 * lmap, MAP2 * gmap )
{
  int                     i, j, h, k, l, hk, tctr;
  int                     e_ctr = 0;
  
  REC2_MOVEMENTS          orig, movements;

  int                     ef, es, er;
  int                     efs = 0, ess = 0, ers = 0;
  logtools_rmove2_t       testmove;
  
  logtools_rpos2_t        npos = { 0.0, 0.0, 0.0 };
  logtools_rpos2_t        last_error_pos = { 0.0, 0.0, 0.0 };
  logtools_rpos2_t        currentpos = { 0.0, 0.0, 0.0 };
  int                     cnt = 0, ct, ctr;
  
  logtools_rmove2_t       lmove, move, bestmove, cmove;

  double              *** error = NULL;
  
  hk = (settings.local_map_kernel_len-1)/2;
  
  if ( settings.use_error_analyze ) {

    efs =
      (int) ceil( settings.error_max_forward / settings.error_forward_step );
    ess =
      (int) ceil( settings.error_max_sideward / settings.error_sideward_step );
    ers =
      (int) ceil( settings.error_max_rotation / settings.error_rotation_step );
    
    error = (double ***) mdalloc( 3, sizeof(double),
				  2*efs+1, 2*ess+1, 2*ers+1 );
    
    for (i=0;i<2*efs+1;i++) {
      for (j=0;j<2*ess+1;j++) {
	for (k=0;k<2*ers+1;k++) {
	  error[i][j][k] = 0;
	}
      }
    }
  }

  
  /****************************************************************
   *                            DATA FILE
   ****************************************************************/
  
  logtools_read_logfile( &rec, settings.data_filename );
  save_rec2_movements( rec, &orig, settings.laser_number );
  if (settings.add_noise)
    add_noise( &rec, orig, settings.laser_number );
  save_rec2_movements( rec, &movements, settings.laser_number );
  history = (int *) malloc( rec.numlaserscans * sizeof(int) );
  
  for (i=0; i<rec.numlaserscans;i++) {
    rec.lsens[i].coord =
      (logtools_laser_coord2_t *) malloc( rec.lsens[i].laser.numvalues *
					  sizeof(logtools_laser_coord2_t) );
  }

  for (i=0; i<rec.numlaserscans;i++) {
    if (rec.lsens[i].id==settings.laser_number)
      break;
  }
    
  
  cnt = 0;
  history[cnt] = i;
  rec.lsens[history[cnt]].estpos = settings.global_map_start_pos;
  
  for (i=history[cnt]+1; i<rec.numlaserscans;i++) {
    if (rec.lsens[i].id==settings.laser_number) {
      if (minimal_rpos_diff( movements.pos[history[cnt]], movements.pos[i],
			     settings.pos_diff_min_dist, 
			     settings.pos_diff_min_rot ) ) {
	
	current_scan =  &(rec.lsens[i]);
	
	move = logtools_movement2_between_rpos2( movements.pos[history[cnt]],
						   movements.pos[i] );
	
	for (j=0;j<rec.lsens[i].laser.numvalues;j++) {
	  rec.lsens[i].coord[j] =
	    map2d_compute_laser2d_coord(rec.lsens[i], j);
	}
	
	printf( "****************************************************\n" );
	printf( "compute scans %d - %d\n",
		history[cnt], i );
	fprintf( stderr,
		 "****************************************************\n" );
	if (settings.add_noise) {
	  cmove = logtools_movement2_between_rpos2( orig.pos[history[cnt]],
						    orig.pos[i] );
	  printf( "correct movment      %.4f %.4f %.4f\n",
		  cmove.forward, cmove.sideward, rad2deg(cmove.rotation) );
	}
	printf( "estimated movment    %.4f %.4f %.4f\n",
		move.forward, move.sideward, rad2deg(move.rotation) );
	if (settings.use_correction) {
	  
	  /* clear local map */
	  clear_local_treemap( &(lmap->qtree), lmap, hk );
	  
	  ctr = cnt; tctr = 0;
	  for (h=cnt-1;
	       h>=0 &&
		 h>cnt-settings.local_map_num_history &&
		 tctr<settings.local_map_max_used_history;
	       h -= settings.local_map_history_skip) {
	    if (intersect_bboxes( rec.lsens[history[cnt]].bbox,
				  rec.lsens[history[h]].bbox ) &&
		( rpos2_dist( rec.lsens[history[h]].estpos,
			      rec.lsens[history[ctr]].estpos ) >
		  settings.local_map_min_bbox_distance ||
		  (cnt-1-h<15) ) ) {
	      lmove =
		logtools_movement2_between_rpos2( rec.lsens[history[h]].estpos,
						  rec.lsens[history[cnt]].estpos );
	      create_local_map( lmap, rec.lsens[history[h]], lmove );
	      ctr = h;
	      tctr++;
	    }
	  }
	  printf( "    - using %d scans for local map in history\n", tctr );
	  
	  histcnt = cnt;
	  ct = 0;
	  
	  /* convolve maps */
	  convolve_map( lmap );
	  
	  bestmove = fit_data_in_local_map( *lmap, &(rec.lsens[i]), move );
	  
	  printf( "best movment         %.4f %.4f %.4f\n",
		  bestmove.forward, bestmove.sideward,
		  rad2deg(bestmove.rotation) );
	  
	  rec.lsens[i].estpos =
	    logtools_rpos2_with_movement2( rec.lsens[history[cnt]].estpos,
					   bestmove );
	  
	  if ( settings.use_error_analyze &&
	       rpos2_dist( rec.lsens[i].estpos, last_error_pos ) >
	       settings.error_min_distance ) {
	    
	    e_ctr++;
	    
	    for (ef=0;ef<2*efs+1;ef++) {
	      for (es=0;es<2*ess+1;es++) {
		for (er=0;er<2*ers+1;er++) {
		  testmove = bestmove;
		  testmove.forward  +=
		    (ef-efs) * settings.error_forward_step;
		  testmove.sideward += 
		    (es-efs) * settings.error_sideward_step;
		  testmove.rotation += 
		    (er-ers) * settings.error_rotation_step;
		  probability_with_move( *lmap, rec.lsens[i], testmove,
					 bestmove, &(error[ef][es][er]) );
		}
	      }
	    }
	    for (ef=0;ef<2*efs+1;ef++) {
	      for (es=0;es<2*ess+1;es++) {
		for (er=0;er<2*ers+1;er++) {
		}
	      }
	    }
	    
	    last_error_pos = rec.lsens[i].estpos;
	  }
	  
	  
	  for (l=0;l<rec.lsens[i].laser.numvalues;l++) {
	    rec.lsens[i].coord[l] =
	      map2d_compute_laser2d_coord(rec.lsens[i], l);
	  }
	  
	  compute_laser2d_bbox( &rec.lsens[i] );
	} else {
	  rec.lsens[i].estpos =
	    logtools_rpos2_with_movement2( rec.lsens[history[cnt]].estpos,
					   move );
	  
	  for (l=0;l<rec.lsens[i].laser.numvalues;l++) {
	    rec.lsens[i].coord[l] =
	      map2d_compute_laser2d_coord(rec.lsens[i], l);
	  }
	  
	}
	current_movement = &bestmove;
	currentpos = rec.lsens[i].estpos;
	  switch(window_maptype()) {
	  case LOCAL_MAP:
	    if (i%5==0)
	      update_map();
	    paint_robot( npos );
	    break;
	  case GLOBAL_MAP:
	    if (settings.show_updates && i%settings.show_updates==0) {
	      center_robot();
	      update_map();
	    }
	    paint_robot( rec.lsens[i].estpos );
	    break;
	  case SHOW_RAYS:
	    window_show_rays();
	    break;
	  }
	  if (settings.use_global_map) {
	    update_global_map( rec.lsens[i], gmap );
	  }
	  cnt++;
	  history[cnt] = i;
	  printf( "****************************************************\n\n" );
	  if (settings.log_output) {
	    write_log_entry( settings.logF, &(rec.lsens[i]) );
	  }
      } else {
	/* interpolate position */
      }
    } else {
      /* it's not the wished laser  */
      if (settings.use_correction && settings.log_output) {
	/* also it's not the right laser it's using the last corrected
	   position. There should be a linear interpolation, but I am
	   to lazy, so I am only using the last pos */
	rec.lsens[i].estpos = currentpos;
	write_log_entry( settings.logF, &(rec.lsens[i]) );
      }
    }
    
    if (settings.use_graphics) {
      app_process_events();
    }
    
    usleep(1);
    
}

  
  if (settings.use_graphics) {
    while (TRUE) {
      app_process_events();
      usleep(1);
    }
  } else {
    if (settings.use_global_map) {
      compute_map_probs( gmap );
    }
    
    free(orig.pos);
    free(orig.move);
    free(history);
  }
  
}
  
