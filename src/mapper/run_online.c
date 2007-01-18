#include "map2d.h"

ONLINE_LASER_DATA       online_data;
int                     online_scan_ctr    = 0;
int                     online_laserupdate = FALSE;
int                     online_odo_update = FALSE;

logtools_lasersens2_data_t   * current_scan;
logtools_rmove2_t            * current_movement;
logtools_rpos2_t               current_pos = {0.0, 0.0, 0.0};

int                       loop = TRUE;
int                       change_map = TRUE;

void
abort_signal( int sig )
{
  fprintf( stderr, "abort with signal %d\n", sig );
  loop = FALSE;
}


void
alloc_online_structures( ONLINE_LASER_DATA * data, REC2_MOVEMENTS * movements )
{
  int i, nh;
  
  nh = settings.local_map_num_history;
  nh = (nh>0?nh:1);
    
  data->lsens =
    (logtools_lasersens2_data_t *)
    malloc( nh * sizeof(logtools_lasersens2_data_t) );
  
  for (i=0; i<nh; i++) {
    data->lsens[i].laser.val        =
      (float *) malloc( MAX_NUM_LASER_BEAMS * sizeof(double) );
    data->lsens[i].laser.angle      =
      (float *) malloc( MAX_NUM_LASER_BEAMS * sizeof(double) );
    data->lsens[i].coord            =
      (logtools_laser_coord2_t *) malloc( MAX_NUM_LASER_BEAMS *
					  sizeof(logtools_laser_coord2_t) );
    /*
      data->lsens[i].poly.pt          =
      (VECTOR2 *) malloc( MAX_NUM_LASER_BEAMS * sizeof(VECTOR2) );
    */
  }
  
  data->lsens[0].estpos = settings.global_map_start_pos;

  movements->nummovements = nh;
  movements->pos          = (logtools_rpos2_t *) malloc( nh * sizeof(logtools_rpos2_t) );
  movements->move         = (logtools_rmove2_t *) malloc( nh * sizeof(logtools_rmove2_t) );
}


/****************************************************************
 *                            ONLINE
 ****************************************************************/

void
run_online( MAP2D_SETTINGS settings, MAP2 * lmap, MAP2 * gmap,
	    int argc, char *argv[] )
{
  static int              update_ctr = 0;

  int                     h, l, hk, tctr, r, r_new, r_old, ctr=0;

  REC2_MOVEMENTS          movements;
  logtools_rpos2_t        npos   = {0.0, 0.0, 0.0};
  
  logtools_rmove2_t       lmove, move, bestmove;
  logtools_rmove2_t       nomove = { 0.0, 0.0, 0.0 };
  logtools_laser_coord2_t coord[MAX_NUM_LASER_VALUES];
  
  int                     moctr, loctr, nhist;

  hk            = (settings.local_map_kernel_len-1)/2;
  nhist         = settings.local_map_num_history;
  current_scan  = NULL;
  
  alloc_online_structures( &online_data, &movements );

  signal(SIGINT, abort_signal);

  ipc_init( argc, argv );

  fprintf( stderr, "***************************************\n" );
  fprintf( stderr, "*        START\n" );
  fprintf( stderr, "***************************************\n" );

  loctr = 0;
  while (loop) {
    
    ipc_update();
    
    if (online_laserupdate) {
      
      if (1 || online_odo_update) {
	
	r = online_scan_ctr%nhist;

	online_data.lsens[r].estpos = current_pos;
	
	online_laserupdate = FALSE;
	update_ctr++;
	
	/*
	  compute_forward_correction( online_data.lsens[r].estpos,
	  center, &pos );
	  online_data.lsens[r].estpos = pos;
	*/
	movements.pos[r] =
	  online_data.lsens[r].estpos;
	
	if (settings.use_correction) {
	  
	  if (online_scan_ctr>0) {
  
	    r_old = (nhist+online_scan_ctr-1)%nhist;
	    r_new = (online_scan_ctr)%nhist;
	    
	    move =
	      logtools_movement2_between_rpos2( movements.pos[r_old],
						current_pos );

	    if (rmove2_length(move)>settings.pos_diff_min_dist) {
	      
	      clear_local_treemap( &(lmap->qtree), lmap, hk );
	      create_local_map( lmap,
				online_data.lsens[r_old], nomove );
	      tctr  = 0;
	      moctr = r_old;
	      for (h=online_scan_ctr-2;
		   h>=0 &&
		     h>online_scan_ctr-settings.local_map_num_history &&
		     tctr<settings.local_map_max_used_history;
		   h -= settings.local_map_history_skip) {
		if ( intersect_bboxes( online_data.lsens[r_old].bbox,
				       online_data.lsens[h%nhist].bbox ) &&
		     rpos2_dist( online_data.lsens[moctr].estpos,
				 online_data.lsens[h%nhist].estpos ) >
		     settings.local_map_min_bbox_distance ) {
		  lmove =
		    logtools_movement2_between_rpos2( online_data.lsens[h%nhist].estpos,
						      online_data.lsens[r_old].estpos );
		  moctr = h%nhist;
		  create_local_map( lmap, online_data.lsens[h%nhist], lmove );
		  tctr++;
		}
	      }
	      convolve_map( lmap );
	      bestmove = fit_data_in_local_map( *lmap,
						&(online_data.lsens[r]),
						move );

	      if ( minimal_rmove_diff( bestmove, 
				       settings.pos_diff_min_dist, 
				       settings.pos_diff_min_rot )) {
#ifdef VERBOSE
		printf( "using %d scans in history\n", tctr );
#endif

		/* position of last scan */
		online_data.lsens[r].estpos =
		  logtools_rpos2_with_movement2( online_data.lsens[r_old].estpos,
						bestmove );
		for (l=0;l<online_data.lsens[r].laser.numvalues;l++) {
		  /*
		    online_data.lsens[r].coord[l] =
		    map2d_compute_laser2d_coord( online_data.lsens[r], l);
		  */
		  coord[l] =
		    map2d_compute_laser2d_coord( online_data.lsens[r], l);
		}
		online_data.lsens[r].bbox =
		  laser2d_bbox( online_data.lsens[r].laser.numvalues,
				online_data.lsens[r].laser.val, coord );
		
		printf( "***************************************\n" );
		printf( "estimated movment    %.4f %.4f %.4f\n",
			move.forward, move.sideward, rad2deg(move.rotation) );
		printf( "best movment         %.4f %.4f %.4f\n",
			bestmove.forward, bestmove.sideward,
			rad2deg(bestmove.rotation) );
		if (settings.use_global_map && change_map) {
		  update_global_map( online_data.lsens[r], gmap );
		}

		if (settings.log_output) {
		  write_log_entry( settings.logF, &(online_data.lsens[r]) );
		}
		switch(window_maptype()) {
		case LOCAL_MAP:
		  if (update_ctr%20==0) {
		    update_map();
		    update_ctr = 0;
		  }
		  paint_robot( npos );
		  break;
		case GLOBAL_MAP:
		  if (settings.show_updates &&
		      update_ctr%settings.show_updates==0)
		    update_map();
		  
		  paint_robot( online_data.lsens[r].estpos );
		  break;
		}
		if (change_map) {
		  online_scan_ctr++;
		} else { 
		  online_data.lsens[r_old].estpos =
		    online_data.lsens[r_new].estpos;
		  movements.pos[r_old]            = movements.pos[r_new];
		  online_data.lsens[r_old].bbox   = online_data.lsens[r_new].bbox;
		  for (l=0;l<online_data.lsens[r_new].laser.numvalues;l++) {
		    online_data.lsens[r_old].coord[l] =
		      online_data.lsens[r_new].coord[l];
		    online_data.lsens[r_old].laser.val[l] =
		      online_data.lsens[r_new].laser.val[l];
		    online_data.lsens[r_old].laser.angle[l] =
		      online_data.lsens[r_new].laser.angle[l];
		  }
		}
	      }
	    } else {
	    }
	  } else {
	    /* getting the first scan */
	    movements.pos[0] = current_pos;
	    for (l=0;l<online_data.lsens[r].laser.numvalues;l++) {
	      online_data.lsens[r].coord[l] =
		map2d_compute_laser2d_coord( online_data.lsens[r], l);
	    }
	    compute_laser2d_bbox( &online_data.lsens[r] );
	    online_scan_ctr = (online_scan_ctr+1)%nhist;
	  }
	  printf( "***************************************\n" );
	  printf( "* online scan ctr %d\n", online_scan_ctr );
	} else {
	  /* no correction */
	  printf( "***************************************\n" );
	  printf( "* scan %d\n", ctr++ );
	  printf( "* pos %.1f %.1f %.1f\n",
		  online_data.lsens[0].estpos.x,
		  online_data.lsens[0].estpos.y,
		  rad2deg(online_data.lsens[0].estpos.o) );
	  for (l=0;l<online_data.lsens[0].laser.numvalues;l++) {
	    online_data.lsens[0].coord[l] =
	      map2d_compute_laser2d_coord( online_data.lsens[0], l);
	  }
	  if (settings.use_global_map) {
	    switch(window_maptype()) {
	    case LOCAL_MAP:
	      if (update_ctr%20==0) {
		update_map();
		update_ctr = 0;
	      }
	      paint_robot( npos );
	      break;
	    case GLOBAL_MAP:
	      if (settings.show_updates &&
		  update_ctr%settings.show_updates==0)
		update_map();
	      paint_robot( online_data.lsens[r].estpos );
	      break;
	    }
	    if (change_map) {
	      update_global_map( online_data.lsens[0], gmap );
	    }
	  }
	}
	
      }
      
    }
    usleep(settings.loop_sleep);
    if (settings.use_graphics) {
      app_process_events();
    }
  }
}
