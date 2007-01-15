#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <errno.h>
#include <sys/types.h>
#include <sys/time.h>
#include <time.h>
#include <unistd.h>
#include <math.h>
#include <string.h>
#include <ctype.h>
#include <values.h>

#include <navigation/utils.h>
#include "map2d.h"

ONLINE_LASER_DATA       online_data;
int                     online_scan_ctr    = 0;
int                     online_laserupdate = FALSE;
int                     online_odo_update = FALSE;

LASERSENS2_DATA         * current_scan;
RMOVE2                  * current_movement;
RPOS2                     current_pos = {0.0, 0.0, 0.0};

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
    (LASERSENS2_DATA *) malloc( nh * sizeof(LASERSENS2_DATA) );
  
  for (i=0; i<nh; i++) {
    data->lsens[i].laser.val        =
      (double *) malloc( MAX_NUM_LASER_BEAMS * sizeof(double) );
    data->lsens[i].laser.angle      =
      (double *) malloc( MAX_NUM_LASER_BEAMS * sizeof(double) );
    data->lsens[i].coord            =
      (LASER_COORD2 *) malloc( MAX_NUM_LASER_BEAMS * sizeof(LASER_COORD2) );
    /*
      data->lsens[i].poly.pt          =
      (VECTOR2 *) malloc( MAX_NUM_LASER_BEAMS * sizeof(VECTOR2) );
    */
  }
  
  data->lsens[0].estpos = settings.global_map_start_pos;

  movements->nummovements = nh;
  movements->pos          = (RPOS2 *) malloc( nh * sizeof(RPOS2) );
  movements->move         = (RMOVE2 *) malloc( nh * sizeof(RMOVE2) );
}

void
data_open_file_stream()
{
  int                slen1, slen2;
  
  slen1 = strlen(FILE_SCRIPT_EXT);
  slen2 = strlen(FILE_REC_EXT);
  
  if ( ((int) strlen(settings.data_filename)) > slen1 &&
       !strcmp( &settings.data_filename[strlen(settings.data_filename)-slen1],
		FILE_SCRIPT_EXT ) ) {
    fprintf( stderr, "* INFO: use script-file-type!\n" );
    settings.script_type = SCRIPT;
  } else if ( ((int) strlen(settings.data_filename)) > slen2 &&
	      !strcmp( &settings.data_filename[strlen(settings.data_filename)-slen2],
		       FILE_REC_EXT ) ) {
    fprintf( stderr, "* INFO: read rec-file-type!\n" );
    settings.script_type = REC;
  } else {
    fprintf( stderr, "* INFO: assuming script-file-type\n" );
    settings.script_type = SCRIPT;
  }
  fprintf( stderr, "***************************************\n" );

  if ((settings.dataF = fopen( settings.data_filename, "r")) == 0){
   fprintf(stderr, "ERROR: could not open input file %s\n",
	   settings.data_filename );
   exit(0);
 }
 fprintf(stderr, "* INFO: open input file %s\n", settings.data_filename ); 
	  
}


enum ENTRY_TYPE
line_type( REC2_DATA rec )
{
  if (rec.numpositions+
      rec.numcpositions+
      rec.numlaserscans>1) {
    return(UNKNOWN);
  } else {
    if (rec.numpositions>0) {
      return(POSITION);
    } else if (rec.numcpositions>0) {
      return(CORR_POSITION);
    } else if (rec.numlaserscans>0) {
      return(LASER_VALUES);
    } else {
      return(UNKNOWN);
    }
  }
}

int
data_read_next_dataset( void )
{
  static char           line[MAX_LINE_LENGTH];
  static REC2_DATA      rec;
  static int            first_time = TRUE;
  static int            file_closed = FALSE;
  int                   r, i;

  if (first_time) {
    rec.entry   =
      (ENTRY_POSITION *) malloc( sizeof(ENTRY_POSITION) );
    rec.psens =
      (POSSENS2_DATA *) malloc( sizeof(POSSENS2_DATA) ); 
    rec.cpsens =
      (POSSENS2_DATA *) malloc( sizeof(POSSENS2_DATA) ); 
    rec.lsens =
      (LASERSENS2_DATA *) malloc( sizeof(LASERSENS2_DATA) );
    rec.lsens[0].laser.val =
      (double *) malloc( MAX_NUM_LASER_VALUES * sizeof(double) );
    rec.lsens[0].laser.maxrange =
      (int *) malloc( MAX_NUM_LASER_VALUES * sizeof(int) );
    rec.lsens[0].laser.angle =
      (double *) malloc( MAX_NUM_LASER_VALUES * sizeof(double) );
    rec.lsens[0].laser.offset =
      (RMOVE2 *) malloc( MAX_NUM_LASER_VALUES * sizeof(RMOVE2) );
    first_time = FALSE;
  }

  if (!file_closed) {
    if (fgets(line,MAX_LINE_LENGTH,settings.dataF) == NULL) {
      fclose(settings.dataF);
      file_closed = TRUE;
    } else {
      rec.numpositions   = 0;
      rec.numcpositions  = 0;
      rec.numlaserscans  = 0;
      rec.numentries     = 0;
      rec2_parse_line( line, &rec, FALSE, FALSE );
      switch(line_type( rec )) {
      case POSITION:
	current_pos.x = rec.psens[0].rpos.x;
	current_pos.y = rec.psens[0].rpos.y;
	current_pos.o = rec.psens[0].rpos.o;
	break;
      case CORR_POSITION:
	break;
      case POS_CORR:
	break;
      case LASER_VALUES:
	if (rec.lsens[0].id == settings.laser_number) {
	  r = online_scan_ctr%settings.local_map_num_history;
	  online_data.lsens[r].laser.numvalues = rec.lsens[0].laser.numvalues;
	  for (i = 0; i < rec.lsens[0].laser.numvalues; i++){
	    online_data.lsens[r].laser.val[i]     = rec.lsens[0].laser.val[i];
	    online_data.lsens[r].laser.angle[i]   = rec.lsens[0].laser.angle[i];
	  }
	  online_data.lsens[r].laser.time       = rec.lsens[0].laser.time;
	  online_data.lsens[r].id               = rec.lsens[0].id;
	  online_data.lsens[r].laser.anglerange = rec.lsens[0].laser.anglerange;
	  online_data.lsens[r].estpos           = current_pos;
	  online_laserupdate = TRUE;
	} 
	break;
      case UNKNOWN:
	break;
      case LASER_VALUES3:
	break;
      case HUMAN_PROB:
	break;
      case AMTEC_POS:
	break;
      case GPS:
	break;
      case COMPASS:
	break;
      default:
	break;
      }
    }
    return(TRUE);
  } else {
    return(FALSE);
  }
 
}


/****************************************************************
 *                            ONLINE
 ****************************************************************/

void
run_online( MAP2D_SETTINGS settings, MAP2 * lmap, MAP2 * rmap,
	    MAP2 * gmap, int online )
{
  static int              update_ctr = 0;

  int                     h, l, hk, tctr, r, r_new, r_old, ctr=0;

  REC2_MOVEMENTS          movements;
  RPOS2                   npos   = {0.0, 0.0, 0.0};
  
  RMOVE2                  lmove, move, bestmove;
  RMOVE2                  nomove = { 0.0, 0.0, 0.0 };
  LASER_COORD2            coord[MAX_NUM_LASER_VALUES];
  
  int                     moctr, loctr, nhist;

  hk            = (settings.local_map_kernel_len-1)/2;
  nhist         = settings.local_map_num_history;
  current_scan  = NULL;
  
  alloc_online_structures( &online_data, &movements );

  signal(SIGINT, abort_signal);

  if (online) {
    ipc_init( "map2d" );
  } else {
    data_open_file_stream();
  }

  fprintf( stderr, "***************************************\n" );
  fprintf( stderr, "*        START\n" );
  fprintf( stderr, "***************************************\n" );

  loctr = 0;
  while (loop) {
    
    if (online) {
      ipc_update();
    } else {
      data_read_next_dataset();
    }
    
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
	      compute_movement2_between_rpos2( movements.pos[r_old],
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
		    compute_movement2_between_rpos2( online_data.lsens[h%nhist].estpos,
						     online_data.lsens[r_old].estpos );
		  moctr = h%nhist;
		  create_local_map( lmap, online_data.lsens[h%nhist], lmove );
		  tctr++;
		}
	      }
	      convolve_map( lmap );
	      bestmove = fit_data_in_local_map( *lmap, *rmap,
						&(online_data.lsens[r]),
						move, tctr, online_scan_ctr, 0 );
	      if ( minimal_rmove_diff( bestmove, 
				       settings.pos_diff_min_dist, 
				       settings.pos_diff_min_rot )) {
#ifdef VERBOSE
		printf( "using %d scans in history\n", tctr );
#endif

		/* position of last scan */
		online_data.lsens[r].estpos =
		  compute_rpos2_with_movement2( online_data.lsens[r_old].estpos,
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
		if (online) {
		  ipc_publish_status( online_data.lsens[r].estpos,
				      online_data.lsens[r].laser.time );
		}
		if (settings.script_output) {
		  write_script_entry( settings.scriptF,
				      online_data.lsens[r],
				      online_data.lsens[r].id );
		}
		if (1) 
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
	if (settings.output_data_map) {
	  write_data_entry( settings.outputF,
			    online_data.lsens[r],
			    online_data.lsens[r].id );
	}
	
      }
      
    }
    if(online)
      usleep(settings.loop_sleep);
    if (settings.use_graphics) {
      app_process_events();
    }
  }
}
