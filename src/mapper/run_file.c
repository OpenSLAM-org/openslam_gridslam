#include <stdio.h>
#include <stdlib.h>
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


extern LASERSENS2_DATA         * current_scan;
extern RMOVE2                  * current_movement;

REC2_DATA                        rec;

int                              histcnt;
int                            * history;

double
rpos2_dist( RPOS2 pos1, RPOS2 pos2 )
{
  return(
	 sqrt( (pos1.x - pos2.x) * (pos1.x - pos2.x) +
	       (pos1.y - pos2.y) * (pos1.y - pos2.y) )
	 );
}


double
rmove2_length( RMOVE2 move )
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
read_input_file( REC2_DATA * rec, char * filename )
{
  int                slen1, slen2;
  
  slen1 = strlen(FILE_SCRIPT_EXT);
  slen2 = strlen(FILE_REC_EXT);
  
  if ( ((int) strlen(filename)) > slen1 &&
       !strcmp( &filename[strlen(filename)-slen1], FILE_SCRIPT_EXT ) ) {
    fprintf( stderr, "* INFO: use script-file-type!\n" );
    settings.script_type = SCRIPT;
  } else if ( ((int) strlen(filename)) > slen2 &&
	      !strcmp( &filename[strlen(filename)-slen2], FILE_REC_EXT ) ) {
    fprintf( stderr, "* INFO: read rec-file-type!\n" );
    settings.script_type = REC;
  } else {
    fprintf( stderr, "* INFO: assuming script-file-type\n" );
    settings.script_type = SCRIPT;
  }
  fprintf( stderr, "***************************************\n" );
	  
  if (settings.script_type==SCRIPT) {
    if (read_script( filename, rec, 1 ) !=0 )
      exit(1);
  } else if (settings.script_type==REC) {
    if (read_rec2d_file( filename, rec, 1 ) !=0 )
      exit(1);
  } else {
    fprintf( stderr, "ERROR: unknown file-type!" );
    exit(1);
  }
}

void
run_data_file( MAP2D_SETTINGS settings,
	       MAP2 * lmap, MAP2 * rmap,
	       MAP2 * gmap, MAP2 * grmap )
{
  int                     i, j, h, k, l, hk, rhk, tctr, lcnt, lccnt;
  int                     x, y, pcnt;
  int                     e_ctr = 0;
  
  char                    filename[MAX_NAME_LENGTH];
 
  REC2_MOVEMENTS          orig, movements;

  double                  bprob, bsum, ptotal;
 
  int                     ef, es, er;
  int                     efs = 0, ess = 0, ers = 0;
  RMOVE2                  testmove;
  
  RPOS2                   npos = { 0.0, 0.0, 0.0 };
  RPOS2                   last_error_pos = { 0.0, 0.0, 0.0 };
  RPOS2                   currentpos = { 0.0, 0.0, 0.0 };
  int                     cnt = 0, ct, ctr;
  
  RMOVE2                  lmove, move, bestmove, cmove;
  RMOVE2                  nomove = { 0.0, 0.0, 0.0 };

  double              *** error = NULL;
  
  settings.script_loop_nr = 1;

  hk = (settings.local_map_kernel_len-1)/2;
  rhk = (settings.local_ray_map_kernel_len-1)/2;
  
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
  
  read_input_file( &rec, settings.data_filename );
  save_rec2_movements( rec, &orig, settings.laser_number );
  if (settings.add_noise)
    add_noise( &rec, orig, settings.laser_number );
  save_rec2_movements( rec, &movements, settings.laser_number );
  history = (int *) malloc( rec.numlaserscans * sizeof(int) );
  
  for (i=0; i<rec.numlaserscans;i++) {
    if (rec.lsens[i].poly.pt==NULL) {
      rec.lsens[i].poly.pt =
	(VECTOR2 *) malloc( rec.lsens[i].laser.numvalues * sizeof(VECTOR2) );
    }
    if (settings.use_local_ray_map || settings.use_global_ray_map) {
      if (rec.lsens[i].ptracking.hprob==NULL) {
	rec.lsens[i].ptracking.hprob=
	  (double *) malloc( rec.lsens[i].laser.numvalues * sizeof(double) );
	for (j=0; j<rec.lsens[i].laser.numvalues; j++) {
	  rec.lsens[i].ptracking.hprob[j] = settings.dynprob_prior;
	}
      }
    }
  }
  
  for (i=0; i<rec.numlaserscans;i++) {
    rec.lsens[i].coord =
      (LASER_COORD2 *) malloc( rec.lsens[i].laser.numvalues *
			       sizeof(LASER_COORD2) );
  }

  for (i=0; i<rec.numlaserscans;i++) {
    if (rec.lsens[i].id==settings.laser_number)
      break;
  }
    
  
  cnt = 0;
  history[cnt] = i;
  rec.lsens[history[cnt]].estpos = settings.global_map_start_pos;
  
  if (settings.output_data_map) {
    write_data_entry( settings.outputF, rec.lsens[history[cnt]],
		      settings.laser_number );
  }
  if (settings.script_output) {
    for (i=0; i<=history[cnt]; i++)
      write_script_entry( settings.scriptF, rec.lsens[i],
			  settings.laser_number );
  }
  
  while (settings.script_loop_nr>0) {
    
    if (settings.use_global_ray_map && settings.script_output) {
      write_script_marker( settings.scriptF, LOOP_START,
			   settings.script_loop_nr );
    }
    
    for (i=history[cnt]+1; i<rec.numlaserscans;i++) {
      if (rec.lsens[i].id==settings.laser_number) {
	if (minimal_rpos_diff( movements.pos[history[cnt]], movements.pos[i],
			       settings.pos_diff_min_dist, 
			       settings.pos_diff_min_rot ) ) {
	  
	  if (settings.output_data_map) {
	    write_data_entry( settings.outputF, rec.lsens[0],
			      settings.laser_number );
	  }
	  
	  current_scan =  &(rec.lsens[i]);
	  
	  move = compute_movement2_between_rpos2( movements.pos[history[cnt]],
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
	    cmove = compute_movement2_between_rpos2( orig.pos[history[cnt]],
						     orig.pos[i] );
	    printf( "correct movment      %.4f %.4f %.4f\n",
		    cmove.forward, cmove.sideward, rad2deg(cmove.rotation) );
	  }
	  if (0 && settings.local_map_use_odometry==ODO_LAST && i>1) {
	    move =
	      compute_movement2_between_rpos2( rec.lsens[i-2].estpos,
					       rec.lsens[i-1].estpos );
	    move.forward = 3.0;
	  }
	  printf( "estimated movment    %.4f %.4f %.4f\n",
		  move.forward, move.sideward, rad2deg(move.rotation) );
	  if (settings.use_correction) {
	    
	    /* clear local map */
	    clear_local_treemap( &(lmap->qtree), lmap, hk );
	    // create_local_map( lmap, rec.lsens[history[cnt]], nomove );
	    
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
		  compute_movement2_between_rpos2( rec.lsens[history[h]].estpos,
						   rec.lsens[history[cnt]].estpos );
		create_local_map( lmap, rec.lsens[history[h]], lmove );
		ctr = h;
		tctr++;
	      }
	    }
	    printf( "    - using %d scans for local map in history\n", tctr );
	    
	    if (settings.use_local_ray_map) {
	      clear_local_treemap( &(rmap->qtree), rmap, rhk );
	      create_local_ray_map( rmap, rec.lsens[history[cnt]], nomove );
	      ctr = cnt; tctr = 0;
	      for (h=cnt-1;
		   h>=0 &&
		     h>cnt-settings.local_ray_map_num_history &&
		     tctr<settings.local_ray_map_max_used_history;
		   h--) {
		if (intersect_bboxes( rec.lsens[history[cnt]].bbox,
				      rec.lsens[history[h]].bbox ) &&
		    ( rpos2_dist( rec.lsens[history[h]].estpos,
				  rec.lsens[history[ctr]].estpos ) >
		      settings.local_ray_map_min_bbox_distance ||
		      (cnt-1-h<5) ) ) {
		  lmove =
		    compute_movement2_between_rpos2( rec.lsens[history[h]].estpos,
						     rec.lsens[history[cnt]].estpos );
		  create_local_ray_map( rmap, rec.lsens[history[h]], lmove );
		  ctr = h;
		  tctr++;
		}
	      }
	      printf( "    - using %d scans for local ray map in history\n", tctr );
	    }
	    histcnt = cnt;
	    ct = 0;
	    
	    /* convolve maps */
	    convolve_map( lmap );

	    if (settings.use_local_ray_map) {
	      convolve_ray_map( rmap );
	    }
	    
	    bestmove = fit_data_in_local_map( *lmap, *rmap, &(rec.lsens[i]),
					      move, tctr, i, settings.script_loop_nr );
	    printf( "best movment         %.4f %.4f %.4f\n",
		    bestmove.forward, bestmove.sideward,
		    rad2deg(bestmove.rotation) );

	    
	    /*
	    printf( "%.4f %.4f %.4f # est move\n",
		    move.forward, move.sideward,
		    rad2deg(move.rotation) );
	    printf( "%.4f %.4f %.4f # best move\n",
		    bestmove.forward, bestmove.sideward,
		    rad2deg(bestmove.rotation) );
	    
	      printf( "%.4f # ratio\n",
		    rmove2_length( move ) /
		    rmove2_length( bestmove ) );
	    */
	    rec.lsens[i].estpos =
	      compute_rpos2_with_movement2( rec.lsens[history[cnt]].estpos,
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
		    fprintf( settings.statisticsF,
			     "%.4f %.4f %.4f %f # ERORR in SCAN %s%s%s%s%s%d\n",
			     (ef-efs) * settings.error_forward_step,
			     (es-efs) * settings.error_sideward_step,
			     rad2deg((er-ers) * settings.error_rotation_step),
			     error[ef][es][er],
			     e_ctr < 100000 ? "0" : "",
			     e_ctr < 10000 ? "0" : "",
			     e_ctr < 1000 ? "0" : "",
			     e_ctr < 100 ? "0" : "",
			     e_ctr < 10 ? "0" : "",
			     e_ctr );
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
	      compute_rpos2_with_movement2( rec.lsens[history[cnt]].estpos,
					    move );
	    
	    for (l=0;l<rec.lsens[i].laser.numvalues;l++) {
	      rec.lsens[i].coord[l] =
		map2d_compute_laser2d_coord(rec.lsens[i], l);
	    }
	    
	  }
	  current_movement = &bestmove;
	  currentpos = rec.lsens[i].estpos;
	  switch(window_maptype()) {
	  case LOCAL_RAY_MAP:
	    if (settings.use_local_ray_map) {
	      if (i%5==0)
		update_map();
	      paint_robot( npos );
	    }
	    break;
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
	  if (settings.use_global_ray_map) {
	    update_global_ray_map( rec.lsens[i], grmap, settings.script_loop_nr );
	  }
	  
	  cnt++;
	  history[cnt] = i;
	  printf( "****************************************************\n\n" );
	  if (settings.script_output) {
	    write_script_entry( settings.scriptF, rec.lsens[i],
				settings.laser_number );
	  }
	  if (settings.output_data_map) {
	    write_data_entry( settings.outputF, rec.lsens[i],
			      settings.laser_number );
	  }
	} else {
	  /* interpolate position */
	}
      } else {
	/* it's not the wished laser  */
	if (settings.use_correction && settings.script_output) {
	  /* also it's not the right laser it's using the last corrected
	     position. There should be a linear interpolation, but I am
	     to lazy, so I am only using the last pos */
	  rec.lsens[i].estpos = currentpos;
	  write_script_entry( settings.scriptF, rec.lsens[i],
			      rec.lsens[i].id );
	}
      }
      
      if (settings.use_graphics) {
	app_process_events();
      }
      
      usleep(1);
      
    }
    
    cnt++;
    if (settings.use_global_ray_map) {
      fprintf( stderr, "compute dynamic probs map\n" );
      compute_probs_of_global_ray_map( grmap );
      fprintf( stderr, "compute dynamic probs for all scans\n" );
      bsum = 0.0;
      for (l=0;l<rec.numlaserscans;l++)
	dynamic_prob_from_global_map( &(rec.lsens[l]), *grmap );
      if (settings.output_statistics) {
	for (l=1;l<rec.numlaserscans;l++) {
	  bprob = probability_with_pos( *grmap,
					rec.lsens[l],
					rec.lsens[l].estpos,
					compute_movement2_between_rpos2( movements.pos[l-1],
									 movements.pos[l] ),
					compute_movement2_between_rpos2( rec.lsens[l-1].estpos,
									 rec.lsens[l].estpos ) );
	  fprintf( settings.statisticsF,
		   "    %.20f # global map scan %d loop %s%s%s%d\n",
		   bprob, l,
		   settings.script_loop_nr<1000? "0":"",
		   settings.script_loop_nr<100? "0":"",
		   settings.script_loop_nr<10? "0":"",
		   settings.script_loop_nr );
	  bsum += bprob;
	}
	lcnt = 0;
	lccnt = 0;
	for (l=1;l<rec.numlaserscans;l++) {
	  for (k=0;k<rec.lsens[l].laser.numvalues;k++) {
	    lcnt++;
	    if (rec.lsens[l].ptracking.hprob[k]<0.4) {
	      lccnt++;
	    }
	  }
	}
	fprintf( settings.statisticsF,
		 "    %.20f # map log sum pobs\n", bsum );
	fprintf( settings.statisticsF,
		 "    %d %d %.5f # corrupted\n",
		 lcnt, lccnt, lccnt / (double) lcnt );
      }
      snprintf( filename, MAX_NAME_LENGTH, "map-%s%s%d.png",
		settings.script_loop_nr<100 ? "0" : "",
		settings.script_loop_nr<10 ? "0" : "",
		settings.script_loop_nr );
      compute_map_probs( gmap );
      write_map( *gmap, filename );
      
      ptotal = 0.0; pcnt = 0;
      for (x=0;x<gmap->mapsize.x;x++) {
	for (y=0;y<gmap->mapsize.y;y++) {
	  ptotal += 1.0-gmap->mapprob[x][y];
	  pcnt++;
	}
      }
      if (pcnt>0)
	fprintf( settings.statisticsF,
		 "    %.20f # map total pobs\n", ptotal/(double)pcnt );
      if (settings.script_output) {
	write_script_marker( settings.scriptF, LOOP_END, settings.script_loop_nr );
	fclose(settings.scriptF);
	create_script_file( settings.script_loop_nr+1 );
      }
      
      fprintf( stderr, "clear map\n" );
      clear_global_map( gmap );
      if (settings.use_global_ray_map) {
	clear_global_map( grmap );
      }
      cnt = 0;
      settings.script_loop_nr++;
    } else {
      settings.script_loop_nr = 0;
    }
    
    
  }
  
  
  if (settings.use_graphics) {
    while (TRUE) {
      app_process_events();
      usleep(1);
    }
  } else {
    if (settings.use_global_map) {
      compute_map_probs( gmap );
      write_map( *gmap, settings.global_map_filename );
    }
    
    if (0 && settings.output_statistics) {
      fprintf( stderr, "print statistics\n" );
      compute_statistics( rec, settings.laser_number );
    }
    
    /*
      if (settings.add_noise && settings.experiment_log) {
      fprintf( stderr, "write experiment log\n" );
      write_total_error( rec, orig, settings.laser_number );
      }
    */
    
    free(orig.pos);
    free(orig.move);
    free(history);
  }
  
}
  
