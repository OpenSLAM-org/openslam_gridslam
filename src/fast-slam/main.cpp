#include <qapplication.h>

#ifdef __cplusplus
extern "C" {
#endif

#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <unistd.h>
#include <time.h>
#include <math.h>
#include <values.h>

#include <navigation/utils.h>

#ifdef __cplusplus
}
#endif
#include "graphics.h"

#define MAX_STRING_LENGTH             80

enum FILE_TYPE                      { SCRIPT, REC };
#define FILE_SCRIPT_EXT             ".script"
#define FILE_REC_EXT                ".rec"

#define NUM_TESTS                200000
#define EPSILON                       0.000000000001


int loop = TRUE;


void
print_usage( void )
{
  fprintf(stderr, "\nusage: grid-fast-slam [-ini <INI-FILE>] <REC-FILE>\n" );
}

void 
shutdown( int sig ) {
  loop = FALSE;
  exit(sig);
}

typedef struct {
  RPOS2           pos;
  struct timeval  time;
} RPOS2_HIST;

void
clear_map( MAP2 * map, RPOS2 pos )
{
  int        x, y;
  for (x=0;x<map->mapsize.x;x++) {
    for (y=0;y<map->mapsize.y;y++) {
      map->maphit[x][y]  = 0.0;
      map->mapsum[x][y]  = 0;
      map->calc[x][y]    = 0.0;
    }
  }
  map->offset     = pos;
}


void
clear_scans( MAP2 * map )
{
  int        x, y;
  for (x=0;x<map->mapsize.x;x++) {
    for (y=0;y<map->mapsize.y;y++) {
      map->calc[x][y]    = 0.0;
    }
  }
}

double
rpos2_length( RPOS2 pos1, RPOS2 pos2 )
{
  return( sqrt( ( (pos1.x-pos2.x) * (pos1.x-pos2.x) ) +
		( (pos1.y-pos2.y) * (pos1.y-pos2.y) ) ) );
}

enum FILE_TYPE
read_input_file( REC2_DATA * rec, char * filename )
{
  enum FILE_TYPE     filetype;
  int                slen1, slen2;
  
  slen1 = strlen(FILE_SCRIPT_EXT);
  slen2 = strlen(FILE_REC_EXT);
  
  if ( ((int) strlen(filename)) > slen1 &&
       !strcmp( &filename[strlen(filename)-slen1], FILE_SCRIPT_EXT ) ) {
    fprintf( stderr, "* INFO: use script-file-type!\n" );
    filetype = SCRIPT;
  } else if ( ((int) strlen(filename)) > slen2 &&
	      !strcmp( &filename[strlen(filename)-slen2], FILE_REC_EXT ) ) {
    fprintf( stderr, "* INFO: read rec-file-type!\n" );
    filetype = REC;
  } else {
    fprintf( stderr, "* INFO: assuming script-file-type\n" );
    filetype = SCRIPT;
  }
  fprintf( stderr, "***************************************\n" );
	  
  if (filetype==SCRIPT) {
    if (read_script( filename, rec, 1 ) !=0 )
      exit(1);
  } else if (filetype==REC) {
    if (read_rec2d_file( filename, rec, 1 ) !=0 )
      exit(1);
  } else {
    fprintf( stderr, "ERROR: unknown file-type!" );
    exit(1);
  }
  return( filetype );
}

int
main( int argc, char** argv)
{
  
  QApplication         app( argc, argv );
  MapPainter           win;

  REC2_DATA            rec;
  MAP2                 map, localmap;

  char                 rec_filename[MAX_STRING_LENGTH];
  char                 ini_filename[MAX_STRING_LENGTH];

  iVECTOR2             rmappos;
  VECTOR2              min, max, abspt;
  BOUNDING_BOX2        bbox;
  RMOVE2               particlemove, odomove, move;
  RMOVE2               lmove, nomove = {0.0, 0.0, 0.0};
  RPOS2                lastpartpos, lastpos, pos, newpos;
  RPOS2                addpos, ucpos, nullpos = {0.0, 0.0, 0.0};

  double               length = 0.0;
  
  int                  h, hist, i, j, p;
  int                  idxctr = 0, ctr = 0, * idx;
  int                  read_ini = FALSE;
  
  int                  bestvalidx = 0;
  int                  check = FALSE;
 
  SAMPLE_SET           pset, tset;
  double               samplesum;
  
    /*
    double               sizex  = 6000.0, sizey  = 3000.0;
    double               startx = 3000.0, starty = 1300.0;
    int                  resolution = 5;
  
    double               sizex  = 2000.0, sizey  = 2000.0;
    double               startx = 1000.0,   starty = 1700.0;
    */

  
  double               v;
  static GAUSS_KERNEL  kernel   = compute_gauss_kernel( 9 );
  static GAUSS_KERNEL  nokernel = compute_gauss_kernel( 1 );
  
  double               lsizex, lsizey, lstartx, lstarty;
  FILE               * iop;

  for (i=1; i<argc-1; i++) {
    if (!strcmp(argv[i],"-ini") && (argc>i+1)) {
      strncpy( ini_filename, argv[++i], MAX_STRING_LENGTH );
      read_ini = TRUE;
    } else {
      print_usage();
      exit(1);
    }
  }
  
  strncpy( rec_filename, argv[argc-1], MAX_STRING_LENGTH );

  set_default();
  
  if (read_ini)
    read_ini_file( ini_filename );

  lsizex   = 50.0 + 1.0 * settings.max_range_length;
  lsizey   = 50.0 + 2.0 * settings.max_range_length;
  lstartx  = lsizex / 2.0;
  lstarty  = 50.0;

  read_input_file( &rec, rec_filename );

  pset.numparticles = settings.num_samples;
  pset.particle = (PARTICLE *) malloc( pset.numparticles * sizeof(PARTICLE) );
  
  tset.numparticles = settings.num_samples;
  tset.particle = (PARTICLE *) malloc( tset.numparticles * sizeof(PARTICLE) );
  
  for (i=0; i<pset.numparticles; i++) {
    pset.particle[i].pos.x = 0.0;
    pset.particle[i].pos.y = 0.0;
    pset.particle[i].pos.o = 0.0;
    pset.particle[i].histlen = 0;
    pset.particle[i].outside = TRUE;
    pset.particle[i].hist =
      (HISTORY *) malloc( rec.numlaserscans * sizeof(HISTORY) );
    tset.particle[i].histlen = 0;
    tset.particle[i].hist =
      (HISTORY *) malloc( rec.numlaserscans * sizeof(HISTORY) );
  }
  
  min.x =  MAXDOUBLE;  min.y =  MAXDOUBLE;
  max.x = -MAXDOUBLE;  max.y = -MAXDOUBLE;

  idxctr = 0;
  idx = (int *) malloc( rec.numlaserscans * sizeof(int) );

  for (i=0; i<rec.numlaserscans; i++) {
    for (j=0;j<rec.lsens[i].laser.numvalues;j++) {
      if (rec.lsens[i].laser.val[j] > settings.max_range_length ) {
	abspt=compute_laser_abs_point( rec.lsens[i].estpos,
				       settings.max_range_length,
				       nomove, rec.lsens[i].laser.angle[j] );
      } else {
	abspt=compute_laser_abs_point( rec.lsens[i].estpos,
				       rec.lsens[i].laser.val[j],
				       nomove, rec.lsens[i].laser.angle[j] );
      }
      if (abspt.x < min.x) {
	min.x = abspt.x;
      }
      if (abspt.x > max.x) {
	max.x = abspt.x;
      }
      if (abspt.y < min.y) {
	min.y = abspt.y;
      }
      if (abspt.y > max.y) {
	max.y = abspt.y;
      }
    }
    // update_map( &map, rec.lsens[i].laser.numvalues, rec.lsens[i].laser.val, 
    //		rec.lsens[i].laser.angle, rec.lsens[i].estpos,
    //		settings.max_range_length, settings.max_usable_length );
  }

  if (settings.detect_size) {

    settings.size_x  = max.x - min.x + 2.0 * settings.detect_size_border;
    settings.size_y  = max.y - min.y + 2.0 * settings.detect_size_border;

    settings.start_x = -(min.x - settings.detect_size_border);
    settings.start_y = -(min.y - settings.detect_size_border);

    fprintf(stderr,"***************************************\nEstimating map size:\n");
    fprintf(stderr,"  start_x: %.1f\n", settings.start_x);
    fprintf(stderr,"  start_y: %.1f\n", settings.start_y);
    fprintf(stderr,"  size_x : %.1f\n", settings.size_x);
    fprintf(stderr,"  size_y : %.1f\n", settings.size_y);
    fprintf(stderr,"***************************************\n");
  }
   
  map_initialize( &map,
		  (int) (settings.size_x/settings.resolution),
		  (int) (settings.size_y/settings.resolution),
		  (int) (settings.start_x/settings.resolution),
		  (int) (settings.start_y/settings.resolution),
		  settings.resolution, nullpos );
  
  map_initialize( &localmap,
		  (int) (lsizex/settings.resolution),
		  (int) (lsizey/settings.resolution),
		  (int) (0),
		  (int) (lsizey/(2.0*settings.resolution)),
		    settings.resolution, nullpos );
  
  if (settings.show_graphics) {

    win.setMinimumSize( 640, 480 );

    int window_size_x = 100 + (int) (settings.size_x/settings.resolution);
    if (window_size_x > 800)
      window_size_x = 800;

    int window_size_y = 100 + (int) (settings.size_y/settings.resolution);
    if (window_size_y > 600)
      window_size_y = 600;

    win.setSize( window_size_x, window_size_y) ;
    
    win.show();
  }      
  
  
  signal(SIGINT,shutdown);


  /* init all positions (including the particles) */
  lastpos = lastpartpos = rec.lsens[0].estpos;
  for (p=0; p<pset.numparticles;p++) {
    pset.particle[p].pos =
      pset.particle[p].hist[0].pos =
      rec.lsens[0].estpos; 
  }
  
  map_pos_from_rpos( nullpos, &localmap, &rmappos );
  rmappos.x++;
  fprintf( stderr, "-->%d %d\n", rmappos.x, rmappos.y );
  
  for (i=0; i<rec.numlaserscans; i++) {
    
    length += rpos2_length( lastpos, rec.lsens[i].estpos );
    odomove =
      compute_movement2_between_rpos2( lastpos, rec.lsens[i].estpos );
    lastpos = rec.lsens[i].estpos;
    
    for (p=0; i>0 && p<pset.numparticles;p++) {
      pset.particle[p].hist[i].pos =
	compute_rpos2_with_movement2( pset.particle[p].hist[i-1].pos,
				      odomove );
    }
    
    if (length>settings.min_step_distance) {
      
      length = 0.0;
      move = compute_movement2_between_rpos2( lastpartpos,
					      rec.lsens[i].estpos );
      
      for (p=0; p<pset.numparticles;p++) {
	
	/* add noise corresponding to the scan-matcher motion
	   model to the positions */
	
	particlemove = move;
	pos = pset.particle[p].pos;
	ucpos = compute_rpos2_with_movement2( pos, particlemove );
	
	pos.o +=
	  (settings.rotation_noise * move.rotation * random_gauss()) +
	  (settings.sideward_noise * move.forward  * random_gauss());
	
	particlemove.forward +=
	  (settings.forward_noise  * move.forward * random_gauss());

	pset.particle[p].pos =
	  compute_rpos2_with_movement2( pos, particlemove );

	
	addpos.x = pset.particle[p].pos.x - ucpos.x;
	addpos.y = pset.particle[p].pos.y - ucpos.y;
	addpos.o = pset.particle[p].pos.o - ucpos.o;
	pset.particle[p].hist[i].pos =  pset.particle[p].pos;
	
	/* count used scans in history */
	bbox = compute_laser_bounding_box( pset.particle[p].pos,
					   rec.lsens[i],
					   settings.max_range_length );
	pset.particle[p].hist[i].bbox = bbox;
	clear_map( &localmap, nullpos );
	ctr = 0; check = FALSE;
	for (h=0; h<idxctr;h++) {
	  if (intersect_bboxes( bbox, pset.particle[p].hist[idx[h]].bbox )) {
	    lmove =
	      compute_movement2_between_rpos2( pset.particle[p].pos,
					       pset.particle[p].hist[idx[h]].pos );
	    
	    newpos = compute_rpos2_with_movement2( nullpos, lmove );
	    update_map( &localmap,
			rec.lsens[idx[h]].laser.numvalues,
			rec.lsens[idx[h]].laser.val, 
			rec.lsens[idx[h]].laser.angle,
			newpos,
			settings.max_range_length,
			settings.max_usable_length );
	    ctr++;
	  }
	  if (localmap.mapsum[rmappos.x][rmappos.y]>0)
	    pset.particle[p].outside = FALSE;
	  else 
	    pset.particle[p].outside = TRUE;
	    check = TRUE;
	}
	compute_probs_of_map( &localmap );
	if (settings.kernel_size==3)
	    simple_convolve_map2( &localmap );
	else
	    simple_convolve_map( &localmap, kernel );
	
	v = compute_scan_probability( &localmap, nullpos,
				      rec.lsens[i],
				      settings.max_range_length,
				      settings.max_usable_length  );
	pset.particle[p].val = v;
	if (settings.show_graphics && settings.show_local_map && p==bestvalidx)
	  win.update( localmap );
      }
      //bestvalidx = find_best_particle_value( pset );
      bestvalidx = find_best_particle_logsum( pset );
      fprintf( stderr, "best idx = %d value %f   -> scan %d\n", bestvalidx, pset.particle[bestvalidx].val, i );
      lastpartpos = rec.lsens[i].estpos;
      
      if (settings.show_graphics) {
	  map_clear( &map );
	  for (h=idxctr-1;h>=0;h--) {
	      j = idx[h];
	      update_map( &map,
			  rec.lsens[j].laser.numvalues,
			  rec.lsens[j].laser.val, 
			  rec.lsens[j].laser.angle,
			  pset.particle[bestvalidx].hist[j].pos,
			  settings.max_range_length, settings.max_usable_length );
	  }
	  compute_probs_of_map( &map );
	  simple_convolve_map( &map, nokernel );
	  win.update( map );
      }

      /* normalization of the weights */
      samplesum = 0;
      for (p=0; p<pset.numparticles;p++) {
	samplesum += pset.particle[p].val;
	//	fprintf( stderr, "(%d:%f)", p, pset.particle[p].val );
	pset.particle[p].histlen++;
      }
      fprintf( stderr, "\nsamplesum = %f\n", samplesum );
      for (p=0; p<pset.numparticles;p++) {
	copy_particle( pset.particle[p], &tset.particle[p] );
	tset.particle[p].val = pset.particle[p].val / samplesum;
      }
      resample( tset, &pset );

      if (settings.show_graphics) {
	  win.centerView( map, pset.particle[bestvalidx].pos);
	  win.drawparticles( map, pset, 1, 1 );
	  win.drawrobot( map, rec.lsens[i].estpos, 0 );
	  win.drawrobot( map, pset.particle[bestvalidx].pos, 1 );
	  win.doPaint();
      }

      if (settings.dump_screen) {
	  win.dumpscreen();
      }

      /* length > xxx */
      idx[idxctr] =  i;
      idxctr++;
    } else {
      for (p=0; p<pset.numparticles;p++) {
	hist = pset.particle[p].histlen;
	pset.particle[p].hist[hist].bbox =
	  compute_laser_bounding_box( pset.particle[p].hist[hist].pos,
				      rec.lsens[i],
				      settings.max_range_length );
	pset.particle[p].histlen++;
      }
    }
    
    /* regular move */
    app.processEvents();
    usleep(500);
  } 
  
//  bestvalidx = find_best_particle_value( pset );
  bestvalidx = find_best_particle_logsum( pset );

  fprintf( stderr, "writing file %s ...\n", settings.result_filename );
  if ((iop = fopen( settings.result_filename, "w")) == 0){
    fprintf(stderr, " WARNING: can't write rec file %s\n",
	    settings.result_filename );
  } else {
    for (i=0; i<idxctr-1; i++) {
      fprintf( iop, "POS %ld %ld: %f %f %f\n",
	       rec.lsens[idx[i]].laser.time.tv_sec,
	       rec.lsens[idx[i]].laser.time.tv_usec,
	       pset.particle[bestvalidx].hist[idx[i]].pos.x,
	       pset.particle[bestvalidx].hist[idx[i]].pos.y,
	       rad2deg(pset.particle[bestvalidx].hist[idx[i]].pos.o) );
      fprintf( iop, "LASER-RANGE %ld %ld %d %d %.1f:",
	       rec.lsens[idx[i]].laser.time.tv_sec,
	       rec.lsens[idx[i]].laser.time.tv_usec,
	       rec.lsens[idx[i]].id,
	       rec.lsens[idx[i]].laser.numvalues,
	       180.0 );
      for (j=0; j<rec.lsens[idx[i]].laser.numvalues;j++) {
	fprintf( iop, " %.1f", rec.lsens[idx[i]].laser.val[j] );
      }
      fprintf( iop, "\n" );
    }
    fclose(iop);
  }
  
  if (settings.show_graphics) {
    for (h=idxctr-1;h>=0;h--) {
      j = idx[h];
      update_map( &map,
		  rec.lsens[j].laser.numvalues,
		  rec.lsens[j].laser.val, 
		  rec.lsens[j].laser.angle,
		  pset.particle[bestvalidx].hist[j].pos,
		  settings.max_range_length, settings.max_usable_length );
    }
    compute_probs_of_map( &map );
    simple_convolve_map( &map, nokernel );
    win.update( map );
    
    while (1) {
      app.processEvents();
      usleep(500);
    }
  }
  

  exit(0);
}

