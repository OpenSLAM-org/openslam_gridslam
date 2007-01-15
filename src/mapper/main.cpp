#include <qapplication.h>

#ifdef __cplusplus
extern "C" {
#endif

#include <navigation/utils.h>

#ifdef __cplusplus
}
#endif

#if QT_VERSION >= 300
#include "gui.h"
#else
#include "gui-2.3.h"
#endif
#include "mapview.h"

#include <qvariant.h>
#include <qframe.h>
#include <qgroupbox.h>
#include <qpushbutton.h>
#include <qscrollbar.h>
#include <qmime.h>
#include <qdragobject.h>
#include <qlayout.h>
#include <qtooltip.h>
#include <qwhatsthis.h>
#include <qaction.h>
#include <qmenubar.h>
#include <qpopupmenu.h>
#include <qtoolbar.h>
#include <qimage.h>
#include <qpixmap.h>

#ifdef __cplusplus
extern "C" {
#endif

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include <math.h>
#include <values.h>
#include <unistd.h>
#include <termios.h>
#include <fcntl.h>
#include <sys/signal.h>
#include <sys/types.h>
#include <sys/time.h>
#include <time.h>
#include <sys/ioctl.h>

#include "map2d.h"

MAP2D_SETTINGS          settings;

MAP2                    * global_map;
MAP2                    * global_ray_map;
MAP2                    * local_map;
MAP2                    * local_ray_map;

LASERSENS2_DATA         * current_scan;
RMOVE2                  * current_movement;

#ifdef __cplusplus
}
#endif

QApplication            * papp;
MapGUI                  * wptr;

void
initialize_maps( MapGUI * window, MAP2 * local_map, MAP2 * ray_map,
		 MAP2 * global_map, MAP2 * global_ray_map )
{
  int         size_x, size_y;
  RPOS2       npos = { 0.0, 0.0, 0.0 };
  
  fprintf( stderr, "***************************************\n" );
  fprintf( stderr, "*        MAPS\n" );
  fprintf( stderr, "***************************************\n" );

  if (settings.use_correction) {
    size_x = (int) ceil((ADDITIONAL_X_SIZE+settings.local_map_max_range)/
			settings.local_map_resolution);
    size_y = (int) ceil((ADDITIONAL_Y_SIZE+settings.local_map_max_range)/
			settings.local_map_resolution);
    fprintf( stderr, "* INFO: create -local- map: %d x %d\n", 2*size_x, size_y );
    initialize_map( local_map, 2*size_x, size_y, 60, size_y/2,
		    settings.local_map_resolution, npos );
    fprintf( stderr, "***************************************\n" );
  }
  
  if (settings.use_local_ray_map) {
    size_x = (int) ceil((ADDITIONAL_X_SIZE+settings.local_ray_map_max_range)/
			settings.local_ray_map_resolution);
    size_y = (int) ceil((ADDITIONAL_Y_SIZE+settings.local_ray_map_max_range)/
			settings.local_ray_map_resolution);
    fprintf( stderr, "* INFO: create -local ray- map: %d x %d\n", size_x, size_y );
    initialize_map( ray_map, size_x, size_y, 20, size_y/2,
		    settings.local_ray_map_resolution, npos );
    fprintf( stderr, "***************************************\n" );
  }
  
  if (settings.use_global_map) {
    fprintf( stderr, "* INFO: create -global- map: %d x %d\n",
	     settings.global_map_size_x, settings.global_map_size_y );
    initialize_global_map( global_map,
			   settings.global_map_size_x,
			   settings.global_map_size_y,
			   settings.global_map_start_x,
			   settings.global_map_start_y,
			   settings.global_map_resolution,
			   settings.global_map_start_pos, 0 );
    fprintf( stderr, "***************************************\n" );
  }
  
  if (settings.use_global_ray_map) {
    fprintf( stderr, "* INFO: create -global ray- map: %d x %d\n",
	     settings.global_ray_map_size_x,
	     settings.global_ray_map_size_y );
    initialize_global_map( global_ray_map,
			   settings.global_ray_map_size_x,
			   settings.global_ray_map_size_y,
			   settings.global_ray_map_start_x,
			   settings.global_ray_map_start_y,
			   settings.global_ray_map_resolution,
			   settings.global_ray_map_start_pos, 1 );
    fprintf( stderr, "***************************************\n" );
  }
  
  if (settings.use_graphics) {
    fprintf( stderr, "* INFO: set graphics -global- map size\n" );
    fprintf( stderr, "***************************************\n" );
    window->Map->setGlobalSize( global_map->mapsize.x, global_map->mapsize.y );
    if (settings.use_correction) {
      fprintf( stderr, "* INFO: set graphics -local- map size\n" );
      fprintf( stderr, "***************************************\n" );
      window->Map->setLocalSize( local_map->mapsize.x, local_map->mapsize.y );
    }
    if (settings.use_local_ray_map) {
      fprintf( stderr, "* INFO: set graphics -local ray- map size\n" );
      fprintf( stderr, "***************************************\n" );
      window->Map->setLocalRaySize( ray_map->mapsize.x, ray_map->mapsize.y );
    }
    if (settings.use_global_ray_map) {
      fprintf( stderr, "* INFO: set graphics -global ray- map size\n" );
      fprintf( stderr, "***************************************\n" );
      window->Map->setGlobalRaySize( global_ray_map->mapsize.x,
				     global_ray_map->mapsize.y );
    }
    fprintf( stderr, "* INFO: show maps\n" );
    fprintf( stderr, "***************************************\n" );
    
    if (settings.use_correction) {
      window->LocalMapType->setEnabled( TRUE );
    } else {
      window->LocalMapType->setEnabled( FALSE );
      window->LocalMapType->hide();
    }
    
    if (settings.use_local_ray_map) {
      window->RayMapType->setEnabled( TRUE );
    } else {
      window->RayMapType->setEnabled( FALSE );
      window->RayMapType->hide();
    }
    
    if (settings.use_global_ray_map) {
      window->GlobalRayMapType->setEnabled( TRUE );
    } else {
      window->GlobalRayMapType->setEnabled( FALSE );
      window->GlobalRayMapType->hide();
    }
    
    window->Map->maptype = GLOBAL_MAP;
    
    window->show();
    window->Map->updateMap();

  }
}

int
window_maptype( void )
{
  return(wptr->Map->maptype);
}

void
window_show_rays( void )
{
  wptr->Map->showRays();
}

void
app_process_events( void )
{
  papp->processEvents();
}

void
paint_robot( RPOS2 pos )
{
  wptr->Map->paintRobot( pos );
}

void
plot_robot( RPOS2 pos )
{
  wptr->Map->plotRobotPosition( pos );
}

void
update_map( void )
{
  wptr->Map->updateMap();
}

void
update_change_map( void )
{
  wptr->ChangeMapAction->setOn(change_map);
}

void
center_robot( void )
{
  wptr->Map->centerRobot();
}

void
print_usage( void )
{
  fprintf( stderr,
	   "usage: map2d [-ini <INI-FILE>] [-file <SCRIPT-FILE>]\n" );
}

int
main( int argc, char *argv[] )
{
  QApplication            app( argc, argv );
  MapGUI                  window;

  int                     i;

  MAP2                    lmap;
  MAP2                    rmap;
  MAP2                    gmap;
  MAP2                    grmap;
  
  
  int                     ini_file = FALSE;
  char                    ini_filename[MAX_NAME_LENGTH];
  
  set_default();

  local_map        = &lmap;
  global_map       = &gmap;
  global_ray_map   = &grmap;
  local_ray_map    = &rmap;

  wptr             = &window;
  papp             = &app;

  settings.mode    = ONLINE;
  
  if (argc>5) {
    print_usage();
    exit(1);
  }
  
  for (i=1; i<argc; i++) {
    if (!strcmp(argv[i],"-ini") && (argc>i+1)) {
      strncpy( ini_filename, argv[++i], MAX_NAME_LENGTH );
      ini_file = TRUE;
    } else if (!strcmp(argv[i],"-file") && (argc>i+1)) {
      strncpy( settings.data_filename, argv[++i], MAX_NAME_LENGTH );
      settings.mode = READ_FILE;
    } else if (!strcmp(argv[i],"-filestream") && (argc>i+1)) {
      strncpy( settings.data_filename, argv[++i], MAX_NAME_LENGTH );
      settings.mode = READ_LINES;
    } else {
      print_usage();
      exit(1);
    }
  }
  
  if (ini_file) {
    read_ini_file( ini_filename );
  }
  
  check_settings();

  fprintf( stderr, "INFO: global map max range:       %.1f\n",
	   settings.global_map_max_range );
  fprintf( stderr, "INFO: global map resolution:      %.1f\n",
	   settings.global_map_resolution );
  fprintf( stderr, "INFO: global ray map max range:   %.1f\n",
	   settings.global_ray_map_max_range );
  fprintf( stderr, "INFO: global ray map resolution:  %.1f\n",
	   settings.global_ray_map_resolution );
  
  if (create_output_files() != 0) {
    exit(1);
  }

  initialize_maps( &window, &lmap, &rmap, &gmap, &grmap );

  fprintf( stderr, "XXXXXXXXXXXXXXXXXXXXX\n" );
  
  switch(settings.mode) {
  case READ_FILE:
    run_data_file( settings, &lmap, &rmap, &gmap, &grmap );
    break;
  case ONLINE:
    settings.script_type = REC;
    run_online( settings, &lmap, &rmap, &gmap, TRUE );
    break;
  case READ_LINES:
    run_online( settings, &lmap, &rmap, &gmap, FALSE );
    break;
  }
  
  close_output_files();
  
  return(0);
  
}
