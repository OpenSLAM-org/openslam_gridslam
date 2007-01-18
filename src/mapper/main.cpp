#include <qapplication.h>
#include <qaction.h>

#include "gui.h"
#include "mapview.h"

QApplication            * papp;
MapGUI                  * wptr;

#ifdef __cplusplus
extern "C" {
#endif

#include <carmen/carmen.h>
#include <carmen/logtools.h>
#include "map2d.h"

MAP2                       * global_map;
MAP2                       * local_map;

extern logtools_lasersens2_data_t * current_scan;
extern logtools_rmove2_t          * current_movement;

void
initialize_maps( MapGUI * window, MAP2 * local_map, MAP2 * global_map )
{
  int                  size_x, size_y;
  logtools_rpos2_t     npos = { 0.0, 0.0, 0.0 };
  
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
  
  if (settings.use_graphics) {
    fprintf( stderr, "* INFO: set graphics -global- map size\n" );
    fprintf( stderr, "***************************************\n" );
    window->Map->setGlobalSize( global_map->mapsize.x, global_map->mapsize.y );
    if (settings.use_correction) {
      fprintf( stderr, "* INFO: set graphics -local- map size\n" );
      fprintf( stderr, "***************************************\n" );
      window->Map->setLocalSize( local_map->mapsize.x, local_map->mapsize.y );
    }
    fprintf( stderr, "* INFO: show maps\n" );
    fprintf( stderr, "***************************************\n" );
    
    if (settings.use_correction) {
      window->LocalMapType->setEnabled( TRUE );
    } else {
      window->LocalMapType->setEnabled( FALSE );
      window->LocalMapType->hide();
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
paint_robot( logtools_rpos2_t pos )
{
  wptr->Map->paintRobot( pos );
}

void
plot_robot( logtools_rpos2_t pos )
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

#ifdef __cplusplus
}
#endif

void
print_usage( void )
{
  fprintf( stderr,
	   "usage: map2d [-ini <INI-FILE>] [-file <LOG-FILE>]\n" );
}

int
main( int argc, char *argv[] )
{
  QApplication            app( argc, argv );
  MapGUI                  window;

  int                     i;

  MAP2                    lmap;
  MAP2                    gmap;
  
  
  int                     ini_file = FALSE;
  char                    ini_filename[MAX_NAME_LENGTH];
  
  set_default();

  local_map        = &lmap;
  global_map       = &gmap;

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
  fprintf( stderr, "INFO: global map size:            %d %d\n",
	   settings.global_map_size_x,
	   settings.global_map_size_y );
  fprintf( stderr, "INFO: global map start:           %d %d\n",
	   settings.global_map_start_x,
	   settings.global_map_start_y );
  fprintf( stderr, "INFO: global map resolution:      %.1f\n",
	   settings.global_map_resolution );
  
  if (create_output_files() != 0) {
    exit(1);
  }

  initialize_maps( &window, &lmap, &gmap );

  switch(settings.mode) {
  case READ_FILE:
    run_data_file( settings, &lmap, &gmap );
    break;
  case ONLINE:
    run_online( settings, &lmap, &gmap, argc, argv );
    break;
  }
  
  close_output_files();
  
  return(0);
  
}
