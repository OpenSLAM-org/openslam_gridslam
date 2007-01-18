#include <carmen/logtools.h>
#include "map2d.h"

MAP2D_SETTINGS          settings;


void
set_default( void )
{
  settings.use_graphics                    = TRUE;

  settings.pos_diff_min_dist               = 4.0;
  settings.pos_diff_min_rot                = deg2rad(2.0);
  settings.pos_diff_max_rot                = 3.1415;

  settings.add_noise                       = FALSE;
  settings.add_noise_val                   = 0.5;
  settings.noise_type                      = 0;
  settings.random_number                   = 1;

  strncpy( settings.robot_name, "", MAX_NAME_LENGTH );
  settings.using_multiple_laser            = FALSE;
  settings.laser_number                    = 0;
  strncpy( settings.laser_char_type, "not set", MAX_NAME_LENGTH );
  settings.laser_type                      = LMS;
  settings.max_usable_laser_range          = -1;
  
  settings.mode                            = READ_FILE;
  settings.show_updates                    = FALSE;
  
  settings.use_correction                  = TRUE;
  settings.local_map_max_range             = 2000.0;
  settings.local_map_resolution            = 5.0;
  settings.local_map_kernel_len            = 9;
  settings.local_map_num_convolve          = 1;
  settings.local_map_std_val               = 0.01;
  settings.local_map_num_history           = 6000;
  settings.local_map_max_used_history      = 150;
  settings.local_map_min_bbox_distance     = 10.0;
  settings.local_map_history_skip          = 1;
  settings.local_map_object_prob           = 0.99;
  settings.local_map_use_odometry          = ODO_STD;
  settings.local_map_history_skip          = 1;

  settings.use_error_analyze               = FALSE;
  settings.error_max_forward               = 10.0;
  settings.error_forward_step              = 1.0;
  settings.error_max_sideward              = 10.0;
  settings.error_sideward_step             = 1.0;
  settings.error_max_rotation              = deg2rad(10.0);
  settings.error_rotation_step             = deg2rad(1.0);
  settings.error_min_distance              = 200.0;
  
  settings.motion_model.forward            = 1.3;
  settings.motion_model.sideward           = 1.3;
  settings.motion_model.rotation           = deg2rad(20.0);
    
  settings.use_global_map                  = TRUE;
  settings.global_map_max_range            = 1500.0;
  settings.global_map_std_val              = 0.3;
  settings.global_map_size_x               = 1000;
  settings.global_map_size_y               = 1000;
  settings.global_map_start_x              = 500;
  settings.global_map_start_y              = 500;
  settings.global_map_start_pos.x          = 0.0;
  settings.global_map_start_pos.y          = 0.0;
  settings.global_map_start_pos.o          = 0.0;
  settings.global_map_resolution           = 5.0;
  settings.global_map_ray_model            = TRUE;  

  settings.display_robot_size              = 15.0; // 53.0;  /* 21 Inch */
  settings.display_pixel_robot_size =
    (int) (settings.display_robot_size / settings.global_map_resolution);
  
  strncpy( settings.global_map_filename, "map2d.map",
	   MAX_NAME_LENGTH );

  settings.pos_corr_step_size_forward      = 7.5;
  settings.pos_corr_step_size_sideward     = 7.5;
  settings.pos_corr_step_size_rotation     = 0.125;
  settings.pos_corr_step_size_loop         = 10;

  settings.log_output                      = TRUE;
  strncpy( settings.log_filename, "mapper-corrected.clf",
	   MAX_NAME_LENGTH );
  settings.logF                            = NULL;

  settings.max_usable_laser_range          = 8192.0;

  settings.view_path                       = TRUE;
  settings.change_map                      = TRUE;
  settings.loop_sleep                      = 50000;

  settings.dump_maps                       = FALSE;
  strncpy( settings.dump_mapnames, "dump_map_", MAX_NAME_LENGTH );

}

#define MAX_COMMAND_LENGTH      1024

void 
read_ini_file( char *filename ) 
{
 int FEnd;
 FILE *iop;
 char command[MAX_COMMAND_LENGTH];
 char inp[MAX_COMMAND_LENGTH];

 if ((iop = fopen( filename, "r")) == 0){
   fprintf(stderr, "ERROR: could not open ini file %s\n", filename );
   exit(0);
 }
 fprintf(stderr, "* INFO: read ini file %s\n", filename ); 
 FEnd=0;
 do{
   if (fscanf(iop, "%s", command) == EOF)
     FEnd=1;
   else{
     if (!strcmp( command, "USE_GRAPHICS") ){
       if (fscanf(iop, "%d", &settings.use_graphics) == EOF)
	 FEnd=1;
       
     } else if (!strcmp( command, "POS_DIFF_MIN_DIST") ){
       if (fscanf(iop, "%s", inp) == EOF) {
	 FEnd=1;
       } else {
	 settings.pos_diff_min_dist = atof(inp);
       }
     } else if (!strcmp( command, "POS_DIFF_MIN_ROTATION") ){
       if (fscanf(iop, "%s", inp) == EOF) {
	 FEnd=1;
       } else {
	 settings.pos_diff_min_rot = deg2rad(atof(inp));
       }
     } else if (!strcmp( command, "POS_DIFF_MAX_ROTATION") ){
       if (fscanf(iop, "%s", inp) == EOF) {
	 FEnd=1;
       } else {
	 settings.pos_diff_max_rot = atof(inp);
       }

     } else if (!strcmp( command, "ADD_NOISE") ){
       if (fscanf(iop, "%d", &settings.add_noise) == EOF)
	 FEnd=1;
     } else if (!strcmp( command, "ADD_NOISE_VAL") ){
       if (fscanf(iop, "%s", inp) == EOF) {
	 FEnd=1;
       } else {
	 settings.add_noise_val = atof(inp);
       }
     } else if (!strcmp( command, "NOISE_TYPE") ){
       if (fscanf(iop, "%d", &settings.noise_type) == EOF)
	 FEnd=1;
     } else if (!strcmp( command, "RANDOM_NUMBER") ){
       if (fscanf(iop, "%d", &settings.random_number) == EOF)
	 FEnd=1;
     } else if (!strcmp( command, "SHOW_UPDATES") ){
       if (fscanf(iop, "%d", &settings.show_updates) == EOF)
	 FEnd=1;

     } else if (!strcmp( command, "LOOP_SLEEP") ){
       if (fscanf(iop, "%d", &settings.loop_sleep) == EOF)
	 FEnd=1;

     } else if (!strcmp( command, "USE_CORRECTION") ){
       if (fscanf(iop, "%d", &settings.use_correction) == EOF)
	 FEnd=1;
     } else if (!strcmp( command, "LOCAL_MAP_MAX_RANGE") ){
       if (fscanf(iop, "%s", inp) == EOF) {
	 FEnd=1;
       } else {
	 settings.local_map_max_range = atof(inp);
       }
     } else if (!strcmp( command, "LOCAL_MAP_RESOLUTION") ){
       if (fscanf(iop, "%s", inp) == EOF) {
	 FEnd=1;
       } else {
	 settings.local_map_resolution = atof(inp);
      }
     } else if (!strcmp( command, "LOCAL_MAP_USE_ODOMETRY") ){
       if (fscanf(iop, "%d", &settings.local_map_use_odometry) == EOF)
	 FEnd=1;
     } else if (!strcmp( command, "LOCAL_MAP_KERNEL_LENGTH") ){
       if (fscanf(iop, "%d", &settings.local_map_kernel_len) == EOF)
	 FEnd=1;
     } else if (!strcmp( command, "LOCAL_MAP_NUM_CONVOLVE") ){
       if (fscanf(iop, "%d", &settings.local_map_num_convolve) == EOF)
	 FEnd=1;
     } else if (!strcmp( command, "LOCAL_MAP_UNKNOWN_VAL") ){
       if (fscanf(iop, "%s", inp) == EOF) {
	 FEnd=1;
       } else {
	 settings.local_map_std_val = atof(inp);
       }
     } else if (!strcmp( command, "LOCAL_MAP_HISTORY_LENGTH") ){
       if (fscanf(iop, "%d", &settings.local_map_num_history) == EOF)
	 FEnd=1;
     } else if (!strcmp( command, "LOCAL_MAP_HISTORY_SKIP") ){
       if (fscanf(iop, "%d", &settings.local_map_history_skip) == EOF)
	 FEnd=1;
     } else if (!strcmp( command, "LOCAL_MAP_MAX_USED_HISTORY") ){
       if (fscanf(iop, "%d", &settings.local_map_max_used_history) == EOF)
	 FEnd=1;
     } else if (!strcmp( command, "LOCAL_MAP_MIN_BBOX_DISTANCE") ){
       if (fscanf(iop, "%s", inp) == EOF)
	 FEnd=1;
       else
	 settings.local_map_min_bbox_distance = atof(inp);
     } else if (!strcmp( command, "POS_CORRECTION_FORWARD_STEP") ){
       if (fscanf(iop, "%s", inp) == EOF) {
	 FEnd=1;
       } else {
	 settings.pos_corr_step_size_forward = atof(inp);
       }
     } else if (!strcmp( command, "POS_CORRECTION_SIDEWARD_STEP") ){
       if (fscanf(iop, "%s", inp) == EOF) {
	 FEnd=1;
       } else {
	 settings.pos_corr_step_size_sideward = atof(inp);
       }
     } else if (!strcmp( command, "POS_CORRECTION_ROTATION_STEP") ){
       if (fscanf(iop, "%s", inp) == EOF) {
	 FEnd=1;
       } else {
	 settings.pos_corr_step_size_rotation = atof(inp);
       }
     } else if (!strcmp( command, "POS_CORRECTION_NUM_DECREASE_LOOP") ){
       if (fscanf(iop, "%d", &settings.pos_corr_step_size_loop) == EOF)
	 FEnd=1;
     } else if (!strcmp( command, "MOTION_MODEL_FORWARD") ){
       if (fscanf(iop, "%s", inp) == EOF)
	 FEnd=1;
       else
	 settings.motion_model.forward = atof(inp);
     } else if (!strcmp( command, "MOTION_MODEL_SIDEWARD") ){
       if (fscanf(iop, "%s", inp) == EOF)
	 FEnd=1;
       else
	 settings.motion_model.sideward = atof(inp);
     } else if (!strcmp( command, "MOTION_MODEL_ROTATION") ){
       if (fscanf(iop, "%s", inp) == EOF)
	 FEnd=1;
       else
	 settings.motion_model.rotation = atof(inp);

     } else if (!strcmp( command, "LOG_OUTPUT") ){
       if (fscanf(iop, "%d", &settings.log_output) == EOF)
	 FEnd=1;
     } else if (!strcmp( command, "LOG_FILENAME") ){
       if (fscanf(iop, "%s", inp) == EOF) {
	 FEnd=1;
       } else {
	 strncpy( settings.log_filename, inp, MAX_NAME_LENGTH );
       }
     } else if (!strcmp( command, "USE_GLOBAL_MAP") ){
       if (fscanf(iop, "%d", &settings.use_global_map) == EOF)
	 FEnd=1;
     } else if (!strcmp( command, "GLOBAL_MAP_FILENAME") ){
       if (fscanf(iop, "%s", inp) == EOF) {
	 FEnd=1;
       } else {
	 strncpy( settings.global_map_filename, inp, MAX_NAME_LENGTH );
       }
     } else if (!strcmp( command, "GLOBAL_MAP_MAX_RANGE") ){
       if (fscanf(iop, "%s", inp) == EOF) {
	 FEnd=1;
       } else {
	 settings.global_map_max_range = atof(inp);
       }
     } else if (!strcmp( command, "GLOBAL_MAP_SIZE_X") ){
       if (fscanf(iop, "%d", &settings.global_map_size_x) == EOF) {
	 FEnd=1;
       } 
     } else if (!strcmp( command, "GLOBAL_MAP_SIZE_Y") ){
       if (fscanf(iop, "%d", &settings.global_map_size_y) == EOF) {
	 FEnd=1;
       } 
     } else if (!strcmp( command, "GLOBAL_MAP_START_X") ){
       if (fscanf(iop, "%d", &settings.global_map_start_x) == EOF) {
	 FEnd=1;
       } 
     } else if (!strcmp( command, "GLOBAL_MAP_START_Y") ){
       if (fscanf(iop, "%d", &settings.global_map_start_y) == EOF) {
	 FEnd=1;
       } 
     } else if (!strcmp( command, "GLOBAL_MAP_RESOLUTION") ){
       if (fscanf(iop, "%s", inp) == EOF) {
	 FEnd=1;
       } else {
	 settings.global_map_resolution = atof(inp);
      }
     } else if (!strcmp( command, "GLOBAL_MAP_UNKNOWN_VAL") ){
       if (fscanf(iop, "%s", inp) == EOF) {
	 FEnd=1;
       } else {
	 settings.global_map_std_val = atof(inp);
       }
     } else if (!strcmp( command, "GLOBAL_MAP_RAY_MODEL") ){
       if (fscanf(iop, "%d", &settings.global_map_ray_model) == EOF) {
	 FEnd=1;
       } 
     } else if (!strcmp( command, "GLOBAL_MAP_START_ROBOT_POS_X") ){
       if (fscanf(iop, "%s", inp) == EOF) {
	 FEnd=1;
       } else {
	 settings.global_map_start_pos.x = atof(inp);
       }
     } else if (!strcmp( command, "GLOBAL_MAP_START_ROBOT_POS_Y") ){
       if (fscanf(iop, "%s", inp) == EOF) {
	 FEnd=1;
       } else {
	 settings.global_map_start_pos.y = atof(inp);
       }
     } else if (!strcmp( command, "GLOBAL_MAP_START_ROBOT_POS_O") ){
       if (fscanf(iop, "%s", inp) == EOF) {
	 FEnd=1;
       } else {
	 settings.global_map_start_pos.o = deg2rad(atof(inp));
       }
       
     } else if (!strcmp( command, "DISPLAY_ROBOT_SIZE") ){
       if (fscanf(iop, "%s", inp) == EOF) {
	 FEnd=1;
       } else {
	 settings.display_robot_size = atof(inp);
      }

     } else if (!strcmp( command, "LASER_TYPE") ){
       if (fscanf(iop, "%s", inp) == EOF) {
	 FEnd=1;
       } else {
	 strncpy( settings.laser_char_type, inp, MAX_NAME_LENGTH );
       }
     } else if (!strcmp( command, "LASER_MAX_USABLE_RANGE") ){
       if (fscanf(iop, "%s", inp) == EOF) {
	 FEnd=1;
       } else {
	 settings.max_usable_laser_range = atof(inp);
       }
     } else if (!strcmp( command, "LASER_ID") ){
       if (fscanf(iop, "%s", inp) == EOF) {
	 FEnd=1;
       } else {
	 settings.laser_number = atoi(inp);
       }
     } else if (!strcmp( command, "LASER_DIRECTION") ){
       if (fscanf(iop, "%s", inp) == EOF) {
	 FEnd=1;
       } else {
	 settings.laser_direction = atoi(inp);
       }
     } else if (!strcmp( command, "ROBOT_NAME") ){
       if (fscanf(iop, "%s", inp) == EOF) {
	 FEnd=1;
       } else {
	 strncpy( settings.robot_name, inp, MAX_NAME_LENGTH );
      }

     } else if (!strcmp( command, "LOCAL_MAP_DUMP_MAPS") ){
       if (fscanf(iop, "%d", &settings.dump_maps) == EOF)
	 FEnd=1;
     } else if (!strcmp( command, "LOCAL_MAP_DUMP_FILENAME") ){
       if (fscanf(iop, "%s", inp) == EOF) {
	 FEnd=1;
       } else {
	 strncpy( settings.dump_mapnames, inp, MAX_NAME_LENGTH );
       }

     } else {
       if (!(command[0]=='#')){
	 fprintf( stderr, "ERROR unknown keyword %s\n", command );
	 fclose(iop);
	 exit(0);
       } else 
	 fgets(command,sizeof(command),iop);
     }
   }
 } while (!FEnd);
 fclose(iop);
 
}

void
check_settings( void )
{
  settings.display_pixel_robot_size =
    (int) (settings.display_robot_size / settings.global_map_resolution);
  
  if (settings.global_map_start_x != -1) {
    settings.global_map_start_x =
      settings.global_map_size_x - settings.global_map_start_x;
  } else {
    settings.global_map_start_x = settings.global_map_size_x / 2;
  }
  if (settings.global_map_start_y != -1) {
    settings.global_map_start_y =
      settings.global_map_size_y - settings.global_map_start_y;
  } else {
    settings.global_map_start_y = settings.global_map_size_y / 2;
  }

  if (strcmp( settings.laser_char_type, "not set" ) ) {
    if (!strcmp( settings.laser_char_type, "PLS" )) {
      settings.laser_type              = PLS;
      if (settings.max_usable_laser_range  == -1)
	settings.max_usable_laser_range  = 5106.0;
    } else if (!strcmp( settings.laser_char_type, "LMS" )) {
      settings.laser_type              = LMS;
      if (settings.max_usable_laser_range  == -1)
	settings.max_usable_laser_range  = 8184.0;
    } else {
      fprintf( stderr, "ERROR: unknown LASER_TYPE %s\n",
	       settings.laser_char_type );
      fprintf( stderr, "       valid types: PLS, LMS\n" );
      exit(1);
    }
  } else {
    if ( settings.laser_type == LMS ) {
      if (settings.max_usable_laser_range  == -1)
	settings.max_usable_laser_range  = 8184.0;
    } else {
      if (settings.max_usable_laser_range  == -1)
	settings.max_usable_laser_range  = 5106.0;
    }
  }

  /*
    LMS - Scanueberlauf Meßbereich bis 8m/80m:

      Wert            Bedeutung
      
    8191 0x1FFF     Messwert nicht gültig ( Zaehler erhielt kein Stoppsignal )
    8190 0x1FFE     Blendung (Hardware meldet Blendung)
    8189 0x1FFD     Operationsüberlauf
                    ( SW-Berechnungsüberläufe, Imp.breite < Tabellen-anfang )
    8187 0x1FFB     Signal Rauschabstand zu klein
                    ( Empfangssignal < Peak- & > Stopp-schwelle )
    8186 0x1FFA     Fehler beim Lesen Kanal1
    8183 0x1FF7     Messwert größer Maximalwert
  */
}




