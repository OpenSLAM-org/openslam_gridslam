#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <sys/types.h>
#include <sys/time.h>
#include <time.h>
#include <signal.h>
#include <unistd.h>
#include <math.h>
#include <string.h>
#include <ctype.h>
#include <values.h>

#include <navigation/utils.h>
#include "map2d.h"

MAP2D_SETTINGS          settings;


void
set_default( void )
{
  settings.use_graphics                    = TRUE;

  settings.communication                   = IPC;

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
  
  settings.mode                            = READ_LINES;
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

  settings.dynprob_prior                   = 0.96;
  settings.max_dynamic_prob                = 0.5;

  /*
    settings.use_local_ray_map               = TRUE;
    settings.local_ray_map_max_range         = 1000.0;
    settings.local_ray_map_resolution        = 15.0;
    settings.local_ray_map_kernel_len        = 3;
    settings.local_ray_map_num_convolve      = 13;
  */
  /* 5, 13 works */

  settings.use_error_analyze               = FALSE;
  settings.error_max_forward               = 10.0;
  settings.error_forward_step              = 1.0;
  settings.error_max_sideward              = 10.0;
  settings.error_sideward_step             = 1.0;
  settings.error_max_rotation              = deg2rad(10.0);
  settings.error_rotation_step             = deg2rad(1.0);
  settings.error_min_distance              = 200.0;
  
  settings.use_local_ray_map               = FALSE;
  settings.local_ray_map_max_range         = 2000.0;
  settings.local_ray_map_resolution        = 7.5;
  settings.local_ray_map_kernel_len        = 3;
  settings.local_ray_map_num_convolve      = 15;
  settings.local_ray_map_dynamic_prob      = 0.5001;
  settings.local_ray_map_static_prob       = 0.4999;
  settings.local_ray_map_shorten_ray       = 2;
  
  settings.motion_model.forward            = 1.3;
  settings.motion_model.sideward           = 1.3;
  settings.motion_model.rotation           = deg2rad(20.0);
    
  settings.use_global_map                  = TRUE;
  settings.global_map_max_range            = 1500.0;
  settings.global_map_std_val              = 0.3;
  settings.global_map_size_x               = 1000;
  settings.global_map_size_y               = 1000;
  settings.global_map_start_x              = -1;
  settings.global_map_start_y              = -1;
  settings.global_map_start_pos.x          = 0.0;
  settings.global_map_start_pos.y          = 0.0;
  settings.global_map_start_pos.o          = 0.0;
  settings.global_map_ray_model            = TRUE;  

  settings.use_global_ray_map              = FALSE;
  settings.global_ray_map_beam_diff        = 100.0;
  settings.global_ray_map_max_range        = 1500.0;
  settings.global_ray_map_clip_val         = 0.6;
  settings.global_ray_map_std_val          = 0.9;
  settings.global_ray_map_size_x           = 1000;
  settings.global_ray_map_size_y           = 1000;
  settings.global_ray_map_start_x          = -1;
  settings.global_ray_map_start_y          = -1;
  settings.global_ray_map_start_pos.x      = 0.0;
  settings.global_ray_map_start_pos.y      = 0.0;
  settings.global_ray_map_start_pos.o      = 0.0;
  settings.global_ray_map_dynamic_prob     = 0.2;
  settings.global_ray_map_static_prob      = 0.96;
  settings.global_ray_map_shorten_ray      = 4;
  
  settings.display_robot_size              = 15.0; // 53.0;  /* 21 Inch */
  settings.display_pixel_robot_size =
    settings.display_robot_size / settings.global_map_resolution;
  
  strncpy( settings.global_map_filename, "map2d.map",
	   MAX_NAME_LENGTH );

  settings.pos_corr_step_size_forward      = 7.5;
  settings.pos_corr_step_size_sideward     = 7.5;
  settings.pos_corr_step_size_rotation     = 0.125;
  settings.pos_corr_step_size_loop         = 10;

  settings.use_people_prob                 = FALSE;
  settings.people_prob_kernel_len          = 5;
  settings.people_prob_num_convolve        = 0;

  settings.output_statistics               = TRUE;
  strncpy( settings.statistics_filename, "map2d.stat",
	   MAX_NAME_LENGTH );  
  settings.statisticsF                     = NULL;

  settings.output_data_map                 = FALSE;
  strncpy( settings.output_data_map_filename, "map2d-data.dump",
	   MAX_NAME_LENGTH );
  settings.outputF                         = NULL;
  
  settings.dump_probabilities              = FALSE;
  strncpy( settings.prob_filename, "prob.dat", MAX_NAME_LENGTH );
  settings.probF                           = NULL;

  settings.script_output                   = FALSE;
  strncpy( settings.script_out_filename, "map2d-corrected.script",
	   MAX_NAME_LENGTH );
  settings.scriptF                         = NULL;

  settings.experiment_log                  = FALSE;
  strncpy( settings.experiment_log_filename,
	   "experiment.log", MAX_NAME_LENGTH );
  settings.experimentF                     = NULL;

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
 char communicationtype[MAX_COMMAND_LENGTH];

 strcpy( communicationtype,   "" );

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
       
     } else if (!strcmp( command, "COMMUNICATION_TYPE") ){
       if (fscanf(iop, "%s", inp) == EOF) {
	 FEnd=1;
	} else {
	  strncpy( communicationtype, inp, MAX_NAME_LENGTH );
	}
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
     } else if (!strcmp( command, "MAX_DYNPROB_CHANGE") ){
       if (fscanf(iop, "%s", inp) == EOF)
	 FEnd=1;
       else
	 settings.max_dynprob_change = atof(inp);
     } else if (!strcmp( command, "DYNPROB_PRIOR") ){
       if (fscanf(iop, "%s", inp) == EOF)
	 FEnd=1;
       else
	 settings.dynprob_prior = atof(inp);
     } else if (!strcmp( command, "MAY_DYNAMIC_PROB") ){
       if (fscanf(iop, "%s", inp) == EOF) {
	 FEnd=1;
       } else {
	 settings.max_dynamic_prob = atof(inp);
       }
     } else if (!strcmp( command, "USE_LOCAL_RAY_MAP") ){
       if (fscanf(iop, "%d", &settings.use_local_ray_map) == EOF)
	 FEnd=1;
     } else if (!strcmp( command, "LOCAL_RAY_MAP_MAX_RANGE") ){
       if (fscanf(iop, "%s", inp) == EOF)
	 FEnd=1;
       else
	 settings.local_ray_map_max_range = atof(inp);
     } else if (!strcmp( command, "LOCAL_RAY_MAP_RESOLUTION") ){
       if (fscanf(iop, "%s", inp) == EOF)
	 FEnd=1;
       else
	 settings.local_ray_map_resolution = atof(inp);
     } else if (!strcmp( command, "LOCAL_RAY_MAP_KERNEL_LENGTH") ){
       if (fscanf(iop, "%s", inp) == EOF)
	 FEnd=1;
       else
	 settings.local_ray_map_kernel_len = atof(inp);
     } else if (!strcmp( command, "LOCAL_RAY_MAP_SHORTEN_RAY") ){
       if (fscanf(iop, "%d", &settings.local_ray_map_shorten_ray) == EOF)
	 FEnd=1;
     } else if (!strcmp( command, "LOCAL_RAY_MAP_NUM_CONVOLVE") ){
       if (fscanf(iop, "%s", inp) == EOF)
	 FEnd=1;
       else
	  settings.local_ray_map_num_convolve = atof(inp);
     } else if (!strcmp( command, "LOCAL_RAY_MAP_HISTORY_LENGTH") ){
       if (fscanf(iop, "%d", &settings.local_ray_map_num_history) == EOF)
	 FEnd=1;
     } else if (!strcmp( command, "LOCAL_RAY_MAP_MAX_USED_HISTORY") ){
       if (fscanf(iop, "%d", &settings.local_ray_map_max_used_history) == EOF)
	 FEnd=1;
     } else if (!strcmp( command, "LOCAL_RAY_MAP_MIN_BBOX_DISTANCE") ){
       if (fscanf(iop, "%s", inp) == EOF)
	 FEnd=1;
       else
	 settings.local_ray_map_min_bbox_distance = atof(inp);

     } else if (!strcmp( command, "USE_PEOPLE_PROB") ){
       if (fscanf(iop, "%d", &settings.use_people_prob) == EOF)
	 FEnd=1;
     } else if (!strcmp( command, "PEOPLE_PROB_KERNEL_LENGTH") ){
       if (fscanf(iop, "%d", &settings.people_prob_kernel_len) == EOF)
	 FEnd=1;
     } else if (!strcmp( command, "PEOPLE_PROB_NUM_CONVOLVE") ){
       if (fscanf(iop, "%d", &settings.people_prob_num_convolve) == EOF)
	 FEnd=1;

     } else if (!strcmp( command, "OUTPUT_SCAN_PROBABILITIES") ){
       if (fscanf(iop, "%d", &settings.dump_probabilities) == EOF)
	 FEnd=1;
     } else if (!strcmp( command, "OUTPUT_SCAN_PROB_FILENAME") ){
       if (fscanf(iop, "%s", inp) == EOF) {
	 FEnd=1;
       } else {
	 strncpy( settings.prob_filename, inp, MAX_NAME_LENGTH );
       }

     } else if (!strcmp( command, "OUTPUT_CORRECTED_SCRIPT") ){
       if (fscanf(iop, "%d", &settings.script_output) == EOF)
	 FEnd=1;
     } else if (!strcmp( command, "OUTPUT_CORR_SCRIPT_FILENAME") ){
       if (fscanf(iop, "%s", inp) == EOF) {
	 FEnd=1;
       } else {
	 strncpy( settings.script_out_filename, inp, MAX_NAME_LENGTH );
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


     } else if (!strcmp( command, "USE_GLOBAL_RAY_MAP") ){
       if (fscanf(iop, "%d", &settings.use_global_ray_map) == EOF) {
	 FEnd=1;
       } 
     } else if (!strcmp( command, "GLOBAL_RAY_MAP_MAX_RANGE") ){
       if (fscanf(iop, "%s", inp) == EOF) {
	 FEnd=1;
       } else {
	 settings.global_ray_map_max_range = atof(inp);
       }
     } else if (!strcmp( command, "GLOBAL_RAY_MAP_SIZE_X") ){
       if (fscanf(iop, "%d", &settings.global_ray_map_size_x) == EOF) {
	 FEnd=1;
       } 
     } else if (!strcmp( command, "GLOBAL_RAY_MAP_SIZE_Y") ){
       if (fscanf(iop, "%d", &settings.global_ray_map_size_y) == EOF) {
	 FEnd=1;
       } 
     } else if (!strcmp( command, "GLOBAL_RAY_MAP_START_X") ){
       if (fscanf(iop, "%d", &settings.global_ray_map_start_x) == EOF) {
	 FEnd=1;
       } 
     } else if (!strcmp( command, "GLOBAL_RAY_MAP_START_Y") ){
       if (fscanf(iop, "%d", &settings.global_ray_map_start_y) == EOF) {
	 FEnd=1;
       } 
     } else if (!strcmp( command, "GLOBAL_RAY_MAP_RESOLUTION") ){
       if (fscanf(iop, "%s", inp) == EOF) {
	 FEnd=1;
       } else {
	 settings.global_ray_map_resolution = atof(inp);
      }
     } else if (!strcmp( command, "GLOBAL_RAY_MAP_UNKNOWN_VAL") ){
       if (fscanf(iop, "%s", inp) == EOF) {
	 FEnd=1;
       } else {
	 settings.global_ray_map_std_val = atof(inp);
       }
     } else if (!strcmp( command, "GLOBAL_RAY_MAP_START_ROBOT_POS_X") ){
       if (fscanf(iop, "%s", inp) == EOF) {
	 FEnd=1;
       } else {
	 settings.global_ray_map_start_pos.x = atof(inp);
       }
     } else if (!strcmp( command, "GLOBAL_RAY_MAP_START_ROBOT_POS_Y") ){
       if (fscanf(iop, "%s", inp) == EOF) {
	 FEnd=1;
       } else {
	 settings.global_ray_map_start_pos.y = atof(inp);
       }
     } else if (!strcmp( command, "GLOBAL_RAY_MAP_START_ROBOT_POS_O") ){
       if (fscanf(iop, "%s", inp) == EOF) {
	 FEnd=1;
       } else {
	 settings.global_ray_map_start_pos.o = deg2rad(atof(inp));
       }
     } else if (!strcmp( command, "GLOBAL_RAY_MAP_SHORTEN_RAY") ){
       if (fscanf(iop, "%d", &settings.global_ray_map_shorten_ray) == EOF) {
	 FEnd=1;
       } 
       
     } else if (!strcmp( command, "DISPLAY_ROBOT_SIZE") ){
       if (fscanf(iop, "%s", inp) == EOF) {
	 FEnd=1;
       } else {
	 settings.display_robot_size = atof(inp);
      }

     } else if (!strcmp( command, "OUTPUT_DATA_MAP") ){
       if (fscanf(iop, "%d", &settings.output_data_map) == EOF)
	 FEnd=1;
     } else if (!strcmp( command, "OUTPUT_DATA_MAP_FILENAME") ){
       if (fscanf(iop, "%s", inp) == EOF) {
	 FEnd=1;
       } else {
	 strncpy( settings.output_data_map_filename, inp, MAX_NAME_LENGTH );
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

     } else if (!strcmp( command, "EXPERIMENT_LOG") ){
       if (fscanf(iop, "%d", &settings.experiment_log) == EOF)
	 FEnd=1;
     } else if (!strcmp( command, "EXPERIMENT_LOG_FILENAME") ){
       if (fscanf(iop, "%s", inp) == EOF) {
	 FEnd=1;
       } else {
	 strncpy( settings.experiment_log_filename, inp, MAX_NAME_LENGTH );
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

     } else if (!strcmp( command, "OUTPUT_STATISTICS") ){
       if (fscanf(iop, "%d", &settings.output_statistics) == EOF)
	 FEnd=1;

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
 
 if (strcmp( communicationtype, "" )) {
   if (!strcmp( communicationtype, "IPC" )) {
     settings.communication = IPC;
   } else if (!strcmp( communicationtype, "TCX" )) {
     settings.communication = TCX;
   } else {
     fprintf( stderr, "ERROR: unknown COMMUNICATION_TYPE %s\n",
	      communicationtype );
     fprintf( stderr, "       valid types: TCX, IPC\n" );
     exit(1);
   }
 }

}

void
check_settings( void )
{
  settings.display_pixel_robot_size =
    settings.display_robot_size / settings.global_map_resolution;
  
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

  if (settings.global_ray_map_start_x != -1) {
    settings.global_ray_map_start_x =
      settings.global_ray_map_size_x - settings.global_ray_map_start_x;
  } else {
    settings.global_ray_map_start_x = settings.global_ray_map_size_x / 2;
  }
  if (settings.global_ray_map_start_y != -1) {
    settings.global_ray_map_start_y =
      settings.global_ray_map_size_y - settings.global_ray_map_start_y;
  } else {
    settings.global_ray_map_start_y = settings.global_ray_map_size_y / 2;
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




