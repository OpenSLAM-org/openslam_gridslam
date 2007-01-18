#include "fast-slam.h"

FASTSLAM_SETTINGS          settings;

void
set_default( void )
{

  settings.num_samples                     = 150;
    
  //  settings.detect_size                     = FALSE;
  settings.detect_size                     = TRUE;

  settings.detect_size_border              = 500.0;

  
  settings.size_x                          = 8000.0;
  settings.size_y                          = 8000.0;
  settings.start_x                         = 4000.0;
  settings.start_y                         = 4000.0;

  settings.resolution                      =  7.5;
    
  settings.dump_screen                     = FALSE;
  strncpy( settings.dump_filename, "dump_", MAX_NAME_LENGTH );
  strncpy( settings.result_filename, "fastslam-result.rec", MAX_NAME_LENGTH );

  settings.max_range_length                = 450.0;
  settings.max_usable_length               = 8000.0;
 
  settings.show_graphics                    = TRUE;

  settings.rotation_noise                  = 0.02;
  settings.sideward_noise                  = deg2rad(0.001);
  settings.forward_noise                   = 0.05;

  settings.min_likelihood                  = 0.8;
  settings.max_likelihood                  = 1.0;
  settings.unknown_likelihood              = 0.83;

  settings.min_step_distance               = 500.0;

  settings.laser_id                        = 0;
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
      if (!strcmp( command, "DETECT_SIZE") ){
	if (fscanf(iop, "%d", &settings.detect_size) == EOF)
	  FEnd=1;
      } else if (!strcmp( command, "DETECT_SIZE_BORDER") ){
	if (fscanf(iop, "%s", inp) == EOF) {
	  FEnd=1;
	} else {
	  settings.detect_size_border = atof(inp);
	}
      } else if (!strcmp( command, "SIZE_X") ){
	if (fscanf(iop, "%s", inp) == EOF) {
	  FEnd=1;
	} else {
	  settings.size_x = atof(inp);
	}
      } else if (!strcmp( command, "SIZE_Y") ){
	if (fscanf(iop, "%s", inp) == EOF) {
	  FEnd=1;
	} else {
	  settings.size_y = atof(inp);
	}
      } else if (!strcmp( command, "START_X") ){
	if (fscanf(iop, "%s", inp) == EOF) {
	  FEnd=1;
	} else {
	  settings.start_x = atof(inp);
	}
      } else if (!strcmp( command, "START_Y") ){
	if (fscanf(iop, "%s", inp) == EOF) {
	  FEnd=1;
	} else {
	  settings.start_y = atof(inp);
	}
      } else if (!strcmp( command, "MAX_RANGE_LENGTH") ){
	if (fscanf(iop, "%s", inp) == EOF) {
	  FEnd=1;
	} else {
	  settings.max_range_length = atof(inp);
	}
	
      } else if (!strcmp( command, "MAX_USABLE_LENGTH") ){
	if (fscanf(iop, "%s", inp) == EOF) {
	  FEnd=1;
	} else {
	  settings.max_usable_length = atof(inp);
	}
      } else if (!strcmp( command, "ROTATION_NOISE") ){
	if (fscanf(iop, "%s", inp) == EOF) {
	  FEnd=1;
	} else {
	  settings.rotation_noise = atof(inp);
	}
      } else if (!strcmp( command, "SIDEWARD_NOISE") ){
	if (fscanf(iop, "%s", inp) == EOF) {
	  FEnd=1;
	} else {
	  settings.sideward_noise = deg2rad(atof(inp));
	}
      } else if (!strcmp( command, "FORWARD_NOISE") ){
	if (fscanf(iop, "%s", inp) == EOF) {
	  FEnd=1;
	} else {
	  settings.forward_noise = atof(inp);
	}
      } else if (!strcmp( command, "MIN_LIKELIHOOD") ){
	if (fscanf(iop, "%s", inp) == EOF) {
	  FEnd=1;
	} else {
	  settings.min_likelihood = atof(inp);
	}
      } else if (!strcmp( command, "MAX_LIKELIHOOD") ){
	if (fscanf(iop, "%s", inp) == EOF)
	  FEnd=1;
	else
	  settings.max_likelihood = atof(inp);
      } else if (!strcmp( command, "UNKNOWN_LIKELIHOOD") ){
	if (fscanf(iop, "%s", inp) == EOF) {
	  FEnd=1;
	} else {
	  settings.unknown_likelihood = atof(inp);
	}
      } else if (!strcmp( command, "NUM_SAMPLES") ){
	if (fscanf(iop, "%s", inp) == EOF) {
	  FEnd=1;
	} else {
	  settings.num_samples = atoi(inp);
	}
      } else if (!strcmp( command, "MIN_STEP_DISTANCE") ){
	if (fscanf(iop, "%s", inp) == EOF) {
	  FEnd=1;
	} else {
	  settings.min_step_distance = atof(inp);
	}
      } else if (!strcmp( command, "RESOLUTION") ){
	if (fscanf(iop, "%s", inp) == EOF) {
	  FEnd=1;
	} else {
	  settings.resolution = atof(inp);
	}

      } else if (!strcmp( command, "DUMP_SCREEN") ){
	if (fscanf(iop, "%s", inp) == EOF) {
	  FEnd=1;
	} else {
	  settings.dump_screen = atoi(inp);
	}

      } else if (!strcmp( command, "SHOW_GRAPHICS") ){
	if (fscanf(iop, "%s", inp) == EOF) {
	  FEnd=1;
	} else {
	  settings.show_graphics = atoi(inp);
	}

      } else if (!strcmp( command, "SHOW_LOCAL_MAP") ){
	if (fscanf(iop, "%s", inp) == EOF) {
	  FEnd=1;
	} else {
	  settings.show_local_map = atoi(inp);
	}

      } else if (!strcmp( command, "KERNEL_SIZE") ){
	if (fscanf(iop, "%s", inp) == EOF) {
	  FEnd=1;
	} else {
	  settings.kernel_size = atoi(inp);
	}

      } else if (!strcmp( command, "DUMP_FILENAME_PREFIX") ){
	if (fscanf(iop, "%s", inp) == EOF) {
	  FEnd=1;
	} else {
	  strncpy( settings.dump_filename, inp, MAX_NAME_LENGTH );
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





