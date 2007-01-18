#include "map2d.h"

#define MINIMUM_MAP_SIZE  300
#define MAP_SIZE_STEP     50

void
compute_map_probs( MAP2 * map )
{
  int x, y;
  for (x=0;x<map->mapsize.x;x++) {
    for (y=0;y<map->mapsize.y;y++) {
      if (map->mapsum[x][y]>0) {
	map->mapprob[x][y] =
	  ( map->maphit[x][y] / (double) map->mapsum[x][y] );
      } else {
	map->mapprob[x][y] = settings.global_map_std_val;
      }
    }
  }
}

int
create_output_files( void )
{
  if (settings.log_output && !settings.use_correction) {
    settings.log_output = FALSE;
  }
  
  if ((settings.devNULL =
	 fopen( "/dev/null", "w")) == 0){
    fprintf(stderr, "ERROR: can't open /dev/null\n" );
    exit(-1);
  }

  if (settings.log_output) {
    if (( settings.logF =
	  fopen( settings.log_filename, "w")) == 0){
      fprintf(stderr, "ERROR: can't write carmen log file %s\n",
	      settings.log_filename );
      exit(-1);
    }
  }
  return(0);
}

void
write_log_entry( FILE * fp, logtools_lasersens2_data_t * lsens )
{
  static int firsttime = TRUE;
  static double logtime = 0.0;
  double t = lsens->laser.time.tv_sec + (lsens->laser.time.tv_usec/1000000.0 );
  int i;
  if (firsttime) {
    logtime = t;
    firsttime = FALSE;
  }
  fprintf( fp, "ODOM %f %f %f %f %f %f %f %s %f\n",
	   lsens->estpos.x/100.0,
	   lsens->estpos.y/100.0,
	   lsens->estpos.o,
	   0.0, 0.0, 0.0,
	   t, "mapper", t-logtime );
  fprintf( fp, "RAWLASER%d %d %f %f %f %f %f %d %d", lsens->id+1, 0,
	   -lsens->laser.fov/2.0, lsens->laser.fov,
	   lsens->laser.fov/(double)lsens->laser.numvalues,
	   80.95, 0.05, 0, lsens->laser.numvalues );
  for (i=0; i<lsens->laser.numvalues; i++) {
    fprintf( fp, " %f", lsens->laser.val[i]/100.0 );
  }
  fprintf( fp, " %d %f %s %f\n",
	   0, t, "mapper", t-logtime );

}

void
close_output_files( void )
{
  if (settings.log_output) {
    fclose(settings.logF);
  }
}

