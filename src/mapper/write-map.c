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

#ifdef USE_MAGICK
#include <magick/api.h>
#endif

#include <navigation/utils.h>
#include "map2d.h"

#define MINIMUM_MAP_SIZE  300
#define MAP_SIZE_STEP     50

void
printUnknown( FILE *fp, int n)
{
   while (n-- > 0)
      fprintf( fp, "-1 ");
}

void
write_bee_map( MAP2 map, char *filename )
{
  FILE    * ofp;
  double    prob;
  int       globalSizeX, globalSizeY, extendedSizeX, extendedSizeY; 
  int       top, bottom, left, right, x, y;

  if ((ofp = fopen( filename, "w")) == 0){
    fprintf(stderr, "ERROR: can't write data map file %s\n", filename );
    return;
  }

  if (map.mapsize.x < MINIMUM_MAP_SIZE)
    extendedSizeX = MINIMUM_MAP_SIZE;
  else
    extendedSizeX = ((map.mapsize.x / MAP_SIZE_STEP) + 1) * MAP_SIZE_STEP;
    
  if (map.mapsize.y < MINIMUM_MAP_SIZE)
    extendedSizeY = MINIMUM_MAP_SIZE;
  else
    extendedSizeY = ((map.mapsize.y / MAP_SIZE_STEP) + 1) * MAP_SIZE_STEP;
  
  top         = (extendedSizeY - map.mapsize.y) / 2;
  bottom      = extendedSizeY - top - map.mapsize.y;
  left        = (extendedSizeX - map.mapsize.x) / 2;
  right       = extendedSizeX - left - map.mapsize.x;
  globalSizeX = extendedSizeX * map.resolution;
  globalSizeY = extendedSizeY * map.resolution;
  
  fprintf( ofp, "robot_specifications->global_mapsize_x  %d\n", globalSizeY);
  fprintf( ofp, "robot_specifications->global_mapsize_y  %d\n", globalSizeX);
  fprintf( ofp, "robot_specifications->resolution %d\n", (int) map.resolution);
  fprintf( ofp, "global_map[0]: %d %d\n", extendedSizeY, extendedSizeX);
    
  for (x = 0; x < left; x++){
    printUnknown(ofp, extendedSizeY);
    fprintf( ofp, "\n");
  }
  
  for (x = 0; x < map.mapsize.x; x++){
    printUnknown( ofp, top);
    for (y = 0; y < map.mapsize.y; y++) {
      if (map.mapsum[map.mapsize.x-x-1][map.mapsize.y-y-1] == 0) {
	prob = -1.0;
      } else {
	prob =
	  1.0 - ( map.maphit[map.mapsize.x-x-1][map.mapsize.y-y-1] /
		  (double) (map.maphit[map.mapsize.x-x-1][map.mapsize.y-y-1] +
			    map.mapsum[map.mapsize.x-x-1][map.mapsize.y-y-1] ) );
      }
      fprintf(ofp, "%.4f ", prob);
    }
    printUnknown( ofp, bottom);
    fprintf(ofp, "\n");
  }
  
  for (x = 0; x < right; x++){
    printUnknown( ofp, extendedSizeY);
    fprintf( ofp, "\n");
  }
  
  fclose(ofp);
} 
  
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

#ifndef USE_MAGICK

void 
write_map( MAP2 map __attribute__ ((unused)),
	   char *filename __attribute__ ((unused)) )
{
}

#else

void
write_map( MAP2 map, char *filename )
{

  int                 ok = TRUE;
  int                 i, j, idx;

  Image             * image;
  ImageInfo           image_info;
  double              c = 0.0;

#if defined MagickLibVersion && MagickLibVersion >= 0x500
  ExceptionInfo       exception;
  double            * pixel;
#else
  float             * red, * green, * blue, * opacity;
#endif

#if defined MagickLibVersion && MagickLibVersion >= 0x500
  fprintf( stderr, "alloc memory of pixel map (%d bytes) ... ",
	   map.mapsize.x * map.mapsize.y * 3 * sizeof(double) );
  if ( (pixel = (double *) malloc(map.mapsize.x * map.mapsize.y *
				  3 * sizeof(double)))==NULL )
    ok = FALSE;
  fprintf( stderr, "%s\n", ok?"yes":"no" );
#else
  fprintf( stderr, "alloc memory of pixel map (%d bytes) ... ",
	   map.mapsize.x * map.mapsize.y * 3 * sizeof(float) );
  if ( (red = (float *) malloc(map.mapsize.x * map.mapsize.y *
			       sizeof(float)))==NULL )
    ok = FALSE;
  fprintf( stderr, "red: %s,", ok?"yes":"no" );
  if ( (green = (float *) malloc(map.mapsize.x * map.mapsize.y *
				 sizeof(float)))==NULL )
    ok = FALSE;
  fprintf( stderr, "green: %s,", ok?"yes":"no" );
  if ( (blue = (float *) malloc(map.mapsize.x * map.mapsize.y *
				sizeof(float)))==NULL )
    ok = FALSE;
  fprintf( stderr, "blue: %s,", ok?"yes":"no" );
  if ( (opacity = (float *) malloc(map.mapsize.x * map.mapsize.y *
				   sizeof(float)))==NULL )
    ok = FALSE;
  fprintf( stderr, "opacity: %s,", ok?"yes":"no" );
#endif
  for (i=0;i<map.mapsize.x;i++) {
    for (j=0;j<map.mapsize.y;j++) {
      idx = j*map.mapsize.x+i;
      c = 1.0-map.mapprob[i][map.mapsize.y-j-1];
      if (c<0.0)
	c = 0.0;
#if defined MagickLibVersion && MagickLibVersion >= 0x500
      pixel[idx*3]     = c;
      pixel[idx*3+1]   = c;
      pixel[idx*3+2]   = c;
#else
      red[idx]     = c;
      green[idx]   = c;
      blue[idx]    = c;
      opacity[idx] = 1.0;
#endif
    }
  }
#if defined MagickLibVersion && MagickLibVersion >= 0x430
  GetExceptionInfo(&exception);
#endif
  GetImageInfo(&image_info);
  fprintf( stderr, "create image of map ... " );
#if defined MagickLibVersion && MagickLibVersion >= 0x430
  image = ConstituteImage ( map.mapsize.x, map.mapsize.y, "RGB",
			    DoublePixel, pixel, &exception );
  if (image == (Image *) NULL) {
    fprintf( stderr, "ERROR: no memory!!!\n" );
    exit(1);
  }
#else
#if defined MagickLibVersion && MagickLibVersion >= 0x420
  image = CreateImage( map.mapsize.x, map.mapsize.y,
		       red, green, blue, opacity );
#else
  image = CreateImage( map.mapsize.x, map.mapsize.y, "RGB",
		       red, green, blue, NULL );
  if (image == (Image *) NULL) {
    fprintf( stderr, "ERROR: no memory!!!\n" );
    exit(1);
  }
#endif
#endif
  strcpy( image->filename, filename );
  fprintf( stderr, "done\n" );
  fprintf( stderr, "write image in file %s ... ", filename );
  WriteImage( &image_info, image );
  fprintf( stderr, "done\n" );
  DestroyImage(image);
#if defined MagickLibVersion && MagickLibVersion >= 0x430
  DestroyConstitute();
  free(pixel);
#else
  free(red);
  free(green);
  free(blue);
#endif
  
}

#endif /* USE_MAGICK */


void
create_script_file( int number )
{
  char fname[MAX_NAME_LENGTH];
  snprintf( fname, MAX_NAME_LENGTH, "%s-%s%s%d",
	    settings.script_out_filename,
	    number<100?"0":"",
	    number<10?"0":"",
	    number );
  if (settings.script_output) {
    if (( settings.scriptF = fopen( fname, "w")) == 0){
      fprintf(stderr, "ERROR: can't write script-file %s\n",
	      fname );
      exit(-1);
    }
  }
}

int
create_output_files( void )
{
  if (settings.script_output && !settings.use_correction) {
      settings.script_output = FALSE;
  }
  
  if ((settings.devNULL =
	 fopen( "/dev/null", "w")) == 0){
    fprintf(stderr, "ERROR: can't open /dev/null\n" );
    exit(-1);
  }
  if (settings.output_data_map) {
    if ((settings.outputF =
	 fopen( settings.output_data_map_filename, "w")) == 0){
      fprintf(stderr, "ERROR: can't write data map file %s\n",
	      settings.output_data_map_filename );
      exit(-1);
    }
  }

  if (settings.experiment_log) {
    if ((settings.experimentF =
	 fopen( settings.experiment_log_filename, "w")) == 0){
      fprintf(stderr, "ERROR: can't write experiment log file %s\n",
	      settings.experiment_log_filename );
      exit(-1);
    }
  }

  if (settings.dump_probabilities) {
    if (( settings.probF = fopen( settings.prob_filename, "w")) == 0){
      fprintf(stderr, "ERROR: can't write prob file %s\n",
	      settings.prob_filename );
      exit(-1);
    }
  }

  if (settings.script_output) {
    if (settings.use_global_ray_map) {
      create_script_file( 1 );
    } else {
      if (( settings.scriptF =
	    fopen( settings.script_out_filename, "w")) == 0){
	fprintf(stderr, "ERROR: can't write script-file %s\n",
		settings.script_out_filename );
	exit(-1);
      }
    }
  }
  if (settings.output_statistics) {
    if ((settings.statisticsF =
	 fopen( settings.statistics_filename, "w")) == 0){
      fprintf(stderr, "ERROR: can't write statistics file %s\n",
	      settings.statistics_filename );
      exit(-1);
    }
  }
  return(0);
}



void
close_output_files( void )
{
  if (settings.output_data_map) {
    fclose(settings.outputF);
  }
  if (settings.experiment_log) {
    fclose(settings.experimentF);
  }
  if (settings.dump_probabilities) {
    fclose(settings.probF);
  }
  if (settings.script_output) {
    fclose(settings.scriptF);
  }
  if (settings.output_statistics) {
    fclose(settings.statisticsF);
  }
}

