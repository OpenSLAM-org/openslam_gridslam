#ifndef GRID_FAST_SLAM_H
#define  GRID_FAST_SLAM_H

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

#include <carmen/carmen.h>
#include <carmen/logtools.h>

#define MAX_NAME_LENGTH       256 

typedef struct {
  
  int      detect_size;

  double   detect_size_border;
  double   size_x;
  double   size_y;
  double   start_x;
  double   start_y;

  double   resolution;
  
  int      dump_screen;
  char     dump_filename[MAX_NAME_LENGTH];

  char     result_filename[MAX_NAME_LENGTH];

  int      num_samples;
  
  double   min_step_distance;
  
  double   max_range_length;
  double   max_usable_length;

  double   rotation_noise;
  double   sideward_noise;
  double   forward_noise;

  double   min_likelihood;
  double   max_likelihood;
  double   unknown_likelihood;

  int      show_graphics;
  int      show_local_map;

  int      kernel_size;

  int      laser_id;
} FASTSLAM_SETTINGS;

typedef struct {
  logtools_rpos2_t       offset;
  double                 resolution;
  float               ** maphit;
  short               ** mapsum;
  float               ** mapprob;
  float               ** calc;
  logtools_ivector2_t    mapsize;
  logtools_vector2_t     center;
} MAP2;

#define logtools_grid_object_t logtools_grid_line_t

typedef struct {
  logtools_bounding_box_t      bbox;
  logtools_rpos2_t             pos;
} HISTORY;

typedef struct {
  logtools_rpos2_t             pos;
  double            val;
  double            logsum;
  int               histlen;
  HISTORY         * hist;
  int               outside;
} PARTICLE;

typedef struct {
  int               numparticles;
  PARTICLE        * particle;
} SAMPLE_SET;

typedef struct {
  double            r;
  double            g;
  double            b;
} RGB;

extern FASTSLAM_SETTINGS          settings;


void
map_initialize( MAP2 *map, int sx, int sy, int center_x, int center_y,
		double resolution, logtools_rpos2_t start );

void
map_clear( MAP2 *map );

void
update_map( MAP2 * map, int numvalues, float * val, float * angle,
	    logtools_rpos2_t estpos, double max_range, double max_usable  );

void
compute_probs_of_map( MAP2 * map );

void
compute_voronoi_of_map( MAP2 * map );

void
compute_calc_of_map( MAP2 * map );

void
compute_intersections_of_map( MAP2 * map );

void
grid_circle( logtools_ivector2_t center, int radius, 
	     logtools_grid_object_t * circle );

void
grid_line( logtools_ivector2_t start, logtools_ivector2_t end, 
	   logtools_grid_line_t *line );

void
show_scan( MAP2 * map, logtools_lasersens2_data_t lsens, double max_range );

double
compute_scan_probability( MAP2 * map, logtools_rpos2_t pos, logtools_lasersens2_data_t lsens,
			  double max_range, double max_usable );

double
compute_beam_prob( MAP2 * map, logtools_rpos2_t pos, double length,
		   double max_range, double * prob1, double * prob2 );

int
map_pos_from_rpos( logtools_rpos2_t rpos, MAP2 *map, logtools_ivector2_t *v );

double
get_map_val( logtools_ivector2_t pos, MAP2 map );

void
simple_convolve_map( MAP2 *map, logtools_gauss_kernel_t kernel );

void
simple_convolve_map2( MAP2 *map );

double
prob_unknown_space( double length, int endpoint );

int
map_pos_from_rpos( logtools_rpos2_t rpos, MAP2 *map, logtools_ivector2_t *v );

int
map_pos_from_vec2( logtools_vector2_t pos, MAP2 *map, logtools_ivector2_t *v );

void
resample( SAMPLE_SET sample1, SAMPLE_SET * sample2 );

void
copy_particle( PARTICLE src, PARTICLE *dest );

int
intersect_bboxes( logtools_bounding_box_t box1, logtools_bounding_box_t box2 );

logtools_bounding_box_t
compute_laser_bounding_box( logtools_rpos2_t pos,
			    logtools_lasersens2_data_t lsens,
			    double laser_max_range );

int
find_best_particle_logsum( SAMPLE_SET pset );

int
find_best_particle_value( SAMPLE_SET pset );


void
set_default( void );

void 
read_ini_file( char *filename );

void
write_map( MAP2 map, char *filename );

#endif 

