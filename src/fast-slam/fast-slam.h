#ifndef GRID_FAST_SLAM_H
#define  GRID_FAST_SLAM_H

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
  
} FASTSLAM_SETTINGS;

typedef struct {
  int          numgrids;
  iVECTOR2   * grid;
} GRID_LINE;

typedef struct {
  RPOS2             offset;
  double            resolution;
  float          ** maphit;
  short          ** mapsum;
  float          ** mapprob;
  float          ** calc;
  iVECTOR2          mapsize;
  VECTOR2           center;
} MAP2;

typedef struct {
  int          numgrids;
  iVECTOR2   * grid;
} GRID_OBJECT;


typedef struct {
  BOUNDING_BOX2     bbox;
  RPOS2             pos;
} HISTORY;

typedef struct {
  RPOS2             pos;
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


extern FASTSLAM_SETTINGS          settings;


void
map_initialize( MAP2 *map, int sx, int sy, int center_x, int center_y,
		double resolution, RPOS2 start );

void
map_clear( MAP2 *map );

void
update_map( MAP2 * map, int numvalues, double  * val, double * angle,
	    RPOS2 estpos, double max_range, double max_usable  );

void
compute_probs_of_map( MAP2 * map );

void
compute_voronoi_of_map( MAP2 * map );

void
compute_calc_of_map( MAP2 * map );

void
compute_intersections_of_map( MAP2 * map );

void
grid_circle( iVECTOR2 center, int radius, GRID_OBJECT * circle );

void
grid_line( iVECTOR2 start, iVECTOR2 end, GRID_LINE *line );

void
show_scan( MAP2 * map, LASERSENS2_DATA lsens, double max_range );

double
compute_scan_probability( MAP2 * map, RPOS2 pos, LASERSENS2_DATA lsens,
			  double max_range, double max_usable );

double
compute_beam_prob( MAP2 * map, RPOS2 pos, double length, double max_range,
		   double * prob1, double * prob2 );

int
map_pos_from_rpos( RPOS2 rpos, MAP2 *map, iVECTOR2 *v );

double
get_map_val( iVECTOR2 pos, MAP2 map );

void
simple_convolve_map( MAP2 *map, GAUSS_KERNEL kernel );

void
simple_convolve_map2( MAP2 *map );

double
prob_unknown_space( double length, int endpoint );

int
map_pos_from_rpos( RPOS2 rpos, MAP2 *map, iVECTOR2 *v );

int
map_pos_from_vec2( VECTOR2 pos, MAP2 *map, iVECTOR2 *v );

void
resample( SAMPLE_SET sample1, SAMPLE_SET * sample2 );

void
copy_particle( PARTICLE src, PARTICLE *dest );

int
intersect_bboxes( BOUNDING_BOX2 box1, BOUNDING_BOX2 box2 );

BOUNDING_BOX2
compute_laser_bounding_box( RPOS2 pos, LASERSENS2_DATA lsens,
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

