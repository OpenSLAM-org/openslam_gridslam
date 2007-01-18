#ifndef MAP2D_H
#define MAP2D_H

#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <sys/types.h>
#include <sys/time.h>
#include <unistd.h>
#include <math.h>
#include <string.h>
#include <ctype.h>
#include <values.h>

#include <carmen/carmen.h>
#include <carmen/logtools.h>

#define MAX_NAME_LENGTH        256
#define EPSILON                0.00000001

#define MAX_NUM_LASER_VALUES   401
#define GAUSS_NOISE            0

#define MAX_LINE_LENGTH          4096
#define LOCAL_MAP_OBSTACLE_PROB  0.1

enum LASER_TYPE         { PLS, LMS };

#define ADDITIONAL_X_SIZE     0.0
#define ADDITIONAL_Y_SIZE     0.0
#define MAX_NUM_LASER_BEAMS   401

#define ODO_NOTHING       0
#define ODO_STD           1
#define ODO_LAST          2

#define GLOBAL_MAP        0
#define LOCAL_MAP         1
#define SHOW_RAYS         4

#define UPDATE_TCX_LOOP   5

typedef struct {
  int                    nummovements;
  logtools_rpos2_t     * pos;
  logtools_rmove2_t    * move;
} REC2_MOVEMENTS;

typedef struct {
  short                          x;
  short                          y;
} logtools_svector2_t;

typedef struct QUAD_TREE {
  struct QUAD_TREE  * elem[4];
  logtools_svector2_t center;
  unsigned char       level;
  unsigned char       inuse;
} QUAD_TREE;

#define UPDT_NOT    0
#define UPDT_X      1
#define UPDT_Y      2

typedef struct {
  QUAD_TREE               qtree;
  logtools_rpos2_t        offset;
  double                  resolution;
  unsigned char        ** updated;
  float                ** maphit;
  short                ** mapsum;
  float                ** mapprob;
  float                ** calc;
  logtools_ivector2_t     mapsize;
  logtools_vector2_t      center;
} MAP2;

enum MODES         { ONLINE, READ_FILE };

typedef struct {
  int                   use_graphics;

  int                   use_correction;

  char                  robot_name[MAX_NAME_LENGTH];
  int                   using_multiple_laser;
  int                   laser_number;
  int                   laser_direction;
  char                  laser_char_type[MAX_NAME_LENGTH];
  enum LASER_TYPE       laser_type;
  
  enum MODES            mode;
  char                  data_filename[MAX_NAME_LENGTH];
  FILE                * dataF;

  int                   dump_maps;
  char                  dump_mapnames[MAX_NAME_LENGTH];

  double                max_usable_laser_range;
  
  double                local_map_max_range;
  double                local_map_resolution;
  int                   local_map_kernel_len;
  int                   local_map_use_odometry;
  int                   local_map_num_convolve;
  double                local_map_std_val;
  int                   local_map_num_history;
  int                   local_map_max_used_history;
  double                local_map_min_bbox_distance;
  int                   local_map_history_skip;
  double                local_map_object_prob;

  double                max_dynprob_change;
  double                max_dynamic_prob;
  
  logtools_rmove2_t     motion_model;
  
  int                   add_noise;
  double                add_noise_val;
  int                   noise_type;
  int                   random_number;

  double                pos_diff_min_dist;
  double                pos_diff_max_dist;
  double                pos_diff_min_rot;
  double                pos_diff_max_rot;

  double                pos_corr_step_size_forward;
  double                pos_corr_step_size_sideward;
  double                pos_corr_step_size_rotation;
  int                   pos_corr_step_size_loop;

  int                   use_error_analyze;
  double                error_max_forward;
  double                error_forward_step;
  double                error_max_sideward;
  double                error_sideward_step;
  double                error_max_rotation;
  double                error_rotation_step;
  double                error_min_distance;
  
  int                   use_global_map;
  int                   global_map_ray_model;
  double                global_map_max_range;
  int                   global_map_size_x;
  int                   global_map_size_y;
  int                   global_map_start_x;
  int                   global_map_start_y;
  double                global_map_resolution;
  double                global_map_std_val;
  logtools_rpos2_t      global_map_start_pos;
  char                  global_map_filename[MAX_NAME_LENGTH];

  double                display_robot_size;
  int                   display_pixel_robot_size;

  FILE                * devNULL;
  
  int                   log_output;
  char                  log_filename[MAX_NAME_LENGTH];
  FILE                * logF;

  int                   view_path;
  int                   show_updates;
  int                   change_map;

  int                   loop_sleep;

} MAP2D_SETTINGS;

typedef struct {
  int                          numlaserscans;
  logtools_lasersens2_data_t * lsens;
} ONLINE_LASER_DATA;

extern MAP2D_SETTINGS          settings;
extern ONLINE_LASER_DATA       online_data;
extern int                     online_dptr;
extern int                     online_laserupdate;
extern int                     online_scan_ctr;
extern int                     online_odo_update;

void    grid_line( logtools_ivector2_t   start,
		   logtools_ivector2_t   end,
		   logtools_grid_line_t *line );


void    initialize_map( MAP2 *map, int sx, int sy, int center_x, int center_y,
			double resolution, logtools_rpos2_t start );

void    clear_local_map( MAP2 *map );

void    create_local_map( MAP2 *map,
			  logtools_lasersens2_data_t data,
			  logtools_rmove2_t move );

logtools_vector2_t
compute_rel_coord2_with_offset( logtools_lasersens2_data_t lsens, int i,
				logtools_rmove2_t offset );

logtools_rmove2_t  fit_data_in_local_map( MAP2 map, 
					  logtools_lasersens2_data_t *data,
					  logtools_rmove2_t movement );

void    set_default( void );

void    read_ini_file( char *filename );

void    check_settings( void );

int     compute_map_pos_from_rpos( logtools_rpos2_t rpos, MAP2 *map, logtools_ivector2_t *v );

int     map_pos_from_vector2( logtools_vector2_t pos, MAP2 *map, logtools_ivector2_t *v );

int     compute_map_pos_from_vector2( logtools_vector2_t pos, MAP2 *map, logtools_ivector2_t *v );

void    compute_map( MAP2 * map );

void    compute_map_probs( MAP2 * map );

void    clear_global_map( MAP2 *map );

double  compute_orientation_diff( double start, double end );

void    initialize_global_map( MAP2 *map, int size_x, int size_y,
			       int start_x, int start_y, double resolution,
			       logtools_rpos2_t start, int convolve );

void    update_global_map( logtools_lasersens2_data_t lsens, MAP2 * map );

void    update_global_ray_map( logtools_lasersens2_data_t lsens, MAP2 * map, int loop );

logtools_laser_coord2_t   map2d_compute_laser2d_coord( logtools_lasersens2_data_t lsens, int i );

void    compute_laser2d_points( logtools_log_data_t *rec, int laserID );

logtools_vector2_t compute_rel_coord2_with_offset( logtools_lasersens2_data_t lsens, int i,
					logtools_rmove2_t offset );

int     minimal_rpos_diff( logtools_rpos2_t pos1, logtools_rpos2_t pos2, 
			   double pos_diff_min_dist, 
			   double pos_diff_min_rot );

int     minimal_rmove_diff( logtools_rmove2_t move,
			    double pos_diff_min_dist, 
			    double pos_diff_min_rot );

void    write_log_entry( FILE * fp, logtools_lasersens2_data_t * lsens );

int     isInBox( logtools_vector2_t p, logtools_vector2_t ll, logtools_vector2_t ur );

void    write_data_entry( FILE *fp, logtools_lasersens2_data_t lsens, int laserID );

void    remove_rear_scans( logtools_log_data_t * rec, int laserID );

void    save_rec2_movements( logtools_log_data_t rec, REC2_MOVEMENTS * save,
			     int laserID );

void    add_noise( logtools_log_data_t * rec, REC2_MOVEMENTS orig, int laserID );

void    tree_list( QUAD_TREE *tree, int *ct );

void    clear_local_treemap( QUAD_TREE *tree, MAP2 *map, int hk );

void    convolve_map( MAP2 *map );

void    compute_laser2d_bbox( logtools_lasersens2_data_t *lsens );

int     intersect_bboxes( logtools_bounding_box_t box1,
			  logtools_bounding_box_t box2 );

logtools_vector2_t map2d_compute_laser_abs_point( logtools_rpos2_t rpos, double val,
				       logtools_rmove2_t offset, double angle );

void    conncect_tcx( char* robotName );

void    init_modul_names( char *extension );

void    check_tcx_connections( void );

void    simple_compute_map( MAP2 *map );

void    simple_clear_map( MAP2 *map );

void    compute_ray_map( MAP2 *map );

int     compute_rmap_pos_from_vector2( logtools_vector2_t vec, MAP2 map, logtools_ivector2_t *v );

void    dynamic_prob_from_global_map( logtools_lasersens2_data_t * data, MAP2 map );

void    compute_probs_of_global_map( MAP2 * global_map );

void    simple_convolve_map( MAP2 *map, logtools_gauss_kernel_t kernel );


int     map_pos_from_rpos( logtools_rpos2_t rpos, MAP2 *map, logtools_ivector2_t *v );

void    rpos_pos_from_map_pos( logtools_ivector2_t v, MAP2 *map, logtools_rpos2_t *rpos );

void    vector2_from_map_pos( logtools_ivector2_t v, MAP2 *map, logtools_vector2_t *pos );

int     map_pos_from_vector2( logtools_vector2_t pos, MAP2 *map, logtools_ivector2_t *v );

double  probability_with_pos( MAP2 map, logtools_lasersens2_data_t data,
			      logtools_rpos2_t rpos, logtools_rmove2_t move, logtools_rmove2_t odo_move );

void    app_process_events( void );

void    update_screen( logtools_lasersens2_data_t lsens );

void    paint_robot( logtools_rpos2_t pos );

void    plot_robot( logtools_rpos2_t pos );

void    plot_robot_path( void );

void    update_map( void );

void    center_robot( void );

int     window_maptype( void );

void    window_show_rays( void );

double  rpos2_dist( logtools_rpos2_t pos1, logtools_rpos2_t pos2 );

double  rmove2_length( logtools_rmove2_t move );

void    TCX_send_update_status( logtools_rpos2_t pos, struct timeval time );

int     create_output_files( void );

void    close_output_files( void );

void    run_data_file( MAP2D_SETTINGS settings,
		       MAP2 * lmap, MAP2 * gmap );

void    run_online( MAP2D_SETTINGS settings,
		    MAP2 * lmap, MAP2 * gmap, 
		    int argc, char *argv[]  );

void    create_script_file( int number );

double  get_mapval( int pos_x, int pos_y, MAP2 map );

logtools_bounding_box_t laser2d_bbox( int numvalues, float *val,
				      logtools_laser_coord2_t *coord );

void    ipc_update( void );

void    ipc_init( int argc, char *argv[] );

void    ipc_stop( void );

void    ipc_publish_status( logtools_rpos2_t pos, struct timeval time );

void    map_change_settings( int set );

void    update_change_map( void );

extern int change_map;

double  probability_with_move( MAP2 map, logtools_lasersens2_data_t data,
			       logtools_rmove2_t move, logtools_rmove2_t odo_move,
			       double *laserprob );

#endif
