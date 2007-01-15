#include <stdio.h>

#ifndef MAP2D_H
#define MAP2D_H

#define MAX_NAME_LENGTH        256
#define EPSILON                0.00000001

#define MAX_NUM_LASER_VALUES   401
#define GAUSS_NOISE            0

#define MAX_LINE_LENGTH          4096
#define LOCAL_MAP_OBSTACLE_PROB  0.1

enum FILE_TYPE          { SCRIPT, REC };

enum LASER_TYPE         { PLS, LMS };

#define FILE_SCRIPT_EXT   ".script"
#define FILE_REC_EXT      ".rec"

#define ADDITIONAL_X_SIZE     0.0
#define ADDITIONAL_Y_SIZE     0.0
#define MAX_NUM_LASER_BEAMS   401

#define ODO_NOTHING       0
#define ODO_STD           1
#define ODO_LAST          2

#define LOOP_START        0
#define LOOP_END          1

#define GLOBAL_MAP        0
#define LOCAL_MAP         1
#define LOCAL_RAY_MAP     2
#define GLOBAL_RAY_MAP    3
#define SHOW_RAYS         4


#define UPDATE_TCX_LOOP   5

typedef struct {
  int          nummovements;
  RPOS2      * pos;
  RMOVE2     * move;
} REC2_MOVEMENTS;

typedef struct {
  short           x;
  short           y;
} sVECTOR2;

typedef struct QUAD_TREE {
  struct QUAD_TREE  * elem[4];
  sVECTOR2            center;
  unsigned char       level;
  unsigned char       inuse;
} QUAD_TREE;

#define UPDT_NOT    0
#define UPDT_X      1
#define UPDT_Y      2

typedef struct {
  QUAD_TREE         qtree;
  RPOS2             offset;
  double            resolution;
  unsigned char  ** updated;
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
} GRID_LINE;

enum COMMUNICATION { TCX, IPC };
enum MODES         { ONLINE, READ_FILE, READ_LINES };

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
  enum COMMUNICATION    communication;
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

  int                   use_global_ray_map;

  int                   use_local_ray_map;
  double                local_ray_map_resolution;
  double                local_ray_map_max_range;
  int                   local_ray_map_kernel_len;
  int                   local_ray_map_num_convolve;
  double                local_ray_map_dynamic_prob;
  double                local_ray_map_static_prob;
  int                   local_ray_map_num_history;
  int                   local_ray_map_max_used_history;
  double                local_ray_map_min_bbox_distance;
  int                   local_ray_map_shorten_ray;
  
  double                max_dynprob_change;
  double                max_dynamic_prob;
  
  RMOVE2                motion_model;
  
  int                   add_noise;
  double                add_noise_val;
  int                   noise_type;
  int                   random_number;

  int                   use_people_prob;
  int                   people_prob_kernel_len;
  int                   people_prob_num_convolve;

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
  RPOS2                 global_map_start_pos;
  char                  global_map_filename[MAX_NAME_LENGTH];

  double                global_ray_map_max_range;
  double                global_ray_map_clip_val;
  double                global_ray_map_beam_diff;
  int                   global_ray_map_size_x;
  int                   global_ray_map_size_y;
  int                   global_ray_map_start_x;
  int                   global_ray_map_start_y;
  double                global_ray_map_resolution;
  double                global_ray_map_std_val;
  RPOS2                 global_ray_map_start_pos;
  int                   global_ray_map_shorten_ray;
  double                global_ray_map_dynamic_prob;
  double                global_ray_map_static_prob;
  double                dynprob_prior;
  
  double                display_robot_size;
  int                   display_pixel_robot_size;

  int                   output_data_map;
  char                  output_data_map_filename[MAX_NAME_LENGTH];
  FILE                * outputF;

  int                   dump_probabilities;
  char                  prob_filename[MAX_NAME_LENGTH];
  FILE                * probF;

  FILE                * devNULL;
  
  enum FILE_TYPE        script_type;
  int                   script_output;
  int                   script_loop_nr;
  char                  script_out_filename[MAX_NAME_LENGTH];
  FILE                * scriptF;

  int                   experiment_log;
  char                  experiment_log_filename[MAX_NAME_LENGTH];
  FILE                * experimentF;

  int                   output_statistics;
  char                  statistics_filename[MAX_NAME_LENGTH];
  FILE                * statisticsF;

  int                   view_path;
  int                   show_updates;
  int                   change_map;

  int                   loop_sleep;

} MAP2D_SETTINGS;

typedef struct {
  int                 numlaserscans;
  LASERSENS2_DATA   * lsens;
} ONLINE_LASER_DATA;

extern MAP2D_SETTINGS          settings;
extern ONLINE_LASER_DATA       online_data;
extern int                     online_dptr;
extern int                     online_laserupdate;
extern int                     online_scan_ctr;
extern int                     online_odo_update;

void    grid_line( iVECTOR2 start, iVECTOR2 end, GRID_LINE *line );


void    initialize_map( MAP2 *map, int sx, int sy, int center_x, int center_y,
			double resolution, RPOS2 start );

void    initialize_ray_map( MAP2 *map, int sx, int sy, 
			    double resolution, RPOS2 start );

void    clear_local_map( MAP2 *map );

void    create_local_map( MAP2 *map, LASERSENS2_DATA data, RMOVE2 move );

void    create_local_ray_map( MAP2 *map, LASERSENS2_DATA data, RMOVE2 move );

VECTOR2 compute_rel_coord2_with_offset( LASERSENS2_DATA lsens, int i,
					RMOVE2 offset );

RMOVE2  fit_data_in_local_map( MAP2 map, MAP2 ray_map, LASERSENS2_DATA *data,
			       RMOVE2 movement, int num_used_scans,
			       int scannr, int loopnr );

void    set_default( void );

void    read_ini_file( char *filename );

void    check_settings( void );

int     compute_map_pos_from_rpos( RPOS2 rpos, MAP2 *map, iVECTOR2 *v );

int     map_pos_from_vec2( VECTOR2 pos, MAP2 *map, iVECTOR2 *v );

int     compute_map_pos_from_vec2( VECTOR2 pos, MAP2 *map, iVECTOR2 *v );

void    compute_map( MAP2 * map );

void    compute_map_probs( MAP2 * map );

void    write_map( MAP2 map, char *filename );

void    write_bee_map( MAP2 map, char *filename );

void    clear_global_map( MAP2 *map );

void    initialize_global_map( MAP2 *map, int size_x, int size_y,
			       int start_x, int start_y, double resolution,
			       RPOS2 start, int convolve );

void    update_global_map( LASERSENS2_DATA lsens, MAP2 * map );

void    update_global_ray_map( LASERSENS2_DATA lsens, MAP2 * map, int loop );

LASER_COORD2   map2d_compute_laser2d_coord( LASERSENS2_DATA lsens, int i );

void    compute_laser2d_points( REC2_DATA *rec, int laserID );

VECTOR2 compute_rel_coord2_with_offset( LASERSENS2_DATA lsens, int i,
					RMOVE2 offset );

int     minimal_rpos_diff( RPOS2 pos1, RPOS2 pos2, 
			   double pos_diff_min_dist, 
			   double pos_diff_min_rot );

int     minimal_rmove_diff( RMOVE2 move,
			    double pos_diff_min_dist, 
			    double pos_diff_min_rot );

void    write_sens( FILE  *fp, struct timeval time );

void    check_hprob( REC2_DATA *rec, int laserID );

void    convolve_hprob( REC2_DATA *rec, int laserID );

void    write_script_entry( FILE *fp, LASERSENS2_DATA lsens, int laserID );

void    write_script_marker( FILE * fp, int type, int data );

int     isInBox( VECTOR2 p, VECTOR2 ll, VECTOR2 ur );

int     isPeople( VECTOR2 p, LASERSENS2_DATA lsens, int laserID );

void    write_data_entry( FILE *fp, LASERSENS2_DATA lsens, int laserID );

void    remove_rear_scans( REC2_DATA * rec, int laserID );

void    save_rec2_movements( REC2_DATA rec, REC2_MOVEMENTS * save,
			     int laserID );

void    add_noise( REC2_DATA * rec, REC2_MOVEMENTS orig, int laserID );

void    write_experiment_error( int index, RMOVE2 cmove, RMOVE2 lmove,
				RMOVE2 emove, RPOS2 estpos, RPOS2 origpos );

void    write_total_error( REC2_DATA rec, REC2_MOVEMENTS orig, int laserID );

void    compute_statistics( REC2_DATA rec, int laserID );

void    tree_list( QUAD_TREE *tree, int *ct );

void    clear_local_treemap( QUAD_TREE *tree, MAP2 *map, int hk );

void    convolve_map( MAP2 *map );

void    convolve_ray_map( MAP2 *map );

void    compute_laser2d_bbox( LASERSENS2_DATA *lsens );

int     intersect_bboxes( BOUNDING_BOX2 box1, BOUNDING_BOX2 box2 );

VECTOR2 map2d_compute_laser_abs_point( RPOS2 rpos, double val,
				       RMOVE2 offset, double angle );

void    conncect_tcx( char* robotName );

void    init_modul_names( char *extension );

void    check_tcx_connections( void );

void    simple_compute_map( MAP2 *map );

void    simple_clear_map( MAP2 *map );

void    compute_ray_map( MAP2 *map );

void    convole_ray_map( MAP2 *map );

int     compute_rmap_pos_from_vec2( VECTOR2 vec, MAP2 map, iVECTOR2 *v );

void    dynamic_prob_from_global_map( LASERSENS2_DATA * data, MAP2 map );

void    compute_probs_of_global_map( MAP2 * global_map );

void    compute_probs_of_global_ray_map( MAP2 * global_map );

void    simple_convolve_map( MAP2 *map, GAUSS_KERNEL kernel );


int     map_pos_from_rpos( RPOS2 rpos, MAP2 *map, iVECTOR2 *v );

void    rpos_pos_from_map_pos( iVECTOR2 v, MAP2 *map, RPOS2 *rpos );

void    vec2_from_map_pos( iVECTOR2 v, MAP2 *map, VECTOR2 *pos );

int     map_pos_from_vec2( VECTOR2 pos, MAP2 *map, iVECTOR2 *v );

double  probability_with_pos( MAP2 map, LASERSENS2_DATA data,
			      RPOS2 rpos, RMOVE2 move, RMOVE2 odo_move );

void    app_process_events( void );

void    update_screen( LASERSENS2_DATA lsens );

void    paint_robot( RPOS2 pos );

void    plot_robot( RPOS2 pos );

void    plot_robot_path( void );

void    update_map( void );

void    center_robot( void );

int     window_maptype( void );

void    window_show_rays( void );

double  rpos2_dist( RPOS2 pos1, RPOS2 pos2 );

double  rmove2_length( RMOVE2 move );

void    TCX_send_update_status( RPOS2 pos, struct timeval time );

int     create_output_files( void );

void    close_output_files( void );

void    run_data_file( MAP2D_SETTINGS settings,
		       MAP2 * lmap, MAP2 * rmap,
		       MAP2 * gmap, MAP2 * grmap );

void    run_online( MAP2D_SETTINGS settings,
		    MAP2 * lmap, MAP2 * rmap, MAP2 * gmap, int online );

void    create_script_file( int number );

double  get_mapval( int pos_x, int pos_y, MAP2 map );

BOUNDING_BOX2 laser2d_bbox( int numvalues, double *val, LASER_COORD2 *coord );

void    ipc_update( void );

void    ipc_init( char * name );

void    ipc_stop( void );

void    ipc_publish_status( RPOS2 pos, struct timeval time );

void    map_change_settings( int set );

void    update_change_map( void );

extern int change_map;

double  probability_with_move( MAP2 map, LASERSENS2_DATA data,
			       RMOVE2 move, RMOVE2 odo_move,
			       double *laserprob );

#endif
