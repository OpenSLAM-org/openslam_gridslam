#ifndef NAV_UTILS_H
#define NAV_UTILS_H

#ifndef GEOMETRY_H
#include "geometry.h"
#endif

#ifndef _STDIO_H
#include <stdio.h>
#endif

#ifndef _SYS_TIME_H
#include <sys/time.h>
#endif


#ifndef TRUE
#define TRUE  1
#endif

#ifndef FALSE
#define FALSE 0
#endif

#define READ_MODE_SILENT    0x00
#define READ_MODE_VERBOSE   0x01
#define READ_MODE_DONT_STOP 0x02

typedef struct {
  double        x;
  double        y;
  double        o;
} RPOS2;

typedef struct {
  double        forward;
  double        sideward;
  double        rotation;
} RMOVE2;

typedef struct {
  double        forward;
  double        sideward;
  double        upward;
} MOVEMENT3; 

typedef struct {
  MOVEMENT3     trans;
  VECTOR3       rot;
} RMOVE3;

typedef struct {
  double        tv;
  double        rv;
} RVEL2;

typedef struct {
  VECTOR3       pos;
  VECTOR3       rot;
} RPOS3;

typedef struct {
  struct timeval      time;
  double              x;
  double              y;
  double              o;
} POS_CORR2_DATA;

typedef struct {
  struct timeval      time;
  double              x;
  double              y;
  double              o;
  double              xvar;
  double              yvar;
  double              ovar;
  int                 numsats;
} GPS_DATA;

typedef struct {
  struct timeval      time;
  double              rx;
  double              ry;
  double              rz;
} COMPASS3_DATA;

typedef struct {
  struct timeval      time;
  double              o;
} COMPASS2_DATA;

typedef struct {
  VECTOR2             min;
  VECTOR2             max;
} BOUNDING_BOX2;

typedef struct {
  /*
    p[0] = xmin, ymin, zmin
    p[1] = xmin, ymax, zmin
    p[2] = xmax, ymax, zmin
    p[3] = xmax, ymin, zmin
    p[4] = xmin, ymin, zmax
    p[5] = xmin, ymax, zmax
    p[6] = xmax, ymax, zmax
    p[7] = xmax, ymin, zmax
  */
  VECTOR3             min;
  VECTOR3             max;
  VECTOR3             p[8];
} BOUNDING_BOX3;

typedef struct {
  VECTOR2             relpt;
  VECTOR2             abspt;
  int                 tag;
  int                 info;
} LASER_COORD2;

typedef struct {
  double              start;
  double              end;
  double              delta;
} LASER_ANGLE_RANGE;

typedef struct {
  RMOVE2              offset;
  LASER_ANGLE_RANGE   range;
} LASER_PROPERTIES2;

typedef struct {
  double              base_height;
  double              top_height;
} AMTEC_SETTINGS;

typedef struct {
  VECTOR3             offset;   /* offset to robot center */
  int                 on_amtec;
  AMTEC_SETTINGS      amtec;
} LASER_PROPERTIES3;

typedef struct {
  struct timeval      time;
  int                 numvalues;
  double              anglerange;
  double            * val;
  double            * angle;
  RMOVE2            * offset;
  int               * maxrange;
} LASER_DATA;

typedef struct {
  RPOS2               pos;
  double              vel;
  double              cov_sx;
  double              cov_sy;
  double              cov_sxy;
  VECTOR2             ll;       /* lower left */
  VECTOR2             ur;       /* upper right */
} P_STATE;

typedef struct {
  int                 numhprobs;
  double            * hprob;    /* probability that's the beam is a human */
  int                 numpstates;
  P_STATE           * pstate;
} P_TRACKING;

typedef struct {
  int                 numprobs;
  double            * prob;   
} DISTRIBUTION;

typedef struct {
  RPOS2               estpos;
  LASER_DATA          laser;
  LASER_COORD2      * coord;
  POLYGON             poly;
  BOUNDING_BOX2       bbox;
  P_TRACKING          ptracking;
  DISTRIBUTION        dynamic;
  int                 id;
} LASERSENS2_DATA;

typedef struct {
  struct timeval      time;
  RPOS2               rpos;
  RVEL2               rvel;
} POSSENS2_DATA;

typedef struct {
  struct timeval      time;
  double              x;
  double              y;
  double              z;
  double              pitch;
  double              yaw;
  double              roll;
} POSSENS3D_DATA;

typedef struct {
  struct timeval      time;
  double              utc;
  double              latitude;
  char                lat_orient;
  double              longitude;
  char                long_orient;
  int                 gps_quality;
  int                 num_sattelites;
  double              hdop;
  double              sea_level;
  double              alitude;
  double              geo_sea_level;
  double              geo_sep;
  int                 data_age;
} NMEA_GGA_DATA;

enum ENTRY_TYPE   { POSITION,
		    CORR_POSITION,
		    LASER_VALUES,
		    LASER_VALUES3,
		    AMTEC_POS,
		    HUMAN_PROB,
		    GPS,
		    NMEA_GGA,
		    COMPASS,
		    POS_CORR,
		    MARK_POS,
		    RFID_TAG,
                    UNKNOWN };

typedef struct {
  enum ENTRY_TYPE          type;
  int                      index;
  int                      idx1, idx2;
} ENTRY_POSITION;

typedef struct {

  int                numdynamicprobs;
  
} REC2_INFO;


typedef struct {

  
  struct timeval      time;
  RPOS2               pos;
  char              * text; 

} MARK_POS_DATA;

typedef struct {

  struct timeval      time;
  RPOS2               estpos;
  unsigned long long  tag;
  int                 antenna;
  int                 count;

} RFID_DATA;



typedef struct {

  /* entries */
  int                 numentries;
  ENTRY_POSITION    * entry;  

  /* positions */
  int                 numpositions;
  POSSENS2_DATA     * psens;
  
  /* positions */
  int                 numpositions3d;
  POSSENS3D_DATA   *  psens3d;
  
  /* corrected positions */
  int                 numcpositions;
  POSSENS2_DATA     * cpsens;
  
  /* laser scans */
  int                 numlaserscans;
  LASERSENS2_DATA   * lsens;

  int                 numcompass;
  COMPASS3_DATA     * compass;

  int                 numgps;
  GPS_DATA          * gps;

  int                 numnmea_gga;
  NMEA_GGA_DATA     * nmea_gga;

  int                 nummarkings;
  MARK_POS_DATA    *  marking;
  
  int                 numrfid;
  RFID_DATA        *  rfid;
  
  int                 numposcorr;
  POS_CORR2_DATA    * poscorr;
  
  REC2_INFO           info;
  
} REC2_DATA;

typedef struct {
  double              pan;
  double              tilt;
} DECL_TYPE;

typedef struct {
  VECTOR3             relpt;
  VECTOR3             abspt;
  int                 tag;
  int                 info;
} LASER_COORD3;

typedef struct {
  RPOS3               estpos;
  VECTOR3             laseroffset;
  DECL_TYPE           declination;
  LASER_DATA          laser;
  int                 id;
  VECTOR3             origin;
  LASER_COORD3      * coord;
  DISTRIBUTION        dynamic;
} LASERSENS3_DATA;

typedef struct {
  int                 numswscans;
  LASERSENS3_DATA   * swscan;
} LASERSWEEP3_DATA;

typedef struct {
  double              axisdist;          /* distance between tilt axis and
					    laserrangefinder center */
  MOVEMENT3           displace;          /* position of the tilt axis to
					    the center of the robot */
} PANTILT_SETTINGS;
  
typedef struct {
  PANTILT_SETTINGS    pantilt;
} HARDWARE_SETTINGS;
  
typedef struct {
  int                 numsweeps;
  HARDWARE_SETTINGS   settings;
  LASERSWEEP3_DATA  * sweep;
  int                 numscans;
  LASERSENS2_DATA   * scan;
} LASERSCAN3_DATA;


typedef struct {
  struct timeval      time;
  RPOS3               rpos;
} POSSENS3_DATA;

typedef struct {
  struct timeval      time;
  DECL_TYPE           pos;
} AMTEC_POSITION;

typedef struct {
  double              space;
} REC3_SETTINGS;

typedef struct {

  /* settings */
  REC3_SETTINGS       settings;

  /* entries */
  int                 numentries;
  ENTRY_POSITION    * entry;  

  /* positions */
  int                 numpositions;
  POSSENS3_DATA     * psens;

  /* corrected positions */
  int                 numcpositions;
  POSSENS3_DATA     * cpsens;

  /* amtec positions */
  int                 numamtecpos;
  AMTEC_POSITION    * amtec;

  /* laser scans */
  LASERSCAN3_DATA     lsens;

  int                 numcompass;
  COMPASS3_DATA     * compass;

  int                 numgps;
  GPS_DATA          * gps;

} REC3_DATA;

typedef struct {
  int                numfaces;
  int              * facept;
} POLYGON_REF;

typedef struct {
  int                numvectors;
  int              * vecref;
} VECTORREF_SET;

typedef struct {
  int                numpoints;
  int              * ptref;
} POINTREF_SET;

typedef struct {
  int                numvalues;
  double           * val;
} VALUE_SET;

typedef struct {
  int                numpoints;
  int              * planept;
  PLANE_PARAMS       param;
} PLANE_POINTREF;

typedef struct {
  int                sweep;
  int                scan;
  int                beam;
} BEAMREF;

typedef struct {
  int                numpoints;
  BEAMREF          * planebeam;
  PLANE_PARAMS       param;
} PLANE_BEAMREF;

typedef struct {
  int                numpoints;
  int              * plane;
} POINT_PLANEREF;

typedef struct {
  int                numpoints;
  POINT3           * point;
  int                numpolygons;
  POLYGON_REF      * polygon;
  int                numplanes;
  PLANE_POINTREF   * plane;
} SPF_POINTS_DATA;

typedef struct {
  int                numplanes;
  PLANE_BEAMREF    * plane;
} PLANE_BEAM_DATA;

typedef struct {
  BEAMREF            ref0;
  BEAMREF            ref1;
  BEAMREF            ref2;
} TRIANGLE3_BEAMREF;

typedef struct {
  int                pt0;
  int                pt1;
  int                pt2;
} TRIANGLEREF;

typedef struct {
  int                 numtriangles;
  TRIANGLE3_BEAMREF * triangle;
} TRIANGLE3_BEAMREF_SET;

typedef struct {
  int                 numtriangles;
  TRIANGLEREF       * triangle;
} TRIANGLEREF_SET;

typedef struct {
  int                 numlines;
  LINE3             * line;
} LINE3_SET;

typedef struct {
  int          numpoints;
  double       xm;
  double       ym;
  double       phi;
  double       ndist;
  double       error;
  VECTOR2      pos;
} LINE2_LSQFT;

typedef struct {
  int idx0;
  int idx1;
} RANGE;

typedef struct {
  int          numranges;
  RANGE      * range;
} RANGE_SET;

typedef struct {
  int       len;
  double  * val;
} GAUSS_KERNEL;

typedef struct {
  double    y;
  double    u;
  double    v;
} YUV;

typedef struct {
  double    r;
  double    g;
  double    b;
} RGB;

typedef struct {
  double    h;
  double    s;
  double    v;
} HSV;

typedef struct {
  double    dx;
  double    dy;
  double    dz;
  double    droll;
  double    dyaw;
  double    dpitch;
  int       from;
  int       to;
} LINK;

typedef struct {
  int       id;
  double    x;
  double    y;
  double    z;
  double    roll;
  double    yaw;
  double    pitch;
  int       numlinks;
  LINK    * lnk;
} NODE;

typedef struct {
  int       numnodes;
  NODE    * nodes;
} MESH_DATA;

#define rad2deg rad2Deg
double        rad2Deg(double val);

#define deg2rad deg2Rad
double        deg2Rad(double val);

double        normalize_theta( double theta ); 

double        fsgn( double val );

int           sgn( int val );

double        random_gauss( void );

int           timeCompare ( struct timeval time1, struct timeval time2 );

double        timeDiff( struct timeval  t1, struct timeval t2);

void *        mdalloc(int ndim, int width, ...);

void          mdfree(void *tip, int ndim);

int           read_rec2d_file( char *filename, REC2_DATA *rec, int verbose );

int           write_rec2d_file( char *filename, REC2_DATA rec );

int           rec2_parse_line( char *line, REC2_DATA *rec, int alloc, int verbose );

int           read_rec3d_file( char *filename, REC3_DATA *rec, int verbose  );

int           write_rec3d_file( char *filename, REC3_DATA rec );

int           read_script( char *filename, REC2_DATA *script, int verbose );

int           read_beam_planes( char *filename, PLANE_BEAM_DATA *planes,
				int verbose );

void          init_spf_point_data( SPF_POINTS_DATA *spf );

int           insert_spf_point_data( char *filename, SPF_POINTS_DATA *spf  );

void          spf_get_component( SPF_POINTS_DATA spf, POINTREF_SET component,
				 SPF_POINTS_DATA * spf_comp );

void          spf_filter_small_components( SPF_POINTS_DATA spf,
					   int num_components, POINTREF_SET *components,
					   int min_comp_size, SPF_POINTS_DATA * spf_comp );

void          spf_write_data( SPF_POINTS_DATA spf, char * filename );

void          insert_smf_data( char *filename, SPF_POINTS_DATA *spf  );

VECTOR3       translate_and_rotate_vector3( VECTOR3 p, VECTOR3 rot,
					    VECTOR3 trans );

VECTOR3       rotate_and_translate_vector3( VECTOR3 p, VECTOR3 rot,
					    VECTOR3 trans );

VECTOR3       rotate_vector3( VECTOR3 p, VECTOR3 rot );

VECTOR3       rotate3_x( VECTOR3 p, double rot );

VECTOR3       rotate3_y( VECTOR3 p, double rot );

VECTOR3       rotate3_z( VECTOR3 p, double rot );

VECTOR2       rotate_vector2( VECTOR2 p, double angle );

VECTOR2       rotate_and_translate_vector2( VECTOR2 p, double rot,
					    VECTOR2 trans );

double        vector2_length( VECTOR2 v1 );

double        vector3_length( VECTOR3 v1 );

double        vector2_distance( VECTOR2 p1, VECTOR2 p2 );

double        vector3_distance( VECTOR3 v1, VECTOR3 v2 );

VECTOR3       vector3_add( VECTOR3 v1, VECTOR3 v2 );

VECTOR3       vector3_diff( VECTOR3 v1, VECTOR3 v2 );

VECTOR3       vector3_cross( VECTOR3 v1, VECTOR3 v2 );

VECTOR3       vector3_scalar_mult( VECTOR3 v1, double m );

double        vector3_dot_mult( VECTOR3 v1, VECTOR3 v2 );


void          compute_alloc_rec3_coordpts( REC3_DATA *rec );

void          compute_rel_rec3_coordpts( REC3_DATA *rec );

void          compute_abs_rec3_coordpts( REC3_DATA *rec );

void          compute_abs_scan_coordpts( LASERSENS3_DATA *data );

void          compute_rec2d_coordpts( REC2_DATA *rec );

VECTOR2       compute_laser_abs_point( RPOS2 rpos, double val,
				       RMOVE2 offset, double angle );

void          compute_estpos( REC3_DATA *rec );

void          mark_maxrange( REC3_DATA *rec, double MAX_RANGE );

void          compute_rec3_triangles( REC3_DATA rec,
				      TRIANGLE3_BEAMREF_SET *tri,
				      double minsize, double maxsize );

void          compute_vec2_convex_hull( VECTOR2_SET pts,
					VECTORREF_SET *hull );

void          init_bounding_box( BOUNDING_BOX3 *box );

void          update_bounding_box( VECTOR3 v, BOUNDING_BOX3 *box );

void          compute_bounding_box( VECTOR3_SET vset, BOUNDING_BOX3 *box );

void          compute_bounding_box_with_min_max( VECTOR3 min, VECTOR3 max,
						 BOUNDING_BOX3 *box );

void          compute_bounding_box_plus_dist( VECTOR3_SET vset,
					      BOUNDING_BOX3 *box, double dist);

double        compute_triangle3_area( TRIANGLE3 tri );


double        distance_vec3_to_plane( PLANE_PARAMS pl, VECTOR3 p );

double        distance_vec3_to_line3( VECTOR3 p, LINE3 l );

double        distance_vec2_to_line2( VECTOR2 p, LINE2 l );

double        compute_factor_to_line( VECTOR3 p, LINE3 l );

VECTOR3       projection_vec3_to_line( VECTOR3 p, LINE3 l );

VECTOR2       projection_vec3_to_vec2( PLANE_PARAMS pl, VECTOR3 pt );

void          projection_vec3set_to_vec2set( PLANE_PARAMS pl, VECTOR3_SET pts,
					     VECTOR2_SET *pset );

VECTOR3       projection_vec3_to_plane( PLANE_PARAMS pl, VECTOR3 p );

void          projection_vec3set_to_plane( PLANE_PARAMS pl, VECTOR3_SET pts,
					    VECTOR3_SET *pset );

PLANE_PARAMS  compute_plane_from_three_vec3( VECTOR3 p1, VECTOR3 p2,
					     VECTOR3 p3 );

int           intersection_two_planes( PLANE_PARAMS pl1, PLANE_PARAMS pl2,
				       LINE3* L );

double        gauss_function( double x, double mu, double sigma );

GAUSS_KERNEL  compute_gauss_kernel( int length );
/*
  int           compute_alpha_shape( VECTOR2_SET vset, int alpha, EDGEREF_SET *e_set );
*/

RMOVE2        compute_movement2_between_rpos2( RPOS2 s, RPOS2 e );

RPOS2         compute_rpos2_with_movement2( RPOS2 start, RMOVE2 move );

RPOS2         compute_rpos2_backwards_with_movement2( RPOS2 start,
						      RMOVE2 move );

double        convert_orientation_to_range( double angle );

double        compute_orientation_diff( double start, double end );

void          robot2map( RPOS2 pos, RPOS2 corr, RPOS2 *map );

void          map2robot( RPOS2 map, RPOS2 corr, RPOS2 *pos );

void          computeCorr( RPOS2 pos, RPOS2 map, RPOS2 *corr );

VECTOR3       compute_triangle_normal( TRIANGLE3 tri );

double        angle_between_two_vec3( VECTOR3 v1, VECTOR3 v2 );

LINE2_LSQFT   compute_lsqf_line( VECTOR2_SET p );


YUV           convert_from_rgb( RGB color );

RGB           convert_from_yuv( YUV color );

RGB           hsv_to_rgb( HSV color );


void          compute_forward_correction( RPOS2 pos, RPOS2 corr, RPOS2 *cpos );

void          compute_backward_correction( RPOS2 cpos, RPOS2 corr, RPOS2 *pos );

void          compute_correction_parameters( RPOS2 pos, RPOS2 cpos, RPOS2 *corr );

void          update_correction_parameters( RPOS2 cpos, RPOS2 delta, RPOS2 *corr );

QUAT          quaternions_conjugate( QUAT q );

VECTOR3       quaternions_convert_to_euler( QUAT q );

QUAT          quaternions_create_from_euler( VECTOR3 a );

double        quaternions_magnitude( QUAT q );

QUAT          quaternions_multiply( QUAT q1, QUAT q2 );

void          mesh_from_spf( SPF_POINTS_DATA scan, MESH_DATA * mesh );

int           mesh_components( MESH_DATA mesh, POINTREF_SET ** components );

void          convert_time( double tval, struct timeval *time );

#endif /* ifdef NAV_UTILS_H */

