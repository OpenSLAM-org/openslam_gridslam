#ifndef GEOMETRY_H
#define GEOMETRY_H


#ifndef MIN
#define MIN(x,y) (x < y ? x : y)
#endif

#ifndef MIN3
#define MIN3(x,y,z) MIN(MIN(x,y),z)
#endif

#ifndef MAX
#define MAX(x,y) (x > y ? x : y)
#endif

#ifndef MAX3
#define MAX3(x,y,z) MAX(MAX(x,y),z)
#endif


#define INSIDE   0
#define OUTSIDE  1

typedef struct {
  double        x;
  double        y;
  double        z;
} VECTOR3;

typedef struct {
  double        q0;
  double        q1;
  double        q2;
  double        q3;
} QUAT;

typedef struct {
  int           numvectors;
  VECTOR3     * vec;
} VECTOR3_SET;

#define POINT3 VECTOR3

typedef struct {
  int           numpoints;
  POINT3      * p;
} POINT3_SET;

typedef struct {
  int           x;
  int           y;
  int           z;
} iVECTOR3;

typedef struct {
  int           numvectors;
  iVECTOR3    * vec;
} iVECTOR3_SET;

#define iPOINT3 iVECTOR3

typedef struct {
  double        x;
  double        y;
} VECTOR2;

typedef struct {
  int           numvectors;
  VECTOR2     * vec;
} VECTOR2_SET;

typedef struct {
  int           x;
  int           y;
} iVECTOR2;

typedef struct {
  int           numvectors;
  iVECTOR2    * vec;
} iVECTOR2_SET;

#define POINT2 VECTOR2

typedef struct {
  int           numpoints;
  POINT2      * p;
} POINT2_SET;

#define iPOINT2 iVECTOR2

typedef struct {  
  VECTOR3       origin;
  VECTOR3       direction;
} RAY3;

typedef struct {
  VECTOR3          origin;
  VECTOR3          edge0;
  VECTOR3          edge1;
} TRIANGLE_EDGE3;

typedef struct {
  VECTOR3          point0;
  VECTOR3          point1;
  VECTOR3          point2;
} TRIANGLE3;

typedef struct {
  int              numtriangles;
  TRIANGLE3      * triangle;
} TRIANGLE3_SET;

typedef struct {
  /* Ax + By + CZ + D = 0 */
  double      a;
  double      b;
  double      c;
  double      d;
} PLANE_PARAMS;


typedef struct {
  VECTOR3     point0;
  VECTOR3     point1;
} LINE3;

typedef struct {
  VECTOR2     point0;
  VECTOR2     point1;
} LINE2;

typedef struct {
  int         start;
  int         end;
} EDGEREF;

typedef struct {
  int              numedges;
  EDGEREF        * edge;
} EDGEREF_SET;


typedef struct {
  int                numpoints;
  POINT2           * pt;
} POLYGON;

#endif
