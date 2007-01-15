#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <sys/types.h>
#include <sys/time.h>
#include <signal.h>
#include <unistd.h>
#include <math.h>
#include <string.h>
#include <ctype.h>
#include <values.h>

#include <navigation/utils.h>

#include "defines.h"

/*
   intersect3D_2Planes(): the 3D intersect of two planes
   Input:  two planes Pn1 and Pn2
    Output: *L = the intersection line (when it exists)
    Return: 0 = disjoint (no intersection)
            1 = the two planes coincide
            2 = intersection in the unique line *L
*/

#define PLANE_EPSILON 0.0000001
#define dot(u,v)   ((u).x * (v).x + (u).y * (v).y + (u).z * (v).z)

typedef struct {
  VECTOR3 n;
  VECTOR3 V0;
} Plane;

typedef struct {
  VECTOR3 P0;
  VECTOR3 P1;
} Line;

int
intersect3D_2Planes( Plane Pn1, Plane Pn2, Line* L )
{
  VECTOR3  u = vector3_cross( Pn1.n, Pn2.n );     /* cross product */
  VECTOR3  iP;                                    /* intersect point */
  VECTOR3  v;
  
  double   ax = (u.x >= 0 ? u.x : -u.x);
  double   ay = (u.y >= 0 ? u.y : -u.y);
  double   az = (u.z >= 0 ? u.z : -u.z);

  double   d1, d2;                              /* the constants in the
 					           2 plane equations */

  int      maxc;


  /* test if the two planes are parallel */
  if ( (ax+ay+az) < PLANE_EPSILON ) {       /* Pn1 and Pn2 are near parallel */
    /* test if disjoint or coincide */
    v = vector3_diff( Pn2.V0, Pn1.V0 );
    if (dot(Pn1.n, v) == 0)       /* Pn2.V0 lies in Pn1 */
      return 1;                   /* Pn1 and Pn2 coincide */
    else 
      return 0;                   /* Pn1 and Pn2 are disjoint */
  }

  /* Pn1 and Pn2 intersect in a line
     first determine max abs coordinate of cross product */
  
  /* find max coordinate */
  if (ax > ay) {
    if (ax > az)
      maxc = 1;
    else
      maxc = 3;
  } else {
    if (ay > az)
      maxc = 2;
    else
      maxc = 3;
  }

  /* next, to get a point on the intersect line
     zero the max coord, and solve for the other two */
  
  d1 = -dot(Pn1.n, Pn1.V0);  /* note: could be pre-stored with plane */
  d2 = -dot(Pn2.n, Pn2.V0);  /* ditto */

  switch (maxc) {            /* select max coordinate */
  case 1:                    /* intersect with x=0 */
    iP.x = 0;
    iP.y = (d2*Pn1.n.z - d1*Pn2.n.z) / u.x;
    iP.z = (d1*Pn2.n.y - d2*Pn1.n.y) / u.x;
    break;
  case 2:                    /* intersect with y=0 */
    iP.x = (d1*Pn2.n.z - d2*Pn1.n.z) / u.y;
    iP.y = 0;
    iP.z = (d2*Pn1.n.x - d1*Pn2.n.x) / u.y;
    break;
  case 3:                    /* intersect with z=0 */
    iP.x = (d2*Pn1.n.y - d1*Pn2.n.y) / u.z;
    iP.y = (d1*Pn2.n.x - d2*Pn1.n.x) / u.z;
    iP.z = 0;
  }
  L->P0 = iP;
  L->P1 = vector3_add( iP, u );
  
  return 2;
}
/*===================================================================*/

int
intersection_two_planes( PLANE_PARAMS pl1, PLANE_PARAMS pl2, LINE3* L )
{
  int r;
  Line l;
  double vl;
  
  Plane a, b;

  a.n.x  = pl1.a;
  a.n.y  = pl1.b;
  a.n.z  = pl1.c;

  vl = vector3_length( a.n );
  
  a.V0.x = -pl1.d * ( pl1.a / vl );
  a.V0.y = -pl1.d * ( pl1.b / vl );
  a.V0.z = -pl1.d * ( pl1.c / vl );

  b.n.x  = pl2.a;
  b.n.y  = pl2.b;
  b.n.z  = pl2.c;
  
  vl = vector3_length( b.n );
  
  b.V0.x = -pl2.d * ( pl2.a / vl );
  b.V0.y = -pl2.d * ( pl2.b / vl );
  b.V0.z = -pl2.d * ( pl2.c / vl );

  r = intersect3D_2Planes( a, b, &l );

  L->point0 = l.P0;
  L->point1 = l.P1;
  
  return(r);
}

#define EPSILON 0.000001
#define CROSS(dest,v1,v2) \
          dest[0]=v1[1]*v2[2]-v1[2]*v2[1]; \
          dest[1]=v1[2]*v2[0]-v1[0]*v2[2]; \
          dest[2]=v1[0]*v2[1]-v1[1]*v2[0];
#define DOT(v1,v2) (v1[0]*v2[0]+v1[1]*v2[1]+v1[2]*v2[2])
#define SUB(dest,v1,v2) \
          dest[0]=v1[0]-v2[0]; \
          dest[1]=v1[1]-v2[1]; \
          dest[2]=v1[2]-v2[2]; 

int
intersect_triangle(double orig[3], double dir[3],
                   double vert0[3], double vert1[3], double vert2[3],
                   double *t, double *u, double *v)
{
   double edge1[3], edge2[3], tvec[3], pvec[3], qvec[3];
   double det,inv_det;

   /* find vectors for two edges sharing vert0 */
   SUB(edge1, vert1, vert0);
   SUB(edge2, vert2, vert0);

   /* begin calculating determinant - also used to calculate U parameter */
   CROSS(pvec, dir, edge2);

   /* if determinant is near zero, ray lies in plane of triangle */
   det = DOT(edge1, pvec);

#ifdef TEST_CULL           /* define TEST_CULL if culling is desired */
   if (det < EPSILON)
      return 0;

   /* calculate distance from vert0 to ray origin */
   SUB(tvec, orig, vert0);

   /* calculate U parameter and test bounds */
   *u = DOT(tvec, pvec);
   if (*u < 0.0 || *u > det)
      return 0;

   /* prepare to test V parameter */
   CROSS(qvec, tvec, edge1);

    /* calculate V parameter and test bounds */
   *v = DOT(dir, qvec);
   if (*v < 0.0 || *u + *v > det)
      return 0;

   /* calculate t, scale parameters, ray intersects triangle */
   *t = DOT(edge2, qvec);
   inv_det = 1.0 / det;
   *t *= inv_det;
   *u *= inv_det;
   *v *= inv_det;
#else                    /* the non-culling branch */
   if (det > -EPSILON && det < EPSILON)
     return 0;
   inv_det = 1.0 / det;

   /* calculate distance from vert0 to ray origin */
   SUB(tvec, orig, vert0);

   /* calculate U parameter and test bounds */
   *u = DOT(tvec, pvec) * inv_det;
   if (*u < 0.0 || *u > 1.0)
     return 0;

   /* prepare to test V parameter */
   CROSS(qvec, tvec, edge1);

   /* calculate V parameter and test bounds */
   *v = DOT(dir, qvec) * inv_det;
   if (*v < 0.0 || *u + *v > 1.0)
     return 0;

   /* calculate t, ray intersects triangle */
   *t = DOT(edge2, qvec) * inv_det;
#endif
   return 1;
}

