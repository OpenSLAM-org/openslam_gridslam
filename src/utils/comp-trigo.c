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

double
distance_vec3_to_plane( PLANE_PARAMS pl, VECTOR3 p )
{
  double pld;
  double dist;

  pld = fsgn(pl.d) * (-sqrt(pl.a*pl.a + pl.b*pl.b + pl.c*pl.c) );
  dist = fabs( (pl.a*p.x + pl.b*p.y + pl.c*p.z + pl.d) / pld );

  return( dist );
}

double
distance_vec3_to_line3( VECTOR3 p, LINE3 l )
{
  VECTOR3 v = vector3_diff( l.point1, l.point0 );
  VECTOR3 w = vector3_diff( p, l.point0 );

  double c1 = vector3_dot_mult( w, v );
  double c2 = vector3_dot_mult( v, v );
  double b  = c1 / c2;
  
  VECTOR3 pb = vector3_add( l.point0, vector3_scalar_mult( v, b ) );

  return( vector3_distance( p, pb ) );
}

double
distance_vec2_to_line2( VECTOR2 p, LINE2 l ) {
  VECTOR2 hp;
  double d1 = l.point1.x - l.point0.x;
  double d2 = l.point1.y - l.point0.y;
  double u  =
    ( (p.x-l.point0.x)*d1 + (p.y-l.point0.y)*d2 ) / (d1*d1+d2*d2);
  hp.x = l.point0.x + u*d1;
  hp.y = l.point0.y + u*d2;
  return(vector2_distance(p,hp));
}


VECTOR3
projection_vec3_to_line( VECTOR3 p, LINE3 l )
{
  VECTOR3 v = vector3_diff( l.point1, l.point0 );
  VECTOR3 w = vector3_diff( p, l.point0 );

  double c1 = vector3_dot_mult( w, v );
  double c2 = vector3_dot_mult( v, v );
  double b  = c1 / c2;
  
  VECTOR3 pb = vector3_add( l.point0, vector3_scalar_mult( v, b ) );

  return( pb );
}

double
compute_factor_to_line( VECTOR3 p, LINE3 l )
{
  VECTOR3 v = vector3_diff( l.point1, l.point0 );
  VECTOR3 w = vector3_diff( p, l.point0 );

  double c1 = vector3_dot_mult( w, v );
  double c2 = vector3_dot_mult( v, v );
  double b  = c1 / c2;
  
  return( b );
}

VECTOR3
projection_vec3_to_plane( PLANE_PARAMS pl, VECTOR3 p )
{
  VECTOR3 pp;
  double d, pld;
  
  pld = -sqrt(pl.a*pl.a + pl.b*pl.b + pl.c*pl.c) ;
  d = (pl.a*p.x + pl.b*p.y + pl.c*p.z + pl.d) / pld;

  pp.x = p.x + (d * pl.a);
  pp.y = p.y + (d * pl.b);
  pp.z = p.z + (d * pl.c);

  return(pp);
}

void
projection_vec3set_to_plane( PLANE_PARAMS pl, VECTOR3_SET pts,
			     VECTOR3_SET *pset )
{
  int i;
  
  if ( (pset->vec == NULL) || (pset->numvectors == 0 ) ) {
    pset->vec = (VECTOR3 *) malloc( pts.numvectors * sizeof(VECTOR3) );
  } else if ( pset->numvectors != pts.numvectors ) {
    pset->vec = (VECTOR3 *) realloc( pset->vec,
				     pts.numvectors * sizeof(VECTOR3) );
  }
  pset->numvectors = pts.numvectors;
  
  for (i=0; i<pts.numvectors; i++) {
    pset->vec[i] = projection_vec3_to_plane( pl, pts.vec[i] );
  }
}

PLANE_PARAMS
compute_plane_from_three_vec3( VECTOR3 p1, VECTOR3 p2, VECTOR3 p3 )
{
  PLANE_PARAMS pl;
  pl.a = p1.y*(p2.z-p3.z) + p2.y*(p3.z-p1.z) + p3.y*(p1.z-p2.z);
  pl.b = p1.z*(p2.x-p3.x) + p2.z*(p3.x-p1.x) + p3.z*(p1.x-p2.x);
  pl.c = p1.x*(p2.y-p3.y) + p2.x*(p3.y-p1.y) + p3.x*(p1.y-p2.y);
  pl.d =
    - p1.x * ( p2.y * p3.z - p3.y * p2.z )
    - p2.x * ( p3.y * p1.z - p1.y * p3.z )
    - p3.x * ( p1.y * p2.z - p2.y * p1.z );
  return(pl);
}

VECTOR2
projection_vec3_to_vec2( PLANE_PARAMS pl, VECTOR3 pt )
{

  VECTOR3 p, pj1, pj2, pj3;
  VECTOR2 projp;
  double a, b, g, sq, pld;

  pld = fsgn(pl.d) * (-sqrt(pl.a*pl.a + pl.b*pl.b + pl.c*pl.c) );

  sq = -sqrt( pl.a * pl.a + pl.b * pl.b + pl.c * pl.c );
  a = acos( ( pl.a ) / sq );
  b = acos( ( pl.b ) / sq );
  g = acos( ( pl.c ) / sq );

  p = projection_vec3_to_plane( pl, pt );
  
  pj1.x = p.x*cos(a) - p.y*sin(a);
  pj1.y = p.x*sin(a) + p.y*cos(a);
  pj1.z = p.z;
  
  pj2.x = pj1.x*cos(b) + pj1.z*sin(b);
  pj2.y = pj1.y;
  pj2.z = -pj1.x*sin(b) + pj1.z*cos(b);
  
  pj3.x = pj2.x;
  pj3.y = pj2.y*cos(g) - pj2.z*sin(g);
  pj3.z = pj2.y*sin(g) + pj2.z*cos(g);
  
  projp.x = pj2.y;
  projp.y = pj2.z;

  return(projp);
}


void
projection_vec3set_to_vec2set( PLANE_PARAMS pl, VECTOR3_SET pts,
			       VECTOR2_SET *pset )
{

  int i;
  VECTOR3 p, *pj1, *pj2, *pj3;

  double a, b, g, sq, pld;

  /*
  Matrix for a 3D rotation around the x axis of q 

      1       0       0       0
      0       cos(q)  sin(q)  0
      0      -sin(q)  cos(q)  0
      0       0       0       1

  Matrix for a 3D rotation around the y axis of q 

      cos(q)  0      -sin(q)  0
      0       1       0       0
      sin(q)  0       cos(q)  0
      0       0       0       1

  Matrix for a 3D rotation around the z axis of q
  
      cos(q)  sin(q)  0       0
     -sin(q)  cos(q)  0       0
      0       0       1       0
      0       0       0       1
  */
  
  pld = fsgn(pl.d) * (-sqrt(pl.a*pl.a + pl.b*pl.b + pl.c*pl.c) );

  if ( (pset->vec == NULL) || (pset->numvectors == 0 ) ) {
    pset->vec = (VECTOR2 *) malloc( pts.numvectors * sizeof(VECTOR2) );
  } else if ( pset->numvectors != pts.numvectors ) {
    pset->vec = (VECTOR2 *) realloc( pset->vec,
				     pts.numvectors * sizeof(VECTOR2) );
  }
  pset->numvectors = pts.numvectors;
  
  pj1 = (VECTOR3 *) malloc( pts.numvectors * sizeof(VECTOR3) );
  pj2 = (VECTOR3 *) malloc( pts.numvectors * sizeof(VECTOR3) );
  pj3 = (VECTOR3 *) malloc( pts.numvectors * sizeof(VECTOR3) );

  sq = -sqrt( pl.a * pl.a + pl.b * pl.b + pl.c * pl.c );
  a = acos( ( pl.a ) / sq );
  b = acos( ( pl.b ) / sq );
  g = acos( ( pl.c ) / sq );

  for (i=0; i<pts.numvectors; i++) {
    p = projection_vec3_to_plane( pl, pts.vec[i] );
    /*
      pj[i].x = p.x*cos(a) + p.z*sin(b);
      pj[i].y = p.x*sin(a)*sin(b) + p.y*cos(a) - p.z*sin(a)*cos(b);
      pj[i].z = -p.x*cos(a)*sin(b) + p.y*sin(a) + p.z*cos(a)*cos(b);
    */

    pj1[i].x = p.x*cos(a) - p.y*sin(a);
    pj1[i].y = p.x*sin(a) + p.y*cos(a);
    pj1[i].z = p.z;

    pj2[i].x = pj1[i].x*cos(b) + pj1[i].z*sin(b);
    pj2[i].y = pj1[i].y;
    pj2[i].z = -pj1[i].x*sin(b) + pj1[i].z*cos(b);

    pj3[i].x = pj2[i].x;
    pj3[i].y = pj2[i].y*cos(g) - pj2[i].z*sin(g);
    pj3[i].z = pj2[i].y*sin(g) + pj2[i].z*cos(g);

    pset->vec[i].x = pj2[i].y;
    pset->vec[i].y = pj2[i].z;

  }

  free(pj1);
  free(pj2);
  free(pj3);

}

int
test_route_intersection( VECTOR2 a, VECTOR2 b, VECTOR2 c, VECTOR2 d )
{
  double r,s;

  /*
      (Ay-Cy)(Dx-Cx)-(Ax-Cx)(Dy-Cy)
  r = -----------------------------  (eqn 1)
      (Bx-Ax)(Dy-Cy)-(By-Ay)(Dx-Cx)

      (Ay-Cy)(Bx-Ax)-(Ax-Cx)(By-Ay)
  s = -----------------------------  (eqn 2)
      (Bx-Ax)(Dy-Cy)-(By-Ay)(Dx-Cx)
  */

  r =
    ( (a.y-c.y)*(d.x-c.x)-(a.x-c.x)*(d.y-c.y) ) /
    ( (b.x-a.x)*(d.y-c.y)-(b.y-a.y)*(d.x-c.x) );
    
  s =
    ( (a.y-c.y)*(b.x-a.x)-(a.x-c.x)*(b.y-a.y) ) /
    ( (b.x-a.x)*(d.y-c.y)-(b.y-a.y)*(d.x-c.x) );

  /*
    0<=r<=1 & 0<=s<=1, intersection exists
    r<0 or r>1 or s<0 or s>1 line segments do not intersect
  */

  if (r>=0 && r<=1 && s>=0 && s<=1)
    return(1);
  else
    return(0);
  
}

/* 1=inside, 0=outside                 */
int
test_vec2_in_polygon( VECTOR2 p, VECTOR2_SET poly )
{
  double xnew,ynew, xold,yold;
  double x1,y1, x2,y2;
  int i, inside=0;
  
  if (poly.numvectors < 3) {
    return(0);
  }
  xold=poly.vec[poly.numvectors-1].x;
  yold=poly.vec[poly.numvectors-1].y;
  for (i=0 ; i < poly.numvectors ; i++) {
    xnew=poly.vec[i].x;
    ynew=poly.vec[i].y;
    if (xnew > xold) {
      x1=xold;
      x2=xnew;
      y1=yold;
      y2=ynew;
    } else {
      x1=xnew;
      x2=xold;
      y1=ynew;
      y2=yold;
    }
    if ( (xnew < p.x) == (p.x <= xold) &&
	 (p.y-y1)*(x2-x1) < (y2-y1)*(p.x-x1) ) {
      inside=!inside;
    }
    xold=xnew;
    yold=ynew;
  }
  return(inside);
}

int
inlist( int num, int *l, int len )
{
  int i;
  for (i=0;i<len;i++) {
    if (l[i]==num)
      return(i);
  }
  return(-1);
}

double
area( VECTOR2 a, VECTOR2 b, VECTOR2 c )
{
  return( (b.x-a.x) * (c.y-a.y) -
	  (c.x-a.x) * (b.y-a.y) );
}

int
left( VECTOR2 a, VECTOR2 b, VECTOR2 c )
{
  return( area(a,b,c)>0.0 );
}

int
collinear( VECTOR2 a, VECTOR2 b, VECTOR2 c )
{
  return( area(a,b,c)==0.0 );
}

int
xor( int x, int y )
{
  return( !x^!y );
}

int
intersect_prop( VECTOR2 a, VECTOR2 b, VECTOR2 c, VECTOR2 d )
{
  if ( collinear(a,b,c) ||
       collinear(a,b,d) ||
       collinear(c,d,a) ||
       collinear(c,d,b) ) {
    return(0);
  }
  return( xor( left(a,b,c), left(a,b,d) ) &&
	  xor( left(c,d,a), left(c,d,b) ) );
}

int
compute_hull( VECTOR2_SET p, VECTOR3_SET p3d,
	      double gap_distance, EDGEREF_SET *eS )
{
  int               loop, start, last, i,j;
  int               lnr, best, min, cut, l, ret = 0, cnt;
  double            as, angle, bAngle, d, gdistance;
  double            compareA=0.0, compareAngle, angleD;
  int               *hp, hc=0;
  VECTOR2_SET       poly;


  poly.numvectors  = 0;
  poly.vec         = (VECTOR2 *) malloc( 10000 * sizeof( VECTOR2 ) );
  
  hp = (int *) malloc( p3d.numvectors * sizeof(int) );

  min=0;
  for (i=1; i<p3d.numvectors; i++)
    if ( p.vec[i].x<p.vec[min].x )
      min=i;

  hp[0] = min;
  hc++;
  j = min;
  best = -1; last = -1;
  compareAngle = 0;
  fprintf( stderr, "hull points: " );
  bAngle = 0;
  start = 0;
  loop = 1;
  while(loop) {
    gdistance = gap_distance;
    bAngle = MAXDOUBLE;
    for (i=0; i<p3d.numvectors; i++) {
      d  = sqrt( ( p3d.vec[i].x - p3d.vec[j].x ) *
		 ( p3d.vec[i].x - p3d.vec[j].x ) +
		 ( p3d.vec[i].y - p3d.vec[j].y ) *
		 ( p3d.vec[i].y - p3d.vec[j].y ) +
		 ( p3d.vec[i].z - p3d.vec[j].z ) *
		 ( p3d.vec[i].z - p3d.vec[j].z ) );
      /* der Abstand darf nicht ueberschritten werden */
      if ( (d>0) && (d<gdistance) ) {
	d  = sqrt( ( p.vec[i].x - p.vec[j].x ) * ( p.vec[i].x - p.vec[j].x ) +
		   ( p.vec[i].y - p.vec[j].y ) * ( p.vec[i].y - p.vec[j].y ) );
	as = asin((p.vec[i].x-p.vec[j].x)/d);
	if (p.vec[i].y>=p.vec[j].y) {
	  angle = as;
	} else {
	  angle = M_PI - as;
	}
	if (angle<compareAngle)
	  angleD = 2.0 * M_PI + angle;
	else 
	  angleD = angle;
	if (angleD<bAngle) {
	  cut=0;
	  for (l=0;l<hc-2;l++) {
	    ret = intersect_prop( p.vec[i], p.vec[hp[hc-1]],
				  p.vec[hp[l]], p.vec[hp[l+1]] );
	    cut += ret;
	  }
	  if (cut==0) {
	    best = i;
	    bAngle = angleD;
	    compareA = angle;
	  }
	} /* end: if (angleD<bAngle) */
      } /* end: if ( (d>0) && (d<gdistance) ) */
    } /* end: for all points */
    lnr = inlist(best,hp,hc);
    if ( (lnr!=-1) && (best!=min) ) {
      /* eine Schleife wurde gefunden! Nehme Schleife als eigenes Polygon! */
      start = lnr;
      loop = 0;
      break;
    }
    if (best==min) {
      loop = 0;
    } else if (best==last) {
      fprintf( stderr, "\n--- break (%d)---", best );
      break;
    } else {
      fprintf( stderr, "*" );
      last = best;
      j = best;
      compareAngle = compareA-0.99999999999*M_PI;
      if (compareAngle<-M_PI)
	compareAngle += 2.0 * M_PI;
      hp[hc++] = best;
    }
  } /* end of while(loop) */
  fprintf( stderr, "\n" );
  
  for (i=start; i<hc; i++) {
    poly.vec[i-start] = p.vec[hp[i]];
  }
  cnt=0;
  for (i=start; i<p3d.numvectors; i++) {
    poly.numvectors = hc-1-start;
    if ( !test_vec2_in_polygon( p.vec[i], poly ) ) {
      cnt++;
    }
  }
  fprintf( stderr, "############ free points = %d\n", cnt );
  if (cnt>1000)
    ret=1;
  else
    ret=0;
  eS->numedges = hc-start;
  eS->edge = (EDGEREF *) malloc( hc * sizeof(EDGEREF) );
  for (i=start; i<(hc-1); i++) {
    eS->edge[i-start].start = hp[i];
    eS->edge[i-start].end = hp[i+1];
  }
  eS->edge[hc-1-start].start = hp[hc-1];
  eS->edge[hc-1-start].end = min;
  fprintf( stderr, "############ num Edges = %d\n", hc-start  );
  free(hp);
  return(ret);
}

VECTOR3
compute_triangle_normal( TRIANGLE3 tri )
{
  return( vector3_cross( vector3_diff( tri.point1, tri.point0 ),
			 vector3_diff( tri.point2, tri.point0 ) ) );
}

double
angle_between_two_vec3( VECTOR3 v1, VECTOR3 v2 )
{
  return( acos( ( v1.x*v2.x + v1.y*v2.y + v1.z*v2.z ) /
		( vector3_length(v1) * vector3_length(v2) ) ) );
}
