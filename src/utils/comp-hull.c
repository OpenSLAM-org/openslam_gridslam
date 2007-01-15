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

typedef struct {
  double          x;
  double          y;
  int             id;
} VECTOR2ID;

typedef struct {
  int             numvectors;
  VECTOR2ID     * vec;
} VECTOR2ID_SET;


/* isLeft(): tests if a point is Left|On|Right of an infinite line. */
/*    Input:  three points P0, P1, and P2 */
/*    Return: >0 for P2 left of the line through P0 and P1 */
/*            =0 for P2 on the line */
/*            <0 for P2 right of the line */
/*    See: the January 2001 Algorithm on Area of Triangles */
double
isLeft( VECTOR2ID P0, VECTOR2ID P1, VECTOR2ID P2 )
{
    return (P1.x - P0.x)*(P2.y - P0.y) - (P2.x - P0.x)*(P1.y - P0.y);
}

#define EPSILON  0.0000000001

int
compareVec2( const void *a, const void *b )
{
  VECTOR2ID da = (*(const VECTOR2ID *)a);
  VECTOR2ID db = (*(const VECTOR2ID *)b);
  if (db.x-da.x>EPSILON) {
    return(-1);
  } else if (da.x-db.x>EPSILON) {
    return(1);
  } else {
    if (db.y-da.y>EPSILON) {
    return(-1);
    } else if (da.y-db.y>EPSILON) {
      return(1);
    } else {
      return(0);
    }
  }
}


/*=================================================================== */
void
compute_vec2_convex_hull( VECTOR2_SET IP, VECTORREF_SET *hull )
{
  int    *H;
  /* the output array H[] will be used as the stack */
  int    bot=0, top=(-1);   /* indices for bottom and top of the stack */
  int    i;                 /* array scan index */
  
  /* Get the indices of points with min x-coord and min|max y-coord */
  int    maxmin, maxmax, minmax;
  int    minmin = 0;

  double xmin, xmax;

  VECTOR2ID_SET P;

  P.vec = (VECTOR2ID *) malloc( IP.numvectors * sizeof(VECTOR2ID) );
  
  for (i=0;i<IP.numvectors;i++) {
    P.vec[i].x = IP.vec[i].x;
    P.vec[i].y = IP.vec[i].y;
    P.vec[i].id = i;
  }
  P.numvectors = IP.numvectors;
  
  H = (int *) malloc( IP.numvectors * sizeof(int) );

  qsort( P.vec, P.numvectors, sizeof(VECTOR2ID), compareVec2 );

  xmin = P.vec[0].x;

  for (i=1; i<P.numvectors; i++)
    if (P.vec[i].x != xmin) break;
  minmax = i-1;
  if (minmax == P.numvectors-1) {
    /* degenerate case: all x-coords == xmin */
    H[++top] = minmin;
    if (P.vec[minmax].y != P.vec[minmin].y) /* a nontrivial segment */
      H[++top] = minmax;
    hull->numvectors = top+1;
    hull->vecref = (int *) malloc( hull->numvectors * sizeof(int) );
    for (i=0;i<hull->numvectors;i++)
      hull->vecref[i] = P.vec[H[i]].id;
    free(P.vec);
    free(H);
    return;
  }
  
  /* Get the indices of points with max x-coord and min|max y-coord */
  maxmax = P.numvectors-1;
  xmax = P.vec[P.numvectors-1].x;
  for (i=P.numvectors-2; i>=0; i--)
    if (P.vec[i].x != xmax) break;
  maxmin = i+1;
  
  /* Compute the lower hull on the stack H */
  H[++top] = minmin;       /* push minmin point onto stack */
  i = minmax;
  while (++i <= maxmin) {
    /* the lower line joins P.vec[minmin] with P.vec[maxmin] */
    if (isLeft( P.vec[minmin], P.vec[maxmin], P.vec[i]) >= 0 && i < maxmin)
      continue;            /* ignore P.vec[i] above or on the lower line */
    
    while (top > 0) {      /* there are at least 2 points on the stack */
      /* test if P.vec[i] is left of the line at the stack top */
      if (isLeft( P.vec[H[top-1]], P.vec[H[top]], P.vec[i]) > 0)
	break;             /* P.vec[i] is a new hull vertex */
      else
	top--;             /* pop top point off stack */
    }
    H[++top] = i;          /* push P.vec[i] onto stack */
  }
  
  /* Next, compute the upper hull on the stack H above the bottom hull */
  if (maxmax != maxmin)    /* if distinct xmax points */
    H[++top] = maxmax;     /* push maxmax point onto stack */
  bot = top;               /* the bottom point of the upper hull stack */
  i = maxmin;
  while (--i >= minmax) {
    /* the upper line joins P.vec[maxmax] with P.vec[minmax] */
    if (isLeft( P.vec[maxmax], P.vec[minmax], P.vec[i]) >= 0 && i > minmax)
      continue;            /* ignore P.vec[i] below or on the upper line */
    
    while (top > bot) {    /* at least 2 points on the upper stack */
      /* test if P.vec[i] is left of the line at the stack top */
      if (isLeft( P.vec[H[top-1]], P.vec[H[top]], P.vec[i]) > 0)
	break;             /* P.vec[i] is a new hull vertex */
      else
	top--;             /* pop top point off stack */
    }
    H[++top] = i;          /* push P.vec[i] onto stack */
  }
  if (minmax != minmin)
    H[++top] = minmin;     /* push joining endpoint onto stack */

  hull->numvectors = top+1;
  hull->vecref = (int *) malloc( hull->numvectors * sizeof(int) );
  for (i=0;i<hull->numvectors;i++)
    hull->vecref[i] = P.vec[H[i]].id;
  free(P.vec);
  free(H);

}
