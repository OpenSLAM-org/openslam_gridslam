#include "fast-slam.h"

void
grid_line_core( logtools_ivector2_t start, logtools_ivector2_t end, logtools_grid_line_t *line )
{
  int dx, dy, incr1, incr2, d, x, y, xend, yend, xdirflag, ydirflag;

  int cnt = 0;

  dx = abs(end.x-start.x); dy = abs(end.y-start.y);
  
  if (dy <= dx) {
    d = 2*dy - dx; incr1 = 2 * dy; incr2 = 2 * (dy - dx);
    if (start.x > end.x) {
      x = end.x; y = end.y;
      ydirflag = (-1);
      xend = start.x;
    } else {
      x = start.x; y = start.y;
      ydirflag = 1;
      xend = end.x;
    }
    line->grid[cnt].x=x;
    line->grid[cnt].y=y;
    cnt++;
    if (((end.y - start.y) * ydirflag) > 0) {
      while (x < xend) {
	x++;
	if (d <0) {
	  d+=incr1;
	} else {
	  y++; d+=incr2;
	}
	line->grid[cnt].x=x;
	line->grid[cnt].y=y;
	cnt++;
      }
    } else {
      while (x < xend) {
	x++;
	if (d <0) {
	  d+=incr1;
	} else {
	  y--; d+=incr2;
	}
	line->grid[cnt].x=x;
	line->grid[cnt].y=y;
	cnt++;
      }
    }		
  } else {
    d = 2*dx - dy;
    incr1 = 2*dx; incr2 = 2 * (dx - dy);
    if (start.y > end.y) {
      y = end.y; x = end.x;
      yend = start.y;
      xdirflag = (-1);
    } else {
      y = start.y; x = start.x;
      yend = end.y;
      xdirflag = 1;
    }
    line->grid[cnt].x=x;
    line->grid[cnt].y=y;
    cnt++;
    if (((end.x - start.x) * xdirflag) > 0) {
      while (y < yend) {
	y++;
	if (d <0) {
	  d+=incr1;
	} else {
	  x++; d+=incr2;
	}
	line->grid[cnt].x=x;
	line->grid[cnt].y=y;
	cnt++;
      }
    } else {
      while (y < yend) {
	y++;
	if (d <0) {
	  d+=incr1;
	} else {
	  x--; d+=incr2;
	}
	line->grid[cnt].x=x;
	line->grid[cnt].y=y;
	cnt++;
      }
    }
  }
  line->numgrids = cnt;
}

void
grid_line( logtools_ivector2_t start, logtools_ivector2_t end, logtools_grid_line_t *line ) {
  int i,j;
  int half;
  logtools_ivector2_t v;
  grid_line_core( start, end, line );
  if ( start.x!=line->grid[0].x ||
       start.y!=line->grid[0].y ) {
    half = line->numgrids/2;
    for (i=0,j=line->numgrids - 1;i<half; i++,j--) {
      v = line->grid[i];
      line->grid[i] = line->grid[j];
      line->grid[j] = v;
    }
  }
}
     
