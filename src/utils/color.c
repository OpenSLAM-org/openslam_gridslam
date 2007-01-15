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

YUV
rgb_to_yuv( RGB color )
{
  YUV ret;
  ret.y =  (0.257 * color.r) + (0.504 * color.g) + (0.098 * color.b) + 16;
  ret.u = -(0.148 * color.r) - (0.291 * color.g) + (0.439 * color.b) + 128;
  ret.v =  (0.439 * color.r) - (0.368 * color.g) - (0.071 * color.b) + 128;
  return(ret);
}

RGB
yuv_to_rgb( YUV color )
{
  RGB ret;
  ret.b = 1.164*(color.y-16)                       + 2.018*(color.u-128);
  ret.g = 1.164*(color.y-16) - 0.813*(color.v-128) - 0.391*(color.u-128);
  ret.r = 1.164*(color.y-16) + 1.596*(color.v-128);
  return(ret);
}

#define NO_HUE   -1

HSV
rgb_to_hsv( RGB color )
{
  HSV    ret;
  double
    max = MAX (color.r, MAX (color.g, color.b)),
    min = MIN (color.r, MIN (color.g, color.b));
  double delta = max - min;

  ret.v = max;
  if (max != 0.0)
    ret.s = delta / max;
  else
    ret.s = 0.0;
  if (ret.s == 0.0)
    ret.h = NO_HUE;
  else {
    if (color.r == max)
      ret.h = (color.g - color.b) / delta;
    else if (color.g == max)
      ret.h = 2 + (color.b - color.r) / delta;
    else if (color.b == max)
      ret.h = 4 + (color.r - color.g) / delta;
    ret.h *= 60.0;
    if (ret.h < 0)
      ret.h += 360.0;
    ret.h /= 360.0;
  }
  return(ret);
}

/*
HSV
rgb_to_hsv( RGB color )
{
  HSV ret;
  double minv, maxv, delta;
  
  minv = MIN3( color.r, color.g, color.b );
  ret.v = maxv = MAX3( color.r, color.g, color.b );
  
  delta = maxv - minv;
  
  if( maxv != 0 )
    ret.s = delta / maxv; 
  else {
    ret.s = 0;
    ret.h = -1;
    return(ret);
  }

  if( color.r == maxv )
    ret.h = ( color.g - color.b ) / delta;      
  else if( color.g == maxv )
    ret.h = 2 + ( color.b - color.r ) / delta;  
  else
    ret.h = 4 + ( color.r - color.g ) / delta;  
  
  ret.h *= 60;    
  if( ret.h < 0 )
    ret.h += 360;

  return(ret);
}
*/

RGB
hsv_to_rgb( HSV color )
{
   RGB ret;
   int i;
   double aa, bb, cc, f;

  if (color.s == 0)
    ret.r = ret.g = ret.b = color.v;
  else {
    if (color.h == 1.0)
      color.h = 0;
    color.h *= 6.0;
    i = floor (color.h);
    f = color.h - i;
    aa = color.v * (1 - color.s);
    bb = color.v * (1 - (color.s * f));
    cc = color.v * (1 - (color.s * (1 - f)));
    switch (i) {
    case 0:
      ret.r = color.v;
      ret.g = cc;
      ret.b = aa;
      break;
    case 1:
      ret.r = bb;
      ret.g = color.v;
      ret.b = aa;
      break;
    case 2:
      ret.r = aa;
      ret.g = color.v;
      ret.b = cc;
      break;
    case 3:
      ret.r = aa;
      ret.g = bb;
      ret.b = color.v;
      break;
    case 4:
      ret.r = cc;
      ret.g = aa;
      ret.b = color.v;
      break;
    case 5:
      ret.r = color.v;
      ret.g = aa;
      ret.b = bb;
      break;
    }
  }
  return(ret);
}

/*
RGB
hsv_tp_rgb( HSV color )
{
  RGB ret;
  int i;
  double f, p, q, t;
  
  if( color.s == 0 ) {
    ret.r = ret.g = ret.b = color.v;
    return(ret);
  }
  
  color.h /= 60;                    
  i = floor( color.h );
  f = color.h - i;                  
  p = color.v * ( 1 - color.s );
  q = color.v * ( 1 - color.s * f );
  t = color.v * ( 1 - color.s * ( 1 - f ) );
  
  switch( i ) {
  case 0:
    ret.r = color.v;
    ret.g = t;
    ret.b = p;
    break;
  case 1:
    ret.r = q;
    ret.g = color.v;
    ret.b = p;
    break;
  case 2:
    ret.r = p;
    ret.g = color.v;
    ret.b = t;
    break;
  case 3:
    ret.r = p;
    ret.g = q;
    ret.b = color.v;
    break;
  case 4:
    ret.r = t;
    ret.g = p;
    ret.b = color.v;
    break;
  default:             
    ret.r = color.v;
    ret.g = p;
    ret.b = q;
    break;
  }
  return(ret);
  
}
*/
