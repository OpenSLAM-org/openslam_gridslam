#include <time.h>
#include <math.h>
#include <signal.h>
#include <unistd.h>
#include <string.h>
#include <ctype.h>
#include <values.h>

#include <navigation/utils.h>

QUAT
quaternions_conjugate( QUAT q )
{
  QUAT qq;
  qq.q0 = q.q0;
  qq.q1 = -q.q1;
  qq.q2 = -q.q2;
  qq.q3 = -q.q3;
  return(qq);
}

VECTOR3
quaternions_convert_to_euler( QUAT q )
{
  VECTOR3 a;
  a.x = atan( 2.0 * (q.q2*q.q3+q.q0*q.q1) / 
	      (q.q0*q.q0-q.q1*q.q1-q.q2*q.q2+q.q3*q.q3) );
  a.y = asin( -2.0*(q.q1*q.q3-q.q0*q.q2) );
  a.z = atan( 2.0 * (q.q1*q.q2+q.q0*q.q3) /
	      (q.q0*q.q0+q.q1*q.q1-q.q2*q.q2-q.q3*q.q3) );
  return(a);
}

QUAT
quaternions_create_from_euler( VECTOR3 a )
{
  QUAT q;
  double w1 = a.x/2.0;
  double w2 = a.y/2.0;
  double w3 = a.z/2.0;
  q.q0 = cos(w1)*cos(w2)*cos(w3) + sin(w1)*sin(w2)*sin(w3);
  q.q1 = sin(w1)*cos(w2)*cos(w3) - cos(w1)*sin(w2)*sin(w3);
  q.q2 = cos(w1)*sin(w2)*cos(w3) + sin(w1)*cos(w2)*sin(w3);
  q.q3 = cos(w1)*cos(w2)*sin(w3) - sin(w1)*sin(w2)*sin(w3);
  return(q);
}

double
quaternions_magnitude( QUAT q )
{
  return(sqrt( q.q0 * q.q0 + q.q1 * q.q1 + q.q2 * q.q2 + q.q3 * q.q3 ));
}

QUAT
quaternions_multiply( QUAT q1, QUAT q2 )
{
  double mag;
  QUAT q;
  q.q0 =  q1.q0 * q2.q0 - q1.q1 * q2.q1 - q1.q2 * q2.q2 - q1.q3 * q2.q3;
  q.q1 =  q1.q0 * q2.q1 + q1.q1 * q2.q0 + q1.q2 * q2.q3 - q1.q3 * q2.q2;
  q.q2 =  q1.q0 * q2.q2 - q1.q1 * q2.q3 + q1.q2 * q2.q0 + q1.q3 * q2.q1;
  q.q3 =  q1.q0 * q2.q3 + q1.q1 * q2.q2 - q1.q2 * q2.q1 + q1.q3 * q2.q0;
  mag = quaternions_magnitude( q );
  q.q0 /= mag; q.q1 /= mag; q.q2 /= mag; q.q3 /= mag;
  return(q);
}

