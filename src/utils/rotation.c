#include <time.h>
#include <math.h>
#include <signal.h>
#include <unistd.h>
#include <string.h>
#include <ctype.h>
#include <values.h>

#include <navigation/utils.h>

typedef double ROT_MATRIX[3][3];

void
rot_rotation_rx( ROT_MATRIX * matrix, double a ) {

  *(matrix[0][0])=1.0;     *(matrix[1][0])=0.0;     *(matrix[2][0])=0.0;
  *(matrix[0][1])=0.0;     *(matrix[1][1])=cos(a);  *(matrix[2][1])=sin(a);
  *(matrix[0][2])=0.0;     *(matrix[1][2])=-sin(a); *(matrix[2][2])=cos(a);

}

void
rot_rotation_ry( ROT_MATRIX * matrix, double a ) {

  *(matrix[0][0])=cos(a);  *(matrix[1][0])=0.0;     *(matrix[2][0])=-sin(a);
  *(matrix[0][1])=0.0;     *(matrix[1][1])=1.0;     *(matrix[2][1])=0.0;
  *(matrix[0][2])=sin(a);  *(matrix[1][2])=0.0;     *(matrix[2][2])=cos(a);

}

void
rot_rotation_rz( ROT_MATRIX * matrix, double a ) {

  *(matrix[0][0])=cos(a);  *(matrix[1][0])=sin(a);  *(matrix[2][0])=0.0;
  *(matrix[0][1])=-sin(a); *(matrix[1][1])=cos(a);  *(matrix[2][1])=0.0;
  *(matrix[0][2])=0.0;     *(matrix[1][2])=0.0;     *(matrix[2][2])=1.0;
  
}

void
rot_mult_matrices( ROT_MATRIX mat1, ROT_MATRIX mat2, ROT_MATRIX * matrix ) {
  
  *(matrix[0][0]) =
    mat1[0][0] * mat2[0][0] +
    mat1[1][0] * mat2[0][1] +
    mat1[2][0] * mat2[0][2];
  *(matrix[0][1]) =
    mat1[0][1] * mat2[0][0] +
    mat1[1][1] * mat2[0][1] +
    mat1[2][1] * mat2[0][2];
  *(matrix[0][2]) =
    mat1[0][2] * mat2[0][0] +
    mat1[1][2] * mat2[0][1] +
    mat1[2][2] * mat2[0][2];
  
  *(matrix[1][0]) =
    mat1[0][0] * mat2[1][0] +
    mat1[1][0] * mat2[1][1] +
    mat1[2][0] * mat2[1][2];
  *(matrix[1][1]) =
    mat1[0][1] * mat2[1][0] +
    mat1[1][1] * mat2[1][1] +
    mat1[2][1] * mat2[1][2];
  *(matrix[1][2]) =
    mat1[0][2] * mat2[1][0] +
    mat1[1][2] * mat2[1][1] +
    mat1[2][2] * mat2[1][2];
  
  *(matrix[2][0])=
    mat1[0][0] * mat2[2][0] +
    mat1[1][0] * mat2[2][1] +
    mat1[2][0] * mat2[2][2];
  *(matrix[2][1]) =
    mat1[0][1] * mat2[2][0] +
    mat1[1][1] * mat2[2][1] +
    mat1[2][1] * mat2[2][2];
  *(matrix[2][2]) =
    mat1[0][2] * mat2[2][0] +
    mat1[1][2] * mat2[2][1] +
    mat1[2][2] * mat2[2][2];
}

VECTOR3
rot_rotate_vec3( ROT_MATRIX matrix, VECTOR3 v ) {

  VECTOR3 r;
  
  r.x = matrix[0][0] * v.x + matrix[1][0] * v.y + matrix[2][0] * v.z;
  r.y = matrix[0][1] * v.x + matrix[1][1] * v.y + matrix[2][1] * v.z;
  r.z = matrix[0][2] * v.x + matrix[1][2] * v.y + matrix[2][2] * v.z;
  return(r);
  
}
