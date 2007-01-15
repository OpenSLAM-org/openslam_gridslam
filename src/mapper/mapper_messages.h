#ifndef CMRT_MAP2D_MESSAGES_H
#define CMRT_MAP2D_MESSAGES_H

#ifdef __cplusplus
extern "C" {
#endif

#define REGISTRATION_MODE     0
#define MAPPING_MODE          1

typedef struct {
  int mode;
  double timestamp;
  char host[10];
} navigation_mapper_set_mode_message;

#define NAVIGATION_MAPPER_SET_MODE_NAME      "navigation_mapper_set_mode"

#define NAVIGATION_MAPPER_SET_MODE_FMT       "{ int, double, [char:10] }"

 
typedef struct {
  double  robot_x;
  double  robot_y;
  double  robot_theta;
  double  corr_x;
  double  corr_y;
  double  corr_theta;
  double timestamp;
  char host[10];
} navigation_mapper_status_message;

#define NAVIGATION_MAPPER_STATUS_NAME        "navigation_mapper_status"

#define NAVIGATION_MAPPER_STATUS_FMT         "{ double, double, double, double, double, double, double, [char:10] }"


#ifdef __cplusplus
}
#endif

#endif








