#include <carmen/carmen.h>
#include <carmen/base_messages.h>
#include <carmen/laser_messages.h>

#include "map2d.h"

extern logtools_rpos2_t          current_pos;
extern int                       change_map;

void convert_time( double tval, struct timeval *time );

static void 
ipc_odometry_handler(MSG_INSTANCE msgRef, BYTE_ARRAY callData,
		     void *clientData __attribute__ ((unused)))
{
  static carmen_base_odometry_message   data;
  IPC_RETURN_TYPE                       err = IPC_OK;
  FORMATTER_PTR                         formatter;
  
  formatter = IPC_msgInstanceFormatter(msgRef);
  err = IPC_unmarshallData(formatter, callData, &data,
			   sizeof(carmen_base_odometry_message));
  IPC_freeByteArray(callData);

  carmen_test_ipc_return(err, "Could not unmarshall data", 
			 IPC_msgInstanceName(msgRef));
  current_pos.x = data.x * 100.0;
  current_pos.y = data.y * 100.0;
  current_pos.o = data.theta;

  online_odo_update = TRUE;
}

double
apex_angle( int num )
{
  switch( num ) {
  case(180):
    return(180.0);
    break;
  case(181):
    return(180.0);
    break;
  case(360):
    return(180.0);
    break;
  case(361):
    return(180.0);
    break;
  case(401):
    return(100.0);
    break;
  default:
    return(180.0);
    break;
  }
}

static void 
ipc_carmen_front_laser_handler( MSG_INSTANCE msgRef, BYTE_ARRAY callData,
				void *clientData __attribute__ ((unused)))
{
  IPC_RETURN_TYPE                      err = IPC_OK;
  FORMATTER_PTR                        formatter;
  static int                           allocated = FALSE;
  static carmen_laser_laser_message    data;
  int                                  i, r;
  double                               anglediff, rangestart, apxangle;

  if (!allocated) {
    data.range= (float *) malloc( MAX_NUM_LASER_VALUES * sizeof(float));
    allocated = TRUE;
  }
  
  formatter = IPC_msgInstanceFormatter(msgRef);
  err = IPC_unmarshallData(formatter, callData, &data,
			   sizeof(carmen_laser_laser_message));
  IPC_freeByteArray(callData);
  carmen_test_ipc_return(err, "Could not unmarshall", 
			 IPC_msgInstanceName(msgRef));


  apxangle     =  deg2rad(apex_angle( data.num_readings ));
  anglediff    =  (apxangle / (double) (data.num_readings-1));
  rangestart   = -(apxangle / 2.0);
  r            =  online_scan_ctr%settings.local_map_num_history;
  online_data.lsens[r].laser.numvalues = data.num_readings;
  for (i = 0; i < data.num_readings; i++){
    online_data.lsens[r].laser.val[i]      = data.range[i]*100.0;
    online_data.lsens[r].laser.angle[i]    = rangestart+(i*anglediff);
  } 
  convert_time( data.timestamp, &online_data.lsens[r].laser.time );
  online_data.lsens[r].id               = settings.laser_number;
  online_data.lsens[r].laser.fov        = apxangle;
  online_data.lsens[r].estpos           = current_pos;
  online_laserupdate = TRUE;

}


void
ipc_initialize_messages( void )
{
  IPC_RETURN_TYPE err = IPC_OK;
  
  err = IPC_subscribe(CARMEN_BASE_ODOMETRY_NAME, ipc_odometry_handler, NULL);
  IPC_setMsgQueueLength(CARMEN_BASE_ODOMETRY_NAME, 1);
  carmen_test_ipc(err, "Could not subscribe", CARMEN_BASE_ODOMETRY_NAME);

  err = IPC_subscribe(CARMEN_LASER_FRONTLASER_NAME,
		      ipc_carmen_front_laser_handler, NULL);
  IPC_setMsgQueueLength(CARMEN_LASER_FRONTLASER_NAME, 1);
  carmen_test_ipc(err, "Could not subscribe", CARMEN_LASER_FRONTLASER_NAME);
  
}

void
ipc_update( void )
{
  IPC_listen(0);
}

void
ipc_init( int argc, char *argv[] )
{
  carmen_ipc_initialize( argc, argv );
  ipc_initialize_messages();
}

void
ipc_stop( void )
{
  fprintf( stderr, "INFO: close connection to CENTRAL\n" );
  IPC_disconnect();
}

