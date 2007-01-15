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
#include <fnmatch.h>

#include <navigation/utils.h>

#include "defines.h"

int quit_if_unknown = 0;

enum FILE_TYPE          { SCRIPT, REC };

#define MAX_NAME_LENGTH    256

int
read_data2d_file( REC2_DATA * rec, char * filename )
{
  enum FILE_TYPE     inp_type = REC;
  char               fname[MAX_NAME_LENGTH];

  if ( !fnmatch( "script:*", filename, 0) ) {
    fprintf( stderr, "* INFO: use script-file-type!\n" );
    strncpy( fname, &(filename[7]), MAX_NAME_LENGTH );
    inp_type = SCRIPT;
  } else if ( !fnmatch( "rec:*", filename, 0) ) {
    fprintf( stderr, "* INFO: read rec-file-type!\n" );
    strncpy( fname, &(filename[4]), MAX_NAME_LENGTH );
    inp_type = REC;
  } else if ( !fnmatch( "carmen:*", filename, 0) ) {
    fprintf( stderr, "* INFO: read carmen-file-type!\n" );
    strncpy( fname, &(filename[7]), MAX_NAME_LENGTH );
    inp_type = REC;
  } else if ( !fnmatch( "*.script", filename, 0) ) {
    fprintf( stderr, "* INFO: use script-file-type!\n" );
    strncpy( fname, filename, MAX_NAME_LENGTH );
    inp_type = SCRIPT;
  } else if ( !fnmatch( "*.rec", filename, 0) ) {
    fprintf( stderr, "* INFO: read rec-file-type!\n" );
    strncpy( fname, filename, MAX_NAME_LENGTH );
    inp_type = REC;
  } else if ( !fnmatch( "*.log", filename, 0) ) {
    fprintf( stderr, "* INFO: read carmen-file-type!\n" );
    strncpy( fname, filename, MAX_NAME_LENGTH );
    inp_type = REC;
  }

  switch (inp_type) {
  case SCRIPT:
    if (read_script( fname, rec, 1 ) !=0 )
      return(FALSE);
    break;
  case REC:
    if (read_rec2d_file( fname, rec, READ_MODE_DONT_STOP ) !=0 )
      return(FALSE);
    break;
  default:
    fprintf( stderr, "ERROR: unknown file-type!" );
    return(FALSE);
  }

  return(TRUE);
}

int
read_rec2d_file( char *filename, REC2_DATA *rec, int mode )
{

  char      line[MAX_LINE_LENGTH];
  int       FEnd, numPos, numCorrPos, numScan, numDynProb;
  char      command[MAX_CMD_LENGTH];
  FILE    * iop;
  int       linectr = 0, corrposctr = 0, posctr = 0, poscorrctr = 0;
  int       laserctr = 0, dynctr = 0, gpsctr = 0, compassctr = 0;
  int       numEntries = 0, markposctr = 0, rfidctr = 0, pos3dctr = 0;
  int       nmeaggactr = 0;

  fprintf( stderr, "reading file %s ...\n", filename );
  if ((iop = fopen( filename, "r")) == 0){
    fprintf(stderr, " WARNING no rec file %s\n", filename );
    return(-1);
  }

  rec->info.numdynamicprobs = 0;

  FEnd = 0;
  do{
    linectr++;
    if (fscanf(iop, "%s", command) == EOF)
      FEnd=1;
    else{
      if (!strcmp( command, "POS")) {
	posctr++;
      }	else if (!strcmp( command, "ODOM")) {
	posctr++;
      }	else if (!strcmp( command, "CORR_POS")) {
	corrposctr++;
      }	else if (!strcmp( command, "POS-CORR")) {
	poscorrctr++;
      }	else if (!strcmp( command, "LASER") ){
	laserctr++;
      }	else if (!strcmp( command, "LASER-RANGE") ){
	laserctr++;
      }	else if (!strcmp( command, "CARMEN-LASER") ){
	laserctr++;
      }	else if (!strcmp( command, "FLASER") ){
	laserctr++;
      }	else if (!strcmp( command, "HELI-POS") ){
	pos3dctr++;
      }	else if (!strcmp( command, "RLASER") ){
	laserctr++;
      }	else if (!strcmp( command, "DYNAMIC-PROB") ){
	dynctr++;
      } else if (!strcmp( command, "GPS")) {
	gpsctr++;
      } else if (!strcmp( command, "NMEA-GGA")) {
	nmeaggactr++;
      } else if (!strcmp( command, "COMPASS2D")) {
	compassctr++;
      } else if (!strcmp( command, "COMPASS3D")) {
	compassctr++;
      } else if (!strcmp( command, "MARK-POS")) {
	markposctr++;
      } else if (!strcmp( command, "RFID")) {
	rfidctr++;
      }
      fgets(command,sizeof(command),iop);
    }
  } while (!FEnd);
  
  if (mode && READ_MODE_VERBOSE) {
    fprintf( stderr, "*******************************\n" );
    //    fprintf( stderr, "num lines         = %d\n", linectr );
    fprintf( stderr, "num positions     = %d\n", posctr );
    fprintf( stderr, "num positions3d   = %d\n", pos3dctr );
    fprintf( stderr, "num pos corr      = %d\n", poscorrctr );
    fprintf( stderr, "num corr pos      = %d\n", corrposctr );
    fprintf( stderr, "num laserscans    = %d\n", laserctr );
    fprintf( stderr, "num dynamic probs = %d\n", dynctr );
    fprintf( stderr, "num compass pos   = %d\n", compassctr );
    fprintf( stderr, "num gps pos       = %d\n", gpsctr );
    fprintf( stderr, "num nmea gga      = %d\n", nmeaggactr );
    fprintf( stderr, "num mark pos      = %d\n", markposctr );
    fprintf( stderr, "num rfid          = %d\n", rfidctr );
    fprintf( stderr, "*******************************\n" );
  }
  
  numEntries =
    posctr + poscorrctr + corrposctr + laserctr + dynctr +
    gpsctr + compassctr + markposctr + rfidctr + pos3dctr +
    nmeaggactr;

  rec->numentries = 0;
  
  rec->info.numdynamicprobs = dynctr;

  rec->entry   =
    (ENTRY_POSITION *) malloc( numEntries * sizeof(ENTRY_POSITION) );

  rewind(iop);

  if (posctr>0) {
    rec->psens =
      (POSSENS2_DATA *) malloc( posctr * sizeof(POSSENS2_DATA) );
  } else
    rec->psens = NULL;

  if (pos3dctr>0) {
    rec->psens3d =
      (POSSENS3D_DATA *) malloc( pos3dctr * sizeof(POSSENS3D_DATA) );
  } else
    rec->psens3d = NULL;

  if (corrposctr>0)
    rec->cpsens =
      (POSSENS2_DATA *) malloc( corrposctr * sizeof(POSSENS2_DATA) ); 
  else
    rec->cpsens = NULL;

  if (poscorrctr>0)
    rec->poscorr =
      (POS_CORR2_DATA *) malloc( poscorrctr * sizeof(POS_CORR2_DATA) ); 
  else
    rec->poscorr = NULL;

  if (laserctr>0)
    rec->lsens =
      (LASERSENS2_DATA *) malloc( laserctr * sizeof(LASERSENS2_DATA) ); 
  else
    rec->lsens = NULL;

  if (gpsctr>0)
    rec->gps =
      (GPS_DATA *) malloc( gpsctr * sizeof(GPS_DATA) );
  else
    rec->gps = NULL;

  if (nmeaggactr>0)
    rec->nmea_gga =
      (NMEA_GGA_DATA *) malloc( nmeaggactr * sizeof(NMEA_GGA_DATA) );
  else
    rec->nmea_gga = NULL;

  if (compassctr>0)
    rec->compass =
      (COMPASS3_DATA *) malloc( compassctr * sizeof(COMPASS3_DATA) );
  else 
    rec->compass = NULL;

  if (markposctr>0)
    rec->marking =
      (MARK_POS_DATA *) malloc( markposctr * sizeof(MARK_POS_DATA) );
  else 
    rec->marking = NULL;
  
  if (rfidctr>0)
    rec->rfid =
      (RFID_DATA *) malloc( rfidctr * sizeof(RFID_DATA) );
  else 
    rec->rfid = NULL;
  
  numScan    = 0;
  numPos     = 0;
  numCorrPos = 0;
  numDynProb = 0;

  rec->numpositions    = 0;
  rec->numpositions3d  = 0;
  rec->numposcorr      = 0;
  rec->numcpositions   = 0;
  rec->numlaserscans   = 0;
  rec->numgps          = 0;
  rec->numnmea_gga     = 0;
  rec->numcompass      = 0;
  rec->nummarkings     = 0;
  rec->numrfid         = 0;
  
  FEnd = 0;
  do{
    if (fgets(line,MAX_LINE_LENGTH,iop) == NULL) {
      break;
    }
  } while (rec2_parse_line( line, rec, TRUE, TRUE ));
    
  if (mode && READ_MODE_VERBOSE) {
    fprintf( stderr, "*******************************\n" );
    fprintf( stderr, "num positions     = %d\n",
	     rec->numpositions );
    fprintf( stderr, "num laserscans    = %d\n",
	     rec->numlaserscans );
    fprintf( stderr, "*******************************\n" );
  }

  fclose(iop);
  return(0);
}


int
write_rec2d_file( char *filename, REC2_DATA rec )
{
  FILE  * iop;
  int     i, k, tag1, tag2, tag3, tag4;
  
  fprintf( stderr, "writing file %s ...\n", filename );
  if ((iop = fopen( filename, "w")) == 0){
    fprintf(stderr, " WARNING: can't write rec file %s\n", filename );
    return(-1);
  }

  for (i=0; i<rec.numentries; i++) {
    switch( rec.entry[i].type ) {
    case LASER_VALUES:
      fprintf(iop, "LASER-RANGE %11ld %7ld %d %d %.1f:",
	      rec.lsens[rec.entry[i].index].laser.time.tv_sec,
	      rec.lsens[rec.entry[i].index].laser.time.tv_usec,
	      rec.lsens[rec.entry[i].index].id,
	      rec.lsens[rec.entry[i].index].laser.numvalues,
	      rad2deg(rec.lsens[rec.entry[i].index].laser.anglerange) );
      for (k=0;k<rec.lsens[rec.entry[i].index].laser.numvalues;k++) {
	fprintf(iop, " %.1f", rec.lsens[rec.entry[i].index].laser.val[k] );
      }
      fprintf(iop, "\n" );
      break;
    case POSITION:
      fprintf( iop, "POS %11ld %7ld: %.5f %.5f %.5f %.5f %.5f\n",
	       rec.psens[rec.entry[i].index].time.tv_sec,
	       rec.psens[rec.entry[i].index].time.tv_usec,
	       rec.psens[rec.entry[i].index].rpos.x,
	       rec.psens[rec.entry[i].index].rpos.y,
	       rad2deg(rec.psens[rec.entry[i].index].rpos.o),
	       rec.psens[rec.entry[i].index].rvel.tv,
	       rec.psens[rec.entry[i].index].rvel.rv );
      break;
    case CORR_POSITION:
      fprintf( iop, "CORR-POS %11ld %7ld: %.5f %.5f %.5f %.5f %.5f\n",
	       rec.cpsens[rec.entry[i].index].time.tv_sec,
	       rec.cpsens[rec.entry[i].index].time.tv_usec,
	       rec.cpsens[rec.entry[i].index].rpos.x,
	       rec.cpsens[rec.entry[i].index].rpos.y,
	       rad2deg(rec.cpsens[rec.entry[i].index].rpos.o),
	       rec.cpsens[rec.entry[i].index].rvel.tv,
	       rec.cpsens[rec.entry[i].index].rvel.rv );
      break;
    case GPS:
      fprintf( iop, "GPS %11ld %7ld: %f %f %f %f %f %f %d\n",
	       rec.gps[rec.entry[i].index].time.tv_sec,
	       rec.gps[rec.entry[i].index].time.tv_usec,
	       rec.gps[rec.entry[i].index].x,
	       rec.gps[rec.entry[i].index].y,
	       rad2deg(rec.gps[rec.entry[i].index].o),
	       rec.gps[rec.entry[i].index].xvar,
	       rec.gps[rec.entry[i].index].yvar,
	       rec.gps[rec.entry[i].index].ovar,
	       rec.gps[rec.entry[i].index].numsats );
      break;
    case COMPASS:
      fprintf( iop, "COMPASS3D %11ld %7ld: %.3f %.3f %.3f\n",
	       rec.compass[rec.entry[i].index].time.tv_sec,
	       rec.compass[rec.entry[i].index].time.tv_usec,
	       rad2deg(rec.compass[rec.entry[i].index].rx),
	       rad2deg(rec.compass[rec.entry[i].index].ry),
	       rad2deg(rec.compass[rec.entry[i].index].rz) );
      break;
    case POS_CORR:
      fprintf( iop, "POS-CORR %11ld %7ld: %.3f %.3f %.3f\n",
	       rec.poscorr[rec.entry[i].index].time.tv_sec,
	       rec.poscorr[rec.entry[i].index].time.tv_usec,
	       rec.poscorr[rec.entry[i].index].x,
	       rec.poscorr[rec.entry[i].index].y,
	       rad2deg(rec.poscorr[rec.entry[i].index].o) );
      break;
    case MARK_POS:
      fprintf( iop, "MARK-POS %11ld %7ld: %.3f %.3f %.3f\n",
	       rec.marking[rec.entry[i].index].time.tv_sec,
	       rec.marking[rec.entry[i].index].time.tv_usec,
	       rec.marking[rec.entry[i].index].pos.x,
	       rec.marking[rec.entry[i].index].pos.y,
	       rad2deg(rec.marking[rec.entry[i].index].pos.o) );
      break;
    case RFID_TAG:
      tag1 = (int) ((rec.rfid[rec.entry[i].index].tag >> 48));
      tag2 = (int) ((rec.rfid[rec.entry[i].index].tag >> 32) & 0xffff );
      tag3 = (int) ((rec.rfid[rec.entry[i].index].tag >> 16) & 0xffff );
      tag4 = (int) ((rec.rfid[rec.entry[i].index].tag) & 0xffff );
      fprintf( iop,
	       "RFID %11ld %7ld: %s%s%s%x %s%s%s%x %s%s%s%x %s%s%s%x %d %d\n",
	       rec.rfid[rec.entry[i].index].time.tv_sec,
	       rec.rfid[rec.entry[i].index].time.tv_usec,
	       tag1<0xfff?"0":"", tag1<0xff?"0":"", tag1<0xf?"0":"", tag1,
	       tag2<0xfff?"0":"", tag2<0xff?"0":"", tag2<0xf?"0":"", tag2,
	       tag3<0xfff?"0":"", tag3<0xff?"0":"", tag3<0xf?"0":"", tag3,
	       tag4<0xfff?"0":"", tag4<0xff?"0":"", tag4<0xf?"0":"", tag4,
	       rec.rfid[rec.entry[i].index].antenna,
	       rec.rfid[rec.entry[i].index].count );
      break;
    case UNKNOWN:
      break;
    default:
      break;
    }
  }
  return(0);
}
