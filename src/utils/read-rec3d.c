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
compAngleDiff( double angleRange, int nVal )
{
  int val;
  if (nVal == 180 ) {
    val = nVal+1;
  } else {
    val = nVal;
  }
  return( angleRange / (double) (nVal-1) );
}

int
read_rec3d_file( char *filename, REC3_DATA *rec, int verbose )
{

  FILE  * iop;

  double  astart, angleDiff;
  float   fRange;
  
  int     i;
  char    line[MAX_LINE_LENGTH];
  char    command[MAX_CMD_LENGTH];

  long    sec, usec;
  int     nLas, nVal;
  char    strF1[MAX_NUM_LENGTH],strF2[MAX_NUM_LENGTH],strF3[MAX_NUM_LENGTH];
  char    strF4[MAX_NUM_LENGTH],strF5[MAX_NUM_LENGTH],strF6[MAX_NUM_LENGTH];
  char    strF7[MAX_NUM_LENGTH];

  int     laser_using_degrees = TRUE;
  int     pos_using_degrees   = TRUE;
  int     cnt        = 0;
  int     scan       = TRUE;
  int     FEnd       = 0;
  int     lineCtr    = 0;
  int     posCtr     = 0;
  int     corrPosCtr = 0;
  int     laserCtr   = 0;
  int     laserACtr  = 0;
  int     amtecCtr   = 0;
  int     amtecPosCtr= 0;
  int     scanCtr    = 0;
  int     sweepCtr   = 0;
  int     scanSwCtr  = 0;
  int     gpsCtr     = 0;
  int     compassCtr = 0;
  int     numEntries = 0;
  int     numScansInSweep[MAX_NUM_SWEEPS];

  fprintf( stderr, "reading file %s ...\n", filename );
  if ((iop = fopen( filename, "r")) == 0){
    fprintf(stderr, " WARNING no rec file %s\n", filename );
    return(-1);
  }

  rec->lsens.settings.pantilt.axisdist   = DEFAULT_PANTILT_AXIS_DIST;
  rec->lsens.settings.pantilt.displace.forward  = DEFAULT_PANTILT_AXIS_FORWARD;
  rec->lsens.settings.pantilt.displace.sideward = DEFAULT_PANTILT_AXIS_SIDEWARD;
  rec->lsens.settings.pantilt.displace.upward   = DEFAULT_PANTILT_AXIS_UPWARD;
  
  for (i=0;i<MAX_NUM_SWEEPS;i++) {
    numScansInSweep[i] = 0;
  }
  sweepCtr  = 0;
  
  do{
    if (fgets(line,MAX_LINE_LENGTH,iop) == NULL) {
      FEnd=1;
    } else {
      lineCtr++;
      sscanf(line, "%s", command);
      if (!strcmp( command, "POS")) {
	posCtr++;
      } else if (!strcmp( command, "POS3D")) {
	posCtr++;
      } else if (!strcmp( command, "HELI-IMU")) {
	posCtr++;
      } else if (!strcmp( command, "GPS")) {
	gpsCtr++;
      } else if (!strcmp( command, "COMPASS3D")) {
	compassCtr++;
      } else if (!strcmp( command, "COMPASS2D")) {
	compassCtr++;
      } else if (!strcmp( command, "CORR-POS")) {
	corrPosCtr++;
      } else if (!strcmp( command, "CORR-POS3D")) {
	corrPosCtr++;
      } else if (!strcmp( command, "AMTEC-PANTILT")) {
	amtecPosCtr++;
      } else if (!strcmp( command, "SWIVEL-START") ){
	if (scan && sweepCtr<MAX_NUM_SWEEPS) {
	  numScansInSweep[sweepCtr] = scanSwCtr;
	  sweepCtr++;
	}
	scanSwCtr = 0;
	scan = TRUE;
      } else if (!strcmp( command, "SWIVEL-END") ){
	if (scan && sweepCtr<MAX_NUM_SWEEPS) {
	  numScansInSweep[sweepCtr] = scanSwCtr;
	  sweepCtr++;
	}
	scanSwCtr = 0;
	scan =  FALSE;
      } else if (!strcmp( command, "LASER-AMTEC")) {
	laserACtr++;
	if (scan)
	  scanSwCtr++;
      } else if (!strcmp( command, "LASER")) {
	laserCtr++;
      }
    }
  } while (!FEnd);

  if (scan && sweepCtr<MAX_NUM_SWEEPS) {
    numScansInSweep[sweepCtr] = scanSwCtr;
    sweepCtr++;
    scan = 0;
  }

  amtecCtr = 0;
  for (i=0;i<sweepCtr;i++) {
    amtecCtr += numScansInSweep[i];
  }

  if (verbose) {
    fprintf( stderr, "*******************************\n" );
    fprintf( stderr, "num  lines         = %d\n", lineCtr );
    fprintf( stderr, "num  pos   - scans = %d\n", posCtr );
    fprintf( stderr, "num  laser - scans = %d\n", laserCtr );
    fprintf( stderr, "num  amtec - lines = %d\n", laserACtr );
    fprintf( stderr, "used amtec - lines = %d\n", amtecCtr );
    fprintf( stderr, "num  amtec - scans = %d\n", sweepCtr );
    fprintf( stderr, "num  amtec - pos   = %d\n", amtecPosCtr );
    fprintf( stderr, "num  compass pos   = %d\n", compassCtr );
    fprintf( stderr, "num  gps pos       = %d\n", gpsCtr );
    fprintf( stderr, "*******************************\n" );
  }
 
  rewind(iop);
  
  numEntries =
    posCtr + laserCtr + laserACtr + amtecCtr + gpsCtr + compassCtr +
    amtecPosCtr + corrPosCtr;
  
  rec->entry   =
    (ENTRY_POSITION *) malloc( numEntries * sizeof(ENTRY_POSITION) );
  rec->psens =
    (POSSENS3_DATA *) malloc( posCtr * sizeof(POSSENS3_DATA) ); 
  rec->cpsens =
    (POSSENS3_DATA *) malloc( corrPosCtr * sizeof(POSSENS3_DATA) ); 

  rec->amtec =
    (AMTEC_POSITION *) malloc( amtecPosCtr * sizeof(AMTEC_POSITION));

  rec->lsens.scan =
    (LASERSENS2_DATA *) malloc( laserCtr * sizeof(LASERSENS2_DATA) ); 
  rec->lsens.sweep =
    (LASERSWEEP3_DATA *) malloc( sweepCtr * sizeof(LASERSWEEP3_DATA) ); 

  rec->gps =
    (GPS_DATA *) malloc( gpsCtr * sizeof(GPS_DATA) ); 

  rec->compass =
    (COMPASS3_DATA *) malloc( compassCtr * sizeof(COMPASS3_DATA) ); 

  rec->numentries = numEntries;

  for (i=0;i<sweepCtr;i++) {
    rec->lsens.sweep[i].swscan =
      (LASERSENS3_DATA *) malloc( numScansInSweep[i] *
				  sizeof(LASERSENS3_DATA) ); 
  }

  FEnd = 0;
  
  posCtr       = 0;
  amtecPosCtr  = 0;
  corrPosCtr   = 0;

  laserCtr     = 0;
  sweepCtr     = 0;
  scanSwCtr    = 0;
  compassCtr   = 0;
  gpsCtr       = 0;
  
  scan         = TRUE;
  
  do{
    
    if (fscanf(iop, "%s", command) == EOF)
      FEnd=1;
    else{
      if (!strcmp( command, "LASER-AMTEC") ){
	if (fscanf(iop, "%ld %ld %d %d %f %s %s:",
		   &sec, &usec, &nLas, &nVal, &fRange, strF2, strF3 ) == EOF) {
	  FEnd=1;
	} else {
	  if (scan && sweepCtr<MAX_NUM_SWEEPS) {
	    /* nVal = 361; */
	    /* fRange = 180.0; */
	    rec->entry[cnt].type   = LASER_VALUES;
	    rec->entry[cnt].idx1   = sweepCtr;
	    rec->entry[cnt].idx2   = scanSwCtr;
	    rec->lsens.sweep[sweepCtr].swscan[scanSwCtr].laser.time.tv_sec  = sec;
	    rec->lsens.sweep[sweepCtr].swscan[scanSwCtr].laser.time.tv_usec = usec;
	    rec->lsens.sweep[sweepCtr].swscan[scanSwCtr].laser.numvalues    = nVal;
	    rec->lsens.sweep[sweepCtr].swscan[scanSwCtr].id                 = nLas;
	    rec->lsens.sweep[sweepCtr].swscan[scanSwCtr].laser.anglerange   = deg2rad(fRange);
	    if (laser_using_degrees) {
	      rec->lsens.sweep[sweepCtr].swscan[scanSwCtr].declination.pan    = -deg2rad(atof(strF2));
	      rec->lsens.sweep[sweepCtr].swscan[scanSwCtr].declination.tilt   = deg2rad(atof(strF3));
	    } else {
	      rec->lsens.sweep[sweepCtr].swscan[scanSwCtr].declination.pan    = atof(strF2);
	      rec->lsens.sweep[sweepCtr].swscan[scanSwCtr].declination.tilt   = atof(strF3);
	    }
	    
	    rec->lsens.sweep[sweepCtr].swscan[scanSwCtr].laseroffset.x = 0.0;
	    rec->lsens.sweep[sweepCtr].swscan[scanSwCtr].laseroffset.y =
	      rec->lsens.settings.pantilt.axisdist *
	      sin(rec->lsens.sweep[sweepCtr].swscan[scanSwCtr].declination.pan);
	    rec->lsens.sweep[sweepCtr].swscan[scanSwCtr].laseroffset.z =
	      rec->lsens.settings.pantilt.displace.upward +
	      rec->lsens.settings.pantilt.axisdist *
	      cos(fabs(rec->lsens.sweep[sweepCtr].swscan[scanSwCtr].declination.tilt));
	    
	    rec->lsens.sweep[sweepCtr].swscan[scanSwCtr].laseroffset.x = 0.0;
	    rec->lsens.sweep[sweepCtr].swscan[scanSwCtr].laseroffset.y = 0.0;
	    rec->lsens.sweep[sweepCtr].swscan[scanSwCtr].laseroffset.z = 0.0;
	    
	    rec->lsens.sweep[sweepCtr].swscan[scanSwCtr].laser.val =
	      (double *) malloc( nVal * sizeof(double) );
	    rec->lsens.sweep[sweepCtr].swscan[scanSwCtr].laser.maxrange =
	      (int *) malloc( nVal * sizeof(int) );
	    rec->lsens.sweep[sweepCtr].swscan[scanSwCtr].laser.angle =
	      (double *) malloc( nVal * sizeof(double) );
	    angleDiff = compAngleDiff(rec->lsens.sweep[sweepCtr].swscan[scanSwCtr].laser.anglerange, nVal);
	    astart = -(rec->lsens.sweep[sweepCtr].swscan[scanSwCtr].laser.anglerange/2.0);
	    if (posCtr>0) {
	      rec->lsens.sweep[sweepCtr].swscan[scanSwCtr].estpos =
		rec->psens[posCtr-1].rpos;
	    } else {
	      rec->lsens.sweep[sweepCtr].swscan[scanSwCtr].estpos.pos.x = 0.0;
	      rec->lsens.sweep[sweepCtr].swscan[scanSwCtr].estpos.pos.y = 0.0;
	      rec->lsens.sweep[sweepCtr].swscan[scanSwCtr].estpos.pos.z = 0.0;
	      rec->lsens.sweep[sweepCtr].swscan[scanSwCtr].estpos.rot.x = 0.0;
	      rec->lsens.sweep[sweepCtr].swscan[scanSwCtr].estpos.rot.y = 0.0;
	      rec->lsens.sweep[sweepCtr].swscan[scanSwCtr].estpos.rot.z = 0.0;
	    }
	    for (i=0;i<nVal;i++) {
	      if (fscanf(iop, "%s", strF1 ) == EOF) {
		FEnd=1;
		break;
	      } else {
		rec->lsens.sweep[sweepCtr].swscan[scanSwCtr].laser.val[i] = 
		  atof(strF1);
		rec->lsens.sweep[sweepCtr].swscan[scanSwCtr].laser.angle[i] =
		  astart + i*angleDiff;
		rec->lsens.sweep[sweepCtr].swscan[scanSwCtr].laser.maxrange[i] =
		  FALSE;
	      }
	    }
	    scanSwCtr++;
	    cnt++;
	  }
	}
      } else if (!strcmp( command, "LASER-AMTEC-USING-DEGREE") ){
	laser_using_degrees = TRUE;
      } else if (!strcmp( command, "LASER-AMTEC-USING-RADIAN") ){
	laser_using_degrees = FALSE;
      } else if (!strcmp( command, "POS3D-USING-DEGREE") ){
	pos_using_degrees = TRUE;
      } else if (!strcmp( command, "POS3D-USING-RADIAN") ){
	pos_using_degrees = FALSE;
      } else if (!strcmp( command, "AMTEC-AXIS-HEIGHT") ){
	if (fscanf(iop, "%s", strF1 ) == EOF) {
	  FEnd=1;
	} else {
	  rec->lsens.settings.pantilt.displace.upward = atof(strF1);
	}
      } else if (!strcmp( command, "AMTEC-AXIS-DIST") ){
	if (fscanf(iop, "%s", strF1 ) == EOF) {
	  FEnd=1;
	} else {
	  rec->lsens.settings.pantilt.axisdist = atof(strF1);
	}
      } else if (!strcmp( command, "SWIVEL-START") ){
	if (scan && sweepCtr<MAX_NUM_SWEEPS) {
	  sweepCtr++;
	}
	scanSwCtr = 0;
	scan = TRUE;
      } else if (!strcmp( command, "SWIVEL-END") ){
	if (scan && sweepCtr<MAX_NUM_SWEEPS) {
	  rec->lsens.sweep[sweepCtr].numswscans = scanSwCtr-1;
	  sweepCtr++;
	}
	scanSwCtr = 0;
	scan =  FALSE;
      } else if (!strcmp( command, "LASER") ){
	if (fscanf(iop, "%ld %ld %d %d %f:",
		   &sec, &usec, &nLas, &nVal, &fRange ) == EOF) {
	  FEnd=1;
	} else {
	  rec->entry[cnt].type   = LASER_VALUES;
	  rec->entry[cnt].index  = laserCtr;
	  rec->lsens.scan[laserCtr].id                 = nLas;
	  rec->lsens.scan[laserCtr].laser.time.tv_sec  = sec;
	  rec->lsens.scan[laserCtr].laser.time.tv_usec = usec;
	  rec->lsens.scan[laserCtr].laser.numvalues    = nVal;
	  rec->lsens.scan[laserCtr].laser.anglerange   = deg2rad(fRange);
	  rec->lsens.scan[laserCtr].laser.val =
	    (double *) malloc( nVal * sizeof(double) );
	  rec->lsens.scan[laserCtr].laser.maxrange =
	    (int *) malloc( nVal * sizeof(int) );
	  rec->lsens.scan[laserCtr].laser.angle =
	    (double *) malloc( nVal * sizeof(double) );
	  angleDiff = compAngleDiff(rec->lsens.scan[laserCtr].laser.anglerange,nVal);
	  astart = -(rec->lsens.scan[laserCtr].laser.anglerange/2.0);
	  for (i=0;i<nVal;i++) {
	    if (fscanf(iop, "%s", strF1 ) == EOF) {
	      FEnd=1;
	      break;
	    } else {
	      rec->lsens.scan[laserCtr].laser.val[i] = 
		atof(strF1);
	      rec->lsens.scan[laserCtr].laser.angle[i]      = 
		astart + i*angleDiff;
	      rec->lsens.scan[laserCtr].laser.maxrange[i]   = 
		FALSE;
	    }
	  }
	  laserCtr++;
	  cnt++;
	}
      } else if (!strcmp( command, "POS")) {
	if (fscanf( iop, "%ld %ld: %s %s %s %s %s",
		    &sec, &usec, strF1, strF2, strF3, strF4, strF5 ) == EOF) {
	  FEnd=1;
	} else {
	  rec->entry[cnt].type   = POSITION;
	  rec->entry[cnt].index  = posCtr;
	  rec->psens[posCtr].time.tv_sec  = sec;
	  rec->psens[posCtr].time.tv_usec = usec;
	  rec->psens[posCtr].rpos.pos.x   = atof(strF1);
	  rec->psens[posCtr].rpos.pos.y   = atof(strF2);
	  rec->psens[posCtr].rpos.pos.z   = 0.0;
	  rec->psens[posCtr].rpos.rot.x   = 0.0;
	  rec->psens[posCtr].rpos.rot.y   = 0.0;
	  rec->psens[posCtr].rpos.rot.z   = deg2rad(atof(strF3));
	  //	  rec->psens[posCtr].rpos.rz      = deg2rad(atof(strF3)-90.0);
	  posCtr++;
	}
	
      } else if (!strcmp( command, "GPS")) {
	if (fscanf( iop, "%ld %ld: %s %s %s %s %s %s %s",
		    &sec, &usec, strF1, strF2, strF3, strF4,
		    strF5, strF6, strF7 ) == EOF) {
	  FEnd=1;
	} else {
	  rec->entry[cnt].type   = GPS;
	  rec->entry[cnt].index  = gpsCtr;
	  rec->gps[gpsCtr].time.tv_sec  = sec;
	  rec->gps[gpsCtr].time.tv_usec = usec;
	  rec->gps[gpsCtr].x      = atof(strF1);
	  rec->gps[gpsCtr].y      = atof(strF2);
	  rec->gps[gpsCtr].o      = deg2rad(atof(strF3));
	  rec->gps[gpsCtr].xvar   = atof(strF4);
	  rec->gps[gpsCtr].yvar   = atof(strF5);
	  rec->gps[gpsCtr].ovar   = atof(strF6);
	  rec->gps[gpsCtr].numsats= atoi(strF7);
	  gpsCtr++;
	  cnt++;
	}
      } else if (!strcmp( command, "COMPASS2D")) {
	if (fscanf( iop, "%ld %ld: %s",
		    &sec, &usec, strF1 ) == EOF) {
	  FEnd=1;
	} else {
	  rec->entry[cnt].type   = COMPASS;
	  rec->entry[cnt].index  = compassCtr;
	  rec->compass[compassCtr].time.tv_sec  = sec;
	  rec->compass[compassCtr].time.tv_usec = usec;
	  rec->compass[compassCtr].rx   = 0.0;
	  rec->compass[compassCtr].ry   = 0.0;
	  rec->compass[compassCtr].rz   = deg2rad(atof(strF1));
	  compassCtr++;
	  cnt++;
	}
      } else if (!strcmp( command, "COMPASS3D")) {
	if (fscanf( iop, "%ld %ld: %s %s %s",
		    &sec, &usec, strF1, strF2, strF3 ) == EOF) {
	  FEnd=1;
	} else {
	  rec->entry[cnt].type   = COMPASS;
	  rec->entry[cnt].index  = compassCtr;
	  rec->compass[compassCtr].time.tv_sec  = sec;
	  rec->compass[compassCtr].time.tv_usec = usec;
	  rec->compass[compassCtr].rx   = deg2rad(atof(strF1));
	  rec->compass[compassCtr].ry   = deg2rad(atof(strF2));
	  rec->compass[compassCtr].rz   = deg2rad(atof(strF3));
	  compassCtr++;
	  cnt++;
	}
      } else if (!strcmp( command, "POS3D")) {
	if (fscanf( iop, "%ld %ld: %s %s %s %s %s %s",
		    &sec, &usec, strF1, strF2, strF3,
		    strF4, strF5, strF6 ) == EOF) {
	  FEnd=1;
	} else {
	  rec->entry[cnt].type   = POSITION;
	  rec->entry[cnt].index  = posCtr;
	  rec->psens[posCtr].time.tv_sec  = sec;
	  rec->psens[posCtr].time.tv_usec = usec;
	  rec->psens[posCtr].rpos.pos.x   = atof(strF1);
	  rec->psens[posCtr].rpos.pos.y   = atof(strF2);
	  rec->psens[posCtr].rpos.pos.z   = atof(strF3);
	  if (pos_using_degrees) {
	    rec->psens[posCtr].rpos.rot.x   = deg2rad(atof(strF4));
	    rec->psens[posCtr].rpos.rot.y   = deg2rad(atof(strF5));
	    rec->psens[posCtr].rpos.rot.z   = deg2rad(atof(strF6));
	  } else {
	    rec->psens[posCtr].rpos.rot.x   = atof(strF4);
	    rec->psens[posCtr].rpos.rot.y   = atof(strF5);
	    rec->psens[posCtr].rpos.rot.z   = atof(strF6);
	  }
	  posCtr++;
	  cnt++;
	}
      } else if (!strcmp( command, "HELI-IMU")) {
	if (fscanf( iop, "%ld %ld: %s %s %s %s %s %s",
		    &sec, &usec, strF1, strF2, strF3,
		    strF4, strF5, strF6 ) == EOF) {
	  FEnd=1;
	} else {
	  rec->entry[cnt].type   = POSITION;
	  rec->entry[cnt].index  = posCtr;
	  rec->psens[posCtr].time.tv_sec  = sec;
	  rec->psens[posCtr].time.tv_usec = usec;
	  rec->psens[posCtr].rpos.pos.x   = atof(strF1);
	  rec->psens[posCtr].rpos.pos.y   = atof(strF2);
	  rec->psens[posCtr].rpos.pos.z   = atof(strF3);
	  if (pos_using_degrees) {
	    rec->psens[posCtr].rpos.rot.x   = deg2rad(atof(strF4));
	    rec->psens[posCtr].rpos.rot.y   = deg2rad(atof(strF5));
	    rec->psens[posCtr].rpos.rot.z   = deg2rad(atof(strF6));
	  } else {
	    rec->psens[posCtr].rpos.rot.x   = atof(strF4);
	    rec->psens[posCtr].rpos.rot.y   = atof(strF5);
	    rec->psens[posCtr].rpos.rot.z   = atof(strF6);
	  }
	  posCtr++;
	  cnt++;
	}
      } else if (!strcmp( command, "AMTEC-PANTILT")) {
	if (fscanf( iop, "%ld %ld: %s %s",
		    &sec, &usec, strF1, strF2 ) == EOF) {
	  FEnd=1;
	} else {
	  rec->entry[cnt].type   = AMTEC_POS;
	  rec->entry[cnt].index  = amtecPosCtr;
	  rec->amtec[amtecPosCtr].time.tv_sec  = sec;
	  rec->amtec[amtecPosCtr].time.tv_usec = usec;
	  rec->amtec[amtecPosCtr].pos.pan      = deg2rad(atof(strF1));
	  rec->amtec[amtecPosCtr].pos.tilt     = deg2rad(atof(strF2));
	  amtecPosCtr++;
	  cnt++;
	}
      } else if (!strcmp( command, "CORR-POS")) {
	if (fscanf( iop, "%ld %ld: %s %s %s",
		    &sec, &usec, strF1, strF2, strF3 ) == EOF) {
	  FEnd=1;
	} else {
	  rec->entry[cnt].type   = CORR_POSITION;
	  rec->entry[cnt].index  = posCtr;
	  rec->cpsens[posCtr].time.tv_sec  = sec;
	  rec->cpsens[posCtr].time.tv_usec = usec;
	  rec->cpsens[posCtr].rpos.pos.x   = atof(strF1);
	  rec->cpsens[posCtr].rpos.pos.y   = atof(strF2);
	  rec->cpsens[posCtr].rpos.pos.z   = 0.0;
	  rec->cpsens[posCtr].rpos.rot.x   = 0.0;
	  rec->cpsens[posCtr].rpos.rot.y   = 0.0;
	  rec->cpsens[posCtr].rpos.rot.z   = deg2rad(atof(strF3));
	  //	  rec->cpsens[posCtr].rpos.rz      = deg2rad(atof(strF3)-90.0);
	  corrPosCtr++;
	  cnt++;
	}
      } else if (!strcmp( command, "CORR-POS3D")) {
	if (fscanf( iop, "%ld %ld: %s %s %s %s %s %s",
		    &sec, &usec, strF1, strF2, strF3,
		    strF4, strF5, strF6 ) == EOF) {
	  FEnd=1;
	} else {
	  rec->entry[cnt].type   = CORR_POSITION;
	  rec->entry[cnt].index  = posCtr;
	  rec->cpsens[posCtr].time.tv_sec  = sec;
	  rec->cpsens[posCtr].time.tv_usec = usec;
	  rec->cpsens[posCtr].rpos.pos.x   = atof(strF1);
	  rec->cpsens[posCtr].rpos.pos.y   = atof(strF2);
	  rec->cpsens[posCtr].rpos.pos.z   = atof(strF3);
	  rec->cpsens[posCtr].rpos.rot.x   = deg2rad(atof(strF4));
	  rec->cpsens[posCtr].rpos.rot.y   = deg2rad(atof(strF5));
	  rec->cpsens[posCtr].rpos.rot.z   = deg2rad(atof(strF6));
	  corrPosCtr++;
	  cnt++;
	}
      } else if ( command[0] == '#') {
	fgets( line, MAX_LINE_LENGTH, iop );
      }
    }
  } while (!FEnd);

  if (scan && sweepCtr<MAX_NUM_SWEEPS) {
    rec->lsens.sweep[sweepCtr].numswscans = scanSwCtr-1;
    sweepCtr++;
  }

  rec->numpositions      = posCtr;
  rec->numcpositions     = corrPosCtr;
  rec->numentries        = cnt;
  rec->lsens.numscans    = scanCtr;
  rec->lsens.numsweeps   = sweepCtr;

  fclose(iop);

  return(0);
}


int
write_rec3d_file( char *filename, REC3_DATA rec )
{

  FILE  * iop;

  int     i, k;

  fprintf( stderr, "writing file %s ...\n", filename );
  if ((iop = fopen( filename, "w")) == 0){
    fprintf(stderr, " WARNING: can't write rec file %s\n", filename );    return(-1);
  }

  for (i=0; i<rec.numentries; i++) {
    switch( rec.entry[i].type ) {
    case LASER_VALUES3:
      fprintf(iop, "LASER-AMTEC %ld %ld %d %d %.2f %.4f %.4f:",
	      rec.lsens.sweep[rec.entry[i].idx1].swscan[rec.entry[i].idx2].laser.time.tv_sec,
	      rec.lsens.sweep[rec.entry[i].idx1].swscan[rec.entry[i].idx2].laser.time.tv_usec,
	      rec.lsens.sweep[rec.entry[i].idx1].swscan[rec.entry[i].idx2].id,
	      rec.lsens.sweep[rec.entry[i].idx1].swscan[rec.entry[i].idx2].laser.numvalues,
	      rad2deg(rec.lsens.sweep[rec.entry[i].idx1].swscan[rec.entry[i].idx2].laser.anglerange),
	      rad2deg(-rec.lsens.sweep[rec.entry[i].idx1].swscan[rec.entry[i].idx2].declination.pan),
	      rad2deg(rec.lsens.sweep[rec.entry[i].idx1].swscan[rec.entry[i].idx2].declination.tilt) );
      for (k=0;k<rec.lsens.sweep[rec.entry[i].idx1].swscan[rec.entry[i].idx2].laser.numvalues;k++) {
	fprintf(iop, " %.1f", rec.lsens.sweep[rec.entry[i].idx1].swscan[rec.entry[i].idx2].laser.val[k] );
      }
      fprintf(iop, "\n" );
      break;
    case LASER_VALUES:
      fprintf(iop, "LASER %ld %ld %d %d %.2f:",
	      rec.lsens.scan[rec.entry[i].index].laser.time.tv_sec,
	      rec.lsens.scan[rec.entry[i].index].laser.time.tv_usec,
	      rec.lsens.scan[rec.entry[i].index].laser.numvalues,
	      rec.lsens.scan[rec.entry[i].index].id,
	      rad2deg(rec.lsens.scan[rec.entry[i].index].laser.anglerange) );
      for (k=0;k<rec.lsens.scan[rec.entry[i].index].laser.numvalues;k++) {
	fprintf(iop, " %.1f", rec.lsens.scan[rec.entry[i].index].laser.val[k] );
      }
      fprintf(iop, "\n" );
      break;
    case POSITION:
      fprintf( iop, "POS3D %ld %ld: %f %f %f %f %f %f\n",
	       rec.psens[rec.entry[i].index].time.tv_sec,
	       rec.psens[rec.entry[i].index].time.tv_usec,
	       rec.psens[rec.entry[i].index].rpos.pos.x,
	       rec.psens[rec.entry[i].index].rpos.pos.y,
	       rec.psens[rec.entry[i].index].rpos.pos.z,
	       rec.psens[rec.entry[i].index].rpos.rot.x,
	       rec.psens[rec.entry[i].index].rpos.rot.y,
	       rec.psens[rec.entry[i].index].rpos.rot.z );
      break;
    case CORR_POSITION:
      break;
    case HUMAN_PROB:
      break;
    case POS_CORR:
      break;
    case AMTEC_POS:
      fprintf( iop, "AMTEC-PANTILT %ld %ld: %.8f %.8f\n",
	       rec.amtec[rec.entry[i].index].time.tv_sec,
	       rec.amtec[rec.entry[i].index].time.tv_usec,
	       rad2deg(rec.amtec[rec.entry[i].index].pos.pan),
	       rad2deg(rec.amtec[rec.entry[i].index].pos.tilt) );
      break;
    case GPS:
      fprintf( iop, "GPS %ld %ld: %f %f %f %f %f %f %d\n",
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
      fprintf( iop, "COMPASS3D %ld %ld: %f %f %f\n",
	       rec.compass[rec.entry[i].index].time.tv_sec,
	       rec.compass[rec.entry[i].index].time.tv_usec,
	       rad2deg(rec.compass[rec.entry[i].index].rx),
	       rad2deg(rec.compass[rec.entry[i].index].ry),
	       rad2deg(rec.compass[rec.entry[i].index].rz) );
      break;
    case UNKNOWN:
      break;
    default:
      break;
    }
  }
  return(0);
}



