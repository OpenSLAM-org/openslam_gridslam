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

int
rec2_parse_line( char *line, REC2_DATA *rec, int alloc, int mode )
{
  static RPOS2   npos = {0.0, 0.0, 0.0};

  static char    command[MAX_CMD_LENGTH];
  static char    dummy[MAX_CMD_LENGTH];
  static char    str1[MAX_CMD_LENGTH];
  static char    str2[MAX_CMD_LENGTH];
  static char    str3[MAX_CMD_LENGTH];
  static char    str4[MAX_CMD_LENGTH];
  static char    str5[MAX_CMD_LENGTH];
  static char    str6[MAX_CMD_LENGTH];
  static char    str7[MAX_CMD_LENGTH];
  static char    str8[MAX_CMD_LENGTH];
  static char    str9[MAX_CMD_LENGTH];
  static char    str10[MAX_CMD_LENGTH];
  static char    str11[MAX_CMD_LENGTH];
  static char    str12[MAX_CMD_LENGTH];
  static char    str13[MAX_CMD_LENGTH];
  static char  * running, * valptr;

  float          range;
  double         angleDiff, time;
  long           sec, usec;
  int            i, l, nLas, nVal, tag1, tag2, tag3, tag4, antenna, count;

  static LASER_PROPERTIES2  lprop;
  static int                firsttime = TRUE;


  if (firsttime) {
    lprop.range.start     =   -M_PI_2;
    lprop.range.end       =    M_PI_2;
    lprop.range.delta     =      M_PI;
    lprop.offset.forward  =       0.0;
    lprop.offset.forward  =       0.0;
    lprop.offset.sideward =       0.0;
    lprop.offset.rotation =       0.0;
    firsttime = FALSE;
  }

  if (strlen(line)==1 && line[0]==10 )
    return(TRUE);
	   
  if (sscanf( line, "%s", command ) == EOF) {

    return(FALSE);
	  
  }
  
  if (!strncmp( command, "POS", MAX_CMD_LENGTH )) {
    
    if (sscanf( line, "%s %ld %ld: %s %s %s %s %s",
		dummy, &sec, &usec, str1, str2, str3, str4, str5 ) == EOF) {
      
      return(FALSE);
      
    } else {

      rec->entry[rec->numentries].type   = POSITION;
      rec->entry[rec->numentries].index  = rec->numpositions;
      rec->numentries++;
      
      rec->psens[rec->numpositions].time.tv_sec    = sec;
      rec->psens[rec->numpositions].time.tv_usec   = usec;
      
      rec->psens[rec->numpositions].rpos.x         = atof(str1);
      rec->psens[rec->numpositions].rpos.y         = atof(str2);
      rec->psens[rec->numpositions].rpos.o         = deg2rad(atof(str3));
      
      rec->psens[rec->numpositions].rvel.tv        = atof(str4);
      rec->psens[rec->numpositions].rvel.rv        = atof(str5);

      rec->numpositions++;
    }
    
  } else if (!strncmp( command, "POS-CORR", MAX_CMD_LENGTH )) {
    
    if (sscanf( line, "%s %ld %ld: %s %s %s",
		dummy, &sec, &usec, str1, str2, str3 ) == EOF) {
      
      return(FALSE);
      
    } else {
      
      rec->entry[rec->numentries].type   = POS_CORR;
      rec->entry[rec->numentries].index  = rec->numposcorr;
      rec->numentries++;
      
      rec->poscorr[rec->numposcorr].time.tv_sec    = sec;
      rec->poscorr[rec->numposcorr].time.tv_usec   = usec;
      
      rec->poscorr[rec->numposcorr].x         = atof(str1);
      rec->poscorr[rec->numposcorr].y         = atof(str2);
      rec->poscorr[rec->numposcorr].o         = deg2rad(atof(str3));
      
      rec->numposcorr++;
    }
    
  } else if (!strncmp( command, "GPS", MAX_CMD_LENGTH )) {
    
    if (sscanf( line, "%s %ld %ld: %s %s %s %s %s %s %s",
		dummy, &sec, &usec, str1, str2, str3, str4,
		str5, str6, str7  ) == EOF) {
      
      return(FALSE);
      
    } else {
      
      rec->entry[rec->numentries].type   = GPS;
      rec->entry[rec->numentries].index  = rec->numgps;
      rec->numentries++;
      
      rec->gps[rec->numgps].time.tv_sec  = sec;
      rec->gps[rec->numgps].time.tv_usec = usec;
      rec->gps[rec->numgps].x      = atof(str1);
      rec->gps[rec->numgps].y      = atof(str2);
      rec->gps[rec->numgps].o      = deg2rad(atof(str3));
      rec->gps[rec->numgps].xvar   = atof(str4);
      rec->gps[rec->numgps].yvar   = atof(str5);
      rec->gps[rec->numgps].ovar   = atof(str6);
      rec->gps[rec->numgps].numsats= atoi(str7);
      rec->numgps++;
      
    }
    
  } else if (!strncmp( command, "NMEA-GGA", MAX_CMD_LENGTH )) {
    
    if (sscanf( line, "%s %ld %ld: %s %s %s %s %s %s %s %s %s %s %s %s %s",
		dummy, &sec, &usec,
		str1, str2, str3, str4, str5, str6, str7, str8,
		str9, str10, str11, str12, str13 ) == EOF) {
      
      return(FALSE);
      
    } else {
      
      rec->entry[rec->numentries].type   = NMEA_GGA;
      rec->entry[rec->numentries].index  = rec->numnmea_gga;
      rec->numentries++;
      
      rec->nmea_gga[rec->numnmea_gga].time.tv_sec  = sec;
      rec->nmea_gga[rec->numnmea_gga].time.tv_usec = usec;

      rec->nmea_gga[rec->numnmea_gga].utc              = atof(str1);
      rec->nmea_gga[rec->numnmea_gga].latitude         = atof(str2);
      rec->nmea_gga[rec->numnmea_gga].lat_orient       = str3[0];
      rec->nmea_gga[rec->numnmea_gga].longitude        = atof(str4);
      rec->nmea_gga[rec->numnmea_gga].long_orient      = str5[0];
      rec->nmea_gga[rec->numnmea_gga].gps_quality      = atoi(str6);
      rec->nmea_gga[rec->numnmea_gga].num_sattelites   = atoi(str7);
      rec->nmea_gga[rec->numnmea_gga].hdop             = atof(str8);
      rec->nmea_gga[rec->numnmea_gga].sea_level        = atof(str9);
      rec->nmea_gga[rec->numnmea_gga].alitude          = atof(str10);
      rec->nmea_gga[rec->numnmea_gga].geo_sea_level    = atof(str11);
      rec->nmea_gga[rec->numnmea_gga].geo_sep          = atof(str12);
      rec->nmea_gga[rec->numnmea_gga].data_age         = atoi(str13);
      
      rec->numnmea_gga++;
      
    }
    
  } else if (!strncmp( command, "COMPASS2D", MAX_CMD_LENGTH )) {
    
    if (sscanf( line, "%s %ld %ld: %s",
		dummy, &sec, &usec, str1 ) == EOF) {
      
      return(FALSE);
      
    } else {
      
      rec->entry[rec->numentries].type   = COMPASS;
      rec->entry[rec->numentries].index  = rec->numcompass;
      rec->numentries++;
      
      rec->compass[rec->numcompass].time.tv_sec  = sec;
      rec->compass[rec->numcompass].time.tv_usec = usec;
      rec->compass[rec->numcompass].rx   = 0.0;
      rec->compass[rec->numcompass].ry   = 0.0;
      rec->compass[rec->numcompass].rz   = deg2rad(atof(str1));
      rec->numcompass++;
      
    }
    
  } else if (!strncmp( command, "COMPASS3D", MAX_CMD_LENGTH )) {
    
    if (sscanf( line, "%s %ld %ld: %s %s %s",
		dummy, &sec, &usec, str1, str2, str3 ) == EOF) {
      
      return(FALSE);
      
    } else {
      
      rec->entry[rec->numentries].type   = COMPASS;
      rec->entry[rec->numentries].index  = rec->numcompass;
      rec->numentries++;
	  
      rec->compass[rec->numcompass].time.tv_sec  = sec;
      rec->compass[rec->numcompass].time.tv_usec = usec;
      rec->compass[rec->numcompass].rx   = deg2rad(atof(str1));
      rec->compass[rec->numcompass].ry   = deg2rad(atof(str2));
      rec->compass[rec->numcompass].rz   = deg2rad(atof(str3));
      rec->numcompass++;
      
    }
    
  } else if (!strcmp( command, "CORR-POS")) {
    
    if (sscanf( line, "%s %ld %ld: %s %s %s",
		dummy, &sec, &usec, str1, str2, str3 ) == EOF) {
      
      return(FALSE);
      
    } else {
      
      rec->entry[rec->numentries].type   = CORR_POSITION;
      rec->entry[rec->numentries].index  = rec->numcpositions;
      rec->numentries++;
	  
      rec->cpsens[rec->numcpositions].time.tv_sec    = sec;
      rec->cpsens[rec->numcpositions].time.tv_usec   = usec;
      
      rec->cpsens[rec->numcpositions].rpos.x         = atof(str1);
      rec->cpsens[rec->numcpositions].rpos.y         = atof(str2);
      rec->cpsens[rec->numcpositions].rpos.o         = deg2rad(atof(str3));
      
      rec->cpsens[rec->numcpositions].rvel.tv        = 0.0;
      rec->cpsens[rec->numcpositions].rvel.rv        = 0.0;
      
      rec->numcpositions++;
      
    }
    
  } else if (!strcmp( command, "CORR-PARAM")) {
    
    if (sscanf( line, "%s %s %s %s %s",
		dummy, str1, str2, str3, str4 ) == EOF) {
      
      return(FALSE);
      
    }
    
  } else if (!strncmp( command, "MARK-POS", MAX_CMD_LENGTH )) {
    
    if (sscanf( line, "%s %ld %ld: %s %s %s %[^\n]",
		dummy, &sec, &usec, str1, str2, str3, str4 ) == EOF) {
      
      return(FALSE);
      
    } else {
      
      rec->entry[rec->numentries].type   = MARK_POS;
      rec->entry[rec->numentries].index  = rec->nummarkings;
      rec->numentries++;
      
      rec->marking[rec->nummarkings].time.tv_sec  = sec;
      rec->marking[rec->nummarkings].time.tv_usec = usec;
      rec->marking[rec->nummarkings].pos.x        = atof(str1);
      rec->marking[rec->nummarkings].pos.y        = atof(str2);
      rec->marking[rec->nummarkings].pos.o        = deg2rad(atof(str3));

      l = strlen(str4);
      rec->marking[rec->nummarkings].text = (char *) malloc(l * sizeof(char));
      strncpy( rec->marking[rec->nummarkings].text, str4, l );
      rec->nummarkings++;
      
    }
    
  } else if (!strncmp( command, "RFID", MAX_CMD_LENGTH )) {
    
    if (sscanf( line, "%s %ld %ld: %x %x %x %x %d %d",
		dummy, &sec, &usec, &tag1, &tag2, &tag3, &tag4,
		&antenna, &count ) == EOF) {
      
      return(FALSE);
      
    } else {
      
      rec->entry[rec->numentries].type   = RFID_TAG;
      rec->entry[rec->numentries].index  = rec->numrfid;
      rec->numentries++;
      
      rec->rfid[rec->numrfid].time.tv_sec  = sec;
      rec->rfid[rec->numrfid].time.tv_usec = usec;

      rec->rfid[rec->numrfid].tag     = 
	(((unsigned long long int) tag1) << 48) +
	(((unsigned long long int) tag2) << 32) +
	(((unsigned long long int) tag3) << 16) +
	((unsigned long long int) tag4);
      
      rec->rfid[rec->numrfid].antenna = antenna;
      rec->rfid[rec->numrfid].count   = count;

      if (rec->numpositions>0)
	rec->rfid[rec->numrfid].estpos =
	  rec->psens[rec->numpositions-1].rpos;
      else
	rec->rfid[rec->numrfid].estpos = npos;

      rec->numrfid++;
      
    }
    
  } else if (!strcmp( command, "CARMEN-LASER") ){
    
    if (sscanf( line, "%s %ld %ld %d %d %f:",
		dummy, &sec, &usec, &nLas, &nVal, &range ) == EOF) {
      return(FALSE);
      
    } else {

      rec->entry[rec->numentries].type   = LASER_VALUES;
      rec->entry[rec->numentries].index  = rec->numlaserscans;
      rec->numentries++;
	  
      rec->lsens[rec->numlaserscans].id                 = nLas;
      rec->lsens[rec->numlaserscans].laser.time.tv_sec  = sec;
      rec->lsens[rec->numlaserscans].laser.time.tv_usec = usec;
      rec->lsens[rec->numlaserscans].laser.numvalues    = nVal;
      rec->lsens[rec->numlaserscans].coord              = NULL;
      rec->lsens[rec->numlaserscans].ptracking.hprob    = NULL;
      rec->lsens[rec->numlaserscans].ptracking.pstate   = NULL;
      
      rec->lsens[rec->numlaserscans].dynamic.prob       = NULL;
      rec->lsens[rec->numlaserscans].dynamic.numprobs   = 0;

      if (alloc) {
	rec->lsens[rec->numlaserscans].laser.val =
	  (double *) malloc( nVal * sizeof(double) );
	rec->lsens[rec->numlaserscans].laser.maxrange =
	  (int *) malloc( nVal * sizeof(int) );
	rec->lsens[rec->numlaserscans].laser.angle =
	  (double *) malloc( nVal * sizeof(double) );
	rec->lsens[rec->numlaserscans].laser.offset =
	  (RMOVE2 *) malloc( nVal * sizeof(RMOVE2) );
      }
      rec->lsens[rec->numlaserscans].laser.anglerange = deg2rad(range);
      if (fabs(rad2deg(lprop.range.delta)-range)>DEFAULT_EPSILON) {
	lprop.range.delta = deg2rad(range);
	lprop.range.start = -(lprop.range.delta/2.0);
	lprop.range.end   = (lprop.range.delta/2.0);
	
      }
      angleDiff = lprop.range.delta / (double) (nVal-1);

      running = line;
      strtok( running, " ");
      strtok( NULL, " ");
      strtok( NULL, " ");
      strtok( NULL, " ");
      strtok( NULL, " ");
      strtok( NULL, " ");
      for (i=0;i<nVal;i++) {
	valptr = strtok( NULL, " ");
	if (valptr==NULL) {
	  return(FALSE);
	} else {
	  rec->lsens[rec->numlaserscans].laser.maxrange[i]  =
	    FALSE;
	  rec->lsens[rec->numlaserscans].laser.val[i]       =
	    100.0 * atof(valptr);
	  rec->lsens[rec->numlaserscans].laser.angle[i]     =
	    lprop.range.start+(i*angleDiff);
	  rec->lsens[rec->numlaserscans].laser.offset[i]    =
	    lprop.offset;
	}
      }
      if (rec->numpositions>0)
	rec->lsens[rec->numlaserscans].estpos =
	  rec->psens[rec->numpositions-1].rpos;
      else
	rec->lsens[rec->numlaserscans].estpos = npos;
      
      rec->numlaserscans++;
    }
    
  } else if (!strcmp( command, "ODOM") ){

    if (sscanf( line, "%s %s %s %s %s %s %s %s",
		dummy, str1, str2, str3, str4, str5, str6, str7 ) == EOF) {
      
      return(FALSE);
      
    } else {

      rec->entry[rec->numentries].type   = POSITION;
      rec->entry[rec->numentries].index  = rec->numpositions;
      rec->numentries++;
      
      rec->psens[rec->numpositions].rpos.x         = atof(str1)*100.0;
      rec->psens[rec->numpositions].rpos.y         = atof(str2)*100.0;
      rec->psens[rec->numpositions].rpos.o         = atof(str3);
      
      rec->psens[rec->numpositions].rvel.tv        = atof(str4);
      rec->psens[rec->numpositions].rvel.rv        = atof(str5);
 
      time = atof(str7);
      convert_time( time, &rec->psens[rec->numpositions].time );
      rec->numpositions++;
    }
    
    
  } else if (!strcmp( command, "FLASER") ){
    
    if (sscanf( line, "%s %d ",
		dummy, &nVal ) == EOF) {
      return(FALSE);
      
    } else {

      rec->entry[rec->numentries].type   = LASER_VALUES;
      rec->entry[rec->numentries].index  = rec->numlaserscans;
      rec->numentries++;
	  
      rec->lsens[rec->numlaserscans].id                 = 0;
      rec->lsens[rec->numlaserscans].laser.numvalues    = nVal;

      rec->lsens[rec->numlaserscans].coord              = NULL;
      rec->lsens[rec->numlaserscans].ptracking.hprob    = NULL;
      rec->lsens[rec->numlaserscans].ptracking.pstate   = NULL;
      rec->lsens[rec->numlaserscans].dynamic.prob       = NULL;
      rec->lsens[rec->numlaserscans].dynamic.numprobs   = 0;

      if (alloc) {
	rec->lsens[rec->numlaserscans].laser.val =
	  (double *) malloc( nVal * sizeof(double) );
	rec->lsens[rec->numlaserscans].laser.maxrange =
	  (int *) malloc( nVal * sizeof(int) );
	rec->lsens[rec->numlaserscans].laser.angle =
	  (double *) malloc( nVal * sizeof(double) );
	rec->lsens[rec->numlaserscans].laser.offset =
	  (RMOVE2 *) malloc( nVal * sizeof(RMOVE2) );
      }
      rec->lsens[rec->numlaserscans].laser.anglerange = M_PI;
      if (fabs(rad2deg(lprop.range.delta)-range)>DEFAULT_EPSILON) {
	lprop.range.delta = M_PI;
	lprop.range.start = -(lprop.range.delta/2.0);
	lprop.range.end   = (lprop.range.delta/2.0);
      }
      angleDiff = lprop.range.delta / (double) (nVal-1);

      running = line;
      strtok( running, " ");
      strtok( NULL, " ");
      for (i=0;i<nVal;i++) {
	valptr = strtok( NULL, " ");
	if (valptr==NULL) {
	  return(FALSE);
	} else {
	  rec->lsens[rec->numlaserscans].laser.maxrange[i]  =
	    FALSE;
	  rec->lsens[rec->numlaserscans].laser.val[i]       =
	    100.0 * atof(valptr);
	  rec->lsens[rec->numlaserscans].laser.angle[i]     =
	    lprop.range.start+(i*angleDiff);
	  rec->lsens[rec->numlaserscans].laser.offset[i]    =
	    lprop.offset;
	}
      }
      valptr = strtok( NULL, " ");
      rec->lsens[rec->numlaserscans].estpos.x = 100.0 * atof(valptr);
      valptr = strtok( NULL, " ");
      rec->lsens[rec->numlaserscans].estpos.y = 100.0 * atof(valptr);
      valptr = strtok( NULL, " ");
      rec->lsens[rec->numlaserscans].estpos.o = atof(valptr);
      valptr = strtok( NULL, " ");
      time = atof(valptr);
      convert_time( time, &rec->lsens[rec->numlaserscans].laser.time );
      rec->numlaserscans++;
    }
    
  } else if (!strcmp( command, "RLASER") ){
    
    if (sscanf( line, "%s %d ",
		dummy, &nVal ) == EOF) {
      return(FALSE);
      
    } else {

      rec->entry[rec->numentries].type   = LASER_VALUES;
      rec->entry[rec->numentries].index  = rec->numlaserscans;
      rec->numentries++;
	  
      rec->lsens[rec->numlaserscans].id                 = 1;
      rec->lsens[rec->numlaserscans].laser.numvalues    = nVal;

      rec->lsens[rec->numlaserscans].coord              = NULL;
      rec->lsens[rec->numlaserscans].ptracking.hprob    = NULL;
      rec->lsens[rec->numlaserscans].ptracking.pstate   = NULL;
      rec->lsens[rec->numlaserscans].dynamic.prob       = NULL;
      rec->lsens[rec->numlaserscans].dynamic.numprobs   = 0;

      if (alloc) {
	rec->lsens[rec->numlaserscans].laser.val =
	  (double *) malloc( nVal * sizeof(double) );
	rec->lsens[rec->numlaserscans].laser.maxrange =
	  (int *) malloc( nVal * sizeof(int) );
	rec->lsens[rec->numlaserscans].laser.angle =
	  (double *) malloc( nVal * sizeof(double) );
	rec->lsens[rec->numlaserscans].laser.offset =
	  (RMOVE2 *) malloc( nVal * sizeof(RMOVE2) );
      }
      rec->lsens[rec->numlaserscans].laser.anglerange = M_PI;
      if (fabs(rad2deg(lprop.range.delta)-range)>DEFAULT_EPSILON) {
	lprop.range.delta = M_PI;
	lprop.range.start = -(lprop.range.delta/2.0);
	lprop.range.end   = (lprop.range.delta/2.0);
      }
      angleDiff = lprop.range.delta / (double) (nVal-1);

      running = line;
      strtok( running, " ");
      strtok( NULL, " ");
      for (i=0;i<nVal;i++) {
	valptr = strtok( NULL, " ");
	if (valptr==NULL) {
	  return(FALSE);
	} else {
	  rec->lsens[rec->numlaserscans].laser.maxrange[i]  =
	    FALSE;
	  rec->lsens[rec->numlaserscans].laser.val[i]       =
	    100.0 * atof(valptr);
	  rec->lsens[rec->numlaserscans].laser.angle[i]     =
	    lprop.range.start+(i*angleDiff);
	  rec->lsens[rec->numlaserscans].laser.offset[i]    =
	    lprop.offset;
	}
      }
      valptr = strtok( NULL, " ");
      rec->lsens[rec->numlaserscans].estpos.x = 100.0 * atof(valptr);
      valptr = strtok( NULL, " ");
      rec->lsens[rec->numlaserscans].estpos.y = 100.0 * atof(valptr);
      valptr = strtok( NULL, " ");
      rec->lsens[rec->numlaserscans].estpos.o = atof(valptr);
      valptr = strtok( NULL, " ");
      time = atof(valptr);
      convert_time( time, &rec->lsens[rec->numlaserscans].laser.time );
      rec->numlaserscans++;
    }
    
  } else if (!strcmp( command, "LASER-RANGE") ){
    
    if (sscanf( line, "%s %ld %ld %d %d %f:",
		dummy, &sec, &usec, &nLas, &nVal, &range ) == EOF) {
      return(FALSE);
      
    } else {

      rec->entry[rec->numentries].type   = LASER_VALUES;
      rec->entry[rec->numentries].index  = rec->numlaserscans;
      rec->numentries++;
	  
      rec->lsens[rec->numlaserscans].id                 = nLas;
      rec->lsens[rec->numlaserscans].laser.time.tv_sec  = sec;
      rec->lsens[rec->numlaserscans].laser.time.tv_usec = usec;
      rec->lsens[rec->numlaserscans].laser.numvalues    = nVal;
      rec->lsens[rec->numlaserscans].coord              = NULL;
      rec->lsens[rec->numlaserscans].ptracking.hprob    = NULL;
      rec->lsens[rec->numlaserscans].ptracking.pstate   = NULL;
      
      rec->lsens[rec->numlaserscans].dynamic.prob       = NULL;
      rec->lsens[rec->numlaserscans].dynamic.numprobs   = 0;

      if (alloc) {
	rec->lsens[rec->numlaserscans].laser.val =
	  (double *) malloc( nVal * sizeof(double) );
	rec->lsens[rec->numlaserscans].laser.maxrange =
	  (int *) malloc( nVal * sizeof(int) );
	rec->lsens[rec->numlaserscans].laser.angle =
	  (double *) malloc( nVal * sizeof(double) );
	rec->lsens[rec->numlaserscans].laser.offset =
	  (RMOVE2 *) malloc( nVal * sizeof(RMOVE2) );
      }
      rec->lsens[rec->numlaserscans].laser.anglerange = deg2rad(range);
      if (fabs(rad2deg(lprop.range.delta)-range)>DEFAULT_EPSILON) {
	lprop.range.delta = deg2rad(range);
	lprop.range.start = -(lprop.range.delta/2.0);
	lprop.range.end   = (lprop.range.delta/2.0);
	
      }
      angleDiff = lprop.range.delta / (double) (nVal-1);

      running = line;
      strtok( running, " ");
      strtok( NULL, " ");
      strtok( NULL, " ");
      strtok( NULL, " ");
      strtok( NULL, " ");
      strtok( NULL, " ");
      for (i=0;i<nVal;i++) {
	valptr = strtok( NULL, " ");
	if (valptr==NULL) {
	  return(FALSE);
	} else {
	  rec->lsens[rec->numlaserscans].laser.maxrange[i]  =
	    FALSE;
	  rec->lsens[rec->numlaserscans].laser.val[i]       =
	    atof(valptr);
	  rec->lsens[rec->numlaserscans].laser.angle[i]     =
	    lprop.range.start+(i*angleDiff);
	  rec->lsens[rec->numlaserscans].laser.offset[i]    =
	    lprop.offset;
	}
      }
      if (rec->numpositions>0)
	rec->lsens[rec->numlaserscans].estpos =
	  rec->psens[rec->numpositions-1].rpos;
      else
	rec->lsens[rec->numlaserscans].estpos = npos;
      
      rec->numlaserscans++;
    }
    
  } else if (!strcmp( command, "LASER") ){
    
    if (sscanf( line, "%s %ld %ld %d %d:",
		dummy, &sec, &usec, &nLas, &nVal ) == EOF) {
      
      return(FALSE);
      
    } else {
      
      rec->entry[rec->numentries].type   = LASER_VALUES;
      rec->entry[rec->numentries].index  = rec->numlaserscans;
      rec->numentries++;
	  
      rec->lsens[rec->numlaserscans].id                 = nLas;
      rec->lsens[rec->numlaserscans].laser.time.tv_sec  = sec;
      rec->lsens[rec->numlaserscans].laser.time.tv_usec = usec;
      rec->lsens[rec->numlaserscans].laser.numvalues    = nVal;

      if (alloc) {
	rec->lsens[rec->numlaserscans].laser.val =
	  (double *) malloc( nVal * sizeof(double) );
	rec->lsens[rec->numlaserscans].laser.maxrange =
	  (int *) malloc( nVal * sizeof(int) );
	rec->lsens[rec->numlaserscans].laser.angle =
	  (double *) malloc( nVal * sizeof(double) );
	rec->lsens[rec->numlaserscans].laser.offset =
	  (RMOVE2 *) malloc( nVal * sizeof(RMOVE2) );
      }
      
      rec->lsens[rec->numlaserscans].coord            = NULL;
      rec->lsens[rec->numlaserscans].ptracking.hprob  = NULL;
      rec->lsens[rec->numlaserscans].ptracking.pstate = NULL;
      
      rec->lsens[rec->numlaserscans].dynamic.prob     = NULL;
      rec->lsens[rec->numlaserscans].dynamic.numprobs = 0;
      
      if (nVal==360 || nVal==180)
	angleDiff = lprop.range.delta / (double) (nVal);
      else
	angleDiff = lprop.range.delta / (double) (nVal-1);
      rec->lsens[rec->numlaserscans].laser.anglerange = 
	lprop.range.delta;
      
      rec->lsens[rec->numlaserscans].laser.anglerange   = M_PI;
      running = line;
      strtok( running, " ");
      strtok( NULL, " ");
      strtok( NULL, " ");
      strtok( NULL, " ");
      strtok( NULL, " ");
      for (i=0;i<nVal;i++) {
	valptr = strtok( NULL, " ");
	if (valptr==NULL) {
	  return(FALSE);
	} else {
	  rec->lsens[rec->numlaserscans].laser.maxrange[i]   =
	    FALSE;
	  rec->lsens[rec->numlaserscans].laser.val[i] =
	    atof(valptr);
	  rec->lsens[rec->numlaserscans].laser.angle[i]      =
	    lprop.range.start + i*angleDiff;
	  rec->lsens[rec->numlaserscans].laser.offset[i]     =
	    lprop.offset;
	}
      }
      if (rec->numpositions>0)
	rec->lsens[rec->numlaserscans].estpos =
	  rec->psens[rec->numpositions-1].rpos;
      else
	rec->lsens[rec->numlaserscans].estpos = npos;
      
      rec->numlaserscans++;
      
    }
    
  } else if (!strcmp( command, "DYNAMIC-PROB") ){
    
    /*
       ***************************************************************
       ***************************************************************
       **                                                           **
       **                                                           **
       **                 HUMAN PROBABILITY                         **
       **                                                           **
       **                                                           **
       ***************************************************************
       ***************************************************************
    */
    
    if (sscanf( line, "%s %ld %ld %d %d %f:",
		dummy, &sec, &usec, &nLas, &nVal, &range ) == EOF) {
      
      return(FALSE);
      
    } else {
      
      if ( rec->numlaserscans>0 &&
	   rec->lsens[rec->numlaserscans-1].id==nLas ) {
	if (alloc) {
	  rec->lsens[rec->numlaserscans-1].ptracking.hprob =
	    (double *) malloc( nVal * sizeof(double) );
	}
	running = line;
	strtok( running, " ");
	strtok( NULL, " ");
	strtok( NULL, " ");
	strtok( NULL, " ");
	strtok( NULL, " ");
	strtok( NULL, " ");
	for ( i=0; i<nVal ;i++) {
	  valptr = strtok( NULL, " ");
	  if (valptr==NULL) {
	    return(FALSE);
	  } else {
	    rec->lsens[rec->numlaserscans-1].ptracking.hprob[i] =
	      atof(valptr);
	  }
	}
	
      }
    }
    
  } else {
    
    if (!(command[0]=='#' || command[0]=='*')) {
      if (!(mode && READ_MODE_DONT_STOP)) {
	fprintf( stderr, "ERROR: unknown keyword %s\n", command );
	return(FALSE);
      }
    }
    
  }

  return(TRUE);
}
