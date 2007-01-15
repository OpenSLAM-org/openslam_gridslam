#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <sys/types.h>
#include <sys/time.h>
#include <time.h>
#include <signal.h>
#include <unistd.h>
#include <math.h>
#include <string.h>
#include <ctype.h>
#include <values.h>

#include <navigation/utils.h>

#define MAX_LINE_LENGTH 40000

int
read_script( char *filename, REC2_DATA *script, int verbose ) {

  FILE   *fp;

  RPOS2  rpos, npos = {0.0, 0.0, 0.0};
  RVEL2  rvel = {0.0, 0.0};
  
  char inp00[80], inp01[80], inp02[80], inp03[80];
  char inp04[80], inp05[80], inp06[80], inp07[80];
  char inp08[80], inp09[80], inp10[80];
  char inp0[80], inp1[80], inp2[80], sstr[80], secstr[80];

  char line[MAX_LINE_LENGTH];
  
  char command[80];

  struct timeval curTime;
  struct tm      dayTime;
  
  int i, cnt;

  int numValues1, numValues2;

  int day, month, year, hour, minute, sec;

  double angleDiff;

  LASER_PROPERTIES2  lprop[2];
  
  int numPositions      = 0;
  int numCorrPositions  = 0;
  int numSonarScans     = 0;
  int numLaserScans     = 0;
  int numRLaserScans    = 0;
  int numHumanProb      = 0;
  int numHumanState     = 0;
  int numDynamicProb    = 0;
  int numTimes          = 0;
  int numEntries        = 0;

  /* this is one of the most awful things in beeSoft */
  int rotation_90_minus = FALSE;

  int lastFScan         = -1;
  int lastRScan         = -1;

  int FileEnd = FALSE;

  lprop[0].range.start     =   -M_PI_2;
  lprop[0].range.end       =    M_PI_2;
  lprop[0].range.delta     =      M_PI;
  lprop[0].offset.forward  =      11.5;
  lprop[0].offset.forward  =       0.0;
  lprop[0].offset.sideward =       0.0;
  lprop[0].offset.rotation =       0.0;

  lprop[1].range.start     =    M_PI_2;
  lprop[1].range.end       =  3*M_PI_2;
  lprop[1].range.delta     =      M_PI;
  lprop[1].offset.forward  =     -11.5;
  lprop[1].offset.sideward =       0.0;
  lprop[1].offset.rotation =       0.0;

  if ((fp = fopen( filename, "r")) == 0){
    fprintf(stderr, "ERROR: can't read script file %s\n",
	    filename );
    return(-1);
  }

  do{
    if (fscanf(fp, "%s", command) == EOF)
      FileEnd = TRUE;
    else{
      if (!strcmp( command, "#ROBOT") ){
	numPositions++;
	if (fscanf(fp, "%s %s %s", inp0, inp1, inp2 ) == EOF)
	  FileEnd = TRUE;
      } else if (!strcmp( command, "position:") ){
	numCorrPositions++;
	if (fscanf(fp, "%s %s %s", inp0, inp1, inp2 ) == EOF)
	  FileEnd = TRUE;
      } else if (!strcmp( command, "@SENS") ){
	numTimes++;
	if (fscanf(fp, "%d-%d-%d %d:%d:%s",
		   &day, &month, &year, &hour, &minute, inp0 ) == EOF)
	  FileEnd = TRUE;
      } else if (!strcmp( command, "#SONAR") ){
	numSonarScans++;
	if (fscanf(fp, "%d:", &numValues1) == EOF)
	  FileEnd = TRUE;
	else {
	  fgets( line, MAX_LINE_LENGTH,fp );
	}
      } else if (!strcmp( command, "#LASER") ){
	numLaserScans++;
	if (fscanf(fp, "%d %d:", &numValues1, &numValues2) == EOF)
	    FileEnd = TRUE;
	else {
	  fgets( line, MAX_LINE_LENGTH,fp );
	}
	if (numValues1>0)
	  numRLaserScans++;
	if (numValues2>0)
	  numRLaserScans++;
      } else if (!strcmp( command, "#P_HUMAN") ){
        numHumanProb++;
	if (fscanf(fp, "%d %d:", &numValues1, &numValues2) == EOF)
	    FileEnd = TRUE;
	else {
	  fgets( line, MAX_LINE_LENGTH,fp );
	}
      } else if (!strcmp( command, "#DYNAMIC") ){
        numDynamicProb++;
	if (fscanf(fp, "%d %d:", &numValues1, &numValues2) == EOF)
	    FileEnd = TRUE;
	else {
	  fgets( line, MAX_LINE_LENGTH,fp );
	}
      } else {
	fgets(command,sizeof(command),fp);
      }
    }
  } while (!FileEnd);

  FileEnd = FALSE;

  rewind( fp );

  numEntries = numPositions + numCorrPositions + numRLaserScans;
  
  script->entry   =
    (ENTRY_POSITION *) malloc( numEntries * sizeof(ENTRY_POSITION) );
  script->psens  =
    (POSSENS2_DATA *) malloc( numPositions * sizeof(POSSENS2_DATA) );
  script->cpsens =
    (POSSENS2_DATA *) malloc( numCorrPositions * sizeof(POSSENS2_DATA) );
  script->lsens  =
    (LASERSENS2_DATA *) malloc( numRLaserScans * sizeof(LASERSENS2_DATA) );

  script->numentries = numEntries;
  
  if (verbose) {
    fprintf( stderr, "**************************************************\n" );
    fprintf( stderr, "*   information\n" );
    fprintf( stderr, "**************************************************\n" );
    fprintf( stderr, "*   found %d times\n", numTimes );    
    fprintf( stderr, "*   found %d positions\n", numPositions );    
    fprintf( stderr, "*   found %d corr positions\n", numCorrPositions );    
    fprintf( stderr, "*   found %d sonar scans\n", numSonarScans );    
    fprintf( stderr, "*   found %d laser scans\n", numLaserScans );
    fprintf( stderr, "*   found %d real laser scans\n", numRLaserScans );
    fprintf( stderr, "*   found %d human probability\n", numHumanProb );
    fprintf( stderr, "**************************************************\n" );
    fprintf( stderr, "*   read script file %s\n", filename ); 
    fprintf( stderr, "**************************************************\n" );
  }

  cnt               = 0;
  numPositions      = 0;
  numCorrPositions  = 0;
  numRLaserScans    = 0;
  numSonarScans     = 0;
  numLaserScans     = 0;
  numHumanProb      = 0;

  curTime.tv_sec = 0;
  curTime.tv_usec = 0;


  do{
    
    if (fscanf(fp, "%s", command) == EOF)
      FileEnd = TRUE;
    else{
      
      /* ****************************************************************
         ****************************************************************
         **                                                            **
         **                                                            **
	 **                ROBOT POSITON                               **
	 **                                                            **
	 **                                                            **
         ****************************************************************
	 **************************************************************** */
	 
      if (!strcmp( command, "#ROBOT") ){

	script->entry[cnt].type  = POSITION;
	script->entry[cnt].index = numPositions;
	if (fscanf(fp, "%s %s %s", inp0, inp1, inp2 ) == EOF)
	  FileEnd = TRUE;
	else {
	  /*
	    rpos.y = atof( inp0 );
	    rpos.x = -atof( inp1 );
	    rpos.o = deg2rad(atof( inp2 ));
	  */
	  rpos.x = atof( inp0 );
	  rpos.y = atof( inp1 );
	  if (rotation_90_minus) {
	    rpos.o = deg2rad(90.0-atof(inp2));
	  } else {
	    rpos.o = deg2rad(atof(inp2));
	  }
	  script->psens[numPositions].time = curTime;
	  script->psens[numPositions].rpos = rpos;
	  script->psens[numPositions].rvel = rvel;
	}
	numPositions++;
	cnt++;
	
      } else if (!strcmp( command, "#ROTATION_90_MINUS") ){
	rotation_90_minus = TRUE;
	fprintf( stderr, "INFO: using ROTATION_90_MINUS mode\n" );
	
      } else if (!strcmp( command, "@SENS") ){
	
        /* **************************************************************
           **************************************************************
           **                                                          **
	   **                                                          **
	   **                TIME                                      **
	   **                                                          **
	   **                                                          **
	   **************************************************************
	   ************************************************************** */

	  if (fscanf(fp, "%d-%d-%d %d:%d:%d.%s",
		     &day, &month, &year, &hour, &minute, &sec, inp0 ) == EOF)
	    FileEnd = TRUE;
	  else {
	    
	    if (year<1900) {
	      year+=100;
	    } else {
	      year-=1900;
	    }
	    dayTime.tm_year = year;
	    dayTime.tm_mon  = month;
	    dayTime.tm_mday = day;
	    dayTime.tm_hour = hour;
	    dayTime.tm_min  = minute;
	    dayTime.tm_sec  = sec;
	    snprintf( sstr, 80, "%s000000", inp0 );
	    strncpy( secstr, sstr, 6 );
	    curTime.tv_usec = (long) atoi(secstr);
	    curTime.tv_sec  = (long) mktime( &dayTime );
	  }

      } else if (!strcmp( command, "#LASER") ){
	
        /* ***************************************************************
           ***************************************************************
	   **                                                           **
	   **                                                           **
	   **                 LASER VALUES                              **
	   **                                                           **
	   **                                                           **
	   ***************************************************************
	   *************************************************************** */

	if (fscanf(fp, "%d %d:", &numValues1, &numValues2) == EOF)
	  FileEnd = TRUE;
	else {

	  if ( numValues1>0 ) {

	    script->entry[cnt].type  = LASER_VALUES;
	    script->entry[cnt].index = numRLaserScans;

	    if (numPositions>0) {
	      script->lsens[numRLaserScans].estpos =
		script->psens[numPositions-1].rpos;
	    } else {
	      script->lsens[numRLaserScans].estpos = npos;
	    }

	    script->lsens[numRLaserScans].coord            = NULL;
	    script->lsens[numRLaserScans].ptracking.hprob  = NULL;
	    script->lsens[numRLaserScans].ptracking.pstate = NULL;

	    script->lsens[numRLaserScans].dynamic.prob     = NULL;
	    script->lsens[numRLaserScans].dynamic.numprobs = 0;

	    script->lsens[numRLaserScans].laser.time       = curTime;
	    script->lsens[numRLaserScans].id               = 0;
	    script->lsens[numRLaserScans].laser.numvalues  = numValues1;
	    script->lsens[numRLaserScans].laser.val        =
	      (double *) malloc( numValues1 * sizeof(double) );
	    script->lsens[numRLaserScans].laser.angle      =
	      (double *) malloc( numValues1 * sizeof(double) );
	    script->lsens[numRLaserScans].laser.maxrange   =
	      (int *) malloc( numValues1 * sizeof(int) );
	    script->lsens[numRLaserScans].laser.offset     =
	      (RMOVE2 *) malloc( numValues1 * sizeof(RMOVE2) );
	    
	    angleDiff = lprop[0].range.delta / (double) (numValues1-1);
	    script->lsens[numRLaserScans].laser.anglerange = 
	      lprop[0].range.delta;
	    for ( i=0; i<numValues1 ;i++) {
	      if (fscanf(fp, "%s", inp0) == EOF) {
		FileEnd = TRUE;
		break;
	      } else {
		script->lsens[numRLaserScans].laser.val[i]      = atof(inp0);
		script->lsens[numRLaserScans].laser.maxrange[i] = FALSE;
		script->lsens[numRLaserScans].laser.angle[i]    =
		  lprop[0].range.start+(i*angleDiff);
		script->lsens[numRLaserScans].laser.offset[i]   =
		  lprop[0].offset;
	      }
	    }
	    
	    lastFScan = numRLaserScans;
	    numRLaserScans++;
	    cnt++;

	  }
	  
	  if ( numValues2>0 ) {
	    
	    script->entry[cnt].type  = LASER_VALUES;
	    script->entry[cnt].index = numRLaserScans;

	    if (numPositions>0) {
	      script->lsens[numRLaserScans].estpos =
		script->psens[numPositions-1].rpos;
	    } else {
	      script->lsens[numRLaserScans].estpos = npos;
	    }
	  
	    script->lsens[numRLaserScans].coord            = NULL;
	    script->lsens[numRLaserScans].ptracking.hprob  = NULL;
	    script->lsens[numRLaserScans].ptracking.pstate = NULL;
	    
	    script->lsens[numRLaserScans].dynamic.prob     = NULL;
	    script->lsens[numRLaserScans].dynamic.numprobs = 0;

	    script->lsens[numRLaserScans].laser.time       = curTime;
	    script->lsens[numRLaserScans].id               = 1;
	    script->lsens[numRLaserScans].laser.numvalues  = numValues2;
	    script->lsens[numRLaserScans].laser.val        =
	      (double *) malloc( numValues2 * sizeof(double) );
	    script->lsens[numRLaserScans].laser.angle      =
	      (double *) malloc( numValues2 * sizeof(double) );
	    script->lsens[numRLaserScans].laser.maxrange   =
	      (int *) malloc( numValues2 * sizeof(int) );
	    script->lsens[numRLaserScans].laser.offset     =
	      (RMOVE2 *) malloc( numValues2 * sizeof(RMOVE2) );
	    angleDiff = lprop[1].range.delta / (double) (numValues2-1);
	    script->lsens[numRLaserScans].laser.anglerange = 
	      lprop[0].range.delta;
	    for ( i=0; i<numValues2 ;i++) {
	      if (fscanf(fp, "%s", inp0) == EOF) {
		FileEnd = TRUE;
		break;
	      } else {
		script->lsens[numRLaserScans].laser.val[i] =
		  atof(inp0);
		script->lsens[numRLaserScans].laser.maxrange[i] =
		  FALSE;
		script->lsens[numRLaserScans].laser.angle[i] =
		  lprop[1].range.start+(i*angleDiff);
		script->lsens[numRLaserScans].laser.offset[i] =
		  lprop[1].offset;
	      }
	    }
	    lastRScan = numRLaserScans;
	    numRLaserScans++;
	    cnt++;
	  }
	}
	
      } else if (!strcmp( command, "#DYNAMIC-PROB") ){
	
        /* ***************************************************************
           ***************************************************************
	   **                                                           **
	   **                                                           **
	   **                 HUMAN PROBABILITY                         **
	   **                                                           **
	   **                                                           **
	   ***************************************************************
	   *************************************************************** */

	if (fscanf(fp, "%d %d:", &numValues1, &numValues2) == EOF)
	  FileEnd = TRUE;
	else {

	  if ( numValues1>0 ) {
	    script->lsens[lastFScan].ptracking.hprob =
	      (double *) malloc( numValues1 * sizeof(double) );
	    if (lastFScan!=-1) {
	      for ( i=0; i<numValues1 ;i++) {
		if (fscanf(fp, "%s", inp0) == EOF) {
		  FileEnd = TRUE;
		  break;
		} else {
		  script->lsens[lastFScan].ptracking.hprob[i] = atof(inp0);
		}
	      }
	    } else {
	      for ( i=0; i<numValues1 ;i++) {
		if (fscanf(fp, "%s", inp0) == EOF) {
		  FileEnd = TRUE;
		  break;
		}
	      }
	    }
	    numHumanProb++;
	  }
	  script->lsens[lastFScan].ptracking.numhprobs = numValues1;
	  if ( numValues2>0 ) {
	    script->lsens[lastRScan].ptracking.hprob =
	      (double *) malloc( numValues2 * sizeof(double) );
	    if (lastRScan!=-1) {
	      for ( i=0; i<numValues2 ;i++) {
		if (fscanf(fp, "%s", inp0) == EOF) {
		  FileEnd = TRUE;
		  break;
		} else {
		  script->lsens[lastRScan].ptracking.hprob[i] = atof(inp0);
		}
	      }
	    } else {
	      for ( i=0; i<numValues2 ;i++) {
		if (fscanf(fp, "%s", inp0) == EOF) {
		  FileEnd = TRUE;
		  break;
		}
	      }
	    }
	    numHumanProb++;
	  }
	  script->lsens[lastRScan].ptracking.numhprobs = numValues2;
	}
	
      } else if (!strcmp( command, "P_STATES:") ){
	
        /* ***************************************************************
           ***************************************************************
	   **                                                           **
	   **                                                           **
	   **                 HUMAN PROBABILITY                         **
	   **                                                           **
	   **                                                           **
	   ***************************************************************
	   *************************************************************** */

	if (fscanf(fp, "%d:", &numValues1) == EOF)
	  FileEnd = TRUE;
	else {
	  
	  if (lastFScan!=-1) {
	    
	    script->lsens[lastFScan].ptracking.numpstates = numValues1;
	    if ( numValues1>0 ) {
	      script->lsens[lastFScan].ptracking.pstate =
		(P_STATE *) malloc( numValues1 * sizeof(P_STATE) );
	      for ( i=0; i<numValues1 ;i++) {
		if (fscanf(fp, "%s %s %s %s %s %s %s %s %s %s %s",
			   inp00, inp01, inp02, inp03, inp04, inp05,
			   inp06, inp07, inp08, inp09, inp10 ) == EOF) {
		  FileEnd = TRUE;
		  break;
		} else {
		  script->lsens[lastFScan].ptracking.pstate[i].pos.x   = atof(inp00);
		  script->lsens[lastFScan].ptracking.pstate[i].pos.y   = atof(inp01);
		  script->lsens[lastFScan].ptracking.pstate[i].pos.o   = atof(inp02);
		  script->lsens[lastFScan].ptracking.pstate[i].vel     = atof(inp03);
		  script->lsens[lastFScan].ptracking.pstate[i].cov_sx  = atof(inp04);
		  script->lsens[lastFScan].ptracking.pstate[i].cov_sy  = atof(inp05);
		  script->lsens[lastFScan].ptracking.pstate[i].cov_sxy = atof(inp06);
		  script->lsens[lastFScan].ptracking.pstate[i].ll.x    = atof(inp07);
		  script->lsens[lastFScan].ptracking.pstate[i].ll.y    = atof(inp08);
		  script->lsens[lastFScan].ptracking.pstate[i].ur.x    = atof(inp09);
		  script->lsens[lastFScan].ptracking.pstate[i].ur.y    = atof(inp10);
		}
	      }
	    }
	  } else {
	    for ( i=0; i<numValues1 ;i++) {
	      if (fscanf(fp, "%s %s %s %s %s %s %s %s %s %s %s",
			 inp00, inp01, inp02, inp03, inp04, inp05,
			 inp06, inp07, inp08, inp09, inp10 ) == EOF) {
		FileEnd = TRUE;
		break;
	      }
	    }
	  }
	  numHumanState++;
	}
	
      } else if (!strcmp( command, "#DYNAMIC") ){
	
        /* ***************************************************************
           ***************************************************************
	   **                                                           **
	   **                                                           **
	   **                 DYNAMIC PROBABILITY                       **
	   **                                                           **
	   **                                                           **
	   ***************************************************************
	   *************************************************************** */

	if (fscanf(fp, "%d %d:", &numValues1, &numValues2 ) == EOF)
	  FileEnd = TRUE;
	else {

	  if ( numValues1>0 ) {
	    script->lsens[lastFScan].dynamic.prob =
	      (double *) malloc( numValues1 * sizeof(double) );
	    if (lastFScan!=-1) {
	      for ( i=0; i<numValues1 ;i++) {
		if (fscanf(fp, "%s", inp0) == EOF) {
		  FileEnd = TRUE;
		  break;
		} else {
		  script->lsens[lastFScan].dynamic.prob[i] = atof(inp0);
		}
	      }
	    } else {
	      for ( i=0; i<numValues1 ;i++) {
		if (fscanf(fp, "%s", inp0) == EOF) {
		  FileEnd = TRUE;
		  break;
		}
	      }
	    }
	    numDynamicProb++;
	  }
	  script->lsens[lastFScan].dynamic.numprobs = numValues1;
	  if ( numValues2>0 ) {
	    script->lsens[lastRScan].dynamic.prob =
	      (double *) malloc( numValues2 * sizeof(double) );
	    if (lastRScan!=-1) {
	      for ( i=0; i<numValues2 ;i++) {
		if (fscanf(fp, "%s", inp0) == EOF) {
		  FileEnd = TRUE;
		  break;
		} else {
		  script->lsens[lastRScan].dynamic.prob[i] = atof(inp0);
		}
	      }
	    } else {
	      for ( i=0; i<numValues2 ;i++) {
		if (fscanf(fp, "%s", inp0) == EOF) {
		  FileEnd = TRUE;
		  break;
		}
	      }
	    }
	    numDynamicProb++;
	  }
	  script->lsens[lastRScan].dynamic.numprobs = numValues2;
	}
	
        /* ***************************************************************
           ***************************************************************
	   **                                                           **
	   **                                                           **
	   **                 CORR POSITION                             **
	   **                                                           **
	   **                                                           **
	   ***************************************************************
	   *************************************************************** */

      } else if (!strcmp( command, "position:") ){
	if (fscanf(fp, "%s %s %s", inp0, inp1, inp2 ) == EOF)
	  FileEnd = TRUE;
	else {
	  script->entry[cnt].type  = CORR_POSITION;
	  script->entry[cnt].index = numCorrPositions;
	  rpos.x = atof( inp0 );
	  rpos.y = atof( inp1 );
	  rpos.o = deg2rad(atof(inp2));
	  script->cpsens[numCorrPositions].time = curTime;
	  script->cpsens[numCorrPositions].rpos = rpos;
	  script->cpsens[numCorrPositions].rvel = rvel;
	  numCorrPositions++;
	  cnt++;
	}
      } else {
	if (!(command[0]=='%')){
	  /*
	    fprintf( stderr, "%s: unknown keyword %s\n",
	    prgname, command );
	    fclose(fp);
	    exit(0);
	  */
	} else {
	  fgets(command,sizeof(command),fp);
	}
      }
    }
  } while (!FileEnd);

  script->numentries    = cnt;
  script->numpositions  = numPositions;
  script->numcpositions = numCorrPositions;
  script->numlaserscans = numRLaserScans;

  if (verbose) {
    fprintf( stderr, "**************************************************\n" );
    fprintf( stderr, "*   information\n" );
    fprintf( stderr, "**************************************************\n" );
    fprintf( stderr, "*   read %d times\n", numTimes );    
    fprintf( stderr, "*   read %d positions\n", numPositions );    
    fprintf( stderr, "*   read %d corr positions\n", numCorrPositions );    
    fprintf( stderr, "*   read %d sonar scans\n", numSonarScans );    
    fprintf( stderr, "*   read %d real laser scans\n", numRLaserScans );
    fprintf( stderr, "*   read %d human probability\n", numHumanProb );
    fprintf( stderr, "*   read %d human states\n", numHumanState );
    fprintf( stderr, "**************************************************\n" );

  }

  return( 0 );
}
