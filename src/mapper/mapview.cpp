#include "mapview.h"

#ifdef __cplusplus
extern "C" {
#endif

#include <math.h>
#include <navigation/utils.h>

#include "map2d.h"

int map_pos_from_rpos( RPOS2 pos, MAP2 *map, iVECTOR2 *v );
int map_pos_from_vec2( VECTOR2 pos, MAP2 *map, iVECTOR2 *v );
int compute_rmap_pos_from_vec2( VECTOR2 vec, MAP2 map, iVECTOR2 *v );

extern MAP2                   * global_map;
extern MAP2                   * global_ray_map;
extern MAP2                   * local_map;
extern MAP2                   * local_ray_map;
extern LASERSENS2_DATA        * current_scan;
extern RMOVE2                 * current_movement;

#ifdef __cplusplus
}
#endif

int scale = 1;

double beam_factor = 5.0;
  

MapView::MapView( QWidget* parent, const char * )
  : QScrollView( parent )
{
#if QT_VERSION >= 300
  setSizePolicy( QSizePolicy( (QSizePolicy::SizeType)7,
			      (QSizePolicy::SizeType)7,
			      1, 1,
			      sizePolicy().hasHeightForWidth() ) );
#else
  setSizePolicy( QSizePolicy( (QSizePolicy::SizeType)7,
			      (QSizePolicy::SizeType)7,
			      sizePolicy().hasHeightForWidth() ) );
#endif  
  viewport()->setBackgroundMode( PaletteBase );
  pt = new QPainter( viewport() );
}


void
MapView::setGlobalSize( int size_x, int size_y )
{
  if (settings.use_global_map) {
    gimage = new QImage( size_x, size_y, 8, 256 );
    
    for (int i=0; i<256; i++) {
      gimage->setColor( i, qRgb( i, i, i ) );
    }
    gimage->setColor( 0, qRgb( 200, 200, 255 ) );
    gimage->setColor( 1, qRgb( 0, 255, 0 ) );
  }
}

void
MapView::setGlobalRaySize( int size_x, int size_y )
{
  if (settings.use_global_ray_map) {
    grimage = new QImage( size_x, size_y, 8, 256 );
    for (int i=0; i<256; i++) {
      grimage->setColor( i, qRgb( i, i, i ) );
    }
  }
}

void
MapView::setLocalSize( int size_x, int size_y )
{
  if (settings.use_correction) {
    limage = new QImage( size_x*scale, size_y*scale, 8, 256 );
    
    for (int i=0; i<250; i++) {
      limage->setColor( i, qRgb( (int) (255-(i*2.55)),
				 (int) (255-(i*2.55)),
				 (int) (255-(i*2.55)) ) );
    }
    limage->setColor( 255, qRgb( 255, 0, 0 ) );
    limage->setColor( 254, qRgb( 0, 0, 255 ) );
  }
}

void
MapView::setLocalRaySize( int size_x, int size_y )
{
  if (settings.use_local_ray_map) {
    rimage = new QImage( size_x*scale, size_y*scale, 8, 256 );
    
    for (int i=0; i<=100; i++) {
      rimage->setColor( i, qRgb( (int) (255-(i*2.55)),
				 (int) (255-(i*2.55)),
				 (int) (255-(i*2.55)) ) );
    }
    rimage->setColor( 255, qRgb( 255, 0, 0 ) );
    rimage->setColor( 254, qRgb( 0, 0, 255 ) );
  }
}

QString
number2str( int num )
{
  if (num<10) {
    return("000"+QString::number( num ));
  } else if (num<100) {
    return("00"+QString::number( num ));
  } else if (num<1000) {
    return("0"+QString::number( num ));
  }
  return(QString::number( num ));
}

char *
dumpMapName( char * prefix )
{
  static int dumpCtr = 0;  
  static char name[MAX_NAME_LENGTH];

  QString str = "/dev/null";
  QFileInfo  fi;

  do {
    str = QString( QString( prefix ) + number2str( dumpCtr ) + ".png" );
    fi = str;
    dumpCtr++;
  } while( fi.exists());
  strncpy( name, str.ascii(), MAX_NAME_LENGTH );

  return( name );
}

void
MapView::drawContents( QPainter *p, int cx, int cy, int cw, int ch )
{
  p->fillRect( cx, cy, cw, ch, colorGroup().brush( QColorGroup::Base ) );
  switch(maptype) {
  case GLOBAL_MAP:
    if (settings.use_global_map) {
      p->drawImage( 0, 0, *gimage );
    }
    break;
  case LOCAL_MAP:
    if (settings.use_correction) {
      p->drawImage( 0, 0, *limage );
    }
    if (settings.dump_maps) {
	write_map( *local_map, dumpMapName(settings.dump_mapnames) );
    }
    break;
  case LOCAL_RAY_MAP:
    if (settings.use_local_ray_map) {
      p->drawImage( 0, 0, *rimage );
    }
    break;
  case GLOBAL_RAY_MAP:
    if (settings.use_global_ray_map) {
      p->drawImage( 0, 0, *grimage );
    }
    break;
  case SHOW_RAYS:
    break;
  }
}

void
MapView::plotRobotPosition( RPOS2 pos  )
{
  iVECTOR2  mpos;
  map_pos_from_rpos( pos, global_map, &mpos );
  gimage->setPixel( global_ray_map->mapsize.x-1-mpos.x,
		    mpos.y, 1 );
  fprintf( stderr, "(%d:%d)", mpos.x, mpos.y );
}

void
MapView::updateMap(  )
{
  RMOVE2    nomove = {0.0, 0.0, 0.0};
  RPOS2     rpos,npos = {0.0, 0.0, 0.0};
  VECTOR2   pt;
  int       x, y, i, j, si, sj;
  iVECTOR2  v;

  switch (maptype) {
  case GLOBAL_RAY_MAP:
    if (settings.use_global_ray_map) {
      compute_probs_of_global_ray_map( global_ray_map );
      for (x=0;x<global_ray_map->mapsize.x;x++) {
	for (y=0;y<global_ray_map->mapsize.y;y++) {
	  grimage->setPixel( (int) (global_ray_map->mapsize.x-1-x), (int) y,
			     (int) (255*(1.0-global_ray_map->mapprob[x][y])) );
	}
      }
      resizeContents( grimage->size().width(),
		      grimage->size().height() );
    }
    break;
  case GLOBAL_MAP:
    if (settings.use_global_map) {
      compute_probs_of_global_map( global_map );
      for (x=0;x<global_map->mapsize.x;x++) {
	for (y=0;y<global_map->mapsize.y;y++) {
	  if (global_map->mapsum[x][y]>0) {
	    gimage->setPixel( (int) (global_map->mapsize.x-1-x), (int) y,
			      (int) (255-253*global_map->mapprob[x][y]) );
	  } else {
	    global_map->mapprob[x][y] = settings.global_map_std_val;
	    gimage->setPixel( (int) (global_map->mapsize.x-1-x), (int) y, 0 );
	  }
	}
      }
      resizeContents( gimage->size().width(), gimage->size().height() );
    }
    break;
  case LOCAL_MAP:
    if (settings.use_correction) {
      for (x=0;x<local_map->mapsize.x;x++) {
	for (y=0;y<local_map->mapsize.y;y++) {
	  for (i=0;i<scale;i++) {
	    for (j=0;j<scale;j++) {
	      limage->setPixel( (int) (x*scale+i),
				(int) ((local_map->mapsize.y-y-1)*scale+j),
				(int) (200.0 * local_map->mapprob[x][y]) );
	    }
	  }
	}
      }
      /*
      if ( current_scan !=NULL ) {
	rpos = compute_rpos2_with_movement2( npos, *current_movement );
	for (i=0; i<current_scan->laser.numvalues; i++) {
	  
	  pt = compute_laser_abs_point( rpos, current_scan->laser.val[i],
					// current_scan->laser.offset[i],
					nomove,
					current_scan->laser.angle[i] );
	  if (compute_rmap_pos_from_vec2(pt, *local_map, &v )) {
	    for (si=-2;si<scale+2;si++) {
	      for (sj=-2;sj<scale+2;sj++) {
		if ( v.x*scale+si>=0 &&
		     v.x*scale+si<local_map->mapsize.x*scale &&
		     (local_map->mapsize.y-v.y-1)*scale+sj>=0 &&
		     (local_map->mapsize.y-v.y-1)*scale+sj<local_map->mapsize.y*scale) {
		  limage->setPixel( (int) (v.x*scale+si),
				    (int) ((local_map->mapsize.y-v.y-1)*
					   scale+sj),
				    255 );
		}
	      }
	    }
	  }
	}
	}*/
      resizeContents( limage->size().width(),
		      limage->size().height() );
    }
    break;
  case LOCAL_RAY_MAP:
    if (settings.use_local_ray_map) {
      for (x=0;x<local_ray_map->mapsize.x;x++) {
	for (y=0;y<local_ray_map->mapsize.y;y++) {
	  for (i=0;i<scale;i++) {
	    for (j=0;j<scale;j++) {
	      rimage->setPixel( (int) (x*scale+i),
				(int) ((local_ray_map->mapsize.y-y-1)*scale+j),
				(int) (100*local_ray_map->mapprob[x][y]) );
	    }
	  }
	}
      }
      if ( current_scan !=NULL ) {
	rpos = compute_rpos2_with_movement2( npos, *current_movement );
	for (i=0; i<current_scan->laser.numvalues; i++) {

	  pt = compute_laser_abs_point( rpos, current_scan->laser.val[i],
					// current_scan->laser.offset[i],
					nomove,
					current_scan->laser.angle[i] );
	  if (compute_rmap_pos_from_vec2(pt, *local_ray_map, &v )) {
	    for (si=0;si<scale;si++) {
	      for (sj=0;sj<scale;sj++) {
		if (current_scan->ptracking.hprob[i]<settings.max_dynamic_prob)
		//		if (local_ray_map->mapprob[v.x][v.y] < 0.5 )
		  rimage->setPixel( v.x*scale+si,
				    (local_ray_map->mapsize.y-v.y-1)*scale+sj,
				    255 );
		else
		  rimage->setPixel( v.x*scale+si,
				    (local_ray_map->mapsize.y-v.y-1)*scale+sj,
				    254 );
	      }
	    }
	  }
	}
      }
      resizeContents( rimage->size().width(),
		      rimage->size().height() );
    }
    break;
  case SHOW_RAYS:
    resizeContents( viewport()->size().width(),
		    viewport()->size().height() );
    break;
  }
  
  viewport()->repaint( FALSE );
  //      plot_robot_path();
}

void
MapView::clearMap( void )
{
  int x, y;
  for (x=0;x<global_map->mapsize.x;x++) {
    for (y=0;y<global_map->mapsize.y;y++) {
      global_map->mapsum[x][y]  = 0;
      global_map->maphit[x][y]  = 0;
      global_map->mapprob[x][y] = settings.global_map_std_val;
    }
  }
  global_map->offset = rpos;
  updateMap();
}

void
MapView::paintRobot( RPOS2 pos )
{
  int       rsize;
  iVECTOR2  mpos;
  QBrush    brush( yellow );
  int       vx, vy;
  switch (maptype) {
  case GLOBAL_MAP:
    if (map_pos_from_rpos( pos, global_map, &mpos )) {
      contentsToViewport( global_map->mapsize.x-1-mpos.x, mpos.y, vx, vy );  
      if ( vx>=0 && vx<global_map->mapsize.x &&
	   vy>=0 && vy<global_map->mapsize.y) {
	pt->setBrush( brush );
	pt->setPen(red);
	pt->drawEllipse( vx, vy,
			 settings.display_pixel_robot_size,
			 settings.display_pixel_robot_size );
      }
      rpos = pos;
    }
    break;
  case GLOBAL_RAY_MAP:
    if (map_pos_from_rpos( pos, global_ray_map, &mpos )) {
      contentsToViewport( global_ray_map->mapsize.x-1-mpos.x, mpos.y,
			  vx, vy );  
      if ( vx>=0 && vx<global_ray_map->mapsize.x &&
	   vy>=0 && vy<global_ray_map->mapsize.y) {
	pt->setBrush( brush );
	pt->setPen(red);
	pt->drawEllipse( vx, vy,
			 settings.display_pixel_robot_size,
			 settings.display_pixel_robot_size );
      }
      rpos = pos;
    }
    break;
  case LOCAL_RAY_MAP:
    rsize = (int) ( ( settings.display_robot_size * scale ) /
		    settings.local_ray_map_resolution );
    if (map_pos_from_rpos( pos, local_ray_map, &mpos )) {
      contentsToViewport( mpos.x, local_ray_map->mapsize.y*scale-mpos.y-1,
			  vx, vy );  
      if ( vx>=0 && vx<local_ray_map->mapsize.x &&
	   vy>=0 && vy<local_ray_map->mapsize.y) {
	pt->setBrush( brush );
	pt->setPen(red);
	pt->drawEllipse( vx, vy, rsize, rsize );
      }
    }
    break;
  case LOCAL_MAP:
    rsize = (int) ( ( settings.display_robot_size * scale )  /
		    settings.local_map_resolution );
    if (map_pos_from_rpos( pos, local_map, &mpos )) {
      contentsToViewport( mpos.x, local_map->mapsize.y*scale-mpos.y-1,
			  vx, vy );  
      if ( vx>=0 && vx<local_ray_map->mapsize.x &&
	   vy>=0 && vy<local_ray_map->mapsize.y) {
	pt->setBrush( brush );
	pt->setPen(red);
	pt->drawEllipse( vx, vy, rsize, rsize );
      }
    }
    /*
    if ( current_scan !=NULL ) {
      rpos = compute_rpos2_with_movement2( npos, *current_movement );
      for (i=0; i<current_scan->laser.numvalues; i++) {
	apt = compute_laser_abs_point( rpos, current_scan->laser.val[i],
				       // current_scan->laser.offset[i],
				       nomove,
				       current_scan->laser.angle[i] );
	
	if (compute_rmap_pos_from_vec2(apt, *local_map, &mpos )) {
	  contentsToViewport( mpos.x, local_map->mapsize.y-mpos.y-1,
			      vx, vy );  
	  pt->drawEllipse( vx, vy, 3, 3 );
	}
      }
      }*/
    break;
  }
}

void
MapView::showRays( void )
{
  int c, i, sy;

  
  pt->eraseRect ( 0, 0,
		  viewport()->size().width(),
		  viewport()->size().height() );
  sy = (int) (viewport()->size().height() / 2.0);
  if (current_scan != NULL) {
    for (i=0; i<current_scan->laser.numvalues; i++) {
      if (settings.use_global_ray_map || settings.use_local_ray_map)
	c = (int) (255-255*current_scan->ptracking.hprob[i]);
      else
	c = 255;
      pt->setPen( QPen( qRgb( c, c, c ), 3 ) );
      pt->drawLine( 5, sy,
		    (int) (5+( current_scan->coord[i].relpt.x ) / beam_factor),
		    (int) (sy-( current_scan->coord[i].relpt.y ) / beam_factor) );
    }
  }
}


void
MapView::centerRobot( void )
{
  iVECTOR2 mpos;
  switch(maptype) {
  case GLOBAL_MAP:
    if (map_pos_from_rpos( rpos, global_map, &mpos )) {
      center( (int) (global_map->mapsize.x-1-mpos.x), (int) mpos.y );
    }
    break;
  case GLOBAL_RAY_MAP:
    if (map_pos_from_rpos( rpos, global_ray_map, &mpos )) {
      center( (int) (global_ray_map->mapsize.x-1-mpos.x), (int) mpos.y );
    }
    break;
  case LOCAL_MAP:
    center( (int) ( local_map->center.x * scale ),
	    (int) ( local_map->center.y * scale ) );
    
    break;
  case LOCAL_RAY_MAP:
    center( (int) ( local_ray_map->center.x * scale ),
	    (int) ( local_ray_map->center.y * scale ) );
    
    break;
  }
}

void
MapView::saveMap( void )
{
  updateMap();
  write_bee_map( *global_map, settings.global_map_filename );
}

void
MapView::saveMapAs( char * filename  )
{
  updateMap();
  write_map( *global_map, filename );
}


