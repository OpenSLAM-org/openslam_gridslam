#include "mapview.h"

#ifdef __cplusplus
extern "C" {
#endif

#include "map2d.h"

extern MAP2                              * global_map;
extern MAP2                              * local_map;
extern logtools_lasersens2_data_t        * current_scan;
extern logtools_rmove2_t                 * current_movement;

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
    break;
  case SHOW_RAYS:
    break;
  }
}

void
MapView::plotRobotPosition( logtools_rpos2_t pos  )
{
  logtools_ivector2_t  mpos;
  map_pos_from_rpos( pos, global_map, &mpos );
  gimage->setPixel( global_map->mapsize.x-1-mpos.x,
		    mpos.y, 1 );
  fprintf( stderr, "(%d:%d)", mpos.x, mpos.y );
}

void
MapView::updateMap(  )
{
  int       x, y, i, j;

  switch (maptype) {
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
      resizeContents( limage->size().width(),
		      limage->size().height() );
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
MapView::paintRobot( logtools_rpos2_t pos )
{
  int       rsize;
  logtools_ivector2_t  mpos;
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
  case LOCAL_MAP:
    rsize = (int) ( ( settings.display_robot_size * scale )  /
		    settings.local_map_resolution );
    if (map_pos_from_rpos( pos, local_map, &mpos )) {
      contentsToViewport( mpos.x, local_map->mapsize.y*scale-mpos.y-1,
			  vx, vy );  
      if ( vx>=0 && vx<local_map->mapsize.x &&
	   vy>=0 && vy<local_map->mapsize.y) {
	pt->setBrush( brush );
	pt->setPen(red);
	pt->drawEllipse( vx, vy, rsize, rsize );
      }
    }
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
  logtools_ivector2_t mpos;
  switch(maptype) {
  case GLOBAL_MAP:
    if (map_pos_from_rpos( rpos, global_map, &mpos )) {
      center( (int) (global_map->mapsize.x-1-mpos.x), (int) mpos.y );
    }
    break;
  case LOCAL_MAP:
    center( (int) ( local_map->center.x * scale ),
	    (int) ( local_map->center.y * scale ) );
    
    break;
  }
}

