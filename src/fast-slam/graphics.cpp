#include <unistd.h>
#include <stdio.h>

#ifdef __cplusplus
extern "C" {
#endif

#include "fast-slam.h"

#ifdef __cplusplus
}
#endif

#include "graphics.h"

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

QString
dumpMapName( char * prefix )
{
  static int dumpCtr = 0;  

  QString str = "/dev/null";
  QFileInfo  fi;

  do {
    str = QString( QString( prefix ) + number2str( dumpCtr ) + ".png" );
    fi = str;
    dumpCtr++;
  } while( fi.exists());

  return( str );
}

MapPainter::MapPainter( QWidget* parent, const char * )
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

  // 4000 pixel is the maximum size a qpainter can paint
  pm = new QPixmap(4000,4000);
  pm->fill( QColor( 200, 200, 255 ) );

  viewport()->setBackgroundMode( PaletteBase );
  pt = new QPainter( pm );
}

void
MapPainter::setSize( int size_x, int size_y )
{
  if (settings.dump_screen) {
    pix->resize( size_x, size_y );
  }
  image = new QImage( size_x, size_y, 8, 256 );
  for (int i=0; i<256; i++) {
    image->setColor( i, qRgb( i, i, i ) );
  }
  image->setColor( 0, qRgb( 200, 200, 255 ) );
  image->setColor( 1, qRgb( 255, 0, 0 ) );
}


void
MapPainter::drawContents( QPainter * p, int , int  , int , int )
{
  p->drawPixmap( 0, 0, *pm );
}

void
MapPainter::update( MAP2 map )
{
  int maxx = map.mapsize.x;
  int maxy = map.mapsize.y;
  
  // 4000 pixel is the maximum size a qpainter can paint
  if (maxx > 4000)
    maxx=4000;
  if (maxy > 4000)
    maxy=4000;

  for (int x=0;x<maxx;x++) {
    for (int y=0;y<maxy;y++) {
      if (map.calc[x][y]<0) {
	image->setPixel( (int) (maxx-1-x), (int) y, 1 );
      } else {
	if (map.mapsum[x][y]>0) {
	  image->setPixel( (int) (maxx-1-x), (int) y,
			   (int) (255-253*map.mapprob[x][y]) );
	} else {
	  image->setPixel( (int) (maxx-1-x), (int) y, 0 );
	}
      }
    }
  }
  if (settings.dump_screen)
    pix->convertFromImage( *image );
  pt->drawImage( 0, 0, *image );
  doPaint();
  if (settings.dump_screen) {
      QPixmap pixmap = QPixmap::grabWindow( this->winId() );
      fprintf( stderr, "dump picture\n" );
      pixmap.save( dumpMapName(settings.dump_filename), "PNG" );
  }
}

void
MapPainter::refresh( MAP2 )
{
  viewport()->repaint( TRUE );
}

void
MapPainter::showscan( MAP2 map, logtools_lasersens2_data_t lsens, double maxrange )
{
  int                   i;
  logtools_rmove2_t     nomove = {0.0, 0.0, 0.0};
  logtools_ivector2_t   start, end;
  logtools_vector2_t    abspt;
  QPen                  usedpen = QPen( qRgb( 255, 0, 0 ) );
  QPen                  maxrpen = QPen( qRgb( 255, 255, 0 ) );

  if (map_pos_from_rpos( lsens.estpos, &map, &start ) ) {
    for (i=0; i<lsens.laser.numvalues; i++) {
      if (lsens.laser.val[i]<=maxrange) {
	pt->setPen( usedpen );
	abspt = logtools_compute_laser_points( lsens.estpos, lsens.laser.val[i],
					       nomove, lsens.laser.angle[i] );
      } else {
	pt->setPen( maxrpen );
	abspt = logtools_compute_laser_points( lsens.estpos, maxrange,
					       nomove, lsens.laser.angle[i] );
      }
      if ( map_pos_from_vec2( abspt, &map, &end ) ) {
	pt->drawLine( (map.mapsize.x-1-start.x), start.y,
		      (map.mapsize.x-1-end.x), end.y );
      }
    }
  }
}

#define NUM_COLORS 4

RGB colors[] = { { 255,   0,   0 },
		 {   0, 255,   0 },
		 { 255, 255,   0 },
		 {   0,   0,   255 } };

void
MapPainter::drawrobot( MAP2 map, logtools_rpos2_t pos, int color )
{
  
  logtools_ivector2_t   vec;
  int                   cidx = color % NUM_COLORS;
  QBrush                brush = QBrush( qRgb( (int) colors[cidx].r,
					      (int) colors[cidx].g,
					      (int) colors[cidx].b ) );
  
  if (map_pos_from_rpos( pos, &map, &vec ) ) {

    pt->setBrush( brush );
    pt->drawEllipse( (map.mapsize.x-1-vec.x)-4, vec.y-4, 8, 8 );
    if (settings.dump_screen) {
      QPainter dump;
      dump.begin(pix);
      dump.setBrush( brush );
      dump.drawEllipse( (map.mapsize.x-1-vec.x)-4, vec.y-4, 8, 8 );
      dump.end();
    }
  }

}

void
MapPainter::doPaint(  ) {
  resizeContents( image->size().width(),  image->size().height() );
  viewport()->repaint( FALSE );
}


void
MapPainter::centerView( MAP2 map, logtools_rpos2_t pos )
{
  logtools_ivector2_t    vec;
  if (map_pos_from_rpos( pos, &map, &vec ) ) {
    //center( (map.mapsize.x-1-vec.x)-4, vec.y-4);
    ensureVisible( (map.mapsize.x-1-vec.x)-4, vec.y-4);

  }
}


void
MapPainter::drawparticles( MAP2 map, SAMPLE_SET pset, int, int showpath )
{

  int                   hist, i, j;
  logtools_ivector2_t   vec, vec1, vec2;
  QPen                  pen1 = QPen( qRgb( 0, 0, 0 ) ); /* black */
  QPainter              dump;

  if (0) {
    pt->setBrush( NoBrush );
    hist = pset.particle[0].histlen;
    for (j=0;j<hist;j++) {
      if ( map_pos_from_vec2(pset.particle[0].hist[j].bbox.min,
			     &map, &vec1) &&
	   map_pos_from_vec2(pset.particle[0].hist[j].bbox.max,
			     &map, &vec2)) {
	pt->drawRect( (map.mapsize.x-1-vec2.x)-1, vec1.y-1, 
		      vec2.x-vec1.x, vec2.y-vec1.y );
      }
    }
  }
  
  pt->setPen( pen1 );
  if (settings.dump_screen) {
    dump.begin(pix);
    dump.setPen( pen1 );
  }

  for (i=0;i<pset.numparticles;i++) {
    if (0 && map_pos_from_rpos( pset.particle[i].pos, &map, &vec ) ) {
      pt->drawEllipse( (map.mapsize.x-1-vec.x)-1, vec.y-1, 2, 2 );
    }
    if (showpath) {
      for (j=1;j<pset.particle[i].histlen;j++) {
	if ( map_pos_from_rpos(pset.particle[i].hist[j-1].pos, &map, &vec1) &&
	     map_pos_from_rpos(pset.particle[i].hist[j].pos, &map, &vec2) ) {
	  pt->drawLine( (map.mapsize.x-1-vec1.x)-1, vec1.y-1, 
			(map.mapsize.x-1-vec2.x)-1, vec2.y-1 );
	  if (settings.dump_screen) {
	    dump.drawLine( (map.mapsize.x-1-vec1.x)-1, vec1.y-1, 
			   (map.mapsize.x-1-vec2.x)-1, vec2.y-1 );
	  }
	}
      }
    }
  }

  if (settings.dump_screen) {
    dump.end();
  }

}

void
MapPainter::dumpscreen( void )
{
  QString filename;
  if (settings.dump_screen) {
    filename = dumpMapName(settings.dump_filename);
    fprintf( stderr, "dump picture %s\n", filename.ascii() );
    pix->save( filename, "PNG" );
  } else {
    fprintf( stderr, "can't dump picture\n" );
  }
}


