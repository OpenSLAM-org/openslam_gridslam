/****************************************************************************
** $Id: mapview.h,v 1.2 2004/06/11 13:57:56 haehnel Exp $
**
** Copyright (C) 1992-2000 Trolltech AS.  All rights reserved.
**
** This file is part of an example program for Qt.  This example
** program may be used, distributed and modified without limitation.
**
*****************************************************************************/

#include <qscrollview.h>
#include <qapplication.h>
#include <qmenubar.h>
#include <qpopupmenu.h>
#include <qpushbutton.h>
#include <qpainter.h>
#include <qimage.h>
#include <qpixmap.h>
#include <qmessagebox.h>
#include <qlayout.h>
#include <qlabel.h>
#include <qmultilineedit.h>
#include <qsizegrip.h>
#include <qvbox.h>
#include <qfileinfo.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

#include <navigation/utils.h>
#include "map2d.h"

#ifdef __cplusplus
}
#endif

class MapView : public QScrollView {
  Q_OBJECT
    
 public:
  MapView(QWidget* parent, const char * = 0 );

  void drawContents( QPainter *p, int cx, int cy, int cw, int ch );
  void paintRobot( RPOS2 pos );
  void plotRobotPosition( RPOS2 pos );
  void centerRobot( void );
  void saveMap( void );
  void saveMapAs( char * filename );
  void updateMap( void );
  void clearMap( void );
  void setGlobalSize( int size_x, int size_y );
  void setGlobalRaySize( int size_x, int size_y );
  void setLocalSize( int size_x, int size_y );
  void setLocalRaySize( int size_x, int size_y );
  void showRays();
  int  maptype;
  
 private:
  QImage      * gimage;
  QImage      * grimage;
  QImage      * limage;
  QImage      * rimage;
  QPainter    * pt;
  RPOS2         rpos;

};
