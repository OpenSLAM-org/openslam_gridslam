#include <qapplication.h>
#include <qvariant.h>
#include <qframe.h>
#include <qgroupbox.h>
#include <qpushbutton.h>
#include <qscrollbar.h>
#include <qmime.h>
#include <qdragobject.h>
#include <qlayout.h>
#include <qtooltip.h>
#include <qwhatsthis.h>
#include <qaction.h>
#include <qmenubar.h>
#include <qpopupmenu.h>
#include <qtoolbar.h>
#include <qimage.h>
#include <qpixmap.h>
#include <qmainwindow.h>
#include <qlayout.h>
#include <qpainter.h>
#include <qframe.h>
#include <qwidget.h>
#include <qvbox.h>
#include <qstring.h>
#include <qscrollview.h>
#include <qfileinfo.h>

#ifdef __cplusplus
extern "C" {
#endif

#include <navigation/utils.h>

#include "fast-slam.h"

#ifdef __cplusplus
}
#endif

class MapPainter : public QScrollView {
  Q_OBJECT
    
 public:
  MapPainter( QWidget* parent = 0, const char *name = 0 );
  ~MapPainter( void ){};

  void centerView( MAP2 map, RPOS2 pos );  
  void drawContents( QPainter *p, int cx, int cy, int cw, int ch );
  void setSize( int size_x, int size_y );
  void update( MAP2 map );
  void refresh( MAP2 map );
  void showscan( MAP2 map, LASERSENS2_DATA lsens, double maxrange );
  void drawrobot( MAP2 map, RPOS2 pos, int color );
  void drawparticles( MAP2 map, SAMPLE_SET, int color, int showpath );
  void dumpscreen( void );
  void doPaint();

 private:
  QImage      * image;
  QPainter    * pt;
  QPixmap     * pix;
  QPixmap     * pm;

};
