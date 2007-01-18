#include <qapplication.h>
#include <qpainter.h>
#include <qimage.h>
#include <qfileinfo.h>
#include <qscrollview.h>
#include <qframe.h>
#include <qstring.h>

#ifdef __cplusplus
extern "C" {
#endif

#include "fast-slam.h"

#ifdef __cplusplus
}
#endif

class MapPainter : public QScrollView {
  Q_OBJECT
    
 public:
  MapPainter( QWidget* parent = 0, const char *name = 0 );
  ~MapPainter( void ){};

  void centerView( MAP2 map, logtools_rpos2_t pos );  
  void drawContents( QPainter *p, int cx, int cy, int cw, int ch );
  void setSize( int size_x, int size_y );
  void update( MAP2 map );
  void refresh( MAP2 map );
  void showscan( MAP2 map, logtools_lasersens2_data_t lsens, double maxrange );
  void drawrobot( MAP2 map, logtools_rpos2_t pos, int color );
  void drawparticles( MAP2 map, SAMPLE_SET, int color, int showpath );
  void dumpscreen( void );
  void doPaint();

 private:
  QImage      * image;
  QPainter    * pt;
  QPixmap     * pix;
  QPixmap     * pm;

};
