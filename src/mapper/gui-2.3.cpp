/****************************************************************************
** Form implementation generated from reading ui file 'gui.ui'
**
** Created: Sat Oct 12 04:38:25 2002
**      by:  The User Interface Compiler (uic)
**
** WARNING! All changes made in this file will be lost!
****************************************************************************/
#include "gui-2.3.h"

#include <qvariant.h>
#include <./mapview.h>
#include <qframe.h>
#include <qgroupbox.h>
#include <qpushbutton.h>
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

#include "gui.ui.h"

static QPixmap uic_load_pixmap_MapGUI( const QString &name )
{
    const QMimeSource *m = QMimeSourceFactory::defaultFactory()->data( name );
    if ( !m )
	return QPixmap();
    QPixmap pix;
    QImageDrag::decode( m, pix );
    return pix;
}
/* 
 *  Constructs a MapGUI which is a child of 'parent', with the 
 *  name 'name' and widget flags set to 'f'.
 *
 */
MapGUI::MapGUI( QWidget* parent,  const char* name, WFlags fl )
    : QMainWindow( parent, name, fl )
{
    (void)statusBar();
    if ( !name )
	setName( "MapGUI" );
    resize( 557, 479 ); 
    setSizePolicy( QSizePolicy( (QSizePolicy::SizeType)7, (QSizePolicy::SizeType)7, sizePolicy().hasHeightForWidth() ) );
    setMinimumSize( QSize( 540, 454 ) );
    setCaption( "Map 2D" );
    setCentralWidget( new QWidget( this, "qt_central_widget" ) );
    MapGUILayout = new QHBoxLayout( centralWidget(), 0, 0, "MapGUILayout"); 

    Layout5 = new QHBoxLayout( 0, 0, 6, "Layout5"); 

    Map = new MapView( centralWidget(), "Map" );
    Map->setSizePolicy( QSizePolicy( (QSizePolicy::SizeType)7, (QSizePolicy::SizeType)7, Map->sizePolicy().hasHeightForWidth() ) );
    Map->setMinimumSize( QSize( 400, 400 ) );
    Layout5->addWidget( Map );

    Actions = new QGroupBox( centralWidget(), "Actions" );
    Actions->setSizePolicy( QSizePolicy( (QSizePolicy::SizeType)7, (QSizePolicy::SizeType)7, Actions->sizePolicy().hasHeightForWidth() ) );
    Actions->setMinimumSize( QSize( 130, 400 ) );
    Actions->setMaximumSize( QSize( 130, 32767 ) );
    Actions->setLineWidth( 1 );
    Actions->setMargin( 0 );
    Actions->setMidLineWidth( 0 );
    Actions->setTitle( "Actions" );

    QWidget* privateLayoutWidget = new QWidget( Actions, "Layout3" );
    privateLayoutWidget->setGeometry( QRect( 10, 21, 109, 288 ) ); 
    Layout3 = new QVBoxLayout( privateLayoutWidget, 0, 6, "Layout3"); 

    PushButton1 = new QPushButton( privateLayoutWidget, "PushButton1" );
    PushButton1->setText( "Update Map" );
    Layout3->addWidget( PushButton1 );

    PushButton2 = new QPushButton( privateLayoutWidget, "PushButton2" );
    PushButton2->setText( "Center Robot" );
    Layout3->addWidget( PushButton2 );

    Frame3_2 = new QFrame( privateLayoutWidget, "Frame3_2" );
    Frame3_2->setSizePolicy( QSizePolicy( (QSizePolicy::SizeType)7, (QSizePolicy::SizeType)0, Frame3_2->sizePolicy().hasHeightForWidth() ) );
    Frame3_2->setMinimumSize( QSize( 0, 20 ) );
    Frame3_2->setFrameShape( QFrame::StyledPanel );
    Frame3_2->setFrameShadow( QFrame::Raised );
    Frame3_2->setLineWidth( 0 );
    Layout3->addWidget( Frame3_2 );

    GlobalMapType = new QPushButton( privateLayoutWidget, "GlobalMapType" );
    GlobalMapType->setText( "Global Map" );
    Layout3->addWidget( GlobalMapType );

    GlobalRayMapType = new QPushButton( privateLayoutWidget, "GlobalRayMapType" );
    GlobalRayMapType->setText( "Global Ray Map" );
    Layout3->addWidget( GlobalRayMapType );

    LocalMapType = new QPushButton( privateLayoutWidget, "LocalMapType" );
    LocalMapType->setText( "Local Map" );
    Layout3->addWidget( LocalMapType );

    RayMapType = new QPushButton( privateLayoutWidget, "RayMapType" );
    RayMapType->setText( "Local Ray Map" );
    Layout3->addWidget( RayMapType );

    ShowBeamsType = new QPushButton( privateLayoutWidget, "ShowBeamsType" );
    ShowBeamsType->setText( "Show Beams" );
    Layout3->addWidget( ShowBeamsType );

    Frame3_2_2 = new QFrame( privateLayoutWidget, "Frame3_2_2" );
    Frame3_2_2->setSizePolicy( QSizePolicy( (QSizePolicy::SizeType)7, (QSizePolicy::SizeType)0, Frame3_2_2->sizePolicy().hasHeightForWidth() ) );
    Frame3_2_2->setMinimumSize( QSize( 0, 20 ) );
    Frame3_2_2->setFrameShape( QFrame::StyledPanel );
    Frame3_2_2->setFrameShadow( QFrame::Raised );
    Frame3_2_2->setLineWidth( 0 );
    Layout3->addWidget( Frame3_2_2 );

    PushButton3 = new QPushButton( privateLayoutWidget, "PushButton3" );
    PushButton3->setText( "Clear Map" );
    Layout3->addWidget( PushButton3 );
    Layout5->addWidget( Actions );
    MapGUILayout->addLayout( Layout5 );

    // actions
    fileNewAction = new QAction( this, "fileNewAction" );
    fileNewAction->setIconSet( QIconSet( uic_load_pixmap_MapGUI( "" ) ) );
    fileNewAction->setText( "New" );
    fileNewAction->setMenuText( "&New" );
    fileNewAction->setAccel( 4194382 );
    fileOpenAction = new QAction( this, "fileOpenAction" );
    fileOpenAction->setIconSet( QIconSet( uic_load_pixmap_MapGUI( "" ) ) );
    fileOpenAction->setText( "Open" );
    fileOpenAction->setMenuText( "&Open..." );
    fileOpenAction->setAccel( 4194383 );
    fileSaveAction = new QAction( this, "fileSaveAction" );
    fileSaveAction->setIconSet( QIconSet( uic_load_pixmap_MapGUI( "" ) ) );
    fileSaveAction->setText( "Save" );
    fileSaveAction->setMenuText( "&Save" );
    fileSaveAction->setAccel( 4194387 );
    fileSaveAsAction = new QAction( this, "fileSaveAsAction" );
    fileSaveAsAction->setText( "Save As" );
    fileSaveAsAction->setMenuText( "Save &As..." );
    fileSaveAsAction->setAccel( 0 );
    filePrintAction = new QAction( this, "filePrintAction" );
    filePrintAction->setIconSet( QIconSet( uic_load_pixmap_MapGUI( "" ) ) );
    filePrintAction->setText( "Print" );
    filePrintAction->setMenuText( "&Print..." );
    filePrintAction->setAccel( 4194384 );
    fileExitAction = new QAction( this, "fileExitAction" );
    fileExitAction->setText( "Exit" );
    fileExitAction->setMenuText( "E&xit" );
    fileExitAction->setStatusTip( "Exit2" );
    fileExitAction->setAccel( 0 );


    // toolbars


    // menubar
    menubar = new QMenuBar( this, "menubar" );

    fileMenu = new QPopupMenu( this ); 
    fileNewAction->addTo( fileMenu );
    fileOpenAction->addTo( fileMenu );
    fileSaveAction->addTo( fileMenu );
    fileSaveAsAction->addTo( fileMenu );
    fileMenu->insertSeparator();
    filePrintAction->addTo( fileMenu );
    fileMenu->insertSeparator();
    fileExitAction->addTo( fileMenu );
    menubar->insertItem( "&File",  fileMenu );



    // signals and slots connections
    connect( PushButton3, SIGNAL( clicked() ), this, SLOT( PushButton3_clicked() ) );
    connect( PushButton2, SIGNAL( clicked() ), this, SLOT( PushButton2_clicked() ) );
    connect( PushButton1, SIGNAL( clicked() ), this, SLOT( PushButton1_clicked() ) );
    connect( fileNewAction, SIGNAL( activated() ), this, SLOT( MenuNewSlot() ) );
    connect( fileOpenAction, SIGNAL( activated() ), this, SLOT( MenuOpenSlot() ) );
    connect( fileSaveAction, SIGNAL( activated() ), this, SLOT( MenuSaveSlot() ) );
    connect( fileSaveAsAction, SIGNAL( activated() ), this, SLOT( MenuSaveAsSlot() ) );
    connect( filePrintAction, SIGNAL( activated() ), this, SLOT( MenuPrintSlot() ) );
    connect( fileExitAction, SIGNAL( activated() ), this, SLOT( MenuExitSlot() ) );
    connect( GlobalMapType, SIGNAL( clicked() ), this, SLOT( GlobalMapType_clicked() ) );
    connect( LocalMapType, SIGNAL( clicked() ), this, SLOT( LocalMapType_clicked() ) );
    connect( RayMapType, SIGNAL( clicked() ), this, SLOT( RayMapType_clicked() ) );
    connect( GlobalRayMapType, SIGNAL( clicked() ), this, SLOT( GlobalRayMapType_clicked() ) );
    connect( ShowBeamsType, SIGNAL( clicked() ), this, SLOT( ShowBeamsType_clicked() ) );
}

/*  
 *  Destroys the object and frees any allocated resources
 */
MapGUI::~MapGUI()
{
    // no need to delete child widgets, Qt does it all for us
}

