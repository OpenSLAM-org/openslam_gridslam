/****************************************************************************
** Form interface generated from reading ui file 'gui.ui'
**
** Created: Sat Oct 12 05:15:27 2002
**      by:  The User Interface Compiler (uic)
**
** WARNING! All changes made in this file will be lost!
****************************************************************************/
#ifndef MAPGUI_H
#define MAPGUI_H

#include <qvariant.h>
#include <qmainwindow.h>
#include <navigation/utils.h>

class QVBoxLayout; 
class QHBoxLayout; 
class QGridLayout; 
class QAction;
class QActionGroup;
class QToolBar;
class QPopupMenu;
class MapView;
class QFrame;
class QGroupBox;
class QPushButton;

class MapGUI : public QMainWindow
{ 
    Q_OBJECT

public:
    MapGUI( QWidget* parent = 0, const char* name = 0, WFlags fl = WType_TopLevel );
    ~MapGUI();

    MapView* Map;
    QGroupBox* Actions;
    QPushButton* PushButton1;
    QPushButton* PushButton2;
    QFrame* Frame3_2;
    QPushButton* GlobalMapType;
    QPushButton* GlobalRayMapType;
    QPushButton* LocalMapType;
    QPushButton* RayMapType;
    QPushButton* ShowBeamsType;
    QFrame* Frame3_2_2;
    QPushButton* PushButton3;
    QMenuBar *menubar;
    QPopupMenu *fileMenu;
    QAction* fileNewAction;
    QAction* fileOpenAction;
    QAction* fileSaveAction;
    QAction* fileSaveAsAction;
    QAction* filePrintAction;
    QAction* fileExitAction;


public slots:
    virtual void PushButton1_clicked();
    virtual void PushButton2_clicked();
    virtual void PushButton3_clicked();
    virtual void MenuNewSlot();
    virtual void MenuSaveSlot();
    virtual void MenuOpenSlot();
    virtual void MenuSaveAsSlot();
    virtual void MenuPrintSlot();
    virtual void MenuExitSlot();
    virtual void GlobalMapType_clicked();
    virtual void LocalMapType_clicked();
    virtual void RayMapType_clicked();
    virtual void GlobalRayMapType_clicked();
    virtual void ShowBeamsType_clicked();

protected:
    QHBoxLayout* MapGUILayout;
    QHBoxLayout* Layout5;
    QVBoxLayout* Layout3;
};

#endif // MAPGUI_H
