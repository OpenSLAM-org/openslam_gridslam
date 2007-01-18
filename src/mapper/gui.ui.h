/****************************************************************************
** ui.h extension file, included from the uic-generated form implementation.
**
** If you wish to add, delete or rename slots use Qt Designer which will
** update this file, preserving your code. Create an init() slot in place of
** a constructor, and a destroy() slot in place of a destructor.
*****************************************************************************/

#include <qfiledialog.h>

#ifdef __cplusplus
extern "C" {
#endif

#include "map2d.h"

#ifdef __cplusplus
}
#endif

void MapGUI::UpdateMapSlot()
{
  Map->updateMap();
}

void MapGUI::CenterRobotSlot()
{
  Map->centerRobot();
}

void MapGUI::ClearMapSlot()
{
  Map->clearMap();
}

void MapGUI::MenuNewSlot()
{
  Map->clearMap();
  fprintf( stderr, "pressed new button\n" );
}

void MapGUI::MenuOpenSlot()
{
  fprintf( stderr, "pressed open button\n" );
}

void MapGUI::MenuPrintSlot()
{
  fprintf( stderr, "pressed print button\n" );
}

void MapGUI::MenuExitSlot()
{
  exit(0);
}

void MapGUI::GlobalMapTypeSlot()
{
  Map->maptype = GLOBAL_MAP;
  Map->updateMap();
  Map->centerRobot();
}


void MapGUI::LocalMapTypeSlot()
{
  Map->maptype = LOCAL_MAP;
  Map->updateMap();
  Map->centerRobot();
}


void MapGUI::ShowBeamsTypeSlot()
{
  Map->maptype = SHOW_RAYS;
  Map->updateMap();
  Map->centerRobot();
}

void MapGUI::ViewPathSlot()
{
  fprintf( stderr, "x1\n" );
  if (ViewPathAction->isOn()) {
    settings.view_path = TRUE;
  } else {
    settings.view_path = FALSE;
  }
  
}

void MapGUI::ChangeMapSlot()
{
  change_map = ChangeMapAction->isOn();
}

void MapGUI::ClearMapSlot( bool )
{

}
