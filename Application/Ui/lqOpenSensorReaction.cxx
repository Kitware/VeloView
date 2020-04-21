#include "lqOpenSensorReaction.h"

#include <vtkStream.h>
#include <vtkLidarStream.h>
#include <vtkPositionOrientationStream.h>
#include <vtkPVDataInformation.h>
#include <vtkPVDataSetAttributesInformation.h>
#include <vtkPVArrayInformation.h>
#include <vtkSMProxy.h>
#include <vtkSMParaViewPipelineControllerWithRendering.h>

#include "pqPVApplicationCore.h"
#include "pqView.h"
#include "pqObjectBuilder.h"
#include "pqActiveObjects.h"

#include "lqSensorWidget.h"

#include <QString>

//-----------------------------------------------------------------------------
lqOpenSensorReaction::lqOpenSensorReaction(QAction *action)
  : Superclass(action)
{
}

//-----------------------------------------------------------------------------
void lqOpenSensorReaction::onTriggered()
{
  pqServer* server = pqActiveObjects::instance().activeServer();
  pqObjectBuilder* builder = pqApplicationCore::instance()->getObjectBuilder();
  pqPipelineSource* lidarSource = builder->createSource("sources", "LidarStream", server);
  pqPipelineSource* imuSource = builder->createSource("sources", "PositionOrientationStream", server);

  // update active view
  pqActiveObjects::instance().setActiveSource(lidarSource);
  pqView* view = pqActiveObjects::instance().activeView();
  vtkNew<vtkSMParaViewPipelineControllerWithRendering> controller;
  controller->Show(lidarSource->getSourceProxy(), 0, view->getViewProxy());
  controller->Show(imuSource->getSourceProxy(), 0, view->getViewProxy());
}
