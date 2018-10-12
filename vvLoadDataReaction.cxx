// Copyright 2013 Velodyne Acoustics, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
#include "vvLoadDataReaction.h"

#include "pqVelodyneManager.h"

#include "pqActiveObjects.h"
#include "pqApplicationCore.h"
#include "pqCoreUtilities.h"
#include "pqObjectBuilder.h"
#include "pqPipelineRepresentation.h"
#include "pqPipelineSource.h"
#include "pqServer.h"
#include "pqSettings.h"
#include "pqView.h"
#include "vtkDataObject.h"
#include "vtkPVArrayInformation.h"
#include "vtkPVDataInformation.h"
#include "vtkPVDataSetAttributesInformation.h"
#include "vtkSMPVRepresentationProxy.h"
#include "vtkSMPropertyHelper.h"
#include "vtkSMProxy.h"
#include "vtkSMProxyManager.h"
#include "vtkSMReaderFactory.h"
#include "vtkSMRepresentationProxy.h"

#include <QDir>
#include <QFileDialog>
#include <QFileInfo>

//-----------------------------------------------------------------------------
vvLoadDataReaction::vvLoadDataReaction(QAction* parentAction, bool separatePositionFile)
  : Superclass(parentAction)
  , SeparatePositionFile(separatePositionFile)
{
  // QObject::connect(this, SIGNAL(loadedData(pqPipelineSource*)),
  //  this, SLOT(onDataLoaded(pqPipelineSource*)));
}

//-----------------------------------------------------------------------------
vvLoadDataReaction::~vvLoadDataReaction()
{
}

//-----------------------------------------------------------------------------
void vvLoadDataReaction::onDataLoaded(pqPipelineSource* source)
{
  pqObjectBuilder* builder = pqApplicationCore::instance()->getObjectBuilder();

  if (this->PreviousSource)
  {
    builder->destroy(this->PreviousSource);
  }
  Q_ASSERT(this->PreviousSource == NULL);

  this->PreviousSource = source;
  pqActiveObjects::instance().setActiveSource(source);
  pqPipelineRepresentation* repr =
    qobject_cast<pqPipelineRepresentation*>(builder->createDataRepresentation(
      source->getOutputPort(0), pqActiveObjects::instance().activeView()));
  if (!repr)
  {
    qWarning("Failed to create representation");
    return;
  }

  vtkSMPropertyHelper(repr->getProxy(), "Representation").Set("Points");
  vtkSMPropertyHelper(repr->getProxy(), "InterpolateScalarsBeforeMapping").Set(0);

  // color by "intensity" if array is present.
  vtkSMPVRepresentationProxy* pvrp =
    vtkSMPVRepresentationProxy::SafeDownCast(repr->getRepresentationProxy());
  vtkPVDataInformation* info = repr->getInputDataInformation();
  vtkPVArrayInformation* arrayInfo =
    info->GetPointDataInformation()->GetArrayInformation("intensity");
  if (arrayInfo != NULL && pvrp != NULL)
  {
    pvrp->SetScalarColoring("intensity", vtkDataObject::FIELD_ASSOCIATION_POINTS);
  }

  repr->getProxy()->UpdateVTKObjects();
  repr->renderViewEventually();
  pqActiveObjects::instance().activeView()->resetDisplay();
}

//-----------------------------------------------------------------------------
pqPipelineSource* vvLoadDataReaction::loadData()
{
  pqServer* server = pqActiveObjects::instance().activeServer();
  vtkSMReaderFactory* readerFactory = vtkSMProxyManager::GetProxyManager()->GetReaderFactory();
  QString filters = readerFactory->GetSupportedFileTypes(server->session());
  if (!filters.isEmpty())
  {
    filters += ";;";
  }
  filters += "All files (*)";

  pqSettings* settings = pqApplicationCore::instance()->settings();
  QString defaultDir =
    settings->value("VelodyneHDLPlugin/OpenData/DefaultDir", QDir::homePath()).toString();

  QString positionFileName;
  QString fileName;

  if (this->SeparatePositionFile)
  {
    fileName = QFileDialog::getOpenFileName(pqCoreUtilities::mainWidget(), tr("Open LiDAR File"),
      defaultDir, "Wireshark Capture (*.pcap);;All files(*)");

    if (fileName.isEmpty())
    {
      return NULL;
    }

    positionFileName =
      QFileDialog::getOpenFileName(pqCoreUtilities::mainWidget(), tr("Open Position File"),
        defaultDir, "Applanix POSCAP Position Data Text Export (*.txt);;All files (*)");

    if (positionFileName.isEmpty())
    {
      return NULL;
    }
  }
  else
  {
    fileName = QFileDialog::getOpenFileName(
      pqCoreUtilities::mainWidget(), tr("Open File"), defaultDir, filters);

    if (fileName.isEmpty())
    {
      return NULL;
    }
  }

  settings->setValue(
    "VelodyneHDLPlugin/OpenData/DefaultDir", QFileInfo(fileName).absoluteDir().absolutePath());

  pqVelodyneManager::instance()->openData(fileName, positionFileName);

  return NULL;
}
