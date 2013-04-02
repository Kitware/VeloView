#include "vvLoadDataReaction.h"

#include "pqActiveObjects.h"
#include "pqApplicationCore.h"
#include "pqCoreUtilities.h"
#include "pqObjectBuilder.h"
#include "pqPipelineRepresentation.h"
#include "pqPipelineSource.h"
#include "pqServer.h"
#include "pqView.h"
#include "vtkDataObject.h"
#include "vtkPVArrayInformation.h"
#include "vtkPVDataInformation.h"
#include "vtkPVDataSetAttributesInformation.h"
#include "vtkSMPropertyHelper.h"
#include "vtkSMProxy.h"
#include "vtkSMProxyManager.h"
#include "vtkSMReaderFactory.h"

#include <QFileDialog>
//-----------------------------------------------------------------------------
vvLoadDataReaction::vvLoadDataReaction(QAction* parentAction)
  : Superclass(parentAction)
{
  QObject::connect(this, SIGNAL(loadedData(pqPipelineSource*)),
    this, SLOT(onDataLoaded(pqPipelineSource*)));
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
  pqPipelineRepresentation* repr = qobject_cast<pqPipelineRepresentation*>(
    builder->createDataRepresentation(
    source->getOutputPort(0), pqActiveObjects::instance().activeView()));
  if (!repr)
    {
    qWarning("Failed to create representation");
    return;
    }

  vtkSMPropertyHelper(repr->getProxy(), "Representation").Set("Points");
  vtkSMPropertyHelper(repr->getProxy(), "InterpolateScalarsBeforeMapping").Set(0);

  // color by "intensity" if array is present.
  vtkPVDataInformation* info = repr->getInputDataInformation();
  vtkPVArrayInformation* arrayInfo =
    info->GetPointDataInformation()->GetArrayInformation("intensity");
  if (arrayInfo !=NULL)
    {
    repr->colorByArray("intensity", vtkDataObject::FIELD_ASSOCIATION_POINTS);
    }

  repr->getProxy()->UpdateVTKObjects();
  repr->renderViewEventually();
  pqActiveObjects::instance().activeView()->resetDisplay();
}

//-----------------------------------------------------------------------------
pqPipelineSource* vvLoadDataReaction::loadData()
{
  pqServer* server = pqActiveObjects::instance().activeServer();
  vtkSMReaderFactory* readerFactory =
    vtkSMProxyManager::GetProxyManager()->GetReaderFactory();
  QString filters = readerFactory->GetSupportedFileTypes(server->session());
  if (!filters.isEmpty())
    {
    filters += ";;";
    }
  filters += "All files (*)";

 QString fileName = QFileDialog::getOpenFileName(
    pqCoreUtilities::mainWidget(), tr("Open File"), QString(), filters);
 if (!fileName.isEmpty())
   {
   QStringList files;
   files << fileName;
   return pqLoadDataReaction::loadData(files);
   }
 return NULL;
}
