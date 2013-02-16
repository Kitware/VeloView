#include "vvLoadDataReaction.h"

#include "pqActiveObjects.h"
#include "pqApplicationCore.h"
#include "pqCoreUtilities.h"
#include "pqDataRepresentation.h"
#include "pqObjectBuilder.h"
#include "pqPipelineSource.h"
#include "pqServer.h"
#include "pqView.h"
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
  pqDataRepresentation* repr = builder->createDataRepresentation(
    source->getOutputPort(0), pqActiveObjects::instance().activeView());
  vtkSMPropertyHelper(repr->getProxy(), "Representation").Set("Points");
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
