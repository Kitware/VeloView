#include "vvLoadDataReaction.h"

#include "pqActiveObjects.h"
#include "pqApplicationCore.h"
#include "pqDataRepresentation.h"
#include "pqObjectBuilder.h"
#include "pqPipelineSource.h"
#include "pqView.h"
#include "vtkSMPropertyHelper.h"
#include "vtkSMProxy.h"

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
