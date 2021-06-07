#include "lqOpenRecentFilesReaction.h"

#include "lqHelper.h"
#include "lqOpenPcapReaction.h"

#include <vtkSMPropertyHelper.h>

#include <pqApplicationCore.h>
#include <pqPipelineSource.h>
#include <pqServerManagerModel.h>
#include <pqSettings.h>

#include <QAction>
#include <QMenu>
#include <QSignalMapper>

//-----------------------------------------------------------------------------
lqOpenRecentFilesReaction::lqOpenRecentFilesReaction(QMenu* recentFilesMenu,
                                                     QAction* clearRecentFiles,
                                                     QObject *parent) :
  Superclass(parent)
{
  this->RecentFilesMenu = recentFilesMenu;

  pqServerManagerModel* smmodel = pqApplicationCore::instance()->getServerManagerModel();
  this->connect(smmodel, SIGNAL(sourceAdded(pqPipelineSource*)), SLOT(onSourceAdded(pqPipelineSource*)));

  connect(clearRecentFiles, SIGNAL(triggered()), this, SLOT(onClearMenu()));

  this->Connection = vtkSmartPointer<vtkEventQtSlotConnect>::New();

  this->RestoreRecentFilesFromSettings();
}

//-----------------------------------------------------------------------------
void lqOpenRecentFilesReaction::onSourceAdded(pqPipelineSource* src)
{
  if (IsLidarReaderProxy(src->getProxy()))
  {
    // Get the pcap Name
    vtkSMProperty* pcapProp = src->getProxy()->GetProperty("FileName");
    if (!pcapProp)
    {
      std::cout << "Lidar Reader Proxy has no property named \"FileName\" " << std::endl;
      return;
    }

    QString pcapName = QString(vtkSMPropertyHelper(pcapProp).GetAsString());

    // We connect the property "Filename" of the reader
    // - If "FileName" is not already set (default behavior):
    //   It will allow to create a new recent File entry as soon as it is set
    // - If "FileName" is already set (can happens if the user creates a reader in python):
    //   It will allows to create a new recent File entry if the "FileName" is modified by the user

    // the code in inspired from
    // Paraview-Source/ParaViewCore/ServerManager/Core/vtkSMGlobalPropertiesProxy::TargetPropertyModified()
    // This allows in the receiver to have information on the property which is "listened"
    this->Connection->Connect(src->getProxy()->GetProperty("FileName"),
                         vtkCommand::ModifiedEvent,
                         this, SLOT(onPcapUpdate(vtkObject*, unsigned long, void*))) ;

    if(!pcapName.isNull() && !pcapName.isEmpty())
    {
      createNewRecentFile(pcapName);
      this->SaveRecentFilesInSettings();
    }
  }
}

//-----------------------------------------------------------------------------
void lqOpenRecentFilesReaction::onPcapUpdate(vtkObject* caller, unsigned long, void*)
{
  vtkSMProperty* pcapProp = vtkSMProperty::SafeDownCast(caller);
  if(!pcapProp)
  {
    return;
  }
  QString pcapName = QString(vtkSMPropertyHelper(pcapProp).GetAsString());

  if(!pcapName.isNull() && !pcapName.isEmpty())
  {
    createNewRecentFile(pcapName);
    this->SaveRecentFilesInSettings();
  }
}

//-----------------------------------------------------------------------------
void lqOpenRecentFilesReaction::createNewRecentFile(QString pcapName)
{
  // If the current pcap Name is not already in the recent files list
  // we add a new entry (action) to the recentFilesMenu
  if(!IsAlreadyARecentFiles(pcapName))
  {
    if(recentFilesActions.size() >= this->sizeOfTheQueue)
    {
      this->RemoveFirstRecentFilesAction();
    }

    // This allows to passing the filename to the slot
    // Get from https://stackoverflow.com/questions/5153157/passing-an-argument-to-a-slot
    QAction* openAct = new QAction(pcapName, this);
    QSignalMapper* signalMapper = new QSignalMapper (this) ;
    connect (openAct, SIGNAL(triggered()), signalMapper, SLOT(map()));
    signalMapper->setMapping(openAct, pcapName) ;

    connect(signalMapper, SIGNAL(mapped(QString)), this, SLOT(onOpenRecentFile(QString))) ;

    RecentFilesMenu->addAction(openAct);
    recentFilesActions.push_back(openAct);
  }
}

//-----------------------------------------------------------------------------
void lqOpenRecentFilesReaction::onOpenRecentFile(QString filename)
{
  if (filename.endsWith(".pcap"))
  {
    lqOpenPcapReaction::createSourceFromFile(filename);
  }
  else
  {
    std::cout << "The file is not a pcap file, nothing is done" << std::endl;
  }
}

//-----------------------------------------------------------------------------
void lqOpenRecentFilesReaction::RemoveFirstRecentFilesAction()
{
  if(this->RecentFilesMenu->isEmpty())
  {
    return;
  }
  QAction * action = recentFilesActions.front();
  this->RecentFilesMenu->removeAction(action);
  recentFilesActions.pop_front();
}

//-----------------------------------------------------------------------------
void lqOpenRecentFilesReaction::onClearMenu()
{
  while (!recentFilesActions.empty())
  {
    RemoveFirstRecentFilesAction();
  }
  this->SaveRecentFilesInSettings();
}

//-----------------------------------------------------------------------------
bool lqOpenRecentFilesReaction::IsAlreadyARecentFiles(QString filename)
{
  for(QAction* action : recentFilesActions)
  {
    if(filename.compare(action->text(), Qt::CaseSensitive) == 0)
    {
      return true;
    }
  }
  return false;
}

//-----------------------------------------------------------------------------
void lqOpenRecentFilesReaction::SaveRecentFilesInSettings()
{
  pqSettings* settings = pqApplicationCore::instance()->settings();
  for(int unsigned i = 0; i < sizeOfTheQueue; i++)
  {
    if(i < recentFilesActions.size())
    {
      QAction * action = recentFilesActions[i];
      settings->setValue("LidarPlugin/RecentFiles/" + QString(i), action->text());
    }
    else
    {
      settings->setValue("LidarPlugin/RecentFiles/" + QString(i), "");
    }
  }
}

//-----------------------------------------------------------------------------
void lqOpenRecentFilesReaction::RestoreRecentFilesFromSettings()
{
  pqSettings* settings = pqApplicationCore::instance()->settings();
  for(int unsigned i = 0; i < sizeOfTheQueue; i++)
  {
    QString currentPcap = settings->value("LidarPlugin/RecentFiles/" + QString(i), "a").toString();
    if(!currentPcap.isNull() && !currentPcap.isEmpty())
    {
      this->createNewRecentFile(currentPcap);
    }
  }
}
