#include "vvMainWindow.h"
#include "ui_vvMainWindow.h"

#include "pqActiveObjects.h"
#include "pqApplicationCore.h"
#include "pqAutoLoadPluginXMLBehavior.h"
#include "pqCommandLineOptionsBehavior.h"
#include "pqCrashRecoveryBehavior.h"
#include "pqInterfaceTracker.h"
#include "pqObjectBuilder.h"
#include "pqPersistentMainWindowStateBehavior.h"
#include "pqQtMessageHandlerBehavior.h"
#include "pqRenderView.h"
#include "pqSpreadSheetView.h"
#include "pqSpreadSheetVisibilityBehavior.h"
#include "pqStandardViewModules.h"
#include "vtkPVPlugin.h"
#include "vtkSMPropertyHelper.h"
#include "vvLoadDataReaction.h"
#include "vvSelectionReaction.h"
#include "vvToggleSpreadSheetReaction.h"

#include <QSplitter>


// Declare the plugin to load.
PV_PLUGIN_IMPORT_INIT(VelodyneHDLPlugin);

class vvMainWindow::pqInternals
{
public:
  Ui::vvMainWindow Ui;
  pqInternals(vvMainWindow* window)
    {
    this->Ui.setupUi(window);
    this->paraviewInit(window);
    this->setupUi(window);
    }

private:
  void paraviewInit(vvMainWindow* window)
    {
    pqApplicationCore* core = pqApplicationCore::instance();

    // Register ParaView interfaces.
    pqInterfaceTracker* pgm = core->interfaceTracker();
    pgm->addInterface(new pqStandardViewModules(pgm));

    // Define application behaviors.
    new pqAutoLoadPluginXMLBehavior(window);
    new pqCommandLineOptionsBehavior(window);
    new pqCrashRecoveryBehavior(window);
    new pqPersistentMainWindowStateBehavior(window);
    new pqQtMessageHandlerBehavior(window);
    new pqSpreadSheetVisibilityBehavior(window);

    // Connect to builtin server.
    pqObjectBuilder* builder = core->getObjectBuilder();
    pqServer* server = builder->createServer(pqServerResource("builtin:"));
    pqActiveObjects::instance().setActiveServer(server);

    // Create a default view.
    pqView* view = builder->createView(pqRenderView::renderViewType(), server);

    vtkSMPropertyHelper(view->getProxy(),"CenterAxesVisibility").Set(0);
    double bgcolor[3] = {0, 0, 0};
    vtkSMPropertyHelper(view->getProxy(), "Background").Set(bgcolor, 3);
    // MultiSamples doesn't work, we need to set that up before registering the proxy.
    //vtkSMPropertyHelper(view->getProxy(),"MultiSamples").Set(1);
    view->getProxy()->UpdateVTKObjects();

    // Create a horizontal splitter as the central widget, add views to splitter
    QSplitter* splitter = new QSplitter(Qt::Horizontal);
    window->setCentralWidget(splitter);
    splitter->addWidget(view->getWidget());

    pqView* spreadsheetView = builder->createView(pqSpreadSheetView::spreadsheetViewType(), server);
    spreadsheetView->getProxy()->UpdateVTKObjects();
    splitter->addWidget(spreadsheetView->getWidget());
    new vvToggleSpreadSheetReaction(this->Ui.actionShow_Spreadsheet, spreadsheetView);

    pqActiveObjects::instance().setActiveView(view);
    }

  void setupUi(vvMainWindow* window)
    {
    new vvLoadDataReaction(this->Ui.action_Open);
    new vvSelectionReaction(vvSelectionReaction::SURFACE_POINTS,
      this->Ui.actionSelect_Visible_Points);
    new vvSelectionReaction(vvSelectionReaction::ALL_POINTS,
      this->Ui.actionSelect_All_Points);
    }
};

//-----------------------------------------------------------------------------
vvMainWindow::vvMainWindow() : Internals (new vvMainWindow::pqInternals(this))
{
  PV_PLUGIN_IMPORT(VelodyneHDLPlugin);
}

//-----------------------------------------------------------------------------
vvMainWindow::~vvMainWindow()
{
  delete this->Internals;
  this->Internals = NULL;
}

//-----------------------------------------------------------------------------
