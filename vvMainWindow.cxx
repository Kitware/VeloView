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
#include "pqPythonShellReaction.h"
#include "pqQtMessageHandlerBehavior.h"
#include "pqRenderView.h"
#include "pqServer.h"
#include "pqSpreadSheetView.h"
#include "pqSpreadSheetVisibilityBehavior.h"
#include "pqStandardViewModules.h"
#include "pqVelodyneManager.h"
#include "vtkPVPlugin.h"
#include "vtkSMPropertyHelper.h"
#include "vvSelectionReaction.h"
#include "vvToggleSpreadSheetReaction.h"

#include <QLabel>
#include <QSplitter>

// Declare the plugin to load.
PV_PLUGIN_IMPORT_INIT(VelodyneHDLPlugin);
PV_PLUGIN_IMPORT_INIT(PythonQtPlugin);

class vvMainWindow::pqInternals
{
public:
  Ui::vvMainWindow Ui;
  pqInternals(vvMainWindow* window)
    {
    this->Ui.setupUi(window);
    this->paraviewInit(window);
    this->setupUi(window);

    window->show();
    window->raise();
    window->activateWindow();
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


    pqServer::setCoincidentTopologyResolutionModeSetting(0);

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
    new vvToggleSpreadSheetReaction(this->Ui.actionSpreadsheet, spreadsheetView);

    pqActiveObjects::instance().setActiveView(view);
    }

  void setupUi(vvMainWindow* window)
    {
    new vvSelectionReaction(vvSelectionReaction::SURFACE_POINTS,
      this->Ui.actionSelect_Visible_Points);
    new vvSelectionReaction(vvSelectionReaction::ALL_POINTS,
      this->Ui.actionSelect_All_Points);

    new pqPythonShellReaction(this->Ui.actionPython_Console);

    pqVelodyneManager::instance()->setup(
      this->Ui.action_Open,
      this->Ui.actionClose,
      this->Ui.actionOpen_Sensor_Stream,
      this->Ui.actionChoose_Calibration_File,
      this->Ui.actionReset_Camera,
      this->Ui.actionPlay,
      this->Ui.actionSeek_Forward,
      this->Ui.actionSeek_Backward,
      this->Ui.actionGo_To_Start,
      this->Ui.actionGo_To_End,
      this->Ui.actionRecord,
      this->Ui.actionMeasurement_Grid,
      this->Ui.actionSave_Screenshot,
      this->Ui.actionSave_CSV);
    }
};

//-----------------------------------------------------------------------------
vvMainWindow::vvMainWindow() : Internals (new vvMainWindow::pqInternals(this))
{
  PV_PLUGIN_IMPORT(VelodyneHDLPlugin);
  PV_PLUGIN_IMPORT(PythonQtPlugin);
}

//-----------------------------------------------------------------------------
vvMainWindow::~vvMainWindow()
{
  delete this->Internals;
  this->Internals = NULL;
}

//-----------------------------------------------------------------------------
