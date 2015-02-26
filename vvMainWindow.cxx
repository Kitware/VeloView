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

#include "vvMainWindow.h"
#include "ui_vvMainWindow.h"

#include "vvLoadDataReaction.h"
#include "vvToggleSpreadSheetReaction.h"

#include <vtkSMProxyManager.h>
#include <vtkSMSessionProxyManager.h>

#include <pqActiveObjects.h>
#include <pqApplicationCore.h>
#include <pqAutoLoadPluginXMLBehavior.h>
#include <pqCommandLineOptionsBehavior.h>
#include <pqCrashRecoveryBehavior.h>
#include <pqDataTimeStepBehavior.h>
#include <pqDefaultViewBehavior.h>
#include <pqInterfaceTracker.h>
#include <pqObjectBuilder.h>
#include "pqObjectPickingBehavior.h"
#include <pqPersistentMainWindowStateBehavior.h>
#include <pqPythonShellReaction.h>
#include <pqQtMessageHandlerBehavior.h>
#include <pqRenderView.h>
#include <pqRenderViewSelectionReaction.h>
#include <pqServer.h>
#include <pqSettings.h>
#include <pqSpreadSheetView.h>
#include <pqSpreadSheetVisibilityBehavior.h>
#include <pqSpreadSheetViewDecorator.h>
#include <pqStandardPropertyWidgetInterface.h>
#include <pqStandardViewFrameActionsImplementation.h>
#include <pqVelodyneManager.h>
#include <vtkPVPlugin.h>
#include <vtkSMPropertyHelper.h>

#include <QLabel>
#include <QSplitter>

#include <cassert>

// Declare the plugin to load.
PV_PLUGIN_IMPORT_INIT(VelodyneHDLPlugin);
PV_PLUGIN_IMPORT_INIT(PythonQtPlugin);

class vvMainWindow::pqInternals
{
public:
  pqInternals(vvMainWindow* window) : Ui(), MainView(0)
    {
    this->Ui.setupUi(window);
    this->paraviewInit(window);
    this->setupUi(window);

    QActionGroup* dualReturnFilterActions = new QActionGroup(window);
    dualReturnFilterActions->addAction(this->Ui.actionDualReturnModeDual);
    dualReturnFilterActions->addAction(this->Ui.actionDualReturnDistanceNear);
    dualReturnFilterActions->addAction(this->Ui.actionDualReturnDistanceFar);
    dualReturnFilterActions->addAction(this->Ui.actionDualReturnIntensityHigh);
    dualReturnFilterActions->addAction(this->Ui.actionDualReturnIntensityLow);

    window->show();
    window->raise();
    window->activateWindow();
    }
  Ui::vvMainWindow Ui;
  pqRenderView* MainView;

private:
  void paraviewInit(vvMainWindow* window)
    {
    pqApplicationCore* core = pqApplicationCore::instance();

    // Register ParaView interfaces.
    pqInterfaceTracker* pgm = core->interfaceTracker();
//    pgm->addInterface(new pqStandardViewModules(pgm));
    pgm->addInterface(new pqStandardPropertyWidgetInterface(pgm));
    pgm->addInterface(new pqStandardViewFrameActionsImplementation(pgm));

    // Define application behaviors.
    new pqQtMessageHandlerBehavior(window);
    new pqDataTimeStepBehavior(window);
    new pqSpreadSheetVisibilityBehavior(window);
    new pqObjectPickingBehavior(window);
//    new pqDefaultViewBehavior(window);
    new pqCrashRecoveryBehavior(window);
    new pqAutoLoadPluginXMLBehavior(window);
    new pqCommandLineOptionsBehavior(window);
    new pqPersistentMainWindowStateBehavior(window);

    // Connect to builtin server.
    pqObjectBuilder* builder = core->getObjectBuilder();
    pqServer* server = builder->createServer(pqServerResource("builtin:"));
    pqActiveObjects::instance().setActiveServer(server);

    // Set default render view settings
    vtkSMSessionProxyManager* pxm = vtkSMProxyManager::GetProxyManager()->GetActiveSessionProxyManager();
    vtkSMProxy* renderviewsettings =  pxm->GetProxy("RenderViewSettings");
    assert(renderviewsettings);

    vtkSMPropertyHelper(renderviewsettings,
                        "ResolveCoincidentTopology").Set(0);

    // Create a default view.
    pqRenderView* view = qobject_cast<pqRenderView*>(builder->createView(pqRenderView::renderViewType(), server));
    assert(view);
    this->MainView = view;

    vtkSMPropertyHelper(view->getProxy(),"CenterAxesVisibility").Set(0);
    double bgcolor[3] = {0, 0, 0};
    vtkSMPropertyHelper(view->getProxy(), "Background").Set(bgcolor, 3);
    // MultiSamples doesn't work, we need to set that up before registering the proxy.
    //vtkSMPropertyHelper(view->getProxy(),"MultiSamples").Set(1);
    view->getProxy()->UpdateVTKObjects();

    // Create a horizontal splitter as the central widget, add views to splitter
    QSplitter* splitter = new QSplitter(Qt::Horizontal);
    window->setCentralWidget(splitter);

    // Add the main widget to the left
    splitter->addWidget(view->getWidget());

    QSplitter* vSplitter = new QSplitter(Qt::Vertical);
    splitter->addWidget(vSplitter);

    pqView* overheadView = builder->createView(pqRenderView::renderViewType(), server);
//    overheadView->SetInteractionMode("2D");
    overheadView->getProxy()->UpdateVTKObjects();
    // dont add to the splitter just yet
    // TODO: These sizes should not be absolute things
    overheadView->getWidget()->setMinimumSize(300, 200);
    vSplitter->addWidget(overheadView->getWidget());
    new vvToggleSpreadSheetReaction(this->Ui.actionOverheadView, overheadView);

    pqView* spreadsheetView = builder->createView(pqSpreadSheetView::spreadsheetViewType(), server);
    spreadsheetView->getProxy()->UpdateVTKObjects();
    vSplitter->addWidget(spreadsheetView->getWidget());
    new vvToggleSpreadSheetReaction(this->Ui.actionSpreadsheet, spreadsheetView);
    pqSpreadSheetView* ssview = qobject_cast<pqSpreadSheetView*>(spreadsheetView);
    assert(spreadsheetView);
    pqSpreadSheetViewDecorator* dec = new pqSpreadSheetViewDecorator(ssview);
    dec->setPrecision(3);
    dec->setFixedRepresentation(true);

    pqActiveObjects::instance().setActiveView(view);
    }

  void setupUi(vvMainWindow* window)
    {
    new pqRenderViewSelectionReaction(
                                      this->Ui.actionSelect_Visible_Points,
                                      this->MainView,
                                      pqRenderViewSelectionReaction::SELECT_SURFACE_POINTS
                                      );
    new pqRenderViewSelectionReaction(
                                      this->Ui.actionSelect_All_Points,
                                      this->MainView,
                                      pqRenderViewSelectionReaction::SELECT_FRUSTUM_POINTS
                                      );

    new pqPythonShellReaction(this->Ui.actionPython_Console);

    pqVelodyneManager::instance()->setup();

    pqSettings* const settings = pqApplicationCore::instance()->settings();
    const QVariant& gridVisible =
      settings->value("VelodyneHDLPlugin/MeasurementGrid/Visibility", true);
    this->Ui.actionMeasurement_Grid->setChecked(gridVisible.toBool());

    new vvLoadDataReaction(this->Ui.actionOpenPcap, false);
    new vvLoadDataReaction(this->Ui.actionOpenApplanix, true);

    connect(this->Ui.actionOpen_Sensor_Stream, SIGNAL(triggered()),
            pqVelodyneManager::instance(), SLOT(onOpenSensor()));

    connect(this->Ui.actionMeasurement_Grid, SIGNAL(toggled(bool)),
            pqVelodyneManager::instance(), SLOT(onMeasurementGrid(bool)));
    }
};

//-----------------------------------------------------------------------------
vvMainWindow::vvMainWindow() : Internals (new vvMainWindow::pqInternals(this))
{
//  this->tabifyDockWidget(
//    this->Internal->colorMapEditorDock,
//    this->Internal->memoryInspectorDock);
  pqApplicationCore::instance()->registerManager(
    "COLOR_EDITOR_PANEL", this->Internals->Ui.colorMapEditorDock);
  this->Internals->Ui.colorMapEditorDock->hide();

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
