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

#include "pqObjectPickingBehavior.h"
#include <pqActiveObjects.h>
#include <pqApplicationCore.h>
#include <pqAutoLoadPluginXMLBehavior.h>
#include <pqCommandLineOptionsBehavior.h>
#include <pqCrashRecoveryBehavior.h>
#include <pqDataTimeStepBehavior.h>
#include <pqDefaultViewBehavior.h>
#include <pqInterfaceTracker.h>
#include <pqObjectBuilder.h>
#include <pqPersistentMainWindowStateBehavior.h>
#include <pqPythonShellReaction.h>
#include <pqQtMessageHandlerBehavior.h>
#include <pqRenderView.h>
#include <pqRenderViewSelectionReaction.h>
#include <pqServer.h>
#include <pqSettings.h>
#include <pqSpreadSheetView.h>
#include <pqSpreadSheetViewDecorator.h>
#include <pqSpreadSheetVisibilityBehavior.h>
#include <pqStandardPropertyWidgetInterface.h>
#include <pqStandardViewFrameActionsImplementation.h>
#include <pqVelodyneManager.h>
#include <vtkPVPlugin.h>
#include <vtkSMPropertyHelper.h>

#include <QLabel>
#include <QSplitter>
#include <QToolBar>
#include <qdockwidget.h>

#include <cassert>
#include <iostream>
#include <sstream>
// Declare the plugin to load.
PV_PLUGIN_IMPORT_INIT(VelodyneHDLPlugin);
PV_PLUGIN_IMPORT_INIT(PythonQtPlugin);

class vvMainWindow::pqInternals
{
public:
  pqInternals(vvMainWindow* window)
    : Ui()
    , MainView(0)
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

    // Check if the settings are well formed i.e. if an OriginalMainWindow
    // state was previously saved. If not, we don't want to automatically
    // restore the settings state nor save it on quitting VeloView.
    // An OriginalMainWindow state will be force saved once the UI is completly
    // set up.
    pqSettings* const settings = pqApplicationCore::instance()->settings();
    bool shouldClearSettings = false;
    QStringList keys = settings->allKeys();

    if (keys.size() == 0)
    {
      // There were no settings before, let's save the current state as
      // OriginalMainWindow state
      shouldClearSettings = true;
    }
    else
    {
      // Checks if the existing settings are well formed and if not, clear them.
      // An original MainWindow state will be force saved later once the UI is
      // entirely set up
      for (int keyIndex = 0; keyIndex < keys.size(); ++keyIndex)
      {
        if (keys[keyIndex].contains("OriginalMainWindow"))
        {
          shouldClearSettings = true;
          break;
        }
      }
    }

    if (shouldClearSettings)
    {
      new pqPersistentMainWindowStateBehavior(window);
    }
    else
    {
      if (keys.size() > 0)
      {
        vtkGenericWarningMacro("Settings weren't set correctly. Clearing settings.")
      }

      // As pqPersistentMainWindowStateBehavior is not created right now,
      // we can clear the settings as the current bad state won't be saved on
      // closing VeloView
      settings->clear();
    }

    // Connect to builtin server.
    pqObjectBuilder* builder = core->getObjectBuilder();
    pqServer* server = builder->createServer(pqServerResource("builtin:"));
    pqActiveObjects::instance().setActiveServer(server);

    // Set default render view settings
    vtkSMSessionProxyManager* pxm =
      vtkSMProxyManager::GetProxyManager()->GetActiveSessionProxyManager();
    vtkSMProxy* renderviewsettings = pxm->GetProxy("RenderViewSettings");
    assert(renderviewsettings);

    vtkSMPropertyHelper(renderviewsettings, "ResolveCoincidentTopology").Set(0);

    // Create a default view.
    pqRenderView* view =
      qobject_cast<pqRenderView*>(builder->createView(pqRenderView::renderViewType(), server));
    assert(view);
    this->MainView = view;

    vtkSMPropertyHelper(view->getProxy(), "CenterAxesVisibility").Set(0);
    double bgcolor[3] = { 0, 0, 0 };
    vtkSMPropertyHelper(view->getProxy(), "Background").Set(bgcolor, 3);
    // MultiSamples doesn't work, we need to set that up before registering the proxy.
    // vtkSMPropertyHelper(view->getProxy(),"MultiSamples").Set(1);
    view->getProxy()->UpdateVTKObjects();

    // Create a horizontal splitter as the central widget, add views to splitter
    QSplitter* splitter = new QSplitter(Qt::Horizontal);
    window->setCentralWidget(splitter);

    // Add the main widget to the left
    splitter->addWidget(view->widget());

    QSplitter* vSplitter = new QSplitter(Qt::Vertical);
    splitter->addWidget(vSplitter);

    pqView* overheadView = builder->createView(pqRenderView::renderViewType(), server);
    //    overheadView->SetInteractionMode("2D");
    overheadView->getProxy()->UpdateVTKObjects();
    // dont add to the splitter just yet
    // TODO: These sizes should not be absolute things
    overheadView->widget()->setMinimumSize(300, 200);
    vSplitter->addWidget(overheadView->widget());
    new vvToggleSpreadSheetReaction(this->Ui.actionOverheadView, overheadView);

    pqView* spreadsheetView = builder->createView(pqSpreadSheetView::spreadsheetViewType(), server);
    spreadsheetView->getProxy()->UpdateVTKObjects();
    vSplitter->addWidget(spreadsheetView->widget());
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
    new pqRenderViewSelectionReaction(this->Ui.actionSelect_Visible_Points, this->MainView,
      pqRenderViewSelectionReaction::SELECT_SURFACE_POINTS);
    new pqRenderViewSelectionReaction(this->Ui.actionSelect_All_Points, this->MainView,
      pqRenderViewSelectionReaction::SELECT_FRUSTUM_POINTS);

    new pqPythonShellReaction(this->Ui.actionPython_Console);

    pqVelodyneManager::instance()->setup();

    pqSettings* const settings = pqApplicationCore::instance()->settings();
    const QVariant& gridVisible =
      settings->value("VelodyneHDLPlugin/MeasurementGrid/Visibility", true);
    this->Ui.actionMeasurement_Grid->setChecked(gridVisible.toBool());

    this->Ui.actionEnableCrashAnalysis->setChecked(
      settings
        ->value("VelodyneHDLPlugin/MainWindow/EnableCrashAnalysis",
          this->Ui.actionEnableCrashAnalysis->isChecked())
        .toBool());

    new vvLoadDataReaction(this->Ui.actionOpenPcap, false);
    new vvLoadDataReaction(this->Ui.actionOpenApplanix, true);

    connect(this->Ui.actionOpen_Sensor_Stream, SIGNAL(triggered()), pqVelodyneManager::instance(),
      SLOT(onOpenSensor()));

    connect(this->Ui.actionMeasurement_Grid, SIGNAL(toggled(bool)), pqVelodyneManager::instance(),
      SLOT(onMeasurementGrid(bool)));

    connect(this->Ui.actionEnableCrashAnalysis, SIGNAL(toggled(bool)),
      pqVelodyneManager::instance(), SLOT(onEnableCrashAnalysis(bool)));

    connect(this->Ui.actionResetConfigurationFile, SIGNAL(triggered()),
      pqVelodyneManager::instance(), SLOT(onResetCalibrationFile()));

    connect(this->Ui.actionShowErrorDialog, SIGNAL(triggered()), pqApplicationCore::instance(),
      SLOT(showOutputWindow()));

    // handle connection for the Toolbar Menu
    connect(this->Ui.menuToolbar, SIGNAL(aboutToShow()), window, SLOT(UpdateToolBarMenu()));
    connect(this->Ui.actionBasic_Controls, SIGNAL(triggered()), window, SLOT(switchToolBarVisibility()));
    connect(this->Ui.actionColor_Controls, SIGNAL(triggered()), window, SLOT(switchToolBarVisibility()));
    connect(this->Ui.actionView_Controls, SIGNAL(triggered()), window, SLOT(switchToolBarVisibility()));
    connect(this->Ui.actionPlayback_Controls, SIGNAL(triggered()), window, SLOT(switchToolBarVisibility()));
    connect(this->Ui.actionGeolocation_Controls, SIGNAL(triggered()), window, SLOT(switchToolBarVisibility()));
    connect(this->Ui.actionShowPipelineBrowser, SIGNAL(triggered()), window, SLOT(onSwitchPipelineBrowserVisibility()));
    connect(this->Ui.actionShowPropertiesPanel, SIGNAL(triggered()), window, SLOT(onSwitchPropertiesPanelVisibility()));
  }
};

//-----------------------------------------------------------------------------
vvMainWindow::vvMainWindow()
  : Internals(new vvMainWindow::pqInternals(this))
{
  //  this->tabifyDockWidget(
  //    this->Internal->colorMapEditorDock,
  //    this->Internal->memoryInspectorDock);
  pqApplicationCore::instance()->registerManager(
    "COLOR_EDITOR_PANEL", this->Internals->Ui.colorMapEditorDock);
  this->Internals->Ui.colorMapEditorDock->hide();

  PV_PLUGIN_IMPORT(VelodyneHDLPlugin);
  PV_PLUGIN_IMPORT(PythonQtPlugin);

  // Branding
  std::stringstream ss;
  ss << "Reset " << SOFTWARE_NAME << " settings";
  QString text = QString(ss.str().c_str());
  this->Internals->Ui.actionResetConfigurationFile->setText(text);
  ss.str("");
  ss.clear();

  ss << "This will reset all " << SOFTWARE_NAME << " settings by default";
  text = QString(ss.str().c_str());
  this->Internals->Ui.actionResetConfigurationFile->setIconText(text);
  ss.str("");
  ss.clear();

  ss << "About " << SOFTWARE_NAME;
  text = QString(ss.str().c_str());
  this->Internals->Ui.actionAbout_VeloView->setText(text);
  ss.str("");
  ss.clear();

  ss << SOFTWARE_NAME << " Developer Guide";
  text = QString(ss.str().c_str());
  this->Internals->Ui.actionVeloViewDeveloperGuide->setText(text);
  ss.str("");
  ss.clear();

  ss << SOFTWARE_NAME << " User Guide";
  text = QString(ss.str().c_str());
  this->Internals->Ui.actionVeloViewUserGuide->setText(text);
}

//-----------------------------------------------------------------------------
vvMainWindow::~vvMainWindow()
{
  delete this->Internals;
  this->Internals = NULL;
}

//-----------------------------------------------------------------------------
void vvMainWindow::switchToolBarVisibility()
{
  // check how send the signal
  QObject* obj = QObject::sender();
  QToolBar* tb;
  if (obj == this->Internals->Ui.actionBasic_Controls)
  {
    tb = this->Internals->Ui.toolBar;
  }
  else if (obj == this->Internals->Ui.actionColor_Controls)
  {
    tb = this->Internals->Ui.colorToolBar;
  }
  else if (obj == this->Internals->Ui.actionView_Controls)
  {
    tb = this->Internals->Ui.viewSettings;
  }
  else if (obj == this->Internals->Ui.actionPlayback_Controls)
  {
    tb = this->Internals->Ui.playbackToolbar;
  }
  else if (obj == this->Internals->Ui.actionGeolocation_Controls)
  {
    tb = this->Internals->Ui.geolocationToolbar;
  }
  // switch visibility state
  tb->setVisible(!tb->isVisible());
  //
  QAction* act = dynamic_cast<QAction*> (obj);
  if (act != nullptr)
  {
     act->setChecked(tb->isVisible());
  }
}

//-----------------------------------------------------------------------------
void vvMainWindow::UpdateToolBarMenu()
{
  this->Internals->Ui.actionBasic_Controls->setChecked(this->Internals->Ui.toolBar->isVisible());
  this->Internals->Ui.actionColor_Controls->setChecked(this->Internals->Ui.colorToolBar->isVisible());
  this->Internals->Ui.actionView_Controls->setChecked(this->Internals->Ui.viewSettings->isVisible());
  this->Internals->Ui.actionPlayback_Controls->setChecked(this->Internals->Ui.playbackToolbar->isVisible());
  this->Internals->Ui.actionGeolocation_Controls->setChecked(this->Internals->Ui.geolocationToolbar->isVisible());
}

//-----------------------------------------------------------------------------
void vvMainWindow::onSwitchPipelineBrowserVisibility()
{
  // Change visibility
  QDockWidget* dock = this->Internals->Ui.pipelineBrowserDock;
  dock->setVisible(!dock->isVisible());

  // Switch action isChecked status
  this->Internals->Ui.actionShowPipelineBrowser->setChecked(dock->isVisible());
}

//-----------------------------------------------------------------------------
void vvMainWindow::onSwitchPropertiesPanelVisibility()
{
  // Change visibility
  QDockWidget* dock = this->Internals->Ui.propertiesPanelDock;
  dock->setVisible(!dock->isVisible());

  // Switch action isChecked status
  this->Internals->Ui.actionShowPropertiesPanel->setChecked(dock->isVisible());
}