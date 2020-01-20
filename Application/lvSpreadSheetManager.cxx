// Copyright 2020 Kitware, Inc.
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

#include "lvSpreadSheetManager.h"

#include <pqObjectBuilder.h>
#include <pqApplicationCore.h>
#include <pqActiveObjects.h>

#include <assert.h>

//-----------------------------------------------------------------------------
lvSpreadSheetManager::lvSpreadSheetManager(QObject* parent) : QObject(parent)
{
}

//-----------------------------------------------------------------------------
void lvSpreadSheetManager::setSpreadSheetDockWidget(QDockWidget* dock)
{
  this->spreadSheetDock = dock;
}

//-----------------------------------------------------------------------------
void lvSpreadSheetManager::setMainView(pqRenderView* view)
{
  this->mainView = view;
}

namespace {
//-----------------------------------------------------------------------------
std::vector<QObject*> findChildrenByClassName(QObject* object, std::string className)
{
  std::vector<QObject*> found;
  for (QObject* child : object->children())
  {
     if (std::string(child->metaObject()->className()) == className)
     {
       found.push_back(child);
     }
  }
  return found;
}
}

//-----------------------------------------------------------------------------
void lvSpreadSheetManager::constructSpreadSheet()
{
  assert(this->SpreadSheetView == nullptr);
  this->SpreadSheetView = qobject_cast<pqSpreadSheetView*>
      (pqApplicationCore::instance()->getObjectBuilder()->createView(pqSpreadSheetView::spreadsheetViewType(), pqApplicationCore::instance()->getActiveServer(), true));
  this->SpreadSheetView->rename("main spreadsheet view");
  assert(this->SpreadSheetView != nullptr);

  QObject::connect(this->SpreadSheetView, SIGNAL(endRender()), this, SLOT(onSpreadSheetEndRender()));

  this->SpreadSheetViewDec = new pqSpreadSheetViewDecorator(this->SpreadSheetView);
  this->SpreadSheetViewDec->setPrecision(3);
  this->SpreadSheetViewDec->setFixedRepresentation(true);

  this->spreadSheetDock->setWidget(this->SpreadSheetView->widget());
  this->SpreadSheetView->getProxy()->UpdateVTKObjects();
  this->spreadSheetDock->setVisible(true);

  // Hacky way to hide the line numbers in the spreadsheet (they bring no information)
  std::vector<QObject*> headers = findChildrenByClassName(
                                    findChildrenByClassName(
                                      this->spreadSheetDock->findChild<QObject *>("Viewport"),
                                      "pqSpreadSheetViewWidget")[0],
                                    "QHeaderView");
  // of the two QHeaderViews, we hide the one wich is "vertical"
  for (size_t i = 0; i < headers.size(); i++)
  {
    if (reinterpret_cast<QWidget*>(headers[i])->height() > reinterpret_cast<QWidget*>(headers[i])->width())
    {
      reinterpret_cast<QWidget*>(headers[i])->hide();
    }
  }
}

//-----------------------------------------------------------------------------
void lvSpreadSheetManager::destructSpreadSheet()
{
  pqActiveObjects::instance().setActiveView(this->mainView);

  this->spreadSheetDock->setVisible(false);
  this->spreadSheetDock->setWidget(nullptr);

  delete this->SpreadSheetViewDec;
  this->SpreadSheetViewDec = nullptr;

  delete this->SpreadSheetView;
  this->SpreadSheetView = nullptr;
}

//-----------------------------------------------------------------------------
void lvSpreadSheetManager::onToggleSpreadSheet(bool toggled)
{
  if (toggled)
  {
    this->constructSpreadSheet();
    emit this->spreadSheetEnabled(true);
  }
  else
  {
    this->destructSpreadSheet();
    emit this->spreadSheetEnabled(false);
  }
}

//-----------------------------------------------------------------------------
bool lvSpreadSheetManager::isSpreadSheetOpen()
{
  return this->SpreadSheetView != nullptr;
}

//-----------------------------------------------------------------------------
void lvSpreadSheetManager::conditionnallyHideColumn(const std::string& conditionSrcName,
                                                    const std::string& columnName)
{
  if (!this->SpreadSheetView
      || !this->SpreadSheetView->getViewModel()
      || !this->SpreadSheetView->getViewModel()->activeRepresentation()
      || this->SpreadSheetView->getViewModel()->activeRepresentation()
             ->getInput()->getSMName().toStdString() != conditionSrcName)
  {
    return;
  }

  const int cols = this->SpreadSheetView->getViewModel()->columnCount();
  for (int i = 0; i < cols; i++)
  {
    QVariant colHeader = this->SpreadSheetView->getViewModel()->headerData(i, Qt::Orientation::Horizontal);
    if (colHeader.toString().toStdString() == columnName)
    {
      this->SpreadSheetView->getViewModel()->setVisible(i, false);
    }
  }
}

//-----------------------------------------------------------------------------
void lvSpreadSheetManager::onSpreadSheetEndRender()
{
  // endRender may not be the best signal to use because it will be called at
  // each frame whereas we would prefer to update only when the source is
  // changed. However I tried pqSpreadSheetView::showing and
  // pqSpreadSheetView::viewportUpdated but none worked.
  conditionnallyHideColumn("Data", "Points_m_XYZ"); // hide duplicated Point coordinates
  conditionnallyHideColumn("TrailingFrame", "Points_m_XYZ");
}
