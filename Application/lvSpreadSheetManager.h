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

#ifndef __lvSpreadSheetManager_h
#define __lvSpreadSheetManager_h

#include <pqRenderView.h>
#include <pqSpreadSheetView.h>
#include <pqSpreadSheetViewModel.h>
#include <pqSpreadSheetViewDecorator.h>

#include <QDockWidget>

class lvSpreadSheetManager : public QObject
{
  Q_OBJECT

public:
  lvSpreadSheetManager(QObject* parent);
  void setSpreadSheetDockWidget(QDockWidget* dock);
  void setMainView(pqRenderView* view);
signals:
  void spreadSheetEnabled(bool);

protected slots:
  void onToggleSpreadSheet(bool toggle);
  void onSpreadSheetEndRender();

private:
  QDockWidget* spreadSheetDock = nullptr;
  pqRenderView* mainView = nullptr;

  pqSpreadSheetView* SpreadSheetView = nullptr;
  pqSpreadSheetViewDecorator* SpreadSheetViewDec = nullptr;

  void constructSpreadSheet();
  void destructSpreadSheet();
  bool isSpreadSheetOpen();
  void conditionnallyHideColumn(const std::string& conditionSrcName,
                                const std::string& columnName);
};

#endif // __lvSpreadSheetManager_h
