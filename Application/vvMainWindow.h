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
#ifndef __vvMainWindow_h
#define __vvMainWindow_h

#include <QMainWindow>

class pqDataRepresentation;

class vvMainWindow : public QMainWindow
{
  Q_OBJECT
  typedef QMainWindow Superclass;

public:
  vvMainWindow();
  virtual ~vvMainWindow();

protected:
  void dragEnterEvent(QDragEnterEvent* evt) override;
  void dropEvent(QDropEvent* evt) override;

protected slots:
  void showHelpForProxy(const QString& proxyname, const QString& groupname);
  void handleMessage(const QString &, int type);

private:
  Q_DISABLE_COPY(vvMainWindow);

  class pqInternals;
  pqInternals* Internals;
  friend class pqInternals;
};

#endif
