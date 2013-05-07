
#ifndef __vvPythonQtDecorators_h
#define __vvPythonQtDecorators_h

#include <QObject>
#include "PythonQt.h"

#include "vvCalibrationDialog.h"
#include "vvSelectFramesDialog.h"

class  vvPythonQtDecorators : public QObject
{
  Q_OBJECT

public:

  vvPythonQtDecorators(QObject* parent=0) : QObject(parent)
    {
    this->registerClassForPythonQt(&vvCalibrationDialog::staticMetaObject);
    this->registerClassForPythonQt(&vvSelectFramesDialog::staticMetaObject);
    }

  inline void registerClassForPythonQt(const QMetaObject* metaobject)
    {
    PythonQt::self()->registerClass(metaobject, "paraview");
    }

public slots:


  vvCalibrationDialog* new_vvCalibrationDialog(QWidget* arg0)
    {
    return new vvCalibrationDialog(arg0);
    }


  vvSelectFramesDialog* new_vvSelectFramesDialog(QWidget* arg0)
    {
    return new vvSelectFramesDialog(arg0);
    }



  int frameMode(vvSelectFramesDialog* inst)
    {
    return inst->frameMode();
    }

  void setFrameMode(vvSelectFramesDialog* inst, int arg0)
    {
    inst->setFrameMode(arg0);
    }


  int frameStart(vvSelectFramesDialog* inst)
    {
    return inst->frameStart();
    }

  int frameStop(vvSelectFramesDialog* inst)
    {
    return inst->frameStop();
    }


  void setFrameStart(vvSelectFramesDialog* inst, int arg0)
    {
    inst->setFrameStart(arg0);
    }

  void setFrameStop(vvSelectFramesDialog* inst, int arg0)
    {
    inst->setFrameStop(arg0);
    }


  void setFrameMinimum(vvSelectFramesDialog* inst, int arg0)
    {
    inst->setFrameMinimum(arg0);
    }

  void setFrameMaximum(vvSelectFramesDialog* inst, int arg0)
    {
    inst->setFrameMaximum(arg0);
    }


  void saveState(vvSelectFramesDialog* inst)
    {
    inst->saveState();
    }

  void restoreState(vvSelectFramesDialog* inst)
    {
    inst->restoreState();
    }


};

#endif