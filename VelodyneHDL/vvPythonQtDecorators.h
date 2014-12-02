
#ifndef __vvPythonQtDecorators_h
#define __vvPythonQtDecorators_h

#include <QObject>
#include "PythonQt.h"

#include "pqVelodyneManager.h"
#include "vvCalibrationDialog.h"
#include "vvCropReturnsDialog.h"
#include "vvLaserSelectionDialog.h"
#include "vvSelectFramesDialog.h"

class  vvPythonQtDecorators : public QObject
{
  Q_OBJECT

public:

  vvPythonQtDecorators(QObject* parent=0) : QObject(parent)
    {
    this->registerClassForPythonQt(&pqVelodyneManager::staticMetaObject);
    this->registerClassForPythonQt(&vvCalibrationDialog::staticMetaObject);
    this->registerClassForPythonQt(&vvCropReturnsDialog::staticMetaObject);
    this->registerClassForPythonQt(&vvLaserSelectionDialog::staticMetaObject);
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


  vvCropReturnsDialog* new_vvCropReturnsDialog(QWidget* arg0)
    {
    return new vvCropReturnsDialog(arg0);
    }


  vvSelectFramesDialog* new_vvSelectFramesDialog(QWidget* arg0)
    {
    return new vvSelectFramesDialog(arg0);
    }


  vvLaserSelectionDialog* new_vvLaserSelectionDialog(QWidget* arg0)
    {
    return new vvLaserSelectionDialog(arg0);
    }


  QVector<int> getLaserSelectionSelector(vvLaserSelectionDialog* inst)
    {
    return inst->getLaserSelectionSelector();
    }

  void setLaserSelectionSelector(vvLaserSelectionDialog* inst, const QVector<int>& arg0)
    {
    inst->setLaserSelectionSelector(arg0);
    }

  void setVerticalCorrections(vvLaserSelectionDialog* inst, const QVector<double>& arg0, int arg1)
    {
    inst->setVerticalCorrections(arg0, arg1);
    }


  void static_pqVelodyneManager_saveFramesToPCAP(vtkSMSourceProxy* arg0, int arg1, int arg2, const QString& arg3)
    {
    pqVelodyneManager::saveFramesToPCAP(arg0, arg1, arg2, arg3);
    }

  void static_pqVelodyneManager_saveFramesToLAS(vtkVelodyneHDLReader* arg0, vtkPolyData* arg1, int arg2, int arg3, const QString& arg4, int arg5)
    {
    pqVelodyneManager::saveFramesToLAS(arg0, arg1, arg2, arg3, arg4, arg5);
    }


};

#endif
