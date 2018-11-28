
#ifndef __vvPythonQtDecorators_h
#define __vvPythonQtDecorators_h

#include "PythonQt.h"
#include <QObject>

#include "pqVelodyneManager.h"
#include "Ui/vvCalibrationDialog.h"
#include "Ui/vvCropReturnsDialog.h"
#include "Ui/vvLaserSelectionDialog.h"
#include "Ui/vvSelectFramesDialog.h"

class vvPythonQtDecorators : public QObject
{
  Q_OBJECT

public:
  vvPythonQtDecorators(QObject* parent = 0)
    : QObject(parent)
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

  bool isDisplayMoreSelectionsChecked(vvLaserSelectionDialog* inst)
  {
    return inst->isDisplayMoreSelectionsChecked();
  }

  void setDisplayMoreSelectionsChecked(vvLaserSelectionDialog* inst, bool arg0)
  {
    inst->setDisplayMoreSelectionsChecked(arg0);
  }

  void setLasersCorrections(vvLaserSelectionDialog* inst, const QVector<double>& arg0,
    const QVector<double>& arg1, const QVector<double>& arg2, const QVector<double>& arg3,
    const QVector<double>& arg4, const QVector<double>& arg5, const QVector<double>& arg6,
    const QVector<double>& arg7, const QVector<double>& arg8, const QVector<double>& arg9,
    const QVector<double>& arg10, int arg11)
  {
    inst->setLasersCorrections(
      arg0, arg1, arg2, arg3, arg4, arg5, arg6, arg7, arg8, arg9, arg10, arg11);
  }

  void static_pqVelodyneManager_saveFramesToPCAP(
    vtkSMSourceProxy* arg0, int arg1, int arg2, const QString& arg3)
  {
    pqVelodyneManager::saveFramesToPCAP(arg0, arg1, arg2, arg3);
  }

  void static_pqVelodyneManager_saveFramesToLAS(vtkLidarReader* arg0, vtkPolyData* arg1,
    int arg2, int arg3, const QString& arg4, int arg5)
  {
    pqVelodyneManager::saveFramesToLAS(arg0, arg1, arg2, arg3, arg4, arg5);
  }
};

#endif
