#ifndef LQSwitchInterpreterBehavior_H
#define LQSwitchInterpreterBehavior_H

#include "applicationui_export.h"

#include <QObject>

#include <vtkSmartPointer.h>

class pqPipelineSource;
class vtkEventQtSlotConnect;
class vtkObject;
class vtkSMProxy;


/**
* @ingroup Reactions
*/
class APPLICATIONUI_EXPORT lqSwitchInterpreterBehavior : public QObject
{
  Q_OBJECT
  typedef QObject Superclass;

public:
  lqSwitchInterpreterBehavior(QObject* parent = 0);

protected slots:
  void sourceAdded(pqPipelineSource* src);
  void onInterpreterUpdated(vtkObject* caller, unsigned long, void*);
  void onChangeFormatDetected(vtkObject*, unsigned long, void*);

  void uiSelectNewCalibrationFile(vtkSMProxy *proxy);


signals:
    void interpChange(vtkSMProxy *);

private:
  vtkSmartPointer<vtkEventQtSlotConnect> ConnectionInterpreter;
  Q_DISABLE_COPY(lqSwitchInterpreterBehavior)
};

#endif // LQSwitchInterpreterBehavior_H

