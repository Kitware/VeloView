#ifndef LQUpdateCalibrationReaction_H
#define LQUpdateCalibrationReaction_H

#include "applicationui_export.h"

#include "pqReaction.h"

#include "vtkSmartPointer.h"

class pqPipelineSource;
class vtkSMProxy;
class vvCalibrationDialog;

/**
* @ingroup Reactions
* Reaction to update the calibration
*/
class APPLICATIONUI_EXPORT lqUpdateCalibrationReaction : public pqReaction
{
  Q_OBJECT
  typedef pqReaction Superclass;

public:
  lqUpdateCalibrationReaction(QAction* action);

  static void setTransform(vtkSMProxy * proxy, double x, double y, double z, double roll, double pitch, double yaw);

  static void setNetworkCalibration(vtkSMProxy * proxy, double listenningPort, double forwardingPort,
                                    bool isForwarding, QString ipAddressForwarding,
                                    bool isCrashAnalysing, bool multiSensors);

  static void setCalibrationFileAndDefaultInterpreter(vtkSMProxy * proxy, QString calibrationFile);


  static void UpdateCalibration(pqPipelineSource* & lidarSource,
                                pqPipelineSource* & posOrSource,
                                const vvCalibrationDialog& dialog);

  static void UpdateExistingSource(pqPipelineSource* & lidarSource, pqPipelineSource* & posOrSource);

public slots:
  /**
  * Called when the action is triggered.
  */
  void onTriggered() override;

  /**
   * Monitor the added/deleted source to enable/disable the QAction
   */
  void onSourceAdded(pqPipelineSource* src);
  void onSourceRemoved(pqPipelineSource* src);

private:
  Q_DISABLE_COPY(lqUpdateCalibrationReaction)

};

#endif // LQUpdateCalibrationReaction_H
