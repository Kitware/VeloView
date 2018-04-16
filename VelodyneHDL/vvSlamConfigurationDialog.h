#ifndef VVSLAMCONFIGURATIONDIALOG_H
#define VVSLAMCONFIGURATIONDIALOG_H

#include <QDialog>

#include "vvConfigure.h"

namespace Ui {
  class vvSlamConfigurationDialog;
}

class VelodyneHDLPlugin_EXPORT vvSlamConfigurationDialog : public QDialog
{
  Q_OBJECT
  Q_PROPERTY(int frameMode READ frameMode)
  Q_PROPERTY(int frameStart READ frameStart)
  Q_PROPERTY(int frameStop READ frameStop)
  // General
  Q_PROPERTY(int NbVoxel READ NbVoxel)
  Q_PROPERTY(double AngleResolution READ AngleResolution)
  //Keypoint
  Q_PROPERTY(int Keypoint_MaxEdgePerScanLine READ Keypoint_MaxEdgePerScanLine)
  Q_PROPERTY(int Keypoint_MaxPlanarsPerScanLine READ Keypoint_MaxPlanarsPerScanLine)
  Q_PROPERTY(double Keypoint_MinDistanceToSensor READ Keypoint_MinDistanceToSensor)
  Q_PROPERTY(double Keypoint_PlaneCurvatureThreshold READ Keypoint_PlaneCurvatureThreshold)
  Q_PROPERTY(double Keypoint_EdgeCurvatureThreshold READ Keypoint_EdgeCurvatureThreshold)
  //Egomotion
  Q_PROPERTY(int EgoMotion_MaxIter READ EgoMotion_MaxIter)
  Q_PROPERTY(int EgoMotion_IcpFrequence READ EgoMotion_IcpFrequence)
  Q_PROPERTY(int EgoMotion_LineDistance_k READ EgoMotion_LineDistance_k)
  Q_PROPERTY(int EgoMotion_LineDistance_factor READ EgoMotion_LineDistance_factor)
  Q_PROPERTY(int EgoMotion_PlaneDistance_k READ EgoMotion_PlaneDistance_k)
  Q_PROPERTY(int EgoMotion_PlaneDistance_factor1 READ EgoMotion_PlaneDistance_factor1)
  Q_PROPERTY(int EgoMotion_PlaneDistance_factor2 READ EgoMotion_PlaneDistance_factor2)
  //Mapping
  Q_PROPERTY(int Mapping_MaxIter READ Mapping_MaxIter)
  Q_PROPERTY(int Mapping_IcpFrequence READ Mapping_IcpFrequence)
  Q_PROPERTY(int Mapping_LineDistance_k READ Mapping_LineDistance_k)
  Q_PROPERTY(int Mapping_LineDistance_factor READ Mapping_LineDistance_factor)
  Q_PROPERTY(int Mapping_PlaneDistance_k READ Mapping_PlaneDistance_k)
  Q_PROPERTY(int Mapping_PlaneDistance_factor1 READ Mapping_PlaneDistance_factor1)
  Q_PROPERTY(int Mapping_PlaneDistance_factor2 READ Mapping_PlaneDistance_factor2)
  Q_ENUMS(FrameMode)

public:
  explicit vvSlamConfigurationDialog(QWidget *parent = 0);
  ~vvSlamConfigurationDialog();

  int frameMode() const;
  int frameStart() const;
  int frameStop() const;

  // General
  int NbVoxel();
  double AngleResolution();

  //Keypoint
  int Keypoint_MaxEdgePerScanLine();
  int Keypoint_MaxPlanarsPerScanLine();
  double Keypoint_MinDistanceToSensor();
  double Keypoint_PlaneCurvatureThreshold();
  double Keypoint_EdgeCurvatureThreshold();

  // Egomotion
  int EgoMotion_MaxIter();
  int EgoMotion_IcpFrequence();
  int EgoMotion_LineDistance_k();
  double EgoMotion_LineDistance_factor();
  int EgoMotion_PlaneDistance_k();
  double EgoMotion_PlaneDistance_factor1();
  double EgoMotion_PlaneDistance_factor2();

  // Mapping
  int Mapping_MaxIter();
  int Mapping_IcpFrequence();
  int Mapping_LineDistance_k();
  double Mapping_LineDistance_factor();
  int Mapping_PlaneDistance_k();
  double Mapping_PlaneDistance_factor1();
  double Mapping_PlaneDistance_factor2();

  enum FrameMode
  {
    CURRENT_FRAME = 0,
    ALL_FRAMES,
    FRAME_RANGE
  };

public slots:
  void accept();

private:
  Ui::vvSlamConfigurationDialog *ui;
};

#endif // VVSLAMCONFIGURATIONDIALOG_H
