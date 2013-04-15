
#ifndef __pqVelodyneManager_h
#define __pqVelodyneManager_h

#include <QObject>

#include "vvConfigure.h"

class pqServer;
class pqView;
class pqPipelineSource;
class QWidget;

class vvAppLogic;

class QAction;
class QLabel;

class VelodyneHDLPlugin_EXPORT pqVelodyneManager : public QObject
{

  Q_OBJECT

public:

  static pqVelodyneManager *instance();

  virtual ~pqVelodyneManager();

  /// Convenience function for getting the current server.
  static pqServer* getActiveServer();

  /// Convenience function for getting the main window.
  static QWidget* getMainWindow();

  static pqView* findView(pqPipelineSource *source, int port, const QString &viewType);

  static pqView* getRenderView();

  /// Convenience function for destroying a pipeline object and all of its
  /// consumers.
  //static void destroyPipelineSourceAndConsumers(pqPipelineSource *source);

  /// Finds a pipeline source with the given SM XML name.  If there is more than
  /// one, the first is returned.
  //static pqPipelineSource *findPipelineSource(const char *SMName);

  void setSource(pqPipelineSource* source);
  pqPipelineSource* source();


  QLabel* statusBarLogo();
  QLabel* filenameLabel();
  QLabel* statusLabel();
  QLabel* timeLabel();


  void setup(QAction* openFile, QAction* close, QAction* openSensor, QAction* chooseCalibrationFile,
             QAction* resetView, QAction* play, QAction* seekForward, QAction* seekBackward, QAction* gotoStart, QAction* gotoEnd,
             QAction* record, QAction* measurementGrid, QAction* saveScreenshot);

  void openData(const QString& filename);

  void runPython(const QString& statements);

public slots:

  void pythonStartup();

  void onClose();
  void onResetView();
  void onPlay();
  void onSeekForward();
  void onSeekBackward();
  void onGotoStart();
  void onGotoEnd();
  void onOpenSensor();
  void onChooseCalibrationFile();
  void onRecord();
  void onMeasurementGrid();
  void onSaveScreenshot();
  void onPollSource();

signals:

  void sourceCreated();

private:

  void setStreaming(bool streaming);

  void updateTimeLabel();

  pqVelodyneManager(QObject *p);

  class pqInternal;
  pqInternal *Internal;

  Q_DISABLE_COPY(pqVelodyneManager);
};

#endif
