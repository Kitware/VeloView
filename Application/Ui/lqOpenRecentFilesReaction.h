#ifndef lqOpenRecentFilesReaction_h
#define lqOpenRecentFilesReaction_h

#include "applicationui_export.h"

#include <vtkEventQtSlotConnect.h>
#include <vtkSmartPointer.h>

class pqPipelineSource;
class QAction;
class QMenu;

#include <deque>

/**
 * @class lqOpenRecentFilesReaction
 * @brief manages recent files menu used in LidarView
 */

class APPLICATIONUI_EXPORT lqOpenRecentFilesReaction : public QObject
{
  Q_OBJECT
  typedef QObject Superclass;

public:
  lqOpenRecentFilesReaction(QMenu* recentFilesMenu, QAction* clearRecentFiles, QObject* parent = 0);

protected slots:
  void onSourceAdded(pqPipelineSource* src);
  void onOpenRecentFile(QString filename);
  void onClearMenu();
  void onPcapUpdate(vtkObject* caller, unsigned long, void*);

private:
  Q_DISABLE_COPY(lqOpenRecentFilesReaction)

  std::deque<QAction *> recentFilesActions;

  unsigned int sizeOfTheQueue = 5;

  QMenu* RecentFilesMenu;

  vtkSmartPointer<vtkEventQtSlotConnect> Connection;

  /**
   * @brief createNewRecentFile
   *        Create a new entry (action) in the "Recent File Menu" for pcapName (if it's not already in it)
   * @param pcapName file name to add
   */
  void createNewRecentFile(QString pcapName);

  /**
   * @brief RemoveFirstRecentFilesAction
   * Pop the front of this->recentFilesActions
   * And remove the associated action in the RecentFilesMenu
   */
  void RemoveFirstRecentFilesAction();

  /**
   * @brief HasAlreadyBeenAdded
   * @param filename
   * @return True if filename is already represented by an action in this->recentFilesActions
   */
  bool IsAlreadyARecentFiles(QString filename);

  /**
   * @brief SaveRecentFilesInSettings
   * Save the RecentFilesMenu in the pqApplicationCore::instance()->settings()
   */
  void SaveRecentFilesInSettings();

  /**
   * @brief RestoreRecentFilesInSettings
   * Restore the RecentFilesMenu with data in the pqApplicationCore::instance()->settings()
   */
  void RestoreRecentFilesFromSettings();
};

#endif // lqOpenRecentFilesReaction_h
