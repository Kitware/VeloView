#ifndef __vvCalibrationDialog_h
#define __vvCalibrationDialog_h

#include <QDialog>

class vvCalibrationDialog : public QDialog
{
  Q_OBJECT
public:

  vvCalibrationDialog(QWidget *p=0);
  virtual ~vvCalibrationDialog();

  QString selectedCalibrationFile();

  QStringList calibrationFiles();

protected slots:

  void addFile();
  void removeSelectedFile();
  void onCurrentRowChanged(int row);

private:

  void saveFileList();
  void saveSelectedRow();
  void restoreSelectedRow();

  class pqInternal;
  pqInternal* Internal;

  Q_DISABLE_COPY(vvCalibrationDialog);
};

#endif
