#ifndef __vvMainWindow_h
#define __vvMainWindow_h

#include <QMainWindow>

class vvMainWindow : public QMainWindow
{
  Q_OBJECT
  typedef QMainWindow Superclass;
public:
  vvMainWindow();
  virtual ~vvMainWindow();

private:
  Q_DISABLE_COPY(vvMainWindow);

  class pqInternals;
  pqInternals* Internals;
  friend class pqInternals;
};

#endif
