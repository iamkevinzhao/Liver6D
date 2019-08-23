#include <QApplication>
#include "main_window.h"
#include "pclviewer.h"

int main(int argc, char *argv[]) {
  QApplication app(argc, argv);
  PCLViewer win;
  win.show();
  return app.exec();
}
