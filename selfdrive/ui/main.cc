#include <QApplication>

#include "qt/window.hpp"
#include "qt/qt_window.hpp"

int main(int argc, char *argv[]) {
  QSurfaceFormat fmt;
#ifdef __APPLE__
  fmt.setVersion(3, 2);
  fmt.setProfile(QSurfaceFormat::OpenGLContextProfile::CoreProfile);
  fmt.setRenderableType(QSurfaceFormat::OpenGL);
#else
  fmt.setRenderableType(QSurfaceFormat::OpenGLES);
#endif
  QSurfaceFormat::setDefaultFormat(fmt);

#ifdef QCOM
  QApplication::setAttribute(Qt::AA_ShareOpenGLContexts);
#endif

  QApplication a(argc, argv);
  MainWindow w;
  setMainWindow(&w);
  a.installEventFilter(&w);
  return a.exec();
}
