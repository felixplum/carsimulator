#include "gui.h"
#include <QApplication>
#include "udp_server.h"
#include <boost/bind.hpp>
#include <boost/thread/thread.hpp>
#include <boost/chrono.hpp>


int main(int argc, char *argv[])
{
  QApplication a(argc, argv);
  GUI w;
  w.setFixedSize(w.size());
  w.show();
  return a.exec();
}
