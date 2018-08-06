#include "Garfield_control.h"
#include <QApplication>
#include <QDebug>
#include <QThread>

#include <string>
int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    Garfield_control w;
    w.show();

    return a.exec();
}
