#include "MainWidget.h"

#include <QApplication>
#include <QDebug>

#include <unistd.h>

int main(int argc, char** argv)
{
    QApplication app(argc, argv);

    MainWidget gui;
    gui.show();

    return app.exec();
}
