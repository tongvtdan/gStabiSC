#include "mainwindow.h"
#include <QApplication>
#include <src/configuration.h>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    MainWindow w;
    w.setWindowTitle(GSB::APPNAME +" " + GSB::APPLICATIONVERSION_STR + " - "+ GSB::COMPANYNAME);
    w.show();
    
    return a.exec();
}
