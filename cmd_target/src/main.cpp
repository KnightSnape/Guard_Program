#include"mainwindow.h"

int main(int argc,char **argv)
{
    ros::init(argc,argv,"cmd_target");
    QApplication app(argc,argv);
    qt5_solver::MainWindow w(argc,argv);
    w.show();
    return app.exec();
}