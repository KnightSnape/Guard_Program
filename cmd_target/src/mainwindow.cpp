#include"mainwindow.h"

namespace qt5_solver
{

MainWindow::MainWindow(int argc,char **argv,QWidget *parent):QMainWindow(parent),qnode(argc,argv)
{
    ui.setupUi(this);
    have_got_navigation_x = false;
    have_got_navigation_y = false;

    final_navigation_x = 0.0;
    final_navigation_y = 0.0;

    connect(ui.WriteX,SIGNAL(textChanged(QString)),this,SLOT(onWrittingX()));
    connect(ui.WriteY,SIGNAL(textChanged(QString)),this,SLOT(onWrittingY()));
    connect(ui.ClearButton,SIGNAL(clicked()),this,SLOT(onClearWritten()));
    connect(ui.SaveButton,SIGNAL(clicked()),this,SLOT(onSaveWritten()));
    
}

MainWindow::~MainWindow()
{

}

void MainWindow::onWrittingX()
{
    QString text = ui.WriteX->text();
    float value = text.toFloat(&have_got_navigation_x);
    if(have_got_navigation_x)
    {
        navigation_x = value;
    }
    ROS_DEBUG("change input X");
}

void MainWindow::onWrittingY()
{
    QString text = ui.WriteY->text();   
    float value = text.toFloat(&have_got_navigation_y);
    if(have_got_navigation_y)
    {
        navigation_y = value;
    }
    ROS_DEBUG("change input Y");
}

void MainWindow::onSaveWritten()
{
    if(have_got_navigation_x && have_got_navigation_y)
    {
        final_navigation_x = navigation_x;
        final_navigation_y = navigation_y;
        
        ui.WriteX->clear();
        ui.WriteY->clear();

        have_got_navigation_x = false;
        have_got_navigation_y = false;

        qnode.get_final_navigaton(final_navigation_x,final_navigation_y);
    }
}

void MainWindow::onClearWritten()
{
    ui.WriteX->clear();
    ui.WriteY->clear();

    have_got_navigation_x = false;
    have_got_navigation_y = false;
}

void MainWindow::onShutDown()
{
    close();
}

}