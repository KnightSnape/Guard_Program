#include"cmdwindow.h"

namespace qt5_solver
{

Navigation_Window::Navigation_Window(int argc,char **argv,const QNode::Ptr &qnode_,QWidget *parent):QMainWindow(parent),qnode(qnode_)
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

void Navigation_Window::onWrittingX()
{
    QString text = ui.WriteX->text();
    float value = text.toFloat(&have_got_navigation_x);
    if(have_got_navigation_x)
    {
        navigation_x = value;
    }
    ROS_DEBUG("change input X");
}

void Navigation_Window::onWrittingY()
{
    QString text = ui.WriteY->text();   
    float value = text.toFloat(&have_got_navigation_y);
    if(have_got_navigation_y)
    {
        navigation_y = value;
    }
    ROS_DEBUG("change input Y");
}

void Navigation_Window::onSaveWritten()
{
    if(have_got_navigation_x && have_got_navigation_y)
    {
        final_navigation_x = navigation_x;
        final_navigation_y = navigation_y;
        
        ui.WriteX->clear();
        ui.WriteY->clear();

        have_got_navigation_x = false;
        have_got_navigation_y = false;

        qnode->get_final_navigaton(final_navigation_x,final_navigation_y);
    }
}

void Navigation_Window::onClearWritten()
{
    ui.WriteX->clear();
    ui.WriteY->clear();

    have_got_navigation_x = false;
    have_got_navigation_y = false;
}

void Navigation_Window::onShutDown()
{
    close();
}



}
