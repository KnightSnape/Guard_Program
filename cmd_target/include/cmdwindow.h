#pragma once


#include<QMainWindow>
#include<QSignalMapper>
#include"ui_cmd_window.h"
#include<signal.h>
#include"cmd_target_node.h"

QT_BEGIN_NAMESPACE
namespace Ui {class Navigation_Window; }
QT_END_NAMESPACE

namespace qt5_solver
{

class Navigation_Window : public QMainWindow
{
    Q_OBJECT
    public:
        typedef std::shared_ptr<Navigation_Window> Ptr;
        Navigation_Window(int argc,char **argv,const QNode::Ptr &qnode_,QWidget *parent = 0);
        ~Navigation_Window();

    public Q_SLOTS:
        void onWrittingX();
        void onWrittingY();
        void onClearWritten();
        void onSaveWritten();
        void onShutDown();
    
    private:
        QNode::Ptr qnode;
        Ui::Navigation_Window ui;

        float navigation_x;
        float navigation_y;
        float final_navigation_x;
        float final_navigation_y;
        bool have_got_navigation_x;
        bool have_got_navigation_y;


};



}