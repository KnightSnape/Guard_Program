#pragma once


#include<QMainWindow>
#include<QSignalMapper>
#include"ui_main_window.h"
#include<signal.h>
#include"cmd_target_node.h"

QT_BEGIN_NAMESPACE
namespace Ui {class MainWindow; }
QT_END_NAMESPACE

namespace qt5_solver
{

class MainWindow : public QMainWindow
{
    Q_OBJECT
    public:
        MainWindow(int argc,char **argv,QWidget *parent = 0);
        ~MainWindow();

    public Q_SLOTS:
        void onWrittingX();
        void onWrittingY();
        void onClearWritten();
        void onSaveWritten();
        void onShutDown();

    private:
        Ui::MainWindow ui;
        QNode qnode;
        QWidget *widget;
        float navigation_x;
        float navigation_y;
        float final_navigation_x;
        float final_navigation_y;
        bool have_got_navigation_x;
        bool have_got_navigation_y;
        

};

}