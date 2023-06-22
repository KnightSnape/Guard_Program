#pragma once

#include"cmdwindow.h"
#include<Eigen/Eigen>
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
        void initmap();
        void update();

    public Q_SLOTS:
        void onShutDown();
        void onMouseEnterOpenGLWidget();
        void onMouseExitOpenGLWidget();
        void onHandleMouseClick();
        void onClickedWindow();


    public slots:
        void keyPressEvent(QKeyEvent* event) override;

    signals:
        void mouseClicked(const QPoint& pos);

    private:
        Navigation_Window::Ptr navigation_window;
        Ui::MainWindow ui;
        QNode::Ptr qnode;
        QWidget *widget;
        std::string map_path;
        int click_final_pos_x;
        int click_final_pos_y;
        char captain_command;

        inline Eigen::Vector3d map_to_world(Eigen::Vector2i pu)
        {
            Eigen::Vector3d final_pw;
            final_pw.x() = (pu.x() * 1.0 / 840 * 28.0);
            final_pw.y() = (pu.y() * 1.0 / 840 * 28.0);
            final_pw.z() = 0;
            return final_pw;
        }

    public:

        

};

}