#include"mainwindow.h"

namespace qt5_solver
{

MainWindow::MainWindow(int argc,char **argv,QWidget *parent):QMainWindow(parent)
{
    qnode = std::make_shared<QNode>(argc,argv);
    ui.setupUi(this);
    //navigation_window = std::make_shared<Navigation_Window>(argc,argv,qnode);
    captain_command = (char)0;

    connect(ui.openGLWidget,&OpenCVOpenGLWidget::mousePressStart,this,&MainWindow::onHandleMouseClick);
    connect(ui.DirectcmdButton,SIGNAL(clicked()),this,SLOT(onClickedWindow()));

    map_path = ros::package::getPath("cmd_target") + "/map/map_red.jpg";
    initmap();
}

MainWindow::~MainWindow()
{

}

void MainWindow::update()
{

}

void MainWindow::initmap()
{
    cv::Mat img_src;
    img_src = cv::imread(map_path);
    ui.openGLWidget->showImage(img_src);
}

void MainWindow::onShutDown()
{
    close();
}

void MainWindow::keyPressEvent(QKeyEvent* event)
{
    if(ui.openGLWidget->GetCharAvailable())
    {
        switch(event->key())
        {
            case Qt::Key_A:
            {
                captain_command = 'A';
                break;
            }
            case Qt::Key_B:
            {
                captain_command = 'B';
                break;
            }
            case Qt::Key_C:
            {
                captain_command = 'C';
                break;
            }
            case Qt::Key_D:
            {
                captain_command = 'D';
                break;
            }
            case Qt::Key_E:
            {
                captain_command = 'E';
                break;
            }
            case Qt::Key_F:
            {
                captain_command = 'F';
                break;
            }
            case Qt::Key_G:
            {
                captain_command = 'G';
                break;
            }
            case Qt::Key_H:
            {
                captain_command = 'H';
                break;
            }
            case Qt::Key_I:
            {
                captain_command = 'I';
                break;
            }
            case Qt::Key_J:
            {
                captain_command = 'J';
                break;
            }
            case Qt::Key_K:
            {
                captain_command = 'K';
                break;
            }
            case Qt::Key_L:
            {
                captain_command = 'L';
                break;
            }
            case Qt::Key_N:
            {
                captain_command = 'N';
                break;
            }
            case Qt::Key_O:
            {
                captain_command = 'O';
                break;
            }
            case Qt::Key_Q:
            {
                captain_command = 'Q';
                break;
            }
            case Qt::Key_R:
            {
                captain_command = 'R';
                break;
            }
            case Qt::Key_S:
            {
                captain_command = 'S';
                break;
            }
            case Qt::Key_T:
            {
                captain_command = 'T';
                break;
            }
            case Qt::Key_U:
            {
                captain_command = 'U';
                break;
            }
            case Qt::Key_V:
            {
                captain_command = 'V';
                break;
            }
            case Qt::Key_W:
            {
                captain_command = 'W';
                break;
            }
            case Qt::Key_X:
            {
                captain_command = 'X';
                break;
            }
            case Qt::Key_Y:
            {
                captain_command = 'Y';
                break;
            }
            case Qt::Key_Z:
            {
                captain_command = 'Z';
                break;
            }
            default:
            {
                captain_command = (char)0;
                break;
            }

        }
    }

    QMainWindow::keyPressEvent(event);
    
}

void MainWindow::onMouseEnterOpenGLWidget()
{
    
}

void MainWindow::onMouseExitOpenGLWidget()
{

}

void MainWindow::onHandleMouseClick()
{
    QPoint final_point = ui.openGLWidget->getQPoint();
    int point_x = final_point.x();
    int point_y = final_point.y();
    Eigen::Vector2i point_eigen{point_x,point_y};
    Eigen::Vector3d point_eigen_world = map_to_world(point_eigen);
    qnode->publish_command(point_eigen_world,captain_command);
    captain_command = (char)0;
}

void MainWindow::onClickedWindow()
{   
    
}

}