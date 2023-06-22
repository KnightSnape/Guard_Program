#pragma once
#include<QOpenGLFunctions>
#include<QWidget>
#include<GL/gl.h>
#include<GL/glu.h>
#include<opencv2/opencv.hpp>
#include<opencv2/imgproc.hpp>
#include<opencv2/highgui.hpp>
#include<QOpenGLFunctions_2_0>
#include<opencv2/core/opengl.hpp>
#include<QtWidgets/QOpenGLWidget>
#include<QOpenGLFunctions>
#include<QOpenGLWidget>
#include<mutex>
#include<QMouseEvent>
#include<ros/ros.h>

QT_BEGIN_NAMESPACE

class QOpenGLWidgetPrivate;

class Q_WIDGETS_EXPORT OpenCVOpenGLWidget : public QOpenGLWidget , protected QOpenGLFunctions_2_0
{
    Q_OBJECT
    public:
        OpenCVOpenGLWidget(QWidget *parent = nullptr);
        void SetCharAvailable(bool available);
        bool GetCharAvailable();
        QPoint getQPoint();

    signals:
        void imageSizeChanged(int outW,int outH);
        void mousePressStart();
    public slots:
        bool showImage(const cv::Mat &image);
    protected:  
        void initializeGL();
        void paintGL();
        void resizeGL(int w, int h);

        void updateScene();
        void renderImage();

        void enterEvent(QEvent* event) override;
        void leaveEvent(QEvent* event) override;
        void mousePressEvent(QMouseEvent* event) override;



    private:
        QImage mRenderQtImg;
        QImage mResizedImg;
        cv::Mat mOrigImage;

        QColor mBgColor;

        float mImgRatio;

        int mRenderWidth;
        int mRenderHeight;
        int mRenderPosX;
        int mRenderPosY;

        bool is_char_available;

        void recalculatePos();
        std::mutex drawmutex;

        QPoint get_point;

};

